#include "octomap_depth_mapping.hpp"
#include "depth_conversions.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>

#ifdef CUDA
#include "cuda_proj.hpp"
#include <cuda_runtime.h>
#endif

namespace ph = std::placeholders;

namespace octomap_depth_mapping
{

OctomapDemap::OctomapDemap(const rclcpp::NodeOptions &options, const std::string node_name):
    Node(node_name, options),
    max_distance(10.0),
    padding(1),
    encoding("mono16"),
    frame_id("odom"),
    filename(""),
    save_on_shutdown(false)
{
    input_prefixes = this->declare_parameter("input_prefixes", input_prefixes);
    max_distance = this->declare_parameter("output/max_distance", max_distance);
    frame_id = this->declare_parameter("frame_id", frame_id);
    filename = this->declare_parameter("filename", filename);
    encoding = this->declare_parameter("encoding", encoding);
    save_on_shutdown = this->declare_parameter("save_on_shutdown", save_on_shutdown);

    //Set padding based on image encoding
    if (encoding == "mono8")
        padding = 1;
    else if (encoding == "mono16")
        padding = 2;
    else
        assert(false && "Invalid encoding!");

    rclcpp::QoS qos(rclcpp::KeepLast(3));

    // pubs
    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("map_out", 3);

    // subs
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    for (const auto& current_input_prefix : input_prefixes)
    {
        //Depth
        const auto depthTopic = current_input_prefix + "/depth/rect";
        depthSubscribers.emplace_back(
            std::make_shared<depth_subscriber_t>(this, depthTopic, rmw_qos_profile));

        //Pose
        const auto poseTopic = current_input_prefix + "/depth/pose";
        poseSubscribers.emplace_back(
            std::make_shared<pose_subscriber_t>(this, poseTopic, rmw_qos_profile));

        //Camera info
        const auto cameraInfoTopic = current_input_prefix + "/depth/camera_info";
        cameraInfoSubscribers.emplace_back(
            this->create_subscription<sensor_msgs::msg::CameraInfo>(cameraInfoTopic, 10,
                std::bind(&OctomapDemap::camerainfo_callback, this, ph::_1)));

        // Sync depth and pose
        depthPoseSynchronizers.emplace_back(
            std::make_shared<message_filters::Synchronizer<ApproxTimePolicy>>(ApproxTimePolicy(10), *depthSubscribers.back(), *poseSubscribers.back()));

        depthPoseSynchronizers.back()->registerCallback(std::bind(&OctomapDemap::demap_callback, this, ph::_1, ph::_2));
    }

    // depth_sub_.subscribe(this, "image_in", rmw_qos_profile);
    // pose_sub_.subscribe(this, "pose_in", rmw_qos_profile);
    // camerainfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camerainfo_in", 10,
    //     std::bind(&OctomapDemap::camerainfo_callback, this, ph::_1));

    // bind subs with ugly way
    // sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimePolicy>>(ApproxTimePolicy(10), depth_sub_, pose_sub_);
    // sync_->registerCallback(std::bind(&OctomapDemap::demap_callback, this, ph::_1, ph::_2));

    // services
    octomap_srv_ = this->create_service<octomap_msgs::srv::GetOctomap>("get_octomap",
        std::bind(&OctomapDemap::octomap_srv, this, ph::_1, ph::_2));

    reset_srv_ = this->create_service<std_srvs::srv::Empty>("reset",
        std::bind(&OctomapDemap::reset_srv, this, ph::_1, ph::_2));

    save_srv_ = this->create_service<std_srvs::srv::Empty>("save",
        std::bind(&OctomapDemap::save_srv, this, ph::_1, ph::_2));

    double resolution = this->declare_parameter("resolution", 0.1);
    double probHit    = this->declare_parameter("sensor_model/hit", 0.7);
    double probMiss   = this->declare_parameter("sensor_model/miss", 0.4);
    double thresMin   = this->declare_parameter("sensor_model/min", 0.12);
    double thresMax   = this->declare_parameter("sensor_model/max", 0.97);

    ocmap = std::make_shared<octomap::OcTree>(resolution);
    read_ocmap(); // read octomap from file if filename is not empty

    // set map values cause they may be overwritten by read_ocmap()
    ocmap->setResolution(resolution);
    ocmap->setProbHit(probHit);
    ocmap->setProbMiss(probMiss);
    ocmap->setClampingThresMin(thresMin);
    ocmap->setClampingThresMax(thresMax);

    RCLCPP_INFO(this->get_logger(), "--- Launch Parameters ---");

    for (const auto& current_input_prefix : input_prefixes)
        RCLCPP_INFO_STREAM(this->get_logger(), "input_prefix : " << current_input_prefix);

    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/hit : " << probHit);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/miss : " << probMiss);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/min : " << thresMin);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/max : " << thresMax);
    RCLCPP_INFO_STREAM(this->get_logger(), "output/max_distance : " << max_distance);
    RCLCPP_INFO_STREAM(this->get_logger(), "resolution : " << resolution);
    RCLCPP_INFO_STREAM(this->get_logger(), "encoding : " << encoding);
    RCLCPP_INFO_STREAM(this->get_logger(), "frame_id : " << frame_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "filename : " << filename);
    RCLCPP_INFO_STREAM(this->get_logger(), "save_on_shutdown : " << save_on_shutdown);
    RCLCPP_INFO(this->get_logger(), "-------------------------");

#ifdef CUDA
    int devCount;
    cudaGetDeviceCount(&devCount);

    RCLCPP_INFO_STREAM(this->get_logger(), "CUDA Devices: " << devCount);

    for(int i = 0; i < devCount; ++i)
    {
        cudaDeviceProp props;
        cudaGetDeviceProperties(&props, i);
        RCLCPP_INFO_STREAM(this->get_logger(), "Dev-" << i << ": " << props.name << ": " << props.major << "." << props.minor);
        RCLCPP_INFO_STREAM(this->get_logger(), "  Global memory:   " << props.totalGlobalMem / (1024*1024) << "mb");
        RCLCPP_INFO_STREAM(this->get_logger(), "  Shared memory:   " << props.sharedMemPerBlock / 1024 << "kb");
        RCLCPP_INFO_STREAM(this->get_logger(), "  Constant memory: " << props.totalConstMem / 1024 << "kb");
        RCLCPP_INFO_STREAM(this->get_logger(), "  Block registers: " << props.regsPerBlock);
    }

    RCLCPP_INFO(this->get_logger(), "-------------------------");
#endif

    RCLCPP_INFO(this->get_logger(), "Setup is done");
}

OctomapDemap::~OctomapDemap()
{
    if(!save_on_shutdown)
        return;

    if(save_ocmap())
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Save on shutdown successful " << filename);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Save on shutdown failed");
    }

#ifdef CUDA
    // deallocate memory
    cudaFree(gpu_depth);
    cudaFree(gpu_pc);
    free(pc);
#endif
}

bool OctomapDemap::octomap_srv(
    const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
    std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res)
{
    return msg_from_ocmap(res->map);
}

bool OctomapDemap::save_srv(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    if(save_ocmap())
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Octomap is saved to " << filename);
        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Octomap is not saved");
        return false;
    }
}

bool OctomapDemap::reset_srv(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    ocmap->clear();
    RCLCPP_INFO(this->get_logger(), "Octomap reset");
    return true;
}

void OctomapDemap::demap_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)
{
    auto cv_ptr = cv_bridge::toCvCopy(depth_msg, encoding);

    if (encoding == "mono8")
        update_map<uint8_t>(cv_ptr->image, pose_msg->pose);
    else if (encoding == "mono16")
        update_map<uint16_t>(cv_ptr->image, pose_msg->pose);
    else
        assert(false && "Invalid encoding!");

    publish_all();
}

void OctomapDemap::camerainfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cameraInfoMutex);

    if (cameraInfoPtr != nullptr)
        return;

    cameraInfoPtr = msg;

#ifdef CUDA
    pc_count = 0;
    // calculate point count (i can use some math later on, but bruh)
    for(uint32_t i = 0; i < cameraInfoPtr->width; i+=padding)
    {
        for(uint32_t j = 0; j < cameraInfoPtr->height; j+=padding)
        {
            pc_count+=3;
        }
    }

    pc_size = pc_count * sizeof(double);
    depth_size = cameraInfoPtr->width*cameraInfoPtr->height*sizeof(uint8_t);

    RCLCPP_INFO(this->get_logger(), "%d", pc_count);

    // allocate memory
    cudaMalloc<uint8_t>(&gpu_depth, depth_size);
    cudaMalloc<double>(&gpu_pc, pc_size);
    pc = (double*)malloc(pc_size);

    block.x = 32;
    block.y = 32;
    grid.x = (cameraInfoPtr->width + block.x - 1) / block.x;
    grid.y = (cameraInfoPtr->height + block.y - 1) / block.y;
#endif

    RCLCPP_INFO_STREAM(this->get_logger(), "width : " << cameraInfoPtr->width);
    RCLCPP_INFO_STREAM(this->get_logger(), "height : " << cameraInfoPtr->height);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/fx : " << cameraInfoPtr->k.at(K_FX_INDEX));
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/fy : " << cameraInfoPtr->k.at(K_FY_INDEX));
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/cx : " << cameraInfoPtr->k.at(K_CX_INDEX));
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/cy : " << cameraInfoPtr->k.at(K_CY_INDEX));
}

void OctomapDemap::publish_all()
{
    // ocmap->toMaxLikelihood();
    // ocmap->prune();

    octomap_msgs::msg::Octomap msg;
    msg_from_ocmap(msg);
    octomap_publisher_->publish(msg);
}

template<typename T>
void OctomapDemap::update_map(const cv::Mat& depth, const geometry_msgs::msg::Pose& pose)
{
    //Obtain camera intrinsics
    double width, fx, fy, cx, cy;
    {
        std::lock_guard<std::mutex> lock(cameraInfoMutex);

        if (cameraInfoPtr == nullptr)
            return;

        width = cameraInfoPtr->width;
        fx = cameraInfoPtr->k.at(K_FX_INDEX);
        fy = cameraInfoPtr->k.at(K_FY_INDEX);
        cx = cameraInfoPtr->k.at(K_CX_INDEX);
        cy = cameraInfoPtr->k.at(K_CY_INDEX);
    }

    const octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);

    octomap::Pointcloud currentPointCloud;
    currentPointCloud.reserve(depth.rows * depth.cols);

    const auto start = this->now();

#ifdef CUDA

    tf2::Transform t;
    tf2::fromMsg(pose, t);

    cudaMemcpy(gpu_depth ,depth.ptr(), depth_size, cudaMemcpyHostToDevice);

    auto b = t.getBasis();
    auto o = t.getOrigin();

  	project_depth_img(gpu_depth, gpu_pc, width, padding, max_distance,
        grid, block,
        fx, fy, cx, cy,
        b.getRow(0).getX(), b.getRow(0).getY(), b.getRow(0).getZ(),
        b.getRow(1).getX(), b.getRow(1).getY(), b.getRow(1).getZ(),
        b.getRow(2).getX(), b.getRow(2).getY(), b.getRow(2).getZ(),
        o.getX(), o.getY(), o.getZ());

    cudaMemcpy(pc, gpu_pc, pc_size, cudaMemcpyDeviceToHost);

    for(int i = 0, n = pc_count-3; i < n; i+=3)
    {
        if(pc[i] == 0 && pc[i+1] == 0 && pc[i+2] == 0) { continue; }

        currentPointCloud.push_back(octomap::point3d(pc[i], pc[i+1], pc[i+2]));
    }

#else

    const octomap::pose6d transform(
        origin,
        //note that octomath::Quaternion follows w, x, y, z convention
        octomath::Quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z)
    );

    for(int i = 0; i < depth.rows; i+=padding)
    {
        const T* row = depth.ptr<T>(i);

        for(int j = 0; j < depth.cols; j+=padding)
        {
            const double d = depth_to_meters(row[j], max_distance);

            if(d == 0)
                continue;

            currentPointCloud.push_back(
                (j - cx) * d / fx, //x
                (i - cy) * d / fy, //y
                d                  //z
            );
        }
    }

    currentPointCloud.transform(transform);

#endif

    ocmap->insertPointCloud(currentPointCloud, origin, max_distance, false, true);

    const auto end = this->now();
    const auto diff = end - start;

    RCLCPP_INFO(this->get_logger(), "update map time(sec) : %.4f", diff.seconds());
}

bool OctomapDemap::read_ocmap()
{
    if(filename.length() <= 3)
        return false;

    std::string ext = filename.substr(filename.length()-3, 3);

    if(ext == ".bt")
    {
        if (!ocmap->readBinary(filename))
            return false;
    }
    else if(ext == ".ot")
    {
        auto tree = octomap::AbstractOcTree::read(filename);
        octomap::OcTree *octree = dynamic_cast<octomap::OcTree*>(tree);
        ocmap = std::shared_ptr<octomap::OcTree>(octree);
    }
    else
        return false;

    if(!ocmap)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read octomap");
        return false;
    }

    publish_all();
    RCLCPP_INFO_STREAM(this->get_logger(), "Octomap read from " << filename);
    return true;
}

bool OctomapDemap::save_ocmap()
{
    if(filename.length() <= 3)
        return false;

    std::string ext = filename.substr(filename.length()-3, 3);

    if(ext == ".bt")
    {
        if (!ocmap->writeBinary(filename))
            return false;
    }
    else if(ext == ".ot")
    {
        if (!ocmap->write(filename))
            return false;
    }
    else
        return false;

    return true;
}

} // octomap_depth_mapping

RCLCPP_COMPONENTS_REGISTER_NODE(octomap_depth_mapping::OctomapDemap)
