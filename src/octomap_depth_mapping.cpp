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
    fx(524),
    fy(524),
    cx(316.8),
    cy(238.5),
    max_distance(10.0),
    padding(1),
    width(640),
    height(480),
    encoding("mono16"),
    frame_id("map"),
    filename(""),
    save_on_shutdown(false)
{
    fx = this->declare_parameter("sensor_model/fx", fx);
    fy = this->declare_parameter("sensor_model/fy", fy);
    cx = this->declare_parameter("sensor_model/cx", cx);
    cy = this->declare_parameter("sensor_model/cy", cy);
    max_distance = this->declare_parameter("output/max_distance", max_distance);
    frame_id = this->declare_parameter("frame_id", frame_id);
    padding = this->declare_parameter("padding", padding);
    filename = this->declare_parameter("filename", filename);
    encoding = this->declare_parameter("encoding", encoding);
    save_on_shutdown = this->declare_parameter("save_on_shutdown", save_on_shutdown);
    width = this->declare_parameter("width", width);
    height = this->declare_parameter("height", height);

    rclcpp::QoS qos(rclcpp::KeepLast(3));

    // pubs
    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("map_out", qos);

    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    // subs
    depth_sub_.subscribe(this, "image_in", rmw_qos_profile);
    pose_sub_.subscribe(this, "pose_in", rmw_qos_profile);

    // bind subs with ugly way
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
        geometry_msgs::msg::PoseStamped>>(depth_sub_, pose_sub_, 3);
    sync_->registerCallback(std::bind(&OctomapDemap::demap_callback, this, ph::_1, ph::_2));

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

#ifdef CUDA
    pc_count = 0;
    // calculate point count (i can use some math later on, but bruh)
    for(int i = 0; i < width; i+=padding)
    {
        for(int j = 0; j < height; j+=padding)
        {
            pc_count+=3;
        }
    }

    pc_size = pc_count * sizeof(double);
    depth_size = width*height*sizeof(uint8_t);

    RCLCPP_INFO(this->get_logger(), "%d", pc_count);

    // allocate memory
    cudaMalloc<uint8_t>(&gpu_depth, depth_size);
    cudaMalloc<double>(&gpu_pc, pc_size);
    pc = (double*)malloc(pc_size);

    block.x = 32;
    block.y = 32;
    grid.x = (width + block.x - 1) / block.x;
    grid.y = (height + block.y - 1) / block.y;
#endif

    RCLCPP_INFO(this->get_logger(), "--- Launch Parameters ---");
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/fx : " << fx);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/fy : " << fy);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/cx : " << cx);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/cy : " << cy);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/hit : " << probHit);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/miss : " << probMiss);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/min : " << thresMin);
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model/max : " << thresMax);
    RCLCPP_INFO_STREAM(this->get_logger(), "output/max_distance : " << max_distance);
    RCLCPP_INFO_STREAM(this->get_logger(), "resolution : " << resolution);
    RCLCPP_INFO_STREAM(this->get_logger(), "encoding : " << encoding);
    RCLCPP_INFO_STREAM(this->get_logger(), "width : " << width);
    RCLCPP_INFO_STREAM(this->get_logger(), "height : " << height);
    RCLCPP_INFO_STREAM(this->get_logger(), "padding : " << padding);
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

void OctomapDemap::publish_all()
{
    octomap_msgs::msg::Octomap msg;
    msg_from_ocmap(msg);
    octomap_publisher_->publish(msg);
}

template<typename T>
void OctomapDemap::update_map(const cv::Mat& depth, const geometry_msgs::msg::Pose& pose)
{
    static const auto LIMIT_SQUARED = max_distance * max_distance;

    tf2::Transform t;
    tf2::fromMsg(pose, t);
    octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);

    auto start = this->now();

#ifdef CUDA
    cudaMemcpy(gpu_depth ,depth.ptr(), depth_size, cudaMemcpyHostToDevice);

    auto b = t.getBasis();
    auto o = t.getOrigin();

  	project_depth_img(gpu_depth, gpu_pc, width, padding,
        grid, block,
        fx, fy, cx, cy,
        b.getRow(0).getX(), b.getRow(0).getY(), b.getRow(0).getZ(),
        b.getRow(1).getX(), b.getRow(1).getY(), b.getRow(1).getZ(),
        b.getRow(2).getX(), b.getRow(2).getY(), b.getRow(2).getZ(),
        o.getX(), o.getY(), o.getZ());

    cudaMemcpy(pc, gpu_pc, pc_size, cudaMemcpyDeviceToHost);

    for(int i = 0, n = pc_count-3; i < n; i+=3)
    {
        // if(pc[i] == 0 && pc[i+1] == 0 && pc[i+2] == 0) { continue; }

        const auto p = octomap::point3d(pc[i], pc[i+1], pc[i+2]);
        const auto pNormSq = p.norm_sq();

        if (pNormSq == 0 || pNormSq > LIMIT_SQUARED)
            continue;

        ocmap->insertRay(origin, p);
    }
#else
    tf2::Vector3 p;

    for(int i = 0; i < depth.rows; i+=padding)
    {
        const T* row = depth.ptr<T>(i);

        for(int j = 0; j < depth.cols; j+=padding)
        {
            const double d = depth_to_meters(row[j]);

            if(d == 0)
                continue;

            p.setX((j - cx) * d / fx);
            p.setY((i - cy) * d / fy);
            p.setZ(d);

            if (p.length2() > LIMIT_SQUARED)
                continue;

            p = t(p);

            ocmap->insertRay(origin, octomap::point3d(p.getX(), p.getY(), p.getZ()));
        }
    }
#endif

    auto end = this->now();
    auto diff = end - start;
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
