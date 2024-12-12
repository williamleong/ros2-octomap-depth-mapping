#pragma once
#ifndef OCTOMAP_DEPTH_MAPPING_HPP
#define OCTOMAP_DEPTH_MAPPING_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

#include "octomap/octomap.h"
#include "octomap/Pointcloud.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"

#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <opencv2/opencv.hpp>

#ifdef CUDA
#include <cuda_runtime.h>
#endif

namespace octomap_depth_mapping
{

class OctomapDemap : public rclcpp::Node
{
protected:
    std::vector<std::string> input_prefixes;

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    // Projects 2D points in the camera coordinate frame to 3D pixel
    // coordinates using the focal lengths (fx, fy) and principal point
    // (cx, cy).
    // We only need fx, fy, cx, cy

    static constexpr size_t K_FX_INDEX = 0;
    static constexpr size_t K_FY_INDEX = 4;
    static constexpr size_t K_CX_INDEX = 2;
    static constexpr size_t K_CY_INDEX = 5;

    std::mutex cameraInfoMutex;
    sensor_msgs::msg::CameraInfo::SharedPtr cameraInfoPtr = nullptr;

    double min_z;
    double max_z;
    double max_distance;
    uint8_t padding;
    std::string encoding;
    std::string frame_id;
    std::string filename;
    bool save_on_shutdown;

    std::shared_ptr<octomap::OcTree> ocmap;

#ifdef CUDA
    uint8_t* gpu_depth;
    double* gpu_pc;
    double* pc;
    int pc_count;
    size_t pc_size;
    size_t depth_size;
    dim3 block;
    dim3 grid;
#endif

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;

    // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camerainfo_sub_;
    // message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    // message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;

    using depth_subscriber_t = message_filters::Subscriber<sensor_msgs::msg::Image>;
    using pose_subscriber_t = message_filters::Subscriber<geometry_msgs::msg::PoseStamped>;
    using camerainfo_subscriber_t = rclcpp::Subscription<sensor_msgs::msg::CameraInfo>;

    std::vector<std::shared_ptr<depth_subscriber_t>> depthSubscribers;
    std::vector<std::shared_ptr<pose_subscriber_t>> poseSubscribers;
    std::vector<camerainfo_subscriber_t::SharedPtr> cameraInfoSubscribers;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
        geometry_msgs::msg::PoseStamped> ApproxTimePolicy;

    using approxtime_filter_t = std::shared_ptr<message_filters::Synchronizer<ApproxTimePolicy>>;

    std::vector<approxtime_filter_t> depthPoseSynchronizers;

    rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_srv_;

    template<typename T>
    void update_map(const cv::Mat&, const geometry_msgs::msg::Pose&);

    void publish_all();

    void demap_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr&,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr&);

    void camerainfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camerainfo_msg);

    bool octomap_srv(
        const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
        std::shared_ptr<octomap_msgs::srv::GetOctomap::Response>);

    bool reset_srv(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        const std::shared_ptr<std_srvs::srv::Empty::Response>);

    bool save_srv(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        const std::shared_ptr<std_srvs::srv::Empty::Response>);

    bool save_ocmap();
    bool read_ocmap();

    inline bool msg_from_ocmap(octomap_msgs::msg::Octomap& msg)
    {
        ocmap->prune();

        msg.id = "OcTree";
        msg.header.frame_id = frame_id;
        return octomap_msgs::fullMapToMsg(*ocmap, msg);
    }

public:

    explicit OctomapDemap(const rclcpp::NodeOptions&, const std::string = "octomap_depth_mapping");
    ~OctomapDemap();

};

} // octomap_depth_mapping

#endif // OCTOMAP_DEPTH_MAPPING_HPP