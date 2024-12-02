#pragma once
#ifndef OCTOMAP_DEPTH_MAPPING_HPP
#define OCTOMAP_DEPTH_MAPPING_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <optional>

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
    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    // Projects 2D points in the camera coordinate frame to 3D pixel
    // coordinates using the focal lengths (fx, fy) and principal point
    // (cx, cy).
    // We only need fx, fy, cx, cy
    std::mutex kMutex;
    std::optional<std::array<double, 9UL>> k = std::nullopt; // camera intrinsic for projection

    double fx;
    double fy;
    double cx;
    double cy;
    double max_distance;
    int padding;
    int width;
    int height;
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

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camerainfo_sub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
        geometry_msgs::msg::PoseStamped> ApproxTimePolicy;

    std::shared_ptr<message_filters::Synchronizer<ApproxTimePolicy>> sync_;

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