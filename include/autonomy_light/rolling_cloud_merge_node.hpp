#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy_light {

class RollingCloudMergeNode final : public rclcpp::Node {
public:
  explicit RollingCloudMergeNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

  void onD435(Cloud::ConstSharedPtr message);
  void onLidar(Cloud::ConstSharedPtr message);
  void appendD435(const Cloud& camera, const std::string& target_frame,
                  PclCloud& merged, std::vector<std::uint8_t>& sensor_ids);
  void publish(const PclCloud& cloud, const std::vector<std::uint8_t>& sensor_ids,
               const std_msgs::msg::Header& header);

  std::string lidar_cloud_topic_{"/lio/cloud_world"};
  std::string d435_cloud_topic_{"/camera/camera/depth/color/points"};
  std::string output_topic_{"/autonomy_light/rolling_cloud"};
  double max_sync_delta_sec_{0.06};
  double tf_timeout_sec_{0.02};
  double d435_min_range_m_{0.15};
  double d435_max_range_m_{3.0};
  double d435_voxel_size_m_{0.03};

  std::deque<Cloud::ConstSharedPtr> d435_buffer_;
  rclcpp::Subscription<Cloud>::SharedPtr lidar_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr d435_sub_;
  rclcpp::Publisher<Cloud>::SharedPtr merged_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace autonomy_light
