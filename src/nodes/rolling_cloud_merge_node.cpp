#include "autonomy_light/rolling_cloud_merge_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace autonomy_light {
namespace {

double stampSeconds(const std_msgs::msg::Header &header) {
  return static_cast<double>(header.stamp.sec) +
         static_cast<double>(header.stamp.nanosec) * 1.0e-9;
}

} // namespace

RollingCloudMergeNode::RollingCloudMergeNode(const rclcpp::NodeOptions &options)
    : Node("rolling_cloud_merge", options), tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_) {
  lidar_cloud_topic_ =
      declare_parameter<std::string>("lidar_cloud_topic", lidar_cloud_topic_);
  d435_cloud_topic_ =
      declare_parameter<std::string>("d435_cloud_topic", d435_cloud_topic_);
  output_topic_ = declare_parameter<std::string>("output_topic", output_topic_);
  max_sync_delta_sec_ =
      std::max(0.0, declare_parameter<double>("max_sync_delta_sec",
                                              max_sync_delta_sec_));
  tf_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_));
  d435_min_range_m_ = std::max(
      0.0, declare_parameter<double>("d435_min_range_m", d435_min_range_m_));
  d435_max_range_m_ =
      std::max(d435_min_range_m_, declare_parameter<double>("d435_max_range_m",
                                                            d435_max_range_m_));
  d435_voxel_size_m_ = std::max(
      0.0, declare_parameter<double>("d435_voxel_size_m", d435_voxel_size_m_));

  const auto qos = rclcpp::SensorDataQoS();
  d435_sub_ = create_subscription<Cloud>(
      d435_cloud_topic_, qos,
      [this](Cloud::ConstSharedPtr message) { onD435(std::move(message)); });
  lidar_sub_ = create_subscription<Cloud>(
      lidar_cloud_topic_, qos,
      [this](Cloud::ConstSharedPtr message) { onLidar(std::move(message)); });
  merged_pub_ = create_publisher<Cloud>(output_topic_, qos);
  RCLCPP_INFO(get_logger(), "rolling merge: lidar=%s d435=%s output=%s",
              lidar_cloud_topic_.c_str(), d435_cloud_topic_.c_str(),
              output_topic_.c_str());
}

void RollingCloudMergeNode::onD435(Cloud::ConstSharedPtr message) {
  d435_buffer_.push_back(std::move(message));
  while (d435_buffer_.size() > 8U) {
    d435_buffer_.pop_front();
  }
}

void RollingCloudMergeNode::onLidar(Cloud::ConstSharedPtr message) {
  PclCloud merged;
  pcl::fromROSMsg(*message, merged);
  std::vector<std::uint8_t> sensor_ids(merged.size(), 0U);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<std::array<float, 3>> origins(merged.size(), {nan, nan, nan});
  Cloud::ConstSharedPtr closest;
  double closest_delta = std::numeric_limits<double>::infinity();
  for (const auto &camera : d435_buffer_) {
    const double delta =
        std::abs(stampSeconds(message->header) - stampSeconds(camera->header));
    if (delta < closest_delta) {
      closest = camera;
      closest_delta = delta;
    }
  }
  if (closest && closest_delta <= max_sync_delta_sec_) {
    appendD435(*closest, message->header.frame_id, merged, sensor_ids, origins);
  } else if (closest) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "D435 cloud is outside the %.3fs merge window",
                         max_sync_delta_sec_);
  }
  publish(merged, sensor_ids, origins, message->header);
}

void RollingCloudMergeNode::appendD435(
    const Cloud &camera, const std::string &target_frame, PclCloud &merged,
    std::vector<std::uint8_t> &sensor_ids,
    std::vector<std::array<float, 3>> &origins) {
  if (target_frame.empty() || camera.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot merge D435 cloud without source and target frames");
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
        target_frame, camera.header.frame_id, camera.header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "D435 TF %s -> %s at sensor stamp is unavailable: %s",
                         camera.header.frame_id.c_str(), target_frame.c_str(),
                         error.what());
    return;
  }

  PclCloud source;
  pcl::fromROSMsg(camera, source);
  tf2::Transform tf;
  tf2::fromMsg(transform.transform, tf);
  const std::array<float, 3> origin{static_cast<float>(tf.getOrigin().x()),
                                    static_cast<float>(tf.getOrigin().y()),
                                    static_cast<float>(tf.getOrigin().z())};
  PclCloud transformed;
  transformed.reserve(source.size());
  for (const auto &point : source.points) {
    const double range =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (!std::isfinite(range) || range < d435_min_range_m_ ||
        range > d435_max_range_m_) {
      continue;
    }
    const auto result = tf * tf2::Vector3(point.x, point.y, point.z);
    transformed.emplace_back(result.x(), result.y(), result.z());
  }
  if (transformed.empty()) {
    return;
  }
  if (d435_voxel_size_m_ > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(transformed.makeShared());
    const auto leaf = static_cast<float>(d435_voxel_size_m_);
    filter.setLeafSize(leaf, leaf, leaf);
    PclCloud filtered;
    filter.filter(filtered);
    merged += filtered;
    sensor_ids.insert(sensor_ids.end(), filtered.size(), 1U);
    origins.insert(origins.end(), filtered.size(), origin);
  } else {
    merged += transformed;
    sensor_ids.insert(sensor_ids.end(), transformed.size(), 1U);
    origins.insert(origins.end(), transformed.size(), origin);
  }
}

void RollingCloudMergeNode::publish(
    const PclCloud &cloud, const std::vector<std::uint8_t> &sensor_ids,
    const std::vector<std::array<float, 3>> &origins,
    const std_msgs::msg::Header &header) {
  Cloud output;
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(
      7, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "sensor_id", 1,
      sensor_msgs::msg::PointField::UINT8, "origin_x", 1,
      sensor_msgs::msg::PointField::FLOAT32, "origin_y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "origin_z", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(cloud.size());
  sensor_msgs::PointCloud2Iterator<float> x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z(output, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> sensor_id(output, "sensor_id");
  sensor_msgs::PointCloud2Iterator<float> origin_x(output, "origin_x");
  sensor_msgs::PointCloud2Iterator<float> origin_y(output, "origin_y");
  sensor_msgs::PointCloud2Iterator<float> origin_z(output, "origin_z");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (std::size_t index = 0; index < cloud.size(); ++index, ++x, ++y, ++z,
                   ++sensor_id, ++origin_x, ++origin_y, ++origin_z) {
    *x = cloud[index].x;
    *y = cloud[index].y;
    *z = cloud[index].z;
    *sensor_id = index < sensor_ids.size() ? sensor_ids[index] : 0U;
    const auto origin = index < origins.size()
                            ? origins[index]
                            : std::array<float, 3>{nan, nan, nan};
    *origin_x = origin[0];
    *origin_y = origin[1];
    *origin_z = origin[2];
  }
  output.header = header;
  merged_pub_->publish(output);
}

} // namespace autonomy_light
