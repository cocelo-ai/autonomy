#include "autonomy_light/autonomy_light_node.hpp"

namespace autonomy_light {

tf2::Quaternion AutonomyLightNode::yawOnly(
    const geometry_msgs::msg::Quaternion &orientation) const {
  tf2::Quaternion full;
  tf2::fromMsg(orientation, full);
  if (full.length2() < 1.0e-12) {
    return tf2::Quaternion::getIdentity();
  }
  full.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(full).getRPY(roll, pitch, yaw);
  tf2::Quaternion result;
  result.setRPY(0.0, 0.0, yaw);
  result.normalize();
  return result;
}

nav_msgs::msg::Odometry AutonomyLightNode::baseOdom(
    const nav_msgs::msg::Odometry &lidar_odom) const {
  nav_msgs::msg::Odometry base = lidar_odom;
  base.header.frame_id = global_frame_;
  base.child_frame_id = target_frame_;
  if (target_frame_ == lidar_frame_) {
    return base;
  }
  tf2::Quaternion map_lidar;
  tf2::fromMsg(lidar_odom.pose.pose.orientation, map_lidar);
  if (map_lidar.length2() < 1.0e-12) {
    map_lidar = tf2::Quaternion::getIdentity();
  }
  map_lidar.normalize();
  const tf2::Quaternion lidar_base = target_to_lidar_rotation_.inverse();
  const tf2::Vector3 lidar_to_base = tf2::quatRotate(
      lidar_base, -target_to_lidar_translation_);
  const tf2::Vector3 lidar_position(lidar_odom.pose.pose.position.x,
                                    lidar_odom.pose.pose.position.y,
                                    lidar_odom.pose.pose.position.z);
  const tf2::Vector3 base_position = lidar_position +
      tf2::quatRotate(map_lidar, lidar_to_base);
  tf2::Quaternion map_base = map_lidar * lidar_base;
  map_base.normalize();
  base.pose.pose.position.x = base_position.x();
  base.pose.pose.position.y = base_position.y();
  base.pose.pose.position.z = base_position.z();
  base.pose.pose.orientation = tf2::toMsg(map_base);
  return base;
}

Pose2_5D AutonomyLightNode::poseOf(const nav_msgs::msg::Odometry &odom) const {
  const auto yaw = yawOnly(odom.pose.pose.orientation);
  double roll = 0.0;
  double pitch = 0.0;
  Pose2_5D pose;
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;
  pose.z = odom.pose.pose.position.z;
  tf2::Matrix3x3(yaw).getRPY(roll, pitch, pose.yaw);
  return pose;
}

void AutonomyLightNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr message) {
  if (!message || shutdown_requested_) {
    return;
  }
  latest_odom_ = baseOdom(*message);
  if (latest_odom_.header.stamp.sec == 0 && latest_odom_.header.stamp.nanosec == 0) {
    latest_odom_.header.stamp = now();
  }
  latest_odom_.header.frame_id = global_frame_;
  has_odom_ = true;
  ++odom_count_;

  path_.header = latest_odom_.header;
  if (path_.poses.empty() ||
      std::hypot(path_.poses.back().pose.position.x - latest_odom_.pose.pose.position.x,
                 path_.poses.back().pose.position.y - latest_odom_.pose.pose.position.y) > 0.02) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = latest_odom_.header;
    pose.pose = latest_odom_.pose.pose;
    path_.poses.push_back(std::move(pose));
    if (path_.poses.size() > 10'000U) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 1'000);
    }
  }
  publishPose(latest_odom_, latest_odom_.pose.pose.position.z);
}

void AutonomyLightNode::onRegisteredCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr message) {
  if (!message || shutdown_requested_ || !has_odom_) {
    return;
  }
  PclCloud cloud;
  try {
    pcl::fromROSMsg(*message, cloud);
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cannot read Super-LIO cloud: %s", error.what());
    return;
  }
  const rclcpp::Time stamp = message->header.stamp.sec == 0 &&
                                      message->header.stamp.nanosec == 0
                                  ? now()
                                  : rclcpp::Time(message->header.stamp);
  mapper_.integrate(cloud, poseOf(latest_odom_), stamp.seconds());
  ++cloud_count_;
}

void AutonomyLightNode::onTimer() {
  if (shutdown_requested_) {
    return;
  }
  publishMap();
  std_msgs::msg::String heartbeat;
  if (!has_odom_) {
    heartbeat.data = "waiting_for_super_lio_odom";
  } else if (cloud_count_ == 0U) {
    heartbeat.data = "waiting_for_super_lio_map";
  } else {
    heartbeat.data = "ready:source=" + mapper_config_.source +
                     ":odom=" + std::to_string(odom_count_) +
                     ":cloud=" + std::to_string(cloud_count_);
  }
  heartbeat_pub_->publish(heartbeat);
  if (mapping_only_ || !has_odom_) {
    return;
  }
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = height_map_frame_;
  const HeightGrid grid = mapper_.build(poseOf(latest_odom_), header.stamp.sec +
      1.0e-9 * header.stamp.nanosec, header);
  const bool observed = std::any_of(grid.valid.begin(), grid.valid.end(),
                                    [](std::uint8_t value) { return value != 0U; });
  if (observed) {
    publishHeight(grid);
    publishPose(latest_odom_, grid.floor_z);
  }
}

void AutonomyLightNode::publishMap() {
  if (mapper_.empty()) {
    return;
  }
  const auto current = std::chrono::steady_clock::now();
  if (last_map_publish_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(current - last_map_publish_).count() <
          map_publish_interval_sec_) {
    return;
  }
  last_map_publish_ = current;
  const auto map = mapper_.globalMap();
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*map, message);
  message.header.stamp = now();
  message.header.frame_id = global_frame_;
  map_pub_->publish(message);
  if (saved_map_cloud_ && !saved_map_cloud_->empty()) {
    sensor_msgs::msg::PointCloud2 saved;
    pcl::toROSMsg(*saved_map_cloud_, saved);
    saved.header.stamp = message.header.stamp;
    saved.header.frame_id = saved_map_frame_;
    saved_map_pub_->publish(saved);
  }
}

void AutonomyLightNode::publishHeight(const HeightGrid &grid) {
  msg::HeightMap data;
  data.header = grid.header;
  data.resolution = static_cast<float>(grid.spec.resolution);
  data.x_length = static_cast<float>(grid.spec.x_length);
  data.y_length = static_cast<float>(grid.spec.y_length);
  data.data.resize(grid.spec.size(), static_cast<float>(unknown_distance_));
  for (std::size_t index = 0; index < grid.spec.size(); ++index) {
    if (grid.valid[index] != 0U && std::isfinite(grid.height[index])) {
      data.data[index] = static_cast<float>(std::clamp(
          reference_height_ - static_cast<double>(grid.height[index]),
          distance_min_, distance_max_));
    }
  }
  msg::HeightMapQuality quality;
  quality.header = grid.header;
  quality.resolution = data.resolution;
  quality.x_length = data.x_length;
  quality.y_length = data.y_length;
  quality.valid = grid.valid;
  quality.variance = grid.variance;
  quality.age = grid.age;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = grid.header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(static_cast<std::size_t>(
      std::count(grid.valid.begin(), grid.valid.end(), static_cast<std::uint8_t>(1U))));
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (std::uint32_t row = 0; row < grid.spec.height(); ++row) {
    for (std::uint32_t col = 0; col < grid.spec.width(); ++col) {
      const auto index = static_cast<std::size_t>(row) * grid.spec.width() + col;
      if (grid.valid[index] == 0U) {
        continue;
      }
      *x = static_cast<float>(grid.spec.xMin() + (col + 0.5) * grid.spec.resolution);
      *y = static_cast<float>(grid.spec.yMin() + (row + 0.5) * grid.spec.resolution);
      *z = grid.height[index];
      ++x;
      ++y;
      ++z;
    }
  }
  height_pub_->publish(cloud);
  height_msg_pub_->publish(data);
  quality_pub_->publish(quality);
}

void AutonomyLightNode::publishPose(const nav_msgs::msg::Odometry &odom,
                                    const double floor_z) {
  odom_pub_->publish(odom);
  path_pub_->publish(path_);
  geometry_msgs::msg::TransformStamped base;
  base.header = odom.header;
  base.header.frame_id = global_frame_;
  base.child_frame_id = target_frame_;
  base.transform.translation.x = odom.pose.pose.position.x;
  base.transform.translation.y = odom.pose.pose.position.y;
  base.transform.translation.z = odom.pose.pose.position.z;
  base.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_->sendTransform(base);
  if (height_map_frame_ == target_frame_) {
    return;
  }
  const tf2::Quaternion map_base = [&]() {
    tf2::Quaternion value;
    tf2::fromMsg(odom.pose.pose.orientation, value);
    value.normalize();
    return value;
  }();
  const tf2::Quaternion map_height = yawOnly(odom.pose.pose.orientation);
  const tf2::Vector3 map_position(odom.pose.pose.position.x,
                                  odom.pose.pose.position.y,
                                  odom.pose.pose.position.z);
  const tf2::Vector3 map_floor(odom.pose.pose.position.x,
                               odom.pose.pose.position.y, floor_z);
  geometry_msgs::msg::TransformStamped height;
  height.header = odom.header;
  height.header.frame_id = target_frame_;
  height.child_frame_id = height_map_frame_;
  const tf2::Vector3 local_floor = tf2::quatRotate(map_base.inverse(),
                                                    map_floor - map_position);
  height.transform.translation.x = local_floor.x();
  height.transform.translation.y = local_floor.y();
  height.transform.translation.z = local_floor.z();
  tf2::Quaternion base_height = map_base.inverse() * map_height;
  base_height.normalize();
  height.transform.rotation = tf2::toMsg(base_height);
  tf_broadcaster_->sendTransform(height);
}

} // namespace autonomy_light
