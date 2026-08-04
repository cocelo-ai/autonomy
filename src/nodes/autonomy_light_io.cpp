#include "autonomy_light/autonomy_light_node.hpp"

#include <Eigen/Dense>

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

nav_msgs::msg::Odometry
AutonomyLightNode::baseOdom(const nav_msgs::msg::Odometry &imu_odom) const {
  nav_msgs::msg::Odometry base = imu_odom;
  base.header.frame_id = global_frame_;
  base.child_frame_id = target_frame_;
  tf2::Quaternion map_imu;
  tf2::fromMsg(imu_odom.pose.pose.orientation, map_imu);
  if (map_imu.length2() < 1.0e-12) {
    map_imu = tf2::Quaternion::getIdentity();
  }
  map_imu.normalize();
  const tf2::Quaternion imu_target = target_to_imu_rotation_.inverse();
  const tf2::Vector3 imu_to_target =
      tf2::quatRotate(imu_target, -target_to_imu_translation_);
  const tf2::Vector3 imu_position(imu_odom.pose.pose.position.x,
                                  imu_odom.pose.pose.position.y,
                                  imu_odom.pose.pose.position.z);
  const tf2::Vector3 base_position =
      imu_position + tf2::quatRotate(map_imu, imu_to_target);
  tf2::Quaternion map_base = map_imu * imu_target;
  map_base.normalize();
  base.pose.pose.position.x = base_position.x();
  base.pose.pose.position.y = base_position.y();
  base.pose.pose.position.z = base_position.z();
  base.pose.pose.orientation = tf2::toMsg(map_base);
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  bool has_covariance = false;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      const double value = imu_odom.pose.covariance[6 * row + col];
      covariance(row, col) = value;
      has_covariance = has_covariance || (row == col && value > 0.0);
    }
  }
  if (has_covariance && covariance.allFinite()) {
    const tf2::Vector3 offset = tf2::quatRotate(map_imu, imu_to_target);
    Eigen::Matrix3d skew;
    skew << 0.0, -offset.z(), offset.y(), offset.z(), 0.0, -offset.x(),
        -offset.y(), offset.x(), 0.0;
    Eigen::Matrix<double, 6, 6> jacobian =
        Eigen::Matrix<double, 6, 6>::Identity();
    jacobian.block<3, 3>(0, 3) = -skew;
    covariance = jacobian * covariance * jacobian.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        base.pose.covariance[6 * row + col] = covariance(row, col);
      }
    }
  }
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
  pose.has_covariance = true;
  for (std::size_t index = 0; index < pose.covariance.size(); ++index) {
    pose.covariance[index] = odom.pose.covariance[index];
    pose.has_covariance =
        pose.has_covariance && std::isfinite(pose.covariance[index]);
  }
  pose.has_covariance =
      pose.has_covariance &&
      (pose.covariance[0] > 0.0 || pose.covariance[7] > 0.0 ||
       pose.covariance[14] > 0.0 || pose.covariance[21] > 0.0 ||
       pose.covariance[28] > 0.0 || pose.covariance[35] > 0.0);
  return pose;
}

PointObservations AutonomyLightNode::observationsFrom(
    const sensor_msgs::msg::PointCloud2 &message,
    const nav_msgs::msg::Odometry &odom) const {
  PclCloud cloud;
  pcl::fromROSMsg(message, cloud);
  const auto field = [&message](const std::string &name,
                                const std::uint8_t datatype) {
    return std::find_if(
        message.fields.begin(), message.fields.end(),
        [&name, datatype](const sensor_msgs::msg::PointField &value) {
          return value.name == name && value.datatype == datatype;
        });
  };
  const bool has_source =
      field("sensor_id", sensor_msgs::msg::PointField::UINT8) !=
      message.fields.end();
  const bool has_origin =
      field("origin_x", sensor_msgs::msg::PointField::FLOAT32) !=
          message.fields.end() &&
      field("origin_y", sensor_msgs::msg::PointField::FLOAT32) !=
          message.fields.end() &&
      field("origin_z", sensor_msgs::msg::PointField::FLOAT32) !=
          message.fields.end();
  tf2::Quaternion map_base;
  tf2::fromMsg(odom.pose.pose.orientation, map_base);
  map_base.normalize();
  const tf2::Vector3 lidar_origin =
      tf2::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y,
                   odom.pose.pose.position.z) +
      tf2::quatRotate(map_base, target_to_lidar_translation_);
  PointObservations observations;
  observations.reserve(cloud.size());
  using ByteIterator = sensor_msgs::PointCloud2ConstIterator<std::uint8_t>;
  using FloatIterator = sensor_msgs::PointCloud2ConstIterator<float>;
  const auto source = has_source
                          ? std::make_unique<ByteIterator>(message, "sensor_id")
                          : nullptr;
  const auto origin_x =
      has_origin ? std::make_unique<FloatIterator>(message, "origin_x")
                 : nullptr;
  const auto origin_y =
      has_origin ? std::make_unique<FloatIterator>(message, "origin_y")
                 : nullptr;
  const auto origin_z =
      has_origin ? std::make_unique<FloatIterator>(message, "origin_z")
                 : nullptr;
  for (const auto &point : cloud.points) {
    PointObservation observation{point.x,
                                 point.y,
                                 point.z,
                                 source ? **source : 0U,
                                 static_cast<float>(lidar_origin.x()),
                                 static_cast<float>(lidar_origin.y()),
                                 static_cast<float>(lidar_origin.z())};
    if (origin_x && std::isfinite(**origin_x) && std::isfinite(**origin_y) &&
        std::isfinite(**origin_z)) {
      observation.origin_x = **origin_x;
      observation.origin_y = **origin_y;
      observation.origin_z = **origin_z;
    }
    observations.push_back(observation);
    if (source) {
      ++(*source);
    }
    if (origin_x) {
      ++(*origin_x);
      ++(*origin_y);
      ++(*origin_z);
    }
  }
  return observations;
}

void AutonomyLightNode::onTimer() {
  if (shutdown_requested_) {
    return;
  }
  nav_msgs::msg::Odometry odom;
  std::uint64_t odom_count = 0U;
  std::uint64_t cloud_count = 0U;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (has_odom_) {
      odom = latest_odom_;
    }
    odom_count = odom_count_;
    cloud_count = cloud_count_;
  }
  std_msgs::msg::String heartbeat;
  if (odom.header.stamp.sec == 0 && odom.header.stamp.nanosec == 0) {
    heartbeat.data = "waiting_for_super_lio_odom";
  } else if (cloud_count == 0U) {
    heartbeat.data = "waiting_for_super_lio_map";
  } else {
    heartbeat.data = "ready:source=" + mapper_config_.source +
                     ":transport=" + height_map_transport_ +
                     ":odom=" + std::to_string(odom_count) +
                     ":cloud=" + std::to_string(cloud_count);
  }
  heartbeat_pub_->publish(heartbeat);
  if (odom.header.stamp.sec == 0 && odom.header.stamp.nanosec == 0) {
    publishInitialGravityTransform();
    return;
  }
  if (mapping_only_) {
    publishPose(odom);
    return;
  }
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = height_map_frame_;
  {
    std::unique_lock<std::mutex> lock(mapper_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      latest_grid_ = mapper_.build(
          poseOf(odom), header.stamp.sec + 1.0e-9 * header.stamp.nanosec,
          header);
    }
  }
  HeightGrid grid = latest_grid_.value_or(HeightGrid(grid_spec_));
  grid.header = header;
  publishHeight(grid);
  publishPose(odom);
}

void AutonomyLightNode::publishMap() {
  PclCloud::Ptr map;
  {
    std::unique_lock<std::mutex> lock(mapper_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || mapper_.empty()) {
      return;
    }
    map = mapper_.globalMap();
  }
  if (!map || map->empty()) {
    return;
  }
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
  std::vector<float> height_data(grid.spec.size(),
                                 static_cast<float>(unknown_distance_));
  for (std::size_t index = 0; index < grid.spec.size(); ++index) {
    if (grid.valid[index] != 0U && std::isfinite(grid.height[index])) {
      height_data[index] = static_cast<float>(
          std::clamp(-static_cast<double>(grid.height[index]), distance_min_,
                     distance_max_));
    }
  }
  if (publishesDdsHeight() && dds_height_map_pub_ &&
      !dds_height_map_pub_->publish(height_data)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Cyclone DDS height map write failed: %s",
                          dds_height_map_pub_->error().c_str());
  }
  if (!publishesRosHeight()) {
    return;
  }

  msg::HeightMap data;
  data.header = grid.header;
  data.resolution = static_cast<float>(grid.spec.resolution);
  data.x_length = static_cast<float>(grid.spec.x_length);
  data.y_length = static_cast<float>(grid.spec.y_length);
  data.data = std::move(height_data);
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
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(grid.spec.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  for (std::uint32_t row = 0; row < grid.spec.height(); ++row) {
    for (std::uint32_t col = 0; col < grid.spec.width(); ++col) {
      const auto index =
          static_cast<std::size_t>(row) * grid.spec.width() + col;
      *x = static_cast<float>(grid.spec.xMin() +
                              (col + 0.5) * grid.spec.resolution);
      *y = static_cast<float>(grid.spec.yMin() +
                              (row + 0.5) * grid.spec.resolution);
      const bool valid = grid.valid[index] != 0U &&
                         std::isfinite(grid.height[index]);
      *z = valid ? grid.height[index]
                 : static_cast<float>(-unknown_distance_);
      *intensity = valid ? 1.0F : 0.0F;
      ++x;
      ++y;
      ++z;
      ++intensity;
    }
  }
  height_pub_->publish(cloud);
  height_msg_pub_->publish(data);
  quality_pub_->publish(quality);
}

void AutonomyLightNode::publishPose(const nav_msgs::msg::Odometry &odom) {
  nav_msgs::msg::Odometry current = odom;
  current.header.stamp = now();
  std::optional<nav_msgs::msg::Path> path;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (published_path_version_ != path_version_) {
      path = path_;
      path->header = current.header;
      published_path_version_ = path_version_;
    }
  }
  odom_pub_->publish(current);
  if (path) {
    path_pub_->publish(*path);
  }
  publishGravityTransform(current);
}

void AutonomyLightNode::publishGravityTransform(
    const nav_msgs::msg::Odometry &odom) {
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
  geometry_msgs::msg::TransformStamped height;
  height.header.stamp = now();
  height.header.frame_id = target_frame_;
  height.child_frame_id = height_map_frame_;
  tf2::Quaternion base_height = map_base.inverse() * map_height;
  base_height.normalize();
  height.transform.rotation = tf2::toMsg(base_height);
  tf_broadcaster_->sendTransform(height);
}

void AutonomyLightNode::publishInitialGravityTransform() {
  if (height_map_frame_ == target_frame_) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = target_frame_;
  transform.child_frame_id = height_map_frame_;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

} // namespace autonomy_light
