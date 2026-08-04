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

nav_msgs::msg::Odometry AutonomyLightNode::baseOdom(
    const nav_msgs::msg::Odometry &imu_odom) const {
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
  const tf2::Vector3 imu_to_target = tf2::quatRotate(
      imu_target, -target_to_imu_translation_);
  const tf2::Vector3 imu_position(imu_odom.pose.pose.position.x,
                                  imu_odom.pose.pose.position.y,
                                  imu_odom.pose.pose.position.z);
  const tf2::Vector3 base_position = imu_position +
      tf2::quatRotate(map_imu, imu_to_target);
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
    skew << 0.0, -offset.z(), offset.y(),
            offset.z(), 0.0, -offset.x(),
            -offset.y(), offset.x(), 0.0;
    Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Identity();
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
    pose.has_covariance = pose.has_covariance && std::isfinite(pose.covariance[index]);
  }
  pose.has_covariance = pose.has_covariance &&
      (pose.covariance[0] > 0.0 || pose.covariance[7] > 0.0 ||
       pose.covariance[14] > 0.0 || pose.covariance[21] > 0.0 ||
       pose.covariance[28] > 0.0 || pose.covariance[35] > 0.0);
  return pose;
}

const nav_msgs::msg::Odometry *AutonomyLightNode::odomAt(
    const rclcpp::Time &stamp) const {
  const nav_msgs::msg::Odometry *closest = nullptr;
  double closest_delta = std::numeric_limits<double>::infinity();
  for (const auto &odom : odom_history_) {
    const double delta = std::abs((rclcpp::Time(odom.header.stamp) - stamp).seconds());
    if (delta < closest_delta) {
      closest = &odom;
      closest_delta = delta;
    }
  }
  return closest_delta <= odom_sync_tolerance_sec_ ? closest : nullptr;
}

PointObservations AutonomyLightNode::observationsFrom(
    const sensor_msgs::msg::PointCloud2 &message) const {
  PclCloud cloud;
  pcl::fromROSMsg(message, cloud);
  const auto source_field = std::find_if(message.fields.begin(), message.fields.end(),
      [](const sensor_msgs::msg::PointField &field) {
        return field.name == "sensor_id" && field.datatype == sensor_msgs::msg::PointField::UINT8;
      });
  PointObservations observations;
  observations.reserve(cloud.size());
  if (source_field == message.fields.end()) {
    for (const auto &point : cloud.points) {
      observations.push_back({point.x, point.y, point.z, 0U});
    }
    return observations;
  }
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> source(message, "sensor_id");
  for (const auto &point : cloud.points) {
    observations.push_back({point.x, point.y, point.z, *source});
    ++source;
  }
  return observations;
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
  odom_history_.push_back(latest_odom_);
  while (odom_history_.size() > 64U) {
    odom_history_.pop_front();
  }
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
  try {
    const rclcpp::Time stamp = message->header.stamp.sec == 0 &&
                                      message->header.stamp.nanosec == 0
                                  ? now()
                                  : rclcpp::Time(message->header.stamp);
    const auto *odom = odomAt(stamp);
    if (!odom) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping cloud without an odometry sample within %.3fs",
                           odom_sync_tolerance_sec_);
      return;
    }
    mapper_.integrate(observationsFrom(*message), poseOf(*odom), stamp.seconds());
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cannot read Super-LIO cloud: %s", error.what());
    return;
  }
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
                     ":transport=" + height_map_transport_ +
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
  std::vector<float> height_data(grid.spec.size(),
                                 static_cast<float>(unknown_distance_));
  for (std::size_t index = 0; index < grid.spec.size(); ++index) {
    if (grid.valid[index] != 0U && std::isfinite(grid.height[index])) {
      height_data[index] = static_cast<float>(std::clamp(
          reference_height_ - static_cast<double>(grid.height[index]),
          distance_min_, distance_max_));
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
