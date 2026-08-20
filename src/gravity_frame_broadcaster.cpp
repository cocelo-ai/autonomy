#include "autonomy_light/gravity_frame_broadcaster.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy_light {

struct GravityFrameBroadcaster::Impl {
  explicit Impl(GravityFrameBroadcaster &node)
      : node(node), tf_buffer(node.get_clock()), tf_listener(tf_buffer), broadcaster(node) {
    orientation_source = node.declare_parameter<std::string>("orientation_source", "slam");
    map_frame = node.declare_parameter<std::string>("map_frame", "map");
    base_frame = node.declare_parameter<std::string>("base_frame", "base_link");
    gravity_frame = node.declare_parameter<std::string>("gravity_frame", "base_link_gravity");
    rate_hz = node.declare_parameter<double>("publish_rate_hz", 50.0);
    tf_timeout_sec = node.declare_parameter<double>("tf_timeout_sec", 0.02);
    imu_topic = node.declare_parameter<std::string>("imu_topic", "");
    imu_mount_frame = node.declare_parameter<std::string>("imu_mount_frame", "");
    const auto imu_to_mount_rpy = node.declare_parameter<std::vector<double>>(
        "imu_to_mount_rpy", {0.0, 0.0, 0.0});
    imu_timeout_sec = node.declare_parameter<double>("imu.timeout_sec", 0.25);
    accel_correction_gain = node.declare_parameter<double>("imu.accel_correction_gain", 0.02);
    accel_norm_tolerance = node.declare_parameter<double>("imu.accel_norm_tolerance", 3.0);
    imu_roll_offset = node.declare_parameter<double>("imu.roll_offset", 0.0);
    imu_pitch_offset = node.declare_parameter<double>("imu.pitch_offset", 0.0);
    const auto valid_vector3 = [](const std::vector<double> &values) {
      return values.size() == 3U && std::all_of(values.begin(), values.end(),
          [](const double value) { return std::isfinite(value); });
    };
    if ((orientation_source != "slam" && orientation_source != "d435i_imu") ||
        base_frame.empty() || gravity_frame.empty() || gravity_frame == base_frame ||
        rate_hz <= 0.0 || tf_timeout_sec < 0.0 || !valid_vector3(imu_to_mount_rpy) ||
        imu_timeout_sec <= 0.0 || accel_correction_gain < 0.0 ||
        accel_correction_gain > 1.0 || accel_norm_tolerance < 0.0) {
      throw std::invalid_argument("gravity_frame_broadcaster parameters are invalid");
    }
    tf2::Quaternion imu_rotation;
    imu_rotation.setRPY(imu_to_mount_rpy[0], imu_to_mount_rpy[1], imu_to_mount_rpy[2]);
    mount_from_imu.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    mount_from_imu.setRotation(imu_rotation);
    if (orientation_source == "d435i_imu") {
      if (imu_topic.empty() || imu_mount_frame.empty()) {
        throw std::invalid_argument("D435i IMU gravity mode requires imu_topic and imu_mount_frame");
      }
      imu_subscription = node.create_subscription<sensor_msgs::msg::Imu>(
          imu_topic, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::Imu::SharedPtr message) { onImu(message); });
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    timer = node.create_wall_timer(period, [this]() { publish(); });
    RCLCPP_INFO(node.get_logger(), "Gravity frame: %s -> %s from %s @ %.1f Hz",
                base_frame.c_str(), gravity_frame.c_str(), orientation_source.c_str(), rate_hz);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr &message) {
    if (!message || (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0)) {
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    tf2::Transform base_from_mount;
    try {
      const auto transform = tf_buffer.lookupTransform(
          base_frame, imu_mount_frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_sec));
      tf2::fromMsg(transform.transform, base_from_mount);
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for D435i IMU mount TF %s -> %s: %s",
                           base_frame.c_str(), imu_mount_frame.c_str(), error.what());
      return;
    }
    const auto base_from_imu = base_from_mount * mount_from_imu;
    const auto acceleration = base_from_imu.getBasis() * tf2::Vector3(
        message->linear_acceleration.x, message->linear_acceleration.y,
        message->linear_acceleration.z);
    const auto gyro = base_from_imu.getBasis() * tf2::Vector3(
        message->angular_velocity.x, message->angular_velocity.y,
        message->angular_velocity.z);
    if (!std::isfinite(acceleration.x()) || !std::isfinite(acceleration.y()) ||
        !std::isfinite(acceleration.z()) || !std::isfinite(gyro.x()) ||
        !std::isfinite(gyro.y()) || !std::isfinite(gyro.z())) {
      return;
    }
    const double acceleration_norm = acceleration.length();
    const bool trusted_acceleration =
        std::fabs(acceleration_norm - 9.80665) <= accel_norm_tolerance;
    const double accelerometer_roll = std::atan2(acceleration.y(), acceleration.z());
    const double accelerometer_pitch = std::atan2(
        -acceleration.x(), std::hypot(acceleration.y(), acceleration.z()));

    std::lock_guard<std::mutex> lock(imu_mutex);
    if (!have_imu || stamp <= imu_stamp) {
      if (!have_imu && trusted_acceleration) {
        imu_roll = accelerometer_roll;
        imu_pitch = accelerometer_pitch;
        have_imu = true;
      }
      imu_stamp = stamp;
      return;
    }
    const double dt = std::clamp((stamp - imu_stamp).seconds(), 0.0, 0.1);
    const double sin_roll = std::sin(imu_roll);
    const double cos_roll = std::cos(imu_roll);
    const double tan_pitch = std::tan(imu_pitch);
    imu_roll += (gyro.x() + sin_roll * tan_pitch * gyro.y() +
                 cos_roll * tan_pitch * gyro.z()) * dt;
    imu_pitch += (cos_roll * gyro.y() - sin_roll * gyro.z()) * dt;
    if (trusted_acceleration) {
      imu_roll = (1.0 - accel_correction_gain) * imu_roll +
                 accel_correction_gain * accelerometer_roll;
      imu_pitch = (1.0 - accel_correction_gain) * imu_pitch +
                  accel_correction_gain * accelerometer_pitch;
    }
    imu_stamp = stamp;
    have_imu = true;
  }

  bool gravityFromSlam(tf2::Quaternion &gravity_from_base) {
    try {
      const auto message = tf_buffer.lookupTransform(
          map_frame, base_frame, rclcpp::Time(0, 0, node.get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(tf_timeout_sec));
      tf2::Transform map_from_base;
      tf2::fromMsg(message.transform, map_from_base);
      auto rotation = map_from_base.getRotation();
      rotation.normalize();
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
      tf2::Quaternion map_from_gravity;
      map_from_gravity.setRPY(0.0, 0.0, yaw);
      gravity_from_base = map_from_gravity.inverse() * rotation;
      gravity_from_base.normalize();
      return true;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for SLAM TF %s -> %s: %s",
                           map_frame.c_str(), base_frame.c_str(), error.what());
      return false;
    }
  }

  bool gravityFromD435i(tf2::Quaternion &gravity_from_base) {
    std::lock_guard<std::mutex> lock(imu_mutex);
    if (!have_imu || (node.get_clock()->now() - imu_stamp).seconds() > imu_timeout_sec) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for fresh D435i IMU orientation on %s", imu_topic.c_str());
      return false;
    }
    gravity_from_base.setRPY(imu_roll - imu_roll_offset, imu_pitch - imu_pitch_offset, 0.0);
    gravity_from_base.normalize();
    return true;
  }

  void publish() {
    tf2::Quaternion gravity_from_base;
    const bool ready = orientation_source == "slam"
                           ? gravityFromSlam(gravity_from_base)
                           : gravityFromD435i(gravity_from_base);
    if (!ready) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node.get_clock()->now();
    transform.header.frame_id = base_frame;
    transform.child_frame_id = gravity_frame;
    transform.transform.rotation = tf2::toMsg(gravity_from_base.inverse());
    broadcaster.sendTransform(transform);
  }

  GravityFrameBroadcaster &node;
  std::string orientation_source;
  std::string map_frame;
  std::string base_frame;
  std::string gravity_frame;
  std::string imu_topic;
  std::string imu_mount_frame;
  double rate_hz{50.0};
  double tf_timeout_sec{0.02};
  double imu_timeout_sec{0.25};
  double accel_correction_gain{0.02};
  double accel_norm_tolerance{3.0};
  double imu_roll_offset{0.0};
  double imu_pitch_offset{0.0};
  tf2::Transform mount_from_imu;
  std::mutex imu_mutex;
  bool have_imu{false};
  double imu_roll{0.0};
  double imu_pitch{0.0};
  rclcpp::Time imu_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster broadcaster;
};

GravityFrameBroadcaster::GravityFrameBroadcaster(const rclcpp::NodeOptions &options)
    : Node("gravity_frame_broadcaster", options), impl_(std::make_unique<Impl>(*this)) {}

GravityFrameBroadcaster::~GravityFrameBroadcaster() = default;

}  // namespace autonomy_light
