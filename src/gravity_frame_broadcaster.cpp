#include "autonomy_light/gravity_frame_broadcaster.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy_light {

struct GravityFrameBroadcaster::Impl {
  explicit Impl(GravityFrameBroadcaster &node)
      : node(node), tf_buffer(node.get_clock()), tf_listener(tf_buffer), broadcaster(node) {
    map_frame = node.declare_parameter<std::string>("map_frame", "map");
    base_frame = node.declare_parameter<std::string>("base_frame", "base_link");
    gravity_frame = node.declare_parameter<std::string>("gravity_frame", "base_link_gravity");
    rate_hz = node.declare_parameter<double>("publish_rate_hz", 50.0);
    tf_timeout_sec = node.declare_parameter<double>("tf_timeout_sec", 0.02);
    if (map_frame.empty() || base_frame.empty() || gravity_frame.empty() ||
        gravity_frame == base_frame || rate_hz <= 0.0 || tf_timeout_sec < 0.0) {
      throw std::invalid_argument("gravity_frame_broadcaster parameters are invalid");
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    timer = node.create_wall_timer(period, [this]() { publish(); });
    RCLCPP_INFO(node.get_logger(), "Gravity frame: %s -> %s from %s @ %.1f Hz",
                base_frame.c_str(), gravity_frame.c_str(), map_frame.c_str(), rate_hz);
  }

  void publish() {
    try {
      const auto map_from_base_message = tf_buffer.lookupTransform(
          map_frame, base_frame,
          rclcpp::Time(0, 0, node.get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(tf_timeout_sec));
      tf2::Transform map_from_base;
      tf2::fromMsg(map_from_base_message.transform, map_from_base);
      tf2::Quaternion rotation = map_from_base.getRotation();
      rotation.normalize();
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
      tf2::Quaternion map_from_gravity;
      map_from_gravity.setRPY(0.0, 0.0, yaw);
      map_from_gravity.normalize();
      const tf2::Quaternion gravity_from_base = map_from_gravity.inverse() * rotation;
      // A TransformStamped with parent=base and child=gravity stores
      // base_from_gravity (child coordinates expressed in its parent), not
      // gravity_from_base. Publishing the latter reverses the edge and makes
      // map -> base_link_gravity contain roll/pitch twice.
      tf2::Quaternion base_from_gravity = gravity_from_base.inverse();
      base_from_gravity.normalize();

      geometry_msgs::msg::TransformStamped transform;
      // This is a new 50 Hz gravity-aligned attachment evaluated from the
      // latest SLAM pose.  Do not reuse a static-TF stamp (zero), otherwise
      // timestamped camera/elevation lookups cannot interpolate this edge.
      transform.header.stamp = node.get_clock()->now();
      transform.header.frame_id = base_frame;
      transform.child_frame_id = gravity_frame;
      transform.transform.rotation = tf2::toMsg(base_from_gravity);
      broadcaster.sendTransform(transform);
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for SLAM TF %s -> %s: %s",
                           map_frame.c_str(), base_frame.c_str(), error.what());
    }
  }

  GravityFrameBroadcaster &node;
  std::string map_frame;
  std::string base_frame;
  std::string gravity_frame;
  double rate_hz{50.0};
  double tf_timeout_sec{0.02};
  rclcpp::TimerBase::SharedPtr timer;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster broadcaster;
};

GravityFrameBroadcaster::GravityFrameBroadcaster(const rclcpp::NodeOptions &options)
    : Node("gravity_frame_broadcaster", options), impl_(std::make_unique<Impl>(*this)) {}

GravityFrameBroadcaster::~GravityFrameBroadcaster() = default;

}  // namespace autonomy_light
