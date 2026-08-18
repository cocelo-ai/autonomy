#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light {

// Publishes the local, yaw-preserving gravity frame from the latest SLAM pose.
class GravityFrameBroadcaster final : public rclcpp::Node {
public:
  explicit GravityFrameBroadcaster(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~GravityFrameBroadcaster() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy_light
