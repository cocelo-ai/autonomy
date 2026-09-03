#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light::middleware {

/** Direct Cyclone DDS egress for the fixed control-SBC contracts. */
class ExternalDdsBridge final : public rclcpp::Node {
 public:
  explicit ExternalDdsBridge(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ExternalDdsBridge() override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy_light::middleware
