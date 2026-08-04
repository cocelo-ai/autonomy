#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light {

class HeightMapBridge final : public rclcpp::Node {
public:
  explicit HeightMapBridge(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~HeightMapBridge() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace autonomy_light
