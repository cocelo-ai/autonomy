#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light::algorithm::elevation {

/**
 * Edge-preserving elevation extraction from Super-LIO's robot-local global-map
 * crop. One local LiDAR crop produces one robot-relative HeightMap.
 */
class ElevationMap final : public rclcpp::Node {
public:
  explicit ElevationMap(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ElevationMap() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy_light::algorithm::elevation
