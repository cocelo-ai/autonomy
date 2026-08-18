#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light {

// Stateless, edge-preserving elevation extraction ported from the project's
// height_cli_lib algorithm.  One merged observation produces one GridMap.
class PreciseElevationMapping final : public rclcpp::Node {
public:
  explicit PreciseElevationMapping(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PreciseElevationMapping() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy_light
