#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light {

// Merges the latest usable camera observations into one robot-centric,
// gravity-levelled PointCloud2.  It deliberately performs no temporal map
// accumulation: each published cloud is one observation epoch.  LiDAR stays
// on the independent Super-LIO/Nav2 path.
class ObservationMerge final : public rclcpp::Node {
public:
  explicit ObservationMerge(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ObservationMerge() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy_light
