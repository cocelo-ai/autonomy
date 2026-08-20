#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light {

// Merges camera observations and deskewed raw Livox points into one
// robot-centric, gravity-levelled PointCloud2.  Each camera epoch is expressed
// in target_frame; raw LiDAR points are transformed at their offset_time.
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
