#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light {

// Converts one calibrated camera stream into the SLAM map frame at the source
// timestamp. Static calibration is published only for configured sensor frames.
class CameraFrameMapper final : public rclcpp::Node {
public:
  explicit CameraFrameMapper(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~CameraFrameMapper() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy_light
