#include <rclcpp/rclcpp.hpp>

#include "autonomy_light/camera_frame_mapper.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::CameraFrameMapper>());
  rclcpp::shutdown();
  return 0;
}
