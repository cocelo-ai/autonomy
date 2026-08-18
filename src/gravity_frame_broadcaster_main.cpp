#include <rclcpp/rclcpp.hpp>

#include "autonomy_light/gravity_frame_broadcaster.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::GravityFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
