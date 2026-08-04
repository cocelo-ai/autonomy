#include <rclcpp/rclcpp.hpp>

#include "autonomy_light/height_map_bridge.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::HeightMapBridge>());
  rclcpp::shutdown();
  return 0;
}
