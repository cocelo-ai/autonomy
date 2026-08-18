#include <rclcpp/rclcpp.hpp>

#include "autonomy_light/precise_elevation_mapping.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  const auto node = std::make_shared<autonomy_light::PreciseElevationMapping>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
