#include <rclcpp/rclcpp.hpp>

#include "autonomy_light/observation_merge.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::ObservationMerge>());
  rclcpp::shutdown();
  return 0;
}
