#include "autonomy_light/rolling_cloud_merge_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::RollingCloudMergeNode>());
  rclcpp::shutdown();
  return 0;
}
