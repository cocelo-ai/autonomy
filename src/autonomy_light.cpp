#include "autonomy_light/autonomy_light_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<autonomy_light::AutonomyLightNode>();
    std::weak_ptr<autonomy_light::AutonomyLightNode> weak_node = node;
    rclcpp::on_shutdown([weak_node]() {
      if (auto node = weak_node.lock()) {
        node->requestFastShutdown();
      }
    });
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception &ex) {
    std::fprintf(stderr, "autonomy_light failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
