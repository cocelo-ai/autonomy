#pragma once

#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light::common {

/**
 * Runs a normal ROS 2 process with consistent exception reporting and context
 * shutdown. The executor factory keeps each process explicit about its
 * concurrency policy while avoiding divergent `main` implementations.
 */
template <typename NodeFactory, typename ExecutorFactory>
int runNodes(const char *const executable_name, const int argc, char **argv,
             NodeFactory &&make_nodes, ExecutorFactory &&make_executor) {
  bool initialized = false;
  try {
    rclcpp::init(argc, argv);
    initialized = true;

    auto nodes = std::forward<NodeFactory>(make_nodes)();
    if (nodes.empty()) {
      throw std::invalid_argument("at least one ROS node is required");
    }
    auto executor = std::forward<ExecutorFactory>(make_executor)();
    for (const auto &node : nodes) {
      if (!node) {
        throw std::invalid_argument("a ROS node factory returned null");
      }
      executor.add_node(node);
    }
    executor.spin();
    for (const auto &node : nodes) {
      executor.remove_node(node);
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << executable_name << ": " << error.what() << '\n';
  } catch (...) {
    std::cerr << executable_name << ": unhandled non-standard exception\n";
  }

  if (initialized) {
    rclcpp::shutdown();
  }
  return EXIT_FAILURE;
}

template <typename NodeT, typename ExecutorFactory>
int runNode(const char *const executable_name, const int argc, char **argv,
            ExecutorFactory &&make_executor) {
  return runNodes(executable_name, argc, argv,
                 [] { return std::vector<rclcpp::Node::SharedPtr>{std::make_shared<NodeT>()}; },
                 std::forward<ExecutorFactory>(make_executor));
}

}  // namespace autonomy_light::common
