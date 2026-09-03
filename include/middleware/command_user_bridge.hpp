#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light::middleware {

/** Creates the Nav2 Twist to external FSM-command adapter. */
std::shared_ptr<rclcpp::Node> createCommandUserBridge();

}  // namespace autonomy_light::middleware
