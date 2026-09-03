#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autonomy_light::algorithm::planner {

/** Creates the Nav2 occupancy-map adapter from Super-LIO's registered cloud. */
std::shared_ptr<rclcpp::Node> createLiveOccupancyMapper();

}  // namespace autonomy_light::algorithm::planner
