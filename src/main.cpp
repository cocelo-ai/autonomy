#include <memory>
#include <vector>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "algorithm/elevation/elevation_map.hpp"
#include "algorithm/planner/live_occupancy_mapper.hpp"
#include "common/ros_runtime.hpp"
#include "middleware/command_user_bridge.hpp"
#include "middleware/external_dds_bridge.hpp"
#include "sensor/gravity_frame_broadcaster.hpp"

int main(int argc, char **argv) {
  return autonomy_light::common::runNodes(
      "autonomy_light", argc, argv,
      [] {
        using rclcpp::Node;
        return std::vector<Node::SharedPtr>{
            std::make_shared<autonomy_light::sensor::GravityFrameBroadcaster>(),
            std::make_shared<autonomy_light::algorithm::elevation::ElevationMap>(),
            autonomy_light::algorithm::planner::createLiveOccupancyMapper(),
            autonomy_light::middleware::createCommandUserBridge(),
            std::make_shared<autonomy_light::middleware::ExternalDdsBridge>(),
        };
      },
      [] { return rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 5U); });
}
