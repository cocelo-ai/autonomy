#include "autonomy_light/height_map_bridge.hpp"
#include "autonomy_light/dds_height_map_publisher.hpp"
#include "autonomy_light/msg/height_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autonomy_light {

struct HeightMapBridge::Impl {
  explicit Impl(HeightMapBridge &node) : node(node) {
    input_topic = node.declare_parameter<std::string>(
        "input_topic", "/autonomy_light/elevation_map");
    rate_hz = node.declare_parameter<double>("publish_rate_hz", 50.0);
    const int dds_domain = node.declare_parameter<int>("dds.domain_id", 1);
    const auto dds_topic =
        node.declare_parameter<std::string>("dds.topic", "height_map");
    const int dds_history = node.declare_parameter<int>("dds.history_depth", 128);
    const auto dds_network_interface = node.declare_parameter<std::string>(
        "dds.network_interface", "");
    const auto dds_peer_address = node.declare_parameter<std::string>(
        "dds.peer_address", "");
    visualization_topic = node.declare_parameter<std::string>(
        "visualization.topic", "");
    if (rate_hz <= 0.0) {
      throw std::invalid_argument("height-map bridge publish rate must be positive");
    }
    subscriber = node.create_subscription<autonomy_light::msg::HeightMap>(
        input_topic, rclcpp::QoS(1).reliable(),
        [this](const autonomy_light::msg::HeightMap::SharedPtr message) {
          onHeightMap(message);
        });
    dds_publisher = std::make_unique<DdsHeightMapPublisher>(
        static_cast<std::uint32_t>(std::max(0, dds_domain)), dds_topic,
        static_cast<std::uint32_t>(std::max(1, dds_history)),
        dds_network_interface, dds_peer_address);
    if (!dds_publisher->ready()) {
      throw std::runtime_error("DDS height-map writer failed: " +
                               dds_publisher->error());
    }
    if (!visualization_topic.empty()) {
      visualization_publisher =
          node.create_publisher<autonomy_light::msg::HeightMap>(
              visualization_topic, rclcpp::QoS(1).reliable());
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    timer = node.create_wall_timer(period, [this]() { publish(); });
    RCLCPP_INFO(node.get_logger(),
                "HeightMap DDS conversion: input=%s topic=%s interface=%s peer=%s rate=%.1fHz%s",
                input_topic.c_str(), dds_topic.c_str(),
                dds_network_interface.empty() ? "auto" : dds_network_interface.c_str(),
                dds_peer_address.empty() ? "multicast-only" : dds_peer_address.c_str(),
                rate_hz,
                visualization_topic.empty() ? "" : " (private visualization mirror enabled)");
  }

  void onHeightMap(const autonomy_light::msg::HeightMap::SharedPtr &message) {
    if (!message) {
      return;
    }
    const double resolution = message->resolution;
    const double length_x = message->x_length;
    const double length_y = message->y_length;
    const auto width = resolution > 0.0 ? std::llround(length_x / resolution) : 0;
    const auto height = resolution > 0.0 ? std::llround(length_y / resolution) : 0;
    if (!std::isfinite(resolution) || !std::isfinite(length_x) || !std::isfinite(length_y) ||
        resolution <= 0.0 || length_x <= 0.0 || length_y <= 0.0 || width <= 0 || height <= 0 ||
        !std::isfinite(static_cast<double>(width) * resolution) ||
        std::fabs(static_cast<double>(width) * resolution - length_x) > 1.0e-5 ||
        std::fabs(static_cast<double>(height) * resolution - length_y) > 1.0e-5 ||
        message->data.size() != static_cast<std::size_t>(width) *
                                static_cast<std::size_t>(height)) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Height-map geometry is invalid");
      return;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex);
    snapshot = message;
  }

  void publish() {
    autonomy_light::msg::HeightMap::SharedPtr current;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex);
      current = snapshot;
    }
    if (!current) {
      return;
    }
    const auto &data = current->data;

    if (visualization_publisher) {
      autonomy_light::msg::HeightMap visualization;
      visualization.header = current->header;
      visualization.data = data;
      visualization.resolution = current->resolution;
      visualization.x_length = current->x_length;
      visualization.y_length = current->y_length;
      visualization_publisher->publish(std::move(visualization));
    }

    if (!dds_publisher->publish(data)) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 2000, "%s",
                            dds_publisher->error().c_str());
    }
  }

  HeightMapBridge &node;
  std::string input_topic;
  double rate_hz{50.0};
  std::string visualization_topic;
  std::mutex snapshot_mutex;
  autonomy_light::msg::HeightMap::SharedPtr snapshot;
  std::unique_ptr<DdsHeightMapPublisher> dds_publisher;
  rclcpp::Publisher<autonomy_light::msg::HeightMap>::SharedPtr
      visualization_publisher;
  rclcpp::Subscription<autonomy_light::msg::HeightMap>::SharedPtr subscriber;
  rclcpp::TimerBase::SharedPtr timer;
};

HeightMapBridge::HeightMapBridge(const rclcpp::NodeOptions &options)
    : Node("height_map_bridge", options) {
  impl_ = std::make_unique<Impl>(*this);
}

HeightMapBridge::~HeightMapBridge() = default;

} // namespace autonomy_light
