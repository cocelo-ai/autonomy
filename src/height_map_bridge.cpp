#include "autonomy_light/height_map_bridge.hpp"
#include "autonomy_light/dds_height_map_publisher.hpp"
#include "autonomy_light/fov_mask.hpp"
#include "autonomy_light/msg/height_map.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy_light {
namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

int wrap(const int value, const int size) {
  const int result = value % size;
  return result < 0 ? result + size : result;
}

bool hasToken(const std::string &value, const std::string &token) {
  std::string lower;
  lower.reserve(value.size());
  for (const auto character : value) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }
  return lower.find(token) != std::string::npos;
}

double yawOf(const geometry_msgs::msg::Quaternion &orientation) {
  tf2::Quaternion quaternion;
  tf2::fromMsg(orientation, quaternion);
  if (quaternion.length2() < 1.0e-12) {
    return 0.0;
  }
  quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

std::size_t cells(const double length_x, const double length_y,
                  const double resolution) {
  const auto width = static_cast<std::size_t>(
      std::max(1.0, std::round(length_x / resolution)));
  const auto height = static_cast<std::size_t>(
      std::max(1.0, std::round(length_y / resolution)));
  return width * height;
}

} // namespace

struct HeightMapBridge::Impl {
  struct Snapshot {
    std_msgs::msg::Header header;
    double resolution{0.1};
    double length_x{1.8};
    double length_y{0.8};
    double center_x{0.0};
    double center_y{0.0};
    double yaw{0.0};
    int size_x{0};
    int size_y{0};
    int outer_start{0};
    int inner_start{0};
    std::vector<float> elevation;

    [[nodiscard]] float sample(const double x, const double y) const {
      if (size_x <= 0 || size_y <= 0 || elevation.empty()) {
        return kNan;
      }
      const double dx = x - center_x;
      const double dy = y - center_y;
      const double cosine = std::cos(yaw);
      const double sine = std::sin(yaw);
      const double grid_x = cosine * dx + sine * dy;
      const double grid_y = -sine * dx + cosine * dy;
      const int x_index =
          static_cast<int>(std::floor((grid_x + length_x / 2.0) / resolution));
      const int y_index =
          static_cast<int>(std::floor((grid_y + length_y / 2.0) / resolution));
      if (x_index < 0 || x_index >= size_x || y_index < 0 ||
          y_index >= size_y) {
        return kNan;
      }
      const int raw_x = wrap(x_index + outer_start, size_x);
      const int raw_y = wrap(y_index + inner_start, size_y);
      return elevation[static_cast<std::size_t>(raw_x) +
                       static_cast<std::size_t>(raw_y) * size_x];
    }
  };

  explicit Impl(HeightMapBridge &node)
      : node(node), tf_buffer(node.get_clock()), tf_listener(tf_buffer) {
    input_topic = node.declare_parameter<std::string>(
        "input_topic", "/autonomy_light/elevation_map");
    input_layer =
        node.declare_parameter<std::string>("input_layer", "elevation");
    base_frame = node.declare_parameter<std::string>("base_frame", "base_link");
    rate_hz = node.declare_parameter<double>("publish_rate_hz", 50.0);
    fallback_resolution =
        node.declare_parameter<double>("fallback.resolution", 0.1);
    fallback_x_length =
        node.declare_parameter<double>("fallback.x_length", 1.8);
    fallback_y_length =
        node.declare_parameter<double>("fallback.y_length", 0.8);
    output_resolution =
        node.declare_parameter<double>("output.resolution", 0.1);
    output_x_length =
        node.declare_parameter<double>("output.x_length", 1.8);
    output_y_length =
        node.declare_parameter<double>("output.y_length", 0.8);
    fov_mask_file = node.declare_parameter<std::string>("fov.mask_file", "");
    fov_outside_value = node.declare_parameter<double>("fov.outside_value", 0.0);
    floor_radius = node.declare_parameter<double>("floor.radius_m", 0.6);
    floor_percentile = std::clamp(
        node.declare_parameter<double>("floor.percentile", 0.20), 0.0, 1.0);
    reference_height =
        node.declare_parameter<double>("distance.reference_height", 0.48);
    clipping_min = node.declare_parameter<double>("distance.min", 0.0);
    clipping_max = node.declare_parameter<double>("distance.max", 0.75);
    unknown_value = node.declare_parameter<double>("distance.unknown", 0.48);
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
    if (rate_hz <= 0.0 || fallback_resolution <= 0.0 ||
        fallback_x_length <= 0.0 || fallback_y_length <= 0.0) {
      throw std::invalid_argument(
          "height-map bridge rate and geometry must be positive");
    }
    if (output_resolution <= 0.0 || output_x_length <= 0.0 ||
        output_y_length <= 0.0) {
      throw std::invalid_argument("height-map output geometry must be positive");
    }
    if (!std::isfinite(fov_outside_value)) {
      throw std::invalid_argument("height-map FOV outside value must be finite");
    }
    if (clipping_min > clipping_max) {
      std::swap(clipping_min, clipping_max);
    }
    unknown_value = std::clamp(unknown_value, clipping_min, clipping_max);
    const auto output_width = static_cast<std::uint32_t>(
        std::max(1, static_cast<int>(std::round(output_x_length / output_resolution))));
    const auto output_height = static_cast<std::uint32_t>(
        std::max(1, static_cast<int>(std::round(output_y_length / output_resolution))));
    fov_mask = loadFovMask(fov_mask_file, output_width, output_height, output_resolution,
                           output_x_length, output_y_length);

    subscriber = node.create_subscription<grid_map_msgs::msg::GridMap>(
        input_topic, rclcpp::QoS(1).reliable(),
        [this](const grid_map_msgs::msg::GridMap::SharedPtr message) {
          onMap(message);
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
                "HeightMap DDS: input=%s topic=%s interface=%s peer=%s output=%.3fm (%dx%d) rate=%.1fHz, FOV mask=%s%s",
                input_topic.c_str(), dds_topic.c_str(),
                dds_network_interface.empty() ? "auto" : dds_network_interface.c_str(),
                dds_peer_address.empty() ? "multicast-only" : dds_peer_address.c_str(),
                output_resolution,
                static_cast<int>(std::round(output_x_length / output_resolution)),
                static_cast<int>(std::round(output_y_length / output_resolution)),
                rate_hz,
                fov_mask.enabled ? fov_mask_file.c_str() : "disabled",
                visualization_topic.empty() ? "" : " (private visualization mirror enabled)");
  }

  void onMap(const grid_map_msgs::msg::GridMap::SharedPtr &message) {
    if (!message) {
      return;
    }
    const auto layer =
        std::find(message->layers.begin(), message->layers.end(), input_layer);
    if (layer == message->layers.end()) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "GridMap has no '%s' layer", input_layer.c_str());
      return;
    }
    const auto layer_index =
        static_cast<std::size_t>(std::distance(message->layers.begin(), layer));
    const auto &matrix = message->data[layer_index];
    if (matrix.layout.dim.size() != 2U || matrix.layout.dim[0].size == 0U ||
        matrix.layout.dim[1].size == 0U ||
        matrix.data.size() !=
            static_cast<std::size_t>(matrix.layout.dim[0].size) *
                matrix.layout.dim[1].size ||
        message->info.resolution <= 0.0 || message->info.length_x <= 0.0 ||
        message->info.length_y <= 0.0) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "GridMap elevation geometry is invalid");
      return;
    }
    auto next = std::make_shared<Snapshot>();
    next->header = message->header;
    next->resolution = message->info.resolution;
    next->length_x = message->info.length_x;
    next->length_y = message->info.length_y;
    next->center_x = message->info.pose.position.x;
    next->center_y = message->info.pose.position.y;
    next->yaw = yawOf(message->info.pose.orientation);
    next->size_x = static_cast<int>(matrix.layout.dim[0].size);
    next->size_y = static_cast<int>(matrix.layout.dim[1].size);
    next->outer_start = static_cast<int>(message->outer_start_index);
    next->inner_start = static_cast<int>(message->inner_start_index);
    // GridMap serializes Eigen matrices as column-major data.  The old bridge
    // treated that payload as row-major, silently transposing/scrambling a
    // fine grid.  Normalize it once here; Snapshot::sample remains x-major.
    const bool is_column_major =
        hasToken(matrix.layout.dim[0].label, "column") &&
        hasToken(matrix.layout.dim[1].label, "row");
    next->elevation.resize(matrix.data.size());
    for (int y = 0; y < next->size_y; ++y) {
      for (int x = 0; x < next->size_x; ++x) {
        const auto destination = static_cast<std::size_t>(x) +
                                 static_cast<std::size_t>(y) * next->size_x;
        const auto source = is_column_major
                                ? static_cast<std::size_t>(x) * next->size_y + y
                                : destination;
        next->elevation[destination] = matrix.data[source];
      }
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex);
    snapshot = std::move(next);
  }

  void publish() {
    std::shared_ptr<Snapshot> current;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex);
      current = snapshot;
    }
    const double resolution = output_resolution;
    const double length_x = output_x_length;
    const double length_y = output_y_length;
    const int width =
        std::max(1, static_cast<int>(std::round(length_x / resolution)));
    const int height =
        std::max(1, static_cast<int>(std::round(length_y / resolution)));
    std::vector<float> data(
        cells(length_x, length_y, resolution),
        static_cast<float>(unknown_value));

    if (current) {
      try {
        const auto transform =
            tf_buffer.lookupTransform(current->header.frame_id, base_frame,
                                      rclcpp::Time(current->header.stamp),
                                      rclcpp::Duration::from_seconds(0.002));
        const double base_yaw = yawOf(transform.transform.rotation);
        const double base_x = transform.transform.translation.x;
        const double base_y = transform.transform.translation.y;
        const double cosine = std::cos(base_yaw);
        const double sine = std::sin(base_yaw);
        std::vector<float> floor_samples;
        for (int row = 0; row < height; ++row) {
          for (int column = 0; column < width; ++column) {
            const auto sample_index = static_cast<std::size_t>(row) * width + column;
            if (!fov_mask.inside(sample_index)) {
              continue;
            }
            const double x = -length_x / 2.0 + (column + 0.5) * resolution;
            const double y = -length_y / 2.0 + (row + 0.5) * resolution;
            const float elevation = current->sample(
                base_x + cosine * x - sine * y, base_y + sine * x + cosine * y);
            if (std::isfinite(elevation) && std::hypot(x, y) <= floor_radius) {
              floor_samples.push_back(elevation);
            }
          }
        }
        if (!floor_samples.empty()) {
          const auto index = static_cast<std::size_t>(
              std::round(floor_percentile *
                         static_cast<double>(floor_samples.size() - 1U)));
          const auto nth = floor_samples.begin() +
                           static_cast<std::vector<float>::difference_type>(index);
          std::nth_element(floor_samples.begin(), nth, floor_samples.end());
          const float floor = floor_samples[index];
          for (int row = 0; row < height; ++row) {
            for (int column = 0; column < width; ++column) {
              const auto sample_index = static_cast<std::size_t>(row) * width + column;
              if (!fov_mask.inside(sample_index)) {
                continue;
              }
              const double x = -length_x / 2.0 + (column + 0.5) * resolution;
              const double y = -length_y / 2.0 + (row + 0.5) * resolution;
              const float elevation =
                  current->sample(base_x + cosine * x - sine * y,
                                  base_y + sine * x + cosine * y);
              if (std::isfinite(elevation)) {
                const float relative_height = elevation - floor;
                data[sample_index] =
                    static_cast<float>(
                        std::clamp(reference_height - static_cast<double>(relative_height),
                                   clipping_min, clipping_max));
              }
            }
          }
        }
      } catch (const tf2::TransformException &) {
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                             "Waiting for %s -> %s TF for legacy height map",
                             current->header.frame_id.c_str(),
                             base_frame.c_str());
      }
    }

    // Applied after all sampling and fallback logic, so direct DDS and the
    // optional visualization mirror always receive identical zeroed cells.
    if (fov_mask.enabled) {
      for (std::size_t sample_index = 0; sample_index < data.size(); ++sample_index) {
        if (!fov_mask.inside(sample_index)) {
          data[sample_index] = static_cast<float>(fov_outside_value);
        }
      }
    }

    if (visualization_publisher) {
      autonomy_light::msg::HeightMap visualization;
      visualization.header.stamp = node.now();
      visualization.header.frame_id = base_frame;
      visualization.data = data;
      visualization.resolution = static_cast<float>(resolution);
      visualization.x_length = static_cast<float>(length_x);
      visualization.y_length = static_cast<float>(length_y);
      visualization_publisher->publish(std::move(visualization));
    }

    if (!dds_publisher->publish(data)) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 2000, "%s",
                            dds_publisher->error().c_str());
    }
  }

  HeightMapBridge &node;
  std::string input_topic;
  std::string input_layer;
  std::string base_frame;
  double rate_hz{50.0};
  double fallback_resolution{0.1};
  double fallback_x_length{1.8};
  double fallback_y_length{0.8};
  double output_resolution{0.1};
  double output_x_length{1.8};
  double output_y_length{0.8};
  double floor_radius{0.6};
  double floor_percentile{0.20};
  double reference_height{0.48};
  double clipping_min{0.0};
  double clipping_max{0.75};
  double unknown_value{0.48};
  std::string fov_mask_file;
  double fov_outside_value{0.0};
  FovMask fov_mask;
  std::string visualization_topic;
  std::mutex snapshot_mutex;
  std::shared_ptr<Snapshot> snapshot;
  std::unique_ptr<DdsHeightMapPublisher> dds_publisher;
  rclcpp::Publisher<autonomy_light::msg::HeightMap>::SharedPtr
      visualization_publisher;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber;
  rclcpp::TimerBase::SharedPtr timer;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
};

HeightMapBridge::HeightMapBridge(const rclcpp::NodeOptions &options)
    : Node("height_map_bridge", options) {
  impl_ = std::make_unique<Impl>(*this);
}

HeightMapBridge::~HeightMapBridge() = default;

} // namespace autonomy_light
