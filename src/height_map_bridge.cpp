#include "autonomy_light/height_map_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonomy_light/dds_height_map_publisher.hpp"
#include "autonomy_light/msg/height_map.hpp"

namespace autonomy_light {
namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

struct PointXYZ {
  float x;
  float y;
  float z;
};
static_assert(sizeof(PointXYZ) == 3U * sizeof(float));

int wrap(const int value, const int size) {
  const int result = value % size;
  return result < 0 ? result + size : result;
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
    transport = node.declare_parameter<std::string>("transport", "both");
    ros_topic = node.declare_parameter<std::string>(
        "ros2_topic", "/autonomy_light/height_map_data");
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
    pointcloud_topic = node.declare_parameter<std::string>(
        "pointcloud_topic", "/autonomy_light/height_map");
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
    const auto dds_type =
        node.declare_parameter<std::string>("dds.type", "core_dds::HeightMap");
    const int dds_history = node.declare_parameter<int>("dds.history_depth", 128);

    if (transport != "ros2" && transport != "cyclone_dds" &&
        transport != "both") {
      throw std::invalid_argument(
          "transport must be ros2, cyclone_dds, or both");
    }
    if (rate_hz <= 0.0 || fallback_resolution <= 0.0 ||
        fallback_x_length <= 0.0 || fallback_y_length <= 0.0) {
      throw std::invalid_argument(
          "height-map bridge rate and geometry must be positive");
    }
    if (output_resolution <= 0.0 || output_x_length <= 0.0 ||
        output_y_length <= 0.0) {
      throw std::invalid_argument("height-map output geometry must be positive");
    }
    if (clipping_min > clipping_max) {
      std::swap(clipping_min, clipping_max);
    }
    unknown_value = std::clamp(unknown_value, clipping_min, clipping_max);

    subscriber = node.create_subscription<grid_map_msgs::msg::GridMap>(
        input_topic, rclcpp::QoS(1).reliable(),
        [this](const grid_map_msgs::msg::GridMap::SharedPtr message) {
          onMap(message);
        });
    if (publishesRos()) {
      ros_publisher = node.create_publisher<autonomy_light::msg::HeightMap>(
          ros_topic, rclcpp::QoS(1).reliable());
    }
    if (!pointcloud_topic.empty()) {
      pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
          pointcloud_topic, rclcpp::SensorDataQoS());
    }
    if (publishesDds()) {
      dds_publisher = std::make_unique<DdsHeightMapPublisher>(
          static_cast<std::uint32_t>(std::max(0, dds_domain)), dds_topic,
          dds_type, static_cast<std::uint32_t>(std::max(1, dds_history)));
      if (!dds_publisher->ready()) {
        throw std::runtime_error("Cyclone DDS height-map writer failed: " +
                                 dds_publisher->error());
      }
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    timer = node.create_wall_timer(period, [this]() { publish(); });
    RCLCPP_INFO(node.get_logger(),
                "HeightMap bridge: input=%s output=%.3fm (%dx%d) transport=%s rate=%.1fHz",
                input_topic.c_str(), output_resolution,
                static_cast<int>(std::round(output_x_length / output_resolution)),
                static_cast<int>(std::round(output_y_length / output_resolution)),
                transport.c_str(), rate_hz);
  }

  bool publishesRos() const {
    return transport == "ros2" || transport == "both";
  }
  bool publishesDds() const {
    return transport == "cyclone_dds" || transport == "both";
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
    next->elevation = matrix.data;
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
    std::vector<float> data(cells(length_x, length_y, resolution),
                            static_cast<float>(unknown_value));
    std::vector<PointXYZ> points;

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
              const double x = -length_x / 2.0 + (column + 0.5) * resolution;
              const double y = -length_y / 2.0 + (row + 0.5) * resolution;
              const float elevation =
                  current->sample(base_x + cosine * x - sine * y,
                                  base_y + sine * x + cosine * y);
              if (std::isfinite(elevation)) {
                const float relative_height = elevation - floor;
                data[static_cast<std::size_t>(row) * width + column] =
                    static_cast<float>(
                        std::clamp(reference_height - static_cast<double>(relative_height),
                                   clipping_min, clipping_max));
                points.push_back(PointXYZ{static_cast<float>(x),
                                          static_cast<float>(y),
                                          relative_height});
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

    if (ros_publisher) {
      autonomy_light::msg::HeightMap message;
      message.header.stamp = node.now();
      message.header.frame_id = base_frame;
      message.resolution = static_cast<float>(resolution);
      message.x_length = static_cast<float>(length_x);
      message.y_length = static_cast<float>(length_y);
      message.data = data;
      ros_publisher->publish(std::move(message));
    }
    if (dds_publisher && !dds_publisher->publish(data)) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 2000, "%s",
                            dds_publisher->error().c_str());
    }
    if (pointcloud_publisher) {
      sensor_msgs::msg::PointCloud2 cloud;
      if (current) {
        cloud.header.stamp = current->header.stamp;
      } else {
        cloud.header.stamp = node.now();
      }
      cloud.header.frame_id = base_frame;
      cloud.height = 1;
      cloud.width = static_cast<std::uint32_t>(points.size());
      cloud.is_bigendian = false;
      cloud.is_dense = true;
      cloud.point_step = sizeof(PointXYZ);
      cloud.row_step = cloud.point_step * cloud.width;
      cloud.fields.resize(3);
      cloud.fields[0].name = "x";
      cloud.fields[0].offset = 0;
      cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[0].count = 1;
      cloud.fields[1].name = "y";
      cloud.fields[1].offset = 4;
      cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[1].count = 1;
      cloud.fields[2].name = "z";
      cloud.fields[2].offset = 8;
      cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[2].count = 1;
      cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
      if (!points.empty()) {
        std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
      }
      pointcloud_publisher->publish(std::move(cloud));
    }
  }

  HeightMapBridge &node;
  std::string input_topic;
  std::string input_layer;
  std::string base_frame;
  std::string transport;
  std::string ros_topic;
  double rate_hz{50.0};
  double fallback_resolution{0.1};
  double fallback_x_length{1.8};
  double fallback_y_length{0.8};
  double output_resolution{0.1};
  double output_x_length{1.8};
  double output_y_length{0.8};
  std::string pointcloud_topic;
  double floor_radius{0.6};
  double floor_percentile{0.20};
  double reference_height{0.48};
  double clipping_min{0.0};
  double clipping_max{0.75};
  double unknown_value{0.48};
  std::mutex snapshot_mutex;
  std::shared_ptr<Snapshot> snapshot;
  std::unique_ptr<DdsHeightMapPublisher> dds_publisher;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber;
  rclcpp::Publisher<autonomy_light::msg::HeightMap>::SharedPtr ros_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;
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
