#include "autonomy_light/precise_elevation_mapping.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy_light {
namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

struct FieldOffsets {
  std::uint32_t x{0};
  std::uint32_t y{0};
  std::uint32_t z{0};
  std::uint32_t variance{0};
  bool has_variance{false};
};

struct PointXYZIntensity {
  float x;
  float y;
  float z;
  float intensity;
};
static_assert(sizeof(PointXYZIntensity) == 4U * sizeof(float));

const sensor_msgs::msg::PointField *findField(
    const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) {
  const auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                  [&name](const auto &field) {
                                    return field.name == name;
                                  });
  return found == cloud.fields.end() ? nullptr : &*found;
}

bool fields(const sensor_msgs::msg::PointCloud2 &cloud, FieldOffsets &out) {
  const auto *x = findField(cloud, "x");
  const auto *y = findField(cloud, "y");
  const auto *z = findField(cloud, "z");
  if (!x || !y || !z || x->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      y->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      z->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      x->offset + sizeof(float) > cloud.point_step ||
      y->offset + sizeof(float) > cloud.point_step ||
      z->offset + sizeof(float) > cloud.point_step) {
    return false;
  }
  out.x = x->offset;
  out.y = y->offset;
  out.z = z->offset;
  const auto *variance = findField(cloud, "variance");
  out.has_variance = variance &&
                     variance->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                     variance->offset + sizeof(float) <= cloud.point_step;
  if (out.has_variance) {
    out.variance = variance->offset;
  }
  return true;
}

float readFloat(const std::uint8_t *point, const std::uint32_t offset) {
  float value = kNan;
  std::memcpy(&value, point + offset, sizeof(value));
  return value;
}

struct GridGeometry {
  double resolution{0.05};
  double x_min{-1.50};
  double x_max{1.50};
  double y_min{-1.00};
  double y_max{1.00};
  double z_min{-1.20};
  double z_max{0.80};

  [[nodiscard]] std::uint32_t width() const {
    return static_cast<std::uint32_t>(std::ceil((x_max - x_min) / resolution));
  }
  [[nodiscard]] std::uint32_t height() const {
    return static_cast<std::uint32_t>(std::ceil((y_max - y_min) / resolution));
  }
};

struct FrameCell {
  std::uint32_t hits{0};
  std::uint32_t lower_support_count{0};
  float lower_height{std::numeric_limits<float>::infinity()};
  float lower_weighted_sum{0.0F};
  float lower_weight_sum{0.0F};
  float weighted_sum{0.0F};
  float weight_sum{0.0F};
  float min_height{std::numeric_limits<float>::infinity()};
  float max_height{-std::numeric_limits<float>::infinity()};
};

bool finite(const float value) { return std::isfinite(value); }

std::size_t indexOf(const std::uint32_t row, const std::uint32_t column,
                    const std::uint32_t width) {
  return static_cast<std::size_t>(row) * width + column;
}

void removeIsolatedCellsEdgeAware(std::vector<float> &height,
                                  std::vector<float> &variance,
                                  const std::uint32_t rows,
                                  const std::uint32_t columns,
                                  const int radius,
                                  const int min_support_neighbors,
                                  const float support_height_difference,
                                  const float outlier_height_difference,
                                  const float reset_variance) {
  if (radius <= 0 || min_support_neighbors <= 0 ||
      support_height_difference <= 0.0F || outlier_height_difference <= 0.0F) {
    return;
  }
  auto filtered = height;
  auto filtered_variance = variance;
  std::vector<float> neighbours;
  neighbours.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
  for (std::uint32_t row = 0; row < rows; ++row) {
    for (std::uint32_t column = 0; column < columns; ++column) {
      const auto index = indexOf(row, column, columns);
      const float center = height[index];
      if (!finite(center)) {
        continue;
      }
      neighbours.clear();
      int support = 0;
      for (int delta_row = -radius; delta_row <= radius; ++delta_row) {
        const int next_row = static_cast<int>(row) + delta_row;
        if (next_row < 0 || next_row >= static_cast<int>(rows)) {
          continue;
        }
        for (int delta_column = -radius; delta_column <= radius; ++delta_column) {
          const int next_column = static_cast<int>(column) + delta_column;
          if (next_column < 0 || next_column >= static_cast<int>(columns) ||
              (delta_row == 0 && delta_column == 0)) {
            continue;
          }
          const float neighbour = height[indexOf(static_cast<std::uint32_t>(next_row),
                                                   static_cast<std::uint32_t>(next_column), columns)];
          if (!finite(neighbour)) {
            continue;
          }
          neighbours.push_back(neighbour);
          if (std::fabs(neighbour - center) <= support_height_difference) {
            ++support;
          }
        }
      }
      // A true thin edge may have little support.  Only delete it when a
      // populated local neighbourhood votes for a different surface.
      if (static_cast<int>(neighbours.size()) < min_support_neighbors ||
          support >= min_support_neighbors) {
        continue;
      }
      const auto middle = neighbours.begin() + static_cast<std::ptrdiff_t>(neighbours.size() / 2U);
      std::nth_element(neighbours.begin(), middle, neighbours.end());
      if (std::fabs(center - *middle) > outlier_height_difference) {
        filtered[index] = kNan;
        filtered_variance[index] = reset_variance;
      }
    }
  }
  height.swap(filtered);
  variance.swap(filtered_variance);
}

void fillSmallHolesEdgeAware(std::vector<float> &height,
                             std::vector<float> &variance,
                             std::vector<std::uint8_t> &filter_filled,
                             const std::uint32_t rows,
                             const std::uint32_t columns,
                             const int radius,
                             const int min_neighbours,
                             const float max_height_difference,
                             const float maximum_variance) {
  if (radius <= 0 || min_neighbours <= 0 || max_height_difference <= 0.0F) {
    return;
  }
  auto filled = height;
  auto filled_variance = variance;
  auto filled_mask = filter_filled;
  for (std::uint32_t row = 0; row < rows; ++row) {
    for (std::uint32_t column = 0; column < columns; ++column) {
      const auto index = indexOf(row, column, columns);
      if (finite(height[index])) {
        continue;
      }
      float weighted_sum = 0.0F;
      float weight_sum = 0.0F;
      float minimum = std::numeric_limits<float>::infinity();
      float maximum = -std::numeric_limits<float>::infinity();
      float variance_sum = 0.0F;
      int count = 0;
      for (int delta_row = -radius; delta_row <= radius; ++delta_row) {
        const int next_row = static_cast<int>(row) + delta_row;
        if (next_row < 0 || next_row >= static_cast<int>(rows)) {
          continue;
        }
        for (int delta_column = -radius; delta_column <= radius; ++delta_column) {
          const int next_column = static_cast<int>(column) + delta_column;
          if (next_column < 0 || next_column >= static_cast<int>(columns) ||
              (delta_row == 0 && delta_column == 0)) {
            continue;
          }
          const auto neighbour_index = indexOf(static_cast<std::uint32_t>(next_row),
                                               static_cast<std::uint32_t>(next_column), columns);
          const float neighbour = height[neighbour_index];
          if (!finite(neighbour)) {
            continue;
          }
          const float distance_squared = static_cast<float>(
              delta_row * delta_row + delta_column * delta_column);
          const float weight = 1.0F / (1.0F + distance_squared);
          weighted_sum += weight * neighbour;
          weight_sum += weight;
          minimum = std::min(minimum, neighbour);
          maximum = std::max(maximum, neighbour);
          variance_sum += variance[neighbour_index];
          ++count;
        }
      }
      // This is the important stair-preserving gate: a cell surrounded by two
      // levels is left unknown rather than interpolated across a riser.
      if (count >= min_neighbours && weight_sum > 1.0e-6F &&
          maximum - minimum <= max_height_difference) {
        filled[index] = weighted_sum / weight_sum;
        filled_variance[index] = std::min(
            maximum_variance,
            variance_sum / static_cast<float>(count) +
                max_height_difference * max_height_difference);
        // This cell has no source point in the current cohort.  It is valid
        // only because the current-frame, edge-aware filter established one
        // locally continuous surface around it; it must never be confused
        // with retained terrain from an earlier observation.
        filled_mask[index] = 1U;
      }
    }
  }
  height.swap(filled);
  variance.swap(filled_variance);
  filter_filled.swap(filled_mask);
}

void smoothObservedSurfacesEdgeAware(std::vector<float> &height,
                                     std::vector<float> &variance,
                                     const std::uint32_t rows,
                                     const std::uint32_t columns,
                                     const int radius,
                                     const float sigma_spatial,
                                     const float sigma_height,
                                     const float maximum_height_difference,
                                     const int min_support_neighbours,
                                     const float blend,
                                     const int passes) {
  if (radius <= 0 || sigma_spatial <= 0.0F || sigma_height <= 0.0F ||
      maximum_height_difference <= 0.0F || min_support_neighbours <= 0 ||
      blend <= 0.0F || passes <= 0) {
    return;
  }
  const float inverse_spatial = 1.0F / (2.0F * sigma_spatial * sigma_spatial);
  const float inverse_height = 1.0F / (2.0F * sigma_height * sigma_height);
  std::vector<float> source = height;
  std::vector<float> source_variance = variance;
  std::vector<float> destination = height;
  std::vector<float> destination_variance = variance;
  for (int pass = 0; pass < passes; ++pass) {
    destination = source;
    destination_variance = source_variance;
    for (std::uint32_t row = 0; row < rows; ++row) {
      for (std::uint32_t column = 0; column < columns; ++column) {
        const auto index = indexOf(row, column, columns);
        const float center = source[index];
        if (!finite(center)) {
          continue;
        }
        float weighted_sum = 0.0F;
        float weights = 0.0F;
        float inverse_variance_sum = 0.0F;
        int support = 0;
        for (int delta_row = -radius; delta_row <= radius; ++delta_row) {
          const int next_row = static_cast<int>(row) + delta_row;
          if (next_row < 0 || next_row >= static_cast<int>(rows)) {
            continue;
          }
          for (int delta_column = -radius; delta_column <= radius; ++delta_column) {
            const int next_column = static_cast<int>(column) + delta_column;
            if (next_column < 0 || next_column >= static_cast<int>(columns) ||
                (delta_row == 0 && delta_column == 0)) {
              continue;
            }
            const auto neighbour_index = indexOf(static_cast<std::uint32_t>(next_row),
                                                 static_cast<std::uint32_t>(next_column), columns);
            const float neighbour = source[neighbour_index];
            const float delta = neighbour - center;
            if (!finite(neighbour) || std::fabs(delta) > maximum_height_difference) {
              continue;
            }
            const float distance_squared = static_cast<float>(
                delta_row * delta_row + delta_column * delta_column);
            const float measurement_weight = 1.0F / std::max(source_variance[neighbour_index], 1.0e-6F);
            const float weight = measurement_weight *
                std::exp(-distance_squared * inverse_spatial) *
                std::exp(-(delta * delta) * inverse_height);
            weighted_sum += weight * neighbour;
            weights += weight;
            inverse_variance_sum += measurement_weight;
            ++support;
          }
        }
        if (support >= min_support_neighbours && weights > 1.0e-6F) {
          const float filtered = weighted_sum / weights;
          destination[index] = (1.0F - blend) * center + blend * filtered;
          if (inverse_variance_sum > 1.0e-6F) {
            destination_variance[index] = std::min(
                source_variance[index], 1.0F / inverse_variance_sum);
          }
        }
      }
    }
    source.swap(destination);
    source_variance.swap(destination_variance);
  }
  height.swap(source);
  variance.swap(source_variance);
}

std_msgs::msg::Float32MultiArray layer(
    const std::vector<float> &row_major, const std::uint32_t width,
    const std::uint32_t height) {
  std_msgs::msg::Float32MultiArray output;
  output.layout.dim.resize(2);
  output.layout.dim[0].label = "column_index";
  output.layout.dim[0].size = width;
  output.layout.dim[0].stride = width * height;
  output.layout.dim[1].label = "row_index";
  output.layout.dim[1].size = height;
  output.layout.dim[1].stride = height;
  output.data.resize(static_cast<std::size_t>(width) * height, kNan);
  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t column = 0; column < width; ++column) {
      output.data[static_cast<std::size_t>(column) * height + row] =
          row_major[indexOf(row, column, width)];
    }
  }
  return output;
}

}  // namespace

struct PreciseElevationMapping::Impl {
  explicit Impl(PreciseElevationMapping &node)
      : node(node), tf_buffer(node.get_clock()), tf_listener(tf_buffer) {
    loadParameters();
    createIo();
  }

  void loadParameters() {
    input_topic = node.declare_parameter<std::string>(
        "input_topic", "/autonomy_light/merged_observations");
    output_topic = node.declare_parameter<std::string>(
        "output_topic", "/autonomy_light/elevation_map");
    output_frame = node.declare_parameter<std::string>("output_frame", "base_link_gravity");
    publish_rate_hz = node.declare_parameter<double>("publish_rate_hz", 50.0);
    map_frame = node.declare_parameter<std::string>("slam.map_frame", "map");
    temporal_enabled = node.declare_parameter<bool>("algorithm.temporal.enabled", true);
    temporal_require_slam_tf = node.declare_parameter<bool>(
        "algorithm.temporal.require_slam_tf", true);
    temporal_tf_timeout_sec = node.declare_parameter<double>(
        "algorithm.temporal.tf_timeout_sec", 0.02);
    temporal_variance_rate = node.declare_parameter<double>(
        "algorithm.temporal.time_variance_rate", 0.20);
    temporal_mahalanobis_threshold = node.declare_parameter<double>(
        "algorithm.temporal.mahalanobis_threshold", 3.0);
    temporal_dynamic_reset_delta = node.declare_parameter<double>(
        "algorithm.temporal.dynamic_reset_delta", 0.10);
    temporal_dynamic_variance_bump = node.declare_parameter<double>(
        "algorithm.temporal.dynamic_variance_bump", 0.0225);
    grid.resolution = node.declare_parameter<double>("grid.resolution", grid.resolution);
    grid.x_min = node.declare_parameter<double>("grid.x_min", grid.x_min);
    grid.x_max = node.declare_parameter<double>("grid.x_max", grid.x_max);
    grid.y_min = node.declare_parameter<double>("grid.y_min", grid.y_min);
    grid.y_max = node.declare_parameter<double>("grid.y_max", grid.y_max);
    grid.z_min = node.declare_parameter<double>("grid.z_min", grid.z_min);
    grid.z_max = node.declare_parameter<double>("grid.z_max", grid.z_max);
    noise_alpha = node.declare_parameter<double>("algorithm.uncertainty.noise_alpha", 0.001);
    minimum_measurement_variance = node.declare_parameter<double>(
        "algorithm.uncertainty.min_meas_var", 0.0004);
    maximum_cell_variance = node.declare_parameter<double>(
        "algorithm.uncertainty.max_cell_var", 0.01);
    robust_height_gate = node.declare_parameter<double>(
        "algorithm.frame_aggregation.robust_height_gate", 0.04);
    lower_support_gap = node.declare_parameter<double>(
        "algorithm.frame_aggregation.intra_cell_min_support_gap", 0.025);
    lower_support_count = node.declare_parameter<int>(
        "algorithm.frame_aggregation.intra_cell_min_support_count", 2);
    edge_mix_height_difference = node.declare_parameter<double>(
        "algorithm.frame_aggregation.edge_mix_height_diff", 0.035);
    edge_prefer_lower_support_count = node.declare_parameter<int>(
        "algorithm.frame_aggregation.edge_prefer_prev_support_count", 2);
    isolated_radius = node.declare_parameter<int>("algorithm.isolated_filter.radius", 1);
    isolated_min_support = node.declare_parameter<int>(
        "algorithm.isolated_filter.min_support_neighbors", 2);
    isolated_support_difference = node.declare_parameter<double>(
        "algorithm.isolated_filter.support_height_diff", 0.025);
    isolated_outlier_difference = node.declare_parameter<double>(
        "algorithm.isolated_filter.outlier_height_diff", 0.05);
    hole_radius = node.declare_parameter<int>("algorithm.hole_fill.radius", 1);
    hole_min_neighbours = node.declare_parameter<int>("algorithm.hole_fill.min_neighbors", 3);
    hole_max_difference = node.declare_parameter<double>(
        "algorithm.hole_fill.max_height_diff", 0.03);
    smooth_enabled = node.declare_parameter<bool>("algorithm.surface_smoothing.enabled", true);
    smooth_radius = node.declare_parameter<int>("algorithm.surface_smoothing.radius", 1);
    smooth_sigma_spatial = node.declare_parameter<double>(
        "algorithm.surface_smoothing.sigma_spatial", 1.0);
    smooth_sigma_height = node.declare_parameter<double>(
        "algorithm.surface_smoothing.sigma_height", 0.012);
    smooth_max_difference = node.declare_parameter<double>(
        "algorithm.surface_smoothing.max_height_diff", 0.030);
    smooth_min_support = node.declare_parameter<int>(
        "algorithm.surface_smoothing.min_support_neighbors", 3);
    smooth_blend = node.declare_parameter<double>("algorithm.surface_smoothing.blend", 0.45);
    smooth_passes = node.declare_parameter<int>("algorithm.surface_smoothing.passes", 1);
    debug_enabled = node.declare_parameter<bool>("debug.enabled", false);
    debug_pointcloud_topic = node.declare_parameter<std::string>(
        "debug.pointcloud_topic", "/autonomy_light/elevation_map/debug_points");

    if (input_topic.empty() || output_topic.empty() || output_frame.empty() || map_frame.empty() ||
        publish_rate_hz <= 0.0 || grid.resolution <= 0.0 ||
        grid.x_max <= grid.x_min || grid.y_max <= grid.y_min ||
        grid.z_max <= grid.z_min || grid.width() == 0 || grid.height() == 0 ||
        minimum_measurement_variance <= 0.0 || maximum_cell_variance < minimum_measurement_variance ||
        lower_support_gap <= 0.0 || robust_height_gate <= 0.0 ||
        edge_mix_height_difference <= 0.0 || smooth_blend < 0.0 || smooth_blend > 1.0 ||
        temporal_tf_timeout_sec < 0.0 || temporal_variance_rate < 0.0 ||
        temporal_mahalanobis_threshold <= 0.0 || temporal_dynamic_reset_delta < 0.0 ||
        temporal_dynamic_variance_bump < 0.0 ||
        (debug_enabled && debug_pointcloud_topic.empty())) {
      throw std::invalid_argument("precise_elevation_mapping parameters are invalid");
    }
  }

  void createIo() {
    input_callback_group = node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    output_callback_group = node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions input_options;
    input_options.callback_group = input_callback_group;
    input = node.create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
          onCloud(std::move(message));
        }, input_options);
    output = node.create_publisher<grid_map_msgs::msg::GridMap>(
        output_topic, rclcpp::QoS(1).reliable());
    if (debug_enabled) {
      debug_pointcloud = node.create_publisher<sensor_msgs::msg::PointCloud2>(
          debug_pointcloud_topic, rclcpp::SensorDataQoS());
    }
    heartbeat = node.create_publisher<std_msgs::msg::String>(
        "/autonomy_light/heartbeat/precise_elevation_mapping", rclcpp::QoS(10));
    processing_time = node.create_publisher<std_msgs::msg::Float32>(
        "/autonomy_light/elevation_processing_ms", rclcpp::SensorDataQoS());
    const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer = node.create_wall_timer(
        publish_period, [this]() { publishLatestMap(); }, output_callback_group);
    heartbeat_timer = node.create_wall_timer(
        std::chrono::milliseconds(500), [this]() { publishHeartbeat(); }, output_callback_group);
    RCLCPP_INFO(node.get_logger(),
                "Precise elevation map: %s -> %s @ %.1f Hz, %ux%u @ %.3fm, "
                "%s edge-aware filtering (SLAM-reprojected temporal fusion: %s)",
                input_topic.c_str(), output_topic.c_str(), publish_rate_hz, grid.width(), grid.height(),
                grid.resolution, temporal_enabled ? "temporal" : "single-frame",
                temporal_enabled ? "enabled" : "disabled");
    if (debug_enabled) {
      RCLCPP_INFO(node.get_logger(), "Elevation debug PointCloud2: %s",
                  debug_pointcloud_topic.c_str());
    }
  }

  void publishDebugCloud(const std::vector<float> &heights,
                         const rclcpp::Time &stamp,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher) {
    if (!publisher) {
      return;
    }
    const auto width = grid.width();
    const auto height = grid.height();
    std::vector<PointXYZIntensity> points;
    points.reserve(heights.size());
    for (std::uint32_t row = 0; row < height; ++row) {
      const float y = static_cast<float>(grid.y_min +
          (static_cast<double>(row) + 0.5) * grid.resolution);
      for (std::uint32_t column = 0; column < width; ++column) {
        const float z = heights[indexOf(row, column, width)];
        if (!finite(z)) {
          continue;
        }
        const float x = static_cast<float>(grid.x_min +
            (static_cast<double>(column) + 0.5) * grid.resolution);
        points.push_back({x, y, z, z});
      }
    }
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = output_frame;
    message.height = 1;
    message.width = static_cast<std::uint32_t>(points.size());
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = sizeof(PointXYZIntensity);
    message.row_step = message.point_step * message.width;
    message.fields.resize(4);
    const std::array<const char *, 4> names{"x", "y", "z", "intensity"};
    for (std::size_t index = 0; index < message.fields.size(); ++index) {
      message.fields[index].name = names[index];
      message.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
      message.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[index].count = 1;
    }
    message.data.resize(static_cast<std::size_t>(message.row_step));
    if (!points.empty()) {
      std::memcpy(message.data.data(), points.data(), message.data.size());
    }
    publisher->publish(std::move(message));
  }

  bool prepareTemporalPrior(const rclcpp::Time &stamp,
                            const std::uint32_t width,
                            const std::uint32_t height) {
    if (!temporal_enabled) {
      return true;
    }
    try {
      const auto transform_message = tf_buffer.lookupTransform(
          map_frame, output_frame, stamp,
          rclcpp::Duration::from_seconds(temporal_tf_timeout_sec));
      tf2::fromMsg(transform_message.transform, current_map_from_level);
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for Super-LIO attitude TF %s -> %s: %s",
                           map_frame.c_str(), output_frame.c_str(), error.what());
      return false;
    }

    const auto cells = static_cast<std::size_t>(width) * height;
    temporal_elevation.assign(cells, kNan);
    temporal_variance.assign(cells, static_cast<float>(maximum_cell_variance));
    if (!have_temporal_state) {
      return true;
    }

    const double dt_sec = std::max(0.0, (stamp - temporal_stamp).seconds());
    const float propagated_variance = static_cast<float>(std::min(
        maximum_cell_variance,
        temporal_variance_rate * temporal_variance_rate * dt_sec * dt_sec));
    const tf2::Transform level_from_previous =
        current_map_from_level.inverse() * previous_map_from_level;
    const auto previous = temporal_elevation_state;
    const auto previous_variance = temporal_variance_state;
    for (std::uint32_t row = 0; row < height; ++row) {
      for (std::uint32_t column = 0; column < width; ++column) {
        const auto previous_index = indexOf(row, column, width);
        const float previous_height = previous[previous_index];
        if (!finite(previous_height)) {
          continue;
        }
        const double x = grid.x_min + (static_cast<double>(column) + 0.5) * grid.resolution;
        const double y = grid.y_min + (static_cast<double>(row) + 0.5) * grid.resolution;
        const tf2::Vector3 current_point = level_from_previous *
            tf2::Vector3(x, y, previous_height);
        const double current_x = current_point.x();
        const double current_y = current_point.y();
        const double current_z = current_point.z();
        if (!std::isfinite(current_x) || !std::isfinite(current_y) || !std::isfinite(current_z) ||
            current_x < grid.x_min || current_x >= grid.x_max ||
            current_y < grid.y_min || current_y >= grid.y_max ||
            current_z < grid.z_min || current_z > grid.z_max) {
          continue;
        }
        const auto current_column = static_cast<std::uint32_t>(
            (current_x - grid.x_min) / grid.resolution);
        const auto current_row = static_cast<std::uint32_t>(
            (current_y - grid.y_min) / grid.resolution);
        if (current_column >= width || current_row >= height) {
          continue;
        }
        const auto current_index = indexOf(current_row, current_column, width);
        const float candidate_variance = std::clamp(
            previous_variance[previous_index] + propagated_variance,
            static_cast<float>(minimum_measurement_variance),
            static_cast<float>(maximum_cell_variance));
        if (!finite(temporal_elevation[current_index]) ||
            candidate_variance < temporal_variance[current_index]) {
          temporal_elevation[current_index] = static_cast<float>(current_z);
          temporal_variance[current_index] = candidate_variance;
        }
      }
    }
    return true;
  }

  void fuseTemporalObservation(const std::vector<float> &measurement,
                               const std::vector<float> &measurement_variance,
                               std::vector<float> &elevation,
                               std::vector<float> &variance) {
    if (!temporal_enabled) {
      return;
    }
    const float threshold_squared = static_cast<float>(
        temporal_mahalanobis_threshold * temporal_mahalanobis_threshold);
    for (std::size_t index = 0; index < measurement.size(); ++index) {
      const float observed = measurement[index];
      if (!finite(observed)) {
        continue;
      }
      const float observed_variance = measurement_variance[index];
      float &state = temporal_elevation[index];
      float &state_variance = temporal_variance[index];
      if (!finite(state)) {
        state = observed;
        state_variance = observed_variance;
        continue;
      }
      const float difference = observed - state;
      const float combined_variance = std::max(
          state_variance + observed_variance, 1.0e-6F);
      const float mahalanobis_squared = difference * difference / combined_variance;
      if (mahalanobis_squared > threshold_squared &&
          std::fabs(difference) > static_cast<float>(temporal_dynamic_reset_delta)) {
        const float blend = observed < state ? 0.8F : 0.5F;
        state = blend * observed + (1.0F - blend) * state;
        state_variance = std::clamp(
            state_variance + static_cast<float>(temporal_dynamic_variance_bump),
            static_cast<float>(minimum_measurement_variance),
            static_cast<float>(maximum_cell_variance));
        continue;
      }
      const float gain = std::clamp(state_variance / combined_variance, 0.2F, 0.8F);
      state += gain * difference;
      state_variance = std::clamp((1.0F - gain) * state_variance,
                                  static_cast<float>(minimum_measurement_variance),
                                  static_cast<float>(maximum_cell_variance));
    }
    elevation = temporal_elevation;
    variance = temporal_variance;
  }

  void commitTemporalState(const rclcpp::Time &stamp,
                           const std::vector<float> &elevation,
                           const std::vector<float> &variance) {
    if (!temporal_enabled) {
      return;
    }
    temporal_elevation_state = elevation;
    temporal_variance_state = variance;
    previous_map_from_level = current_map_from_level;
    temporal_stamp = stamp;
    have_temporal_state = true;
  }

  void publishHeartbeat() {
    std_msgs::msg::String message;
    message.data = frame_count.load() == 0 ? "waiting_for_observation" : "ready";
    heartbeat->publish(message);
  }

  void publishLatestMap() {
    std::shared_ptr<grid_map_msgs::msg::GridMap> map;
    {
      std::lock_guard<std::mutex> lock(latest_map_mutex);
      map = latest_map;
    }
    if (map) {
      output->publish(*map);
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    const auto processing_started = std::chrono::steady_clock::now();
    if (!cloud) {
      return;
    }
    FieldOffsets offsets;
    const auto expected_bytes = static_cast<std::size_t>(cloud->point_step) * cloud->width * cloud->height;
    if (!fields(*cloud, offsets) || cloud->point_step == 0 || cloud->data.size() < expected_bytes) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Merged observation is not an XYZ/variance PointCloud2");
      return;
    }
    const auto width = grid.width();
    const auto height = grid.height();
    const auto cells = static_cast<std::size_t>(width) * height;
    const rclcpp::Time cloud_stamp(cloud->header.stamp);
    const bool temporal_ready = prepareTemporalPrior(cloud_stamp, width, height);
    if (temporal_enabled && !temporal_ready && temporal_require_slam_tf) {
      return;
    }
    std::vector<FrameCell> frame(cells);
    std::size_t accepted = 0;
    const auto point_count = static_cast<std::size_t>(cloud->width) * cloud->height;
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const auto *point = cloud->data.data() + point_index * cloud->point_step;
      const float x = readFloat(point, offsets.x);
      const float y = readFloat(point, offsets.y);
      const float z = readFloat(point, offsets.z);
      if (!finite(x) || !finite(y) || !finite(z) || x < grid.x_min || x >= grid.x_max ||
          y < grid.y_min || y >= grid.y_max || z < grid.z_min || z > grid.z_max) {
        continue;
      }
      const auto column = static_cast<std::uint32_t>((x - grid.x_min) / grid.resolution);
      const auto row = static_cast<std::uint32_t>((y - grid.y_min) / grid.resolution);
      if (column >= width || row >= height) {
        continue;
      }
      const auto cell_index = indexOf(row, column, width);
      auto &cell = frame[cell_index];
      float measurement_variance = static_cast<float>(minimum_measurement_variance);
      if (offsets.has_variance) {
        const float supplied = readFloat(point, offsets.variance);
        if (finite(supplied) && supplied > 0.0F) {
          measurement_variance = std::max(measurement_variance, supplied);
        }
      } else {
        const float range_squared = x * x + y * y + z * z;
        measurement_variance = std::max(
            measurement_variance, static_cast<float>(noise_alpha) * range_squared);
      }
      const float weight = 1.0F / std::max(measurement_variance, 1.0e-6F);
      ++cell.hits;
      cell.weighted_sum += weight * z;
      cell.weight_sum += weight;
      cell.min_height = std::min(cell.min_height, z);
      cell.max_height = std::max(cell.max_height, z);
      if (z + static_cast<float>(lower_support_gap) < cell.lower_height) {
        cell.lower_height = z;
        cell.lower_support_count = 1;
        cell.lower_weighted_sum = weight * z;
        cell.lower_weight_sum = weight;
      } else if (std::fabs(z - cell.lower_height) <= static_cast<float>(lower_support_gap)) {
        ++cell.lower_support_count;
        cell.lower_weighted_sum += weight * z;
        cell.lower_weight_sum += weight;
      }
      ++accepted;
    }

    std::vector<float> elevation(cells, kNan);
    std::vector<float> variance(cells, static_cast<float>(maximum_cell_variance));
    for (std::size_t cell_index = 0; cell_index < cells; ++cell_index) {
      const auto &cell = frame[cell_index];
      if (cell.hits == 0 || cell.weight_sum <= 1.0e-6F) {
        continue;
      }
      float value = cell.weighted_sum / cell.weight_sum;
      float value_variance = std::max(
          static_cast<float>(minimum_measurement_variance), 1.0F / cell.weight_sum);
      const bool mixed_edge = finite(cell.min_height) && finite(cell.max_height) &&
                              cell.max_height - cell.min_height > edge_mix_height_difference;
      if (cell.lower_support_count >= static_cast<std::uint32_t>(std::max(1, lower_support_count)) &&
          cell.lower_weight_sum > 1.0e-6F) {
        const float lower = cell.lower_weighted_sum / cell.lower_weight_sum;
        // The height_cli_lib lower support cluster prevents a riser and its
        // adjacent tread from being averaged into a false ramp.
        if (mixed_edge || cell.lower_support_count >= static_cast<std::uint32_t>(
                              std::max(1, edge_prefer_lower_support_count)) ||
            std::fabs(lower - value) <= robust_height_gate) {
          value = lower;
          value_variance = std::max(static_cast<float>(minimum_measurement_variance),
                                    1.0F / cell.lower_weight_sum);
        }
      }
      elevation[cell_index] = value;
      variance[cell_index] = std::clamp(value_variance,
                                        static_cast<float>(minimum_measurement_variance),
                                        static_cast<float>(maximum_cell_variance));
    }

    const auto raw_elevation = elevation;
    if (temporal_enabled && temporal_ready) {
      fuseTemporalObservation(raw_elevation, variance, elevation, variance);
    }
    std::vector<std::uint8_t> filter_filled(cells, 0U);

    removeIsolatedCellsEdgeAware(
        elevation, variance, height, width, isolated_radius, isolated_min_support,
        static_cast<float>(isolated_support_difference),
        static_cast<float>(isolated_outlier_difference),
        static_cast<float>(maximum_cell_variance));
    fillSmallHolesEdgeAware(
        elevation, variance, filter_filled, height, width, hole_radius, hole_min_neighbours,
        static_cast<float>(hole_max_difference), static_cast<float>(maximum_cell_variance));
    if (smooth_enabled) {
      smoothObservedSurfacesEdgeAware(
          elevation, variance, height, width, smooth_radius,
          static_cast<float>(smooth_sigma_spatial), static_cast<float>(smooth_sigma_height),
          static_cast<float>(smooth_max_difference), smooth_min_support,
          static_cast<float>(smooth_blend), smooth_passes);
    }
    if (temporal_enabled && temporal_ready) {
      commitTemporalState(cloud_stamp, elevation, variance);
    }

    std::vector<float> lower(cells, kNan);
    std::vector<float> upper(cells, kNan);
    std::vector<float> uncertainty(cells, kNan);
    std::vector<float> current_observation(cells, kNan);
    std::vector<float> filter_filled_layer(cells, kNan);
    std::size_t valid = 0;
    std::size_t raw_observed = 0;
    std::size_t filled = 0;
    for (std::size_t cell_index = 0; cell_index < cells; ++cell_index) {
      if (finite(raw_elevation[cell_index])) {
        current_observation[cell_index] = 1.0F;
        ++raw_observed;
      }
      if (!finite(elevation[cell_index])) {
        continue;
      }
      if (filter_filled[cell_index] != 0U) {
        filter_filled_layer[cell_index] = 1.0F;
        ++filled;
      }
      const float sigma = std::sqrt(std::max(variance[cell_index], 0.0F));
      lower[cell_index] = elevation[cell_index] - 2.0F * sigma;
      upper[cell_index] = elevation[cell_index] + 2.0F * sigma;
      uncertainty[cell_index] = 4.0F * sigma;
      ++valid;
    }
    if (debug_enabled) {
      publishDebugCloud(elevation, rclcpp::Time(cloud->header.stamp), debug_pointcloud);
    }

    grid_map_msgs::msg::GridMap message;
    message.header = cloud->header;
    message.header.frame_id = output_frame;
    message.info.resolution = grid.resolution;
    message.info.length_x = static_cast<double>(width) * grid.resolution;
    message.info.length_y = static_cast<double>(height) * grid.resolution;
    message.info.pose.position.x = grid.x_min + message.info.length_x / 2.0;
    message.info.pose.position.y = grid.y_min + message.info.length_y / 2.0;
    message.info.pose.position.z = 0.0;
    message.info.pose.orientation.w = 1.0;
    message.layers = {"elevation", "lower_bound", "upper_bound", "uncertainty_range"};
    if (debug_enabled) {
      // raw_elevation and current_observation contain only this cloud's
      // measurements. The final elevation may additionally contain a
      // Super-LIO-reprojected temporal estimate; filter_filled marks the
      // current local interpolation exception.
      message.layers.insert(message.layers.end(), {
          "raw_elevation", "current_observation", "filter_filled"});
    }
    message.data.reserve(message.layers.size());
    message.data.push_back(layer(elevation, width, height));
    message.data.push_back(layer(lower, width, height));
    message.data.push_back(layer(upper, width, height));
    message.data.push_back(layer(uncertainty, width, height));
    if (debug_enabled) {
      message.data.push_back(layer(raw_elevation, width, height));
      message.data.push_back(layer(current_observation, width, height));
      message.data.push_back(layer(filter_filled_layer, width, height));
    }
    message.outer_start_index = 0;
    message.inner_start_index = 0;
    {
      std::lock_guard<std::mutex> lock(latest_map_mutex);
      latest_map = std::make_shared<grid_map_msgs::msg::GridMap>(std::move(message));
    }
    std_msgs::msg::Float32 processing_message;
    processing_message.data = static_cast<float>(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   processing_started).count());
    processing_time->publish(processing_message);
    ++frame_count;
    RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                         "Precise frame: input=%zu ROI=%zu raw=%zu filter_fill=%zu valid=%zu/%zu (%.1f%%)",
                         point_count, accepted, raw_observed, filled, valid, cells,
                         100.0 * static_cast<double>(valid) / static_cast<double>(cells));
  }

  PreciseElevationMapping &node;
  std::string input_topic;
  std::string output_topic;
  std::string output_frame;
  std::string map_frame;
  double publish_rate_hz{50.0};
  GridGeometry grid;
  double noise_alpha{0.001};
  double minimum_measurement_variance{0.0004};
  double maximum_cell_variance{0.01};
  double robust_height_gate{0.04};
  double lower_support_gap{0.025};
  int lower_support_count{2};
  double edge_mix_height_difference{0.035};
  int edge_prefer_lower_support_count{2};
  int isolated_radius{1};
  int isolated_min_support{2};
  double isolated_support_difference{0.025};
  double isolated_outlier_difference{0.05};
  int hole_radius{1};
  int hole_min_neighbours{3};
  double hole_max_difference{0.03};
  bool smooth_enabled{true};
  int smooth_radius{1};
  double smooth_sigma_spatial{1.0};
  double smooth_sigma_height{0.012};
  double smooth_max_difference{0.03};
  int smooth_min_support{3};
  double smooth_blend{0.45};
  int smooth_passes{1};
  bool temporal_enabled{true};
  bool temporal_require_slam_tf{true};
  double temporal_tf_timeout_sec{0.02};
  double temporal_variance_rate{0.20};
  double temporal_mahalanobis_threshold{3.0};
  double temporal_dynamic_reset_delta{0.10};
  double temporal_dynamic_variance_bump{0.0225};
  bool debug_enabled{false};
  std::string debug_pointcloud_topic;
  std::atomic<std::uint64_t> frame_count{0};
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  bool have_temporal_state{false};
  rclcpp::Time temporal_stamp{0, 0, RCL_ROS_TIME};
  tf2::Transform previous_map_from_level;
  tf2::Transform current_map_from_level;
  std::vector<float> temporal_elevation_state;
  std::vector<float> temporal_variance_state;
  std::vector<float> temporal_elevation;
  std::vector<float> temporal_variance;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr output;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time;
  rclcpp::CallbackGroup::SharedPtr input_callback_group;
  rclcpp::CallbackGroup::SharedPtr output_callback_group;
  std::mutex latest_map_mutex;
  std::shared_ptr<grid_map_msgs::msg::GridMap> latest_map;
  rclcpp::TimerBase::SharedPtr publish_timer;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
};

PreciseElevationMapping::PreciseElevationMapping(const rclcpp::NodeOptions &options)
    : Node("elevation_mapping", options) {
  impl_ = std::make_unique<Impl>(*this);
}

PreciseElevationMapping::~PreciseElevationMapping() = default;

}  // namespace autonomy_light
