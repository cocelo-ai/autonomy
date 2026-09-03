#include "algorithm/elevation/elevation_map.hpp"
#include "autonomy_light/msg/height_map.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace autonomy_light::algorithm::elevation {
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

struct HeightSample {
  float height;
  float weight;
  float variance;
};

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
  std::vector<HeightSample> samples;
};

bool finite(const float value) { return std::isfinite(value); }

bool selectHighestSupportedCluster(std::vector<HeightSample> samples,
                                   const float cluster_gap,
                                   const std::uint32_t minimum_support,
                                   float &height,
                                   float &variance) {
  if (samples.empty() || cluster_gap <= 0.0F || minimum_support == 0U) {
    return false;
  }
  std::sort(samples.begin(), samples.end(), [](const HeightSample &left,
                                               const HeightSample &right) {
    return left.height > right.height;
  });
  for (std::size_t first = 0U; first < samples.size();) {
    const float upper = samples[first].height;
    float weighted_sum = 0.0F;
    float weight_sum = 0.0F;
    float variance_sum = 0.0F;
    std::size_t last = first;
    while (last < samples.size() && upper - samples[last].height <= cluster_gap) {
      weighted_sum += samples[last].weight * samples[last].height;
      weight_sum += samples[last].weight;
      variance_sum += samples[last].variance;
      ++last;
    }
    if (last - first >= minimum_support && weight_sum > 1.0e-6F) {
      height = weighted_sum / weight_sum;
      variance = variance_sum / static_cast<float>(last - first);
      return true;
    }
    // A detached upper cluster without sufficient support is a noise
    // candidate.  Discard it and consider the next lower cluster.
    first = last;
  }
  return false;
}

bool selectHighestSupportedCluster(std::vector<float> samples,
                                   const float cluster_gap,
                                   const std::uint32_t minimum_support,
                                   float &height) {
  if (samples.empty() || cluster_gap <= 0.0F || minimum_support == 0U) {
    return false;
  }
  std::sort(samples.begin(), samples.end(), std::greater<float>());
  for (std::size_t first = 0U; first < samples.size();) {
    const float upper = samples[first];
    float sum = 0.0F;
    std::size_t last = first;
    while (last < samples.size() && upper - samples[last] <= cluster_gap) {
      sum += samples[last];
      ++last;
    }
    if (last - first >= minimum_support) {
      height = sum / static_cast<float>(last - first);
      return true;
    }
    // Ignore an unsupported isolated high return before trying the next
    // cluster below it.
    first = last;
  }
  return false;
}

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

// This is the reference implementation's bilateral stage: it fills only
// missing cells and never averages an already observed obstacle/edge into its
// neighbours.  It is deliberately different from the temporal-path smoother.
void fillReferenceBilateralHoles(std::vector<float> &height,
                                 const std::uint32_t rows,
                                 const std::uint32_t columns,
                                 const int radius,
                                 const float sigma_spatial,
                                 const float sigma_height,
                                 const float maximum_height_difference,
                                 const int passes) {
  if (radius <= 0 || sigma_spatial <= 0.0F || sigma_height <= 0.0F ||
      maximum_height_difference <= 0.0F || passes <= 0) {
    return;
  }
  const float inverse_spatial = 1.0F / (2.0F * sigma_spatial * sigma_spatial);
  const float inverse_height = 1.0F / (2.0F * sigma_height * sigma_height);
  auto source = height;
  auto destination = height;
  std::vector<float> neighbours;
  neighbours.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
  for (int pass = 0; pass < passes; ++pass) {
    destination = source;
    for (std::uint32_t row = 0; row < rows; ++row) {
      for (std::uint32_t column = 0; column < columns; ++column) {
        const auto index = indexOf(row, column, columns);
        if (finite(source[index])) {
          continue;
        }
        neighbours.clear();
        float minimum = std::numeric_limits<float>::infinity();
        float maximum = -std::numeric_limits<float>::infinity();
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
            const float neighbour = source[indexOf(static_cast<std::uint32_t>(next_row),
                                                    static_cast<std::uint32_t>(next_column), columns)];
            if (!finite(neighbour)) {
              continue;
            }
            neighbours.push_back(neighbour);
            minimum = std::min(minimum, neighbour);
            maximum = std::max(maximum, neighbour);
          }
        }
        if (neighbours.empty() || maximum - minimum > maximum_height_difference) {
          continue;
        }
        const auto middle = neighbours.begin() +
            static_cast<std::ptrdiff_t>(neighbours.size() / 2U);
        std::nth_element(neighbours.begin(), middle, neighbours.end());
        const float reference_height = *middle;
        float weighted_sum = 0.0F;
        float weight_sum = 0.0F;
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
            const float neighbour = source[indexOf(static_cast<std::uint32_t>(next_row),
                                                    static_cast<std::uint32_t>(next_column), columns)];
            if (!finite(neighbour) ||
                std::fabs(neighbour - reference_height) > maximum_height_difference) {
              continue;
            }
            const float distance_squared = static_cast<float>(
                delta_row * delta_row + delta_column * delta_column);
            const float delta = neighbour - reference_height;
            const float weight = std::exp(-distance_squared * inverse_spatial) *
                std::exp(-(delta * delta) * inverse_height);
            weighted_sum += weight * neighbour;
            weight_sum += weight;
          }
        }
        if (weight_sum > 1.0e-6F) {
          destination[index] = weighted_sum / weight_sum;
        }
      }
    }
    source.swap(destination);
  }
  height.swap(source);
}

void updateReferenceCell(const float measurement,
                         const float measurement_variance,
                         float &state,
                         float &state_variance,
                         const float mahalanobis_threshold_squared,
                         const float dynamic_reset_delta,
                         const float dynamic_variance_bump,
                         const float minimum_variance,
                         const float maximum_variance) {
  if (!finite(state)) {
    state = measurement;
    state_variance = measurement_variance;
    return;
  }
  const float delta = measurement - state;
  const float combined_variance = std::max(state_variance + measurement_variance, 1.0e-6F);
  if (delta * delta / combined_variance > mahalanobis_threshold_squared &&
      std::fabs(delta) > dynamic_reset_delta) {
    const float blend = measurement < state ? 0.8F : 0.5F;
    state = blend * measurement + (1.0F - blend) * state;
    state_variance = std::clamp(state_variance + dynamic_variance_bump,
                                minimum_variance, maximum_variance);
    return;
  }
  const float gain = std::clamp(state_variance / combined_variance, 0.2F, 0.8F);
  state += gain * delta;
  state_variance = std::clamp((1.0F - gain) * state_variance,
                              minimum_variance, maximum_variance);
}

// NaN is useful as an internal "not observed" sentinel while filtering, but
// it is not a valid external height-map value. Propagate nearby
// terrain into each hole first; if an entire map has no observation, use the
// explicit finite fallback required by the interface.
void fillRemainingHolesStrict(std::vector<float> &height,
                              std::vector<float> &variance,
                              const std::uint32_t rows,
                              const std::uint32_t columns,
                              const float fallback_variance,
                              const float fully_unknown_value) {
  if (height.empty() || rows == 0U || columns == 0U) {
    return;
  }

  auto source = height;
  auto destination = height;
  auto source_variance = variance;
  auto destination_variance = variance;
  const auto max_iterations = std::max(rows, columns);
  for (std::uint32_t iteration = 0; iteration < max_iterations; ++iteration) {
    bool filled_any = false;
    destination = source;
    destination_variance = source_variance;
    for (std::uint32_t row = 0; row < rows; ++row) {
      for (std::uint32_t column = 0; column < columns; ++column) {
        const auto index = indexOf(row, column, columns);
        if (finite(source[index])) {
          continue;
        }
        float weighted_sum = 0.0F;
        float weight_sum = 0.0F;
        float variance_sum = 0.0F;
        std::uint32_t support = 0U;
        for (int delta_row = -1; delta_row <= 1; ++delta_row) {
          const int next_row = static_cast<int>(row) + delta_row;
          if (next_row < 0 || next_row >= static_cast<int>(rows)) {
            continue;
          }
          for (int delta_column = -1; delta_column <= 1; ++delta_column) {
            const int next_column = static_cast<int>(column) + delta_column;
            if (next_column < 0 || next_column >= static_cast<int>(columns) ||
                (delta_row == 0 && delta_column == 0)) {
              continue;
            }
            const auto neighbour_index = indexOf(
                static_cast<std::uint32_t>(next_row),
                static_cast<std::uint32_t>(next_column), columns);
            if (!finite(source[neighbour_index])) {
              continue;
            }
            const float distance_squared = static_cast<float>(
                delta_row * delta_row + delta_column * delta_column);
            const float weight = 1.0F / (1.0F + distance_squared);
            weighted_sum += weight * source[neighbour_index];
            weight_sum += weight;
            variance_sum += source_variance[neighbour_index];
            ++support;
          }
        }
        if (weight_sum > 1.0e-6F) {
          destination[index] = weighted_sum / weight_sum;
          destination_variance[index] = variance_sum / static_cast<float>(support);
          filled_any = true;
        }
      }
    }
    source.swap(destination);
    source_variance.swap(destination_variance);
    if (!filled_any) {
      break;
    }
  }

  std::vector<float> finite_values;
  finite_values.reserve(source.size());
  for (const float value : source) {
    if (finite(value)) {
      finite_values.push_back(value);
    }
  }
  float fallback_value = fully_unknown_value;
  if (!finite_values.empty()) {
    const auto middle = finite_values.begin() +
                        static_cast<std::ptrdiff_t>(finite_values.size() / 2U);
    std::nth_element(finite_values.begin(), middle, finite_values.end());
    fallback_value = *middle;
  }
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (!finite(source[index])) {
      source[index] = fallback_value;
      source_variance[index] = fallback_variance;
    }
  }
  height.swap(source);
  variance.swap(source_variance);
}

}  // namespace

struct ElevationMap::Impl {
  explicit Impl(ElevationMap &node)
      : node(node), tf_buffer(node.get_clock()), tf_listener(tf_buffer) {
    loadParameters();
    createIo();
  }

  void loadParameters() {
    input_topic = node.declare_parameter<std::string>(
        "input_topic", "/lio/elevation_cloud");
    output_topic = node.declare_parameter<std::string>(
        "output_topic", "/autonomy_light/elevation_map/raw_input");
    output_frame = node.declare_parameter<std::string>("output_frame", "base_link_gravity");
    target_link = node.declare_parameter<std::string>("output.target_link", "base_link");
    input_height_filter_enabled = node.declare_parameter<bool>(
        "input_filter.target_relative_z.enabled", false);
    input_max_relative_z = node.declare_parameter<double>(
        "input_filter.target_relative_z.max", -0.20);
    publish_rate_hz = node.declare_parameter<double>("publish_rate_hz", 50.0);
    map_frame = node.declare_parameter<std::string>("slam.map_frame", "map");
    temporal_enabled = node.declare_parameter<bool>("algorithm.temporal.enabled", true);
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
    output_clip_min = node.declare_parameter<double>("output.clip_min", 0.0);
    output_clip_max = node.declare_parameter<double>("output.clip_max", grid.z_max);
    unobserved_value = node.declare_parameter<double>("output.unobserved_value", 0.0);
    noise_alpha = node.declare_parameter<double>("algorithm.uncertainty.noise_alpha", 0.001);
    minimum_measurement_variance = node.declare_parameter<double>(
        "algorithm.uncertainty.min_meas_var", 0.0004);
    maximum_cell_variance = node.declare_parameter<double>(
        "algorithm.uncertainty.max_cell_var", 0.01);
    upper_support_gap = node.declare_parameter<double>(
        "algorithm.frame_aggregation.intra_cell_max_support_gap", 0.025);
    upper_support_count = node.declare_parameter<int>(
        "algorithm.frame_aggregation.intra_cell_max_support_count", 2);
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
    single_frame_reference_enabled = node.declare_parameter<bool>(
        "algorithm.single_frame.reference.enabled", true);
    single_frame_reference_isolated_every = node.declare_parameter<int>(
        "algorithm.single_frame.reference.isolated_filter_every_n_frames", 2);
    single_frame_reference_bilateral_every = node.declare_parameter<int>(
        "algorithm.single_frame.reference.bilateral_every_n_frames", 2);
    single_frame_reference_min_runtime_fps = node.declare_parameter<int>(
        "algorithm.single_frame.reference.minimum_runtime_fps", 60);
    debug_enabled = node.declare_parameter<bool>("debug.enabled", false);
    diagnostics_enabled = node.declare_parameter<bool>("diagnostics.enabled", false);
    debug_pointcloud_topic = node.declare_parameter<std::string>(
        "debug.pointcloud_topic", "/autonomy_light/elevation_map/raw_input/debug_points");
    debug_input_cloud_enabled = node.declare_parameter<bool>(
        "debug.input_cloud.enabled", false);
    debug_input_cloud_topic = node.declare_parameter<std::string>(
        "debug.input_cloud.topic", "/autonomy_light/elevation_map/input_points");
    debug_input_x_min = node.declare_parameter<double>("debug.input_cloud.x_min", -3.0);
    debug_input_x_max = node.declare_parameter<double>("debug.input_cloud.x_max", 3.0);
    debug_input_y_min = node.declare_parameter<double>("debug.input_cloud.y_min", -3.0);
    debug_input_y_max = node.declare_parameter<double>("debug.input_cloud.y_max", 3.0);
    debug_input_z_min = node.declare_parameter<double>("debug.input_cloud.z_min", -3.0);
    debug_input_z_max = node.declare_parameter<double>("debug.input_cloud.z_max", 3.0);
    debug_input_surface_enabled = node.declare_parameter<bool>(
        "debug.input_surface.enabled", false);
    debug_input_surface_points_topic = node.declare_parameter<std::string>(
        "debug.input_surface.points_topic",
        "/autonomy_light/elevation_map/input_surface_points");
    debug_input_surface_mesh_topic = node.declare_parameter<std::string>(
        "debug.input_surface.mesh_topic",
        "/autonomy_light/elevation_map/input_surface_mesh");
    debug_input_surface_resolution = node.declare_parameter<double>(
        "debug.input_surface.resolution", 0.10);
    debug_input_surface_min_support = node.declare_parameter<int>(
        "debug.input_surface.min_points_per_cell", 1);
    debug_input_surface_cluster_gap = node.declare_parameter<double>(
        "debug.input_surface.highest_cluster_gap", 0.08);
    debug_input_surface_hole_radius = node.declare_parameter<int>(
        "debug.input_surface.hole_fill_radius", 1);
    debug_input_surface_hole_min_neighbours = node.declare_parameter<int>(
        "debug.input_surface.hole_fill_min_neighbors", 3);
    debug_input_surface_hole_max_difference = node.declare_parameter<double>(
        "debug.input_surface.hole_fill_max_height_difference", 0.12);
    debug_input_surface_smoothing_enabled = node.declare_parameter<bool>(
        "debug.input_surface.smoothing_enabled", true);
    debug_input_surface_mesh_max_difference = node.declare_parameter<double>(
        "debug.input_surface.mesh_max_height_difference", 0.20);
    if (input_topic.empty() || output_topic.empty() || output_frame.empty() || target_link.empty() ||
        map_frame.empty() ||
        publish_rate_hz <= 0.0 || grid.resolution <= 0.0 ||
        grid.x_max <= grid.x_min || grid.y_max <= grid.y_min ||
        grid.z_max <= grid.z_min || grid.width() == 0 || grid.height() == 0 ||
        minimum_measurement_variance <= 0.0 || maximum_cell_variance < minimum_measurement_variance ||
        upper_support_gap <= 0.0 || upper_support_count <= 0 ||
        smooth_blend < 0.0 || smooth_blend > 1.0 ||
        single_frame_reference_isolated_every <= 0 || single_frame_reference_bilateral_every <= 0 ||
        single_frame_reference_min_runtime_fps <= 0 ||
        temporal_tf_timeout_sec < 0.0 || temporal_variance_rate < 0.0 ||
        temporal_mahalanobis_threshold <= 0.0 || temporal_dynamic_reset_delta < 0.0 ||
        temporal_dynamic_variance_bump < 0.0 || !std::isfinite(output_clip_min) ||
        !std::isfinite(output_clip_max) || !std::isfinite(unobserved_value) ||
        !std::isfinite(input_max_relative_z) ||
        (debug_enabled && debug_pointcloud_topic.empty()) ||
        (debug_input_cloud_enabled &&
         (debug_input_cloud_topic.empty() || debug_input_x_max <= debug_input_x_min ||
          debug_input_y_max <= debug_input_y_min || debug_input_z_max <= debug_input_z_min)) ||
        (debug_input_surface_enabled &&
         (debug_input_surface_points_topic.empty() || debug_input_surface_mesh_topic.empty() ||
          debug_input_surface_resolution <= 0.0 || debug_input_surface_min_support <= 0 ||
          debug_input_surface_cluster_gap <= 0.0 || debug_input_surface_hole_radius < 0 ||
          debug_input_surface_hole_min_neighbours <= 0 ||
          debug_input_surface_hole_max_difference <= 0.0 ||
          debug_input_surface_mesh_max_difference <= 0.0))) {
      throw std::invalid_argument("elevation_mapping parameters are invalid");
    }
    if (output_clip_min > output_clip_max) {
      std::swap(output_clip_min, output_clip_max);
    }
    if (unobserved_value < output_clip_min || unobserved_value > output_clip_max) {
      throw std::invalid_argument(
          "output.unobserved_value must be inside output.clip_min/output.clip_max");
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
    output = node.create_publisher<autonomy_light::msg::HeightMap>(
        output_topic, rclcpp::QoS(1).reliable());
    if (debug_enabled) {
      debug_pointcloud = node.create_publisher<sensor_msgs::msg::PointCloud2>(
          debug_pointcloud_topic, rclcpp::QoS(1).reliable());
    }
    if (debug_input_cloud_enabled) {
      debug_input_cloud = node.create_publisher<sensor_msgs::msg::PointCloud2>(
          debug_input_cloud_topic, rclcpp::QoS(1).reliable());
    }
    if (debug_input_surface_enabled) {
      debug_input_surface_points = node.create_publisher<sensor_msgs::msg::PointCloud2>(
          debug_input_surface_points_topic, rclcpp::QoS(1).reliable());
      debug_input_surface_mesh = node.create_publisher<visualization_msgs::msg::Marker>(
          debug_input_surface_mesh_topic, rclcpp::QoS(1).reliable());
    }
    if (diagnostics_enabled) {
      heartbeat = node.create_publisher<std_msgs::msg::String>(
          "/autonomy_light/heartbeat/elevation_mapping", rclcpp::QoS(10));
      processing_time = node.create_publisher<std_msgs::msg::Float32>(
          "/autonomy_light/elevation_processing_ms", rclcpp::SensorDataQoS());
    }
    const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer = node.create_wall_timer(
        publish_period, [this]() { publishLatestMap(); }, output_callback_group);
    if (diagnostics_enabled) {
      heartbeat_timer = node.create_wall_timer(
          std::chrono::milliseconds(500), [this]() { publishHeartbeat(); }, output_callback_group);
    }
    RCLCPP_INFO(node.get_logger(),
                "Robot-attached yaw-aligned Super-LIO elevation sampling: %s around %s -> %s "
                "(non-negative vertical target-to-surface distance) @ %.1f Hz, %ux%u @ %.3fm, "
                "%s edge-aware filtering (SLAM-reprojected temporal fusion: %s, "
                "reference single-frame path: %s)",
                input_topic.c_str(), target_link.c_str(), output_topic.c_str(), publish_rate_hz,
                grid.width(), grid.height(),
                grid.resolution, temporal_enabled ? "temporal" : "single-frame",
                temporal_enabled ? "enabled" : "disabled",
                (!temporal_enabled && single_frame_reference_enabled) ? "enabled" : "disabled");
    if (debug_enabled) {
      RCLCPP_INFO(node.get_logger(), "Elevation debug PointCloud2: %s",
                  debug_pointcloud_topic.c_str());
    }
    if (debug_input_cloud_enabled) {
      RCLCPP_INFO(node.get_logger(),
                  "Elevation input debug PointCloud2: %s in %s, "
                  "x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f] m",
                  debug_input_cloud_topic.c_str(), output_frame.c_str(),
                  debug_input_x_min, debug_input_x_max,
                  debug_input_y_min, debug_input_y_max,
                  debug_input_z_min, debug_input_z_max);
    }
    if (input_height_filter_enabled) {
      RCLCPP_INFO(node.get_logger(),
                  "Elevation input z filter: retain points below %s by more than %.3f m "
                  "(target-relative z < %.3f m)",
                  target_link.c_str(), -input_max_relative_z, input_max_relative_z);
    }
    if (debug_input_surface_enabled) {
      RCLCPP_INFO(node.get_logger(),
                  "Elevation input surface: %s + %s, %.3f m cells, mesh edge gate %.3f m",
                  debug_input_surface_points_topic.c_str(),
                  debug_input_surface_mesh_topic.c_str(),
                  debug_input_surface_resolution,
                  debug_input_surface_mesh_max_difference);
    }
  }

  bool filterInputCloudByTargetHeight(sensor_msgs::msg::PointCloud2 &cloud,
                                      const FieldOffsets &offsets) {
    if (!input_height_filter_enabled) {
      return true;
    }
    double target_z = 0.0;
    try {
      const auto transform = tf_buffer.lookupTransform(
          output_frame, target_link, cloud.header.stamp,
          rclcpp::Duration::from_seconds(temporal_tf_timeout_sec));
      target_z = transform.transform.translation.z;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for input-height-filter TF %s -> %s: %s",
                           output_frame.c_str(), target_link.c_str(), error.what());
      return false;
    }

    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    std::size_t retained = 0U;
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const auto *point = cloud.data.data() + point_index * cloud.point_step;
      const float z = readFloat(point, offsets.z);
      if (!finite(z) ||
          static_cast<double>(z) - target_z >= input_max_relative_z) {
        continue;
      }
      if (retained != point_index) {
        std::memmove(cloud.data.data() + retained * cloud.point_step,
                     point, cloud.point_step);
      }
      ++retained;
    }
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(retained);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
    cloud.is_dense = true;
    RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                         "Input target-relative z filter: retained=%zu/%zu (z < %.3f m)",
                         retained, point_count, input_max_relative_z);
    return true;
  }

  void publishInputDebugCloud(const sensor_msgs::msg::PointCloud2 &cloud,
                              const FieldOffsets &offsets) {
    if (!debug_input_cloud) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message = cloud;
    message.height = 1U;
    message.width = 0U;
    message.row_step = 0U;
    message.data.clear();
    message.data.reserve(cloud.data.size());
    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const auto *point = cloud.data.data() + point_index * cloud.point_step;
      const float x = readFloat(point, offsets.x);
      const float y = readFloat(point, offsets.y);
      const float z = readFloat(point, offsets.z);
      if (!finite(x) || !finite(y) || !finite(z) ||
          x < debug_input_x_min || x > debug_input_x_max ||
          y < debug_input_y_min || y > debug_input_y_max ||
          z < debug_input_z_min || z > debug_input_z_max) {
        continue;
      }
      const auto previous_size = message.data.size();
      message.data.resize(previous_size + cloud.point_step);
      std::memcpy(message.data.data() + previous_size, point, cloud.point_step);
      ++message.width;
    }
    message.row_step = message.point_step * message.width;
    message.is_dense = true;
    debug_input_cloud->publish(std::move(message));
  }

  void publishInputSurface(const sensor_msgs::msg::PointCloud2 &cloud,
                           const FieldOffsets &offsets) {
    if (!debug_input_surface_points || !debug_input_surface_mesh) {
      return;
    }
    const auto columns = static_cast<std::uint32_t>(std::ceil(
        (debug_input_x_max - debug_input_x_min) / debug_input_surface_resolution));
    const auto rows = static_cast<std::uint32_t>(std::ceil(
        (debug_input_y_max - debug_input_y_min) / debug_input_surface_resolution));
    const auto cells = static_cast<std::size_t>(rows) * columns;
    std::vector<std::vector<float>> samples(cells);
    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const auto *point = cloud.data.data() + point_index * cloud.point_step;
      const float x = readFloat(point, offsets.x);
      const float y = readFloat(point, offsets.y);
      const float z = readFloat(point, offsets.z);
      if (!finite(x) || !finite(y) || !finite(z) ||
          x < debug_input_x_min || x >= debug_input_x_max ||
          y < debug_input_y_min || y >= debug_input_y_max ||
          z < debug_input_z_min || z > debug_input_z_max) {
        continue;
      }
      const auto column = static_cast<std::uint32_t>(
          (x - debug_input_x_min) / debug_input_surface_resolution);
      const auto row = static_cast<std::uint32_t>(
          (y - debug_input_y_min) / debug_input_surface_resolution);
      if (column < columns && row < rows) {
        samples[indexOf(row, column, columns)].push_back(z);
      }
    }

    std::vector<float> surface(cells, kNan);
    std::vector<float> variance(cells, static_cast<float>(maximum_cell_variance));
    std::vector<std::uint8_t> filled(cells, 0U);
    for (std::size_t index = 0; index < cells; ++index) {
      auto &cell = samples[index];
      if (cell.empty()) {
        continue;
      }
      float selected_z = kNan;
      if (selectHighestSupportedCluster(
              cell, static_cast<float>(debug_input_surface_cluster_gap),
              static_cast<std::uint32_t>(debug_input_surface_min_support), selected_z)) {
        surface[index] = selected_z;
        variance[index] = static_cast<float>(minimum_measurement_variance);
      }
    }
    fillSmallHolesEdgeAware(
        surface, variance, filled, rows, columns,
        debug_input_surface_hole_radius,
        debug_input_surface_hole_min_neighbours,
        static_cast<float>(debug_input_surface_hole_max_difference),
        static_cast<float>(maximum_cell_variance));
    if (debug_input_surface_smoothing_enabled) {
      smoothObservedSurfacesEdgeAware(
          surface, variance, rows, columns, 1, 1.0F, 0.08F,
          static_cast<float>(debug_input_surface_mesh_max_difference),
          2, 0.35F, 1);
    }

    const auto pointAt = [this, columns](const std::uint32_t row,
                                        const std::uint32_t column,
                                        const std::vector<float> &values) {
      PointXYZIntensity point;
      point.x = static_cast<float>(debug_input_x_min +
          (static_cast<double>(column) + 0.5) * debug_input_surface_resolution);
      point.y = static_cast<float>(debug_input_y_min +
          (static_cast<double>(row) + 0.5) * debug_input_surface_resolution);
      point.z = values[indexOf(row, column, columns)];
      point.intensity = point.z;
      return point;
    };

    std::vector<PointXYZIntensity> points;
    points.reserve(cells);
    float minimum_z = std::numeric_limits<float>::infinity();
    float maximum_z = -std::numeric_limits<float>::infinity();
    for (std::uint32_t row = 0; row < rows; ++row) {
      for (std::uint32_t column = 0; column < columns; ++column) {
        const float z = surface[indexOf(row, column, columns)];
        if (!finite(z)) {
          continue;
        }
        points.push_back(pointAt(row, column, surface));
        minimum_z = std::min(minimum_z, z);
        maximum_z = std::max(maximum_z, z);
      }
    }

    sensor_msgs::msg::PointCloud2 surface_cloud;
    surface_cloud.header = cloud.header;
    surface_cloud.height = 1U;
    surface_cloud.width = static_cast<std::uint32_t>(points.size());
    surface_cloud.is_bigendian = false;
    surface_cloud.is_dense = true;
    surface_cloud.point_step = sizeof(PointXYZIntensity);
    surface_cloud.row_step = surface_cloud.point_step * surface_cloud.width;
    surface_cloud.fields.resize(4U);
    const std::array<const char *, 4> names{"x", "y", "z", "intensity"};
    for (std::size_t index = 0; index < names.size(); ++index) {
      surface_cloud.fields[index].name = names[index];
      surface_cloud.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
      surface_cloud.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
      surface_cloud.fields[index].count = 1U;
    }
    surface_cloud.data.resize(static_cast<std::size_t>(surface_cloud.row_step));
    if (!points.empty()) {
      std::memcpy(surface_cloud.data.data(), points.data(), surface_cloud.data.size());
    }
    debug_input_surface_points->publish(std::move(surface_cloud));

    visualization_msgs::msg::Marker marker;
    marker.header = cloud.header;
    marker.ns = "elevation_input_surface";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0F;
    const auto addVertex = [&marker, minimum_z, maximum_z](const PointXYZIntensity &source) {
      geometry_msgs::msg::Point point;
      point.x = source.x;
      point.y = source.y;
      point.z = source.z;
      marker.points.push_back(point);
      const float span = std::max(0.05F, maximum_z - minimum_z);
      const float normalized = std::clamp((source.z - minimum_z) / span, 0.0F, 1.0F);
      std_msgs::msg::ColorRGBA color;
      color.r = normalized;
      color.g = 0.25F + 0.65F * (1.0F - std::fabs(2.0F * normalized - 1.0F));
      color.b = 1.0F - normalized;
      color.a = 0.88F;
      marker.colors.push_back(color);
    };
    const auto addTriangle = [&addVertex](const PointXYZIntensity &a,
                                          const PointXYZIntensity &b,
                                          const PointXYZIntensity &c) {
      addVertex(a);
      addVertex(b);
      addVertex(c);
    };
    for (std::uint32_t row = 0; row + 1U < rows; ++row) {
      for (std::uint32_t column = 0; column + 1U < columns; ++column) {
        const auto a = pointAt(row, column, surface);
        const auto b = pointAt(row, column + 1U, surface);
        const auto c = pointAt(row + 1U, column + 1U, surface);
        const auto d = pointAt(row + 1U, column, surface);
        if (!finite(a.z) || !finite(b.z) || !finite(c.z) || !finite(d.z)) {
          continue;
        }
        const float minimum = std::min({a.z, b.z, c.z, d.z});
        const float maximum = std::max({a.z, b.z, c.z, d.z});
        if (maximum - minimum > static_cast<float>(debug_input_surface_mesh_max_difference)) {
          continue;
        }
        addTriangle(a, b, c);
        addTriangle(a, c, d);
      }
    }
    if (marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    debug_input_surface_mesh->publish(std::move(marker));
  }

  void publishDebugCloud(const std::vector<float> &heights,
                         const rclcpp::Time &stamp,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
                         const std::string &frame_id = "",
                         const double x_offset = 0.0,
                         const double y_offset = 0.0,
                         const double target_z = 0.0,
                         const bool values_are_distances = false) {
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
        const float value = heights[indexOf(row, column, width)];
        if (!finite(value)) {
          continue;
        }
        const float x = static_cast<float>(grid.x_min +
            (static_cast<double>(column) + 0.5) * grid.resolution);
        const float debug_z = values_are_distances
            ? static_cast<float>(target_z) - value
            : value;
        points.push_back({x + static_cast<float>(x_offset),
                          y + static_cast<float>(y_offset),
                          debug_z, value});
      }
    }
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id.empty() ? output_frame : frame_id;
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
    if (!heartbeat) {
      return;
    }
    std_msgs::msg::String message;
    message.data = frame_count.load() == 0 ? "waiting_for_observation" : "ready";
    heartbeat->publish(message);
  }

  void publishLatestMap() {
    std::shared_ptr<autonomy_light::msg::HeightMap> map;
    {
      std::lock_guard<std::mutex> lock(latest_map_mutex);
      map = latest_map;
    }
    if (map) {
      output->publish(*map);
    }
  }

  void onSingleFrameReference(const sensor_msgs::msg::PointCloud2 &cloud,
                              const FieldOffsets &offsets,
                              const std::chrono::steady_clock::time_point processing_started) {
    const auto width = grid.width();
    const auto height = grid.height();
    const auto cells = static_cast<std::size_t>(width) * height;
    const rclcpp::Time stamp(cloud.header.stamp);
    double target_z = 0.0;
    double target_x = 0.0;
    double target_y = 0.0;
    try {
      const auto target_transform = tf_buffer.lookupTransform(
          output_frame, target_link, stamp,
          rclcpp::Duration::from_seconds(temporal_tf_timeout_sec));
      target_x = target_transform.transform.translation.x;
      target_y = target_transform.transform.translation.y;
      target_z = target_transform.transform.translation.z;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for target-link TF %s -> %s: %s",
                           output_frame.c_str(), target_link.c_str(), error.what());
      return;
    }
    // The input is already a current robot-neighbourhood sample of Super-LIO's
    // accumulated global map.  Do not retain robot-relative cells across
    // callbacks: after the robot moves, the same grid index refers to a
    // different global location.
    reference_elevation.assign(cells, kNan);
    reference_variance.assign(cells, static_cast<float>(maximum_cell_variance));
    reference_stamp = stamp;
    reference_have_stamp = true;

    std::vector<FrameCell> frame(cells);
    std::size_t accepted = 0U;
    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const auto *point = cloud.data.data() + point_index * cloud.point_step;
      const float x = readFloat(point, offsets.x);
      const float y = readFloat(point, offsets.y);
      const float z = readFloat(point, offsets.z);
      const float relative_x = x - static_cast<float>(target_x);
      const float relative_y = y - static_cast<float>(target_y);
      const float relative_z = z - static_cast<float>(target_z);
      if (!finite(x) || !finite(y) || !finite(z) ||
          relative_x < grid.x_min || relative_x >= grid.x_max ||
          relative_y < grid.y_min || relative_y >= grid.y_max ||
          relative_z < grid.z_min || relative_z > grid.z_max) {
        continue;
      }
      const auto column = static_cast<std::uint32_t>(
          (relative_x - grid.x_min) / grid.resolution);
      const auto row = static_cast<std::uint32_t>(
          (relative_y - grid.y_min) / grid.resolution);
      if (column >= width || row >= height) {
        continue;
      }
      const auto cell_index = indexOf(row, column, width);
      auto &cell = frame[cell_index];
      const float range_squared = relative_x * relative_x + relative_y * relative_y +
          relative_z * relative_z;
      const float measurement_variance = std::max(
          static_cast<float>(minimum_measurement_variance),
          static_cast<float>(noise_alpha) * range_squared);
      const float weight = 1.0F / std::max(measurement_variance, 1.0e-6F);
      ++cell.hits;
      cell.samples.push_back(HeightSample{relative_z, weight, measurement_variance});
      ++accepted;
    }

    std::size_t raw_observed = 0U;
    std::vector<std::uint8_t> observed_in_crop(cells, 0U);
    for (std::size_t cell_index = 0; cell_index < cells; ++cell_index) {
      const auto &cell = frame[cell_index];
      if (cell.hits == 0U) {
        continue;
      }
      float surface_z = kNan;
      float measurement_variance = static_cast<float>(maximum_cell_variance);
      if (!selectHighestSupportedCluster(
              cell.samples, static_cast<float>(upper_support_gap),
              static_cast<std::uint32_t>(upper_support_count), surface_z,
              measurement_variance)) {
        continue;
      }
      ++raw_observed;
      observed_in_crop[cell_index] = 1U;
      // In the gravity-aligned frame this is exactly
      // target_world_z - surface_world_z, matching Isaac Lab's height scanner.
      // The selected upper supported surface below base_link therefore produces
      // a positive value.
      const float measurement = -surface_z;
      updateReferenceCell(
          measurement, measurement_variance, reference_elevation[cell_index],
          reference_variance[cell_index], static_cast<float>(
              temporal_mahalanobis_threshold * temporal_mahalanobis_threshold),
          static_cast<float>(temporal_dynamic_reset_delta),
          static_cast<float>(temporal_dynamic_variance_bump),
          static_cast<float>(minimum_measurement_variance),
          static_cast<float>(maximum_cell_variance));
    }

    std::vector<std::uint8_t> filter_filled(cells, 0U);
    fillSmallHolesEdgeAware(reference_elevation, reference_variance, filter_filled,
                            height, width, hole_radius, hole_min_neighbours,
                            static_cast<float>(hole_max_difference),
                            static_cast<float>(maximum_cell_variance));
    ++reference_frame_sequence;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - processing_started).count();
    const double frame_budget_ms = 1000.0 /
        static_cast<double>(single_frame_reference_min_runtime_fps);
    if (elapsed_ms < 0.90 * frame_budget_ms &&
        reference_frame_sequence % static_cast<std::uint64_t>(
            single_frame_reference_isolated_every) == 0U) {
      removeIsolatedCellsEdgeAware(
          reference_elevation, reference_variance, height, width, isolated_radius,
          isolated_min_support, static_cast<float>(isolated_support_difference),
          static_cast<float>(isolated_outlier_difference),
          static_cast<float>(maximum_cell_variance));
    }
    if (elapsed_ms < 0.90 * frame_budget_ms &&
        reference_frame_sequence % static_cast<std::uint64_t>(
            single_frame_reference_bilateral_every) == 0U) {
      fillReferenceBilateralHoles(reference_elevation, height, width, smooth_radius,
                                  static_cast<float>(smooth_sigma_spatial),
                                  static_cast<float>(smooth_sigma_height),
                                  static_cast<float>(smooth_max_difference), smooth_passes);
    }
    // The external HeightMap has no validity mask.  Do not silently convert a
    // missing global-map cell into terrain copied from a neighbouring or old
    // robot-relative cell: output the configured finite sentinel instead.
    std::vector<float> output_elevation(cells, static_cast<float>(unobserved_value));
    for (std::size_t cell_index = 0; cell_index < cells; ++cell_index) {
      if (observed_in_crop[cell_index] == 0U || !finite(reference_elevation[cell_index])) {
        continue;
      }
      output_elevation[cell_index] = std::clamp(
          reference_elevation[cell_index], static_cast<float>(output_clip_min),
          static_cast<float>(output_clip_max));
    }
    if (debug_enabled) {
      publishDebugCloud(output_elevation, stamp, debug_pointcloud, output_frame,
                        target_x, target_y, target_z, true);
    }
    autonomy_light::msg::HeightMap message;
    message.data = std::move(output_elevation);
    message.resolution = static_cast<float>(grid.resolution);
    message.x_length = static_cast<float>(static_cast<double>(width) * grid.resolution);
    message.y_length = static_cast<float>(static_cast<double>(height) * grid.resolution);
    {
      std::lock_guard<std::mutex> lock(latest_map_mutex);
      latest_map = std::make_shared<autonomy_light::msg::HeightMap>(std::move(message));
    }
    if (processing_time) {
      std_msgs::msg::Float32 processing_message;
      processing_message.data = static_cast<float>(std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - processing_started).count());
      processing_time->publish(processing_message);
    }
    ++frame_count;
    RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                         "Reference single-frame: input=%zu ROI=%zu observed=%zu unobserved=%zu/%zu",
                         point_count, accepted, raw_observed, cells - raw_observed, cells);
  }

  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    const auto processing_started = std::chrono::steady_clock::now();
    if (!cloud) {
      return;
    }
    FieldOffsets offsets;
    const auto expected_bytes = static_cast<std::size_t>(cloud->point_step) * cloud->width * cloud->height;
    if (!fields(*cloud, offsets) || cloud->point_step == 0 || cloud->data.size() < expected_bytes) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Super-LIO elevation crop is not an XYZ PointCloud2");
      return;
    }
    if (cloud->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Super-LIO elevation crop has no frame_id");
      return;
    }
    if (cloud->header.frame_id != output_frame) {
      tf2::Transform output_from_input;
      try {
        const auto transform = tf_buffer.lookupTransform(
            output_frame, cloud->header.frame_id, cloud->header.stamp,
            rclcpp::Duration::from_seconds(temporal_tf_timeout_sec));
        tf2::fromMsg(transform.transform, output_from_input);
      } catch (const tf2::TransformException &error) {
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                             "Waiting for elevation crop TF %s -> %s: %s",
                             cloud->header.frame_id.c_str(), output_frame.c_str(), error.what());
        return;
      }
      const auto point_count = static_cast<std::size_t>(cloud->width) * cloud->height;
      for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
        auto *point = cloud->data.data() + point_index * cloud->point_step;
        const float x = readFloat(point, offsets.x);
        const float y = readFloat(point, offsets.y);
        const float z = readFloat(point, offsets.z);
        if (!finite(x) || !finite(y) || !finite(z)) {
          continue;
        }
        const auto transformed = output_from_input * tf2::Vector3(x, y, z);
        const float transformed_x = static_cast<float>(transformed.x());
        const float transformed_y = static_cast<float>(transformed.y());
        const float transformed_z = static_cast<float>(transformed.z());
        std::memcpy(point + offsets.x, &transformed_x, sizeof(transformed_x));
        std::memcpy(point + offsets.y, &transformed_y, sizeof(transformed_y));
        std::memcpy(point + offsets.z, &transformed_z, sizeof(transformed_z));
      }
      cloud->header.frame_id = output_frame;
    }
    if (!filterInputCloudByTargetHeight(*cloud, offsets)) {
      return;
    }
    // Publish the map samples after the map -> robot gravity-frame transform,
    // and target-relative z filter, but before the small HeightMap ROI, cell
    // aggregation, or output-height clipping.
    publishInputDebugCloud(*cloud, offsets);
    publishInputSurface(*cloud, offsets);
    if (!temporal_enabled && single_frame_reference_enabled) {
      onSingleFrameReference(*cloud, offsets, processing_started);
      return;
    }
    const auto width = grid.width();
    const auto height = grid.height();
    const auto cells = static_cast<std::size_t>(width) * height;
    const rclcpp::Time cloud_stamp(cloud->header.stamp);
    const bool temporal_ready = prepareTemporalPrior(cloud_stamp, width, height);
    if (temporal_enabled && !temporal_ready) {
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
      cell.samples.push_back(HeightSample{z, weight, measurement_variance});
      ++accepted;
    }

    std::vector<float> elevation(cells, kNan);
    std::vector<float> variance(cells, static_cast<float>(maximum_cell_variance));
    for (std::size_t cell_index = 0; cell_index < cells; ++cell_index) {
      const auto &cell = frame[cell_index];
      if (cell.hits == 0U) {
        continue;
      }
      float value = kNan;
      float value_variance = static_cast<float>(maximum_cell_variance);
      if (!selectHighestSupportedCluster(
              cell.samples, static_cast<float>(upper_support_gap),
              static_cast<std::uint32_t>(upper_support_count), value,
              value_variance)) {
        continue;
      }
      elevation[cell_index] = value;
      variance[cell_index] = std::clamp(value_variance,
                                        static_cast<float>(minimum_measurement_variance),
                                        static_cast<float>(maximum_cell_variance));
    }

    auto raw_elevation = elevation;
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
    double target_z = 0.0;
    try {
      const auto target_transform = tf_buffer.lookupTransform(
          output_frame, target_link, cloud_stamp,
          rclcpp::Duration::from_seconds(temporal_tf_timeout_sec));
      target_z = target_transform.transform.translation.z;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for target-link TF %s -> %s: %s",
                           output_frame.c_str(), target_link.c_str(), error.what());
      return;
    }
    // Temporal state stays in gravity-frame z coordinates so it can be
    // reprojected geometrically on the next frame. The external HeightMap
    // convention is vertical distance below output.target_link.
    // Therefore, when target_link=base_link, z=-0.50m is emitted as +0.50m.
    if (temporal_enabled && temporal_ready) {
      commitTemporalState(cloud_stamp, elevation, variance);
    }
    for (auto &value : elevation) {
      if (finite(value)) {
        value = std::clamp(static_cast<float>(target_z) - value,
                           static_cast<float>(output_clip_min),
                           static_cast<float>(output_clip_max));
      }
    }

    // The internal filter deliberately preserves holes as NaN to avoid
    // smoothing across edges. Before publishing, the HeightMap contract is
    // contract is finite-only, so fill every remaining cell deterministically.
    fillRemainingHolesStrict(elevation, variance, height, width,
                             static_cast<float>(maximum_cell_variance),
                             static_cast<float>(output_clip_min));

    std::size_t valid = 0;
    std::size_t raw_observed = 0;
    std::size_t filled = 0;
    for (std::size_t cell_index = 0; cell_index < cells; ++cell_index) {
      if (finite(raw_elevation[cell_index])) {
        ++raw_observed;
      }
      if (!finite(elevation[cell_index])) {
        continue;
      }
      if (filter_filled[cell_index] != 0U) {
        ++filled;
      }
      ++valid;
    }
    if (debug_enabled) {
      publishDebugCloud(elevation, rclcpp::Time(cloud->header.stamp), debug_pointcloud);
    }

    autonomy_light::msg::HeightMap message;
    message.data = std::move(elevation);
    message.resolution = static_cast<float>(grid.resolution);
    message.x_length = static_cast<float>(static_cast<double>(width) * grid.resolution);
    message.y_length = static_cast<float>(static_cast<double>(height) * grid.resolution);
    {
      std::lock_guard<std::mutex> lock(latest_map_mutex);
      latest_map = std::make_shared<autonomy_light::msg::HeightMap>(std::move(message));
    }
    std_msgs::msg::Float32 processing_message;
    processing_message.data = static_cast<float>(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   processing_started).count());
    if (processing_time) {
      processing_time->publish(processing_message);
    }
    ++frame_count;
    RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                         "Precise frame: input=%zu ROI=%zu raw=%zu filter_fill=%zu valid=%zu/%zu (%.1f%%)",
                         point_count, accepted, raw_observed, filled, valid, cells,
                         100.0 * static_cast<double>(valid) / static_cast<double>(cells));
  }

  ElevationMap &node;
  std::string input_topic;
  std::string output_topic;
  std::string output_frame;
  std::string target_link;
  std::string map_frame;
  bool input_height_filter_enabled{false};
  double input_max_relative_z{-0.20};
  double publish_rate_hz{50.0};
  GridGeometry grid;
  double noise_alpha{0.001};
  double output_clip_min{0.0};
  double output_clip_max{0.80};
  double unobserved_value{0.0};
  double minimum_measurement_variance{0.0004};
  double maximum_cell_variance{0.01};
  double upper_support_gap{0.025};
  int upper_support_count{2};
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
  bool single_frame_reference_enabled{true};
  int single_frame_reference_isolated_every{2};
  int single_frame_reference_bilateral_every{2};
  int single_frame_reference_min_runtime_fps{60};
  std::uint64_t reference_frame_sequence{0};
  bool reference_have_stamp{false};
  rclcpp::Time reference_stamp{0, 0, RCL_ROS_TIME};
  std::vector<float> reference_elevation;
  std::vector<float> reference_variance;
  bool temporal_enabled{true};
  double temporal_tf_timeout_sec{0.02};
  double temporal_variance_rate{0.20};
  double temporal_mahalanobis_threshold{3.0};
  double temporal_dynamic_reset_delta{0.10};
  double temporal_dynamic_variance_bump{0.0225};
  bool debug_enabled{false};
  bool debug_input_cloud_enabled{false};
  bool debug_input_surface_enabled{false};
  bool diagnostics_enabled{false};
  std::string debug_pointcloud_topic;
  std::string debug_input_cloud_topic;
  std::string debug_input_surface_points_topic;
  std::string debug_input_surface_mesh_topic;
  double debug_input_x_min{-3.0};
  double debug_input_x_max{3.0};
  double debug_input_y_min{-3.0};
  double debug_input_y_max{3.0};
  double debug_input_z_min{-3.0};
  double debug_input_z_max{3.0};
  double debug_input_surface_resolution{0.10};
  int debug_input_surface_min_support{1};
  double debug_input_surface_cluster_gap{0.08};
  int debug_input_surface_hole_radius{1};
  int debug_input_surface_hole_min_neighbours{3};
  double debug_input_surface_hole_max_difference{0.12};
  bool debug_input_surface_smoothing_enabled{true};
  double debug_input_surface_mesh_max_difference{0.20};
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
  rclcpp::Publisher<autonomy_light::msg::HeightMap>::SharedPtr output;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_input_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_input_surface_points;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_input_surface_mesh;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time;
  rclcpp::CallbackGroup::SharedPtr input_callback_group;
  rclcpp::CallbackGroup::SharedPtr output_callback_group;
  std::mutex latest_map_mutex;
  std::shared_ptr<autonomy_light::msg::HeightMap> latest_map;
  rclcpp::TimerBase::SharedPtr publish_timer;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
};

ElevationMap::ElevationMap(const rclcpp::NodeOptions &options)
    : Node("elevation_mapping", options) {
  impl_ = std::make_unique<Impl>(*this);
}

ElevationMap::~ElevationMap() = default;

}  // namespace autonomy_light::algorithm::elevation
