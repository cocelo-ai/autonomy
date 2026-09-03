#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

namespace autonomy_light::contract {

// These names are on-wire contracts with the control SBC. HeightMap keeps
// the legacy raw-DDS v2 contract; CommandCore uses ROS 2 DDS mapping.
inline constexpr char kHeightMapDdsTopic[] = "height_map";
inline constexpr char kAutopilotCommandRosTopic[] = "/autonomy_light/autopilot_command";
inline constexpr char kAutopilotCommandDdsTopic[] = "rt/control_command/autopilot";

inline constexpr std::size_t kHeightMapHistoryDepth = 128U;
inline constexpr std::size_t kAutopilotCommandHistoryDepth = 1U;

/** Converts a ROS topic graph name to the DDS topic name used by ROS 2. */
inline std::string rosTopicToDdsTopic(const std::string_view ros_topic) {
  if (ros_topic.empty() || ros_topic.front() != '/') {
    return {};
  }
  return "rt" + std::string(ros_topic);
}

/**
 * Converts a metric map dimension to its integral cell count without silently
 * accepting a truncated grid.  The tolerance accommodates the binary
 * representation of values transported as `float`, not an arbitrary partial
 * cell at a map boundary.
 */
inline bool dimensionToCellCount(const float length, const float resolution,
                                 std::size_t *const cell_count) {
  if (cell_count == nullptr || !std::isfinite(length) || !std::isfinite(resolution) ||
      length <= 0.0F || resolution <= 0.0F) {
    return false;
  }

  const double exact_count = static_cast<double>(length) / static_cast<double>(resolution);
  const double rounded_count = std::round(exact_count);
  const double tolerance = 1.0e-5 * std::max(1.0, std::fabs(exact_count));
  if (!std::isfinite(exact_count) || rounded_count < 1.0 ||
      std::fabs(exact_count - rounded_count) > tolerance ||
      rounded_count > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    return false;
  }

  *cell_count = static_cast<std::size_t>(rounded_count);
  return true;
}

/**
 * Verifies the complete internal ROS HeightMap before forwarding its data
 * field through the legacy control-network contract.
 */
inline bool isValidHeightMap(const float *const data, const std::size_t count,
                             const float resolution, const float x_length,
                             const float y_length) {
  if (data == nullptr || count == 0U ||
      count > std::numeric_limits<std::uint32_t>::max()) {
    return false;
  }

  std::size_t columns = 0U;
  std::size_t rows = 0U;
  if (!dimensionToCellCount(x_length, resolution, &columns) ||
      !dimensionToCellCount(y_length, resolution, &rows) ||
      columns > std::numeric_limits<std::size_t>::max() / rows ||
      count != columns * rows) {
    return false;
  }
  for (std::size_t index = 0U; index < count; ++index) {
    if (!std::isfinite(data[index])) {
      return false;
    }
  }
  return true;
}

/** Validates the Nav2 velocity fields carried by core/msg/CommandCore. */
inline bool isValidAutopilotCommand(const double linear_x, const double linear_y,
                                    const double angular_z) {
  constexpr double kMaximumFloat = static_cast<double>(std::numeric_limits<float>::max());
  return std::isfinite(linear_x) && std::isfinite(linear_y) && std::isfinite(angular_z) &&
         std::fabs(linear_x) <= kMaximumFloat && std::fabs(linear_y) <= kMaximumFloat &&
         std::fabs(angular_z) <= kMaximumFloat;
}

}  // namespace autonomy_light::contract
