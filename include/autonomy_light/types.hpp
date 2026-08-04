#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace autonomy_light {

using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

struct PointObservation {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t sensor_id{0U}; // 0: LiDAR, 1: D435/D435i
  // World-frame source position. NaN means the mapper uses the LiDAR/base
  // origin.
  float origin_x{std::numeric_limits<float>::quiet_NaN()};
  float origin_y{std::numeric_limits<float>::quiet_NaN()};
  float origin_z{std::numeric_limits<float>::quiet_NaN()};
};

using PointObservations = std::vector<PointObservation>;

struct GridSpec {
  double resolution{0.10};
  double x_length{1.80};
  double y_length{0.80};
  double min_z{-2.0};
  double max_z{2.0};

  [[nodiscard]] double xMin() const { return -0.5 * x_length; }
  [[nodiscard]] double yMin() const { return -0.5 * y_length; }
  [[nodiscard]] std::uint32_t width() const {
    return static_cast<std::uint32_t>(std::ceil(x_length / resolution));
  }
  [[nodiscard]] std::uint32_t height() const {
    return static_cast<std::uint32_t>(std::ceil(y_length / resolution));
  }
  [[nodiscard]] std::size_t size() const {
    return static_cast<std::size_t>(width()) * height();
  }
};

struct HeightGrid {
  std_msgs::msg::Header header;
  GridSpec spec;
  std::vector<float> height;
  std::vector<std::uint8_t> valid;
  std::vector<float> variance;
  std::vector<float> age;

  explicit HeightGrid(const GridSpec &grid = {}) : spec(grid) {
    height.assign(spec.size(), std::numeric_limits<float>::quiet_NaN());
    valid.assign(spec.size(), 0U);
    variance.assign(spec.size(), std::numeric_limits<float>::infinity());
    age.assign(spec.size(), std::numeric_limits<float>::infinity());
  }
};

struct Pose2_5D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  // nav_msgs/Odometry order: x, y, z, roll, pitch, yaw in the global frame.
  std::array<double, 36> covariance{};
  bool has_covariance{false};
};

struct CellKey {
  std::int64_t x{0};
  std::int64_t y{0};

  bool operator==(const CellKey &other) const {
    return x == other.x && y == other.y;
  }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey &key) const {
    return std::hash<std::int64_t>{}(key.x) ^
           (std::hash<std::int64_t>{}(key.y) << 1U);
  }
};

struct VoxelKey {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    const auto x = std::hash<std::int64_t>{}(key.x);
    const auto y = std::hash<std::int64_t>{}(key.y);
    return x ^ (y << 1U) ^ (std::hash<std::int64_t>{}(key.z) << 7U);
  }
};

struct VoxelMean {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  std::uint32_t count{0U};
};

inline std::string shortDouble(const double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

} // namespace autonomy_light
