#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

std::int64_t cellKey(std::int32_t x, std::int32_t y) {
  return static_cast<std::int64_t>(
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
      static_cast<std::uint32_t>(y));
}

std::pair<std::int32_t, std::int32_t> decodeCell(std::int64_t key) {
  const auto bits = static_cast<std::uint64_t>(key);
  return {static_cast<std::int32_t>(bits >> 32U),
          static_cast<std::int32_t>(bits & 0xffffffffU)};
}

struct PointSample {
  double map_x;
  double map_y;
  double map_z;
  double range;
  std::array<float, 3> sensor;
};

struct GroundPlane {
  double nx{0.0};
  double ny{0.0};
  double nz{1.0};
  double d{0.0};
  std::size_t inliers{0U};
  bool valid{false};
};

}  // namespace

class LiveOccupancyMapper : public rclcpp::Node {
 public:
  LiveOccupancyMapper()
      : Node("live_occupancy_mapper"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lio/cloud_world");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    sensor_cloud_topic_ =
        declare_parameter<std::string>("sensor_cloud_topic", "/lio/cloud_sensor");
    obstacle_cloud_topic_ = declare_parameter<std::string>(
        "obstacle_cloud_topic", "/lio/cloud_sensor_obstacles");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "f4/lidar_link");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    publish_frequency_ = declare_parameter<double>("publish_frequency", 2.0);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", 0.08);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.20);
    ground_distance_threshold_ = declare_parameter<double>("ground_distance_threshold", 0.055);
    max_ground_tilt_deg_ = declare_parameter<double>("max_ground_tilt_deg", 35.0);
    min_ground_points_ = declare_parameter<int>("min_ground_points", 100);
    ground_ransac_iterations_ = declare_parameter<int>("ground_ransac_iterations", 100);
    min_range_ = declare_parameter<double>("min_range", 0.10);
    max_range_ = declare_parameter<double>("max_range", 12.0);
    padding_ = declare_parameter<double>("padding", 2.0);
    expansion_chunk_size_ = declare_parameter<double>("expansion_chunk_size", 5.0);
    occupied_increment_ = declare_parameter<int>("occupied_increment", 3);
    free_decrement_ = declare_parameter<int>("free_decrement", 1);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 2);
    min_score_ = declare_parameter<int>("min_score", -10);
    max_score_ = declare_parameter<int>("max_score", 20);
    max_cells_ = declare_parameter<int>("max_cells", 4000000);
    save_map_ = declare_parameter<bool>("save_map", false);
    save_map_directory_ = declare_parameter<std::string>("save_map_directory", "");
    save_map_name_ = declare_parameter<std::string>("save_map_name", "map");

    if (resolution_ <= 0.0 || publish_frequency_ <= 0.0 || max_range_ <= min_range_) {
      throw std::runtime_error("Invalid live occupancy mapper geometry/rate parameters");
    }
    if (save_map_ && (save_map_directory_.empty() || save_map_name_.empty())) {
      throw std::runtime_error("Occupancy-map save directory and name are required when save_map is enabled");
    }

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, map_qos);
    sensor_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        sensor_cloud_topic_, rclcpp::SensorDataQoS());
    obstacle_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        obstacle_cloud_topic_, rclcpp::SensorDataQoS());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LiveOccupancyMapper::onCloud, this, std::placeholders::_1));
    const auto timer_period = std::chrono::duration<double>(1.0 / publish_frequency_);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&LiveOccupancyMapper::publishMap, this));

    RCLCPP_INFO(get_logger(),
                "Dynamic occupancy mapper: %s -> %s, frame=%s, resolution=%.3fm, max_range=%.1fm",
                input_topic_.c_str(), map_topic_.c_str(), map_frame_.c_str(), resolution_,
                max_range_);
  }

  ~LiveOccupancyMapper() override {
    if (save_map_) {
      saveMap();
    }
  }

 private:
  using CellScore = std::int16_t;

  std::int32_t worldToCell(double value) const {
    return static_cast<std::int32_t>(std::floor(value / resolution_));
  }

  void updateBounds(std::int32_t x, std::int32_t y) {
    min_x_ = std::min(min_x_, x);
    max_x_ = std::max(max_x_, x);
    min_y_ = std::min(min_y_, y);
    max_y_ = std::max(max_y_, y);
  }

  void markFree(std::int32_t x, std::int32_t y) {
    auto &score = cells_[cellKey(x, y)];
    score = static_cast<CellScore>(std::max(min_score_, static_cast<int>(score) - free_decrement_));
    updateBounds(x, y);
  }

  void markOccupied(std::int32_t x, std::int32_t y) {
    auto &score = cells_[cellKey(x, y)];
    score = static_cast<CellScore>(
        std::min(max_score_, static_cast<int>(score) + occupied_increment_));
    updateBounds(x, y);
  }

  void raytraceFree(std::int32_t x0, std::int32_t y0, std::int32_t x1, std::int32_t y1) {
    const std::int32_t dx = std::abs(x1 - x0);
    const std::int32_t sx = x0 < x1 ? 1 : -1;
    const std::int32_t dy = -std::abs(y1 - y0);
    const std::int32_t sy = y0 < y1 ? 1 : -1;
    std::int32_t error = dx + dy;

    while (x0 != x1 || y0 != y1) {
      markFree(x0, y0);
      const std::int32_t twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x0 += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y0 += sy;
      }
    }
  }

  void publishSensorCloud(
      const std_msgs::msg::Header &source_header,
      const std::vector<std::array<float, 3>> &sensor_points,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher) {
    if (sensor_points.empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 result;
    result.header = source_header;
    result.header.frame_id = sensor_frame_;
    result.height = 1;
    result.width = static_cast<std::uint32_t>(sensor_points.size());
    result.is_bigendian = false;
    result.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(sensor_points.size());
    sensor_msgs::PointCloud2Iterator<float> x(result, "x");
    sensor_msgs::PointCloud2Iterator<float> y(result, "y");
    sensor_msgs::PointCloud2Iterator<float> z(result, "z");
    for (const auto &point : sensor_points) {
      *x = point[0];
      *y = point[1];
      *z = point[2];
      ++x;
      ++y;
      ++z;
    }
    publisher->publish(result);
  }

  GroundPlane fitGroundPlane(
      const std::vector<PointSample> &points, double origin_x, double origin_y,
      double origin_z) {
    std::vector<std::size_t> candidates;
    candidates.reserve(std::min<std::size_t>(points.size(), 4000U));
    const std::size_t stride = std::max<std::size_t>(1U, points.size() / 4000U);
    for (std::size_t index = 0; index < points.size(); index += stride) {
      const auto &point = points[index];
      const double relative_z = point.map_z - origin_z;
      if (point.range >= 0.5 && point.range <= 8.0 && relative_z >= -1.2 &&
          relative_z <= 0.20) {
        candidates.push_back(index);
      }
    }
    if (candidates.size() < static_cast<std::size_t>(min_ground_points_)) {
      return {};
    }

    const double minimum_normal_z =
        std::cos(max_ground_tilt_deg_ * 3.14159265358979323846 / 180.0);
    std::uniform_int_distribution<std::size_t> choose(0U, candidates.size() - 1U);
    GroundPlane best;
    for (int iteration = 0; iteration < ground_ransac_iterations_; ++iteration) {
      const auto &a = points[candidates[choose(random_engine_)]];
      const auto &b = points[candidates[choose(random_engine_)]];
      const auto &c = points[candidates[choose(random_engine_)]];
      const double abx = b.map_x - a.map_x;
      const double aby = b.map_y - a.map_y;
      const double abz = b.map_z - a.map_z;
      const double acx = c.map_x - a.map_x;
      const double acy = c.map_y - a.map_y;
      const double acz = c.map_z - a.map_z;
      double nx = aby * acz - abz * acy;
      double ny = abz * acx - abx * acz;
      double nz = abx * acy - aby * acx;
      const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (norm <= 1e-8) {
        continue;
      }
      nx /= norm;
      ny /= norm;
      nz /= norm;
      if (nz < 0.0) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
      }
      if (nz < minimum_normal_z) {
        continue;
      }
      const double d = -(nx * a.map_x + ny * a.map_y + nz * a.map_z);
      std::size_t inliers = 0U;
      for (const auto candidate : candidates) {
        const auto &point = points[candidate];
        const double distance =
            std::abs(nx * point.map_x + ny * point.map_y + nz * point.map_z + d);
        inliers += distance <= ground_distance_threshold_ ? 1U : 0U;
      }
      if (inliers > best.inliers) {
        best = {nx, ny, nz, d, inliers, true};
      }
    }
    if (!best.valid || best.inliers < static_cast<std::size_t>(min_ground_points_)) {
      return {};
    }

    // Refine z=a*x+b*y+c on the RANSAC inliers. Coordinates are centered to
    // keep the normal equations stable as the dynamic map grows.
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    std::size_t refined_count = 0U;
    for (const auto candidate : candidates) {
      const auto &point = points[candidate];
      const double distance = std::abs(
          best.nx * point.map_x + best.ny * point.map_y + best.nz * point.map_z + best.d);
      if (distance <= ground_distance_threshold_) {
        center_x += point.map_x;
        center_y += point.map_y;
        center_z += point.map_z;
        ++refined_count;
      }
    }
    if (refined_count < static_cast<std::size_t>(min_ground_points_)) {
      return {};
    }
    center_x /= static_cast<double>(refined_count);
    center_y /= static_cast<double>(refined_count);
    center_z /= static_cast<double>(refined_count);
    double xx = 0.0;
    double xy = 0.0;
    double yy = 0.0;
    double xz = 0.0;
    double yz = 0.0;
    for (const auto candidate : candidates) {
      const auto &point = points[candidate];
      const double distance = std::abs(
          best.nx * point.map_x + best.ny * point.map_y + best.nz * point.map_z + best.d);
      if (distance > ground_distance_threshold_) {
        continue;
      }
      const double x = point.map_x - center_x;
      const double y = point.map_y - center_y;
      const double z = point.map_z - center_z;
      xx += x * x;
      xy += x * y;
      yy += y * y;
      xz += x * z;
      yz += y * z;
    }
    const double determinant = xx * yy - xy * xy;
    if (std::abs(determinant) > 1e-9) {
      const double slope_x = (xz * yy - yz * xy) / determinant;
      const double slope_y = (yz * xx - xz * xy) / determinant;
      double nx = -slope_x;
      double ny = -slope_y;
      double nz = 1.0;
      const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      nx /= norm;
      ny /= norm;
      nz /= norm;
      const double d = -(nx * center_x + ny * center_y + nz * center_z);
      if (nx * origin_x + ny * origin_y + nz * origin_z + d < 0.0) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
      }
      best = {nx, ny, nz, d, refined_count, true};
    }
    return best;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (cloud->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring cloud without frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped sensor_transform;
    try {
      sensor_transform = tf_buffer_.lookupTransform(
          map_frame_, sensor_frame_, cloud->header.stamp, rclcpp::Duration::from_seconds(0.10));
    } catch (const tf2::TransformException &exact_error) {
      try {
        sensor_transform = tf_buffer_.lookupTransform(map_frame_, sensor_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException &latest_error) {
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting for %s -> %s TF (%s; latest: %s)", sensor_frame_.c_str(),
            map_frame_.c_str(), exact_error.what(), latest_error.what());
        return;
      }
    }

    const double origin_x = sensor_transform.transform.translation.x;
    const double origin_y = sensor_transform.transform.translation.y;
    const double origin_z = sensor_transform.transform.translation.z;
    double qx = sensor_transform.transform.rotation.x;
    double qy = sensor_transform.transform.rotation.y;
    double qz = sensor_transform.transform.rotation.z;
    double qw = sensor_transform.transform.rotation.w;
    const double quaternion_norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (quaternion_norm <= 1e-9) {
      return;
    }
    qx /= quaternion_norm;
    qy /= quaternion_norm;
    qz /= quaternion_norm;
    qw /= quaternion_norm;
    // Rotation matrix for map <- sensor. Its transpose converts registered
    // map points back to the physical sensor frame for Nav2 raytracing.
    const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r01 = 2.0 * (qx * qy - qz * qw);
    const double r02 = 2.0 * (qx * qz + qy * qw);
    const double r10 = 2.0 * (qx * qy + qz * qw);
    const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r12 = 2.0 * (qy * qz - qx * qw);
    const double r20 = 2.0 * (qx * qz - qy * qw);
    const double r21 = 2.0 * (qy * qz + qx * qw);
    const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
    const auto origin_cell_x = worldToCell(origin_x);
    const auto origin_cell_y = worldToCell(origin_y);
    std::vector<PointSample> points;
    points.reserve(cloud->width * cloud->height);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const double px = *x;
        const double py = *y;
        const double pz = *z;
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
          continue;
        }
        const double dx = px - origin_x;
        const double dy = py - origin_y;
        const double dz = pz - origin_z;
        const double range = std::hypot(dx, dy);
        if (range < min_range_ || range > max_range_) {
          continue;
        }
        points.push_back({
            px, py, pz, range,
            {static_cast<float>(r00 * dx + r10 * dy + r20 * dz),
             static_cast<float>(r01 * dx + r11 * dy + r21 * dz),
             static_cast<float>(r02 * dx + r12 * dy + r22 * dz)}});
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Cannot read PointCloud2 x/y/z fields: %s", error.what());
      return;
    }

    if (points.empty()) {
      return;
    }

    const auto ground = fitGroundPlane(points, origin_x, origin_y, origin_z);
    std::unordered_set<std::int64_t> ray_endpoints;
    std::unordered_set<std::int64_t> obstacle_endpoints;
    ray_endpoints.reserve(points.size() / 4U + 1U);
    obstacle_endpoints.reserve(points.size() / 8U + 1U);
    std::vector<std::array<float, 3>> sensor_points;
    std::vector<std::array<float, 3>> obstacle_points;
    sensor_points.reserve(points.size());
    obstacle_points.reserve(points.size() / 4U + 1U);
    for (const auto &point : points) {
      sensor_points.push_back(point.sensor);
      const auto key = cellKey(worldToCell(point.map_x), worldToCell(point.map_y));
      ray_endpoints.insert(key);
      double height = point.map_z - origin_z;
      if (ground.valid) {
        height = ground.nx * point.map_x + ground.ny * point.map_y +
                 ground.nz * point.map_z + ground.d;
      }
      if (height >= min_obstacle_height_ && height <= max_obstacle_height_) {
        obstacle_endpoints.insert(key);
        obstacle_points.push_back(point.sensor);
      }
    }
    publishSensorCloud(cloud->header, sensor_points, sensor_cloud_publisher_);
    publishSensorCloud(cloud->header, obstacle_points, obstacle_cloud_publisher_);
    if (ground.valid) {
      const double roll = std::atan2(ground.ny, ground.nz) * 180.0 / 3.14159265358979323846;
      const double pitch = std::atan2(-ground.nx, ground.nz) * 180.0 / 3.14159265358979323846;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ground plane: roll=%.2f deg pitch=%.2f deg, inliers=%zu, obstacle points=%zu/%zu",
          roll, pitch, ground.inliers, obstacle_points.size(), points.size());
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ground plane unavailable; using conservative sensor-height fallback");
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    updateBounds(origin_cell_x, origin_cell_y);
    for (const auto endpoint : ray_endpoints) {
      const auto [end_x, end_y] = decodeCell(endpoint);
      raytraceFree(origin_cell_x, origin_cell_y, end_x, end_y);
    }
    // Ground/ceiling returns are explicit free endpoints. Obstacle endpoints
    // are marked last so another clearing ray cannot erase the same cell.
    for (const auto endpoint : ray_endpoints) {
      const auto [end_x, end_y] = decodeCell(endpoint);
      markFree(end_x, end_y);
    }
    for (const auto endpoint : obstacle_endpoints) {
      const auto [end_x, end_y] = decodeCell(endpoint);
      markOccupied(end_x, end_y);
    }
    last_stamp_ = cloud->header.stamp;
  }

  std::optional<nav_msgs::msg::OccupancyGrid> snapshotMap() {
    nav_msgs::msg::OccupancyGrid map;
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (cells_.empty()) {
      return std::nullopt;
    }

    const auto padding_cells = static_cast<std::int32_t>(std::ceil(padding_ / resolution_));
    const auto chunk_cells = static_cast<std::int32_t>(
        std::max(1.0, std::ceil(expansion_chunk_size_ / resolution_)));
    const auto raw_min_x = min_x_ - padding_cells;
    const auto raw_min_y = min_y_ - padding_cells;
    const auto raw_max_x = max_x_ + padding_cells;
    const auto raw_max_y = max_y_ + padding_cells;
    // Snap dynamic bounds outward so Nav2's StaticLayer is not resized on
    // every single new ray. This preserves expansion while keeping ongoing
    // planning stable.
    const auto out_min_x = static_cast<std::int32_t>(
        std::floor(static_cast<double>(raw_min_x) / chunk_cells) * chunk_cells);
    const auto out_min_y = static_cast<std::int32_t>(
        std::floor(static_cast<double>(raw_min_y) / chunk_cells) * chunk_cells);
    const auto out_max_x = static_cast<std::int32_t>(
        std::ceil(static_cast<double>(raw_max_x + 1) / chunk_cells) * chunk_cells - 1);
    const auto out_max_y = static_cast<std::int32_t>(
        std::ceil(static_cast<double>(raw_max_y + 1) / chunk_cells) * chunk_cells - 1);
    const auto width = static_cast<std::uint64_t>(out_max_x - out_min_x + 1);
    const auto height = static_cast<std::uint64_t>(out_max_y - out_min_y + 1);
    if (width == 0U || height == 0U || width * height > static_cast<std::uint64_t>(max_cells_)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dynamic map bounds are invalid or exceed max_cells (%lux%lu > %d); not publishing",
          width, height, max_cells_);
      return std::nullopt;
    }

    map.header.stamp = last_stamp_;
    map.header.frame_id = map_frame_;
    map.info.map_load_time = first_map_stamp_.nanoseconds() == 0 ? last_stamp_ : first_map_stamp_;
    if (first_map_stamp_.nanoseconds() == 0) {
      first_map_stamp_ = last_stamp_;
      map.info.map_load_time = first_map_stamp_;
    }
    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = static_cast<std::uint32_t>(width);
    map.info.height = static_cast<std::uint32_t>(height);
    map.info.origin.position.x = static_cast<double>(out_min_x) * resolution_;
    map.info.origin.position.y = static_cast<double>(out_min_y) * resolution_;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(width * height, -1);

    for (const auto &[key, score] : cells_) {
      const auto [x, y] = decodeCell(key);
      const auto column = static_cast<std::uint64_t>(x - out_min_x);
      const auto row = static_cast<std::uint64_t>(y - out_min_y);
      if (column >= width || row >= height) {
        continue;
      }
      map.data[row * width + column] = score >= occupied_threshold_ ? 100 : 0;
    }
    return map;
  }

  void publishMap() {
    const auto map = snapshotMap();
    if (map) {
      map_publisher_->publish(*map);
    }
  }

  void saveMap() {
    const auto map = snapshotMap();
    if (!map) {
      RCLCPP_WARN(get_logger(), "No occupancy data was received; not saving a navigation map");
      return;
    }
    const std::filesystem::path directory(save_map_directory_);
    const auto pgm_path = directory / (save_map_name_ + ".pgm");
    const auto yaml_path = directory / (save_map_name_ + ".yaml");
    try {
      std::filesystem::create_directories(directory);
      std::ofstream pgm(pgm_path, std::ios::binary | std::ios::trunc);
      if (!pgm) {
        throw std::runtime_error("cannot open PGM file");
      }
      pgm << "P5\n# autonomy_light occupancy map\n" << map->info.width << " "
          << map->info.height << "\n255\n";
      for (std::int64_t row = static_cast<std::int64_t>(map->info.height) - 1; row >= 0; --row) {
        for (std::uint32_t column = 0; column < map->info.width; ++column) {
          const auto cell = map->data[static_cast<std::size_t>(row) * map->info.width + column];
          const unsigned char pixel = cell < 0 ? 205U : (cell >= 65 ? 0U : 254U);
          pgm.write(reinterpret_cast<const char *>(&pixel), 1);
        }
      }
      if (!pgm) {
        throw std::runtime_error("cannot write PGM data");
      }
      std::ofstream yaml(yaml_path, std::ios::trunc);
      if (!yaml) {
        throw std::runtime_error("cannot open YAML file");
      }
      yaml << "image: " << pgm_path.filename().string() << "\n"
           << "mode: trinary\n"
           << "resolution: " << std::setprecision(10) << map->info.resolution << "\n"
           << "origin: [" << map->info.origin.position.x << ", "
           << map->info.origin.position.y << ", 0.0]\n"
           << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
      if (!yaml) {
        throw std::runtime_error("cannot write YAML metadata");
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Failed to save navigation map in '%s': %s",
                   directory.c_str(), error.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "Saved navigation map: %s and %s", pgm_path.c_str(),
                yaml_path.c_str());
  }

  std::string input_topic_;
  std::string map_topic_;
  std::string sensor_cloud_topic_;
  std::string obstacle_cloud_topic_;
  std::string map_frame_;
  std::string sensor_frame_;
  double resolution_{0.10};
  double publish_frequency_{2.0};
  double min_obstacle_height_{-0.55};
  double max_obstacle_height_{0.80};
  double ground_distance_threshold_{0.055};
  double max_ground_tilt_deg_{35.0};
  int min_ground_points_{100};
  int ground_ransac_iterations_{100};
  double min_range_{0.10};
  double max_range_{12.0};
  double padding_{2.0};
  double expansion_chunk_size_{5.0};
  int occupied_increment_{3};
  int free_decrement_{1};
  int occupied_threshold_{2};
  int min_score_{-10};
  int max_score_{20};
  int max_cells_{4000000};
  bool save_map_{false};
  std::string save_map_directory_;
  std::string save_map_name_{"map"};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::mutex map_mutex_;
  std::unordered_map<std::int64_t, CellScore> cells_;
  std::int32_t min_x_{std::numeric_limits<std::int32_t>::max()};
  std::int32_t max_x_{std::numeric_limits<std::int32_t>::min()};
  std::int32_t min_y_{std::numeric_limits<std::int32_t>::max()};
  std::int32_t max_y_{std::numeric_limits<std::int32_t>::min()};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_map_stamp_{0, 0, RCL_ROS_TIME};
  std::mt19937 random_engine_{0xC0CE10U};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiveOccupancyMapper>());
  rclcpp::shutdown();
  return 0;
}
