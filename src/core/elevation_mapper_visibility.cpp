#include "autonomy_light/elevation_mapper.hpp"

#include <Eigen/Dense>

#include <unordered_set>

namespace autonomy_light {
namespace {

struct Ray {
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
  double horizontal_length{0.0};
};

struct RayHeight {
  double z;
  Eigen::Vector3d direction;
};

Eigen::Vector3d sensorOrigin(const PointObservation &point,
                             const Pose2_5D &robot) {
  if (std::isfinite(point.origin_x) && std::isfinite(point.origin_y) &&
      std::isfinite(point.origin_z)) {
    return {point.origin_x, point.origin_y, point.origin_z};
  }
  return {robot.x, robot.y, robot.z};
}

bool rayTo(const PointObservation &point, const Pose2_5D &robot, Ray &ray) {
  ray.origin = sensorOrigin(point, robot);
  const Eigen::Vector3d endpoint(point.x, point.y, point.z);
  const Eigen::Vector3d delta = endpoint - ray.origin;
  ray.horizontal_length = delta.head<2>().norm();
  if (!ray.origin.allFinite() || !endpoint.allFinite() ||
      ray.horizontal_length <= 1.0e-9) {
    return false;
  }
  ray.direction = delta / ray.horizontal_length;
  return true;
}

} // namespace

bool ElevationMapper::acceptsRollingMeasurement(const PointObservation &point,
                                                const Pose2_5D &robot) const {
  if (!config_.overhang_filter_enabled) {
    return true;
  }
  Ray ray;
  if (!rayTo(point, robot, ray)) {
    return false;
  }
  const double above_sensor = point.z - ray.origin.z();
  const double ramp =
      std::max(0.0, ray.horizontal_length - config_.overhang_ramp_start_m) *
      config_.overhang_ramp_slope;
  const double allowed =
      std::min(config_.overhang_absolute_max_above_sensor_m,
               config_.overhang_near_max_above_sensor_m + ramp);
  return above_sensor <= allowed;
}

void ElevationMapper::cleanupVisibility(const PointObservations &cloud,
                                        const Pose2_5D &robot,
                                        const double stamp_sec) {
  if (!config_.visibility_cleanup_enabled ||
      config_.visibility_max_ray_length_m <= 0.0 || rolling_surfaces_.empty()) {
    return;
  }
  std::unordered_set<CellKey, CellKeyHash> observed;
  std::unordered_map<CellKey, RayHeight, CellKeyHash> ray_heights;
  const double step = config_.grid.resolution / std::sqrt(2.0);
  for (const auto &point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !acceptsRollingMeasurement(point, robot)) {
      continue;
    }
    Ray ray;
    if (!rayTo(point, robot, ray) ||
        ray.horizontal_length < config_.visibility_min_ray_length_m) {
      continue;
    }
    observed.insert(keyFor(point.x, point.y));
    const double length =
        std::min(ray.horizontal_length, config_.visibility_max_ray_length_m);
    CellKey previous{std::numeric_limits<std::int64_t>::min(), 0};
    for (double distance = step; distance < length; distance += step) {
      const Eigen::Vector3d position = ray.origin + ray.direction * distance;
      const CellKey key = keyFor(position.x(), position.y());
      if (key == previous || observed.count(key) != 0U) {
        continue;
      }
      previous = key;
      const auto found = ray_heights.find(key);
      if (found == ray_heights.end() || position.z() < found->second.z) {
        ray_heights[key] = {position.z(), ray.direction.normalized()};
      }
    }
  }
  for (const auto &[key, ray_height] : ray_heights) {
    if (observed.count(key) != 0U) {
      continue;
    }
    auto found = rolling_surfaces_.find(key);
    if (found == rolling_surfaces_.end()) {
      continue;
    }
    Surface &surface = found->second;
    if (stamp_sec - surface.ground_time <
            config_.visibility_min_observation_age_sec ||
        surface.support == 0U || !std::isfinite(surface.ground) ||
        !std::isfinite(surface.ground_variance)) {
      continue;
    }
    const auto neighbor = [&](const std::int64_t dx,
                              const std::int64_t dy) -> const Surface * {
      const auto candidate = rolling_surfaces_.find({key.x + dx, key.y + dy});
      return candidate != rolling_surfaces_.end() &&
                     std::isfinite(candidate->second.ground)
                 ? &candidate->second
                 : nullptr;
    };
    const Surface *left = neighbor(-1, 0);
    const Surface *right = neighbor(1, 0);
    const Surface *down = neighbor(0, -1);
    const Surface *up = neighbor(0, 1);
    if ((!left && !right) || (!down && !up)) {
      continue;
    }
    const auto gradient = [&](const Surface *negative, const Surface *positive,
                              const float center) {
      if (negative && positive) {
        return (positive->ground - negative->ground) /
               (2.0 * config_.grid.resolution);
      }
      return positive ? (positive->ground - center) / config_.grid.resolution
                      : (center - negative->ground) / config_.grid.resolution;
    };
    const Eigen::Vector3d normal(-gradient(left, right, surface.ground),
                                 -gradient(down, up, surface.ground), 1.0);
    if (!normal.allFinite() || !ray_height.direction.allFinite() ||
        std::abs(normal.normalized().dot(ray_height.direction)) <
            config_.visibility_normal_alignment_min) {
      continue;
    }
    const double tolerance =
        config_.visibility_height_margin_m +
        config_.visibility_sigma_scale *
            std::sqrt(std::max(0.0F, surface.ground_variance));
    if (ray_height.z >= surface.ground - tolerance) {
      continue;
    }
    surface.ground = std::numeric_limits<float>::quiet_NaN();
    surface.ground_variance = std::numeric_limits<float>::infinity();
    surface.ground_time = -std::numeric_limits<double>::infinity();
    surface.upper = std::numeric_limits<float>::quiet_NaN();
    surface.upper_variance = std::numeric_limits<float>::infinity();
    surface.upper_time = -std::numeric_limits<double>::infinity();
    surface.support = 0U;
  }
}

} // namespace autonomy_light
