#include "autonomy_light/elevation_mapper.hpp"

namespace autonomy_light {

ElevationMapper::ElevationMapper(ElevationMapperConfig config) {
  configure(std::move(config));
}

void ElevationMapper::configure(ElevationMapperConfig config) {
  if (config.grid.resolution <= 0.0 || config.grid.x_length <= 0.0 ||
      config.grid.y_length <= 0.0) {
    throw std::invalid_argument("elevation grid dimensions must be positive");
  }
  if (config.source != "rolling" && config.source != "global") {
    throw std::invalid_argument("height_map.source must be rolling or global");
  }
  config.min_samples_per_cell = std::max(1, config.min_samples_per_cell);
  config.ground_percentile = std::clamp(config.ground_percentile, 0.0, 1.0);
  config.global_voxel_size = std::max(0.005, config.global_voxel_size);
  config.global_max_voxels = std::max<std::size_t>(1000U, config.global_max_voxels);
  config_ = std::move(config);
  global_surfaces_.clear();
  rolling_surfaces_.clear();
  global_voxels_.clear();
}

CellKey ElevationMapper::keyFor(const double x, const double y) const {
  return {static_cast<std::int64_t>(std::floor(x / config_.grid.resolution)),
          static_cast<std::int64_t>(std::floor(y / config_.grid.resolution))};
}

float ElevationMapper::percentile(std::vector<float> &values,
                                  const double fraction) {
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto index = std::min(
      values.size() - 1U,
      static_cast<std::size_t>(std::round(fraction * (values.size() - 1U))));
  std::nth_element(values.begin(), values.begin() + static_cast<long>(index),
                   values.end());
  return values[index];
}

void ElevationMapper::fuse(float &mean, float &variance, double &last_time,
                            std::uint32_t &support, const float measurement,
                            const double measurement_variance,
                            const double stamp_sec,
                            const ElevationMapperConfig &config) {
  if (!std::isfinite(mean) || !std::isfinite(variance) ||
      stamp_sec - last_time > config.rolling_max_age_sec) {
    mean = measurement;
    variance = static_cast<float>(measurement_variance);
    last_time = stamp_sec;
    support = 1U;
    return;
  }
  const double dt = std::max(0.0, stamp_sec - last_time);
  const double prior = std::min(config.rolling_max_variance,
                                variance + config.rolling_process_variance_per_sec * dt);
  const double innovation = measurement - mean;
  const double innovation_variance = std::max(1.0e-9, prior + measurement_variance);
  if (std::abs(innovation) > config.rolling_mahalanobis_threshold *
                                 std::sqrt(innovation_variance)) {
    variance = static_cast<float>(std::min(config.rolling_max_variance,
                                            prior + config.rolling_outlier_variance));
    last_time = stamp_sec;
    return;
  }
  const double gain = prior / innovation_variance;
  mean = static_cast<float>(mean + gain * innovation);
  variance = static_cast<float>(std::clamp((1.0 - gain) * prior, 1.0e-6,
                                           config.rolling_max_variance));
  last_time = stamp_sec;
  ++support;
}

void ElevationMapper::updateSurfaces(
    SurfaceMap &target,
    std::unordered_map<CellKey, std::vector<float>, CellKeyHash> &samples,
    const Pose2_5D &robot, const double stamp_sec, const bool rolling) {
  for (auto &[key, values] : samples) {
    if (static_cast<int>(values.size()) < config_.min_samples_per_cell) {
      continue;
    }
    const float ground = percentile(values, config_.ground_percentile);
    const float upper = *std::max_element(values.begin(), values.end());
    if (!std::isfinite(ground) || !std::isfinite(upper)) {
      continue;
    }
    auto &surface = target[key];
    if (!rolling) {
      surface.ground = std::isfinite(surface.ground)
                           ? std::min(surface.ground, ground)
                           : ground;
      surface.upper = std::isfinite(surface.upper) ? std::max(surface.upper, upper)
                                                     : upper;
      surface.support += static_cast<std::uint32_t>(values.size());
      continue;
    }
    const double cx = (static_cast<double>(key.x) + 0.5) * config_.grid.resolution;
    const double cy = (static_cast<double>(key.y) + 0.5) * config_.grid.resolution;
    const double range2 = (cx - robot.x) * (cx - robot.x) +
                          (cy - robot.y) * (cy - robot.y) +
                          (static_cast<double>(ground) - robot.z) *
                              (static_cast<double>(ground) - robot.z);
    const double variance = std::clamp(
        config_.rolling_base_variance + config_.rolling_range_variance_factor * range2,
        1.0e-6, config_.rolling_max_variance);
    fuse(surface.ground, surface.ground_variance, surface.ground_time,
         surface.support, ground, variance, stamp_sec, config_);
    if (upper - ground >= config_.obstacle_min_height) {
      fuse(surface.upper, surface.upper_variance, surface.upper_time,
           surface.support, upper, variance, stamp_sec, config_);
    }
  }
}

void ElevationMapper::addVoxels(const PclCloud &cloud) {
  for (const auto &point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const auto leaf = config_.global_voxel_size;
    const VoxelKey key{static_cast<std::int64_t>(std::floor(point.x / leaf)),
                       static_cast<std::int64_t>(std::floor(point.y / leaf)),
                       static_cast<std::int64_t>(std::floor(point.z / leaf))};
    auto found = global_voxels_.find(key);
    if (found == global_voxels_.end()) {
      if (global_voxels_.size() >= config_.global_max_voxels) {
        continue;
      }
      found = global_voxels_.emplace(key, VoxelMean{}).first;
    }
    auto &mean = found->second;
    mean.x += point.x;
    mean.y += point.y;
    mean.z += point.z;
    ++mean.count;
  }
}

void ElevationMapper::integrate(const PclCloud &cloud, const Pose2_5D &robot,
                                const double stamp_sec) {
  if (cloud.empty()) {
    return;
  }
  addVoxels(cloud);
  std::unordered_map<CellKey, std::vector<float>, CellKeyHash> global_samples;
  std::unordered_map<CellKey, std::vector<float>, CellKeyHash> rolling_samples;
  const double radius2 = config_.rolling_max_radius_m * config_.rolling_max_radius_m;
  for (const auto &point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const CellKey key = keyFor(point.x, point.y);
    global_samples[key].push_back(point.z);
    const double dx = point.x - robot.x;
    const double dy = point.y - robot.y;
    if (dx * dx + dy * dy <= radius2) {
      rolling_samples[key].push_back(point.z);
    }
  }
  updateSurfaces(global_surfaces_, global_samples, robot, stamp_sec, false);
  if (config_.source == "rolling") {
    updateSurfaces(rolling_surfaces_, rolling_samples, robot, stamp_sec, true);
  }
}

void ElevationMapper::loadGlobalMap(const PclCloud &cloud) {
  Pose2_5D origin;
  std::unordered_map<CellKey, std::vector<float>, CellKeyHash> samples;
  for (const auto &point : cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      samples[keyFor(point.x, point.y)].push_back(point.z);
    }
  }
  addVoxels(cloud);
  updateSurfaces(global_surfaces_, samples, origin, 0.0, false);
}

bool ElevationMapper::empty() const { return global_surfaces_.empty(); }

const ElevationMapper::Surface *
ElevationMapper::nearestSurface(const SurfaceMap &surfaces, const double x,
                                const double y) const {
  const CellKey center = keyFor(x, y);
  const Surface *best = nullptr;
  double best_distance2 = std::numeric_limits<double>::infinity();
  for (std::int64_t dy = -1; dy <= 1; ++dy) {
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      const auto found = surfaces.find({center.x + dx, center.y + dy});
      if (found == surfaces.end()) {
        continue;
      }
      const double px = (static_cast<double>(center.x + dx) + 0.5) *
                        config_.grid.resolution;
      const double py = (static_cast<double>(center.y + dy) + 0.5) *
                        config_.grid.resolution;
      const double distance2 = (px - x) * (px - x) + (py - y) * (py - y);
      if (distance2 < best_distance2) {
        best = &found->second;
        best_distance2 = distance2;
      }
    }
  }
  return best;
}

bool ElevationMapper::validRolling(const Surface &surface,
                                   const double stamp_sec,
                                   const bool upper) const {
  const double age = stamp_sec - (upper ? surface.upper_time : surface.ground_time);
  const float variance = upper ? surface.upper_variance : surface.ground_variance;
  const double age_limit = upper ? config_.rolling_upper_max_age_sec
                                 : config_.rolling_max_age_sec;
  return age >= 0.0 && age <= age_limit && std::isfinite(variance) &&
         variance <= config_.rolling_max_variance;
}

HeightGrid ElevationMapper::build(const Pose2_5D &robot, const double stamp_sec,
                                  const std_msgs::msg::Header &header) const {
  HeightGrid grid(config_.grid);
  grid.header = header;
  const SurfaceMap &source = config_.source == "global" ? global_surfaces_
                                                           : rolling_surfaces_;
  if (source.empty()) {
    return grid;
  }
  const double cos_yaw = std::cos(robot.yaw);
  const double sin_yaw = std::sin(robot.yaw);
  std::vector<float> local_floor;
  std::vector<const Surface *> selected(grid.spec.size(), nullptr);
  for (std::uint32_t row = 0; row < grid.spec.height(); ++row) {
    for (std::uint32_t col = 0; col < grid.spec.width(); ++col) {
      const double lx = grid.spec.xMin() + (col + 0.5) * grid.spec.resolution;
      const double ly = grid.spec.yMin() + (row + 0.5) * grid.spec.resolution;
      const Surface *surface = nearestSurface(
          source, robot.x + cos_yaw * lx - sin_yaw * ly,
          robot.y + sin_yaw * lx + cos_yaw * ly);
      if (!surface || !std::isfinite(surface->ground) ||
          (config_.source == "rolling" && !validRolling(*surface, stamp_sec, false))) {
        continue;
      }
      const auto index = static_cast<std::size_t>(row) * grid.spec.width() + col;
      selected[index] = surface;
      if (std::hypot(lx, ly) <= config_.floor_radius_m) {
        local_floor.push_back(surface->ground);
      }
    }
  }
  if (local_floor.empty()) {
    for (const auto *surface : selected) {
      if (surface != nullptr) {
        local_floor.push_back(surface->ground);
      }
    }
  }
  if (local_floor.empty()) {
    return grid;
  }
  grid.floor_z = percentile(local_floor, 0.20);
  for (std::size_t index = 0; index < selected.size(); ++index) {
    const Surface *surface = selected[index];
    if (surface == nullptr) {
      continue;
    }
    const bool upper_valid = std::isfinite(surface->upper) &&
        surface->upper - surface->ground >= config_.obstacle_min_height &&
        (config_.source == "global" || validRolling(*surface, stamp_sec, true));
    const float z = (upper_valid ? surface->upper : surface->ground) -
                    static_cast<float>(grid.floor_z);
    if (z < grid.spec.min_z || z > grid.spec.max_z) {
      continue;
    }
    grid.height[index] = z;
    grid.valid[index] = 1U;
    grid.variance[index] = config_.source == "global"
                               ? static_cast<float>(config_.rolling_base_variance)
                               : (upper_valid ? surface->upper_variance
                                              : surface->ground_variance);
    grid.age[index] = config_.source == "global"
                          ? 0.0F
                          : static_cast<float>(std::max(
                                0.0, stamp_sec - (upper_valid ? surface->upper_time
                                                               : surface->ground_time)));
  }
  return grid;
}

PclCloud::Ptr ElevationMapper::globalMap() const {
  PclCloud::Ptr cloud(new PclCloud());
  cloud->reserve(global_voxels_.size());
  for (const auto &[key, mean] : global_voxels_) {
    (void)key;
    if (mean.count != 0U) {
      const double inverse = 1.0 / static_cast<double>(mean.count);
      cloud->push_back(pcl::PointXYZ(static_cast<float>(mean.x * inverse),
                                     static_cast<float>(mean.y * inverse),
                                     static_cast<float>(mean.z * inverse)));
    }
  }
  return cloud;
}

} // namespace autonomy_light
