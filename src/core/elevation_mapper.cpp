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
  config.global_max_voxels =
      std::max<std::size_t>(1000U, config.global_max_voxels);
  config.d435_base_variance = std::max(1.0e-6, config.d435_base_variance);
  config.d435_range_variance_factor =
      std::max(0.0, config.d435_range_variance_factor);
  config.lidar_extrinsic_translation_std_m =
      std::max(0.0, config.lidar_extrinsic_translation_std_m);
  config.lidar_extrinsic_rotation_std_deg =
      std::max(0.0, config.lidar_extrinsic_rotation_std_deg);
  config.d435_extrinsic_translation_std_m =
      std::max(0.0, config.d435_extrinsic_translation_std_m);
  config.d435_extrinsic_rotation_std_deg =
      std::max(0.0, config.d435_extrinsic_rotation_std_deg);
  config.pose_min_position_std_m =
      std::max(0.001, config.pose_min_position_std_m);
  config.pose_min_orientation_std_deg =
      std::max(0.01, config.pose_min_orientation_std_deg);
  config.pose_max_position_std_m =
      std::max(config.pose_min_position_std_m, config.pose_max_position_std_m);
  config.pose_max_orientation_std_deg = std::max(
      config.pose_min_orientation_std_deg, config.pose_max_orientation_std_deg);
  config.pose_splat_max_cells = std::clamp(config.pose_splat_max_cells, 0, 4);
  config.rolling_initial_prior_radius_m =
      std::max(0.0, config.rolling_initial_prior_radius_m);
  config.rolling_initial_prior_ground_distance_m =
      std::max(0.0, config.rolling_initial_prior_ground_distance_m);
  config.rolling_initial_prior_variance =
      std::clamp(config.rolling_initial_prior_variance, 1.0e-6,
                 config.rolling_max_variance);
  config.visibility_max_ray_length_m =
      std::max(0.0, config.visibility_max_ray_length_m);
  config.visibility_min_ray_length_m =
      std::clamp(config.visibility_min_ray_length_m, 0.0,
                 config.visibility_max_ray_length_m);
  config.visibility_min_observation_age_sec =
      std::max(0.0, config.visibility_min_observation_age_sec);
  config.visibility_normal_alignment_min =
      std::clamp(config.visibility_normal_alignment_min, 0.0, 1.0);
  config.visibility_height_margin_m =
      std::max(0.0, config.visibility_height_margin_m);
  config.visibility_sigma_scale = std::max(0.0, config.visibility_sigma_scale);
  config.overhang_near_max_above_sensor_m =
      std::max(0.0, config.overhang_near_max_above_sensor_m);
  config.overhang_ramp_start_m = std::max(0.0, config.overhang_ramp_start_m);
  config.overhang_ramp_slope = std::max(0.0, config.overhang_ramp_slope);
  config.overhang_absolute_max_above_sensor_m =
      std::max(config.overhang_near_max_above_sensor_m,
               config.overhang_absolute_max_above_sensor_m);
  config_ = std::move(config);
  global_surfaces_.clear();
  rolling_grid_ = {};
  global_voxels_.clear();
  rolling_prior_seeded_ = false;
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

ElevationMapper::Sample ElevationMapper::percentile(std::vector<Sample> &values,
                                                    const double fraction) {
  const auto index = std::min(
      values.size() - 1U,
      static_cast<std::size_t>(std::round(fraction * (values.size() - 1U))));
  std::nth_element(
      values.begin(), values.begin() + static_cast<long>(index), values.end(),
      [](const Sample &left, const Sample &right) { return left.z < right.z; });
  return values[index];
}

void ElevationMapper::fuse(float &mean, float &variance, double &last_time,
                           std::uint32_t &support, const float measurement,
                           const double measurement_variance,
                           const double stamp_sec,
                           const ElevationMapperConfig &config) {
  if (!std::isfinite(mean) || !std::isfinite(variance)) {
    mean = measurement;
    variance = static_cast<float>(measurement_variance);
    last_time = stamp_sec;
    support = 1U;
    return;
  }
  const double dt = std::max(0.0, stamp_sec - last_time);
  const double prior =
      std::min(config.rolling_max_variance,
               variance + config.rolling_process_variance_per_sec * dt);
  const double innovation = measurement - mean;
  const double innovation_variance =
      std::max(1.0e-9, prior + measurement_variance);
  if (support != 0U &&
      std::abs(innovation) > config.rolling_mahalanobis_threshold *
                                 std::sqrt(innovation_variance)) {
    variance = static_cast<float>(std::min(
        config.rolling_max_variance, prior + config.rolling_outlier_variance));
    last_time = stamp_sec;
    return;
  }
  const double gain = prior / innovation_variance;
  mean = static_cast<float>(mean + gain * innovation);
  variance = static_cast<float>(
      std::clamp((1.0 - gain) * prior, 1.0e-6, config.rolling_max_variance));
  last_time = stamp_sec;
  ++support;
}

void ElevationMapper::seedInitialRollingPrior(const Pose2_5D &robot,
                                              const double stamp_sec) {
  if (rolling_prior_seeded_ || config_.source != "rolling" ||
      !config_.rolling_initial_prior_enabled || !std::isfinite(robot.x) ||
      !std::isfinite(robot.y) || !std::isfinite(robot.z) ||
      !std::isfinite(stamp_sec)) {
    return;
  }
  const int radius = static_cast<int>(std::ceil(
      config_.rolling_initial_prior_radius_m / config_.grid.resolution));
  const double radius2 = std::pow(config_.rolling_initial_prior_radius_m, 2);
  const double cos_yaw = std::cos(robot.yaw);
  const double sin_yaw = std::sin(robot.yaw);
  const float ground = static_cast<float>(
      robot.z - config_.rolling_initial_prior_ground_distance_m);
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      const double dx = x * config_.grid.resolution;
      const double dy = y * config_.grid.resolution;
      if (dx * dx + dy * dy > radius2) {
        continue;
      }
      Surface *surface =
          rollingSurface(keyFor(robot.x + cos_yaw * dx - sin_yaw * dy,
                                robot.y + sin_yaw * dx + cos_yaw * dy));
      if (surface && !std::isfinite(surface->ground)) {
        surface->ground = ground;
        surface->ground_variance =
            static_cast<float>(config_.rolling_initial_prior_variance);
        surface->ground_time = stamp_sec;
        surface->support = 0U;
      }
    }
  }
  rolling_prior_seeded_ = true;
}

void ElevationMapper::updateSurfaces(SurfaceMap &target, SampleMap &samples) {
  for (auto &[key, values] : samples) {
    if (static_cast<int>(values.size()) < config_.min_samples_per_cell) {
      continue;
    }
    const Sample ground_sample = percentile(values, config_.ground_percentile);
    const Sample upper_sample =
        *std::max_element(values.begin(), values.end(),
                          [](const Sample &left, const Sample &right) {
                            return left.z < right.z;
                          });
    const float ground = ground_sample.z;
    const float upper = upper_sample.z;
    if (!std::isfinite(ground) || !std::isfinite(upper)) {
      continue;
    }
    auto &surface = target[key];
    surface.ground = std::isfinite(surface.ground)
                         ? std::min(surface.ground, ground)
                         : ground;
    surface.upper =
        std::isfinite(surface.upper) ? std::max(surface.upper, upper) : upper;
    surface.support += static_cast<std::uint32_t>(values.size());
  }
}

void ElevationMapper::updateRollingSurfaces(SampleMap &samples,
                                            const double stamp_sec) {
  for (auto &[key, values] : samples) {
    if (static_cast<int>(values.size()) < config_.min_samples_per_cell) {
      continue;
    }
    Surface *surface = rollingSurface(key);
    if (surface == nullptr) {
      continue;
    }
    const Sample ground_sample = percentile(values, config_.ground_percentile);
    const Sample upper_sample =
        *std::max_element(values.begin(), values.end(),
                          [](const Sample &left, const Sample &right) {
                            return left.z < right.z;
                          });
    if (!std::isfinite(ground_sample.z) || !std::isfinite(upper_sample.z)) {
      continue;
    }
    fuse(surface->ground, surface->ground_variance, surface->ground_time,
         surface->support, ground_sample.z,
         std::clamp(static_cast<double>(ground_sample.variance), 1.0e-6,
                    config_.rolling_max_variance),
         stamp_sec, config_);
    if (upper_sample.z - ground_sample.z >= config_.obstacle_min_height) {
      fuse(surface->upper, surface->upper_variance, surface->upper_time,
           surface->support, upper_sample.z,
           std::clamp(static_cast<double>(upper_sample.variance), 1.0e-6,
                      config_.rolling_max_variance),
           stamp_sec, config_);
    }
  }
}

void ElevationMapper::addVoxels(const PointObservations &cloud) {
  for (const auto &point : cloud) {
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

void ElevationMapper::addVoxels(const PclCloud &cloud) {
  PointObservations observations;
  observations.reserve(cloud.size());
  for (const auto &point : cloud.points) {
    observations.push_back({point.x, point.y, point.z, 0U});
  }
  addVoxels(observations);
}

void ElevationMapper::integrate(const PointObservations &cloud,
                                const Pose2_5D &robot, const double stamp_sec) {
  if (cloud.empty()) {
    return;
  }
  addVoxels(cloud);
  SampleMap global_samples;
  SampleMap rolling_samples;
  const bool rolling = config_.source == "rolling";
  if (rolling) {
    moveRollingGrid(robot);
  }
  const double radius2 =
      config_.rolling_max_radius_m * config_.rolling_max_radius_m;
  for (const auto &point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const CellKey key = keyFor(point.x, point.y);
    global_samples[key].push_back(
        {point.z, static_cast<float>(config_.rolling_base_variance)});
    const double dx = point.x - robot.x;
    const double dy = point.y - robot.y;
    if (rolling && dx * dx + dy * dy <= radius2 &&
        acceptsRollingMeasurement(point, robot)) {
      addRollingSamples(rolling_samples, point, robot);
    }
  }
  updateSurfaces(global_surfaces_, global_samples);
  if (rolling) {
    cleanupVisibility(cloud, robot, stamp_sec);
    updateRollingSurfaces(rolling_samples, stamp_sec);
  }
}

void ElevationMapper::loadGlobalMap(const PclCloud &cloud) {
  SampleMap samples;
  for (const auto &point : cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      samples[keyFor(point.x, point.y)].push_back(
          {point.z, static_cast<float>(config_.rolling_base_variance)});
    }
  }
  addVoxels(cloud);
  updateSurfaces(global_surfaces_, samples);
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
      const double px =
          (static_cast<double>(center.x + dx) + 0.5) * config_.grid.resolution;
      const double py =
          (static_cast<double>(center.y + dy) + 0.5) * config_.grid.resolution;
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
                                   const bool upper) const {
  const float variance =
      upper ? surface.upper_variance : surface.ground_variance;
  return std::isfinite(variance) && variance <= config_.rolling_max_variance;
}

HeightGrid ElevationMapper::build(const Pose2_5D &robot, const double stamp_sec,
                                  const std_msgs::msg::Header &header) {
  HeightGrid grid(config_.grid);
  grid.header = header;
  if (config_.source == "rolling") {
    moveRollingGrid(robot);
  }
  seedInitialRollingPrior(robot, stamp_sec);
  if (config_.source == "global" && global_surfaces_.empty()) {
    return grid;
  }
  const double cos_yaw = std::cos(robot.yaw);
  const double sin_yaw = std::sin(robot.yaw);
  std::vector<const Surface *> selected(grid.spec.size(), nullptr);
  for (std::uint32_t row = 0; row < grid.spec.height(); ++row) {
    for (std::uint32_t col = 0; col < grid.spec.width(); ++col) {
      const double lx = grid.spec.xMin() + (col + 0.5) * grid.spec.resolution;
      const double ly = grid.spec.yMin() + (row + 0.5) * grid.spec.resolution;
      const double x = robot.x + cos_yaw * lx - sin_yaw * ly;
      const double y = robot.y + sin_yaw * lx + cos_yaw * ly;
      const Surface *surface = config_.source == "global"
                                   ? nearestSurface(global_surfaces_, x, y)
                                   : nearestRollingSurface(x, y);
      if (!surface || !std::isfinite(surface->ground) ||
          (config_.source == "rolling" && !validRolling(*surface, false))) {
        continue;
      }
      const auto index =
          static_cast<std::size_t>(row) * grid.spec.width() + col;
      selected[index] = surface;
    }
  }
  for (std::size_t index = 0; index < selected.size(); ++index) {
    const Surface *surface = selected[index];
    if (surface == nullptr) {
      continue;
    }
    const bool upper_valid =
        std::isfinite(surface->upper) &&
        surface->upper - surface->ground >= config_.obstacle_min_height &&
        (config_.source == "global" || validRolling(*surface, true));
    const float z = (upper_valid ? surface->upper : surface->ground) -
                    static_cast<float>(robot.z);
    if (z < grid.spec.min_z || z > grid.spec.max_z) {
      continue;
    }
    grid.height[index] = z;
    grid.valid[index] = 1U;
    grid.variance[index] =
        config_.source == "global"
            ? static_cast<float>(config_.rolling_base_variance)
            : (upper_valid ? surface->upper_variance
                           : surface->ground_variance);
    grid.age[index] =
        config_.source == "global"
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
