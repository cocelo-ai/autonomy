#pragma once

#include "autonomy_light/types.hpp"

namespace autonomy_light {

struct ElevationMapperConfig {
  GridSpec grid;
  std::string source{"rolling"};
  double global_voxel_size{0.03};
  std::size_t global_max_voxels{2'000'000U};
  int min_samples_per_cell{2};
  double ground_percentile{0.15};
  double obstacle_min_height{0.035};
  double rolling_max_radius_m{2.0};
  double rolling_base_variance{0.0004};
  double rolling_range_variance_factor{0.0005};
  double rolling_process_variance_per_sec{0.0025};
  double rolling_max_variance{0.04};
  double rolling_outlier_variance{0.0025};
  double rolling_mahalanobis_threshold{3.0};
  bool rolling_initial_prior_enabled{true};
  double rolling_initial_prior_radius_m{0.30};
  double rolling_initial_prior_ground_distance_m{0.48};
  double rolling_initial_prior_variance{0.04};
  bool visibility_cleanup_enabled{true};
  double visibility_max_ray_length_m{2.0};
  double visibility_min_ray_length_m{0.15};
  double visibility_min_observation_age_sec{0.08};
  double visibility_normal_alignment_min{0.15};
  double visibility_height_margin_m{0.015};
  double visibility_sigma_scale{1.0};
  bool overhang_filter_enabled{true};
  double overhang_near_max_above_sensor_m{0.20};
  double overhang_ramp_start_m{1.0};
  double overhang_ramp_slope{0.30};
  double overhang_absolute_max_above_sensor_m{1.0};
  double d435_base_variance{0.0016};
  double d435_range_variance_factor{0.002};
  double lidar_extrinsic_translation_std_m{0.005};
  double lidar_extrinsic_rotation_std_deg{0.5};
  double d435_extrinsic_translation_std_m{0.015};
  double d435_extrinsic_rotation_std_deg{1.0};
  double pose_min_position_std_m{0.015};
  double pose_min_orientation_std_deg{0.5};
  double pose_max_position_std_m{0.25};
  double pose_max_orientation_std_deg{10.0};
  int pose_splat_max_cells{2};
};

class ElevationMapper {
public:
  explicit ElevationMapper(ElevationMapperConfig config = {});

  void configure(ElevationMapperConfig config);
  void integrate(const PointObservations &cloud, const Pose2_5D &robot,
                 double stamp_sec);
  void loadGlobalMap(const PclCloud &cloud);
  [[nodiscard]] bool empty() const;
  [[nodiscard]] HeightGrid build(const Pose2_5D &robot, double stamp_sec,
                                 const std_msgs::msg::Header &header);
  [[nodiscard]] PclCloud::Ptr globalMap() const;

private:
  struct Sample {
    float z;
    float variance;
  };

  struct Surface {
    float ground{std::numeric_limits<float>::quiet_NaN()};
    float upper{std::numeric_limits<float>::quiet_NaN()};
    float ground_variance{std::numeric_limits<float>::infinity()};
    float upper_variance{std::numeric_limits<float>::infinity()};
    double ground_time{-std::numeric_limits<double>::infinity()};
    double upper_time{-std::numeric_limits<double>::infinity()};
    std::uint32_t support{0U};
  };

  struct RollingGrid {
    CellKey origin;
    int width{0};
    int height{0};
    bool initialized{false};
    std::vector<Surface> cells;
  };

  using SurfaceMap = std::unordered_map<CellKey, Surface, CellKeyHash>;
  using SampleMap =
      std::unordered_map<CellKey, std::vector<Sample>, CellKeyHash>;
  using VoxelMap = std::unordered_map<VoxelKey, VoxelMean, VoxelKeyHash>;

  [[nodiscard]] CellKey keyFor(double x, double y) const;
  void updateSurfaces(SurfaceMap &target, SampleMap &samples);
  void updateRollingSurfaces(SampleMap &samples, double stamp_sec);
  void addVoxels(const PointObservations &cloud);
  void addVoxels(const PclCloud &cloud);
  void addRollingSamples(SampleMap &samples, const PointObservation &point,
                         const Pose2_5D &robot) const;
  [[nodiscard]] bool acceptsRollingMeasurement(const PointObservation &point,
                                               const Pose2_5D &robot) const;
  void moveRollingGrid(const Pose2_5D &robot);
  [[nodiscard]] Surface *rollingSurface(const CellKey &key);
  [[nodiscard]] const Surface *rollingSurface(const CellKey &key) const;
  [[nodiscard]] const Surface *nearestRollingSurface(double x, double y) const;
  void cleanupVisibility(const PointObservations &cloud, const Pose2_5D &robot,
                         double stamp_sec);
  void seedInitialRollingPrior(const Pose2_5D &robot, double stamp_sec);
  [[nodiscard]] const Surface *nearestSurface(const SurfaceMap &surfaces,
                                              double x, double y) const;
  [[nodiscard]] bool validRolling(const Surface &surface, bool upper) const;
  [[nodiscard]] static float percentile(std::vector<float> &values,
                                        double fraction);
  [[nodiscard]] static Sample percentile(std::vector<Sample> &values,
                                         double fraction);
  static void fuse(float &mean, float &variance, double &last_time,
                   std::uint32_t &support, float measurement,
                   double measurement_variance, double stamp_sec,
                   const ElevationMapperConfig &config);

  ElevationMapperConfig config_;
  SurfaceMap global_surfaces_;
  RollingGrid rolling_grid_;
  VoxelMap global_voxels_;
  bool rolling_prior_seeded_{false};
};

} // namespace autonomy_light
