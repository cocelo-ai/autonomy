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
  double rolling_max_age_sec{0.45};
  double rolling_upper_max_age_sec{0.25};
  double rolling_max_radius_m{2.0};
  double rolling_base_variance{0.0004};
  double rolling_range_variance_factor{0.0005};
  double rolling_process_variance_per_sec{0.0025};
  double rolling_max_variance{0.04};
  double rolling_outlier_variance{0.0025};
  double rolling_mahalanobis_threshold{3.0};
  double floor_radius_m{0.60};
};

class ElevationMapper {
public:
  explicit ElevationMapper(ElevationMapperConfig config = {});

  void configure(ElevationMapperConfig config);
  void integrate(const PclCloud &cloud, const Pose2_5D &robot,
                 double stamp_sec);
  void loadGlobalMap(const PclCloud &cloud);
  [[nodiscard]] bool empty() const;
  [[nodiscard]] HeightGrid build(const Pose2_5D &robot, double stamp_sec,
                                 const std_msgs::msg::Header &header) const;
  [[nodiscard]] PclCloud::Ptr globalMap() const;

private:
  struct Surface {
    float ground{std::numeric_limits<float>::quiet_NaN()};
    float upper{std::numeric_limits<float>::quiet_NaN()};
    float ground_variance{std::numeric_limits<float>::infinity()};
    float upper_variance{std::numeric_limits<float>::infinity()};
    double ground_time{-std::numeric_limits<double>::infinity()};
    double upper_time{-std::numeric_limits<double>::infinity()};
    std::uint32_t support{0U};
  };

  using SurfaceMap = std::unordered_map<CellKey, Surface, CellKeyHash>;
  using VoxelMap = std::unordered_map<VoxelKey, VoxelMean, VoxelKeyHash>;

  [[nodiscard]] CellKey keyFor(double x, double y) const;
  void updateSurfaces(SurfaceMap &target,
                      std::unordered_map<CellKey, std::vector<float>, CellKeyHash>
                          &samples,
                      const Pose2_5D &robot, double stamp_sec, bool rolling);
  void addVoxels(const PclCloud &cloud);
  [[nodiscard]] const Surface *nearestSurface(const SurfaceMap &surfaces,
                                              double x, double y) const;
  [[nodiscard]] bool validRolling(const Surface &surface, double stamp_sec,
                                  bool upper) const;
  [[nodiscard]] static float percentile(std::vector<float> &values,
                                        double fraction);
  static void fuse(float &mean, float &variance, double &last_time,
                   std::uint32_t &support, float measurement,
                   double measurement_variance, double stamp_sec,
                   const ElevationMapperConfig &config);

  ElevationMapperConfig config_;
  SurfaceMap global_surfaces_;
  SurfaceMap rolling_surfaces_;
  VoxelMap global_voxels_;
};

} // namespace autonomy_light
