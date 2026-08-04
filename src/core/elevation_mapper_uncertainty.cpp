#include "autonomy_light/elevation_mapper.hpp"

#include <Eigen/Dense>

namespace autonomy_light {
namespace {

using Matrix6 = Eigen::Matrix<double, 6, 6>;

Matrix6 poseCovariance(const Pose2_5D &pose, const ElevationMapperConfig &config) {
  Matrix6 covariance = Matrix6::Zero();
  if (pose.has_covariance) {
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        covariance(row, col) = pose.covariance[6 * row + col];
      }
    }
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  const double position_variance = std::pow(config.pose_min_position_std_m, 2);
  const double orientation_variance = std::pow(
      config.pose_min_orientation_std_deg * 0.01745329251994329577, 2);
  for (int index = 0; index < 3; ++index) {
    covariance(index, index) = std::max(covariance(index, index), position_variance);
    covariance(index + 3, index + 3) = std::max(covariance(index + 3, index + 3),
                                                 orientation_variance);
  }
  Eigen::SelfAdjointEigenSolver<Matrix6> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return Matrix6::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  return solver.eigenvectors() * solver.eigenvalues().cwiseMax(0.0).asDiagonal() *
      solver.eigenvectors().transpose();
}

template <int Size>
double maxStd(const Eigen::Matrix<double, Size, Size> &covariance) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, Size, Size>> solver(covariance);
  return solver.info() == Eigen::Success
      ? std::sqrt(std::max(0.0, solver.eigenvalues().maxCoeff()))
      : std::numeric_limits<double>::infinity();
}

} // namespace

void ElevationMapper::addRollingSamples(SampleMap &samples,
                                        const PointObservation &point,
                                        const Pose2_5D &robot) const {
  const Eigen::Vector3d lever(point.x - robot.x, point.y - robot.y, point.z - robot.z);
  const Matrix6 covariance = poseCovariance(robot, config_);
  if (!covariance.allFinite()) {
    return;
  }
  const double orientation_limit = config_.pose_max_orientation_std_deg *
      0.01745329251994329577;
  const Eigen::Matrix2d position_covariance = covariance.topLeftCorner<2, 2>();
  const Eigen::Matrix3d orientation_covariance = covariance.bottomRightCorner<3, 3>();
  if (maxStd(position_covariance) > config_.pose_max_position_std_m ||
      maxStd(orientation_covariance) > orientation_limit) {
    return;
  }

  const bool is_d435 = point.sensor_id == 1U;
  const double range2 = lever.squaredNorm();
  const double sensor_variance = (is_d435 ? config_.d435_base_variance
                                          : config_.rolling_base_variance) +
      (is_d435 ? config_.d435_range_variance_factor
               : config_.rolling_range_variance_factor) * range2;
  const double translation_std = is_d435 ? config_.d435_extrinsic_translation_std_m
                                         : config_.lidar_extrinsic_translation_std_m;
  const double rotation_std = (is_d435 ? config_.d435_extrinsic_rotation_std_deg
                                       : config_.lidar_extrinsic_rotation_std_deg) *
      0.01745329251994329577;
  const double calibration_variance = std::pow(translation_std, 2) +
      std::pow(rotation_std, 2) * (lever.x() * lever.x() + lever.y() * lever.y());

  Eigen::Matrix<double, 1, 6> height_jacobian = Eigen::Matrix<double, 1, 6>::Zero();
  height_jacobian(2) = 1.0;
  height_jacobian(3) = lever.y();
  height_jacobian(4) = -lever.x();
  const double height_variance = std::clamp(
      sensor_variance + calibration_variance +
          (height_jacobian * covariance * height_jacobian.transpose())(0, 0),
      1.0e-6, config_.rolling_max_variance);

  Eigen::Matrix<double, 2, 6> xy_jacobian = Eigen::Matrix<double, 2, 6>::Zero();
  xy_jacobian(0, 0) = 1.0;
  xy_jacobian(1, 1) = 1.0;
  xy_jacobian(0, 4) = lever.z();
  xy_jacobian(0, 5) = -lever.y();
  xy_jacobian(1, 3) = -lever.z();
  xy_jacobian(1, 5) = lever.x();
  Eigen::Matrix2d xy_covariance = xy_jacobian * covariance * xy_jacobian.transpose();
  xy_covariance.diagonal().array() += sensor_variance + calibration_variance;
  const double min_xy_variance = std::pow(0.25 * config_.grid.resolution, 2);
  xy_covariance.diagonal().array() += min_xy_variance;
  const double xy_std = maxStd(xy_covariance);
  if (!xy_covariance.allFinite() || xy_std > config_.pose_max_position_std_m) {
    return;
  }

  const CellKey center = keyFor(point.x, point.y);
  const int radius = std::min(config_.pose_splat_max_cells,
      xy_std <= 0.25 * config_.grid.resolution ? 0 :
          static_cast<int>(std::ceil(2.0 * xy_std / config_.grid.resolution)));
  const Eigen::Matrix2d inverse = xy_covariance.inverse();
  std::vector<std::pair<CellKey, double>> weights;
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      const Eigen::Vector2d offset(
          (static_cast<double>(center.x + x) + 0.5) * config_.grid.resolution - point.x,
          (static_cast<double>(center.y + y) + 0.5) * config_.grid.resolution - point.y);
      const double mahalanobis = offset.transpose() * inverse * offset;
      if (mahalanobis <= 4.0) {
        weights.push_back({{center.x + x, center.y + y}, std::exp(-0.5 * mahalanobis)});
      }
    }
  }
  if (weights.empty()) {
    weights.push_back({center, 1.0});
  }
  double total_weight = 0.0;
  for (const auto &[key, weight] : weights) {
    (void)key;
    total_weight += weight;
  }
  for (const auto &[key, weight] : weights) {
    const double probability = weight / std::max(1.0e-9, total_weight);
    samples[key].push_back({point.z, static_cast<float>(height_variance /
        std::max(0.01, probability))});
  }
}

} // namespace autonomy_light
