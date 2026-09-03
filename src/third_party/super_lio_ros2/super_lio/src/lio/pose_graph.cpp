#include "lio/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

namespace LI2Sup {
namespace {

constexpr int kRings = 20;
constexpr int kSectors = 60;
constexpr float kDescriptorRange = 80.0F;

gtsam::Key key(const std::size_t index) {
  return gtsam::Symbol('x', index);
}

gtsam::SharedNoiseModel noise(const double rotation, const double translation) {
  gtsam::Vector6 sigma;
  sigma << rotation, rotation, rotation, translation, translation, translation;
  return gtsam::noiseModel::Diagonal::Sigmas(sigma);
}

}  // namespace

PoseGraph::PoseGraph(Options options) : options_(options) {}

gtsam::Pose3 PoseGraph::toPose3(const BASIC::SE3& pose) {
  return {gtsam::Rot3(pose.R_.cast<double>()),
          gtsam::Point3(pose.t_[0], pose.t_[1], pose.t_[2])};
}

BASIC::SE3 PoseGraph::fromPose3(const gtsam::Pose3& pose) {
  const auto translation = pose.translation();
  return {BASIC::SO3(pose.rotation().matrix().cast<BASIC::scalar>()),
          BASIC::V3(translation.x(), translation.y(), translation.z())};
}

float PoseGraph::yaw(const BASIC::SE3& pose) {
  return std::atan2(pose.R_(1, 0), pose.R_(0, 0));
}

BASIC::CloudPtr PoseGraph::downsample(const BASIC::CloudPtr& cloud, const float leaf_size) {
  BASIC::CloudPtr result(new BASIC::PointCloudType());
  pcl::VoxelGrid<BASIC::PointType> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.filter(*result);
  return result;
}

PoseGraph::Descriptor PoseGraph::descriptor(const BASIC::CloudPtr& cloud) {
  Descriptor result(kRings * kSectors, -std::numeric_limits<float>::infinity());
  for (const auto& point : cloud->points) {
    const float range = std::hypot(point.x, point.y);
    if (!std::isfinite(range) || range >= kDescriptorRange) {
      continue;
    }
    const float angle = std::atan2(point.y, point.x) + static_cast<float>(M_PI);
    const int ring = std::min(kRings - 1, static_cast<int>(kRings * range / kDescriptorRange));
    const int sector = std::min(kSectors - 1, static_cast<int>(kSectors * angle / (2.0F * M_PI)));
    result[ring * kSectors + sector] = std::max(result[ring * kSectors + sector], point.z);
  }
  for (auto& value : result) {
    if (!std::isfinite(value)) {
      value = 0.0F;
    }
  }
  return result;
}

float PoseGraph::descriptorDistance(const Descriptor& first, const Descriptor& second) {
  float best = std::numeric_limits<float>::infinity();
  for (int shift = 0; shift < kSectors; ++shift) {
    float sum = 0.0F;
    for (int ring = 0; ring < kRings; ++ring) {
      for (int sector = 0; sector < kSectors; ++sector) {
        const float difference = first[ring * kSectors + sector] -
            second[ring * kSectors + (sector + shift) % kSectors];
        sum += difference * difference;
      }
    }
    best = std::min(best, std::sqrt(sum / static_cast<float>(first.size())));
  }
  return best;
}

void PoseGraph::accumulateLatest(const BASIC::SE3& odom_pose,
                                 const BASIC::CloudPtr& cloud,
                                 const double stamp) {
  if (keyframes_.empty() || !cloud || cloud->empty()) {
    return;
  }
  if (last_accumulation_stamp_ >= 0.0 && stamp >= last_accumulation_stamp_ &&
      stamp - last_accumulation_stamp_ < options_.accumulation_interval_sec) {
    return;
  }

  // Keep one pose-graph node per motion threshold, but do not throw away all
  // scans between nodes. Express each intervening scan in the latest
  // keyframe's local coordinates and voxel-merge it into that keyframe. Loop
  // optimization can then move the complete local surface as one rigid block.
  auto& latest = keyframes_.back();
  BASIC::PointCloudType transformed;
  pcl::transformPointCloud(*cloud, transformed,
                           (latest.odom_pose.inverse() * odom_pose).matrix());
  BASIC::CloudPtr merged(new BASIC::PointCloudType(*latest.cloud));
  *merged += transformed;
  latest.cloud = downsample(merged, options_.cloud_leaf_size);
  latest.descriptor = descriptor(latest.cloud);
  last_accumulation_stamp_ = stamp;
  ++accumulated_frame_count_;
  if (accumulated_frame_count_ % 20U == 0U) {
    LOG(INFO) << " ---> [PoseGraph] accumulated scans=" << accumulated_frame_count_
              << " keyframes=" << keyframes_.size()
              << " latest_points=" << latest.cloud->size();
  }
}

int PoseGraph::loopCandidate(const Descriptor& current) const {
  if (keyframes_.size() <= static_cast<std::size_t>(options_.min_loop_keyframes)) {
    return -1;
  }
  float best_score = options_.loop_descriptor_threshold;
  int best = -1;
  const std::size_t limit = keyframes_.size() - options_.min_loop_keyframes;
  for (std::size_t index = 0; index < limit; ++index) {
    const float score = descriptorDistance(current, keyframes_[index].descriptor);
    if (score < best_score) {
      best_score = score;
      best = static_cast<int>(index);
    }
  }
  return best;
}

bool PoseGraph::addLoopFactor(const int candidate, const Keyframe& current) {
  const auto& target = keyframes_[candidate];
  pcl::GeneralizedIterativeClosestPoint<BASIC::PointType, BASIC::PointType> gicp;
  gicp.setInputSource(current.cloud);
  gicp.setInputTarget(target.cloud);
  gicp.setMaxCorrespondenceDistance(options_.loop_max_correspondence);
  gicp.setMaximumIterations(32);
  gicp.setTransformationEpsilon(1.0e-4F);
  gicp.setEuclideanFitnessEpsilon(1.0e-4);
  BASIC::PointCloudType aligned;
  const BASIC::M4 initial = (target.odom_pose.inverse() * current.odom_pose).matrix();
  gicp.align(aligned, initial);
  if (!gicp.hasConverged() || gicp.getFitnessScore() > options_.loop_fitness_threshold) {
    return false;
  }
  const BASIC::M4 loop_transform = gicp.getFinalTransformation();
  gtsam::NonlinearFactorGraph factor;
  factor.add(gtsam::BetweenFactor<gtsam::Pose3>(
      key(candidate), key(keyframes_.size() - 1U),
      gtsam::Pose3(gtsam::Rot3(loop_transform.block<3, 3>(0, 0).cast<double>()),
                   gtsam::Point3(loop_transform(0, 3), loop_transform(1, 3),
                                 loop_transform(2, 3))),
      noise(0.03, 0.15)));
  isam_.update(factor);
  return true;
}

void PoseGraph::optimize(const BASIC::SE3& latest_odom) {
  estimate_ = isam_.calculateEstimate();
  pending_correction_ = fromPose3(estimate_.at<gtsam::Pose3>(key(keyframes_.size() - 1U))) *
      latest_odom.inverse();
  correction_pending_ = true;
}

void PoseGraph::addKeyframe(const BASIC::SE3& odom_pose, const BASIC::CloudPtr& cloud,
                            const double stamp) {
  if (!keyframes_.empty()) {
    const auto& previous = keyframes_.back().odom_pose;
    const float distance = (odom_pose.t_ - previous.t_).norm();
    const float heading = std::abs(std::remainder(yaw(odom_pose) - yaw(previous),
                                                   static_cast<float>(2.0 * M_PI)));
    if (distance < options_.keyframe_distance && heading < options_.keyframe_yaw_rad) {
      accumulateLatest(odom_pose, cloud, stamp);
      return;
    }
  }
  Keyframe current{odom_pose, downsample(cloud, options_.cloud_leaf_size), descriptor(cloud)};
  if (current.cloud->empty()) {
    return;
  }
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;
  const auto index = keyframes_.size();
  if (keyframes_.empty()) {
    factors.add(gtsam::PriorFactor<gtsam::Pose3>(key(index), toPose3(odom_pose), noise(1.0e-3, 1.0e-3)));
  } else {
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key(index - 1U), key(index), toPose3(keyframes_.back().odom_pose.inverse() * odom_pose),
        noise(0.05, 0.20)));
  }
  initial.insert(key(index), toPose3(odom_pose));
  isam_.update(factors, initial);
  const int candidate = loopCandidate(current.descriptor);
  keyframes_.push_back(std::move(current));
  last_accumulation_stamp_ = stamp;
  LOG(INFO) << " ---> [PoseGraph] keyframe=" << keyframes_.size()
            << " points=" << keyframes_.back().cloud->size();
  if (candidate >= 0 && addLoopFactor(candidate, keyframes_.back())) {
    optimize(odom_pose);
  }
  if (keyframes_.size() == 1U) {
    pending_correction_ = BASIC::SE3();
    correction_pending_ = true;
  }
}

bool PoseGraph::takeCorrection(const double stamp, BASIC::SE3& map_to_odom) {
  if (!correction_pending_ ||
      (last_correction_stamp_ >= 0.0 && stamp - last_correction_stamp_ < options_.correction_update_sec)) {
    return false;
  }
  last_correction_stamp_ = stamp;
  correction_pending_ = false;
  map_to_odom = pending_correction_;
  return true;
}

BASIC::CloudPtr PoseGraph::buildMap(const float leaf_size) const {
  BASIC::CloudPtr merged(new BASIC::PointCloudType());
  const gtsam::Values estimate = isam_.calculateEstimate();
  for (std::size_t index = 0; index < keyframes_.size(); ++index) {
    BASIC::PointCloudType transformed;
    pcl::transformPointCloud(*keyframes_[index].cloud, transformed,
                             fromPose3(estimate.at<gtsam::Pose3>(key(index))).matrix());
    *merged += transformed;
  }
  return downsample(merged, leaf_size);
}

BASIC::CloudPtr PoseGraph::buildLocalMap(const BASIC::V3& center, const float radius,
                                         const float leaf_size) const {
  BASIC::CloudPtr local(new BASIC::PointCloudType());
  if (radius <= 0.0F || keyframes_.empty()) {
    return local;
  }
  const float radius_squared = radius * radius;
  const gtsam::Values estimate = isam_.calculateEstimate();
  for (std::size_t index = 0; index < keyframes_.size(); ++index) {
    const auto pose = fromPose3(estimate.at<gtsam::Pose3>(key(index)));
    // A keyframe has points around its sensor origin.  The fixed 12 m margin
    // avoids dropping nearby terrain observed one keyframe before/after it,
    // while keeping the expensive point transform local to the robot.
    if ((pose.t_ - center).head<2>().squaredNorm() > (radius + 12.0F) * (radius + 12.0F)) {
      continue;
    }
    BASIC::PointCloudType transformed;
    pcl::transformPointCloud(*keyframes_[index].cloud, transformed, pose.matrix());
    for (const auto& point : transformed.points) {
      const float dx = point.x - center.x();
      const float dy = point.y - center.y();
      if (dx * dx + dy * dy <= radius_squared) {
        local->push_back(point);
      }
    }
  }
  return downsample(local, leaf_size);
}

}  // namespace LI2Sup
