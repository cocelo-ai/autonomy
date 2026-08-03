#pragma once

#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/point_cloud.h>

#include "basic/Manifold.h"
#include "basic/alias.h"

namespace LI2Sup {

class PoseGraph {
public:
  struct Options {
    float keyframe_distance{1.0F};
    float keyframe_yaw_rad{0.17F};
    float cloud_leaf_size{0.10F};
    float loop_descriptor_threshold{0.25F};
    float loop_fitness_threshold{0.35F};
    float loop_max_correspondence{2.5F};
    double correction_update_sec{1.0};
    int min_loop_keyframes{30};
  };

  explicit PoseGraph(Options options);

  void addKeyframe(const BASIC::SE3& odom_pose, const BASIC::CloudPtr& cloud,
                   double stamp);
  bool takeCorrection(double stamp, BASIC::SE3& map_to_odom);
  BASIC::CloudPtr buildMap(float leaf_size) const;

private:
  struct Keyframe {
    BASIC::SE3 odom_pose;
    BASIC::CloudPtr cloud;
    std::vector<float> descriptor;
  };

  using Descriptor = std::vector<float>;

  static gtsam::Pose3 toPose3(const BASIC::SE3& pose);
  static BASIC::SE3 fromPose3(const gtsam::Pose3& pose);
  static float yaw(const BASIC::SE3& pose);
  static BASIC::CloudPtr downsample(const BASIC::CloudPtr& cloud, float leaf_size);
  static Descriptor descriptor(const BASIC::CloudPtr& cloud);
  static float descriptorDistance(const Descriptor& first, const Descriptor& second);
  int loopCandidate(const Descriptor& current) const;
  bool addLoopFactor(int candidate, const Keyframe& current);
  void optimize(const BASIC::SE3& latest_odom);

  Options options_;
  gtsam::ISAM2 isam_;
  gtsam::Values estimate_;
  std::vector<Keyframe> keyframes_;
  BASIC::SE3 pending_correction_;
  double last_correction_stamp_{-1.0};
  bool correction_pending_{false};
};

}  // namespace LI2Sup
