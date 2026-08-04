#include "autonomy_light/autonomy_light_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace autonomy_light {
namespace {

rclcpp::Time cloudStamp(const sensor_msgs::msg::PointCloud2 &cloud,
                        const rclcpp::Time &fallback) {
  return cloud.header.stamp.sec == 0 && cloud.header.stamp.nanosec == 0
             ? fallback
             : rclcpp::Time(cloud.header.stamp);
}

} // namespace

std::optional<nav_msgs::msg::Odometry>
AutonomyLightNode::odomAt(const rclcpp::Time &stamp) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (odom_history_.empty()) {
    return std::nullopt;
  }
  const nav_msgs::msg::Odometry *before = nullptr;
  const nav_msgs::msg::Odometry *after = nullptr;
  const nav_msgs::msg::Odometry *closest = nullptr;
  double closest_delta = std::numeric_limits<double>::infinity();
  for (const auto &odom : odom_history_) {
    const double delta = (rclcpp::Time(odom.header.stamp) - stamp).seconds();
    if (std::abs(delta) < closest_delta) {
      closest = &odom;
      closest_delta = std::abs(delta);
    }
    if (delta <= 0.0 && (!before || rclcpp::Time(before->header.stamp) <
                                         rclcpp::Time(odom.header.stamp))) {
      before = &odom;
    }
    if (delta >= 0.0 && (!after || rclcpp::Time(after->header.stamp) >
                                        rclcpp::Time(odom.header.stamp))) {
      after = &odom;
    }
  }
  const double duration = before && after
                              ? (rclcpp::Time(after->header.stamp) -
                                 rclcpp::Time(before->header.stamp))
                                    .seconds()
                              : 0.0;
  if (before && after && before != after && duration > 1.0e-9 &&
      (stamp - rclcpp::Time(before->header.stamp)).seconds() <=
          odom_sync_tolerance_sec_ &&
      (rclcpp::Time(after->header.stamp) - stamp).seconds() <=
          odom_sync_tolerance_sec_) {
    const double alpha = std::clamp(
        (stamp - rclcpp::Time(before->header.stamp)).seconds() / duration, 0.0,
        1.0);
    nav_msgs::msg::Odometry result = *before;
    result.header.stamp = stamp;
    const auto blend = [alpha](const double left, const double right) {
      return (1.0 - alpha) * left + alpha * right;
    };
    result.pose.pose.position.x = blend(before->pose.pose.position.x,
                                        after->pose.pose.position.x);
    result.pose.pose.position.y = blend(before->pose.pose.position.y,
                                        after->pose.pose.position.y);
    result.pose.pose.position.z = blend(before->pose.pose.position.z,
                                        after->pose.pose.position.z);
    tf2::Quaternion left;
    tf2::Quaternion right;
    tf2::fromMsg(before->pose.pose.orientation, left);
    tf2::fromMsg(after->pose.pose.orientation, right);
    left.normalize();
    right.normalize();
    result.pose.pose.orientation = tf2::toMsg(left.slerp(right, alpha));
    for (std::size_t index = 0; index < result.pose.covariance.size(); ++index) {
      result.pose.covariance[index] = blend(before->pose.covariance[index],
                                            after->pose.covariance[index]);
    }
    return result;
  }
  return closest && closest_delta <= odom_sync_tolerance_sec_ ? *closest
                                                                : std::nullopt;
}

bool AutonomyLightNode::integrateRegisteredCloud(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const nav_msgs::msg::Odometry &odom) {
  try {
    const rclcpp::Time stamp = cloudStamp(cloud, now());
    const auto observations = observationsFrom(cloud, odom);
    {
      std::lock_guard<std::mutex> lock(mapper_mutex_);
      mapper_.integrate(observations, poseOf(odom), stamp.seconds());
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++cloud_count_;
    }
    return true;
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cannot read Super-LIO cloud: %s", error.what());
    return false;
  }
}

void AutonomyLightNode::retryPendingClouds() {
  std::vector<std::pair<sensor_msgs::msg::PointCloud2::SharedPtr,
                        nav_msgs::msg::Odometry>> ready;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> pending;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> unmatched;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending.swap(pending_clouds_);
  }
  for (const auto &cloud : pending) {
    const auto odom = odomAt(cloudStamp(*cloud, now()));
    if (odom) {
      ready.emplace_back(cloud, *odom);
    } else {
      unmatched.push_back(cloud);
    }
  }
  if (!unmatched.empty()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_clouds_.insert(pending_clouds_.end(), unmatched.begin(),
                           unmatched.end());
    while (pending_clouds_.size() > pending_cloud_queue_size_) {
      pending_clouds_.pop_front();
    }
  }
  for (const auto &[cloud, odom] : ready) {
    integrateRegisteredCloud(*cloud, odom);
  }
}

void AutonomyLightNode::onOdom(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  if (!message || shutdown_requested_) {
    return;
  }
  const auto odom = baseOdom(*message);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_ = odom;
    if (latest_odom_.header.stamp.sec == 0 &&
        latest_odom_.header.stamp.nanosec == 0) {
      latest_odom_.header.stamp = now();
    }
    latest_odom_.header.frame_id = global_frame_;
    odom_history_.push_back(latest_odom_);
    while (odom_history_.size() > 64U) {
      odom_history_.pop_front();
    }
    has_odom_ = true;
    ++odom_count_;
    path_.header = latest_odom_.header;
    if (path_.poses.empty() ||
        std::hypot(path_.poses.back().pose.position.x -
                       latest_odom_.pose.pose.position.x,
                   path_.poses.back().pose.position.y -
                       latest_odom_.pose.pose.position.y) > 0.02) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = latest_odom_.header;
      pose.pose = latest_odom_.pose.pose;
      path_.poses.push_back(std::move(pose));
      ++path_version_;
      if (path_.poses.size() > 10'000U) {
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 1'000);
      }
    }
  }
  retryPendingClouds();
}

void AutonomyLightNode::onRegisteredCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr message) {
  if (!message || shutdown_requested_) {
    return;
  }
  const auto odom = odomAt(cloudStamp(*message, now()));
  if (odom) {
    integrateRegisteredCloud(*message, *odom);
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  pending_clouds_.push_back(message);
  if (pending_clouds_.size() > pending_cloud_queue_size_) {
    pending_clouds_.pop_front();
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping cloud after %zu unmatched odometry samples (tolerance %.3fs)",
        pending_cloud_queue_size_, odom_sync_tolerance_sec_);
  }
}

} // namespace autonomy_light
