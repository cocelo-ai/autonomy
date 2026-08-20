#include "autonomy_light/observation_merge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy_light {
namespace {

struct PointXYZVariance {
  float x;
  float y;
  float z;
  float variance;
};
static_assert(sizeof(PointXYZVariance) == 4U * sizeof(float));

struct FieldOffsets {
  std::uint32_t x{0};
  std::uint32_t y{0};
  std::uint32_t z{0};
  std::uint32_t variance{0};
  bool has_variance{false};
};

const sensor_msgs::msg::PointField *findField(
    const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) {
  const auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                  [&name](const auto &field) {
                                    return field.name == name;
                                  });
  return found == cloud.fields.end() ? nullptr : &*found;
}

bool fields(const sensor_msgs::msg::PointCloud2 &cloud, FieldOffsets &out) {
  const auto *x = findField(cloud, "x");
  const auto *y = findField(cloud, "y");
  const auto *z = findField(cloud, "z");
  if (!x || !y || !z || x->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      y->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      z->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      x->offset + sizeof(float) > cloud.point_step ||
      y->offset + sizeof(float) > cloud.point_step ||
      z->offset + sizeof(float) > cloud.point_step) {
    return false;
  }
  out.x = x->offset;
  out.y = y->offset;
  out.z = z->offset;
  const auto *variance = findField(cloud, "variance");
  out.has_variance = variance &&
                     variance->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                     variance->offset + sizeof(float) <= cloud.point_step;
  if (out.has_variance) {
    out.variance = variance->offset;
  }
  return true;
}

float readFloat(const std::uint8_t *point, const std::uint32_t offset) {
  float value = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(&value, point + offset, sizeof(value));
  return value;
}

struct VoxelKey {
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t result = static_cast<std::uint32_t>(key.x);
    result = (result * 0x9e3779b9U) ^ static_cast<std::uint32_t>(key.y);
    result = (result * 0x9e3779b9U) ^ static_cast<std::uint32_t>(key.z);
    return result;
  }
};

struct VoxelPoint {
  PointXYZVariance point;
  float range_squared;
};

}  // namespace

struct ObservationMerge::Impl {
  struct Source {
    std::string name;
    std::string type;
    std::string topic;
    std::string mount_frame;
    std::string frame_override;
    bool validate_mount_tf{true};
    double noise_stddev{0.02};
    double min_range{0.0};
    std::size_t max_points{50000};
    std::size_t queue_size{12};
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> queue;
    std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> lidar_queue;
    double lidar_window_sec{0.040};
    double lidar_deskew_bin_sec{0.002};
    std::uint64_t received_count{0};
    std::uint64_t stale_drop_count{0};
    std::uint64_t sync_drop_count{0};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_subscription;
  };

  struct SelectedSource {
    Source *source;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  };

  explicit Impl(ObservationMerge &node)
      : node(node), tf_buffer(node.get_clock()), tf_listener(tf_buffer) {
    loadParameters();
    createIo();
  }

  void loadParameters() {
    base_frame = node.declare_parameter<std::string>("base_frame", "base_link");
    map_frame = node.declare_parameter<std::string>("map_frame", "map");
    target_frame = node.declare_parameter<std::string>("target_frame", "base_link_gravity");
    output_topic = node.declare_parameter<std::string>(
        "output_topic", "/autonomy_light/merged_observations");
    publish_rate_hz = node.declare_parameter<double>("publish_rate_hz", 30.0);
    max_cloud_age_sec = node.declare_parameter<double>("max_cloud_age_sec", 0.15);
    sync_require_all_sources = node.declare_parameter<bool>(
        "sync.require_all_sources", true);
    sync_slop_sec = node.declare_parameter<double>("sync.slop_sec", 0.012);
    sync_wait_for_complete_window = node.declare_parameter<bool>(
        "sync.wait_for_complete_window", true);
    sync_single_source_passthrough = node.declare_parameter<bool>(
        "sync.single_source_passthrough", true);
    const int sync_queue_size = node.declare_parameter<int>("sync.queue_size", 12);
    tf_timeout_sec = node.declare_parameter<double>("tf_timeout_sec", 0.015);
    roi_x_min = node.declare_parameter<double>("roi.x_min", -1.50);
    roi_x_max = node.declare_parameter<double>("roi.x_max", 1.50);
    roi_y_min = node.declare_parameter<double>("roi.y_min", -1.00);
    roi_y_max = node.declare_parameter<double>("roi.y_max", 1.00);
    roi_z_min = node.declare_parameter<double>("roi.z_min", -1.20);
    roi_z_max = node.declare_parameter<double>("roi.z_max", 0.80);
    roi_max_range = node.declare_parameter<double>("roi.max_range", 3.0);
    voxel_size = node.declare_parameter<double>("roi.voxel_size", 0.02);
    diagnostics_enabled = node.declare_parameter<bool>("diagnostics.enabled", false);
    const auto source_names = node.declare_parameter<std::vector<std::string>>(
        "inputs", std::vector<std::string>{"camera"});

    if (base_frame.empty() || map_frame.empty() || target_frame.empty() ||
        output_topic.empty() || publish_rate_hz <= 0.0 || tf_timeout_sec < 0.0 ||
        roi_x_max <= roi_x_min || roi_y_max <= roi_y_min ||
        roi_z_max <= roi_z_min || roi_max_range <= 0.0 || voxel_size <= 0.0 ||
        sync_slop_sec < 0.0 || sync_queue_size <= 0) {
      throw std::invalid_argument("observation_merge geometry and timing parameters are invalid");
    }

    sources.clear();
    for (const auto &name : source_names) {
      if (name.empty()) {
        continue;
      }
      if (std::any_of(sources.begin(), sources.end(), [&name](const Source &source) {
            return source.name == name;
          })) {
        throw std::invalid_argument("observation_merge inputs contains duplicate source '" + name + "'");
      }
      Source source;
      source.name = name;
      source.type = node.declare_parameter<std::string>(
          "sources." + name + ".type", "camera");
      source.topic = node.declare_parameter<std::string>(
          "sources." + name + ".topic", "/" + name + "/points");
      source.mount_frame = node.declare_parameter<std::string>(
          "sources." + name + ".mount_frame", "");
      source.frame_override = node.declare_parameter<std::string>(
          "sources." + name + ".frame_override", "");
      source.validate_mount_tf = node.declare_parameter<bool>(
          "sources." + name + ".validate_mount_tf", true);
      source.noise_stddev = node.declare_parameter<double>(
          "sources." + name + ".noise_stddev", source.noise_stddev);
      source.min_range = node.declare_parameter<double>(
          "sources." + name + ".min_range", source.min_range);
      const int max_points = node.declare_parameter<int>(
          "sources." + name + ".max_points", static_cast<int>(source.max_points));
      source.max_points = static_cast<std::size_t>(std::max(1, max_points));
      const int source_queue_size = node.declare_parameter<int>(
          "sources." + name + ".queue_size", sync_queue_size);
      source.queue_size = static_cast<std::size_t>(std::max(1, source_queue_size));
      if (source.type != "camera" || source.topic.empty() || source.mount_frame.empty() ||
          source.noise_stddev <= 0.0 ||
          source.min_range < 0.0) {
        throw std::invalid_argument(
            "observation_merge source '" + name + "' must be a valid camera source");
      }
      sources.emplace_back(std::move(source));
    }
    if (sources.empty()) {
      throw std::invalid_argument("observation_merge has no inputs");
    }
  }

  void createIo() {
    output = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, rclcpp::SensorDataQoS());
    if (diagnostics_enabled) {
      heartbeat = node.create_publisher<std_msgs::msg::String>(
          "/autonomy_light/heartbeat/observation_merge", rclcpp::QoS(10));
      sync_skew = node.create_publisher<std_msgs::msg::Float32>(
          "/autonomy_light/observation_sync_skew_ms", rclcpp::SensorDataQoS());
      processing_time = node.create_publisher<std_msgs::msg::Float32>(
          "/autonomy_light/observation_processing_ms", rclcpp::SensorDataQoS());
    }
    for (auto &source : sources) {
      source.subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
          source.topic, rclcpp::SensorDataQoS(),
          [this, name = source.name](sensor_msgs::msg::PointCloud2::SharedPtr message) {
            if (!message || (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0)) {
              RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                                   "Dropping unstamped cloud from '%s'; strict synchronization requires ROS timestamps",
                                   name.c_str());
              return;
            }
            bool publish_immediately = false;
            {
              std::lock_guard<std::mutex> lock(source_mutex);
              for (auto &candidate : sources) {
                if (candidate.name == name) {
                  ++candidate.received_count;
                  candidate.queue.emplace_back(std::move(message));
                  while (candidate.queue.size() > candidate.queue_size) {
                    candidate.queue.pop_front();
                    ++candidate.sync_drop_count;
                  }
                  // A single selected camera never enters the cohort state
                  // machine. Process its own timestamp in this callback so a
                  // second sensor can neither delay nor replace this sample.
                  publish_immediately = sources.size() == 1U &&
                                        sync_single_source_passthrough;
                  break;
                }
              }
            }
            if (publish_immediately) {
              publishCurrentObservation();
            }
          });
      RCLCPP_INFO(node.get_logger(),
                  "Observation camera '%s': %s (static mount %s, noise %.3fm, min range %.2fm, queue %zu)",
                  source.name.c_str(), source.topic.c_str(), source.mount_frame.c_str(), source.noise_stddev,
                  source.min_range, source.queue_size);
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer = node.create_wall_timer(period, [this]() { publishCurrentObservation(); });
    if (diagnostics_enabled) {
      heartbeat_timer = node.create_wall_timer(
          std::chrono::milliseconds(500), [this]() { publishHeartbeat(); });
    }
    RCLCPP_INFO(node.get_logger(),
                "Observation merge: %s -> %s in %s @ %.1f Hz, strict=%s, single-source=%s, "
                "complete-window=%s, sync slop %.1fms, ROI x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f], voxel %.3fm",
                base_frame.c_str(), output_topic.c_str(), target_frame.c_str(),
                publish_rate_hz,
                sync_require_all_sources ? "true" : "false",
                sync_single_source_passthrough ? "passthrough" : "cohort",
                sync_wait_for_complete_window ? "true" : "false", sync_slop_sec * 1000.0,
                roi_x_min, roi_x_max,
                roi_y_min, roi_y_max, roi_z_min, roi_z_max, voxel_size);
  }

  void publishHeartbeat() {
    if (!heartbeat) {
      return;
    }
    std_msgs::msg::String message;
    if (published_count == 0) {
      message.data = "waiting_for_synchronized_observation";
    } else {
      message.data = "ready matched=" + std::to_string(published_count) +
                     " skew_ms=" + std::to_string(last_sync_skew_sec * 1000.0) +
                     " sync_drops=" + std::to_string(sync_drop_count);
    }
    heartbeat->publish(message);
  }

  bool isFresh(const sensor_msgs::msg::PointCloud2 &cloud,
               const rclcpp::Time &now) const {
    const rclcpp::Time stamp(cloud.header.stamp);
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    if (max_cloud_age_sec <= 0.0) {
      return true;
    }
    const double age = (now - stamp).seconds();
    return age <= max_cloud_age_sec && age >= -max_cloud_age_sec;
  }

  bool fusionFrameFromBase(const rclcpp::Time &stamp, tf2::Transform &target_from_base,
                           tf2::Transform &base_from_map) {
    target_from_base.setIdentity();
    try {
      const auto map_from_base_message = tf_buffer.lookupTransform(
          map_frame, base_frame, stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec));
      tf2::Transform map_from_base;
      tf2::fromMsg(map_from_base_message.transform, map_from_base);
      base_from_map = map_from_base.inverse();
      tf2::Quaternion map_from_base_rotation = map_from_base.getRotation();
      map_from_base_rotation.normalize();
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(map_from_base_rotation).getRPY(roll, pitch, yaw);
      tf2::Quaternion map_from_target;
      map_from_target.setRPY(0.0, 0.0, yaw);
      target_from_base.setRotation(map_from_target.inverse() * map_from_base_rotation);
      return true;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for gravity reference TF %s -> %s: %s",
                           map_frame.c_str(), base_frame.c_str(), error.what());
      return false;
    }
  }

  bool appendSource(Source &source,
                    const sensor_msgs::msg::PointCloud2 &cloud,
                    const rclcpp::Time &source_stamp,
                    const tf2::Transform &target_from_base,
                    const tf2::Transform &base_from_map,
                    std::unordered_map<VoxelKey, VoxelPoint, VoxelKeyHash> &voxels,
                    std::size_t &accepted) {
    FieldOffsets offsets;
    if (!fields(cloud, offsets) || cloud.point_step == 0 ||
        cloud.data.size() < static_cast<std::size_t>(cloud.point_step) * cloud.width * cloud.height) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Dropping malformed cloud from '%s'", source.name.c_str());
      return false;
    }
    const std::string source_frame = source.frame_override.empty()
                                         ? cloud.header.frame_id
                                         : source.frame_override;
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Cloud '%s' has no frame_id", source.name.c_str());
      return false;
    }
    if (source.validate_mount_tf) {
      // Raw source clouds are anchored to the named static camera mount. Mapped
      // map-frame clouds have already passed this check in camera_frame_mapper.
      try {
        tf_buffer.lookupTransform(base_frame, source.mount_frame, source_stamp,
                                  rclcpp::Duration::from_seconds(tf_timeout_sec));
        tf_buffer.lookupTransform(source.mount_frame, source_frame, source_stamp,
                                  rclcpp::Duration::from_seconds(tf_timeout_sec));
      } catch (const tf2::TransformException &error) {
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                             "Waiting for configured camera TF %s -> %s -> %s for '%s': %s",
                             base_frame.c_str(), source.mount_frame.c_str(), source_frame.c_str(),
                             source.name.c_str(), error.what());
        return false;
      }
    }
    tf2::Transform map_from_source;
    map_from_source.setIdentity();
    if (source_frame != map_frame) {
      try {
        const auto transform_message = tf_buffer.lookupTransform(
            map_frame, source_frame, source_stamp,
            rclcpp::Duration::from_seconds(tf_timeout_sec));
        tf2::fromMsg(transform_message.transform, map_from_source);
      } catch (const tf2::TransformException &error) {
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                             "Waiting for TF %s -> %s at observation stamp for '%s': %s",
                             map_frame.c_str(), source_frame.c_str(), source.name.c_str(),
                             error.what());
        return false;
      }
    }
    const tf2::Transform target_from_source = target_from_base * base_from_map * map_from_source;

    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    const float source_variance = static_cast<float>(source.noise_stddev * source.noise_stddev);
    const float maximum_range_squared = static_cast<float>(roi_max_range * roi_max_range);
    std::size_t source_accepted = 0;
    bool had_usable_point = false;
    for (std::size_t index = 0; index < point_count && source_accepted < source.max_points; ++index) {
      const auto *input = cloud.data.data() + index * cloud.point_step;
      const float x = readFloat(input, offsets.x);
      const float y = readFloat(input, offsets.y);
      const float z = readFloat(input, offsets.z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const float sensor_range_squared = x * x + y * y + z * z;
      if (sensor_range_squared < static_cast<float>(source.min_range * source.min_range)) {
        continue;
      }
      const tf2::Vector3 in_target = target_from_source * tf2::Vector3(x, y, z);
      const float px = static_cast<float>(in_target.x());
      const float py = static_cast<float>(in_target.y());
      const float pz = static_cast<float>(in_target.z());
      const float range_squared = px * px + py * py + pz * pz;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
          range_squared > maximum_range_squared || px < roi_x_min || px >= roi_x_max ||
          py < roi_y_min || py >= roi_y_max || pz < roi_z_min || pz > roi_z_max) {
        continue;
      }
      had_usable_point = true;
      float variance = source_variance;
      if (offsets.has_variance) {
        const float supplied = readFloat(input, offsets.variance);
        if (std::isfinite(supplied) && supplied > 0.0F) {
          variance = supplied;
        }
      }
      const VoxelKey key{
          static_cast<std::int32_t>(std::floor((px - roi_x_min) / voxel_size)),
          static_cast<std::int32_t>(std::floor((py - roi_y_min) / voxel_size)),
          static_cast<std::int32_t>(std::floor((pz - roi_z_min) / voxel_size))};
      const VoxelPoint candidate{{px, py, pz, variance}, range_squared};
      const auto inserted = voxels.emplace(key, candidate);
      if (!inserted.second && candidate.point.variance < inserted.first->second.point.variance) {
        inserted.first->second = candidate;
      }
      ++source_accepted;
    }
    accepted += source_accepted;
    // A strict cohort is valid if every source supplied a well-formed,
    // in-ROI observation at the matching stamp. All accepted points from the
    // synchronized source frames are retained; no cross-sensor gate rejects
    // camera data merely because it differs from a LiDAR height.
    return had_usable_point;
  }

  bool takeSynchronizedObservation(const rclcpp::Time &now,
                                   std::vector<SelectedSource> &selected,
                                   rclcpp::Time &fusion_stamp,
                                   double &skew_sec) {
    std::lock_guard<std::mutex> lock(source_mutex);
    for (auto &source : sources) {
      while (!source.queue.empty() && !isFresh(*source.queue.front(), now)) {
        source.queue.pop_front();
        ++source.stale_drop_count;
      }
    }

    if (sources.empty()) {
      return false;
    }
    if (sources.size() == 1U && sync_single_source_passthrough) {
      auto &source = sources.front();
      if (source.queue.empty()) {
        return false;
      }
      const auto cloud = source.queue.front();
      source.queue.pop_front();
      selected.push_back(SelectedSource{&source, cloud});
      fusion_stamp = rclcpp::Time(cloud->header.stamp);
      skew_sec = 0.0;
      return true;
    }
    if (!sync_require_all_sources) {
      for (auto &source : sources) {
        if (!source.queue.empty()) {
          selected.push_back(SelectedSource{&source, source.queue.front()});
          source.queue.pop_front();
        }
      }
      if (selected.empty()) {
        return false;
      }
      fusion_stamp = rclcpp::Time(selected.front().cloud->header.stamp);
      skew_sec = 0.0;
      return true;
    }

    // Evaluate candidate epochs from every selected camera.  The chosen cohort
    // minimizes the pairwise timestamp span, rather than privileging the first
    // configured camera.  Before committing it, wait until all camera queues
    // contain data through the end of its matching window.  This prevents a
    // late frame from a second/third camera from changing the chosen cohort.
    for (;;) {
      if (std::any_of(sources.begin(), sources.end(), [](const Source &source) {
            return source.queue.empty();
          })) {
        return false;
      }

      struct Cohort {
        std::vector<std::size_t> indices;
        rclcpp::Time min_stamp{0, 0, RCL_ROS_TIME};
        rclcpp::Time max_stamp{0, 0, RCL_ROS_TIME};
        double span_sec{std::numeric_limits<double>::infinity()};
        bool valid{false};
      } best;
      bool has_pending_cohort = false;

      for (const auto &anchor_source : sources) {
        for (const auto &anchor_cloud : anchor_source.queue) {
          const rclcpp::Time anchor_stamp(anchor_cloud->header.stamp);
          Cohort candidate;
          candidate.indices.resize(sources.size(), 0U);
          candidate.min_stamp = anchor_stamp;
          candidate.max_stamp = anchor_stamp;
          bool match = true;
          for (std::size_t source_index = 0; source_index < sources.size(); ++source_index) {
            const auto &source = sources[source_index];
            std::size_t nearest_index = 0U;
            double nearest_distance = std::numeric_limits<double>::infinity();
            for (std::size_t queue_index = 0; queue_index < source.queue.size(); ++queue_index) {
              const rclcpp::Time stamp(source.queue[queue_index]->header.stamp);
              const double distance = std::fabs((stamp - anchor_stamp).seconds());
              if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_index = queue_index;
              }
            }
            if (nearest_distance > sync_slop_sec) {
              match = false;
              break;
            }
            const rclcpp::Time matched_stamp(source.queue[nearest_index]->header.stamp);
            candidate.indices[source_index] = nearest_index;
            if (matched_stamp < candidate.min_stamp) {
              candidate.min_stamp = matched_stamp;
            }
            if (matched_stamp > candidate.max_stamp) {
              candidate.max_stamp = matched_stamp;
            }
          }
          if (!match) {
            continue;
          }
          candidate.span_sec = (candidate.max_stamp - candidate.min_stamp).seconds();
          if (candidate.span_sec > sync_slop_sec) {
            continue;
          }
          candidate.valid = true;
          bool complete_window = true;
          if (sync_wait_for_complete_window) {
            for (const auto &source : sources) {
              const rclcpp::Time newest_stamp(source.queue.back()->header.stamp);
              if ((newest_stamp - candidate.max_stamp).seconds() < sync_slop_sec) {
                complete_window = false;
                break;
              }
            }
          }
          if (!complete_window) {
            has_pending_cohort = true;
            continue;
          }
          if (!best.valid || candidate.span_sec < best.span_sec ||
              (candidate.span_sec == best.span_sec && candidate.max_stamp > best.max_stamp)) {
            best = std::move(candidate);
          }
        }
      }

      if (best.valid) {
        selected.reserve(sources.size());
        fusion_stamp = best.max_stamp;
        for (std::size_t source_index = 0; source_index < sources.size(); ++source_index) {
          auto &source = sources[source_index];
          const std::size_t selected_index = best.indices[source_index];
          const auto cloud = source.queue[selected_index];
          selected.push_back(SelectedSource{&source, cloud});
          for (std::size_t queue_index = 0; queue_index <= selected_index; ++queue_index) {
            if (queue_index < selected_index) {
              ++source.sync_drop_count;
              ++sync_drop_count;
            }
            source.queue.pop_front();
          }
        }
        skew_sec = best.span_sec;
        return true;
      }
      if (has_pending_cohort) {
        return false;
      }

      // There is no matching cohort.  Discard the globally oldest sample only
      // after every stream has progressed past its possible matching window;
      // otherwise a delayed camera can still complete it.
      std::size_t oldest_source_index = 0U;
      rclcpp::Time oldest_stamp(sources.front().queue.front()->header.stamp);
      for (std::size_t source_index = 1; source_index < sources.size(); ++source_index) {
        const rclcpp::Time stamp(sources[source_index].queue.front()->header.stamp);
        if (stamp < oldest_stamp) {
          oldest_stamp = stamp;
          oldest_source_index = source_index;
        }
      }
      const bool all_streams_past_window = std::all_of(
          sources.begin(), sources.end(), [&oldest_stamp, this](const Source &source) {
            return (rclcpp::Time(source.queue.back()->header.stamp) - oldest_stamp).seconds() >=
                   sync_slop_sec;
          });
      if (!all_streams_past_window) {
        return false;
      }
      auto &oldest_source = sources[oldest_source_index];
      oldest_source.queue.pop_front();
      ++oldest_source.sync_drop_count;
      ++sync_drop_count;

      // Continue so consecutive unusable samples are pruned in one timer tick.
    }
  }

  void publishCurrentObservation() {
    const auto processing_started = std::chrono::steady_clock::now();
    const auto now = node.get_clock()->now();
    std::vector<SelectedSource> current;
    rclcpp::Time fusion_stamp(0, 0, now.get_clock_type());
    double sync_skew_sec = 0.0;
    if (!takeSynchronizedObservation(now, current, fusion_stamp, sync_skew_sec)) {
      return;
    }

    tf2::Transform target_from_base;
    tf2::Transform base_from_map;
    if (!fusionFrameFromBase(fusion_stamp, target_from_base, base_from_map)) {
      return;
    }

    std::unordered_map<VoxelKey, VoxelPoint, VoxelKeyHash> voxels;
    voxels.reserve(50000);
    std::size_t accepted = 0;
    std::size_t sources_used = 0;
    for (const auto &item : current) {
      const rclcpp::Time source_stamp(item.cloud->header.stamp);
      if (appendSource(*item.source, *item.cloud, source_stamp, target_from_base, base_from_map,
                       voxels, accepted)) {
        ++sources_used;
      }
    }
    if (sources_used == 0 || voxels.empty() ||
        (sync_require_all_sources && sources_used != current.size())) {
      if (sync_require_all_sources && sources_used != current.size()) {
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                             "Dropping synchronized cohort because only %zu/%zu source transforms were usable",
                             sources_used, current.size());
      }
      return;
    }
    std::vector<PointXYZVariance> points;
    points.reserve(voxels.size());
    for (const auto &entry : voxels) {
      points.push_back(entry.second.point);
    }
    sensor_msgs::msg::PointCloud2 output_cloud;
    output_cloud.header.stamp = fusion_stamp;
    output_cloud.header.frame_id = target_frame;
    output_cloud.height = 1;
    output_cloud.width = static_cast<std::uint32_t>(points.size());
    output_cloud.is_bigendian = false;
    output_cloud.is_dense = true;
    output_cloud.point_step = sizeof(PointXYZVariance);
    output_cloud.row_step = output_cloud.point_step * output_cloud.width;
    output_cloud.fields.resize(4);
    const std::array<const char *, 4> names{"x", "y", "z", "variance"};
    for (std::size_t field = 0; field < output_cloud.fields.size(); ++field) {
      output_cloud.fields[field].name = names[field];
      output_cloud.fields[field].offset = static_cast<std::uint32_t>(field * sizeof(float));
      output_cloud.fields[field].datatype = sensor_msgs::msg::PointField::FLOAT32;
      output_cloud.fields[field].count = 1;
    }
    output_cloud.data.resize(static_cast<std::size_t>(output_cloud.row_step));
    if (!points.empty()) {
      std::memcpy(output_cloud.data.data(), points.data(), output_cloud.data.size());
    }
    output->publish(std::move(output_cloud));
    ++published_count;
    last_sync_skew_sec = sync_skew_sec;
    std_msgs::msg::Float32 skew_message;
    skew_message.data = static_cast<float>(sync_skew_sec * 1000.0);
    if (sync_skew) {
      sync_skew->publish(skew_message);
    }
    std_msgs::msg::Float32 processing_message;
    processing_message.data = static_cast<float>(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   processing_started).count());
    if (processing_time) {
      processing_time->publish(processing_message);
    }
    RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                         "Synchronized epoch: sources=%zu skew=%.2fms raw_roi=%zu voxelized=%zu",
                         sources_used, sync_skew_sec * 1000.0, accepted, points.size());
  }

  ObservationMerge &node;
  std::string base_frame;
  std::string map_frame;
  std::string target_frame;
  std::string output_topic;
  double publish_rate_hz{30.0};
  double max_cloud_age_sec{0.15};
  bool sync_require_all_sources{true};
  double sync_slop_sec{0.012};
  bool sync_wait_for_complete_window{true};
  bool sync_single_source_passthrough{true};
  double tf_timeout_sec{0.015};
  double roi_x_min{-1.5};
  double roi_x_max{1.5};
  double roi_y_min{-1.0};
  double roi_y_max{1.0};
  double roi_z_min{-1.2};
  double roi_z_max{0.8};
  double roi_max_range{3.0};
  double voxel_size{0.02};
  bool diagnostics_enabled{false};
  std::vector<Source> sources;
  std::mutex source_mutex;
  std::uint64_t published_count{0};
  std::uint64_t sync_drop_count{0};
  double last_sync_skew_sec{0.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sync_skew;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
};

ObservationMerge::ObservationMerge(const rclcpp::NodeOptions &options)
    : Node("observation_merge", options) {
  impl_ = std::make_unique<Impl>(*this);
}

ObservationMerge::~ObservationMerge() = default;

}  // namespace autonomy_light
