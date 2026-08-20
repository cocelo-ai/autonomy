#include "autonomy_light/camera_frame_mapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
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

const sensor_msgs::msg::PointField *field(const sensor_msgs::msg::PointCloud2 &cloud,
                                           const std::string &name) {
  const auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                  [&name](const auto &candidate) {
                                    return candidate.name == name;
                                  });
  return found == cloud.fields.end() ? nullptr : &*found;
}

bool fields(const sensor_msgs::msg::PointCloud2 &cloud, FieldOffsets &out) {
  const auto *x = field(cloud, "x");
  const auto *y = field(cloud, "y");
  const auto *z = field(cloud, "z");
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
  const auto *variance = field(cloud, "variance");
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

}  // namespace

struct CameraFrameMapper::Impl {
  explicit Impl(CameraFrameMapper &node)
      : node(node), tf_buffer(node.get_clock()), tf_listener(tf_buffer) {
    input_topic = node.declare_parameter<std::string>("input_topic", "/camera/depth/points");
    output_topic = node.declare_parameter<std::string>("output_topic", "/camera/points_map");
    map_frame = node.declare_parameter<std::string>("map_frame", "map");
    base_frame = node.declare_parameter<std::string>("base_frame", "base_link");
    mount_frame = node.declare_parameter<std::string>("mount_frame", "F_camera_link");
    gravity_frame = node.declare_parameter<std::string>("gravity_frame", "base_link_gravity");
    local_gravity_mode = node.declare_parameter<bool>("local_gravity_mode", false);
    frame_override = node.declare_parameter<std::string>("frame_override", "");
    const auto source_to_mount_xyz = node.declare_parameter<std::vector<double>>(
        "source_to_mount_xyz", {0.0, 0.0, 0.0});
    const auto source_to_mount_rpy = node.declare_parameter<std::vector<double>>(
        "source_to_mount_rpy", {0.0, 0.0, 0.0});
    noise_stddev = node.declare_parameter<double>("noise_stddev", 0.025);
    min_range = node.declare_parameter<double>("min_range", 0.30);
    max_range = node.declare_parameter<double>("max_range", 3.0);
    max_points = node.declare_parameter<int>("max_points", 60000);
    tf_timeout_sec = node.declare_parameter<double>("tf_timeout_sec", 0.05);
    const auto valid_vector3 = [](const std::vector<double> &values) {
      return values.size() == 3U && std::all_of(values.begin(), values.end(),
          [](const double value) { return std::isfinite(value); });
    };
    if (input_topic.empty() || output_topic.empty() || base_frame.empty() || mount_frame.empty() ||
        (!local_gravity_mode && map_frame.empty()) ||
        (local_gravity_mode && (gravity_frame.empty() || gravity_frame == base_frame)) ||
        noise_stddev <= 0.0 || min_range < 0.0 ||
        max_range <= min_range || max_points <= 0 || tf_timeout_sec < 0.0 ||
        !valid_vector3(source_to_mount_xyz) || !valid_vector3(source_to_mount_rpy)) {
      throw std::invalid_argument("camera_frame_mapper parameters are invalid");
    }
    tf2::Quaternion source_rotation;
    source_rotation.setRPY(source_to_mount_rpy[0], source_to_mount_rpy[1],
                           source_to_mount_rpy[2]);
    mount_from_source.setOrigin(tf2::Vector3(
        source_to_mount_xyz[0], source_to_mount_xyz[1], source_to_mount_xyz[2]));
    mount_from_source.setRotation(source_rotation);
    publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, rclcpp::SensorDataQoS());
    subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) { onCloud(std::move(cloud)); });
    const auto &target_frame = local_gravity_mode ? gravity_frame : map_frame;
    RCLCPP_INFO(node.get_logger(),
                "Camera frame mapper: %s -> %s (%s via static mount %s, driver TF disabled, "
                "source optical RPY [%.3f, %.3f, %.3f])",
                input_topic.c_str(), output_topic.c_str(), target_frame.c_str(), mount_frame.c_str(),
                source_to_mount_rpy[0], source_to_mount_rpy[1], source_to_mount_rpy[2]);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (!cloud || (cloud->header.stamp.sec == 0 && cloud->header.stamp.nanosec == 0)) {
      return;
    }
    FieldOffsets offsets;
    const auto expected_bytes = static_cast<std::size_t>(cloud->point_step) *
                                cloud->width * cloud->height;
    if (!fields(*cloud, offsets) || cloud->point_step == 0 ||
        cloud->data.size() < expected_bytes) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Ignoring malformed camera cloud from %s", input_topic.c_str());
      return;
    }
    const std::string source_frame = frame_override.empty() ? cloud->header.frame_id : frame_override;
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Camera cloud has no frame_id");
      return;
    }
    const rclcpp::Time stamp(cloud->header.stamp);
    tf2::Transform target_from_source;
    std::string target_frame;
    try {
      if (local_gravity_mode) {
        const auto base_from_mount_message = tf_buffer.lookupTransform(
            base_frame, mount_frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_sec));
        const auto gravity_from_base_message = tf_buffer.lookupTransform(
            gravity_frame, base_frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_sec));
        tf2::Transform base_from_mount;
        tf2::Transform gravity_from_base;
        tf2::fromMsg(base_from_mount_message.transform, base_from_mount);
        tf2::fromMsg(gravity_from_base_message.transform, gravity_from_base);
        target_from_source = gravity_from_base * base_from_mount * mount_from_source;
        target_frame = gravity_frame;
      } else {
        tf_buffer.lookupTransform(base_frame, mount_frame, stamp,
                                  rclcpp::Duration::from_seconds(tf_timeout_sec));
        const auto transform = tf_buffer.lookupTransform(
            map_frame, mount_frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_sec));
        tf2::Transform map_from_mount;
        tf2::fromMsg(transform.transform, map_from_mount);
        target_from_source = map_from_mount * mount_from_source;
        target_frame = map_frame;
      }
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Waiting for camera transform into %s: %s",
                           (local_gravity_mode ? gravity_frame : map_frame).c_str(), error.what());
      return;
    }

    const auto point_count = static_cast<std::size_t>(cloud->width) * cloud->height;
    const float min_range_squared = static_cast<float>(min_range * min_range);
    const float max_range_squared = static_cast<float>(max_range * max_range);
    const float default_variance = static_cast<float>(noise_stddev * noise_stddev);
    std::vector<PointXYZVariance> mapped;
    mapped.reserve(std::min(point_count, static_cast<std::size_t>(max_points)));
    for (std::size_t index = 0; index < point_count &&
                                mapped.size() < static_cast<std::size_t>(max_points); ++index) {
      const auto *point = cloud->data.data() + index * cloud->point_step;
      const float x = readFloat(point, offsets.x);
      const float y = readFloat(point, offsets.y);
      const float z = readFloat(point, offsets.z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const float range_squared = x * x + y * y + z * z;
      if (range_squared < min_range_squared || range_squared > max_range_squared) {
        continue;
      }
      const tf2::Vector3 mapped_point = target_from_source * tf2::Vector3(x, y, z);
      if (!std::isfinite(mapped_point.x()) || !std::isfinite(mapped_point.y()) ||
          !std::isfinite(mapped_point.z())) {
        continue;
      }
      float variance = default_variance;
      if (offsets.has_variance) {
        const float supplied = readFloat(point, offsets.variance);
        if (std::isfinite(supplied) && supplied > 0.0F) {
          variance = supplied;
        }
      }
      mapped.push_back({static_cast<float>(mapped_point.x()), static_cast<float>(mapped_point.y()),
                        static_cast<float>(mapped_point.z()), variance});
    }
    if (mapped.empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp = cloud->header.stamp;
    output.header.frame_id = target_frame;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(mapped.size());
    output.is_bigendian = false;
    output.is_dense = true;
    output.point_step = sizeof(PointXYZVariance);
    output.row_step = output.point_step * output.width;
    output.fields.resize(4);
    constexpr std::array<const char *, 4> names{"x", "y", "z", "variance"};
    for (std::size_t index = 0; index < output.fields.size(); ++index) {
      output.fields[index].name = names[index];
      output.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
      output.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
      output.fields[index].count = 1;
    }
    output.data.resize(static_cast<std::size_t>(output.row_step));
    std::memcpy(output.data.data(), mapped.data(), output.data.size());
    publisher->publish(std::move(output));
  }

  CameraFrameMapper &node;
  std::string input_topic;
  std::string output_topic;
  std::string map_frame;
  std::string base_frame;
  std::string mount_frame;
  std::string gravity_frame;
  std::string frame_override;
  bool local_gravity_mode{false};
  tf2::Transform mount_from_source;
  double noise_stddev{0.025};
  double min_range{0.30};
  double max_range{3.0};
  int max_points{60000};
  double tf_timeout_sec{0.05};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
};

CameraFrameMapper::CameraFrameMapper(const rclcpp::NodeOptions &options)
    : Node("camera_frame_mapper", options), impl_(std::make_unique<Impl>(*this)) {}

CameraFrameMapper::~CameraFrameMapper() = default;

}  // namespace autonomy_light
