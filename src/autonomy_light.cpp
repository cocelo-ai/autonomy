#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autonomy_light/msg/height_map.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>
#endif
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace autonomy_light
{
namespace
{

#if !defined(RCLCPP_VERSION_GTE)
#define AUTONOMY_LIGHT_HAS_RCLCPP_DOMAIN_ID_API 0
#elif RCLCPP_VERSION_GTE(8, 0, 0)
#define AUTONOMY_LIGHT_HAS_RCLCPP_DOMAIN_ID_API 1
#else
#define AUTONOMY_LIGHT_HAS_RCLCPP_DOMAIN_ID_API 0
#endif

int rosDomainIdFromEnvironment()
{
  const char * value = std::getenv("ROS_DOMAIN_ID");
  if (value == nullptr || value[0] == '\0') {
    return 0;
  }

  char * end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed < 0) {
    return 0;
  }
  return static_cast<int>(parsed);
}

class ScopedRosDomainId
{
public:
  explicit ScopedRosDomainId(const int domain_id)
  {
    const char * existing = std::getenv("ROS_DOMAIN_ID");
    if (existing != nullptr) {
      had_existing_ = true;
      existing_value_ = existing;
    }
    setenv("ROS_DOMAIN_ID", std::to_string(std::max(0, domain_id)).c_str(), 1);
  }

  ~ScopedRosDomainId()
  {
    if (had_existing_) {
      setenv("ROS_DOMAIN_ID", existing_value_.c_str(), 1);
    } else {
      unsetenv("ROS_DOMAIN_ID");
    }
  }

private:
  bool had_existing_{false};
  std::string existing_value_;
};

struct GridSpec
{
  double resolution{0.05};
  double x_length{6.0};
  double y_length{6.0};
  double x_center{0.0};
  double y_center{0.0};
  double min_z{-2.0};
  double max_z{2.0};

  [[nodiscard]] double xMin() const { return x_center - 0.5 * x_length; }
  [[nodiscard]] double xMax() const { return x_center + 0.5 * x_length; }
  [[nodiscard]] double yMin() const { return y_center - 0.5 * y_length; }
  [[nodiscard]] double yMax() const { return y_center + 0.5 * y_length; }
  [[nodiscard]] std::uint32_t width() const
  {
    return static_cast<std::uint32_t>(std::ceil((x_length / resolution) - 1.0e-9));
  }
  [[nodiscard]] std::uint32_t height() const
  {
    return static_cast<std::uint32_t>(std::ceil((y_length / resolution) - 1.0e-9));
  }
};

struct ElevationGrid
{
  GridSpec spec;
  std_msgs::msg::Header header;
  std::vector<float> height;

  explicit ElevationGrid(GridSpec grid_spec = {})
  : spec(std::move(grid_spec))
  {
    if (spec.resolution <= 0.0 || spec.x_length <= 0.0 || spec.y_length <= 0.0) {
      throw std::invalid_argument("Invalid elevation grid geometry");
    }
    reset();
  }

  void reset()
  {
    height.assign(
      static_cast<std::size_t>(spec.width()) * spec.height(),
      std::numeric_limits<float>::quiet_NaN());
  }

};

struct MapPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct TimedPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  double time_offset{0.0};
};

struct TimedCloud
{
  rclcpp::Time stamp{0, 0u, RCL_SYSTEM_TIME};
  std::vector<TimedPoint> points;
};

struct CellHeight
{
  float height{std::numeric_limits<float>::quiet_NaN()};
  int support_count{0};
};

struct RigidTransform
{
  tf2::Vector3 translation{0.0, 0.0, 0.0};
  tf2::Matrix3x3 rotation{tf2::Quaternion::getIdentity()};
};

std::string shortDouble(const double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return std::string(buffer);
}

std::string paramDouble(const double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.9g", value);
  std::string text(buffer);
  if (
    std::isfinite(value) &&
    text.find('.') == std::string::npos &&
    text.find('e') == std::string::npos &&
    text.find('E') == std::string::npos)
  {
    text += ".0";
  }
  return text;
}

std::string vectorParam(const std::vector<double> & values)
{
  std::string out = "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out += ",";
    }
    out += paramDouble(values[i]);
  }
  out += "]";
  return out;
}

std::string matrixParam(const tf2::Matrix3x3 & matrix)
{
  std::vector<double> values;
  values.reserve(9);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      values.push_back(matrix[row][col]);
    }
  }
  return vectorParam(values);
}

const sensor_msgs::msg::PointField * findPointField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(),
    cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) { return field.name == name; });
  return it == cloud.fields.end() ? nullptr : &(*it);
}

template<typename T>
T readPointFieldAs(const std::uint8_t * ptr)
{
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

double readPointFieldNumeric(
  const std::uint8_t * base,
  const sensor_msgs::msg::PointField * field,
  const double fallback = 0.0)
{
  if (field == nullptr) {
    return fallback;
  }
  const auto * ptr = base + field->offset;
  switch (field->datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<double>(readPointFieldAs<std::int8_t>(ptr));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(readPointFieldAs<std::uint8_t>(ptr));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(readPointFieldAs<std::int16_t>(ptr));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(readPointFieldAs<std::uint16_t>(ptr));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(readPointFieldAs<std::int32_t>(ptr));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(readPointFieldAs<std::uint32_t>(ptr));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<double>(readPointFieldAs<float>(ptr));
    case sensor_msgs::msg::PointField::FLOAT64:
      return readPointFieldAs<double>(ptr);
    default:
      return fallback;
  }
}

std::vector<double> parseYamlVector(
  const std::string & file_path,
  const std::string & key,
  const std::size_t expected_size,
  const std::vector<double> & fallback)
{
  std::ifstream input(file_path);
  if (!input) {
    return fallback;
  }

  const std::regex key_regex("^\\s*" + key + "\\s*:");
  const std::regex number_regex(
    "[-+]?(?:(?:\\d+\\.?\\d*)|(?:\\.\\d+))(?:[eE][-+]?\\d+)?");
  std::string line;
  bool collecting = false;
  std::string collected;

  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }

    if (!collecting) {
      if (!std::regex_search(line, key_regex)) {
        continue;
      }
      collecting = true;
      const auto colon = line.find(':');
      if (colon != std::string::npos) {
        collected += line.substr(colon + 1);
      }
    } else {
      collected += " ";
      collected += line;
    }

    if (collected.find(']') != std::string::npos) {
      break;
    }
  }

  if (!collecting) {
    return fallback;
  }

  std::vector<double> values;
  for (
    auto it = std::sregex_iterator(collected.begin(), collected.end(), number_regex);
    it != std::sregex_iterator();
    ++it)
  {
    values.push_back(std::stod(it->str()));
  }
  if (values.size() != expected_size) {
    return fallback;
  }
  return values;
}

class ChildProcesses
{
public:
  ~ChildProcesses()
  {
    stopAll();
  }

  void start(
    rclcpp::Logger logger,
    const std::string & name,
    const std::vector<std::string> & command)
  {
    if (command.empty()) {
      RCLCPP_INFO(logger, "%s launch disabled: command is empty", name.c_str());
      return;
    }

    const pid_t pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(logger, "Failed to fork %s: %s", name.c_str(), std::strerror(errno));
      return;
    }

    if (pid == 0) {
      std::vector<char *> argv;
      argv.reserve(command.size() + 1);
      for (const auto & part : command) {
        argv.push_back(const_cast<char *>(part.c_str()));
      }
      argv.push_back(nullptr);
      execvp(argv.front(), argv.data());
      std::fprintf(stderr, "Failed to exec %s: %s\n", command.front().c_str(), std::strerror(errno));
      _exit(127);
    }

    children_.push_back({name, pid});
    RCLCPP_INFO(logger, "Started %s pid=%d", name.c_str(), static_cast<int>(pid));
  }

  void stopAll(const double grace_seconds = 0.8)
  {
    for (const auto & child : children_) {
      kill(child.pid, SIGINT);
    }

    const auto grace = std::chrono::duration<double>(std::max(0.0, grace_seconds));
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(grace);
    while (!children_.empty() && std::chrono::steady_clock::now() < deadline) {
      for (auto it = children_.begin(); it != children_.end(); ) {
        int status = 0;
        const pid_t ret = waitpid(it->pid, &status, WNOHANG);
        if (ret == it->pid || ret < 0) {
          it = children_.erase(it);
        } else {
          ++it;
        }
      }
      if (!children_.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    for (const auto & child : children_) {
      int status = 0;
      const pid_t ret = waitpid(child.pid, &status, WNOHANG);
      if (ret == 0) {
        kill(child.pid, SIGTERM);
        waitpid(child.pid, &status, 0);
      }
    }
    children_.clear();
  }

private:
  struct Child
  {
    std::string name;
    pid_t pid{-1};
  };

  std::vector<Child> children_;
};

class AutonomyLightNode final : public rclcpp::Node
{
public:
  AutonomyLightNode()
  : Node("autonomy_light")
  {
    loadParameters();
    configureTransform();
    loadSavedMap();
    configureDomainOutputs();
    createIo();
    publishStaticTransform();
    startExternalProcesses();

    RCLCPP_INFO(
      get_logger(),
      "Autonomy light ready: lidar=%s target=%s grid=%ux%u res=%.3fm backend=%s publish=%.1fHz",
      lidar_frame_.c_str(),
      target_frame_.c_str(),
      grid_spec_.width(),
      grid_spec_.height(),
      grid_spec_.resolution,
      elevation_backend_.c_str(),
      publish_rate_hz_);
    if (height_map_manual_mode_) {
      RCLCPP_WARN(
        get_logger(),
        "Height map manual debug mode enabled: %s will be filled with %.3f",
        height_map_msg_topic_.c_str(),
        height_map_manual_value_);
    }
    if (mapping_only_) {
      RCLCPP_WARN(
        get_logger(),
        "Mapping-only mode enabled: height-map IO is disabled; Point-LIO PCD save=%s",
        point_lio_pcd_save_en_ ? "true" : "false");
    }
  }

  ~AutonomyLightNode() override
  {
    child_processes_.stopAll(child_shutdown_grace_sec_);
    if (debug_executor_) {
      debug_executor_->cancel();
    }
    if (global_map_executor_) {
      global_map_executor_->cancel();
    }
    if (output_executor_) {
      output_executor_->cancel();
    }
    if (debug_spin_thread_.joinable()) {
      debug_spin_thread_.join();
    }
    if (global_map_spin_thread_.joinable()) {
      global_map_spin_thread_.join();
    }
    if (output_spin_thread_.joinable()) {
      output_spin_thread_.join();
    }
    debug_local_map_pub_.reset();
    point_lio_global_map_pub_.reset();
    point_lio_global_map_refined_pub_.reset();
    height_map_pub_.reset();
    height_map_msg_pub_.reset();
    odom_pub_.reset();
    path_pub_.reset();
    output_static_tf_broadcaster_.reset();
    output_tf_broadcaster_.reset();
    debug_node_.reset();
    global_map_node_.reset();
    output_node_.reset();
    if (debug_context_ && debug_context_->is_valid()) {
      debug_context_->shutdown("autonomy_light shutdown");
    }
    if (global_map_context_ && global_map_context_->is_valid()) {
      global_map_context_->shutdown("autonomy_light shutdown");
    }
    if (output_context_ && output_context_->is_valid()) {
      output_context_->shutdown("autonomy_light shutdown");
    }
  }

private:
  void loadParameters()
  {
    target_frame_ = declare_parameter<std::string>("target_frame", target_frame_);
    height_map_frame_ = declare_parameter<std::string>("height_map_frame", height_map_frame_);
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", lidar_frame_);
    lidar2_frame_ = declare_parameter<std::string>("lidar2_frame", lidar2_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    internal_ros_domain_id_ = static_cast<int>(
      declare_parameter<int>("internal_ros_domain_id", internal_ros_domain_id_));
    external_ros_domain_id_ = static_cast<int>(
      declare_parameter<int>("external_ros_domain_id", external_ros_domain_id_));
    debug_local_map_ros_domain_id_ = static_cast<int>(
      declare_parameter<int>(
        "debug_local_map_ros_domain_id", debug_local_map_ros_domain_id_));
    debug_local_map_topic_ = declare_parameter<std::string>(
      "debug_local_map_topic", debug_local_map_topic_);

    target_to_lidar_xyz_ = declareVectorParameter(
      "target_to_lidar_xyz", target_to_lidar_xyz_, 3);
    target_to_lidar_rpy_ = declareVectorParameter(
      "target_to_lidar_rpy", target_to_lidar_rpy_, 3);
    target_to_lidar2_xyz_ = declareVectorParameter(
      "target_to_lidar2_xyz", target_to_lidar2_xyz_, 3);
    target_to_lidar2_rpy_ = declareVectorParameter(
      "target_to_lidar2_rpy", target_to_lidar2_rpy_, 3);

    grid_spec_.resolution = declare_parameter<double>("elevation_resolution", grid_spec_.resolution);
    grid_spec_.x_length = declare_parameter<double>("elevation_x_length", grid_spec_.x_length);
    grid_spec_.y_length = declare_parameter<double>("elevation_y_length", grid_spec_.y_length);
    grid_spec_.x_center = declare_parameter<double>("elevation_x_center", grid_spec_.x_center);
    grid_spec_.y_center = declare_parameter<double>("elevation_y_center", grid_spec_.y_center);
    grid_spec_.min_z = declare_parameter<double>("elevation_min_z", grid_spec_.min_z);
    grid_spec_.max_z = declare_parameter<double>("elevation_max_z", grid_spec_.max_z);
    height_origin_mode_ = declare_parameter<std::string>("height_origin.mode", height_origin_mode_);
    height_origin_fixed_z_ = declare_parameter<double>("height_origin.fixed_z", height_origin_fixed_z_);
    height_origin_filter_alpha_ = std::clamp(
      declare_parameter<double>("height_origin.filter_alpha", height_origin_filter_alpha_),
      0.0,
      1.0);
    height_origin_max_step_ = std::max(
      0.0,
      declare_parameter<double>("height_origin.max_step", height_origin_max_step_));
    height_origin_floor_radius_ = std::max(
      grid_spec_.resolution,
      declare_parameter<double>("height_origin.floor_radius", height_origin_floor_radius_));
    height_origin_floor_percentile_ = std::clamp(
      declare_parameter<double>("height_origin.floor_percentile", height_origin_floor_percentile_),
      0.0,
      1.0);
    height_origin_floor_min_points_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "height_origin.floor_min_points",
        height_origin_floor_min_points_)));
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", publish_rate_hz_));
    mapping_only_ = declare_parameter<bool>("mapping_only", mapping_only_);
    point_lio_pcd_save_en_ = declare_parameter<bool>(
      "point_lio_pcd_save_en", point_lio_pcd_save_en_);
    point_lio_pcd_save_interval_ = declare_parameter<int>(
      "point_lio_pcd_save_interval", point_lio_pcd_save_interval_);
    child_shutdown_grace_sec_ = std::max(
      0.8,
      declare_parameter<double>("child_shutdown_grace_sec", child_shutdown_grace_sec_));
    saved_map_file_ = declare_parameter<std::string>("saved_map_file", saved_map_file_);
    saved_map_frame_ = declare_parameter<std::string>("saved_map_frame", saved_map_frame_);
    saved_map_topic_ = declare_parameter<std::string>("saved_map_topic", saved_map_topic_);
    saved_map_publish_voxel_leaf_size_ = std::max(
      0.02,
      declare_parameter<double>(
        "saved_map_publish_voxel_leaf_size", saved_map_publish_voxel_leaf_size_));
    saved_map_localization_enabled_ = declare_parameter<bool>(
      "saved_map_localization.enabled", saved_map_localization_enabled_);
    saved_map_global_initialization_ = declare_parameter<bool>(
      "saved_map_localization.global_initialization", saved_map_global_initialization_);
    saved_map_localization_update_interval_sec_ = std::max(
      0.0,
      declare_parameter<double>(
        "saved_map_localization.update_interval_sec",
        saved_map_localization_update_interval_sec_));
    saved_map_initial_submap_duration_sec_ = std::max(
      0.0,
      declare_parameter<double>(
        "saved_map_localization.initial_submap_duration_sec",
        saved_map_initial_submap_duration_sec_));
    saved_map_initial_submap_min_points_ = std::max(
      20,
      static_cast<int>(declare_parameter<int>(
        "saved_map_localization.initial_submap_min_points",
        saved_map_initial_submap_min_points_)));
    saved_map_initial_submap_max_points_ = std::max(
      saved_map_initial_submap_min_points_,
      static_cast<int>(declare_parameter<int>(
        "saved_map_localization.initial_submap_max_points",
        saved_map_initial_submap_max_points_)));
    saved_map_scan_voxel_leaf_size_ = std::max(
      0.02,
      declare_parameter<double>(
        "saved_map_localization.scan_voxel_leaf_size", saved_map_scan_voxel_leaf_size_));
    saved_map_voxel_leaf_size_ = std::max(
      0.02,
      declare_parameter<double>(
        "saved_map_localization.map_voxel_leaf_size", saved_map_voxel_leaf_size_));
    saved_map_normal_radius_ = std::max(
      0.05,
      declare_parameter<double>("saved_map_localization.normal_radius", saved_map_normal_radius_));
    saved_map_feature_radius_ = std::max(
      saved_map_normal_radius_,
      declare_parameter<double>("saved_map_localization.feature_radius", saved_map_feature_radius_));
    saved_map_global_max_iterations_ = std::max(
      100,
      static_cast<int>(declare_parameter<int>(
        "saved_map_localization.global_max_iterations", saved_map_global_max_iterations_)));
    saved_map_global_inlier_fraction_ = std::clamp(
      declare_parameter<double>(
        "saved_map_localization.global_inlier_fraction", saved_map_global_inlier_fraction_),
      0.05,
      1.0);
    saved_map_max_correspondence_distance_ = std::max(
      saved_map_scan_voxel_leaf_size_,
      declare_parameter<double>(
        "saved_map_localization.max_correspondence_distance",
        saved_map_max_correspondence_distance_));
    saved_map_gicp_max_iterations_ = std::max(
      5,
      static_cast<int>(declare_parameter<int>(
        "saved_map_localization.gicp_max_iterations", saved_map_gicp_max_iterations_)));
    saved_map_max_fitness_ = std::max(
      1.0e-5,
      declare_parameter<double>("saved_map_localization.max_fitness", saved_map_max_fitness_));
    saved_map_max_tracking_translation_step_ = std::max(
      0.05,
      declare_parameter<double>(
        "saved_map_localization.max_tracking_translation_step",
        saved_map_max_tracking_translation_step_));
    saved_map_min_scan_points_ = std::max(
      20,
      static_cast<int>(declare_parameter<int>(
        "saved_map_localization.min_scan_points", saved_map_min_scan_points_)));

    raw_lidar_topic_ = declare_parameter<std::string>("raw_lidar_topic", raw_lidar_topic_);
    raw_lidar2_topic_ = declare_parameter<std::string>("raw_lidar2_topic", raw_lidar2_topic_);
    raw_lidar_msg_type_ = declare_parameter<std::string>("raw_lidar_msg_type", raw_lidar_msg_type_);
    raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", raw_imu_topic_);
    raw_imu2_topic_ = declare_parameter<std::string>("raw_imu2_topic", raw_imu2_topic_);
    monitor_raw_lidar_ = declare_parameter<bool>("monitor_raw_lidar", monitor_raw_lidar_);
    merged_lidar_topic_ = declare_parameter<std::string>(
      "merged_lidar_topic", merged_lidar_topic_);
    lidar_merge_sync_tolerance_ = std::max(
      0.0,
      declare_parameter<double>("lidar_merge.sync_tolerance", lidar_merge_sync_tolerance_));
    lidar_merge_max_queue_size_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "lidar_merge.max_queue_size", lidar_merge_max_queue_size_)));
    lidar_merge_publish_lidar1_on_sync_miss_ = declare_parameter<bool>(
      "lidar_merge.publish_lidar1_on_sync_miss",
      lidar_merge_publish_lidar1_on_sync_miss_);
    point_lio_odom_topic_ = declare_parameter<std::string>(
      "point_lio_odom_topic", point_lio_odom_topic_);
    point_lio_map_topic_ = declare_parameter<std::string>(
      "point_lio_map_topic", point_lio_map_topic_);
    point_lio_path_topic_ = declare_parameter<std::string>(
      "point_lio_path_topic", point_lio_path_topic_);
    point_lio_registered_topic_ = declare_parameter<std::string>(
      "point_lio_registered_topic", point_lio_registered_topic_);
    const bool requested_global_map = declare_parameter<bool>(
      "point_lio_global_map.enabled", point_lio_global_map_enabled_);
    point_lio_global_map_topic_ = declare_parameter<std::string>(
      "point_lio_global_map.topic", point_lio_global_map_topic_);
    point_lio_global_map_refined_topic_ = declare_parameter<std::string>(
      "point_lio_global_map.refined_topic", point_lio_global_map_refined_topic_);
    point_lio_global_map_ros_domain_id_ = static_cast<int>(declare_parameter<int>(
      "point_lio_global_map.ros_domain_id", point_lio_global_map_ros_domain_id_));
    const bool requested_global_height_map = declare_parameter<bool>(
      "point_lio_global_map.use_for_height_map", point_lio_global_map_use_for_height_map_);
    // The global map is the sole live source for height-map construction.  Do
    // not allow a launch override to silently fall back to the old local-map
    // pipeline; mapping-only mode remains the one intentional exception.
    point_lio_global_map_enabled_ = !mapping_only_;
    point_lio_global_map_use_for_height_map_ = !mapping_only_;
    if (!mapping_only_ && (!requested_global_map || !requested_global_height_map)) {
      RCLCPP_WARN(
        get_logger(),
        "point_lio_global_map is required for the height map; forcing enabled=true and "
        "use_for_height_map=true");
    }
    point_lio_global_map_height_voxel_leaf_size_ = std::max(
      0.01,
      declare_parameter<double>(
        "point_lio_global_map.height_voxel_leaf_size",
        point_lio_global_map_height_voxel_leaf_size_));
    point_lio_global_map_height_max_points_ = std::max(
      10000,
      static_cast<int>(declare_parameter<int>(
        "point_lio_global_map.height_max_points", point_lio_global_map_height_max_points_)));
    point_lio_global_map_voxel_leaf_size_ = std::max(
      0.02,
      declare_parameter<double>(
        "point_lio_global_map.voxel_leaf_size", point_lio_global_map_voxel_leaf_size_));
    point_lio_global_map_publish_interval_sec_ = std::max(
      0.1,
      declare_parameter<double>(
        "point_lio_global_map.publish_interval_sec",
        point_lio_global_map_publish_interval_sec_));
    point_lio_global_map_max_points_ = std::max(
      1000,
      static_cast<int>(declare_parameter<int>(
        "point_lio_global_map.max_points", point_lio_global_map_max_points_)));
    point_lio_global_map_refined_visual_voxel_leaf_size_ = std::max(
      0.01,
      declare_parameter<double>(
        "point_lio_global_map.refined_visual_voxel_leaf_size",
        point_lio_global_map_refined_visual_voxel_leaf_size_));
    point_lio_global_map_refined_visual_max_points_ = std::max(
      1000,
      static_cast<int>(declare_parameter<int>(
        "point_lio_global_map.refined_visual_max_points",
        point_lio_global_map_refined_visual_max_points_)));
    point_lio_global_map_refine_mean_k_ = std::max(
      2,
      static_cast<int>(declare_parameter<int>(
        "point_lio_global_map.refinement.mean_k", point_lio_global_map_refine_mean_k_)));
    point_lio_global_map_refine_stddev_multiplier_ = std::max(
      0.05,
      declare_parameter<double>(
        "point_lio_global_map.refinement.stddev_multiplier",
        point_lio_global_map_refine_stddev_multiplier_));
    odom_output_topic_ = declare_parameter<std::string>("odom_output_topic", odom_output_topic_);
    height_map_topic_ = declare_parameter<std::string>("height_map_topic", height_map_topic_);
    height_map_msg_topic_ = declare_parameter<std::string>(
      "height_map_msg_topic", height_map_msg_topic_);
    path_output_topic_ = declare_parameter<std::string>("path_output_topic", path_output_topic_);
    heartbeat_topic_ = declare_parameter<std::string>("heartbeat_topic", heartbeat_topic_);
    height_map_manual_mode_ = declare_parameter<bool>(
      "height_map_debug.manual_mode", height_map_manual_mode_);
    height_map_manual_value_ = declare_parameter<double>(
      "height_map_debug.manual_value", height_map_manual_value_);
    interpolation_max_passes_ = std::max(
      0,
      static_cast<int>(declare_parameter<int>("interpolation_max_passes", interpolation_max_passes_)));
    interpolation_min_neighbors_ = std::clamp(
      static_cast<int>(declare_parameter<int>(
        "interpolation_min_neighbors", interpolation_min_neighbors_)),
      1,
      8);
    interpolation_max_height_diff_ = declare_parameter<double>(
      "interpolation_max_height_diff", interpolation_max_height_diff_);
    fill_remaining_height_ = declare_parameter<double>("fill_remaining_height", fill_remaining_height_);
    initial_floor_seed_fill_enabled_ = declare_parameter<bool>(
      "initial_floor_seed_fill.enabled", initial_floor_seed_fill_enabled_);
    initial_floor_seed_side_width_ = std::max(
      0.0,
      declare_parameter<double>(
        "initial_floor_seed_fill.side_width",
        initial_floor_seed_side_width_));
    initial_floor_seed_search_margin_ = std::max(
      0.0,
      declare_parameter<double>(
        "initial_floor_seed_fill.search_margin",
        initial_floor_seed_search_margin_));
    initial_floor_seed_cluster_band_ = std::max(
      1.0e-3,
      declare_parameter<double>(
        "initial_floor_seed_fill.cluster_band",
        initial_floor_seed_cluster_band_));
    initial_floor_seed_lower_fraction_ = std::clamp(
      declare_parameter<double>(
        "initial_floor_seed_fill.lower_fraction",
        initial_floor_seed_lower_fraction_),
      0.05,
      1.0);
    elevation_backend_ = declare_parameter<std::string>(
      "algorithm.elevation_backend", elevation_backend_);
    clipping_enabled_ = declare_parameter<bool>("algorithm.clipping.enabled", clipping_enabled_);
    clipping_min_z_ = declare_parameter<double>("algorithm.clipping.min_z", clipping_min_z_);
    clipping_max_z_ = declare_parameter<double>("algorithm.clipping.max_z", clipping_max_z_);
    min_z_min_points_per_cell_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "algorithm.min_z.min_points_per_cell", min_z_min_points_per_cell_)));
    min_z_supported_min_enabled_ = declare_parameter<bool>(
      "algorithm.min_z.supported_min_enabled", min_z_supported_min_enabled_);
    min_z_support_band_ = std::max(
      0.0,
      declare_parameter<double>("algorithm.min_z.support_band", min_z_support_band_));
    min_z_obstacle_override_enabled_ = declare_parameter<bool>(
      "algorithm.min_z.obstacle_override_enabled", min_z_obstacle_override_enabled_);
    min_z_obstacle_min_height_ = std::max(
      0.0,
      declare_parameter<double>("algorithm.min_z.obstacle_min_height", min_z_obstacle_min_height_));
    min_z_obstacle_min_points_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "algorithm.min_z.obstacle_min_points", min_z_obstacle_min_points_)));
    min_z_obstacle_support_band_ = std::max(
      0.0,
      declare_parameter<double>("algorithm.min_z.obstacle_support_band", min_z_obstacle_support_band_));
    min_z_obstacle_projection_radius_cells_ = std::max(
      0,
      static_cast<int>(declare_parameter<int>(
        "algorithm.min_z.obstacle_projection_radius_cells",
        min_z_obstacle_projection_radius_cells_)));
    const bool requested_registered_fill = declare_parameter<bool>(
      "algorithm.cloud_registered_fill.enabled", cloud_registered_fill_enabled_);
    // Registered scan filling was the old local-map fallback.  It can make a
    // single current scan visibly perturb an otherwise stable global terrain.
    cloud_registered_fill_enabled_ = false;
    if (requested_registered_fill) {
      RCLCPP_WARN(
        get_logger(),
        "algorithm.cloud_registered_fill is ignored: height maps use the global map only");
    }
    cloud_registered_fill_percentile_ = std::clamp(
      declare_parameter<double>(
        "algorithm.cloud_registered_fill.percentile", cloud_registered_fill_percentile_),
      0.0,
      1.0);
    cloud_registered_fill_min_points_per_cell_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "algorithm.cloud_registered_fill.min_points_per_cell",
        cloud_registered_fill_min_points_per_cell_)));
    const bool requested_initial_registered_fill = declare_parameter<bool>(
      "algorithm.cloud_registered_fill.initial_floor_fill_enabled",
      cloud_registered_initial_floor_fill_enabled_);
    cloud_registered_initial_floor_fill_enabled_ = false;
    if (requested_initial_registered_fill) {
      RCLCPP_WARN(
        get_logger(),
        "algorithm.cloud_registered_fill.initial_floor_fill_enabled is ignored: "
        "height maps use the global map only");
    }
    cloud_registered_initial_floor_max_coverage_ = std::clamp(
      declare_parameter<double>(
        "algorithm.cloud_registered_fill.initial_floor_max_local_coverage",
        cloud_registered_initial_floor_max_coverage_),
      0.0,
      1.0);
    cloud_registered_floor_min_points_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "algorithm.cloud_registered_fill.floor_min_points",
        cloud_registered_floor_min_points_)));
    cloud_registered_floor_support_band_ = std::max(
      0.0,
      declare_parameter<double>(
        "algorithm.cloud_registered_fill.floor_support_band",
        cloud_registered_floor_support_band_));
    cloud_registered_initial_keep_min_support_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "algorithm.cloud_registered_fill.initial_keep_min_support",
        cloud_registered_initial_keep_min_support_)));
    robust_height_gate_ = declare_parameter<double>(
      "algorithm.frame_aggregation.robust_height_gate", robust_height_gate_);
    intra_cell_min_support_gap_ = declare_parameter<double>(
      "algorithm.frame_aggregation.intra_cell_min_support_gap", intra_cell_min_support_gap_);
    intra_cell_min_support_count_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "algorithm.frame_aggregation.intra_cell_min_support_count",
        intra_cell_min_support_count_)));
    edge_mix_height_diff_ = declare_parameter<double>(
      "algorithm.frame_aggregation.edge_mix_height_diff", edge_mix_height_diff_);
    edge_prefer_prev_support_count_ = std::max(
      0,
      static_cast<int>(declare_parameter<int>(
        "algorithm.frame_aggregation.edge_prefer_prev_support_count",
        edge_prefer_prev_support_count_)));
    fill_missing_from_previous_grid_ = declare_parameter<bool>(
      "algorithm.frame_aggregation.fill_missing_from_previous_grid",
      fill_missing_from_previous_grid_);
    cell_height_percentile_ = std::clamp(
      declare_parameter<double>("algorithm.frame_aggregation.cell_height_percentile", cell_height_percentile_),
      0.0,
      1.0);
    temporal_alpha_ = std::clamp(
      declare_parameter<double>("algorithm.frame_aggregation.temporal_alpha", temporal_alpha_),
      0.0,
      1.0);
    isolated_filter_radius_ = std::max(
      0, static_cast<int>(declare_parameter<int>(
        "algorithm.isolated_filter.radius", isolated_filter_radius_)));
    isolated_filter_min_support_neighbors_ = std::max(
      0,
      static_cast<int>(declare_parameter<int>(
        "algorithm.isolated_filter.min_support_neighbors",
        isolated_filter_min_support_neighbors_)));
    isolated_filter_support_height_diff_ = declare_parameter<double>(
      "algorithm.isolated_filter.support_height_diff", isolated_filter_support_height_diff_);
    isolated_filter_outlier_height_diff_ = declare_parameter<double>(
      "algorithm.isolated_filter.outlier_height_diff", isolated_filter_outlier_height_diff_);
    isolated_filter_every_n_frames_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
        "algorithm.isolated_filter.every_n_frames",
        isolated_filter_every_n_frames_)));
    hole_fill_radius_ = std::max(
      0, static_cast<int>(declare_parameter<int>("algorithm.hole_fill.radius", hole_fill_radius_)));
    hole_fill_min_neighbors_ = std::max(
      0, static_cast<int>(declare_parameter<int>(
        "algorithm.hole_fill.min_neighbors", hole_fill_min_neighbors_)));
    hole_fill_max_height_diff_ = declare_parameter<double>(
      "algorithm.hole_fill.max_height_diff", hole_fill_max_height_diff_);
    bilateral_radius_ = std::max(
      0, static_cast<int>(declare_parameter<int>("algorithm.bilateral.radius", bilateral_radius_)));
    bilateral_sigma_spatial_ = std::max(
      1.0e-6,
      declare_parameter<double>("algorithm.bilateral.sigma_spatial", bilateral_sigma_spatial_));
    bilateral_sigma_height_ = std::max(
      1.0e-6,
      declare_parameter<double>("algorithm.bilateral.sigma_height", bilateral_sigma_height_));
    bilateral_max_height_diff_ = declare_parameter<double>(
      "algorithm.bilateral.max_height_diff", bilateral_max_height_diff_);
    bilateral_passes_ = std::max(
      0, static_cast<int>(declare_parameter<int>("algorithm.bilateral.passes", bilateral_passes_)));
    bilateral_every_n_frames_ = std::max(
      1, static_cast<int>(declare_parameter<int>(
        "algorithm.bilateral.every_n_frames", bilateral_every_n_frames_)));

    start_lidar_driver_ = declare_parameter<bool>("start_lidar_driver", start_lidar_driver_);
    start_point_lio_ = declare_parameter<bool>("start_point_lio", start_point_lio_);
    child_use_sim_time_ = declare_parameter<bool>("child_use_sim_time", child_use_sim_time_);
    lidar_driver_command_ = declare_parameter<std::vector<std::string>>(
      "lidar_driver_command", lidar_driver_command_);
    point_lio_command_ = declare_parameter<std::vector<std::string>>(
      "point_lio_command", point_lio_command_);
    lidar_driver2_command_ = declare_parameter<std::vector<std::string>>(
      "lidar_driver2_command", lidar_driver2_command_);
    point_lio_config_file_ = declare_parameter<std::string>(
      "point_lio_config_file", point_lio_config_file_);

    const auto actual_domain = currentRosDomainId();
    if (internal_ros_domain_id_ < 0) {
      internal_ros_domain_id_ = actual_domain;
    }
    if (external_ros_domain_id_ < 0) {
      external_ros_domain_id_ = actual_domain;
    }
    if (point_lio_global_map_ros_domain_id_ < 0) {
      point_lio_global_map_ros_domain_id_ = external_ros_domain_id_;
    }
    if (debug_local_map_ros_domain_id_ < 0) {
      debug_local_map_ros_domain_id_ = internal_ros_domain_id_;
    }
    if (debug_local_map_topic_.empty()) {
      debug_local_map_topic_ = point_lio_map_topic_;
    }

    latest_grid_ = ElevationGrid(grid_spec_);
    latest_ground_grid_ = ElevationGrid(grid_spec_);
  }

  std::vector<double> declareVectorParameter(
    const std::string & name,
    const std::vector<double> & defaults,
    const std::size_t expected_size)
  {
    auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != expected_size) {
      throw std::invalid_argument(name + " must contain " + std::to_string(expected_size) + " values");
    }
    return values;
  }

  void configureTransform()
  {
    tf2::Quaternion q;
    q.setRPY(target_to_lidar_rpy_[0], target_to_lidar_rpy_[1], target_to_lidar_rpy_[2]);
    q.normalize();
    target_to_lidar_rotation_ = tf2::Matrix3x3(q);
    target_to_lidar_translation_ = tf2::Vector3(
      target_to_lidar_xyz_[0],
      target_to_lidar_xyz_[1],
      target_to_lidar_xyz_[2]);
    target_to_lidar_quaternion_ = q;

    tf2::Quaternion q2;
    q2.setRPY(target_to_lidar2_rpy_[0], target_to_lidar2_rpy_[1], target_to_lidar2_rpy_[2]);
    q2.normalize();
    target_to_lidar2_rotation_ = tf2::Matrix3x3(q2);
    target_to_lidar2_translation_ = tf2::Vector3(
      target_to_lidar2_xyz_[0],
      target_to_lidar2_xyz_[1],
      target_to_lidar2_xyz_[2]);
    target_to_lidar2_quaternion_ = q2;
  }

  void loadSavedMap()
  {
    if (saved_map_file_.empty()) {
      return;
    }

    pcl::PCLPointCloud2 cloud_blob;
    const int ret = pcl::io::loadPCDFile(saved_map_file_, cloud_blob);
    if (ret < 0) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to load saved map PCD: %s",
        saved_map_file_.c_str());
      throw std::runtime_error("failed to load saved map PCD: " + saved_map_file_);
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    try {
      pcl::fromPCLPointCloud2(cloud_blob, cloud_xyz);
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to convert saved map PCD %s: %s",
        saved_map_file_.c_str(),
        ex.what());
      throw;
    }

    auto points = std::make_shared<std::vector<MapPoint>>();
    points->reserve(cloud_xyz.size());
    for (const auto & point : cloud_xyz.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      points->push_back({point.x, point.y, point.z});
    }

    if (points->empty()) {
      RCLCPP_FATAL(
        get_logger(),
        "Saved map PCD has no finite xyz points: %s",
        saved_map_file_.c_str());
      throw std::runtime_error("saved map PCD has no finite xyz points: " + saved_map_file_);
    }

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      saved_map_points_ = points;
      latest_map_frame_ = saved_map_frame_;
      saved_map_loaded_ = true;
      has_map_ = true;
    }
    initializeSavedMapLocalization(cloud_xyz);
    saved_map_visualization_cloud_ = voxelDownsample(
      cloud_xyz.makeShared(), saved_map_publish_voxel_leaf_size_);
    last_map_time_ = now();
    ++map_count_;

    RCLCPP_INFO(
      get_logger(),
      "Saved-map mode enabled: loaded %zu points from %s. %s",
      points->size(),
      saved_map_file_.c_str(),
      savedMapRelocalizationActive() ?
      "Registered scans will be relocalized before saved-map height extraction." :
      "Relocalization is disabled; Point-LIO odom is assumed to already be in this map frame.");
  }

  using PclCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PclNormals = pcl::PointCloud<pcl::Normal>;
  using PclFeatures = pcl::PointCloud<pcl::FPFHSignature33>;

  bool savedMapRelocalizationActive() const
  {
    return saved_map_loaded_ && saved_map_localization_enabled_;
  }

  void publishSavedMap()
  {
    if (!saved_map_pub_ || !saved_map_visualization_cloud_ || saved_map_visualization_cloud_->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*saved_map_visualization_cloud_, message);
    message.header.stamp = now();
    message.header.frame_id = saved_map_frame_;
    saved_map_pub_->publish(message);
    RCLCPP_INFO(
      get_logger(),
      "Published saved map for visualization: topic=%s frame=%s points=%zu voxel=%.3fm",
      saved_map_topic_.c_str(),
      saved_map_frame_.c_str(),
      saved_map_visualization_cloud_->size(),
      saved_map_publish_voxel_leaf_size_);
  }

  static PclCloud::Ptr voxelDownsample(
    const PclCloud::ConstPtr & cloud,
    const double leaf_size)
  {
    PclCloud::Ptr result(new PclCloud());
    if (!cloud || cloud->empty() || !std::isfinite(leaf_size) || leaf_size <= 0.0) {
      return result;
    }

    // pcl::VoxelGrid allocates a dense integer grid over the full bounding box.
    // A centimetre leaf on a large global map can overflow that index space even
    // though only a small number of voxels actually contain a point.  Sparse
    // hashing has no bounding-box-dependent allocation and therefore preserves
    // the requested high resolution.
    struct VoxelKey
    {
      std::int64_t x;
      std::int64_t y;
      std::int64_t z;

      bool operator==(const VoxelKey & other) const
      {
        return x == other.x && y == other.y && z == other.z;
      }
    };
    struct VoxelKeyHash
    {
      std::size_t operator()(const VoxelKey & key) const
      {
        const auto h1 = std::hash<std::int64_t>{}(key.x);
        const auto h2 = std::hash<std::int64_t>{}(key.y);
        const auto h3 = std::hash<std::int64_t>{}(key.z);
        return h1 ^ (h2 << 1U) ^ (h3 << 7U);
      }
    };
    struct VoxelAccumulator
    {
      double x{0.0};
      double y{0.0};
      double z{0.0};
      std::uint32_t count{0};
    };

    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    voxels.reserve(cloud->size());
    constexpr double kMaxCoordinateMagnitudeM = 1.0e6;
    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        std::abs(static_cast<double>(point.x)) > kMaxCoordinateMagnitudeM ||
        std::abs(static_cast<double>(point.y)) > kMaxCoordinateMagnitudeM ||
        std::abs(static_cast<double>(point.z)) > kMaxCoordinateMagnitudeM)
      {
        continue;
      }
      const VoxelKey key{
        static_cast<std::int64_t>(std::floor(static_cast<double>(point.x) / leaf_size)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(point.y) / leaf_size)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(point.z) / leaf_size))};
      auto & accumulator = voxels[key];
      accumulator.x += point.x;
      accumulator.y += point.y;
      accumulator.z += point.z;
      ++accumulator.count;
    }

    result->reserve(voxels.size());
    for (const auto & entry : voxels) {
      const auto & accumulator = entry.second;
      if (accumulator.count == 0U) {
        continue;
      }
      const double inverse_count = 1.0 / static_cast<double>(accumulator.count);
      result->push_back(pcl::PointXYZ(
        static_cast<float>(accumulator.x * inverse_count),
        static_cast<float>(accumulator.y * inverse_count),
        static_cast<float>(accumulator.z * inverse_count)));
    }
    return result;
  }

  static PclCloud::Ptr removeStatisticalOutliers(
    const PclCloud::ConstPtr & cloud,
    const int mean_k,
    const double stddev_multiplier)
  {
    if (!cloud || cloud->size() < 4U || mean_k < 2 || stddev_multiplier <= 0.0) {
      return cloud ? PclCloud::Ptr(new PclCloud(*cloud)) : PclCloud::Ptr(new PclCloud());
    }
    PclCloud::Ptr result(new PclCloud());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(std::min<int>(mean_k, static_cast<int>(cloud->size()) - 1));
    filter.setStddevMulThresh(stddev_multiplier);
    filter.filter(*result);
    return result;
  }

  bool computeFpfhFeatures(
    const PclCloud::ConstPtr & cloud,
    PclFeatures::Ptr & features) const
  {
    if (!cloud || cloud->size() < static_cast<std::size_t>(saved_map_min_scan_points_)) {
      return false;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    PclNormals::Ptr normals(new PclNormals());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(saved_map_normal_radius_);
    normal_estimation.compute(*normals);
    if (normals->size() != cloud->size()) {
      return false;
    }

    features.reset(new PclFeatures());
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(saved_map_feature_radius_);
    fpfh.compute(*features);
    return features->size() == cloud->size() && !features->empty();
  }

  void initializeSavedMapLocalization(const PclCloud & cloud)
  {
    if (!saved_map_localization_enabled_) {
      return;
    }

    saved_map_localization_cloud_ = voxelDownsample(
      cloud.makeShared(), saved_map_voxel_leaf_size_);
    if (saved_map_localization_cloud_->size() <
      static_cast<std::size_t>(saved_map_min_scan_points_))
    {
      RCLCPP_WARN(
        get_logger(),
        "Saved-map relocalization disabled: map has only %zu voxelized points (need %d)",
        saved_map_localization_cloud_->size(),
        saved_map_min_scan_points_);
      saved_map_localization_enabled_ = false;
      return;
    }

    if (saved_map_global_initialization_ && !computeFpfhFeatures(
        saved_map_localization_cloud_, saved_map_localization_features_))
    {
      RCLCPP_WARN(
        get_logger(),
        "Saved-map global relocalization is unavailable because FPFH features could not be built; "
        "GICP will require Point-LIO to start near the mapped pose.");
      saved_map_global_initialization_ = false;
    }
  }

  static bool isFiniteTransform(const Eigen::Matrix4f & transform)
  {
    return transform.array().isFinite().all() &&
      std::abs(transform(3, 0)) < 1.0e-4F &&
      std::abs(transform(3, 1)) < 1.0e-4F &&
      std::abs(transform(3, 2)) < 1.0e-4F &&
      std::abs(transform(3, 3) - 1.0F) < 1.0e-4F;
  }

  void setSavedMapRelocalizationProgress(
    const std::string & phase,
    const std::size_t submap_points = 0,
    const double elapsed_sec = 0.0)
  {
    std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
    saved_map_relocalization_phase_ = phase;
    saved_map_relocalization_submap_points_ = submap_points;
    saved_map_relocalization_elapsed_sec_ = elapsed_sec;
  }

  void resetInitialRelocalizationSubmap()
  {
    initial_relocalization_submap_.reset();
    initial_relocalization_submap_start_ = std::chrono::steady_clock::time_point{};
  }

  void appendInitialRelocalizationSubmap(const PclCloud::ConstPtr & registered_cloud)
  {
    if (!initial_relocalization_submap_) {
      initial_relocalization_submap_.reset(new PclCloud());
      initial_relocalization_submap_start_ = std::chrono::steady_clock::now();
    }
    *initial_relocalization_submap_ += *registered_cloud;
    initial_relocalization_submap_ = voxelDownsample(
      initial_relocalization_submap_, saved_map_scan_voxel_leaf_size_);

    const auto max_points = static_cast<std::size_t>(saved_map_initial_submap_max_points_);
    if (initial_relocalization_submap_->size() > max_points) {
      PclCloud::Ptr capped(new PclCloud());
      capped->reserve(max_points);
      const std::size_t stride = (initial_relocalization_submap_->size() + max_points - 1) /
        max_points;
      for (std::size_t index = 0; index < initial_relocalization_submap_->size(); index += stride) {
        capped->push_back(initial_relocalization_submap_->points[index]);
      }
      initial_relocalization_submap_ = std::move(capped);
    }
    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - initial_relocalization_submap_start_).count();
    setSavedMapRelocalizationProgress(
      "collecting_submap", initial_relocalization_submap_->size(), elapsed_sec);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Saved-map relocalization: collecting submap %.1f/%.1fs, %zu/%d points",
      elapsed_sec,
      saved_map_initial_submap_duration_sec_,
      initial_relocalization_submap_->size(),
      saved_map_initial_submap_min_points_);
  }

  bool initialRelocalizationSubmapReady() const
  {
    return initial_relocalization_submap_ &&
      initial_relocalization_submap_->size() >=
      static_cast<std::size_t>(saved_map_initial_submap_min_points_) &&
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - initial_relocalization_submap_start_).count() >=
      saved_map_initial_submap_duration_sec_;
  }

  bool tryRelocalize(const PclCloud::ConstPtr & registered_cloud)
  {
    if (!savedMapRelocalizationActive() || !registered_cloud ||
      registered_cloud->size() < static_cast<std::size_t>(saved_map_min_scan_points_))
    {
      return false;
    }

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    bool already_localized = false;
    {
      std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
      initial_guess = saved_map_from_odom_;
      already_localized = saved_map_relocalized_;
    }

    const bool global_attempt = !already_localized && saved_map_global_initialization_;
    PclCloud::Ptr source;
    if (global_attempt) {
      appendInitialRelocalizationSubmap(registered_cloud);
      if (!initialRelocalizationSubmapReady()) {
        return false;
      }
      source = initial_relocalization_submap_;
    } else {
      source = voxelDownsample(registered_cloud, saved_map_scan_voxel_leaf_size_);
    }
    if (source->size() < static_cast<std::size_t>(saved_map_min_scan_points_)) {
      return false;
    }

    const auto attempt_time = std::chrono::steady_clock::now();
    if (last_saved_map_localization_attempt_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(attempt_time - last_saved_map_localization_attempt_).count() <
      saved_map_localization_update_interval_sec_)
    {
      return false;
    }
    last_saved_map_localization_attempt_ = attempt_time;

    if (global_attempt) {
      setSavedMapRelocalizationProgress(
        "global_feature_matching", source->size(), saved_map_initial_submap_duration_sec_);
      RCLCPP_INFO(
        get_logger(),
        "Saved-map relocalization: running FPFH/RANSAC on a %zu-point submap",
        source->size());
      PclFeatures::Ptr source_features;
      if (!computeFpfhFeatures(source, source_features)) {
        setSavedMapRelocalizationProgress("retrying_after_feature_failure");
        resetInitialRelocalizationSubmap();
        return false;
      }
      pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> global;
      global.setInputSource(source);
      global.setSourceFeatures(source_features);
      global.setInputTarget(saved_map_localization_cloud_);
      global.setTargetFeatures(saved_map_localization_features_);
      global.setNumberOfSamples(3);
      global.setCorrespondenceRandomness(5);
      global.setSimilarityThreshold(0.9F);
      global.setMaxCorrespondenceDistance(
        static_cast<float>(2.5 * saved_map_max_correspondence_distance_));
      global.setInlierFraction(static_cast<float>(saved_map_global_inlier_fraction_));
      global.setMaximumIterations(saved_map_global_max_iterations_);
      PclCloud global_aligned;
      global.align(global_aligned);
      if (!global.hasConverged() || !isFiniteTransform(global.getFinalTransformation())) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Saved-map global relocalization has not found a valid FPFH/RANSAC submap candidate yet");
        setSavedMapRelocalizationProgress("retrying_after_global_match_failure");
        resetInitialRelocalizationSubmap();
        return false;
      }
      initial_guess = global.getFinalTransformation();
    }

    setSavedMapRelocalizationProgress(
      global_attempt ? "gicp_refinement" : "gicp_tracking", source->size(), 0.0);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Saved-map relocalization: running GICP %s with %zu points",
      global_attempt ? "refinement" : "tracking", source->size());
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(saved_map_localization_cloud_);
    gicp.setMaximumIterations(saved_map_gicp_max_iterations_);
    gicp.setMaxCorrespondenceDistance(static_cast<float>(saved_map_max_correspondence_distance_));
    gicp.setTransformationEpsilon(1.0e-5);
    gicp.setEuclideanFitnessEpsilon(1.0e-5);
    gicp.setCorrespondenceRandomness(20);
    PclCloud aligned;
    gicp.align(aligned, initial_guess);
    const Eigen::Matrix4f candidate = gicp.getFinalTransformation();
    const double fitness = gicp.getFitnessScore(
      saved_map_max_correspondence_distance_ * saved_map_max_correspondence_distance_);
    if (!gicp.hasConverged() || !isFiniteTransform(candidate) ||
      !std::isfinite(fitness) || fitness > saved_map_max_fitness_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Saved-map GICP rejected: converged=%s fitness=%.4f (limit %.4f)",
        gicp.hasConverged() ? "true" : "false", fitness, saved_map_max_fitness_);
      if (global_attempt) {
        setSavedMapRelocalizationProgress("retrying_after_gicp_failure");
        resetInitialRelocalizationSubmap();
      }
      return false;
    }

    if (already_localized &&
      (candidate.block<3, 1>(0, 3) - initial_guess.block<3, 1>(0, 3)).norm() >
      saved_map_max_tracking_translation_step_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Saved-map GICP rejected a %.2fm correction jump (limit %.2fm)",
        (candidate.block<3, 1>(0, 3) - initial_guess.block<3, 1>(0, 3)).norm(),
        saved_map_max_tracking_translation_step_);
      setSavedMapRelocalizationProgress("tracking_jump_rejected");
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
      saved_map_from_odom_ = candidate;
      saved_map_relocalized_ = true;
      saved_map_last_fitness_ = fitness;
      saved_map_relocalization_phase_ = "tracking";
      saved_map_relocalization_submap_points_ = source->size();
      saved_map_relocalization_elapsed_sec_ = 0.0;
    }
    applySavedMapCorrectionToLatestOdom();
    if (global_attempt) {
      resetInitialRelocalizationSubmap();
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Saved-map relocalization accepted: %s fitness=%.4f",
      already_localized ? "GICP tracking" : "global FPFH/RANSAC submap + GICP", fitness);
    return true;
  }

  nav_msgs::msg::Odometry savedMapCorrectedOdom(
    const nav_msgs::msg::Odometry & raw_odom,
    const Eigen::Matrix4f & map_from_odom) const
  {
    nav_msgs::msg::Odometry corrected = raw_odom;
    const Eigen::Vector4f raw_position(
      static_cast<float>(raw_odom.pose.pose.position.x),
      static_cast<float>(raw_odom.pose.pose.position.y),
      static_cast<float>(raw_odom.pose.pose.position.z),
      1.0F);
    const Eigen::Vector4f corrected_position = map_from_odom * raw_position;
    corrected.pose.pose.position.x = corrected_position.x();
    corrected.pose.pose.position.y = corrected_position.y();
    corrected.pose.pose.position.z = corrected_position.z();

    Eigen::Quaternionf raw_orientation(
      static_cast<float>(raw_odom.pose.pose.orientation.w),
      static_cast<float>(raw_odom.pose.pose.orientation.x),
      static_cast<float>(raw_odom.pose.pose.orientation.y),
      static_cast<float>(raw_odom.pose.pose.orientation.z));
    if (raw_orientation.squaredNorm() < 1.0e-6F) {
      raw_orientation = Eigen::Quaternionf::Identity();
    } else {
      raw_orientation.normalize();
    }
    Eigen::Quaternionf corrected_orientation(map_from_odom.block<3, 3>(0, 0) *
      raw_orientation.toRotationMatrix());
    corrected_orientation.normalize();
    corrected.pose.pose.orientation.x = corrected_orientation.x();
    corrected.pose.pose.orientation.y = corrected_orientation.y();
    corrected.pose.pose.orientation.z = corrected_orientation.z();
    corrected.pose.pose.orientation.w = corrected_orientation.w();
    corrected.header.frame_id = saved_map_frame_;
    corrected.child_frame_id = target_frame_;
    return corrected;
  }

  void applySavedMapCorrectionToPoints(
    std::vector<MapPoint> & points,
    const Eigen::Matrix4f & map_from_odom) const
  {
    for (auto & point : points) {
      const Eigen::Vector4f raw(point.x, point.y, point.z, 1.0F);
      const Eigen::Vector4f corrected = map_from_odom * raw;
      point.x = corrected.x();
      point.y = corrected.y();
      point.z = corrected.z();
    }
  }

  void applySavedMapCorrectionToLatestOdom()
  {
    Eigen::Matrix4f map_from_odom = Eigen::Matrix4f::Identity();
    bool relocalized = false;
    {
      std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
      map_from_odom = saved_map_from_odom_;
      relocalized = saved_map_relocalized_;
    }
    if (!relocalized) {
      return;
    }
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (has_raw_odom_) {
      latest_odom_ = savedMapCorrectedOdom(latest_raw_odom_, map_from_odom);
      has_odom_ = true;
    }
  }

  geometry_msgs::msg::Pose savedMapCorrectedPose(
    const geometry_msgs::msg::Pose & raw_pose,
    const Eigen::Matrix4f & map_from_odom) const
  {
    nav_msgs::msg::Odometry raw;
    raw.pose.pose = raw_pose;
    return savedMapCorrectedOdom(raw, map_from_odom).pose.pose;
  }

  rclcpp::Node::SharedPtr createDomainNode(
    const std::string & name,
    const int domain_id,
    rclcpp::Context::SharedPtr & context_storage) const
  {
    rclcpp::InitOptions init_options;
#if AUTONOMY_LIGHT_HAS_RCLCPP_DOMAIN_ID_API
    init_options.set_domain_id(static_cast<std::size_t>(std::max(0, domain_id)));
#else
    ScopedRosDomainId scoped_domain_id(domain_id);
#endif
    context_storage = std::make_shared<rclcpp::Context>();
    const char * argv[] = {name.c_str()};
    context_storage->init(1, argv, init_options);

    rclcpp::NodeOptions options;
    options.context(context_storage);
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    return std::make_shared<rclcpp::Node>(name, options);
  }

  void startAuxiliaryExecutor(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> & executor,
    std::thread & spin_thread,
    const std::string & label)
  {
    rclcpp::ExecutorOptions options;
    options.context = node->get_node_base_interface()->get_context();
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);
    executor->add_node(node);
    spin_thread = std::thread(
      [this, executor, label]() {
        try {
          executor->spin();
        } catch (const std::exception & ex) {
          RCLCPP_WARN(
            get_logger(),
            "%s executor stopped: %s",
            label.c_str(),
            ex.what());
        }
      });
  }

  void configureDomainOutputs()
  {
    const auto actual_internal_domain = currentRosDomainId();
    if (internal_ros_domain_id_ != actual_internal_domain) {
      RCLCPP_WARN(
        get_logger(),
        "Configured internal_ros_domain_id=%d but current node domain is %d. "
        "launch.sh should export ROS_DOMAIN_ID before starting this node.",
        internal_ros_domain_id_,
        actual_internal_domain);
      internal_ros_domain_id_ = actual_internal_domain;
    }

    if (mapping_only_) {
      RCLCPP_INFO(
        get_logger(),
        "Mapping-only mode: skipping external height-map/odom/path publishers");
      return;
    }

    rclcpp::Node * output_node = this;
    if (external_ros_domain_id_ != internal_ros_domain_id_) {
      output_node_ = createDomainNode(
        "autonomy_light_external_output",
        external_ros_domain_id_,
        output_context_);
      startAuxiliaryExecutor(
        output_node_,
        output_executor_,
        output_spin_thread_,
        "external output");
      output_node = output_node_.get();
    }

    height_map_pub_ = output_node->create_publisher<sensor_msgs::msg::PointCloud2>(
      height_map_topic_,
      rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
    height_map_msg_pub_ = output_node->create_publisher<::autonomy_light::msg::HeightMap>(
      height_map_msg_topic_,
      rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
    odom_pub_ = output_node->create_publisher<nav_msgs::msg::Odometry>(
      odom_output_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    path_pub_ = output_node->create_publisher<nav_msgs::msg::Path>(
      path_output_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    if (saved_map_loaded_) {
      saved_map_pub_ = output_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        saved_map_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
      publishSavedMap();
    }
    if (point_lio_global_map_enabled_) {
      rclcpp::Node * global_map_output_node = this;
      if (point_lio_global_map_ros_domain_id_ == external_ros_domain_id_) {
        global_map_output_node = output_node;
      } else if (point_lio_global_map_ros_domain_id_ != internal_ros_domain_id_) {
        global_map_node_ = createDomainNode(
          "autonomy_light_global_map",
          point_lio_global_map_ros_domain_id_,
          global_map_context_);
        startAuxiliaryExecutor(
          global_map_node_,
          global_map_executor_,
          global_map_spin_thread_,
          "global map");
        global_map_output_node = global_map_node_.get();
      }
      point_lio_global_map_pub_ = global_map_output_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        point_lio_global_map_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
      point_lio_global_map_refined_pub_ =
        global_map_output_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        point_lio_global_map_refined_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
      RCLCPP_INFO(
        get_logger(),
        "Global map enabled: raw=%s refined=%s on ROS_DOMAIN_ID=%d (height-map source)",
        point_lio_global_map_topic_.c_str(),
        point_lio_global_map_refined_topic_.c_str(),
        point_lio_global_map_ros_domain_id_);
    }
    output_static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(output_node);
    output_tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(output_node);

    if (debug_local_map_ros_domain_id_ != internal_ros_domain_id_) {
      rclcpp::Node * debug_output_node = nullptr;
      if (debug_local_map_ros_domain_id_ == external_ros_domain_id_) {
        debug_output_node = output_node;
      } else {
        debug_node_ = createDomainNode(
          "autonomy_light_debug_local_map",
          debug_local_map_ros_domain_id_,
          debug_context_);
        startAuxiliaryExecutor(
          debug_node_,
          debug_executor_,
          debug_spin_thread_,
          "debug local map");
        debug_output_node = debug_node_.get();
      }
      debug_local_map_pub_ = debug_output_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_local_map_topic_,
        rclcpp::SensorDataQoS());
      RCLCPP_INFO(
        get_logger(),
        "Debug local map republish enabled: %s on ROS_DOMAIN_ID=%d",
        debug_local_map_topic_.c_str(),
        debug_local_map_ros_domain_id_);
    }

    RCLCPP_INFO(
      get_logger(),
      "ROS domains: internal=%d external=%d global_map=%d debug_local_map=%d%s",
      internal_ros_domain_id_,
      external_ros_domain_id_,
      point_lio_global_map_ros_domain_id_,
      debug_local_map_ros_domain_id_,
      debug_local_map_pub_ ? "" : " (not republished)");
  }

  int currentRosDomainId()
  {
#if AUTONOMY_LIGHT_HAS_RCLCPP_DOMAIN_ID_API
    return static_cast<int>(get_node_base_interface()->get_context()->get_domain_id());
#else
    return rosDomainIdFromEnvironment();
#endif
  }

  void createIo()
  {
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(heartbeat_topic_, 10);

    if (lidarMergeEnabled()) {
      merged_lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        merged_lidar_topic_,
        rclcpp::SensorDataQoS());
      createLidarMergeSubscriptions();
      RCLCPP_INFO(
        get_logger(),
        "LiDAR merge enabled: lidar1=%s lidar2=%s output=%s sync_tolerance=%.3fs",
        raw_lidar_topic_.c_str(),
        raw_lidar2_topic_.c_str(),
        merged_lidar_topic_.c_str(),
        lidar_merge_sync_tolerance_);
    } else if (monitor_raw_lidar_) {
      lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        raw_lidar_topic_,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          onLidarCloud(std::move(msg));
        });
    }
    if (!mapping_only_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        point_lio_odom_topic_,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          onPointLioOdom(std::move(msg));
        });
      path_sub_ = create_subscription<nav_msgs::msg::Path>(
        point_lio_path_topic_,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        [this](nav_msgs::msg::Path::SharedPtr msg) {
          onPointLioPath(std::move(msg));
        });
      if (savedMapRelocalizationActive() || point_lio_global_map_enabled_) {
        registered_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          point_lio_registered_topic_,
          rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            onPointLioRegistered(std::move(msg));
          });
      }
    }

    if (!mapping_only_) {
      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
      publish_timer_ = create_wall_timer(period, [this]() { publishLatest(); });
    }
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publishHeartbeat(); });
  }

  bool lidarMergeEnabled() const
  {
    return !raw_lidar2_topic_.empty();
  }

  void createLidarMergeSubscriptions()
  {
    const auto qos = rclcpp::SensorDataQoS();
    if (raw_lidar_msg_type_ == "livox_custom" || raw_lidar_msg_type_ == "custom") {
      lidar1_custom_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        raw_lidar_topic_,
        qos,
        [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          onMergeCustomCloud(0, std::move(msg));
        });
      lidar2_custom_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        raw_lidar2_topic_,
        qos,
        [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          onMergeCustomCloud(1, std::move(msg));
        });
      return;
    }

    lidar1_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_lidar_topic_,
      qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        onMergePointCloud(0, std::move(msg));
      });
    lidar2_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_lidar2_topic_,
      qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        onMergePointCloud(1, std::move(msg));
      });
  }

  TimedPoint transformLidarPoint(
    const int lidar_index,
    const float x,
    const float y,
    const float z,
    const float intensity,
    const double time_offset) const
  {
    const auto & rotation = lidar_index == 0 ? target_to_lidar_rotation_ : target_to_lidar2_rotation_;
    const auto & translation =
      lidar_index == 0 ? target_to_lidar_translation_ : target_to_lidar2_translation_;
    const tf2::Vector3 p_target = translation + rotation * tf2::Vector3(x, y, z);
    return {
      static_cast<float>(p_target.x()),
      static_cast<float>(p_target.y()),
      static_cast<float>(p_target.z()),
      intensity,
      time_offset};
  }

  void onMergeCustomCloud(
    const int lidar_index,
    livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    TimedCloud cloud;
    cloud.stamp = rclcpp::Time(msg->header.stamp);
    cloud.points.reserve(msg->points.size());
    for (const auto & point : msg->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      cloud.points.push_back(transformLidarPoint(
        lidar_index,
        point.x,
        point.y,
        point.z,
        static_cast<float>(point.reflectivity),
        static_cast<double>(point.offset_time) * 1.0e-9));
    }
    pushMergeCloud(lidar_index, std::move(cloud));
  }

  void onMergePointCloud(
    const int lidar_index,
    sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto * x_field = findPointField(*msg, "x");
    const auto * y_field = findPointField(*msg, "y");
    const auto * z_field = findPointField(*msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr || msg->point_step == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "LiDAR merge input cloud missing xyz fields");
      return;
    }
    auto * intensity_field = findPointField(*msg, "intensity");
    if (intensity_field == nullptr) {
      intensity_field = findPointField(*msg, "reflectivity");
    }
    const auto * time_field = findPointField(*msg, "timestamp");
    if (time_field == nullptr) {
      time_field = findPointField(*msg, "time");
    }
    if (time_field == nullptr) {
      time_field = findPointField(*msg, "t");
    }
    if (time_field == nullptr) {
      time_field = findPointField(*msg, "offset_time");
    }

    TimedCloud cloud;
    cloud.stamp = rclcpp::Time(msg->header.stamp);
    const auto point_count = static_cast<std::size_t>(msg->width) * msg->height;
    if (point_count == 0) {
      return;
    }
    cloud.points.reserve(point_count);
    const auto * data = msg->data.data();
    const double first_time = time_field != nullptr ?
      readPointFieldNumeric(data, time_field, 0.0) :
      0.0;

    for (std::size_t i = 0; i < point_count; ++i) {
      const auto * base = data + i * msg->point_step;
      const auto x = static_cast<float>(readPointFieldNumeric(base, x_field));
      const auto y = static_cast<float>(readPointFieldNumeric(base, y_field));
      const auto z = static_cast<float>(readPointFieldNumeric(base, z_field));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const auto intensity = static_cast<float>(readPointFieldNumeric(base, intensity_field, 0.0));
      double time_offset = 0.0;
      if (time_field != nullptr) {
        time_offset = readPointFieldNumeric(base, time_field, first_time) - first_time;
        if (std::abs(time_offset) > 1.0) {
          time_offset *= 1.0e-9;
        }
      }
      cloud.points.push_back(transformLidarPoint(lidar_index, x, y, z, intensity, time_offset));
    }
    pushMergeCloud(lidar_index, std::move(cloud));
  }

  void pushMergeCloud(const int lidar_index, TimedCloud cloud)
  {
    if (cloud.points.empty()) {
      return;
    }
    if (monitor_raw_lidar_ && lidar_index == 0) {
      last_lidar_time_ = now();
      ++lidar_count_;
    }

    {
      std::lock_guard<std::mutex> lock(lidar_merge_mutex_);
      auto & queue = lidar_index == 0 ? lidar1_queue_ : lidar2_queue_;
      queue.push_back(std::move(cloud));
      while (static_cast<int>(queue.size()) > lidar_merge_max_queue_size_) {
        queue.pop_front();
      }
      tryPublishMergedLidarLocked();
    }
  }

  static double stampSeconds(const rclcpp::Time & stamp)
  {
    return static_cast<double>(stamp.nanoseconds()) * 1.0e-9;
  }

  void tryPublishMergedLidarLocked()
  {
    if (!merged_lidar_pub_) {
      return;
    }
    while (!lidar1_queue_.empty()) {
      if (lidar2_queue_.empty()) {
        return;
      }

      const auto & lidar1 = lidar1_queue_.front();
      const double lidar1_stamp = stampSeconds(lidar1.stamp);
      auto best_it = lidar2_queue_.begin();
      double best_diff = std::abs(stampSeconds(best_it->stamp) - lidar1_stamp);
      for (auto it = std::next(lidar2_queue_.begin()); it != lidar2_queue_.end(); ++it) {
        const double diff = std::abs(stampSeconds(it->stamp) - lidar1_stamp);
        if (diff < best_diff) {
          best_diff = diff;
          best_it = it;
        }
      }

      if (best_diff <= lidar_merge_sync_tolerance_) {
        publishMergedLidar(lidar1, &(*best_it));
        lidar2_queue_.erase(lidar2_queue_.begin(), std::next(best_it));
        lidar1_queue_.pop_front();
        continue;
      }

      const double newest_lidar2_stamp = stampSeconds(lidar2_queue_.back().stamp);
      if (newest_lidar2_stamp + lidar_merge_sync_tolerance_ < lidar1_stamp) {
        lidar2_queue_.pop_front();
        continue;
      }
      if (lidar1_stamp + lidar_merge_sync_tolerance_ < newest_lidar2_stamp) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "LiDAR merge sync miss: nearest dt=%.4fs tolerance=%.4fs",
          best_diff,
          lidar_merge_sync_tolerance_);
        if (lidar_merge_publish_lidar1_on_sync_miss_) {
          publishMergedLidar(lidar1, nullptr);
        }
        lidar1_queue_.pop_front();
        continue;
      }
      return;
    }
  }

  void publishMergedLidar(const TimedCloud & lidar1, const TimedCloud * lidar2) const
  {
    const std::size_t lidar2_size = lidar2 == nullptr ? 0 : lidar2->points.size();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = lidar1.stamp;
    cloud.header.frame_id = target_frame_;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(lidar1.points.size() + lidar2_size);

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> time_it(cloud, "time");

    const double base_stamp = stampSeconds(lidar1.stamp);
    auto write_point = [&](const TimedCloud & source, const TimedPoint & point) {
      *x_it = point.x;
      *y_it = point.y;
      *z_it = point.z;
      *intensity_it = point.intensity;
      const double source_delta = stampSeconds(source.stamp) - base_stamp;
      *time_it = static_cast<float>(std::max(0.0, source_delta + point.time_offset) * 1.0e9);
      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
      ++time_it;
    };

    for (const auto & point : lidar1.points) {
      write_point(lidar1, point);
    }
    if (lidar2 != nullptr) {
      for (const auto & point : lidar2->points) {
        write_point(*lidar2, point);
      }
    }
    merged_lidar_pub_->publish(cloud);
  }

  void publishStaticTransform()
  {
    publishLidarStaticTransform(lidar_frame_, target_to_lidar_translation_, target_to_lidar_quaternion_);
    if (lidarMergeEnabled() && !lidar2_frame_.empty()) {
      publishLidarStaticTransform(
        lidar2_frame_, target_to_lidar2_translation_, target_to_lidar2_quaternion_);
    }
  }

  void publishLidarStaticTransform(
    const std::string & child_frame,
    const tf2::Vector3 & translation,
    const tf2::Quaternion & rotation)
  {
    if (child_frame.empty()) {
      return;
    }
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = target_frame_;
    msg.child_frame_id = child_frame;
    msg.transform.translation.x = translation.x();
    msg.transform.translation.y = translation.y();
    msg.transform.translation.z = translation.z();
    msg.transform.rotation = tf2::toMsg(rotation);
    if (output_static_tf_broadcaster_) {
      output_static_tf_broadcaster_->sendTransform(msg);
    }
  }

  tf2::Quaternion yawOnlyQuaternion(const geometry_msgs::msg::Quaternion & orientation) const
  {
    tf2::Quaternion q;
    tf2::fromMsg(orientation, q);
    q.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, yaw);
    q_yaw.normalize();
    return q_yaw;
  }

  void publishHeightMapFrameTransform(const nav_msgs::msg::Odometry & odom)
  {
    // Keep a single external TF tree: map/odom -> target(base_link) ->
    // base_link_gravity.  Publishing gravity directly from odom creates a
    // separate branch that can look localized while base_link's other children
    // remain in a different tree.
    if (height_map_frame_.empty() || target_frame_.empty() || height_map_frame_ == target_frame_) {
      return;
    }

    tf2::Quaternion q_map_target;
    tf2::fromMsg(odom.pose.pose.orientation, q_map_target);
    q_map_target.normalize();
    const tf2::Quaternion q_map_height = yawOnlyQuaternion(odom.pose.pose.orientation);
    const tf2::Vector3 p_map_target(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);
    const tf2::Vector3 p_map_height(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      latest_height_origin_z_);
    tf2::Quaternion q_target_height = q_map_target.inverse() * q_map_height;
    q_target_height.normalize();
    const tf2::Vector3 p_target_height = tf2::quatRotate(
      q_map_target.inverse(), p_map_height - p_map_target);

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = odom.header.stamp;
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      msg.header.stamp = now();
    }
    msg.header.frame_id = target_frame_;
    msg.child_frame_id = height_map_frame_;
    msg.transform.translation.x = p_target_height.x();
    msg.transform.translation.y = p_target_height.y();
    msg.transform.translation.z = p_target_height.z();
    msg.transform.rotation = tf2::toMsg(q_target_height);
    if (output_tf_broadcaster_) {
      output_tf_broadcaster_->sendTransform(msg);
    }
  }

  void publishTargetFrameTransform(const nav_msgs::msg::Odometry & odom)
  {
    const std::string & parent_frame = odom.header.frame_id.empty() ? odom_frame_ : odom.header.frame_id;
    if (target_frame_.empty() || target_frame_ == parent_frame) {
      return;
    }

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = odom.header.stamp;
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      msg.header.stamp = now();
    }
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = target_frame_;
    msg.transform.translation.x = odom.pose.pose.position.x;
    msg.transform.translation.y = odom.pose.pose.position.y;
    msg.transform.translation.z = odom.pose.pose.position.z;
    msg.transform.rotation = odom.pose.pose.orientation;
    if (output_tf_broadcaster_) {
      output_tf_broadcaster_->sendTransform(msg);
    }
  }

  void startExternalProcesses()
  {
    if (start_lidar_driver_) {
      auto command = lidar_driver_command_;
      if (command.empty()) {
        command = {"ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"};
      }
      child_processes_.start(get_logger(), "Livox driver", command);

      if (lidarMergeEnabled()) {
        if (lidar_driver2_command_.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "raw_lidar2_topic is set but lidar_driver2_command is empty; "
            "expecting the second LiDAR topic to be published externally.");
        } else {
          child_processes_.start(get_logger(), "Livox driver 2", lidar_driver2_command_);
        }
      }
    }

    if (start_point_lio_) {
      auto command = point_lio_command_;
      if (command.empty()) {
        command = defaultPointLioCommand();
      }
      child_processes_.start(get_logger(), "Point-LIO", command);
    }
  }

  std::vector<std::string> defaultPointLioCommand() const
  {
    std::string config_file = point_lio_config_file_;
    if (config_file.empty()) {
      config_file = ament_index_cpp::get_package_share_directory("autonomy_light") +
        "/config/point_lio_mid360.yaml";
    }
    const auto target_to_point_lio_body = pointLioTargetToBodyTransform(config_file);
    const std::string point_lio_lidar_topic = lidarMergeEnabled() ? merged_lidar_topic_ : raw_lidar_topic_;
    const std::string point_lio_lidar_msg_type = lidarMergeEnabled() ? "pointcloud2" : raw_lidar_msg_type_;
    const auto child_to_body_t = lidarMergeEnabled() ?
      std::vector<double>{0.0, 0.0, 0.0} :
      std::vector<double>{
        target_to_point_lio_body.translation.x(),
        target_to_point_lio_body.translation.y(),
        target_to_point_lio_body.translation.z()
      };
    const auto child_to_body_r = lidarMergeEnabled() ?
      tf2::Matrix3x3(tf2::Quaternion::getIdentity()) :
      target_to_point_lio_body.rotation;
    const bool point_lio_local_map_en = false;
    const bool point_lio_scan_publish_en =
      !mapping_only_ &&
      (cloud_registered_fill_enabled_ || savedMapRelocalizationActive() || point_lio_global_map_enabled_);

    auto command = std::vector<std::string>{
      "ros2", "run", "autonomy_light", "autonomy_light_pointlio_mapping",
      "--ros-args",
      "--params-file", config_file,
      "-p", "common.lid_topic:=" + point_lio_lidar_topic,
      "-p", "common.imu_topic:=" + raw_imu_topic_,
      "-p", "common.lidar_msg_type:=" + point_lio_lidar_msg_type,
      "-p", "odom_header_frame_id:=" + odom_frame_,
      "-p", "odom_child_frame_id:=" + target_frame_,
      "-p", "odom.child_to_body_T:=" + vectorParam(child_to_body_t),
      "-p", "odom.child_to_body_R:=" + matrixParam(child_to_body_r),
      "-p", "preprocess.lidar_type:=1",
      "-p", "preprocess.timestamp_unit:=3",
      "-p", "preprocess.scan_line:=4",
      "-p", "preprocess.blind:=0.5",
      "-p", "point_filter_num:=1",
      "-p", std::string("publish.path_en:=") + (mapping_only_ ? "false" : "true"),
      "-p", std::string("publish.scan_publish_en:=") +
        (point_lio_scan_publish_en ? "true" : "false"),
      "-p", "publish.scan_bodyframe_pub_en:=false",
      "-p", std::string("publish.local_map_en:=") + (point_lio_local_map_en ? "true" : "false"),
      "-p", "publish.local_map_topic:=" + point_lio_map_topic_,
      "-p", "publish.local_map_x_length:=" + shortDouble(pointLioLocalMapXLength()),
      "-p", "publish.local_map_y_length:=" + shortDouble(pointLioLocalMapYLength()),
      "-p", "publish.local_map_z_length:=" + shortDouble(pointLioLocalMapZLength()),
      "-p", "publish.local_map_every_n_frames:=1",
      "-p", std::string("pcd_save.pcd_save_en:=") + (point_lio_pcd_save_en_ ? "true" : "false"),
      "-p", "pcd_save.interval:=" + std::to_string(point_lio_pcd_save_interval_),
      "-p", "runtime_pos_log_enable:=false",
    };
    if (child_use_sim_time_) {
      command.push_back("-p");
      command.push_back("use_sim_time:=true");
    }
    if (lidarMergeEnabled()) {
      command.push_back("-p");
      command.push_back("mapping.extrinsic_T:=[0.0,0.0,0.0]");
      command.push_back("-p");
      command.push_back("mapping.extrinsic_R:=[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]");
    }
    return command;
  }

  double pointLioLocalMapXLength() const
  {
    return std::max(1.0, grid_spec_.x_length + 2.0 * std::abs(target_to_lidar_translation_.x()) + 1.0);
  }

  double pointLioLocalMapYLength() const
  {
    return std::max(1.0, grid_spec_.y_length + 2.0 * std::abs(target_to_lidar_translation_.y()) + 1.0);
  }

  double pointLioLocalMapZLength() const
  {
    return std::max(
      1.0,
      (grid_spec_.max_z - grid_spec_.min_z) + 2.0 * std::abs(target_to_lidar_translation_.z()) + 1.0);
  }

  RigidTransform pointLioTargetToBodyTransform(const std::string & config_file) const
  {
    const auto body_to_lidar_t = parseYamlVector(
      config_file,
      "extrinsic_T",
      3,
      {0.0, 0.0, 0.0});
    const tf2::Vector3 body_p_lidar(
      body_to_lidar_t[0],
      body_to_lidar_t[1],
      body_to_lidar_t[2]);

    // Point-LIO odometry is already expressed in its body axes. target_to_lidar_rpy
    // only describes the raw LiDAR TF, so odom child correction must not rotate it.
    RigidTransform target_to_body;
    target_to_body.translation = target_to_lidar_translation_ - body_p_lidar;
    return target_to_body;
  }

  void onLidarCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    (void)msg;
    last_lidar_time_ = now();
    ++lidar_count_;
  }

  void onPointLioMap(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (debug_local_map_pub_) {
      auto debug_msg = *msg;
      debug_local_map_pub_->publish(debug_msg);
    }

    auto points = std::make_shared<std::vector<MapPoint>>();
    points->reserve(static_cast<std::size_t>(msg->width) * msg->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");

      for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it) {
        if (!std::isfinite(*x_it) || !std::isfinite(*y_it) || !std::isfinite(*z_it)) {
          continue;
        }
        points->push_back({*x_it, *y_it, *z_it});
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Point-LIO map cloud cannot be sampled: %s",
        ex.what());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_map_points_ = std::move(points);
      latest_map_frame_ = msg->header.frame_id;
      has_map_ = true;
    }
    last_map_time_ = now();
    ++map_count_;
  }

  void onPointLioRegistered(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto points = std::make_shared<std::vector<MapPoint>>();
    points->reserve(static_cast<std::size_t>(msg->width) * msg->height);
    PclCloud::Ptr registration_cloud(new PclCloud());
    registration_cloud->reserve(static_cast<std::size_t>(msg->width) * msg->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");

      for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it) {
        if (!std::isfinite(*x_it) || !std::isfinite(*y_it) || !std::isfinite(*z_it)) {
          continue;
        }
        points->push_back({*x_it, *y_it, *z_it});
        registration_cloud->push_back(pcl::PointXYZ(*x_it, *y_it, *z_it));
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Point-LIO registered cloud cannot be sampled: %s",
        ex.what());
      return;
    }

    tryRelocalize(registration_cloud);
    bool registered_cloud_relocalized = false;
    if (savedMapRelocalizationActive()) {
      Eigen::Matrix4f map_from_odom = Eigen::Matrix4f::Identity();
      bool relocalized = false;
      {
        std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
        map_from_odom = saved_map_from_odom_;
        relocalized = saved_map_relocalized_;
      }
      if (relocalized) {
        applySavedMapCorrectionToPoints(*points, map_from_odom);
        registered_cloud_relocalized = true;
      }
    }
    if (point_lio_global_map_enabled_ &&
      (!savedMapRelocalizationActive() || registered_cloud_relocalized))
    {
      updatePointLioGlobalMap(
        *points,
        registered_cloud_relocalized ? saved_map_frame_ :
        (msg->header.frame_id.empty() ? odom_frame_ : msg->header.frame_id));
    }
    {
      std::lock_guard<std::mutex> lock(registered_mutex_);
      latest_registered_points_ = std::move(points);
      has_registered_cloud_ = true;
    }
  }

  void updatePointLioGlobalMap(
    const std::vector<MapPoint> & points,
    const std::string & frame_id)
  {
    if (!point_lio_global_map_pub_ || points.empty()) {
      return;
    }
    PclCloud::Ptr incoming(new PclCloud());
    incoming->reserve(points.size());
    for (const auto & point : points) {
      incoming->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    const auto incoming_count = incoming->size();
    PclCloud::Ptr refined_incoming = removeStatisticalOutliers(
      incoming,
      point_lio_global_map_refine_mean_k_,
      point_lio_global_map_refine_stddev_multiplier_);
    if (refined_incoming->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Global-map refinement rejected an entire registered scan (%zu input points)",
        incoming_count);
      return;
    }
    incoming = voxelDownsample(refined_incoming, point_lio_global_map_voxel_leaf_size_);
    if (!point_lio_global_map_points_) {
      point_lio_global_map_points_.reset(new PclCloud());
    }
    *point_lio_global_map_points_ += *incoming;
    if (point_lio_global_map_use_for_height_map_) {
      PclCloud::Ptr fine_incoming(new PclCloud());
      fine_incoming->reserve(refined_incoming->size());
      *fine_incoming = *refined_incoming;
      fine_incoming = voxelDownsample(
        fine_incoming, point_lio_global_map_height_voxel_leaf_size_);
      if (!point_lio_global_map_height_points_) {
        point_lio_global_map_height_points_.reset(new PclCloud());
      }
      *point_lio_global_map_height_points_ += *fine_incoming;
    }

    const auto publish_time = std::chrono::steady_clock::now();
    if (last_point_lio_global_map_publish_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(publish_time - last_point_lio_global_map_publish_).count() <
      point_lio_global_map_publish_interval_sec_)
    {
      return;
    }
    last_point_lio_global_map_publish_ = publish_time;
    point_lio_global_map_points_ = voxelDownsample(
      point_lio_global_map_points_, point_lio_global_map_voxel_leaf_size_);
    PclCloud::Ptr refined_visual_cloud;
    if (point_lio_global_map_use_for_height_map_ && point_lio_global_map_height_points_) {
      point_lio_global_map_height_points_ = voxelDownsample(
        point_lio_global_map_height_points_, point_lio_global_map_height_voxel_leaf_size_);
      const auto fine_limit = static_cast<std::size_t>(point_lio_global_map_height_max_points_);
      if (point_lio_global_map_height_points_->size() > fine_limit) {
        PclCloud::Ptr capped_fine(new PclCloud());
        capped_fine->reserve(fine_limit);
        const std::size_t stride =
          (point_lio_global_map_height_points_->size() + fine_limit - 1) / fine_limit;
        for (std::size_t index = 0; index < point_lio_global_map_height_points_->size(); index += stride) {
          capped_fine->push_back(point_lio_global_map_height_points_->points[index]);
        }
        point_lio_global_map_height_points_ = std::move(capped_fine);
      }
      pcl::search::KdTree<pcl::PointXYZ>::Ptr fine_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
      fine_tree->setInputCloud(point_lio_global_map_height_points_);
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        point_lio_global_map_height_cloud_ = point_lio_global_map_height_points_;
        point_lio_global_map_height_tree_ = std::move(fine_tree);
      }
      refined_visual_cloud = voxelDownsample(
        point_lio_global_map_height_points_,
        point_lio_global_map_refined_visual_voxel_leaf_size_);
      const auto refined_limit =
        static_cast<std::size_t>(point_lio_global_map_refined_visual_max_points_);
      if (refined_visual_cloud->size() > refined_limit) {
        PclCloud::Ptr capped_refined(new PclCloud());
        capped_refined->reserve(refined_limit);
        const std::size_t stride =
          (refined_visual_cloud->size() + refined_limit - 1) / refined_limit;
        for (std::size_t index = 0; index < refined_visual_cloud->size(); index += stride) {
          capped_refined->push_back(refined_visual_cloud->points[index]);
        }
        refined_visual_cloud = std::move(capped_refined);
      }
    }
    const auto max_points = static_cast<std::size_t>(point_lio_global_map_max_points_);
    if (point_lio_global_map_points_->size() > max_points) {
      PclCloud::Ptr capped(new PclCloud());
      capped->reserve(max_points);
      const std::size_t stride = (point_lio_global_map_points_->size() + max_points - 1) / max_points;
      for (std::size_t index = 0; index < point_lio_global_map_points_->size(); index += stride) {
        capped->push_back(point_lio_global_map_points_->points[index]);
      }
      point_lio_global_map_points_ = std::move(capped);
    }

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*point_lio_global_map_points_, message);
    message.header.stamp = now();
    message.header.frame_id = frame_id;
    point_lio_global_map_pub_->publish(message);
    if (point_lio_global_map_refined_pub_ && refined_visual_cloud) {
      sensor_msgs::msg::PointCloud2 refined_message;
      pcl::toROSMsg(*refined_visual_cloud, refined_message);
      refined_message.header.stamp = message.header.stamp;
      refined_message.header.frame_id = frame_id;
      point_lio_global_map_refined_pub_->publish(refined_message);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Point-LIO global map refined: scan=%zu->%zu raw=%zu refined=%zu frame=%s",
      incoming_count,
      refined_incoming->size(),
      point_lio_global_map_points_->size(),
      refined_visual_cloud ? refined_visual_cloud->size() : 0U,
      frame_id.c_str());
  }

  bool buildElevationGridFromMap(ElevationGrid & grid)
  {
    std::shared_ptr<const std::vector<MapPoint>> map_points;
    std::shared_ptr<const std::vector<MapPoint>> registered_points;
    nav_msgs::msg::Odometry odom;
    bool using_saved_map = false;
    bool using_global_map = false;
    PclCloud::Ptr global_height_cloud;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr global_height_tree;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (saved_map_loaded_ && saved_map_points_) {
        map_points = saved_map_points_;
        using_saved_map = true;
      }
      if (!saved_map_loaded_ && point_lio_global_map_enabled_ &&
        point_lio_global_map_use_for_height_map_ && point_lio_global_map_height_cloud_ &&
        point_lio_global_map_height_tree_)
      {
        global_height_cloud = point_lio_global_map_height_cloud_;
        global_height_tree = point_lio_global_map_height_tree_;
      }
    }
    if (using_saved_map && savedMapRelocalizationActive()) {
      std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
      if (!saved_map_relocalized_) {
        return false;
      }
    }
    {
      std::lock_guard<std::mutex> lock(registered_mutex_);
      if (has_registered_cloud_ && latest_registered_points_) {
        registered_points = latest_registered_points_;
      }
    }
    if (!map_points && !global_height_cloud) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_odom_) {
        return false;
      }
      odom = latest_odom_;
    }

    if (global_height_cloud && global_height_tree) {
      const double query_radius = std::max(
        height_origin_floor_radius_,
        0.5 * std::hypot(grid_spec_.x_length, grid_spec_.y_length) + 2.0 * grid_spec_.resolution);
      std::vector<int> indices;
      std::vector<float> distances;
      global_height_tree->radiusSearch(
        pcl::PointXYZ(
          static_cast<float>(odom.pose.pose.position.x),
          static_cast<float>(odom.pose.pose.position.y),
          static_cast<float>(odom.pose.pose.position.z)),
        static_cast<float>(query_radius), indices, distances);
      auto nearby = std::make_shared<std::vector<MapPoint>>();
      nearby->reserve(indices.size());
      for (const int index : indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= global_height_cloud->size()) {
          continue;
        }
        const auto & point = global_height_cloud->points[static_cast<std::size_t>(index)];
        nearby->push_back({point.x, point.y, point.z});
      }
      if (!nearby->empty()) {
        map_points = std::move(nearby);
        using_global_map = true;
      }
    }

    grid = ElevationGrid(grid_spec_);
    grid.header.stamp = now();
    grid.header.frame_id = height_map_frame_;

    const tf2::Quaternion q_map_height = yawOnlyQuaternion(odom.pose.pose.orientation);
    const tf2::Quaternion q_height_map = q_map_height.inverse();
    static const std::vector<MapPoint> empty_map_points;
    const double height_origin_z = resolveHeightOriginZ(
      map_points ? *map_points : empty_map_points,
      odom,
      q_height_map);
    latest_height_origin_z_ = height_origin_z;
    const tf2::Vector3 p_map_height_origin(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      height_origin_z);

    const auto width = grid.spec.width();
    const auto height = grid.spec.height();
    const double x_min = grid.spec.xMin();
    const double x_max = grid.spec.xMax();
    const double y_min = grid.spec.yMin();
    const double y_max = grid.spec.yMax();
    const double roi_prefilter_radius =
      0.5 * std::hypot(grid.spec.x_length, grid.spec.y_length) +
      2.0 * grid.spec.resolution;
    const double roi_prefilter_radius2 = roi_prefilter_radius * roi_prefilter_radius;

    std::vector<std::vector<float>> cell_samples(static_cast<std::size_t>(width) * height);
    std::vector<int> support_counts(static_cast<std::size_t>(width) * height, 0);
    // Keep obstacle candidates separate from the terrain grid.  Terrain is allowed
    // to be spatially/temporally filtered; obstacle candidates are overlaid only
    // after those filters so a small step is never averaged into its surroundings.
    std::vector<CellHeight> obstacle_candidates(static_cast<std::size_t>(width) * height);
    std::vector<float> startup_floor_seed_samples;
    std::size_t local_observed_cells = 0;
    const bool initial_floor_seed_pending =
      initial_floor_seed_fill_enabled_ && !initial_floor_seed_fill_applied_;
    if (initial_floor_seed_pending && map_points) {
      startup_floor_seed_samples.reserve(map_points->size());
    }

    if (map_points) {
      for (const auto & point : *map_points) {
        if (using_saved_map || using_global_map) {
          const double dx = static_cast<double>(point.x) - odom.pose.pose.position.x;
          const double dy = static_cast<double>(point.y) - odom.pose.pose.position.y;
          if ((dx * dx + dy * dy) > roi_prefilter_radius2) {
            continue;
          }
        }
        const tf2::Vector3 p_height = tf2::quatRotate(
          q_height_map,
          tf2::Vector3(point.x, point.y, point.z) - p_map_height_origin);
        const double x = p_height.x();
        const double y = p_height.y();
        const double z = p_height.z();

        const bool z_in_range = z >= grid.spec.min_z && z <= grid.spec.max_z;
        if (!z_in_range) {
          continue;
        }
        if (initial_floor_seed_pending && isInitialFloorSeedRegion(grid, x, y)) {
          startup_floor_seed_samples.push_back(static_cast<float>(z));
        }
        if (x < x_min || x >= x_max || y < y_min || y >= y_max) {
          continue;
        }

        const auto col = static_cast<std::uint32_t>((x - x_min) / grid.spec.resolution);
        const auto row = static_cast<std::uint32_t>((y - y_min) / grid.spec.resolution);
        if (col >= width || row >= height) {
          continue;
        }

        const auto index = static_cast<std::size_t>(row) * width + col;
        cell_samples[index].push_back(static_cast<float>(z));
      }
    }

    for (std::size_t index = 0; index < cell_samples.size(); ++index) {
      if (cell_samples[index].empty()) {
        continue;
      }
      const auto cell = selectGroundCellHeight(cell_samples[index]);
      const std::uint32_t row = static_cast<std::uint32_t>(index / width);
      const std::uint32_t col = static_cast<std::uint32_t>(index % width);
      const auto obstacle = selectObstacleCellHeight(cell_samples[index]);
      if (std::isfinite(obstacle.height)) {
        projectObstacleCandidate(obstacle_candidates, width, height, row, col, obstacle);
      }
      if (!std::isfinite(cell.height)) {
        continue;
      }
      grid.height[index] = cell.height;
      support_counts[index] = cell.support_count;
      ++local_observed_cells;
    }

    const double local_coverage = localCoverage(local_observed_cells, grid.height.size());
    fillMissingFromRegisteredCells(grid, support_counts, registered_points, q_height_map, p_map_height_origin);
    if (!initial_floor_seed_pending) {
      mergeWithPreviousGrid(grid, support_counts);
    }
    fillInitialFloorFromRegisteredCloud(
      grid,
      support_counts,
      registered_points,
      q_height_map,
      p_map_height_origin,
      local_coverage);
    ++filter_frame_count_;
    applyIsolatedFilter(grid);
    fillHoles(grid);
    applyBilateralFilter(grid);
    interpolateMissingCells(grid);
    if (initial_floor_seed_pending) {
      initial_floor_seed_fill_applied_ =
        fillInitialMissingFromFloorSeed(grid, startup_floor_seed_samples);
    }
    fillRemainingCells(grid);
    updateLatestGroundGrid(grid);
    overlayObstacleCandidates(grid, obstacle_candidates);
    return true;
  }

  double resolveHeightOriginZ(
    const std::vector<MapPoint> & map_points,
    const nav_msgs::msg::Odometry & odom,
    const tf2::Quaternion & q_height_map)
  {
    const double odom_z = odom.pose.pose.position.z;
    double raw_origin_z = odom_z;
    if (height_origin_mode_ == "fixed") {
      raw_origin_z = height_origin_fixed_z_;
    } else if (height_origin_mode_ == "local_floor") {
      double floor_z = std::numeric_limits<double>::quiet_NaN();
      if (estimateLocalFloorOriginZ(map_points, odom, q_height_map, floor_z)) {
        raw_origin_z = floor_z;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Not enough local floor points for height_origin.mode=local_floor; falling back to odom z");
      }
    } else if (height_origin_mode_ != "odom") {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Unknown height_origin.mode '%s'; falling back to odom z",
        height_origin_mode_.c_str());
    }

    if (!std::isfinite(raw_origin_z)) {
      raw_origin_z = odom_z;
    }
    return filterHeightOriginZ(raw_origin_z);
  }

  bool estimateLocalFloorOriginZ(
    const std::vector<MapPoint> & map_points,
    const nav_msgs::msg::Odometry & odom,
    const tf2::Quaternion & q_height_map,
    double & floor_z) const
  {
    if (map_points.empty()) {
      return false;
    }

    const tf2::Vector3 p_map_target_xy(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      0.0);
    const double radius2 = height_origin_floor_radius_ * height_origin_floor_radius_;

    std::vector<float> z_candidates;
    z_candidates.reserve(map_points.size());
    for (const auto & point : map_points) {
      const tf2::Vector3 p_height_xy = tf2::quatRotate(
        q_height_map,
        tf2::Vector3(point.x, point.y, 0.0) - p_map_target_xy);
      const double dist2 = p_height_xy.x() * p_height_xy.x() + p_height_xy.y() * p_height_xy.y();
      if (dist2 > radius2 || !std::isfinite(point.z)) {
        continue;
      }
      z_candidates.push_back(point.z);
    }

    if (static_cast<int>(z_candidates.size()) < height_origin_floor_min_points_) {
      return false;
    }

    std::sort(z_candidates.begin(), z_candidates.end());
    const auto index = std::min(
      static_cast<std::size_t>(
        std::round(height_origin_floor_percentile_ * static_cast<double>(z_candidates.size() - 1))),
      z_candidates.size() - 1);
    floor_z = z_candidates[index];
    return std::isfinite(floor_z);
  }

  double filterHeightOriginZ(const double raw_origin_z)
  {
    if (!height_origin_initialized_) {
      height_origin_initialized_ = true;
      filtered_height_origin_z_ = raw_origin_z;
      return filtered_height_origin_z_;
    }

    double target_z = raw_origin_z;
    if (height_origin_max_step_ > 0.0) {
      const double delta = std::clamp(
        raw_origin_z - filtered_height_origin_z_,
        -height_origin_max_step_,
        height_origin_max_step_);
      target_z = filtered_height_origin_z_ + delta;
    }

    filtered_height_origin_z_ =
      height_origin_filter_alpha_ * target_z +
      (1.0 - height_origin_filter_alpha_) * filtered_height_origin_z_;
    return filtered_height_origin_z_;
  }

  CellHeight robustCellHeight(std::vector<float> & samples) const
  {
    std::sort(samples.begin(), samples.end());
    const float low = samples.front();
    const float support_limit = low + static_cast<float>(intra_cell_min_support_gap_);
    const auto support_end = std::upper_bound(samples.begin(), samples.end(), support_limit);
    const int support_count = static_cast<int>(std::distance(samples.begin(), support_end));

    if (support_count >= intra_cell_min_support_count_) {
      float sum = 0.0F;
      for (auto it = samples.begin(); it != support_end; ++it) {
        sum += *it;
      }
      return {sum / static_cast<float>(support_count), support_count};
    }

    const auto percentile_index = static_cast<std::size_t>(
      std::round(cell_height_percentile_ * static_cast<double>(samples.size() - 1)));
    const auto clamped_index = std::min(percentile_index, samples.size() - 1);
    return {samples[clamped_index], support_count};
  }

  CellHeight minZCellHeight(std::vector<float> & samples) const
  {
    if (static_cast<int>(samples.size()) < min_z_min_points_per_cell_) {
      return {};
    }

    if (!min_z_supported_min_enabled_ || min_z_min_points_per_cell_ <= 1) {
      const auto min_it = std::min_element(samples.begin(), samples.end());
      if (min_it == samples.end()) {
        return {};
      }
      return {*min_it, static_cast<int>(samples.size())};
    }

    std::sort(samples.begin(), samples.end());
    CellHeight floor_cell;
    for (auto candidate = samples.begin(); candidate != samples.end(); ++candidate) {
      const float support_limit = *candidate + static_cast<float>(min_z_support_band_);
      const auto support_end = std::upper_bound(candidate, samples.end(), support_limit);
      const int support_count = static_cast<int>(std::distance(candidate, support_end));
      if (support_count >= min_z_min_points_per_cell_) {
        float sum = 0.0F;
        for (auto it = candidate; it != support_end; ++it) {
          sum += *it;
        }
        floor_cell = {sum / static_cast<float>(support_count), support_count};
        break;
      }
    }

    if (!std::isfinite(floor_cell.height)) {
      return {};
    }

    return floor_cell;
  }

  CellHeight selectGroundCellHeight(std::vector<float> & samples)
  {
    if (elevation_backend_ == "autonomy_min_z" || elevation_backend_ == "min_z") {
      return minZCellHeight(samples);
    }
    if (elevation_backend_ != "robust") {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Unknown algorithm.elevation_backend '%s'; falling back to robust",
        elevation_backend_.c_str());
    }
    return robustCellHeight(samples);
  }

  CellHeight selectObstacleCellHeight(std::vector<float> samples) const
  {
    if (
      !min_z_obstacle_override_enabled_ ||
      static_cast<int>(samples.size()) < min_z_obstacle_min_points_)
    {
      return {};
    }

    std::sort(samples.begin(), samples.end());
    // Search down from the highest return for a supported top-surface cluster.
    // This is deliberately independent of min_points_per_cell: that parameter
    // governs smooth terrain confidence, while obstacles may be sparse.
    for (auto candidate = samples.end(); candidate != samples.begin();) {
      --candidate;
      const float support_limit = *candidate - static_cast<float>(min_z_obstacle_support_band_);
      const auto support_begin = std::lower_bound(samples.begin(), std::next(candidate), support_limit);
      const int support_count = static_cast<int>(std::distance(support_begin, std::next(candidate)));
      if (support_count < min_z_obstacle_min_points_) {
        continue;
      }

      return {*candidate, support_count};
    }
    return {};
  }

  static void retainHigherObstacle(
    std::vector<CellHeight> & obstacle_candidates,
    const std::size_t index,
    const CellHeight & obstacle)
  {
    if (index >= obstacle_candidates.size() || !std::isfinite(obstacle.height)) {
      return;
    }
    auto & current = obstacle_candidates[index];
    if (!std::isfinite(current.height) || obstacle.height > current.height) {
      current = obstacle;
    }
  }

  void projectObstacleCandidate(
    std::vector<CellHeight> & obstacle_candidates,
    const std::uint32_t width,
    const std::uint32_t height,
    const std::uint32_t row,
    const std::uint32_t col,
    const CellHeight & obstacle) const
  {
    const int radius = min_z_obstacle_projection_radius_cells_;
    const int row_i = static_cast<int>(row);
    const int col_i = static_cast<int>(col);
    const int width_i = static_cast<int>(width);
    const int height_i = static_cast<int>(height);
    const int radius2 = radius * radius;

    for (int dy = -radius; dy <= radius; ++dy) {
      const int nr = row_i + dy;
      if (nr < 0 || nr >= height_i) {
        continue;
      }
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius2) {
          continue;
        }
        const int nc = col_i + dx;
        if (nc < 0 || nc >= width_i) {
          continue;
        }
        const auto index = static_cast<std::size_t>(nr) * width + static_cast<std::size_t>(nc);
        retainHigherObstacle(obstacle_candidates, index, obstacle);
      }
    }
  }

  void overlayObstacleCandidates(
    ElevationGrid & ground_grid,
    const std::vector<CellHeight> & obstacle_candidates) const
  {
    const std::size_t count = std::min(ground_grid.height.size(), obstacle_candidates.size());
    for (std::size_t index = 0; index < count; ++index) {
      const auto & obstacle = obstacle_candidates[index];
      const float ground = ground_grid.height[index];
      if (
        std::isfinite(obstacle.height) &&
        std::isfinite(ground) &&
        obstacle.height - ground >= static_cast<float>(min_z_obstacle_min_height_))
      {
        ground_grid.height[index] = obstacle.height;
      }
    }
  }

  void mergeWithPreviousGrid(ElevationGrid & grid, const std::vector<int> & support_counts)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (!has_ground_grid_ || latest_ground_grid_.height.size() != grid.height.size()) {
      return;
    }

    for (std::size_t i = 0; i < grid.height.size(); ++i) {
      const float previous = latest_ground_grid_.height[i];
      if (!std::isfinite(previous)) {
        continue;
      }

      float & current = grid.height[i];
      if (!std::isfinite(current)) {
        if (fill_missing_from_previous_grid_) {
          current = previous;
        }
        continue;
      }

      const float diff = std::abs(current - previous);
      if (diff <= edge_mix_height_diff_ || diff <= robust_height_gate_) {
        current = static_cast<float>(
          temporal_alpha_ * current + (1.0 - temporal_alpha_) * previous);
      } else if (
        i < support_counts.size() &&
        support_counts[i] < edge_prefer_prev_support_count_)
      {
        current = previous;
      }
    }
  }

  void applyIsolatedFilter(ElevationGrid & grid) const
  {
    if (
      isolated_filter_radius_ <= 0 ||
      isolated_filter_min_support_neighbors_ <= 0 ||
      (filter_frame_count_ % isolated_filter_every_n_frames_) != 0)
    {
      return;
    }

    const int width = static_cast<int>(grid.spec.width());
    const int height = static_cast<int>(grid.spec.height());
    auto next = grid.height;

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        const auto index = static_cast<std::size_t>(row) * width + col;
        const float value = grid.height[index];
        if (!std::isfinite(value)) {
          continue;
        }
        std::vector<float> neighbors;
        int support = 0;
        collectNeighbors(grid, row, col, isolated_filter_radius_, neighbors);
        for (const auto neighbor : neighbors) {
          if (std::abs(neighbor - value) <= isolated_filter_support_height_diff_) {
            ++support;
          }
        }

        if (support >= isolated_filter_min_support_neighbors_ || neighbors.empty()) {
          continue;
        }

        const float median = medianValue(neighbors);
        if (std::isfinite(median) && std::abs(value - median) >= isolated_filter_outlier_height_diff_) {
          next[index] = median;
        }
      }
    }

    grid.height.swap(next);
  }

  void fillHoles(ElevationGrid & grid) const
  {
    if (hole_fill_radius_ <= 0 || hole_fill_min_neighbors_ <= 0) {
      return;
    }

    const int width = static_cast<int>(grid.spec.width());
    const int height = static_cast<int>(grid.spec.height());
    auto next = grid.height;

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        const auto index = static_cast<std::size_t>(row) * width + col;
        if (std::isfinite(grid.height[index])) {
          continue;
        }

        std::vector<float> neighbors;
        collectNeighbors(grid, row, col, hole_fill_radius_, neighbors);
        if (static_cast<int>(neighbors.size()) < hole_fill_min_neighbors_) {
          continue;
        }

        const auto minmax = std::minmax_element(neighbors.begin(), neighbors.end());
        if ((*minmax.second - *minmax.first) > hole_fill_max_height_diff_) {
          continue;
        }

        float sum = 0.0F;
        for (const auto value : neighbors) {
          sum += value;
        }
        next[index] = sum / static_cast<float>(neighbors.size());
      }
    }

    grid.height.swap(next);
  }

  void applyBilateralFilter(ElevationGrid & grid) const
  {
    if (
      bilateral_radius_ <= 0 ||
      bilateral_passes_ <= 0 ||
      (filter_frame_count_ % bilateral_every_n_frames_) != 0)
    {
      return;
    }

    const int width = static_cast<int>(grid.spec.width());
    const int height = static_cast<int>(grid.spec.height());
    const double spatial_denom = 2.0 * bilateral_sigma_spatial_ * bilateral_sigma_spatial_;
    const double height_denom = 2.0 * bilateral_sigma_height_ * bilateral_sigma_height_;

    for (int pass = 0; pass < bilateral_passes_; ++pass) {
      auto next = grid.height;
      for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
          const auto index = static_cast<std::size_t>(row) * width + col;
          const float center = grid.height[index];
          if (!std::isfinite(center)) {
            continue;
          }

          double weighted_sum = center;
          double weight_sum = 1.0;
          for (int dy = -bilateral_radius_; dy <= bilateral_radius_; ++dy) {
            const int nr = row + dy;
            if (nr < 0 || nr >= height) {
              continue;
            }
            for (int dx = -bilateral_radius_; dx <= bilateral_radius_; ++dx) {
              if (dx == 0 && dy == 0) {
                continue;
              }
              const int nc = col + dx;
              if (nc < 0 || nc >= width) {
                continue;
              }
              const auto neighbor_index = static_cast<std::size_t>(nr) * width + nc;
              const float neighbor = grid.height[neighbor_index];
              if (!std::isfinite(neighbor)) {
                continue;
              }

              const double height_diff = static_cast<double>(neighbor - center);
              if (std::abs(height_diff) > bilateral_max_height_diff_) {
                continue;
              }
              const double spatial_dist2 = static_cast<double>(dx * dx + dy * dy);
              const double weight = std::exp(
                -(spatial_dist2 / spatial_denom) - ((height_diff * height_diff) / height_denom));
              weighted_sum += weight * neighbor;
              weight_sum += weight;
            }
          }

          next[index] = static_cast<float>(weighted_sum / weight_sum);
        }
      }
      grid.height.swap(next);
    }
  }

  void collectNeighbors(
    const ElevationGrid & grid,
    const int row,
    const int col,
    const int radius,
    std::vector<float> & neighbors) const
  {
    const int width = static_cast<int>(grid.spec.width());
    const int height = static_cast<int>(grid.spec.height());
    neighbors.clear();
    neighbors.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1) - 1));

    for (int dy = -radius; dy <= radius; ++dy) {
      const int nr = row + dy;
      if (nr < 0 || nr >= height) {
        continue;
      }
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nc = col + dx;
        if (nc < 0 || nc >= width) {
          continue;
        }
        const auto index = static_cast<std::size_t>(nr) * width + nc;
        const float value = grid.height[index];
        if (std::isfinite(value)) {
          neighbors.push_back(value);
        }
      }
    }
  }

  static float medianValue(std::vector<float> values)
  {
    if (values.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  void fillFromPreviousGrid(ElevationGrid & grid)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (!has_grid_ || latest_grid_.height.size() != grid.height.size()) {
      return;
    }

    for (std::size_t i = 0; i < grid.height.size(); ++i) {
      if (!std::isfinite(grid.height[i]) && std::isfinite(latest_grid_.height[i])) {
        grid.height[i] = latest_grid_.height[i];
      }
    }
  }

  bool isInitialFloorSeedRegion(
    const ElevationGrid & grid,
    const double x,
    const double y) const
  {
    const double x_min = grid.spec.xMin();
    const double x_max = grid.spec.xMax();
    const double y_min = grid.spec.yMin();
    const double y_max = grid.spec.yMax();
    const double margin = std::max(initial_floor_seed_search_margin_, 0.0);
    const double side_width = std::max(initial_floor_seed_side_width_, grid.spec.resolution);

    const bool inside_roi = x >= x_min && x < x_max && y >= y_min && y < y_max;
    const bool inside_expanded =
      x >= (x_min - margin) && x < (x_max + margin) &&
      y >= (y_min - margin) && y < (y_max + margin);
    if (!inside_expanded) {
      return false;
    }
    if (!inside_roi) {
      return true;
    }

    return
      (x - x_min) <= side_width ||
      (x_max - x) <= side_width ||
      (y - y_min) <= side_width ||
      (y_max - y) <= side_width;
  }

  bool estimateInitialFloorSeed(std::vector<float> samples, float & seed_z) const
  {
    if (samples.empty()) {
      return false;
    }

    std::sort(samples.begin(), samples.end());
    const auto keep_count = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil(
        initial_floor_seed_lower_fraction_ * static_cast<double>(samples.size()))));
    samples.resize(std::min(keep_count, samples.size()));

    if (samples.size() == 1) {
      seed_z = samples.front();
      return std::isfinite(seed_z);
    }

    const float min_z = samples.front();
    const float max_z = samples.back();
    const double bin_size = std::max(1.0e-3, initial_floor_seed_cluster_band_);
    const auto bin_count = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil((max_z - min_z) / bin_size)) + 1);
    std::vector<int> bins(bin_count, 0);
    for (const auto z : samples) {
      const auto bin = std::min(
        bin_count - 1,
        static_cast<std::size_t>(std::floor((z - min_z) / bin_size)));
      ++bins[bin];
    }

    std::size_t best_bin = 0;
    int best_count = bins.front();
    for (std::size_t i = 1; i < bins.size(); ++i) {
      if (bins[i] > best_count) {
        best_bin = i;
        best_count = bins[i];
      }
    }

    const float bin_center = min_z + static_cast<float>(best_bin) * static_cast<float>(bin_size);
    std::vector<float> cluster;
    cluster.reserve(samples.size());
    for (const auto z : samples) {
      if (std::abs(z - bin_center) <= initial_floor_seed_cluster_band_) {
        cluster.push_back(z);
      }
    }

    seed_z = cluster.empty() ? medianValue(std::move(samples)) : medianValue(std::move(cluster));
    return std::isfinite(seed_z);
  }

  bool fillInitialMissingFromFloorSeed(
    ElevationGrid & grid,
    const std::vector<float> & startup_floor_seed_samples) const
  {
    const int width = static_cast<int>(grid.spec.width());
    const int height = static_cast<int>(grid.spec.height());
    if (width <= 0 || height <= 0 || grid.height.empty()) {
      return false;
    }

    std::vector<float> fallback_grid_samples;
    std::size_t missing_count = 0;
    fallback_grid_samples.reserve(grid.height.size());
    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        const auto index = static_cast<std::size_t>(row) * width + col;
        if (index >= grid.height.size()) {
          continue;
        }

        const float value = grid.height[index];
        if (!std::isfinite(value)) {
          ++missing_count;
          continue;
        }

        fallback_grid_samples.push_back(value);
      }
    }

    if (missing_count == 0) {
      return true;
    }

    float seed_z = std::numeric_limits<float>::quiet_NaN();
    std::vector<float> seed_samples = startup_floor_seed_samples;
    const char * source_name = "side-ring";
    if (!estimateInitialFloorSeed(std::move(seed_samples), seed_z)) {
      source_name = "observed-grid";
      if (!estimateInitialFloorSeed(std::move(fallback_grid_samples), seed_z)) {
        RCLCPP_WARN(
          get_logger(),
          "Initial floor seed fill skipped: no startup side-ring or observed-grid height samples.");
        return false;
      }
    }

    for (auto & height_value : grid.height) {
      if (!std::isfinite(height_value)) {
        height_value = seed_z;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Initial floor seed fill seeded %zu missing cell(s) with z=%.3f from %zu %s sample(s).",
      missing_count,
      static_cast<double>(seed_z),
      startup_floor_seed_samples.empty() ? fallback_grid_samples.size() : startup_floor_seed_samples.size(),
      source_name);
    return true;
  }

  void interpolateMissingCells(ElevationGrid & grid) const
  {
    const auto width = static_cast<int>(grid.spec.width());
    const auto height = static_cast<int>(grid.spec.height());
    if (width <= 0 || height <= 0) {
      return;
    }

    for (int pass = 0; pass < interpolation_max_passes_; ++pass) {
      bool changed = false;
      auto next = grid.height;
      for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
          const auto index = static_cast<std::size_t>(row) * width + col;
          if (std::isfinite(grid.height[index])) {
            continue;
          }

          float sum = 0.0F;
          int count = 0;
          float min_neighbor = std::numeric_limits<float>::infinity();
          float max_neighbor = -std::numeric_limits<float>::infinity();
          for (int dy = -1; dy <= 1; ++dy) {
            const int nr = row + dy;
            if (nr < 0 || nr >= height) {
              continue;
            }
            for (int dx = -1; dx <= 1; ++dx) {
              if (dx == 0 && dy == 0) {
                continue;
              }
              const int nc = col + dx;
              if (nc < 0 || nc >= width) {
                continue;
              }
              const auto neighbor_index = static_cast<std::size_t>(nr) * width + nc;
              const float value = grid.height[neighbor_index];
              if (std::isfinite(value)) {
                sum += value;
                ++count;
                min_neighbor = std::min(min_neighbor, value);
                max_neighbor = std::max(max_neighbor, value);
              }
            }
          }

          if (count >= interpolation_min_neighbors_) {
            if (
              std::isfinite(interpolation_max_height_diff_) &&
              interpolation_max_height_diff_ >= 0.0 &&
              (max_neighbor - min_neighbor) > static_cast<float>(interpolation_max_height_diff_))
            {
              continue;
            }
            next[index] = sum / static_cast<float>(count);
            changed = true;
          }
        }
      }

      grid.height.swap(next);
      if (!changed) {
        break;
      }
    }

  }

  void fillMissingFromRegisteredCells(
    ElevationGrid & grid,
    std::vector<int> & support_counts,
    const std::shared_ptr<const std::vector<MapPoint>> & registered_points,
    const tf2::Quaternion & q_height_map,
    const tf2::Vector3 & p_map_target) const
  {
    if (!cloud_registered_fill_enabled_ || !registered_points || registered_points->empty()) {
      return;
    }

    const auto width = grid.spec.width();
    const auto height = grid.spec.height();
    std::vector<std::vector<float>> registered_cell_samples(
      static_cast<std::size_t>(width) * height);

    const double x_min = grid.spec.xMin();
    const double x_max = grid.spec.xMax();
    const double y_min = grid.spec.yMin();
    const double y_max = grid.spec.yMax();

    for (const auto & point : *registered_points) {
      const tf2::Vector3 p_height = tf2::quatRotate(
        q_height_map,
        tf2::Vector3(point.x, point.y, point.z) - p_map_target);
      const double x = p_height.x();
      const double y = p_height.y();
      const double z = p_height.z();

      if (x < x_min || x >= x_max || y < y_min || y >= y_max) {
        continue;
      }
      if (z < grid.spec.min_z || z > grid.spec.max_z) {
        continue;
      }

      const auto col = static_cast<std::uint32_t>((x - x_min) / grid.spec.resolution);
      const auto row = static_cast<std::uint32_t>((y - y_min) / grid.spec.resolution);
      if (col >= width || row >= height) {
        continue;
      }

      const auto index = static_cast<std::size_t>(row) * width + col;
      if (std::isfinite(grid.height[index])) {
        continue;
      }
      registered_cell_samples[index].push_back(static_cast<float>(z));
    }

    for (std::size_t index = 0; index < registered_cell_samples.size(); ++index) {
      auto & samples = registered_cell_samples[index];
      if (
        std::isfinite(grid.height[index]) ||
        static_cast<int>(samples.size()) < cloud_registered_fill_min_points_per_cell_)
      {
        continue;
      }

      std::sort(samples.begin(), samples.end());
      const auto sample_index = std::min(
        static_cast<std::size_t>(
          std::round(cloud_registered_fill_percentile_ * static_cast<double>(samples.size() - 1))),
        samples.size() - 1);
      grid.height[index] = samples[sample_index];
      if (index < support_counts.size()) {
        support_counts[index] = std::max(
          support_counts[index],
          static_cast<int>(samples.size()));
      }
    }
  }

  static double localCoverage(const std::size_t observed_cells, const std::size_t total_cells)
  {
    if (total_cells == 0) {
      return 0.0;
    }
    return static_cast<double>(observed_cells) / static_cast<double>(total_cells);
  }

  void fillInitialFloorFromRegisteredCloud(
    ElevationGrid & grid,
    const std::vector<int> & support_counts,
    const std::shared_ptr<const std::vector<MapPoint>> & registered_points,
    const tf2::Quaternion & q_height_map,
    const tf2::Vector3 & p_map_target,
    const double local_coverage) const
  {
    if (
      !cloud_registered_initial_floor_fill_enabled_ ||
      local_coverage > cloud_registered_initial_floor_max_coverage_ ||
      !registered_points ||
      registered_points->empty())
    {
      return;
    }

    float floor_z = std::numeric_limits<float>::quiet_NaN();
    if (!estimateRegisteredFloorHeight(grid, registered_points, q_height_map, p_map_target, floor_z)) {
      return;
    }

    for (std::size_t index = 0; index < grid.height.size(); ++index) {
      const bool has_enough_current_support =
        index < support_counts.size() &&
        support_counts[index] >= cloud_registered_initial_keep_min_support_;
      if (!has_enough_current_support || !std::isfinite(grid.height[index])) {
        grid.height[index] = floor_z;
      }
    }
  }

  bool estimateRegisteredFloorHeight(
    const ElevationGrid & grid,
    const std::shared_ptr<const std::vector<MapPoint>> & registered_points,
    const tf2::Quaternion & q_height_map,
    const tf2::Vector3 & p_map_target,
    float & floor_z) const
  {
    std::vector<float> z_candidates;
    z_candidates.reserve(registered_points->size());

    const double x_min = grid.spec.xMin();
    const double x_max = grid.spec.xMax();
    const double y_min = grid.spec.yMin();
    const double y_max = grid.spec.yMax();

    for (const auto & point : *registered_points) {
      const tf2::Vector3 p_height = tf2::quatRotate(
        q_height_map,
        tf2::Vector3(point.x, point.y, point.z) - p_map_target);
      const double x = p_height.x();
      const double y = p_height.y();
      const double z = p_height.z();

      if (x < x_min || x >= x_max || y < y_min || y >= y_max) {
        continue;
      }
      if (z < grid.spec.min_z || z > grid.spec.max_z) {
        continue;
      }
      z_candidates.push_back(static_cast<float>(z));
    }

    if (static_cast<int>(z_candidates.size()) < cloud_registered_floor_min_points_) {
      return false;
    }

    std::sort(z_candidates.begin(), z_candidates.end());
    const auto upper_index = std::min(
      z_candidates.size() - 1,
      static_cast<std::size_t>(std::round(0.60 * static_cast<double>(z_candidates.size() - 1))));
    z_candidates.resize(upper_index + 1);
    if (static_cast<int>(z_candidates.size()) < cloud_registered_floor_min_points_) {
      return false;
    }

    const auto minmax = std::minmax_element(z_candidates.begin(), z_candidates.end());
    const float min_z = *minmax.first;
    const float max_z = *minmax.second;
    const double bin_size = std::max(1.0e-3, cloud_registered_floor_support_band_);
    const std::size_t bin_count = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil((max_z - min_z) / bin_size)) + 1);
    std::vector<int> bins(bin_count, 0);
    for (const auto z : z_candidates) {
      const auto bin = std::min(
        bin_count - 1,
        static_cast<std::size_t>(std::floor((z - min_z) / bin_size)));
      ++bins[bin];
    }

    std::size_t best_bin = 0;
    int best_count = bins.front();
    for (std::size_t i = 1; i < bins.size(); ++i) {
      if (bins[i] > best_count) {
        best_bin = i;
        best_count = bins[i];
      }
    }

    if (best_count < cloud_registered_floor_min_points_) {
      const auto fallback_index = std::min(
        static_cast<std::size_t>(
          std::round(cloud_registered_fill_percentile_ * static_cast<double>(z_candidates.size() - 1))),
        z_candidates.size() - 1);
      floor_z = z_candidates[fallback_index];
      return std::isfinite(floor_z);
    }

    const float bin_center = min_z + static_cast<float>(best_bin) * static_cast<float>(bin_size);

    std::vector<float> floor_cluster;
    floor_cluster.reserve(z_candidates.size());
    for (const auto z : z_candidates) {
      if (std::abs(z - bin_center) <= cloud_registered_floor_support_band_) {
        floor_cluster.push_back(z);
      }
    }

    if (static_cast<int>(floor_cluster.size()) >= cloud_registered_floor_min_points_) {
      floor_z = medianValue(std::move(floor_cluster));
    } else {
      floor_z = bin_center;
    }
    return std::isfinite(floor_z);
  }

  void fillRemainingCells(ElevationGrid & grid) const
  {
    if (!std::isfinite(fill_remaining_height_)) {
      return;
    }

    for (auto & height_value : grid.height) {
      if (!std::isfinite(height_value)) {
        height_value = static_cast<float>(fill_remaining_height_);
      }
    }
  }

  void updateLatestGroundGrid(const ElevationGrid & grid)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    latest_ground_grid_ = grid;
    has_ground_grid_ = true;
  }

  void onPointLioOdom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Matrix4f map_from_odom = Eigen::Matrix4f::Identity();
    bool relocalized = false;
    {
      std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
      map_from_odom = saved_map_from_odom_;
      relocalized = saved_map_relocalized_;
    }
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      latest_raw_odom_ = *msg;
      has_raw_odom_ = true;
      latest_odom_ = relocalized ?
        savedMapCorrectedOdom(*msg, map_from_odom) : *msg;
      latest_odom_.child_frame_id = target_frame_;
      has_odom_ = true;
    }
    last_odom_time_ = now();
    ++odom_count_;
  }

  void onPointLioPath(nav_msgs::msg::Path::SharedPtr msg)
  {
    if (path_pub_) {
      Eigen::Matrix4f map_from_odom = Eigen::Matrix4f::Identity();
      bool relocalized = false;
      {
        std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
        map_from_odom = saved_map_from_odom_;
        relocalized = saved_map_relocalized_;
      }
      if (!relocalized) {
        path_pub_->publish(*msg);
        return;
      }
      auto corrected = *msg;
      corrected.header.frame_id = saved_map_frame_;
      for (auto & pose : corrected.poses) {
        pose.header.frame_id = saved_map_frame_;
        pose.pose = savedMapCorrectedPose(pose.pose, map_from_odom);
      }
      path_pub_->publish(corrected);
    }
  }

  void publishLatest()
  {
    ElevationGrid grid;
    if (height_map_manual_mode_) {
      grid = buildManualHeightGrid();
      {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        latest_grid_ = grid;
        latest_ground_grid_ = grid;
        has_grid_ = true;
        has_ground_grid_ = true;
      }
      height_map_pub_->publish(gridToPointCloud(grid));
      height_map_msg_pub_->publish(manualHeightMapMsg(grid));
    } else if (buildElevationGridFromMap(grid)) {
      {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        latest_grid_ = grid;
        has_grid_ = true;
      }
      height_map_pub_->publish(gridToPointCloud(grid));
      height_map_msg_pub_->publish(gridToHeightMapMsg(grid));
    }

    nav_msgs::msg::Odometry odom;
    bool publish_odom = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (has_odom_) {
        odom = latest_odom_;
        publish_odom = true;
      }
    }
    if (publish_odom) {
      odom_pub_->publish(odom);
      publishTargetFrameTransform(odom);
      publishHeightMapFrameTransform(odom);
    }
  }

  ElevationGrid buildManualHeightGrid() const
  {
    ElevationGrid grid(grid_spec_);
    grid.header.stamp = now();
    grid.header.frame_id = height_map_frame_;
    grid.height.assign(
      static_cast<std::size_t>(grid.spec.width()) * grid.spec.height(),
      -static_cast<float>(height_map_manual_value_));
    return grid;
  }

  sensor_msgs::msg::PointCloud2 gridToPointCloud(const ElevationGrid & grid) const
  {
    std::size_t valid_count = 0;
    for (const auto z : grid.height) {
      if (std::isfinite(z)) {
        ++valid_count;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = grid.header;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(valid_count);

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");

    const auto width = grid.spec.width();
    for (std::uint32_t row = 0; row < grid.spec.height(); ++row) {
      for (std::uint32_t col = 0; col < width; ++col) {
        const auto index = static_cast<std::size_t>(row) * width + col;
        const auto z = grid.height[index];
        if (!std::isfinite(z)) {
          continue;
        }

        *x_it = static_cast<float>(grid.spec.xMin() + (col + 0.5) * grid.spec.resolution);
        *y_it = static_cast<float>(grid.spec.yMin() + (row + 0.5) * grid.spec.resolution);
        *z_it = z;
        ++x_it;
        ++y_it;
        ++z_it;
      }
    }

    return cloud;
  }

  ::autonomy_light::msg::HeightMap manualHeightMapMsg(const ElevationGrid & grid) const
  {
    ::autonomy_light::msg::HeightMap msg;
    msg.header = grid.header;
    msg.resolution = static_cast<float>(grid.spec.resolution);
    msg.x_length = static_cast<float>(grid.spec.x_length);
    msg.y_length = static_cast<float>(grid.spec.y_length);

    const auto count = static_cast<std::size_t>(grid.spec.width()) * grid.spec.height();
    msg.data.assign(count, static_cast<float>(height_map_manual_value_));
    return msg;
  }

  ::autonomy_light::msg::HeightMap gridToHeightMapMsg(const ElevationGrid & grid) const
  {
    ::autonomy_light::msg::HeightMap msg;
    msg.header = grid.header;
    msg.resolution = static_cast<float>(grid.spec.resolution);
    msg.x_length = static_cast<float>(grid.spec.x_length);
    msg.y_length = static_cast<float>(grid.spec.y_length);

    const auto count = static_cast<std::size_t>(grid.spec.width()) * grid.spec.height();
    msg.data.assign(count, 0.0F);

    const double z_min = clipping_enabled_ ?
      std::min(clipping_min_z_, clipping_max_z_) :
      grid.spec.min_z;
    const double z_max = clipping_enabled_ ?
      std::max(clipping_min_z_, clipping_max_z_) :
      grid.spec.max_z;
    const float base_height = static_cast<float>(std::max(0.0, z_max));

    for (std::size_t index = 0; index < count && index < grid.height.size(); ++index) {
      const float z = grid.height[index];
      if (!std::isfinite(z)) {
        msg.data[index] = base_height;
        continue;
      }

      const float z_clamped = static_cast<float>(
        std::clamp(static_cast<double>(z), z_min, z_max));
      msg.data[index] = std::clamp(base_height - z_clamped, 0.0F, base_height);
    }

    return msg;
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    const bool lidar_stale = monitor_raw_lidar_ &&
      (lidar_count_ == 0 || (now() - last_lidar_time_) > rclcpp::Duration::from_seconds(2.0));
    const bool odom_stale = odom_count_ == 0 ||
      (now() - last_odom_time_) > rclcpp::Duration::from_seconds(2.0);
    const bool map_stale = !saved_map_loaded_ &&
      (map_count_ == 0 || (now() - last_map_time_) > rclcpp::Duration::from_seconds(2.0));
    bool waiting_for_saved_map_relocalization = false;
    double saved_map_fitness = std::numeric_limits<double>::infinity();
    std::string saved_map_relocalization_phase;
    std::size_t saved_map_submap_points = 0;
    double saved_map_submap_elapsed_sec = 0.0;
    if (savedMapRelocalizationActive()) {
      std::lock_guard<std::mutex> lock(saved_map_localization_mutex_);
      waiting_for_saved_map_relocalization = !saved_map_relocalized_;
      saved_map_fitness = saved_map_last_fitness_;
      saved_map_relocalization_phase = saved_map_relocalization_phase_;
      saved_map_submap_points = saved_map_relocalization_submap_points_;
      saved_map_submap_elapsed_sec = saved_map_relocalization_elapsed_sec_;
    }

    if (height_map_manual_mode_) {
      msg.data = "ready:manual_height_map:value=" +
        shortDouble(height_map_manual_value_) +
        ":publish_hz=" + shortDouble(publish_rate_hz_);
    } else if (mapping_only_) {
      msg.data = "mapping_only:point_lio_pcd_save=" +
        std::string(point_lio_pcd_save_en_ ? "true" : "false");
    } else if (lidar_stale) {
      msg.data = "waiting_for_lidar";
    } else if (odom_stale) {
      msg.data = "degraded:waiting_for_point_lio_odom";
    } else if (waiting_for_saved_map_relocalization) {
      msg.data = "relocalizing:phase=" + saved_map_relocalization_phase +
        ":submap_points=" + std::to_string(saved_map_submap_points) +
        ":elapsed_sec=" + shortDouble(saved_map_submap_elapsed_sec);
    } else if (map_stale) {
      msg.data = "degraded:waiting_for_point_lio_map";
    } else {
      msg.data = "ready:lidar=" +
        (monitor_raw_lidar_ ? std::to_string(lidar_count_) : std::string("unmonitored")) +
        ":odom=" + std::to_string(odom_count_) +
        ":map=" + (saved_map_loaded_ ? std::string("saved") : std::to_string(map_count_)) +
        (savedMapRelocalizationActive() ? ":relocalization_fitness=" +
        shortDouble(saved_map_fitness) : "") +
        ":publish_hz=" + shortDouble(publish_rate_hz_);
    }
    heartbeat_pub_->publish(msg);
  }

  std::string target_frame_{"base_link"};
  std::string height_map_frame_{"base_link_gravity"};
  std::string lidar_frame_{"mid360"};
  std::string lidar2_frame_{"lidar2_link"};
  std::string odom_frame_{"odom"};
  int internal_ros_domain_id_{-1};
  int external_ros_domain_id_{-1};
  int debug_local_map_ros_domain_id_{-1};
  std::string debug_local_map_topic_{"/point_lio/local_map"};
  std::vector<double> target_to_lidar_xyz_{0.0, 0.0, 0.3};
  std::vector<double> target_to_lidar_rpy_{0.0, 0.0, 0.0};
  std::vector<double> target_to_lidar2_xyz_{0.0, 0.0, 0.3};
  std::vector<double> target_to_lidar2_rpy_{0.0, 0.0, 0.0};
  tf2::Vector3 target_to_lidar_translation_{0.0, 0.0, 0.3};
  tf2::Matrix3x3 target_to_lidar_rotation_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion target_to_lidar_quaternion_{tf2::Quaternion::getIdentity()};
  tf2::Vector3 target_to_lidar2_translation_{0.0, 0.0, 0.3};
  tf2::Matrix3x3 target_to_lidar2_rotation_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion target_to_lidar2_quaternion_{tf2::Quaternion::getIdentity()};

  GridSpec grid_spec_;
  double publish_rate_hz_{50.0};
  bool mapping_only_{false};
  bool point_lio_pcd_save_en_{false};
  int point_lio_pcd_save_interval_{-1};
  double child_shutdown_grace_sec_{0.8};
  std::string height_origin_mode_{"local_floor"};
  double height_origin_fixed_z_{0.0};
  double height_origin_filter_alpha_{0.25};
  double height_origin_max_step_{0.03};
  double height_origin_floor_radius_{0.6};
  double height_origin_floor_percentile_{0.20};
  int height_origin_floor_min_points_{20};
  bool height_origin_initialized_{false};
  double filtered_height_origin_z_{0.0};
  double latest_height_origin_z_{0.0};
  std::string raw_lidar_topic_{"/livox/lidar"};
  std::string raw_lidar2_topic_;
  std::string raw_lidar_msg_type_{"livox_custom"};
  std::string raw_imu_topic_{"/livox/imu"};
  std::string raw_imu2_topic_{"/livox2/imu"};
  bool monitor_raw_lidar_{false};
  std::string merged_lidar_topic_{"/autonomy_light/merged_lidar"};
  double lidar_merge_sync_tolerance_{0.005};
  int lidar_merge_max_queue_size_{8};
  bool lidar_merge_publish_lidar1_on_sync_miss_{false};
  std::string point_lio_odom_topic_{"/aft_mapped_to_init"};
  std::string point_lio_map_topic_{"/point_lio/local_map"};
  std::string point_lio_path_topic_{"/path"};
  std::string point_lio_registered_topic_{"/cloud_registered"};
  bool point_lio_global_map_enabled_{true};
  std::string point_lio_global_map_topic_{"/point_lio/global_map"};
  std::string point_lio_global_map_refined_topic_{"/point_lio/global_map_refined"};
  int point_lio_global_map_ros_domain_id_{-1};
  bool point_lio_global_map_use_for_height_map_{true};
  double point_lio_global_map_height_voxel_leaf_size_{0.025};
  int point_lio_global_map_height_max_points_{2000000};
  double point_lio_global_map_voxel_leaf_size_{0.10};
  double point_lio_global_map_publish_interval_sec_{1.0};
  int point_lio_global_map_max_points_{500000};
  double point_lio_global_map_refined_visual_voxel_leaf_size_{0.02};
  int point_lio_global_map_refined_visual_max_points_{1500000};
  int point_lio_global_map_refine_mean_k_{16};
  double point_lio_global_map_refine_stddev_multiplier_{0.8};
  PclCloud::Ptr point_lio_global_map_points_;
  PclCloud::Ptr point_lio_global_map_height_points_;
  PclCloud::Ptr point_lio_global_map_height_cloud_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr point_lio_global_map_height_tree_;
  std::chrono::steady_clock::time_point last_point_lio_global_map_publish_{};
  std::string odom_output_topic_{"/autonomy_light/odom"};
  std::string height_map_topic_{"/autonomy_light/height_map"};
  std::string height_map_msg_topic_{"/autonomy_light/height_map_data"};
  std::string path_output_topic_{"/path"};
  std::string heartbeat_topic_{"/autonomy_light/heartbeat"};
  bool height_map_manual_mode_{false};
  double height_map_manual_value_{0.48};
  int interpolation_max_passes_{8};
  int interpolation_min_neighbors_{1};
  double interpolation_max_height_diff_{0.03};
  double fill_remaining_height_{0.0};
  bool initial_floor_seed_fill_enabled_{true};
  bool initial_floor_seed_fill_applied_{false};
  double initial_floor_seed_side_width_{0.20};
  double initial_floor_seed_search_margin_{0.40};
  double initial_floor_seed_cluster_band_{0.08};
  double initial_floor_seed_lower_fraction_{0.70};
  std::string elevation_backend_{"robust"};
  bool clipping_enabled_{false};
  double clipping_min_z_{-std::numeric_limits<double>::infinity()};
  double clipping_max_z_{std::numeric_limits<double>::infinity()};
  int min_z_min_points_per_cell_{1};
  bool min_z_supported_min_enabled_{false};
  double min_z_support_band_{0.03};
  bool min_z_obstacle_override_enabled_{false};
  double min_z_obstacle_min_height_{0.06};
  int min_z_obstacle_min_points_{2};
  double min_z_obstacle_support_band_{0.06};
  int min_z_obstacle_projection_radius_cells_{1};
  bool cloud_registered_fill_enabled_{true};
  double cloud_registered_fill_percentile_{0.15};
  int cloud_registered_fill_min_points_per_cell_{2};
  bool cloud_registered_initial_floor_fill_enabled_{true};
  double cloud_registered_initial_floor_max_coverage_{0.25};
  int cloud_registered_floor_min_points_{20};
  double cloud_registered_floor_support_band_{0.08};
  int cloud_registered_initial_keep_min_support_{3};
  double robust_height_gate_{0.04};
  double intra_cell_min_support_gap_{0.025};
  int intra_cell_min_support_count_{3};
  double edge_mix_height_diff_{0.035};
  int edge_prefer_prev_support_count_{0};
  bool fill_missing_from_previous_grid_{false};
  double cell_height_percentile_{0.20};
  double temporal_alpha_{1.0};
  int isolated_filter_radius_{1};
  int isolated_filter_min_support_neighbors_{2};
  double isolated_filter_support_height_diff_{0.025};
  double isolated_filter_outlier_height_diff_{0.05};
  int isolated_filter_every_n_frames_{2};
  int hole_fill_radius_{1};
  int hole_fill_min_neighbors_{3};
  double hole_fill_max_height_diff_{0.03};
  int bilateral_radius_{1};
  double bilateral_sigma_spatial_{1.1};
  double bilateral_sigma_height_{0.025};
  double bilateral_max_height_diff_{0.04};
  int bilateral_passes_{2};
  int bilateral_every_n_frames_{2};
  std::uint64_t filter_frame_count_{0};

  bool start_lidar_driver_{true};
  bool start_point_lio_{true};
  bool child_use_sim_time_{false};
  std::vector<std::string> lidar_driver_command_;
  std::vector<std::string> lidar_driver2_command_;
  std::vector<std::string> point_lio_command_;
  std::string point_lio_config_file_;

  std::mutex map_mutex_;
  std::mutex lidar_merge_mutex_;
  std::deque<TimedCloud> lidar1_queue_;
  std::deque<TimedCloud> lidar2_queue_;
  std::shared_ptr<const std::vector<MapPoint>> latest_map_points_;
  std::string latest_map_frame_;
  bool has_map_{false};
  std::shared_ptr<const std::vector<MapPoint>> saved_map_points_;
  bool saved_map_loaded_{false};
  std::string saved_map_file_;
  std::string saved_map_frame_{"odom"};
  std::string saved_map_topic_{"/autonomy_light/saved_map"};
  double saved_map_publish_voxel_leaf_size_{0.10};
  bool saved_map_localization_enabled_{true};
  bool saved_map_global_initialization_{true};
  double saved_map_localization_update_interval_sec_{0.5};
  double saved_map_initial_submap_duration_sec_{2.0};
  int saved_map_initial_submap_min_points_{500};
  int saved_map_initial_submap_max_points_{30000};
  double saved_map_scan_voxel_leaf_size_{0.10};
  double saved_map_voxel_leaf_size_{0.10};
  double saved_map_normal_radius_{0.45};
  double saved_map_feature_radius_{0.75};
  int saved_map_global_max_iterations_{5000};
  double saved_map_global_inlier_fraction_{0.20};
  double saved_map_max_correspondence_distance_{0.75};
  int saved_map_gicp_max_iterations_{50};
  double saved_map_max_fitness_{0.04};
  double saved_map_max_tracking_translation_step_{1.0};
  int saved_map_min_scan_points_{80};
  PclCloud::Ptr saved_map_localization_cloud_;
  PclFeatures::Ptr saved_map_localization_features_;
  PclCloud::Ptr saved_map_visualization_cloud_;
  PclCloud::Ptr initial_relocalization_submap_;
  std::chrono::steady_clock::time_point initial_relocalization_submap_start_{};
  std::mutex saved_map_localization_mutex_;
  Eigen::Matrix4f saved_map_from_odom_{Eigen::Matrix4f::Identity()};
  bool saved_map_relocalized_{false};
  double saved_map_last_fitness_{std::numeric_limits<double>::infinity()};
  std::string saved_map_relocalization_phase_{"idle"};
  std::size_t saved_map_relocalization_submap_points_{0};
  double saved_map_relocalization_elapsed_sec_{0.0};
  std::chrono::steady_clock::time_point last_saved_map_localization_attempt_{};
  std::mutex registered_mutex_;
  std::shared_ptr<const std::vector<MapPoint>> latest_registered_points_;
  bool has_registered_cloud_{false};
  std::mutex grid_mutex_;
  ElevationGrid latest_grid_;
  bool has_grid_{false};
  ElevationGrid latest_ground_grid_;
  bool has_ground_grid_{false};
  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_raw_odom_;
  bool has_raw_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_{false};
  std::uint64_t lidar_count_{0};
  std::uint64_t odom_count_{0};
  std::uint64_t map_count_{0};
  rclcpp::Time last_lidar_time_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Time last_odom_time_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Time last_map_time_{0, 0u, RCL_SYSTEM_TIME};

  ChildProcesses child_processes_;
  rclcpp::Context::SharedPtr output_context_;
  rclcpp::Node::SharedPtr output_node_;
  rclcpp::Context::SharedPtr debug_context_;
  rclcpp::Node::SharedPtr debug_node_;
  rclcpp::Context::SharedPtr global_map_context_;
  rclcpp::Node::SharedPtr global_map_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> output_executor_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> debug_executor_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> global_map_executor_;
  std::thread output_spin_thread_;
  std::thread debug_spin_thread_;
  std::thread global_map_spin_thread_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_lidar_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> output_static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> output_tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar1_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar2_cloud_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar1_custom_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar2_custom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_local_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr saved_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_lio_global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_lio_global_map_refined_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr height_map_pub_;
  rclcpp::Publisher<::autonomy_light::msg::HeightMap>::SharedPtr height_map_msg_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace
}  // namespace autonomy_light

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<autonomy_light::AutonomyLightNode>());
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "autonomy_light failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
