#include "autonomy_light/autonomy_light_node.hpp"

#include <filesystem>

namespace autonomy_light {

namespace {

std::string rosArray(const std::vector<double> &values) {
  std::string result{"["};
  for (std::size_t index = 0; index < values.size(); ++index) {
    char value[32];
    std::snprintf(value, sizeof(value), "%.17g", values[index]);
    std::string formatted(value);
    if (formatted.find_first_of(".eE") == std::string::npos) {
      formatted += ".0";
    }
    result += (index == 0U ? "" : ",") + formatted;
  }
  return result + "]";
}

} // namespace

AutonomyLightNode::AutonomyLightNode() : Node("autonomy_light") {
  loadParameters();
  loadSavedMap();
  createInterfaces();
  publishStaticTransform();
  startProcesses();
  RCLCPP_INFO(get_logger(),
              "Elevation runtime ready: source=%s transport=%s grid=%ux%u @ %.3fm frame=%s",
              mapper_config_.source.c_str(), height_map_transport_.c_str(),
              grid_spec_.width(), grid_spec_.height(), grid_spec_.resolution,
              global_frame_.c_str());
}

AutonomyLightNode::~AutonomyLightNode() {
  requestFastShutdown();
  saveMap();
}

void AutonomyLightNode::requestFastShutdown() {
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  if (timer_) {
    timer_->cancel();
  }
  children_.stopAll(child_shutdown_grace_sec_);
}

void AutonomyLightNode::loadParameters() {
  target_frame_ = declare_parameter<std::string>("target_frame", target_frame_);
  height_map_frame_ = declare_parameter<std::string>("height_map_frame", height_map_frame_);
  imu_frame_ = declare_parameter<std::string>("imu_frame", imu_frame_);
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", lidar_frame_);
  global_frame_ = declare_parameter<std::string>("odom_frame", global_frame_);
  target_to_lidar_xyz_ = declare_parameter<std::vector<double>>(
      "target_to_lidar_xyz", target_to_lidar_xyz_);
  target_to_lidar_rpy_ = declare_parameter<std::vector<double>>(
      "target_to_lidar_rpy", target_to_lidar_rpy_);
  imu_from_lidar_xyz_ = declare_parameter<std::vector<double>>(
      "imu_from_lidar_xyz", imu_from_lidar_xyz_);
  imu_from_lidar_rpy_ = declare_parameter<std::vector<double>>(
      "imu_from_lidar_rpy", imu_from_lidar_rpy_);
  if (target_to_lidar_xyz_.size() != 3U || target_to_lidar_rpy_.size() != 3U ||
      imu_from_lidar_xyz_.size() != 3U || imu_from_lidar_rpy_.size() != 3U) {
    throw std::invalid_argument("all *_xyz and *_rpy extrinsics must contain 3 values");
  }
  target_to_lidar_translation_ = tf2::Vector3(target_to_lidar_xyz_[0],
                                                target_to_lidar_xyz_[1],
                                                target_to_lidar_xyz_[2]);
  target_to_lidar_rotation_.setRPY(target_to_lidar_rpy_[0], target_to_lidar_rpy_[1],
                                   target_to_lidar_rpy_[2]);
  target_to_lidar_rotation_.normalize();
  imu_from_lidar_translation_ = tf2::Vector3(imu_from_lidar_xyz_[0],
                                               imu_from_lidar_xyz_[1],
                                               imu_from_lidar_xyz_[2]);
  imu_from_lidar_rotation_.setRPY(imu_from_lidar_rpy_[0], imu_from_lidar_rpy_[1],
                                  imu_from_lidar_rpy_[2]);
  imu_from_lidar_rotation_.normalize();
  const tf2::Quaternion lidar_from_imu = imu_from_lidar_rotation_.inverse();
  target_to_imu_rotation_ = target_to_lidar_rotation_ * lidar_from_imu;
  target_to_imu_rotation_.normalize();
  target_to_imu_translation_ = target_to_lidar_translation_ + tf2::quatRotate(
      target_to_lidar_rotation_, tf2::quatRotate(lidar_from_imu,
                                                   -imu_from_lidar_translation_));

  grid_spec_.resolution = std::max(0.01, declare_parameter<double>(
      "elevation_resolution", grid_spec_.resolution));
  grid_spec_.x_length = std::max(grid_spec_.resolution, declare_parameter<double>(
      "elevation_x_length", grid_spec_.x_length));
  grid_spec_.y_length = std::max(grid_spec_.resolution, declare_parameter<double>(
      "elevation_y_length", grid_spec_.y_length));
  grid_spec_.min_z = declare_parameter<double>("elevation_min_z", grid_spec_.min_z);
  grid_spec_.max_z = declare_parameter<double>("elevation_max_z", grid_spec_.max_z);
  publish_rate_hz_ = std::max(1.0, declare_parameter<double>(
      "publish_rate_hz", publish_rate_hz_));
  mapper_config_.grid = grid_spec_;
  mapper_config_.source = declare_parameter<std::string>("height_map.source", "rolling");
  mapper_config_.global_voxel_size = std::max(0.005, declare_parameter<double>(
      "live_global_map.voxel_leaf_size", mapper_config_.global_voxel_size));
  mapper_config_.global_max_voxels = static_cast<std::size_t>(std::max<std::int64_t>(
      1000, declare_parameter<std::int64_t>("live_global_map.max_points", 2'000'000)));
  mapper_config_.min_samples_per_cell = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("algorithm.min_z.min_points_per_cell",
                                          mapper_config_.min_samples_per_cell)));
  mapper_config_.ground_percentile = declare_parameter<double>(
      "algorithm.frame_aggregation.cell_height_percentile", mapper_config_.ground_percentile);
  mapper_config_.obstacle_min_height = std::max(0.0, declare_parameter<double>(
      "algorithm.min_z.obstacle_min_height", mapper_config_.obstacle_min_height));
  mapper_config_.rolling_max_age_sec = std::max(0.01, declare_parameter<double>(
      "rolling_elevation.max_age_sec", mapper_config_.rolling_max_age_sec));
  mapper_config_.rolling_upper_max_age_sec = std::max(0.01, declare_parameter<double>(
      "rolling_elevation.upper_max_age_sec", mapper_config_.rolling_upper_max_age_sec));
  mapper_config_.rolling_max_radius_m = std::max(0.1, declare_parameter<double>(
      "rolling_elevation.max_radius_m", mapper_config_.rolling_max_radius_m));
  mapper_config_.rolling_base_variance = std::max(1.0e-6, declare_parameter<double>(
      "rolling_elevation.base_variance", mapper_config_.rolling_base_variance));
  mapper_config_.rolling_range_variance_factor = std::max(0.0, declare_parameter<double>(
      "rolling_elevation.range_variance_factor", mapper_config_.rolling_range_variance_factor));
  mapper_config_.rolling_process_variance_per_sec = std::max(0.0, declare_parameter<double>(
      "rolling_elevation.process_variance_per_sec", mapper_config_.rolling_process_variance_per_sec));
  mapper_config_.rolling_max_variance = std::max(1.0e-6, declare_parameter<double>(
      "rolling_elevation.max_variance", mapper_config_.rolling_max_variance));
  mapper_config_.rolling_outlier_variance = std::max(0.0, declare_parameter<double>(
      "rolling_elevation.outlier_variance", mapper_config_.rolling_outlier_variance));
  mapper_config_.rolling_mahalanobis_threshold = std::max(0.1, declare_parameter<double>(
      "rolling_elevation.mahalanobis_threshold", mapper_config_.rolling_mahalanobis_threshold));
  mapper_config_.d435_base_variance = std::max(1.0e-6, declare_parameter<double>(
      "rolling_elevation.sensor.d435.base_variance", mapper_config_.d435_base_variance));
  mapper_config_.d435_range_variance_factor = std::max(0.0, declare_parameter<double>(
      "rolling_elevation.sensor.d435.range_variance_factor",
      mapper_config_.d435_range_variance_factor));
  mapper_config_.lidar_extrinsic_translation_std_m = std::max(0.0,
      declare_parameter<double>("rolling_elevation.sensor.lidar.extrinsic_translation_std_m",
                                mapper_config_.lidar_extrinsic_translation_std_m));
  mapper_config_.lidar_extrinsic_rotation_std_deg = std::max(0.0,
      declare_parameter<double>("rolling_elevation.sensor.lidar.extrinsic_rotation_std_deg",
                                mapper_config_.lidar_extrinsic_rotation_std_deg));
  mapper_config_.d435_extrinsic_translation_std_m = std::max(0.0,
      declare_parameter<double>("rolling_elevation.sensor.d435.extrinsic_translation_std_m",
                                mapper_config_.d435_extrinsic_translation_std_m));
  mapper_config_.d435_extrinsic_rotation_std_deg = std::max(0.0,
      declare_parameter<double>("rolling_elevation.sensor.d435.extrinsic_rotation_std_deg",
                                mapper_config_.d435_extrinsic_rotation_std_deg));
  mapper_config_.pose_min_position_std_m = std::max(0.001, declare_parameter<double>(
      "rolling_elevation.localization.min_position_std_m",
      mapper_config_.pose_min_position_std_m));
  mapper_config_.pose_min_orientation_std_deg = std::max(0.01, declare_parameter<double>(
      "rolling_elevation.localization.min_orientation_std_deg",
      mapper_config_.pose_min_orientation_std_deg));
  mapper_config_.pose_max_position_std_m = std::max(0.001, declare_parameter<double>(
      "rolling_elevation.localization.max_position_std_m",
      mapper_config_.pose_max_position_std_m));
  mapper_config_.pose_max_orientation_std_deg = std::max(0.01, declare_parameter<double>(
      "rolling_elevation.localization.max_orientation_std_deg",
      mapper_config_.pose_max_orientation_std_deg));
  mapper_config_.pose_splat_max_cells = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("rolling_elevation.localization.splat_max_cells",
                                      mapper_config_.pose_splat_max_cells),
      0, 4));
  mapper_config_.floor_radius_m = std::max(0.1, declare_parameter<double>(
      "height_origin.floor_radius", mapper_config_.floor_radius_m));
  mapper_config_.floor_plane_min_samples = static_cast<int>(std::max<std::int64_t>(
      3, declare_parameter<std::int64_t>("height_origin.plane.min_samples",
                                          mapper_config_.floor_plane_min_samples)));
  mapper_config_.floor_plane_max_slope_deg = std::clamp(declare_parameter<double>(
      "height_origin.plane.max_slope_deg", mapper_config_.floor_plane_max_slope_deg),
      0.0, 85.0);
  mapper_config_.floor_plane_huber_delta_m = std::max(0.001, declare_parameter<double>(
      "height_origin.plane.huber_delta_m", mapper_config_.floor_plane_huber_delta_m));
  mapper_.configure(mapper_config_);

  reference_height_ = declare_parameter<double>("height_map_distance.reference_height", reference_height_);
  distance_min_ = declare_parameter<double>("height_map_distance.min", distance_min_);
  distance_max_ = declare_parameter<double>("height_map_distance.max", distance_max_);
  unknown_distance_ = declare_parameter<double>("height_map_distance.unknown", unknown_distance_);
  height_map_transport_ = declare_parameter<std::string>(
      "height_map_output.transport", height_map_transport_);
  if (height_map_transport_ != "ros2" && height_map_transport_ != "cyclone_dds" &&
      height_map_transport_ != "both") {
    throw std::invalid_argument(
        "height_map_output.transport must be ros2, cyclone_dds, or both");
  }
  const auto dds_domain = declare_parameter<std::int64_t>(
      "height_map_output.cyclone_dds.domain_id", dds_height_map_domain_id_);
  dds_height_map_domain_id_ = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
      dds_domain, 0, std::numeric_limits<std::uint32_t>::max()));
  dds_height_map_topic_ = declare_parameter<std::string>(
      "height_map_output.cyclone_dds.topic", dds_height_map_topic_);
  dds_height_map_type_ = declare_parameter<std::string>(
      "height_map_output.cyclone_dds.type", dds_height_map_type_);
  const auto dds_history = declare_parameter<std::int64_t>(
      "height_map_output.cyclone_dds.history_depth", dds_height_map_history_depth_);
  dds_height_map_history_depth_ = static_cast<std::uint32_t>(
      std::clamp<std::int64_t>(dds_history, 1, 1000));
  mapping_only_ = declare_parameter<bool>("mapping_only", mapping_only_);
  mapping_pcd_file_ = declare_parameter<std::string>("mapping_pcd_file", mapping_pcd_file_);
  saved_map_file_ = declare_parameter<std::string>("saved_map_file", saved_map_file_);
  saved_map_frame_ = declare_parameter<std::string>("saved_map_frame", saved_map_frame_);

  raw_lidar_topic_ = declare_parameter<std::string>("raw_lidar_topic", raw_lidar_topic_);
  raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", raw_imu_topic_);
  raw_lidar_msg_type_ = declare_parameter<std::string>("raw_lidar_msg_type", raw_lidar_msg_type_);
  super_lio_odom_topic_ = declare_parameter<std::string>("super_lio_odom_topic", super_lio_odom_topic_);
  super_lio_registered_topic_ = declare_parameter<std::string>(
      "super_lio_registered_topic", super_lio_registered_topic_);
  odom_sync_tolerance_sec_ = std::max(0.0, declare_parameter<double>(
      "rolling_elevation.localization.odom_sync_tolerance_sec", odom_sync_tolerance_sec_));
  rolling_merge_enabled_ = declare_parameter<bool>("rolling_merge.enabled", rolling_merge_enabled_);
  rolling_merge_output_topic_ = declare_parameter<std::string>(
      "rolling_merge.output_topic", rolling_merge_output_topic_);
  super_lio_config_file_ = declare_parameter<std::string>(
      "super_lio_config_file", super_lio_config_file_);
  start_lidar_driver_ = declare_parameter<bool>("start_lidar_driver", start_lidar_driver_);
  start_super_lio_ = declare_parameter<bool>("start_super_lio", start_super_lio_);
  livox_publish_freq_ = std::clamp(declare_parameter<double>(
      "livox_publish_freq", livox_publish_freq_), 0.5, 100.0);
  child_use_sim_time_ = declare_parameter<bool>("child_use_sim_time", child_use_sim_time_);
  child_shutdown_grace_sec_ = std::max(0.0, declare_parameter<double>(
      "child_shutdown_grace_sec", child_shutdown_grace_sec_));
  lidar_driver_command_ = declare_parameter<std::vector<std::string>>(
      "lidar_driver_command", lidar_driver_command_);
  super_lio_command_ = declare_parameter<std::vector<std::string>>(
      "super_lio_command", super_lio_command_);

  live_map_topic_ = declare_parameter<std::string>("live_global_map.topic", live_map_topic_);
  map_publish_interval_sec_ = std::max(0.1, declare_parameter<double>(
      "live_global_map.publish_interval_sec", map_publish_interval_sec_));
  saved_map_topic_ = declare_parameter<std::string>("saved_map_topic", saved_map_topic_);
  height_map_topic_ = declare_parameter<std::string>("height_map_topic", height_map_topic_);
  height_map_msg_topic_ = declare_parameter<std::string>("height_map_msg_topic", height_map_msg_topic_);
  height_map_quality_topic_ = declare_parameter<std::string>(
      "height_map_quality_topic", height_map_quality_topic_);
  odom_output_topic_ = declare_parameter<std::string>("odom_output_topic", odom_output_topic_);
  path_output_topic_ = declare_parameter<std::string>("path_output_topic", path_output_topic_);
  heartbeat_topic_ = declare_parameter<std::string>("heartbeat_topic", heartbeat_topic_);
}

void AutonomyLightNode::loadSavedMap() {
  if (saved_map_file_.empty()) {
    return;
  }
  PclCloud map;
  if (pcl::io::loadPCDFile(saved_map_file_, map) < 0 || map.empty()) {
    throw std::runtime_error("cannot load saved_map_file: " + saved_map_file_);
  }
  mapper_.loadGlobalMap(map);
  saved_map_cloud_.reset(new PclCloud(map));
  global_frame_ = saved_map_frame_;
  RCLCPP_INFO(get_logger(), "Loaded saved map: %s (%zu points)",
              saved_map_file_.c_str(), map.size());
}

void AutonomyLightNode::createInterfaces() {
  const auto data_qos = rclcpp::SensorDataQoS();
  const auto output_qos = rclcpp::QoS(10).reliable();
  createDdsOutput();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      super_lio_odom_topic_, output_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(std::move(msg)); });
  const std::string elevation_cloud_topic =
      mapper_config_.source == "rolling" && rolling_merge_enabled_
          ? rolling_merge_output_topic_ : super_lio_registered_topic_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      elevation_cloud_topic, data_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onRegisteredCloud(std::move(msg)); });
  RCLCPP_INFO(get_logger(), "Elevation cloud input: %s", elevation_cloud_topic.c_str());
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      live_map_topic_, rclcpp::QoS(1).reliable().transient_local());
  saved_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      saved_map_topic_, rclcpp::QoS(1).reliable().transient_local());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic_, output_qos);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_output_topic_, output_qos);
  heartbeat_pub_ = create_publisher<std_msgs::msg::String>(heartbeat_topic_, output_qos);
  if (!mapping_only_ && publishesRosHeight()) {
    height_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(height_map_topic_, output_qos);
    height_msg_pub_ = create_publisher<msg::HeightMap>(height_map_msg_topic_, output_qos);
    quality_pub_ = create_publisher<msg::HeightMapQuality>(height_map_quality_topic_, output_qos);
  }
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period, [this]() { onTimer(); });
}

void AutonomyLightNode::createDdsOutput() {
  if (!publishesDdsHeight() || mapping_only_) {
    return;
  }
  dds_height_map_pub_ = std::make_unique<DdsHeightMapPublisher>(
      dds_height_map_domain_id_, dds_height_map_topic_, dds_height_map_type_,
      dds_height_map_history_depth_);
  if (!dds_height_map_pub_->isReady()) {
    throw std::runtime_error("Cyclone DDS height map writer failed: " +
                             dds_height_map_pub_->error());
  }
  RCLCPP_INFO(get_logger(),
              "Cyclone DDS height map enabled: domain=%u topic=%s type=%s "
              "history=%u",
              dds_height_map_domain_id_, dds_height_map_topic_.c_str(),
              dds_height_map_type_.c_str(), dds_height_map_history_depth_);
}

bool AutonomyLightNode::publishesRosHeight() const {
  return height_map_transport_ == "ros2" || height_map_transport_ == "both";
}

bool AutonomyLightNode::publishesDdsHeight() const {
  return height_map_transport_ == "cyclone_dds" || height_map_transport_ == "both";
}

void AutonomyLightNode::startProcesses() {
  if (start_lidar_driver_) {
    auto command = lidar_driver_command_;
    if (command.empty()) {
      command = {"ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py",
                 "publish_freq:=" + std::to_string(livox_publish_freq_)};
    }
    children_.start(get_logger(), "Livox driver", command);
  }
  if (start_super_lio_) {
    children_.start(get_logger(), "Super-LIO",
                    super_lio_command_.empty() ? superLioCommand() : super_lio_command_);
  }
}

std::vector<std::string> AutonomyLightNode::superLioCommand() const {
  if (raw_lidar_msg_type_ != "livox_custom") {
    throw std::invalid_argument("bundled Super-LIO requires Livox CustomMsg input");
  }
  const std::string config = super_lio_config_file_.empty()
      ? ament_index_cpp::get_package_share_directory("autonomy_light") +
            "/config/super_lio_mid360.yaml"
      : super_lio_config_file_;
  const bool relocation = !saved_map_file_.empty();
  const bool full_slam = mapping_only_ && !mapping_pcd_file_.empty();
  const tf2::Matrix3x3 lidar_in_imu(imu_from_lidar_rotation_);
  const std::vector<double> lidar_imu{
      imu_from_lidar_translation_.x(), imu_from_lidar_translation_.y(),
      imu_from_lidar_translation_.z(),
      lidar_in_imu[0][0], lidar_in_imu[0][1], lidar_in_imu[0][2],
      lidar_in_imu[1][0], lidar_in_imu[1][1], lidar_in_imu[1][2],
      lidar_in_imu[2][0], lidar_in_imu[2][1], lidar_in_imu[2][2]};
  std::vector<std::string> command{
      "ros2", "run", "super_lio", relocation ? "relocation_node" : "super_lio_node",
      "--ros-args", "--params-file", config,
      "-p", "lio.ros.lidar_topic:=" + raw_lidar_topic_,
      "-p", "lio.ros.imu_topic:=" + raw_imu_topic_,
      "-p", "lio.ros.global_frame:=" + global_frame_,
      "-p", "lio.extrinsic.lidar_imu:=" + rosArray(lidar_imu),
      "-p", "lio.sensor.lidar_type:=1",
      "-p", "lio.output.map:=true",
      "-p", "lio.output.dense:=true",
      "-p", "lio.output.pub_step:=1",
      "-p", "lio.map.save_map:=" + std::string(full_slam ? "true" : "false")};
  if (full_slam) {
    const auto map_path = std::filesystem::absolute(mapping_pcd_file_);
    command.insert(command.end(), {"-p", "lio.slam.enable:=true",
                                   "-p", "lio.map.save_map_dir:=" + map_path.parent_path().string(),
                                   "-p", "lio.map.map_name:=" + map_path.filename().string()});
  }
  if (relocation) {
    const auto map_path = std::filesystem::absolute(saved_map_file_);
    command.insert(command.end(), {"-p", "lio.map.save_map_dir:=" + map_path.parent_path().string(),
                                   "-p", "lio.map.map_name:=" + map_path.filename().string()});
  }
  if (child_use_sim_time_) {
    command.insert(command.end(), {"-p", "use_sim_time:=true"});
  }
  return command;
}

void AutonomyLightNode::publishStaticTransform() {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  if (imu_frame_ != target_frame_) {
    geometry_msgs::msg::TransformStamped imu_to_target;
    imu_to_target.header.stamp = now();
    imu_to_target.header.frame_id = imu_frame_;
    imu_to_target.child_frame_id = target_frame_;
    const tf2::Quaternion rotation = target_to_imu_rotation_.inverse();
    const tf2::Vector3 translation = tf2::quatRotate(
        rotation, -target_to_imu_translation_);
    imu_to_target.transform.translation.x = translation.x();
    imu_to_target.transform.translation.y = translation.y();
    imu_to_target.transform.translation.z = translation.z();
    imu_to_target.transform.rotation = tf2::toMsg(rotation);
    transforms.push_back(std::move(imu_to_target));
  }
  if (target_frame_ != lidar_frame_) {
    geometry_msgs::msg::TransformStamped target_to_lidar;
    target_to_lidar.header.stamp = now();
    target_to_lidar.header.frame_id = target_frame_;
    target_to_lidar.child_frame_id = lidar_frame_;
    target_to_lidar.transform.translation.x = target_to_lidar_translation_.x();
    target_to_lidar.transform.translation.y = target_to_lidar_translation_.y();
    target_to_lidar.transform.translation.z = target_to_lidar_translation_.z();
    target_to_lidar.transform.rotation = tf2::toMsg(target_to_lidar_rotation_);
    transforms.push_back(std::move(target_to_lidar));
  }
  if (!transforms.empty()) {
    static_tf_broadcaster_->sendTransform(transforms);
  }
}

void AutonomyLightNode::saveMap() {
  if (mapping_only_ && start_super_lio_) {
    return;
  }
  if (mapping_pcd_file_.empty()) {
    return;
  }
  const auto map = mapper_.globalMap();
  if (!map || map->empty()) {
    RCLCPP_ERROR(get_logger(), "No registered points: map was not saved to %s",
                 mapping_pcd_file_.c_str());
    return;
  }
  if (pcl::io::savePCDFileBinary(mapping_pcd_file_, *map) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to save map PCD: %s", mapping_pcd_file_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Saved Super-LIO map: %s (%zu points)",
              mapping_pcd_file_.c_str(), map->size());
}

} // namespace autonomy_light
