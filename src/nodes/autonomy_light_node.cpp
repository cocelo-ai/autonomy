#include "autonomy_light/autonomy_light_node.hpp"

namespace autonomy_light {

AutonomyLightNode::AutonomyLightNode() : Node("autonomy_light") {
  loadParameters();
  loadSavedMap();
  createInterfaces();
  publishStaticTransform();
  startProcesses();
  RCLCPP_INFO(get_logger(),
              "Elevation runtime ready: source=%s transport=%s grid=%ux%u @ "
              "%.3fm frame=%s",
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
  height_map_frame_ =
      declare_parameter<std::string>("height_map_frame", height_map_frame_);
  imu_frame_ = declare_parameter<std::string>("imu_frame", imu_frame_);
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", lidar_frame_);
  global_frame_ = declare_parameter<std::string>("odom_frame", global_frame_);
  local_odom_frame_ =
      declare_parameter<std::string>("local_odom_frame", local_odom_frame_);
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
    throw std::invalid_argument(
        "all *_xyz and *_rpy extrinsics must contain 3 values");
  }
  target_to_lidar_translation_ =
      tf2::Vector3(target_to_lidar_xyz_[0], target_to_lidar_xyz_[1],
                   target_to_lidar_xyz_[2]);
  target_to_lidar_rotation_.setRPY(target_to_lidar_rpy_[0],
                                   target_to_lidar_rpy_[1],
                                   target_to_lidar_rpy_[2]);
  target_to_lidar_rotation_.normalize();
  imu_from_lidar_translation_ = tf2::Vector3(
      imu_from_lidar_xyz_[0], imu_from_lidar_xyz_[1], imu_from_lidar_xyz_[2]);
  imu_from_lidar_rotation_.setRPY(
      imu_from_lidar_rpy_[0], imu_from_lidar_rpy_[1], imu_from_lidar_rpy_[2]);
  imu_from_lidar_rotation_.normalize();
  const tf2::Quaternion lidar_from_imu = imu_from_lidar_rotation_.inverse();
  target_to_imu_rotation_ = target_to_lidar_rotation_ * lidar_from_imu;
  target_to_imu_rotation_.normalize();
  target_to_imu_translation_ =
      target_to_lidar_translation_ +
      tf2::quatRotate(
          target_to_lidar_rotation_,
          tf2::quatRotate(lidar_from_imu, -imu_from_lidar_translation_));

  grid_spec_.resolution =
      std::max(0.01, declare_parameter<double>("elevation_resolution",
                                               grid_spec_.resolution));
  grid_spec_.x_length = std::max(
      grid_spec_.resolution,
      declare_parameter<double>("elevation_x_length", grid_spec_.x_length));
  grid_spec_.y_length = std::max(
      grid_spec_.resolution,
      declare_parameter<double>("elevation_y_length", grid_spec_.y_length));
  grid_spec_.min_z =
      declare_parameter<double>("elevation_min_z", grid_spec_.min_z);
  grid_spec_.max_z =
      declare_parameter<double>("elevation_max_z", grid_spec_.max_z);
  publish_rate_hz_ = std::max(
      1.0, declare_parameter<double>("publish_rate_hz", publish_rate_hz_));
  mapper_config_.grid = grid_spec_;
  mapper_config_.source =
      declare_parameter<std::string>("height_map.source", "rolling");
  mapper_config_.global_voxel_size = std::max(
      0.005, declare_parameter<double>("live_global_map.voxel_leaf_size",
                                       mapper_config_.global_voxel_size));
  mapper_config_.global_max_voxels =
      static_cast<std::size_t>(std::max<std::int64_t>(
          1000, declare_parameter<std::int64_t>("live_global_map.max_points",
                                                2'000'000)));
  mapper_config_.min_samples_per_cell = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("algorithm.min_z.min_points_per_cell",
                                         mapper_config_.min_samples_per_cell)));
  mapper_config_.ground_percentile = declare_parameter<double>(
      "algorithm.frame_aggregation.cell_height_percentile",
      mapper_config_.ground_percentile);
  mapper_config_.obstacle_min_height = std::max(
      0.0, declare_parameter<double>("algorithm.min_z.obstacle_min_height",
                                     mapper_config_.obstacle_min_height));
  mapper_config_.rolling_max_age_sec = std::max(
      0.01, declare_parameter<double>("rolling_elevation.max_age_sec",
                                      mapper_config_.rolling_max_age_sec));
  mapper_config_.rolling_upper_max_age_sec = std::max(
      0.01,
      declare_parameter<double>("rolling_elevation.upper_max_age_sec",
                                mapper_config_.rolling_upper_max_age_sec));
  mapper_config_.rolling_max_radius_m = std::max(
      0.1, declare_parameter<double>("rolling_elevation.max_radius_m",
                                     mapper_config_.rolling_max_radius_m));
  mapper_config_.rolling_base_variance = std::max(
      1.0e-6, declare_parameter<double>("rolling_elevation.base_variance",
                                        mapper_config_.rolling_base_variance));
  mapper_config_.rolling_range_variance_factor = std::max(
      0.0,
      declare_parameter<double>("rolling_elevation.range_variance_factor",
                                mapper_config_.rolling_range_variance_factor));
  mapper_config_.rolling_process_variance_per_sec =
      std::max(0.0, declare_parameter<double>(
                        "rolling_elevation.process_variance_per_sec",
                        mapper_config_.rolling_process_variance_per_sec));
  mapper_config_.rolling_max_variance = std::max(
      1.0e-6, declare_parameter<double>("rolling_elevation.max_variance",
                                        mapper_config_.rolling_max_variance));
  mapper_config_.rolling_outlier_variance = std::max(
      0.0, declare_parameter<double>("rolling_elevation.outlier_variance",
                                     mapper_config_.rolling_outlier_variance));
  mapper_config_.rolling_mahalanobis_threshold = std::max(
      0.1,
      declare_parameter<double>("rolling_elevation.mahalanobis_threshold",
                                mapper_config_.rolling_mahalanobis_threshold));
  mapper_config_.rolling_initial_prior_enabled =
      declare_parameter<bool>("rolling_elevation.initial_prior.enabled",
                              mapper_config_.rolling_initial_prior_enabled);
  mapper_config_.rolling_initial_prior_radius_m =
      std::max(0.0, declare_parameter<double>(
                        "rolling_elevation.initial_prior.footprint_radius_m",
                        mapper_config_.rolling_initial_prior_radius_m));
  mapper_config_.rolling_initial_prior_ground_distance_m = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.initial_prior.ground_distance_m",
               mapper_config_.rolling_initial_prior_ground_distance_m));
  mapper_config_.rolling_initial_prior_variance = std::max(
      1.0e-6,
      declare_parameter<double>("rolling_elevation.initial_prior.variance",
                                mapper_config_.rolling_initial_prior_variance));
  mapper_config_.visibility_cleanup_enabled =
      declare_parameter<bool>("rolling_elevation.visibility.enabled",
                              mapper_config_.visibility_cleanup_enabled);
  mapper_config_.visibility_max_ray_length_m = std::max(
      0.0,
      declare_parameter<double>("rolling_elevation.visibility.max_ray_length_m",
                                mapper_config_.visibility_max_ray_length_m));
  mapper_config_.visibility_min_ray_length_m = std::max(
      0.0,
      declare_parameter<double>("rolling_elevation.visibility.min_ray_length_m",
                                mapper_config_.visibility_min_ray_length_m));
  mapper_config_.visibility_min_observation_age_sec =
      std::max(0.0, declare_parameter<double>(
                        "rolling_elevation.visibility.min_observation_age_sec",
                        mapper_config_.visibility_min_observation_age_sec));
  mapper_config_.visibility_normal_alignment_min =
      std::clamp(declare_parameter<double>(
                     "rolling_elevation.visibility.normal_alignment_min",
                     mapper_config_.visibility_normal_alignment_min),
                 0.0, 1.0);
  mapper_config_.visibility_height_margin_m = std::max(
      0.0,
      declare_parameter<double>("rolling_elevation.visibility.height_margin_m",
                                mapper_config_.visibility_height_margin_m));
  mapper_config_.visibility_sigma_scale = std::max(
      0.0, declare_parameter<double>("rolling_elevation.visibility.sigma_scale",
                                     mapper_config_.visibility_sigma_scale));
  mapper_config_.overhang_filter_enabled =
      declare_parameter<bool>("rolling_elevation.overhang_filter.enabled",
                              mapper_config_.overhang_filter_enabled);
  mapper_config_.overhang_near_max_above_sensor_m = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.overhang_filter.near_max_above_sensor_m",
               mapper_config_.overhang_near_max_above_sensor_m));
  mapper_config_.overhang_ramp_start_m =
      std::max(0.0, declare_parameter<double>(
                        "rolling_elevation.overhang_filter.ramp_start_m",
                        mapper_config_.overhang_ramp_start_m));
  mapper_config_.overhang_ramp_slope = std::max(
      0.0,
      declare_parameter<double>("rolling_elevation.overhang_filter.ramp_slope",
                                mapper_config_.overhang_ramp_slope));
  mapper_config_.overhang_absolute_max_above_sensor_m = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.overhang_filter.absolute_max_above_sensor_m",
               mapper_config_.overhang_absolute_max_above_sensor_m));
  mapper_config_.d435_base_variance = std::max(
      1.0e-6,
      declare_parameter<double>("rolling_elevation.sensor.d435.base_variance",
                                mapper_config_.d435_base_variance));
  mapper_config_.d435_range_variance_factor =
      std::max(0.0, declare_parameter<double>(
                        "rolling_elevation.sensor.d435.range_variance_factor",
                        mapper_config_.d435_range_variance_factor));
  mapper_config_.lidar_extrinsic_translation_std_m = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.sensor.lidar.extrinsic_translation_std_m",
               mapper_config_.lidar_extrinsic_translation_std_m));
  mapper_config_.lidar_extrinsic_rotation_std_deg = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.sensor.lidar.extrinsic_rotation_std_deg",
               mapper_config_.lidar_extrinsic_rotation_std_deg));
  mapper_config_.d435_extrinsic_translation_std_m = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.sensor.d435.extrinsic_translation_std_m",
               mapper_config_.d435_extrinsic_translation_std_m));
  mapper_config_.d435_extrinsic_rotation_std_deg = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.sensor.d435.extrinsic_rotation_std_deg",
               mapper_config_.d435_extrinsic_rotation_std_deg));
  mapper_config_.pose_min_position_std_m =
      std::max(0.001, declare_parameter<double>(
                          "rolling_elevation.localization.min_position_std_m",
                          mapper_config_.pose_min_position_std_m));
  mapper_config_.pose_min_orientation_std_deg = std::max(
      0.01, declare_parameter<double>(
                "rolling_elevation.localization.min_orientation_std_deg",
                mapper_config_.pose_min_orientation_std_deg));
  mapper_config_.pose_max_position_std_m =
      std::max(0.001, declare_parameter<double>(
                          "rolling_elevation.localization.max_position_std_m",
                          mapper_config_.pose_max_position_std_m));
  mapper_config_.pose_max_orientation_std_deg = std::max(
      0.01, declare_parameter<double>(
                "rolling_elevation.localization.max_orientation_std_deg",
                mapper_config_.pose_max_orientation_std_deg));
  mapper_config_.pose_splat_max_cells =
      static_cast<int>(std::clamp<std::int64_t>(
          declare_parameter<std::int64_t>(
              "rolling_elevation.localization.splat_max_cells",
              mapper_config_.pose_splat_max_cells),
          0, 4));
  mapper_.configure(mapper_config_);

  distance_min_ =
      declare_parameter<double>("height_map_distance.min", distance_min_);
  distance_max_ =
      declare_parameter<double>("height_map_distance.max", distance_max_);
  unknown_distance_ = declare_parameter<double>("height_map_distance.unknown",
                                                unknown_distance_);
  height_map_transport_ = declare_parameter<std::string>(
      "height_map_output.transport", height_map_transport_);
  if (height_map_transport_ != "ros2" &&
      height_map_transport_ != "cyclone_dds" &&
      height_map_transport_ != "both") {
    throw std::invalid_argument(
        "height_map_output.transport must be ros2, cyclone_dds, or both");
  }
  const auto dds_domain = declare_parameter<std::int64_t>(
      "height_map_output.cyclone_dds.domain_id", dds_height_map_domain_id_);
  dds_height_map_domain_id_ =
      static_cast<std::uint32_t>(std::clamp<std::int64_t>(
          dds_domain, 0, std::numeric_limits<std::uint32_t>::max()));
  dds_height_map_topic_ = declare_parameter<std::string>(
      "height_map_output.cyclone_dds.topic", dds_height_map_topic_);
  dds_height_map_type_ = declare_parameter<std::string>(
      "height_map_output.cyclone_dds.type", dds_height_map_type_);
  const auto dds_history = declare_parameter<std::int64_t>(
      "height_map_output.cyclone_dds.history_depth",
      dds_height_map_history_depth_);
  dds_height_map_history_depth_ = static_cast<std::uint32_t>(
      std::clamp<std::int64_t>(dds_history, 1, 1000));
  mapping_only_ = declare_parameter<bool>("mapping_only", mapping_only_);
  mapping_pcd_file_ =
      declare_parameter<std::string>("mapping_pcd_file", mapping_pcd_file_);
  saved_map_file_ =
      declare_parameter<std::string>("saved_map_file", saved_map_file_);
  saved_map_frame_ =
      declare_parameter<std::string>("saved_map_frame", saved_map_frame_);

  raw_lidar_topic_ =
      declare_parameter<std::string>("raw_lidar_topic", raw_lidar_topic_);
  raw_imu_topic_ =
      declare_parameter<std::string>("raw_imu_topic", raw_imu_topic_);
  raw_lidar_msg_type_ =
      declare_parameter<std::string>("raw_lidar_msg_type", raw_lidar_msg_type_);
  super_lio_odom_topic_ = declare_parameter<std::string>("super_lio_odom_topic",
                                                         super_lio_odom_topic_);
  super_lio_registered_topic_ = declare_parameter<std::string>(
      "super_lio_registered_topic", super_lio_registered_topic_);
  odom_sync_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>(
               "rolling_elevation.localization.odom_sync_tolerance_sec",
               odom_sync_tolerance_sec_));
  pending_cloud_queue_size_ = static_cast<std::size_t>(
      std::clamp<std::int64_t>(
          declare_parameter<std::int64_t>(
              "rolling_elevation.localization.pending_cloud_queue_size",
              static_cast<std::int64_t>(pending_cloud_queue_size_)),
          1, 256));
  rolling_merge_enabled_ =
      declare_parameter<bool>("rolling_merge.enabled", rolling_merge_enabled_);
  rolling_merge_output_topic_ = declare_parameter<std::string>(
      "rolling_merge.output_topic", rolling_merge_output_topic_);
  super_lio_config_file_ = declare_parameter<std::string>(
      "super_lio_config_file", super_lio_config_file_);
  start_lidar_driver_ =
      declare_parameter<bool>("start_lidar_driver", start_lidar_driver_);
  start_super_lio_ =
      declare_parameter<bool>("start_super_lio", start_super_lio_);
  livox_publish_freq_ = std::clamp(
      declare_parameter<double>("livox_publish_freq", livox_publish_freq_), 0.5,
      100.0);
  child_use_sim_time_ =
      declare_parameter<bool>("child_use_sim_time", child_use_sim_time_);
  child_shutdown_grace_sec_ =
      std::max(0.0, declare_parameter<double>("child_shutdown_grace_sec",
                                              child_shutdown_grace_sec_));
  lidar_driver_command_ = declare_parameter<std::vector<std::string>>(
      "lidar_driver_command", lidar_driver_command_);
  super_lio_command_ = declare_parameter<std::vector<std::string>>(
      "super_lio_command", super_lio_command_);

  live_map_topic_ =
      declare_parameter<std::string>("live_global_map.topic", live_map_topic_);
  map_publish_interval_sec_ = std::max(
      0.1, declare_parameter<double>("live_global_map.publish_interval_sec",
                                     map_publish_interval_sec_));
  saved_map_topic_ =
      declare_parameter<std::string>("saved_map_topic", saved_map_topic_);
  height_map_topic_ =
      declare_parameter<std::string>("height_map_topic", height_map_topic_);
  height_map_msg_topic_ = declare_parameter<std::string>("height_map_msg_topic",
                                                         height_map_msg_topic_);
  height_map_quality_topic_ = declare_parameter<std::string>(
      "height_map_quality_topic", height_map_quality_topic_);
  odom_output_topic_ =
      declare_parameter<std::string>("odom_output_topic", odom_output_topic_);
  path_output_topic_ =
      declare_parameter<std::string>("path_output_topic", path_output_topic_);
  heartbeat_topic_ =
      declare_parameter<std::string>("heartbeat_topic", heartbeat_topic_);
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
  odom_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cloud_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  output_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions odom_options;
  odom_options.callback_group = odom_callback_group_;
  rclcpp::SubscriptionOptions cloud_options;
  cloud_options.callback_group = cloud_callback_group_;
  createDdsOutput();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      super_lio_odom_topic_, output_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        onOdom(std::move(msg));
      }, odom_options);
  const std::string elevation_cloud_topic =
      mapper_config_.source == "rolling" && rolling_merge_enabled_
          ? rolling_merge_output_topic_
          : super_lio_registered_topic_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      elevation_cloud_topic, data_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        onRegisteredCloud(std::move(msg));
      }, cloud_options);
  RCLCPP_INFO(get_logger(), "Elevation cloud input: %s",
              elevation_cloud_topic.c_str());
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      live_map_topic_, rclcpp::QoS(1).reliable().transient_local());
  saved_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      saved_map_topic_, rclcpp::QoS(1).reliable().transient_local());
  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(odom_output_topic_, output_qos);
  path_pub_ =
      create_publisher<nav_msgs::msg::Path>(path_output_topic_, output_qos);
  heartbeat_pub_ =
      create_publisher<std_msgs::msg::String>(heartbeat_topic_, output_qos);
  if (!mapping_only_ && publishesRosHeight()) {
    height_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        height_map_topic_, output_qos);
    height_msg_pub_ =
        create_publisher<msg::HeightMap>(height_map_msg_topic_, output_qos);
    quality_pub_ = create_publisher<msg::HeightMapQuality>(
        height_map_quality_topic_, output_qos);
  }
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period, [this]() { onTimer(); },
                             output_callback_group_);
  const auto map_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(map_publish_interval_sec_));
  map_timer_ = create_wall_timer(map_period, [this]() { publishMap(); },
                                 map_callback_group_);
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
  return height_map_transport_ == "cyclone_dds" ||
         height_map_transport_ == "both";
}

} // namespace autonomy_light
