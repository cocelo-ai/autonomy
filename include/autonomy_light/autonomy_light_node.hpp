#pragma once

#include "autonomy_light/child_processes.hpp"
#include "autonomy_light/dds_height_map_publisher.hpp"
#include "autonomy_light/elevation_mapper.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autonomy_light/msg/height_map.hpp>
#include <autonomy_light/msg/height_map_quality.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
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

#include <deque>

namespace autonomy_light {

class AutonomyLightNode final : public rclcpp::Node {
public:
  AutonomyLightNode();
  ~AutonomyLightNode() override;
  void requestFastShutdown();

private:
  void loadParameters();
  void loadSavedMap();
  void createInterfaces();
  void createDdsOutput();
  void startProcesses();
  [[nodiscard]] std::vector<std::string> superLioCommand() const;
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message);
  void onRegisteredCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message);
  void onTimer();
  void publishMap();
  void publishHeight(const HeightGrid &grid);
  void publishPose(const nav_msgs::msg::Odometry &odom, double floor_z);
  void publishStaticTransform();
  void saveMap();
  [[nodiscard]] bool publishesRosHeight() const;
  [[nodiscard]] bool publishesDdsHeight() const;
  [[nodiscard]] nav_msgs::msg::Odometry baseOdom(
      const nav_msgs::msg::Odometry &imu_odom) const;
  [[nodiscard]] const nav_msgs::msg::Odometry *odomAt(const rclcpp::Time &stamp) const;
  [[nodiscard]] PointObservations observationsFrom(
      const sensor_msgs::msg::PointCloud2 &cloud) const;
  [[nodiscard]] Pose2_5D poseOf(const nav_msgs::msg::Odometry &odom) const;
  [[nodiscard]] tf2::Quaternion yawOnly(
      const geometry_msgs::msg::Quaternion &orientation) const;

  std::string target_frame_{"base_link"};
  std::string height_map_frame_{"base_link_gravity"};
  std::string imu_frame_{"imu"};
  std::string lidar_frame_{"lidar_link"};
  std::string global_frame_{"map"};
  std::string local_odom_frame_{"odom"};
  std::vector<double> target_to_lidar_xyz_{0.0, 0.0, 0.3};
  std::vector<double> target_to_lidar_rpy_{0.0, 0.0, 0.0};
  std::vector<double> imu_from_lidar_xyz_{0.0, 0.0, 0.0};
  std::vector<double> imu_from_lidar_rpy_{0.0, 0.0, 0.0};
  tf2::Vector3 target_to_lidar_translation_{0.0, 0.0, 0.3};
  tf2::Quaternion target_to_lidar_rotation_{tf2::Quaternion::getIdentity()};
  tf2::Vector3 imu_from_lidar_translation_;
  tf2::Quaternion imu_from_lidar_rotation_{tf2::Quaternion::getIdentity()};
  tf2::Vector3 target_to_imu_translation_;
  tf2::Quaternion target_to_imu_rotation_{tf2::Quaternion::getIdentity()};

  GridSpec grid_spec_;
  ElevationMapperConfig mapper_config_;
  ElevationMapper mapper_;
  double publish_rate_hz_{50.0};
  double map_publish_interval_sec_{1.0};
  double reference_height_{0.48};
  double distance_min_{0.0};
  double distance_max_{0.75};
  double unknown_distance_{0.48};
  std::string height_map_transport_{"ros2"};
  std::uint32_t dds_height_map_domain_id_{1U};
  std::string dds_height_map_topic_{"height_map"};
  std::string dds_height_map_type_{"core_dds::HeightMap"};
  std::uint32_t dds_height_map_history_depth_{1U};
  bool mapping_only_{false};
  std::string mapping_pcd_file_;
  std::string saved_map_file_;
  std::string saved_map_frame_{"map"};
  PclCloud::Ptr saved_map_cloud_;

  std::string raw_lidar_topic_{"/livox/lidar"};
  std::string raw_imu_topic_{"/livox/imu"};
  std::string raw_lidar_msg_type_{"livox_custom"};
  std::string super_lio_odom_topic_{"/lio/odom"};
  std::string super_lio_registered_topic_{"/lio/cloud_world"};
  double odom_sync_tolerance_sec_{0.03};
  bool rolling_merge_enabled_{false};
  std::string rolling_merge_output_topic_{"/autonomy_light/rolling_cloud"};
  std::string super_lio_config_file_;
  bool start_lidar_driver_{true};
  bool start_super_lio_{true};
  double livox_publish_freq_{50.0};
  bool child_use_sim_time_{false};
  double child_shutdown_grace_sec_{0.8};
  std::vector<std::string> lidar_driver_command_;
  std::vector<std::string> super_lio_command_;

  std::string live_map_topic_{"/autonomy_light/live_map"};
  std::string saved_map_topic_{"/autonomy_light/saved_map"};
  std::string height_map_topic_{"/autonomy_light/height_map"};
  std::string height_map_msg_topic_{"/autonomy_light/height_map_data"};
  std::string height_map_quality_topic_{"/autonomy_light/height_map_quality"};
  std::string odom_output_topic_{"/autonomy_light/odom"};
  std::string path_output_topic_{"/autonomy_light/path"};
  std::string heartbeat_topic_{"/autonomy_light/heartbeat"};

  bool has_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  nav_msgs::msg::Path path_;
  std::uint64_t odom_count_{0U};
  std::uint64_t cloud_count_{0U};
  std::chrono::steady_clock::time_point last_map_publish_{};
  bool shutdown_requested_{false};

  ChildProcesses children_;
  std::unique_ptr<DdsHeightMapPublisher> dds_height_map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr saved_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr height_pub_;
  rclcpp::Publisher<msg::HeightMap>::SharedPtr height_msg_pub_;
  rclcpp::Publisher<msg::HeightMapQuality>::SharedPtr quality_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

} // namespace autonomy_light
