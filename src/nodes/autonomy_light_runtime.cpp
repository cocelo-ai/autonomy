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
                    super_lio_command_.empty() ? superLioCommand()
                                               : super_lio_command_);
  }
}

std::vector<std::string> AutonomyLightNode::superLioCommand() const {
  if (raw_lidar_msg_type_ != "livox_custom") {
    throw std::invalid_argument(
        "bundled Super-LIO requires Livox CustomMsg input");
  }
  const std::string config =
      super_lio_config_file_.empty()
          ? ament_index_cpp::get_package_share_directory("autonomy_light") +
                "/config/super_lio_mid360.yaml"
          : super_lio_config_file_;
  const bool relocation = !saved_map_file_.empty();
  const bool full_slam = mapping_only_ && !mapping_pcd_file_.empty();
  const tf2::Matrix3x3 lidar_in_imu(imu_from_lidar_rotation_);
  const std::vector<double> lidar_imu{
      imu_from_lidar_translation_.x(), imu_from_lidar_translation_.y(),
      imu_from_lidar_translation_.z(), lidar_in_imu[0][0], lidar_in_imu[0][1],
      lidar_in_imu[0][2], lidar_in_imu[1][0], lidar_in_imu[1][1],
      lidar_in_imu[1][2], lidar_in_imu[2][0], lidar_in_imu[2][1],
      lidar_in_imu[2][2]};
  std::vector<std::string> command{
      "ros2", "run", "super_lio", relocation ? "relocation_node" : "super_lio_node",
      "--ros-args", "--params-file", config, "-p",
      "lio.ros.lidar_topic:=" + raw_lidar_topic_, "-p",
      "lio.ros.imu_topic:=" + raw_imu_topic_, "-p",
      "lio.ros.global_frame:=" + global_frame_, "-p",
      "lio.ros.odom_frame:=" + local_odom_frame_, "-p",
      "lio.extrinsic.lidar_imu:=" + rosArray(lidar_imu), "-p",
      "lio.sensor.lidar_type:=1", "-p", "lio.output.map:=true", "-p",
      "lio.output.dense:=true", "-p", "lio.output.pub_step:=1", "-p",
      "lio.map.save_map:=" + std::string(full_slam ? "true" : "false")};
  if (full_slam) {
    const auto map_path = std::filesystem::absolute(mapping_pcd_file_);
    command.insert(command.end(),
                   {"-p", "lio.slam.enable:=true", "-p",
                    "lio.map.save_map_dir:=" + map_path.parent_path().string(),
                    "-p", "lio.map.map_name:=" + map_path.filename().string()});
  }
  if (relocation) {
    const auto map_path = std::filesystem::absolute(saved_map_file_);
    command.insert(command.end(),
                   {"-p", "lio.map.save_map_dir:=" + map_path.parent_path().string(),
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
    const tf2::Vector3 translation =
        tf2::quatRotate(rotation, -target_to_imu_translation_);
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
    RCLCPP_ERROR(get_logger(), "Failed to save map PCD: %s",
                 mapping_pcd_file_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Saved Super-LIO map: %s (%zu points)",
              mapping_pcd_file_.c_str(), map->size());
}

} // namespace autonomy_light
