#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace BASIC;

namespace LI2Sup{

inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}

void ROSWrapper::pub_odom(const NavState& state){
  const bool publish_map_to_odom = has_map_to_odom_ && g_global_frame != g_odom_frame;
  const SE3 odom_to_imu = state.GetSE3();
  const SE3 global_to_imu = publish_map_to_odom ? map_to_odom_ * odom_to_imu
                                                 : odom_to_imu;
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = g_global_frame;
  odom.child_frame_id = "imu";

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = global_to_imu.t_[0];
  odom.pose.pose.position.y = global_to_imu.t_[1];
  odom.pose.pose.position.z = global_to_imu.t_[2];

  V4 temp_q = global_to_imu.so3().coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  // ESKF keeps rotation error in the IMU body frame and position error in odom.
  // Export a ROS covariance in the selected global frame: x, y, z, roll, pitch, yaw.
  if (eskf_) {
    const auto eskf_covariance = eskf_->GetCov();
    if (eskf_covariance.allFinite()) {
      const M3 correction_rotation = global_to_imu.R_ * odom_to_imu.R_.transpose();
      const M3 global_rotation = global_to_imu.R_;
      Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
      covariance.block<3, 3>(0, 0) = (
          correction_rotation * eskf_covariance.template block<3, 3>(3, 3) *
          correction_rotation.transpose()).template cast<double>();
      covariance.block<3, 3>(3, 3) = (
          global_rotation * eskf_covariance.template block<3, 3>(0, 0) *
          global_rotation.transpose()).template cast<double>();
      covariance.block<3, 3>(0, 3) = (
          correction_rotation * eskf_covariance.template block<3, 3>(3, 0) *
          global_rotation.transpose()).template cast<double>();
      covariance.block<3, 3>(3, 0) = covariance.block<3, 3>(0, 3).transpose();
      covariance = 0.5 * (covariance + covariance.transpose());
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          odom.pose.covariance[6 * row + col] = covariance(row, col);
        }
      }
    }
  }

  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 robo_position = global_to_imu.R_ * (-g_odom_robo.R_ * g_odom_robo.t_) +
                     global_to_imu.t_;

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    M3 robo_rotation = global_to_imu.R_ * g_odom_robo.R_;
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  const Quat tf_quat(odom_to_imu.R_);
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = odom.header.stamp;
  tf_msg.header.frame_id = publish_map_to_odom ? g_odom_frame : g_global_frame;
  tf_msg.child_frame_id = "imu";

  tf_msg.transform.translation.x = odom_to_imu.t_[0];
  tf_msg.transform.translation.y = odom_to_imu.t_[1];
  tf_msg.transform.translation.z = odom_to_imu.t_[2];

  tf_msg.transform.rotation.x = tf_quat.x();
  tf_msg.transform.rotation.y = tf_quat.y();
  tf_msg.transform.rotation.z = tf_quat.z();
  tf_msg.transform.rotation.w = tf_quat.w();

  tf_broadcaster_->sendTransform(tf_msg);

  if (publish_map_to_odom) {
    publishMapToOdom(rclcpp::Time(odom.header.stamp));
  }

  // tf_msg.child_frame_id = "god";
  // tf_msg.transform.rotation.x = 0.0;
  // tf_msg.transform.rotation.y = 0.0;
  // tf_msg.transform.rotation.z = 0.0;
  // tf_msg.transform.rotation.w = 1.0;
  // tf_broadcaster_->sendTransform(tf_msg);

}

void ROSWrapper::publishMapToOdom(const rclcpp::Time& stamp) {
  if (!tf_broadcaster_ || !has_map_to_odom_ || g_global_frame == g_odom_frame) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = g_global_frame;
  transform.child_frame_id = g_odom_frame;
  transform.transform.translation.x = map_to_odom_.t_[0];
  transform.transform.translation.y = map_to_odom_.t_[1];
  transform.transform.translation.z = map_to_odom_.t_[2];
  Quat correction(map_to_odom_.R_);
  correction.normalize();
  transform.transform.rotation.x = correction.x();
  transform.transform.rotation.y = correction.y();
  transform.transform.rotation.z = correction.z();
  transform.transform.rotation.w = correction.w();
  tf_broadcaster_->sendTransform(transform);
}

void ROSWrapper::setMapToOdom(const SE3& map_to_odom) {
  map_to_odom_ = map_to_odom;
  has_map_to_odom_ = true;
  LOG(INFO) << GREEN << " ---> map -> odom correction is ready." << RESET;
}

SE3 ROSWrapper::globalPose(const SE3& odom_pose) const {
  return has_map_to_odom_ && g_global_frame != g_odom_frame
      ? map_to_odom_ * odom_pose
      : odom_pose;
}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_global_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_global_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static auto pub_cloud_body_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/body/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp); 
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_cloud_body_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static auto pub_cloud_world_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/world/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  pub_cloud_world_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = g_global_frame;

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/global_map", 10);

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          V3 init_translation;
          init_translation << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              0.2;

          double x = msg->pose.pose.orientation.x;
          double y = msg->pose.pose.orientation.y;
          double z = msg->pose.pose.orientation.z;
          double w = msg->pose.pose.orientation.w;

          Quat init_rotation(w, x, y, z);

          init_pose = BASIC::SE3(SO3(init_rotation.toRotationMatrix()), init_translation);

          flg_get_init_guess = true;

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess: "
                  << init_translation.transpose()
                  << " yaw: "
                  << init_rotation.toRotationMatrix()
                          .eulerAngles(0, 1, 2)
                          .transpose()
                  << RESET;
        });

  if (flg_finish_init) {
    init_pose_sub.reset();
  }
}


} // namespace END.
