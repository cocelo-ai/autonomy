
#include "lio/super_lio_reloc.h"

#include <algorithm>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/crop_box.h>


using namespace BASIC;

namespace LI2Sup{

void SuperLIOReLoc::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());
  point_map_.reset(new PointCloudType());
  init_obs_data_.reset(new PointCloudType());
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;

  auto start_time = std::chrono::high_resolution_clock::now();
  SuperLIOReLoc::map_init();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init success. Time: " << duration.count() << " ms." << RESET;

  state_fn_ = &SuperLIOReLoc::stateWaitKFInit;
}


bool SuperLIOReLoc::map_init(){
  static bool pcd_loaded = false;
  if(pcd_loaded) return true;

  std::string map_name = g_save_map_dir + "/" + g_map_name;
  if(pcl::io::loadPCDFile<PointType>(map_name, *point_map_) == -1){
    LOG(ERROR) << RED << " ---> Load map failed. File: " << map_name << RESET;
    return false;
  }

  std::vector<int> useless_indices;
  pcl::removeNaNFromPointCloud(*point_map_, *point_map_, useless_indices);

  LOG(INFO) << GREEN << " ---> Load map success. File: " << map_name << RESET;
  LOG(INFO) << GREEN << " ---> Map size: " << point_map_->size() << RESET;
  ivox_->printInfo();

  pcd_loaded = true;

  data_wrapper_->set_global_map(point_map_);
  data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_);
  return true;
}


bool SuperLIOReLoc::kf_init(){
  const int need_init_frames = 10;
  static int imu_cout = 0;
  static int init_frame_count = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  /// get init guess from ROS topic.
  if(flg_get_init_guess_){
    imu_cout = 0;
    init_frame_count = 0;
    init_obs_data_->clear();
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    flg_get_init_guess_ = false;
    return false;
  }

  CloudPtr point_cloud_pcl = CloudPtr(new PointCloudType());
  for(std::size_t i = 0; i < measures_.lidar.pc->size(); i++){
    auto p = measures_.lidar.pc->points[i];
    PointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point_cloud_pcl->points.push_back(point);
  }

  if(init_frame_count < need_init_frames){
    *init_obs_data_ += *point_cloud_pcl;
  }
  init_frame_count++;

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  if(imu_cout < 20){
    return false;
  }

  if(init_frame_count < need_init_frames){
    return false;
  }

  LOG(INFO) << YELLOW << " ---> INIT start... obs_data size: " << init_obs_data_->size() << " target size: " << point_map_->size() << RESET;

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 
  M3 rot = R_yaw_inv * init_rot;

  M3 init_guess_R_ = re_init_pose_.R_ * rot;
  V3 init_guess_t_ = re_init_pose_.t_;
  M4 init_guess_T = M4::Identity();
  init_guess_T.block<3, 3>(0, 0) = init_guess_R_;
  init_guess_T.block<3, 1>(0, 3) = init_guess_t_;


  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_src(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*init_obs_data_, *tmp_src, g_lidar_imu.matrix().cast<float>());

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon(1e-4);
  ndt.setEuclideanFitnessEpsilon(1e-4);
  ndt.setMaximumIterations(25);
  ndt.setResolution(1.0);
  ndt.setInputTarget(point_map_);

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(4.0);
  icp.setMaximumIterations(40);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-4);
  icp.setRANSACIterations(0);
  icp.setInputTarget(point_map_);

  ndt.setInputSource(tmp_src);
  icp.setInputSource(tmp_src);

  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
  ndt.align(*unused_result, init_guess_T.matrix().cast<float>());
  icp.align(*unused_result, ndt.getFinalTransformation());

  if (icp.hasConverged() == false || icp.getFitnessScore() > 1.5)
  // if (icp.hasConverged() == false)
  {
    /// reset init state.
    imu_cout = 0;
    init_frame_count = 0;
    init_obs_data_->clear();
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    LOG(INFO) << RED << " ---> Global ICP Converged Fail! FitnessScore: " << icp.getFitnessScore() << RESET;
    return false;
  } else{
    init_guess_T = icp.getFinalTransformation().cast<scalar>();
    LOG(INFO) << GREEN << " ---> Global ICP Converged Succeed! FitnessScore: " << icp.getFitnessScore() << RESET;
  }

  LOG(INFO) << GREEN << "\n" << init_guess_T << RESET;

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  const SE3 odom_to_imu(SO3(rot), V3::Zero());
  state.R = odom_to_imu.so3();
  state.p = odom_to_imu.t_;
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  map_to_odom_ = SE3(init_guess_T) * odom_to_imu.inverse();
  data_wrapper_->setMapToOdom(map_to_odom_);
  VV3 initial_points;
  initial_points.reserve(init_obs_data_->size());
  for (const auto& point : init_obs_data_->points) {
    initial_points.push_back(odom_to_imu * (g_lidar_imu * V3(point.x, point.y, point.z)));
  }
  ivox_->insert(initial_points);
  kf_->SetLastObsTime(measures_.lidar.end_time);
  last_global_update_ = measures_.lidar.end_time;

  {
    init_obs_data_->clear();
    init_obs_data_ = nullptr;
    data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_, true);
  }

  return true;
}


void SuperLIOReLoc::UpdateMap() {
  SuperLIO::UpdateMap();
  const double stamp = measures_.lidar.end_time;
  if (stamp - last_global_update_ >= g_reloc_global_update_sec) {
    last_global_update_ = stamp;
    updateMapCorrection();
  }
}


void SuperLIOReLoc::updateMapCorrection() {
  if (point_map_->empty() || ds_undistort_->empty()) {
    return;
  }
  const SE3 odom_to_imu = kf_->GetSE3();
  const SE3 predicted = map_to_odom_ * odom_to_imu;
  pcl::CropBox<PointType> crop;
  crop.setInputCloud(point_map_);
  const float radius = g_reloc_global_search_radius;
  crop.setMin(Eigen::Vector4f(predicted.t_[0] - radius, predicted.t_[1] - radius,
                              predicted.t_[2] - radius, 1.0F));
  crop.setMax(Eigen::Vector4f(predicted.t_[0] + radius, predicted.t_[1] + radius,
                              predicted.t_[2] + radius, 1.0F));
  CloudPtr target(new PointCloudType());
  crop.filter(*target);
  if (target->size() < 100U) {
    return;
  }
  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  ndt.setInputSource(ds_undistort_);
  ndt.setInputTarget(target);
  ndt.setResolution(1.0);
  ndt.setMaximumIterations(15);
  PointCloudType aligned;
  ndt.align(aligned, predicted.matrix());
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(ds_undistort_);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(3.0);
  icp.setMaximumIterations(24);
  icp.align(aligned, ndt.getFinalTransformation());
  if (!icp.hasConverged() || icp.getFitnessScore() > g_reloc_global_fitness_threshold) {
    return;
  }
  const SE3 target_correction = SE3(icp.getFinalTransformation()) * odom_to_imu.inverse();
  const float alpha = std::clamp(g_reloc_correction_alpha, 0.0F, 1.0F);
  const Quat rotation(map_to_odom_.R_);
  const Quat target_rotation(target_correction.R_);
  map_to_odom_ = SE3(rotation.slerp(alpha, target_rotation).toRotationMatrix(),
                      (1.0F - alpha) * map_to_odom_.t_ + alpha * target_correction.t_);
  data_wrapper_->setMapToOdom(map_to_odom_);
}



} // namespace END.
