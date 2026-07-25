// #ifndef PARAM_H
// #define PARAM_H
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include <string>
#include "preprocess.h"

extern bool odom_only;
extern std::string odom_header_frame_id;
extern std::string odom_child_frame_id;
extern std::vector<double> odom_child_to_body_T;
extern std::vector<double> odom_child_to_body_R;
extern std::string odom_footprint_frame_id;
extern bool odom_publish_footprint_tf;
extern double odom_footprint_base_height;

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;

extern std::string lid_topic, imu_topic;
extern std::string lidar_msg_type;
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int init_map_size, con_frame_num, max_lidar_buffer_size;
extern int path_publish_stride, max_path_pose_count;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern float plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len;
extern float DET_RANGE;
extern bool imu_en, gravity_align, non_station_start;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov;
extern int lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern bool runtime_pos_log, pcd_save_en, path_en;
extern bool scan_pub_en, scan_body_pub_en;
extern bool local_map_pub_en;
extern std::string local_map_topic;
extern double local_map_x_length, local_map_y_length, local_map_z_length;
extern int local_map_publish_every_n_frames;
extern bool map_update_floor_gate_en;
extern double map_update_floor_gate_radius;
extern double map_update_floor_gate_percentile;
extern int map_update_floor_gate_min_points;
extern double map_update_floor_gate_max_jump;
extern double map_update_floor_gate_stable_band;
extern int map_update_floor_gate_stable_frames;
extern int map_update_floor_gate_skip_frames_after_jump;
extern bool map_update_floor_gate_reset_map_after_jump;
extern shared_ptr<Preprocess> p_pre;
extern double time_lag_imu_to_lidar;

void readParameters(shared_ptr<rclcpp::Node> &nh);
