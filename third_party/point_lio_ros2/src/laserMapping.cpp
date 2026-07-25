#include <omp.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "IMU_Processing.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "parameters.h"
#include "Estimator.h"


#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;

int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

int frame_ct = 0;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false, flg_first_scan = true;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());

// Time Log Variables
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool lidar_pushed = false, flg_reset = false, flg_exit = false;

vector<BoxPointType> cub_needrm;

deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

// The regular pcl::VoxelGrid converts the full scan bounding box into a dense
// integer index space.  At centimetre-scale leaves this can overflow on a
// wide-range LiDAR scan. ApproximateVoxelGrid keeps a sparse hash instead.
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

MeasureGroup Measures;

sensor_msgs::msg::Imu imu_last, imu_next;
sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

auto logger = rclcpp::get_logger("laserMapping");

V3D OdomChild_T_wrt_Body(Zero3d);
M3D OdomChild_R_wrt_Body(Eye3d);

void SigHandle(int sig) {
    flg_exit = true;
    RCLCPP_WARN(logger, "catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang;
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    if (use_imu_as_input) {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a  
    } else {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
                kf_output.x_.gravity(2)); // Bias_a
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_body_imu = kf_output.x_.offset_R_L_I.normalized() * p_body_lidar + kf_output.x_.offset_T_L_I;
        } else {
            p_body_imu = kf_input.x_.offset_R_L_I.normalized() * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    } else {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

int points_cache_size = 0;

void points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void lasermap_fov_segment() {
    cub_needrm.shrink_to_fit();

    V3D pos_LiD;
    if (use_imu_as_input) {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
    } else {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if (cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    double time_div = get_time_sec(msg->header.stamp);
    p_pre->process(msg, ptr);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            // cout << "check time:" << ptr->points[i].curvature << endl;
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar; //get_time_sec(msg->header.stamp);
        }
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    double time_div = get_time_sec(msg->header.stamp);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar;
        }
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
    publish_count++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_lag_imu_to_lidar);
    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        RCLCPP_ERROR(logger, "imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas) {
    if (!imu_en) {
        if (!lidar_buffer.empty()) {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            if (meas.lidar->points.size() < 1) {
                cout << "lose lidar" << std::endl;
                return false;
            }
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            return true;
        }
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() < 1) {
            cout << "lose lidar" << endl;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points) {
            if (pt.curvature > end_time) {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    } else if (!init_map) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;

void rebuild_incremental_map_after_floor_jump() {
    PointVector new_map_points;
    if (feats_down_world != nullptr) {
        new_map_points = feats_down_world->points;
    }
    ikdtree.Build(new_map_points);
    PointVector().swap(ikdtree.PCL_Storage);
    cub_needrm.clear();
    for (auto &points : Nearest_Points) {
        points.clear();
    }
    RCLCPP_WARN(
        logger,
        "Point-LIO ikd-tree map rebuilt after floor-height transition: points=%zu",
        new_map_points.size());
}

bool estimate_current_floor_z(double &floor_z) {
    if (!map_update_floor_gate_en || feats_down_world == nullptr || feats_down_world->empty()) {
        return false;
    }

    V3D center;
    if (use_imu_as_input) {
        center = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
    } else {
        center = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }

    const double radius = std::max(0.05, map_update_floor_gate_radius);
    const double radius2 = radius * radius;
    vector<float> z_candidates;
    z_candidates.reserve(feats_down_world->size());
    for (const auto &point : feats_down_world->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        const double dx = static_cast<double>(point.x) - center(0);
        const double dy = static_cast<double>(point.y) - center(1);
        if (dx * dx + dy * dy > radius2) {
            continue;
        }
        z_candidates.push_back(point.z);
    }

    if (static_cast<int>(z_candidates.size()) < std::max(1, map_update_floor_gate_min_points)) {
        return false;
    }

    std::sort(z_candidates.begin(), z_candidates.end());
    const double percentile = std::clamp(map_update_floor_gate_percentile, 0.0, 1.0);
    const auto index = std::min(
        static_cast<std::size_t>(std::round(percentile * static_cast<double>(z_candidates.size() - 1))),
        z_candidates.size() - 1);
    floor_z = z_candidates[index];
    return std::isfinite(floor_z);
}

bool allow_map_incremental_by_floor_gate() {
    if (!map_update_floor_gate_en) {
        return true;
    }

    static bool initialized = false;
    static double stable_floor_z = 0.0;
    static double last_floor_z = 0.0;
    static int stable_count = 0;
    static int skip_frames = 0;
    static bool reset_pending = false;
    static rclcpp::Clock throttle_clock(RCL_SYSTEM_TIME);

    double current_floor_z = 0.0;
    if (!estimate_current_floor_z(current_floor_z)) {
        if (skip_frames > 0) {
            --skip_frames;
            return false;
        }
        return true;
    }

    if (!initialized) {
        initialized = true;
        stable_floor_z = current_floor_z;
        last_floor_z = current_floor_z;
        stable_count = 1;
        return true;
    }

    const double jump = std::abs(current_floor_z - stable_floor_z);
    const double step = std::abs(current_floor_z - last_floor_z);
    last_floor_z = current_floor_z;

    if (jump > map_update_floor_gate_max_jump || step > map_update_floor_gate_max_jump) {
        skip_frames = std::max(skip_frames, std::max(1, map_update_floor_gate_skip_frames_after_jump));
        stable_count = 0;
        reset_pending = map_update_floor_gate_reset_map_after_jump;
        RCLCPP_WARN_THROTTLE(
            logger,
            throttle_clock,
            1000,
            "Map update gated: local floor jump current=%.3f stable=%.3f jump=%.3f step=%.3f skip=%d",
            current_floor_z,
            stable_floor_z,
            jump,
            step,
            skip_frames);
        return false;
    }

    if (std::abs(current_floor_z - stable_floor_z) <= map_update_floor_gate_stable_band) {
        stable_count = std::min(stable_count + 1, std::max(1, map_update_floor_gate_stable_frames));
    } else {
        stable_count = 0;
    }

    if (skip_frames > 0) {
        --skip_frames;
        if (stable_count < std::max(1, map_update_floor_gate_stable_frames)) {
            return false;
        }
    }

    if (reset_pending && stable_count >= std::max(1, map_update_floor_gate_stable_frames)) {
        rebuild_incremental_map_after_floor_jump();
        reset_pending = false;
        stable_floor_z = current_floor_z;
        last_floor_z = current_floor_z;
        return false;
    }

    const double alpha = 0.2;
    stable_floor_z = alpha * current_floor_z + (1.0 - alpha) * stable_floor_z;
    return true;
}

void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            /* If the nearest points is definitely outside the downsample box */
            if (fabs(points_near[0].x - mid_point.x) > 1.732 * filter_size_map_min ||
                fabs(points_near[0].y - mid_point.y) > 1.732 * filter_size_map_min ||
                fabs(points_near[0].z - mid_point.z) > 1.732 * filter_size_map_min) {
                PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                continue;
            }
            /* Check if there is a point already in the downsample box */
            float dist = calc_dist<float>(feats_down_world->points[i], mid_point);
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                /* Those points which are outside the downsample box should not be considered. */
                if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min &&
                    fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min &&
                    fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
        } else {
            // PointToAdd.emplace_back(feats_down_world->points[i]);
            PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        }
    }
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

void publish_init_kdtree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {
    
    if (odom_only) {return;}

    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);

    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_header_frame_id;
    if (!odom_only) {
        pubLaserCloudFullRes->publish(laserCloudmsg);
    }
}

void publish_local_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLocalMap) {
    
    if (odom_only || !local_map_pub_en || !pubLocalMap || ikdtree.Root_Node == nullptr) {return;}

    static int local_map_frame_count = 0;
    const int every_n = std::max(1, local_map_publish_every_n_frames);
    if ((local_map_frame_count++ % every_n) != 0) {return;}

    V3D center;
    if (use_imu_as_input) {
        center = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
    } else {
        center = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }

    BoxPointType roi_box;
    roi_box.vertex_min[0] = center(0) - 0.5 * local_map_x_length;
    roi_box.vertex_max[0] = center(0) + 0.5 * local_map_x_length;
    roi_box.vertex_min[1] = center(1) - 0.5 * local_map_y_length;
    roi_box.vertex_max[1] = center(1) + 0.5 * local_map_y_length;
    roi_box.vertex_min[2] = center(2) - 0.5 * local_map_z_length;
    roi_box.vertex_max[2] = center(2) + 0.5 * local_map_z_length;

    PointVector roi_points;
    ikdtree.Box_Search(roi_box, roi_points);
    if (roi_points.empty()) {return;}

    PointCloudXYZI::Ptr local_map(new PointCloudXYZI());
    local_map->points = std::move(roi_points);
    local_map->height = 1;
    local_map->width = local_map->points.size();
    local_map->is_dense = true;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*local_map, msg);
    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = odom_header_frame_id;
    pubLocalMap->publish(msg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {

    if (odom_only) {return;}

    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            // if (i % 3 == 0)
            // {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; // 
            // }
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = odom_header_frame_id;
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body) {

    if (odom_only) {return;}

    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

template<typename T>
void set_posestamp(T &out) {
    Eigen::Quaterniond body_q;
    V3D body_p;
    if (!use_imu_as_input) {
        body_p = kf_output.x_.pos;
        body_q = kf_output.x_.rot.normalized();
    } else {
        body_p = kf_input.x_.pos;
        body_q = kf_input.x_.rot.normalized();
    }

    const M3D child_R_body = OdomChild_R_wrt_Body.transpose();
    const M3D map_R_child = body_q.toRotationMatrix() * child_R_body;
    const V3D map_p_child = body_p - map_R_child * OdomChild_T_wrt_Body;
    const Eigen::Quaterniond child_q(map_R_child);

    out.position.x = map_p_child(0);
    out.position.y = map_p_child(1);
    out.position.z = map_p_child(2);
    out.orientation.x = child_q.x();
    out.orientation.y = child_q.y();
    out.orientation.z = child_q.z();
    out.orientation.w = child_q.w();
}

template<typename T>
void set_twist(T &out) {
    if (!use_imu_as_input) {
        out.linear.x = kf_output.x_.vel(0);
        out.linear.y = kf_output.x_.vel(1);
        out.linear.z = kf_output.x_.vel(2);
        out.angular.x = kf_output.x_.omg(0);
        out.angular.y = kf_output.x_.omg(1);
        out.angular.z = kf_output.x_.omg(2);
    } else {
        out.linear.x = kf_input.x_.vel(0);
        out.linear.y = kf_input.x_.vel(1);
        out.linear.z = kf_input.x_.vel(2);
        out.angular.x = imu_last.angular_velocity.x;
        out.angular.y = imu_last.angular_velocity.y;
        out.angular.z = imu_last.angular_velocity.z;
    }
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped,
                      std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_br) {

    odomAftMapped.header.frame_id = odom_header_frame_id;
    odomAftMapped.child_frame_id = odom_child_frame_id;

    if (publish_odometry_without_downsample) {
        odomAftMapped.header.stamp = get_ros_time(time_current);
    } else {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    set_twist(odomAftMapped.twist.twist);

    if (odom_only){
        Matrix3d cov = kf_output.get_P().block<3, 3>(0, 0);

        // Get the position components (first 3x3)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                odomAftMapped.pose.covariance[6 * i + j] = cov(i, j);
            }
        }

        odomAftMapped.pose.covariance[21] = 0.0;    // Covariance for roll
        odomAftMapped.pose.covariance[28] = 0.0;    // Covariance for pitch
        odomAftMapped.pose.covariance[35] = 0.05;   // Covariance for yaw

        odomAftMapped.twist.covariance[0] = 0.1;    // Covariance for linear velocity on x
        odomAftMapped.twist.covariance[7] = 0.1;    // Covariance for linear velocity on y
        odomAftMapped.twist.covariance[14] = 0.0;   // Covariance for linear velocity on z
        odomAftMapped.twist.covariance[21] = 0.0;  // Covariance for angular velocity (roll)
        odomAftMapped.twist.covariance[28] = 0.0;  // Covariance for angular velocity (pitch)
        odomAftMapped.twist.covariance[35] = 0.05;  // Covariance for angular velocity (yaw)
    }

    pubOdomAftMapped->publish(odomAftMapped);

    //static tf2_ros::TransformBroadcaster br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = odom_header_frame_id;
    transform.child_frame_id = odom_child_frame_id;

    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;

    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

    transform.header.stamp = odomAftMapped.header.stamp;

    tf_br->sendTransform(transform);

    if (odom_publish_footprint_tf && !odom_footprint_frame_id.empty()) {
        const Eigen::Quaterniond map_q_base(
            odomAftMapped.pose.pose.orientation.w,
            odomAftMapped.pose.pose.orientation.x,
            odomAftMapped.pose.pose.orientation.y,
            odomAftMapped.pose.pose.orientation.z);
        const Eigen::Matrix3d map_R_base = map_q_base.normalized().toRotationMatrix();
        const double yaw = std::atan2(map_R_base(1, 0), map_R_base(0, 0));
        const Eigen::AngleAxisd map_R_footprint_yaw(yaw, Eigen::Vector3d::UnitZ());
        const Eigen::Matrix3d base_R_footprint =
            map_R_base.transpose() * map_R_footprint_yaw.toRotationMatrix();
        const Eigen::Vector3d base_p_footprint(0.0, 0.0, -std::max(0.0, odom_footprint_base_height));
        const Eigen::Quaterniond base_q_footprint(base_R_footprint);

        geometry_msgs::msg::TransformStamped footprint_transform;
        footprint_transform.header = transform.header;
        footprint_transform.header.frame_id = odom_child_frame_id;
        footprint_transform.child_frame_id = odom_footprint_frame_id;
        footprint_transform.transform.translation.x = base_p_footprint.x();
        footprint_transform.transform.translation.y = base_p_footprint.y();
        footprint_transform.transform.translation.z = base_p_footprint.z();
        footprint_transform.transform.rotation.x = base_q_footprint.x();
        footprint_transform.transform.rotation.y = base_q_footprint.y();
        footprint_transform.transform.rotation.z = base_q_footprint.z();
        footprint_transform.transform.rotation.w = base_q_footprint.w();
        tf_br->sendTransform(footprint_transform);
    }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath) {

    if (odom_only) {return;}

    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = odom_header_frame_id;
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");
    readParameters(nh);
    cout << "lidar_type: " << lidar_type << endl;

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = odom_header_frame_id;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;

    /*** initialize variables ***/
    double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    OdomChild_T_wrt_Body << VEC_FROM_ARRAY(odom_child_to_body_T);
    OdomChild_R_wrt_Body << MAT_FROM_ARRAY(odom_child_to_body_R);
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
    kf_input.change_P(P_init);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_out, fout_imu_pbp;
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
    if (lidar_msg_type == "livox_custom" || lidar_msg_type == "custom") {
        sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, rclcpp::SensorDataQoS(), livox_pcl_cbk);
        RCLCPP_INFO(logger, "LiDAR input type: livox_ros_driver2/CustomMsg on %s", lid_topic.c_str());
    } else {
        sub_pcl = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        RCLCPP_INFO(logger, "LiDAR input type: sensor_msgs/PointCloud2 on %s", lid_topic.c_str());
    }
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(2000);
    auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imu_qos, imu_cbk);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_body;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLocalMap;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    if (!odom_only){
        pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_registered", 10);
        pubLaserCloudFullRes_body = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_registered_body", 10);
        pubLaserCloudEffect = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_effected", 10);
        pubLaserCloudMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/Laser_map", 1);
        pubLocalMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                (local_map_topic, 2);
        pubPath = nh->create_publisher<nav_msgs::msg::Path>
                ("/path", 10);
    }

    // Choose topic name depending on odom_only value
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    if (odom_only){
        pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>
                ("/odom_corrected", 20);
    } else {
        pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>
                ("/aft_mapped_to_init", 20);
    }

    //auto plane_pub = nh->create_publisher<visualization_msgs::msg::Marker>
    //        ("/planner_normal", 1000);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    while (rclcpp::ok()) {
        if (flg_exit) break;
        //ros::spinOnce();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(nh);
        executor.spin_some(); // 处理当前可用的回调

        if (sync_packages(Measures)) {
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                cout << "first lidar time" << first_lidar_time << endl;
            }

            if (flg_reset) {
                RCLCPP_WARN(logger, "reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            double t0, t1, t2, t3, t4, t5, match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, feats_undistort);

            if (feats_undistort->empty() || feats_undistort == nullptr) {
                continue;
            }
            if (imu_en) {
                if (!p_imu->gravity_align_) {
                    while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp)) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        // imu_deque.pop();
                    }
                    if (non_station_start) {
                        state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc *= -1;
                    } else {
                        state_in.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                        state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                        state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
                    }
                    if (gravity_align) {
                        Eigen::Matrix3d rot_init;
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                        p_imu->Set_init(state_in.gravity, rot_init);
                        state_in.gravity = state_out.gravity = p_imu->gravity_;
                        state_in.rot = state_out.rot = rot_init;
                        state_in.rot.normalize();
                        state_out.rot.normalize();
                        state_out.acc = -rot_init.transpose() * state_out.gravity;
                    }
                    kf_input.change_x(state_in);
                    kf_output.change_x(state_out);
                }
            } else {
                if (!p_imu->gravity_align_) {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                }
            }
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();
            /*** downsample the feature points in a scan ***/
            t1 = omp_get_wtime();
            if (space_down_sample) {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            } else {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            time_seq = time_compressing<int>(feats_down_body);
            feats_down_size = feats_down_body->points.size();

            /*** initialize the map kdtree ***/
            if (!init_map) {
                if (ikdtree.Root_Node == nullptr) //
                    // if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                }

                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if (init_feats_world->size() < init_map_size) continue;
                ikdtree.Build(init_feats_world->points);
                init_map = true;
                publish_init_kdtree(pubLaserCloudMap); //(pubLaserCloudFullRes);
                continue;
            }
            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);

            for (size_t i = 0; i < feats_down_body->size(); i++) {
                V3D point_this(feats_down_body->points[i].x,
                               feats_down_body->points[i].y,
                               feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                if (extrinsic_est_en) {
                    if (!use_imu_as_input) {
                        point_this = kf_output.x_.offset_R_L_I.normalized() * point_this + kf_output.x_.offset_T_L_I;
                    } else {
                        point_this = kf_input.x_.offset_R_L_I.normalized() * point_this + kf_input.x_.offset_T_L_I;
                    }
                } else {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i] = point_crossmat;
            }

            if (!use_imu_as_input) {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** point by point update ****/

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame) {
                        if (imu_en) {
                            while (time_current > get_time_sec(imu_next.header.stamp)) {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_deque.pop_front();
                                // imu_deque.pop();
                            }

                            angvel_avr
                                    << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr
                                    << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        imu_upda_cov = true;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if (imu_en) {
                        bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                        while (imu_comes) {
                            imu_upda_cov = true;
                            angvel_avr
                                    << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr
                                    << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const = get_time_sec(imu_last.header.stamp); // big problem
                            imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            // if (!imu_comes)
                            {
                                double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;

                                if (dt_cov > 0.0) {
                                    time_update_last = get_time_sec(imu_last.header.stamp);
                                    double propag_imu_start = omp_get_wtime();

                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                    propag_time += omp_get_wtime() - propag_imu_start;
                                    double solve_imu_start = omp_get_wtime();
                                    kf_output.update_iterated_dyn_share_IMU();
                                    solve_time += omp_get_wtime() - solve_imu_start;
                                }
                            }
                        }
                    }

                    double dt = time_current - time_predict_last_const;
                    double propag_state_start = omp_get_wtime();
                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    // if(k == 0)
                    // {
                    //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " " << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
                    // }

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    if (prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (!imu_en && (dt_cov >= imu_time_inte)) // (point_cov_not_prop && imu_prop_cov)
                        {
                            double propag_cov_start = omp_get_wtime();
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            imu_upda_cov = false;
                            time_update_last = time_current;
                            propag_time += omp_get_wtime() - propag_cov_start;
                        }
                    }

                    solve_start = omp_get_wtime();

                    if (publish_odometry_without_downsample) {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_out = kf_output.x_;
                            euler_cur = SO3ToEuler(state_out.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                     << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                     << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                     << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }

                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                    // cout << "pbp output effect feat num:" << effct_feat_num << endl;
                }
            } else {
                bool imu_prop_cov = false;
                effct_feat_num = 0;

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame) {
                        while (time_current > get_time_sec(imu_next.header.stamp)) {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            // imu_deque.pop();
                        }
                        imu_prop_cov = true;
                        // imu_upda_cov = true;

                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current;
                        // if(prop_at_freq_of_imu)
                        {
                            input_in.gyro << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;

                            input_in.acc << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                            // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
                            //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
                            //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);

                            // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
                            //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
                            // 0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);

                            // angvel_avr -= state.bias_g;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }

                    while (time_current > get_time_sec(imu_next.header.stamp)) // && !imu_deque.empty())
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        input_in.gyro
                                << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc
                                << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                        // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
                        //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
                        //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);

                        // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
                        //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
                        //             0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        double dt = get_time_sec(imu_last.header.stamp) - t_last;

                        // if(!prop_at_freq_of_imu)
                        // {       
                        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = get_time_sec(imu_last.header.stamp); //time_current;
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);
                        t_last = get_time_sec(imu_last.header.stamp);
                        imu_prop_cov = true;
                        // imu_upda_cov = true;
                    }

                    double dt = time_current - t_last;
                    t_last = time_current;
                    double propag_start = omp_get_wtime();

                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);

                    propag_time += omp_get_wtime() - propag_start;

                    // if(k == 0)
                    // {
                    //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " " << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
                    // }

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();

                    // if(prop_at_freq_of_imu)
                    // {
                    //     double dt_cov = time_current - time_update_last;
                    //     if ((imu_prop_cov && dt_cov > 0.0) || (dt_cov >= imu_time_inte * 1.2)) 
                    //     {
                    //         double propag_cov_start = omp_get_wtime();
                    //         kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                    //         propag_time += omp_get_wtime() - propag_cov_start;
                    //         time_update_last = time_current;
                    //         imu_prop_cov = false;
                    //     }
                    // }
                    if (publish_odometry_without_downsample) {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_in = kf_input.x_;
                            euler_cur = SO3ToEuler(state_in.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                     << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                     << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }
            }

            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample) {
                publish_odometry(pubOdomAftMapped, tf_broadcaster);
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();

            if (feats_down_size > 4 && allow_map_incremental_by_floor_gate()) {
                if (ikdtree.Root_Node == nullptr) {
                    ikdtree.Build(feats_down_world->points);
                } else {
                    map_incremental();
                }
            }

            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en) publish_path(pubPath);
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
            publish_local_map(pubLocalMap);

            /*** Debug variables Logging ***/
            if (runtime_pos_log) {
                frame_num++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                { aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num; }
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
                       t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                       aver_time_propag);
                if (!publish_odometry_without_downsample) {
                    if (!use_imu_as_input) {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                 << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                 << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                 << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    } else {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                 << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                 << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        rate.sleep();
    }
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
       2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();

    return 0;
}
