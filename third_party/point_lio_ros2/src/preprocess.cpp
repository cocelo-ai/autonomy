#include "preprocess.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#define RETURN0     0x00
#define RETURN0AND1 0x10

namespace
{

const sensor_msgs::msg::PointField * find_field(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const std::string &name)
{
    const auto it = std::find_if(
        cloud.fields.begin(),
        cloud.fields.end(),
        [&name](const sensor_msgs::msg::PointField &field) { return field.name == name; });
    return it == cloud.fields.end() ? nullptr : &(*it);
}

template<typename T>
T read_as(const std::uint8_t *ptr)
{
    T value{};
    std::memcpy(&value, ptr, sizeof(T));
    return value;
}

double read_numeric(
    const std::uint8_t *base,
    const sensor_msgs::msg::PointField *field,
    const double fallback = 0.0)
{
    if (field == nullptr) {
        return fallback;
    }

    const auto *ptr = base + field->offset;
    switch (field->datatype) {
        case sensor_msgs::msg::PointField::INT8:
            return static_cast<double>(read_as<std::int8_t>(ptr));
        case sensor_msgs::msg::PointField::UINT8:
            return static_cast<double>(read_as<std::uint8_t>(ptr));
        case sensor_msgs::msg::PointField::INT16:
            return static_cast<double>(read_as<std::int16_t>(ptr));
        case sensor_msgs::msg::PointField::UINT16:
            return static_cast<double>(read_as<std::uint16_t>(ptr));
        case sensor_msgs::msg::PointField::INT32:
            return static_cast<double>(read_as<std::int32_t>(ptr));
        case sensor_msgs::msg::PointField::UINT32:
            return static_cast<double>(read_as<std::uint32_t>(ptr));
        case sensor_msgs::msg::PointField::FLOAT32:
            return static_cast<double>(read_as<float>(ptr));
        case sensor_msgs::msg::PointField::FLOAT64:
            return read_as<double>(ptr);
        default:
            return fallback;
    }
}

bool is_valid_livox_tag(const std::uint8_t tag)
{
    const auto return_type = tag & 0x30;
    return return_type == RETURN0 || return_type == RETURN0AND1;
}

}  // namespace

Preprocess::Preprocess()
        : lidar_type(AVIA),
          blind(0.01),
          point_filter_num(1),
          horizontal_fov_degree(360.0),
          horizontal_fov_center_degree(0.0),
          horizontal_fov_cos_threshold(-1.0),
          horizontal_fov_center_cos(1.0),
          horizontal_fov_center_sin(0.0) {
    inf_bound = 10;
    N_SCANS = 6;
    SCAN_RATE = 10;
    group_size = 8;
    disA = 0.01;
    disA = 0.1; // B?
    p2l_ratio = 225;
    limit_maxmid = 6.25;
    limit_midmin = 6.25;
    limit_maxmin = 3.24;
    jump_up_limit = 170.0;
    jump_down_limit = 8.0;
    cos160 = 160.0;
    edgea = 2;
    edgeb = 0.1;
    smallp_intersect = 172.5;
    smallp_ratio = 1.2;
    given_offset_time = false;

    jump_up_limit = cos(jump_up_limit / 180 * M_PI);
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);
    cos160 = cos(cos160 / 180 * M_PI);
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
    lidar_type = lid_type;
    blind = bld;
    point_filter_num = pfilt_num;
}

void Preprocess::set_horizontal_fov(double fov_degree, double center_degree) {
    horizontal_fov_degree = std::max(1.0, std::min(360.0, fov_degree));
    horizontal_fov_center_degree = center_degree;
    const double center_rad = horizontal_fov_center_degree * M_PI / 180.0;
    horizontal_fov_center_cos = std::cos(center_rad);
    horizontal_fov_center_sin = std::sin(center_rad);
    if (horizontal_fov_degree >= 359.999) {
        horizontal_fov_cos_threshold = -1.0;
    } else {
        horizontal_fov_cos_threshold = std::cos(0.5 * horizontal_fov_degree * M_PI / 180.0);
    }
}

bool Preprocess::in_horizontal_fov(float x, float y) const {
    if (horizontal_fov_degree >= 359.999) {
        return true;
    }
    const double norm = std::hypot(static_cast<double>(x), static_cast<double>(y));
    if (norm <= 1.0e-6) {
        return true;
    }
    const double dot =
        (static_cast<double>(x) * horizontal_fov_center_cos +
        static_cast<double>(y) * horizontal_fov_center_sin) / norm;
    return dot >= horizontal_fov_cos_threshold;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out) {
    switch (time_unit) {
        case SEC:
            time_unit_scale = 1.e3f;
            break;
        case MS:
            time_unit_scale = 1.f;
            break;
        case US:
            time_unit_scale = 1.e-3f;
            break;
        case NS:
            time_unit_scale = 1.e-6f;
            break;
        default:
            time_unit_scale = 1.f;
            break;
    }

    avia_handler(msg);
    *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out) {
    switch (time_unit) {
        case SEC:
            time_unit_scale = 1.e3f;
            break;
        case MS:
            time_unit_scale = 1.f;
            break;
        case US:
            time_unit_scale = 1.e-3f;
            break;
        case NS:
            time_unit_scale = 1.e-6f;
            break;
        default:
            time_unit_scale = 1.f;
            break;
    }

    switch (lidar_type) {
        case AVIA:
            avia_handler(msg);
            break;

        case OUST64:
            oust64_handler(msg);
            break;

        case VELO16:
            velodyne_handler(msg);
            break;

        case HESAIxt32:
            hesai_handler(msg);
            break;

        case UNILIDAR:
            unilidar_handler(msg);
            break;

        default:
            printf("Error LiDAR Type");
            break;
    }
    *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    const int plsize = static_cast<int>(msg->point_num);
    if (plsize <= 1) {
        return;
    }

    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    uint valid_num = 0;
    for (int i = 1; i < plsize; i++) {
        const auto &src = msg->points[static_cast<std::size_t>(i)];
        if ((src.line >= N_SCANS) || !is_valid_livox_tag(src.tag)) {
            continue;
        }

        valid_num++;
        if (point_filter_num > 1 && valid_num % point_filter_num != 0) {
            continue;
        }

        auto &dst = pl_full[static_cast<std::size_t>(i)];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.reflectivity;
        dst.curvature = static_cast<float>(src.offset_time) * time_unit_scale;

        if (!std::isfinite(dst.x) || !std::isfinite(dst.y) || !std::isfinite(dst.z)) {
            continue;
        }
        if (i == 1) {
            dst.curvature = std::fabs(dst.curvature) < 1.0F ? dst.curvature : 0.0F;
        } else {
            const auto &prev = pl_full[static_cast<std::size_t>(i - 1)];
            dst.curvature = std::fabs(dst.curvature - prev.curvature) < 1.0F ?
                dst.curvature :
                prev.curvature + 0.004166667F;
        }

        const auto &prev = pl_full[static_cast<std::size_t>(i - 1)];
        const bool non_duplicate =
            std::fabs(dst.x - prev.x) > 1.0e-7F ||
            std::fabs(dst.y - prev.y) > 1.0e-7F ||
            std::fabs(dst.z - prev.z) > 1.0e-7F;
        const bool outside_blind =
            dst.x * dst.x + dst.y * dst.y + dst.z * dst.z > static_cast<float>(blind * blind);
        if (non_duplicate && outside_blind && in_horizontal_fov(dst.x, dst.y)) {
            pl_surf.push_back(dst);
        }
    }
}

void Preprocess::avia_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    const auto *x_field = find_field(*msg, "x");
    const auto *y_field = find_field(*msg, "y");
    const auto *z_field = find_field(*msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
        printf("Livox PointCloud2 missing xyz fields");
        return;
    }

    const auto *intensity_field = find_field(*msg, "intensity");
    if (intensity_field == nullptr) {
        intensity_field = find_field(*msg, "reflectivity");
    }
    const auto *tag_field = find_field(*msg, "tag");
    const auto *line_field = find_field(*msg, "line");
    if (line_field == nullptr) {
        line_field = find_field(*msg, "ring");
    }
    const auto *time_field = find_field(*msg, "timestamp");
    if (time_field == nullptr) {
        time_field = find_field(*msg, "time");
    }
    if (time_field == nullptr) {
        time_field = find_field(*msg, "t");
    }
    if (time_field == nullptr) {
        time_field = find_field(*msg, "offset_time");
    }

    const auto point_count = static_cast<std::size_t>(msg->width) * msg->height;
    if (point_count == 0 || msg->point_step == 0) {
        return;
    }

    pl_surf.reserve(point_count);
    const auto *data = msg->data.data();
    const double first_stamp = time_field != nullptr ? read_numeric(data, time_field, 0.0) : 0.0;
    const bool has_time = time_field != nullptr;

    for (std::size_t i = 0; i < point_count; ++i) {
        if (point_filter_num > 1 && static_cast<int>(i % point_filter_num) != 0) {
            continue;
        }

        const auto *base = data + i * msg->point_step;
        const auto tag = static_cast<std::uint8_t>(read_numeric(base, tag_field, RETURN0AND1));
        if (tag_field != nullptr && !is_valid_livox_tag(tag)) {
            continue;
        }

        const auto line = static_cast<int>(read_numeric(base, line_field, 0.0));
        if (line_field != nullptr && (line < 0 || line >= N_SCANS)) {
            continue;
        }

        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = static_cast<float>(read_numeric(base, x_field));
        added_pt.y = static_cast<float>(read_numeric(base, y_field));
        added_pt.z = static_cast<float>(read_numeric(base, z_field));
        added_pt.intensity = static_cast<float>(read_numeric(base, intensity_field, 0.0));

        if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
            continue;
        }
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z <= blind * blind ||
            !in_horizontal_fov(added_pt.x, added_pt.y)) {
            continue;
        }

        if (has_time) {
            const double stamp = read_numeric(base, time_field, first_stamp);
            added_pt.curvature = static_cast<float>((stamp - first_stamp) * time_unit_scale);
        } else {
            added_pt.curvature = static_cast<float>(
                1000.0 * static_cast<double>(i) /
                std::max<std::size_t>(1, point_count - 1) /
                std::max(1, SCAN_RATE));
        }

        pl_surf.points.push_back(added_pt);
    }
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);


    double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind * blind) ||
            !in_horizontal_fov(pl_orig.points[i].x, pl_orig.points[i].y)) {
            continue;
        }

        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

        pl_surf.points.push_back(added_pt);
    }

    // pub_func(pl_surf, pub_full, msg->header.stamp);
    // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS, true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time = true;
    } else {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;

        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;
        if (!given_offset_time) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                // printf("layer: %d; is first: %d", layer, is_first[layer]);
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num == 0) {
            if (added_pt.x * added_pt.x
                + added_pt.y * added_pt.y
                + added_pt.z * added_pt.z > (blind * blind) &&
                in_horizontal_fov(added_pt.x, added_pt.y))
            {
                pl_surf.points.push_back(added_pt);
            }
        }
    }

}

void Preprocess::unilidar_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<unilidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    pl_surf.reserve(plsize);

    // std::cout << "plsize = " << plsize << ", given_offset_time = " << given_offset_time << std::endl;
    int countElimnated = 0;
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;

      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      
      added_pt.intensity = pl_orig.points[i].intensity;

      added_pt.curvature = pl_orig.points[i].time * time_unit_scale; 

      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind) &&
          in_horizontal_fov(added_pt.x, added_pt.y))
      {
        pl_surf.points.push_back(added_pt);
      }
      else
      {
        countElimnated++;
      }
    }

    // std::cout << "pl_surf.size() = " << pl_surf.size() << ", countElimnated = " << countElimnated << std::endl;
    
}

void Preprocess::hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS, true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].timestamp > 0) {
        given_offset_time = true;
    } else {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    double time_head = pl_orig.points[0].timestamp;

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = (pl_orig.points[i].timestamp - time_head) *
                             1000.f; // time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;
        if (!given_offset_time) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                // printf("layer: %d; is first: %d", layer, is_first[layer]);
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind) &&
                in_horizontal_fov(added_pt.x, added_pt.y)) {
                pl_surf.points.push_back(added_pt);
            }
        }
    }

}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types) {
    int plsize = pl.size();
    int plsize2;
    if (plsize == 0) {
        printf("something wrong\n");
        return;
    }
    uint head = 0;

    while (types[head].range < blind) {
        head++;
    }

    // Surf
    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex = 0, i2;
    uint last_i = 0;
    uint last_i_nex = 0;
    int last_state = 0;
    int plane_type;

    for (uint i = head; i < plsize2; i++) {
        if (types[i].range < blind) {
            continue;
        }

        i2 = i;

        plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

        if (plane_type == 1) {
            for (uint j = i; j <= i_nex; j++) {
                if (j != i && j != i_nex) {
                    types[j].ftype = Real_Plane;
                } else {
                    types[j].ftype = Poss_Plane;
                }
            }

            // if(last_state==1 && fabs(last_direct.sum())>0.5)
            if (last_state == 1 && last_direct.norm() > 0.1) {
                double mod = last_direct.transpose() * curr_direct;
                if (mod > -0.707 && mod < 0.707) {
                    types[i].ftype = Edge_Plane;
                } else {
                    types[i].ftype = Real_Plane;
                }
            }

            i = i_nex - 1;
            last_state = 1;
        } else // if(plane_type == 2)
        {
            i = i_nex;
            last_state = 0;
        }

        last_i = i2;
        last_i_nex = i_nex;
        last_direct = curr_direct;
    }

    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for (uint i = head + 3; i < plsize2; i++) {
        if (types[i].range < blind || types[i].ftype >= Real_Plane) {
            continue;
        }

        if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) {
            continue;
        }

        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
        Eigen::Vector3d vecs[2];

        for (int j = 0; j < 2; j++) {
            int m = -1;
            if (j == 1) {
                m = 1;
            }

            if (types[i + m].range < blind) {
                if (types[i].range > inf_bound) {
                    types[i].edj[j] = Nr_inf;
                } else {
                    types[i].edj[j] = Nr_blind;
                }
                continue;
            }

            vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
            vecs[j] = vecs[j] - vec_a;

            types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
            if (types[i].angle[j] < jump_up_limit) {
                types[i].edj[j] = Nr_180;
            } else if (types[i].angle[j] > jump_down_limit) {
                types[i].edj[j] = Nr_zero;
            }
        }

        types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
        if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 &&
            types[i].dista > 4 * types[i - 1].dista) {
            if (types[i].intersect > cos160) {
                if (edge_jump_judge(pl, types, i, Prev)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        } else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 &&
                   types[i - 1].dista > 4 * types[i].dista) {
            if (types[i].intersect > cos160) {
                if (edge_jump_judge(pl, types, i, Next)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        } else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
            if (edge_jump_judge(pl, types, i, Prev)) {
                types[i].ftype = Edge_Jump;
            }
        } else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
            if (edge_jump_judge(pl, types, i, Next)) {
                types[i].ftype = Edge_Jump;
            }

        } else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
            if (types[i].ftype == Nor) {
                types[i].ftype = Wire;
            }
        }
    }

    plsize2 = plsize - 1;
    double ratio;
    for (uint i = head + 1; i < plsize2; i++) {
        if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) {
            continue;
        }

        if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) {
            continue;
        }

        if (types[i].ftype == Nor) {
            if (types[i - 1].dista > types[i].dista) {
                ratio = types[i - 1].dista / types[i].dista;
            } else {
                ratio = types[i].dista / types[i - 1].dista;
            }

            if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
                if (types[i - 1].ftype == Nor) {
                    types[i - 1].ftype = Real_Plane;
                }
                if (types[i + 1].ftype == Nor) {
                    types[i + 1].ftype = Real_Plane;
                }
                types[i].ftype = Real_Plane;
            }
        }
    }

    int last_surface = -1;
    for (uint j = head; j < plsize; j++) {
        if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
            if (last_surface == -1) {
                last_surface = j;
            }

            if (j == uint(last_surface + point_filter_num - 1)) {
                PointType ap;
                ap.x = pl[j].x;
                ap.y = pl[j].y;
                ap.z = pl[j].z;
                ap.intensity = pl[j].intensity;
                ap.curvature = pl[j].curvature;
                pl_surf.push_back(ap);

                last_surface = -1;
            }
        } else {
            if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
                pl_corn.push_back(pl[j]);
            }
            if (last_surface != -1) {
                PointType ap;
                for (uint k = last_surface; k < j; k++) {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.intensity += pl[k].intensity;
                    ap.curvature += pl[k].curvature;
                }
                ap.x /= (j - last_surface);
                ap.y /= (j - last_surface);
                ap.z /= (j - last_surface);
                ap.intensity /= (j - last_surface);
                ap.curvature /= (j - last_surface);
                pl_surf.push_back(ap);
            }
            last_surface = -1;
        }
    }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct) {
    pl.height = 1;
    pl.width = pl.size();
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(pl, output);
    output.header.frame_id = "livox";
    output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex,
                            Eigen::Vector3d &curr_direct) {
    double group_dis = disA * types[i_cur].range + disB;
    group_dis = group_dis * group_dis;
    // i_nex = i_cur;

    double two_dis;
    vector<double> disarr;
    disarr.reserve(20);

    for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++) {
        if (types[i_nex].range < blind) {
            curr_direct.setZero();
            return 2;
        }
        disarr.push_back(types[i_nex].dista);
    }

    for (;;) {
        if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

        if (types[i_nex].range < blind) {
            curr_direct.setZero();
            return 2;
        }
        vx = pl[i_nex].x - pl[i_cur].x;
        vy = pl[i_nex].y - pl[i_cur].y;
        vz = pl[i_nex].z - pl[i_cur].z;
        two_dis = vx * vx + vy * vy + vz * vz;
        if (two_dis >= group_dis) {
            break;
        }
        disarr.push_back(types[i_nex].dista);
        i_nex++;
    }

    double leng_wid = 0;
    double v1[3], v2[3];
    for (uint j = i_cur + 1; j < i_nex; j++) {
        if ((j >= pl.size()) || (i_cur >= pl.size())) break;
        v1[0] = pl[j].x - pl[i_cur].x;
        v1[1] = pl[j].y - pl[i_cur].y;
        v1[2] = pl[j].z - pl[i_cur].z;

        v2[0] = v1[1] * vz - vy * v1[2];
        v2[1] = v1[2] * vx - v1[0] * vz;
        v2[2] = v1[0] * vy - vx * v1[1];

        double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
        if (lw > leng_wid) {
            leng_wid = lw;
        }
    }


    if ((two_dis * two_dis / leng_wid) < p2l_ratio) {
        curr_direct.setZero();
        return 0;
    }

    uint disarrsize = disarr.size();
    for (uint j = 0; j < disarrsize - 1; j++) {
        for (uint k = j + 1; k < disarrsize; k++) {
            if (disarr[j] < disarr[k]) {
                leng_wid = disarr[j];
                disarr[j] = disarr[k];
                disarr[k] = leng_wid;
            }
        }
    }

    if (disarr[disarr.size() - 2] < 1e-16) {
        curr_direct.setZero();
        return 0;
    }

    if (lidar_type == AVIA) {
        double dismax_mid = disarr[0] / disarr[disarrsize / 2];
        double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

        if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
            curr_direct.setZero();
            return 0;
        }
    } else {
        double dismax_min = disarr[0] / disarr[disarrsize - 2];
        if (dismax_min >= limit_maxmin) {
            curr_direct.setZero();
            return 0;
        }
    }

    curr_direct << vx, vy, vz;
    curr_direct.normalize();
    return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir) {
    if (nor_dir == 0) {
        if (types[i - 1].range < blind || types[i - 2].range < blind) {
            return false;
        }
    } else if (nor_dir == 1) {
        if (types[i + 1].range < blind || types[i + 2].range < blind) {
            return false;
        }
    }
    double d1 = types[i + nor_dir - 1].dista;
    double d2 = types[i + 3 * nor_dir - 2].dista;
    double d;

    if (d1 < d2) {
        d = d1;
        d1 = d2;
        d2 = d;
    }

    d1 = sqrt(d1);
    d2 = sqrt(d2);

    if (d1 > edgea * d2 || (d1 - d2) > edgeb) {
        return false;
    }

    return true;
}
