#include "lio/super_lio.h"

#include <cstdlib>

namespace LI2Sup {

void SuperLIO::caceData() {
  if (!g_save_map || pose_graph_) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = state.GetSE3().matrix();
  pcl::transformPointCloud(g_if_filter ? *ds_undistort_ : *scan_undistort_full_,
                           *world_pc_, transformation);
  static int scan_wait_num = 0;
  if (!world_pc_->empty()) {
    *point_map_ += *world_pc_;
    ++scan_wait_num;
  }
  if (g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }
  static bool reset_cache = true;
  if (reset_cache) {
    reset_cache = false;
    const auto cache = g_save_map_dir + "/PCD";
    std::system(("rm -rf " + cache).c_str());
    std::system(("mkdir -p " + cache).c_str());
  }
  if (point_map_->empty() || scan_wait_num < g_pcd_save_interval) return;
  const auto path = g_save_map_dir + "/PCD/scans_" + std::to_string(++pcd_index_) + ".pcd";
  pcl::io::savePCDFileBinary(path, *point_map_);
  point_map_->clear();
  scan_wait_num = 0;
}

void SuperLIO::ProcessCaceMap() {
  namespace fs = std::filesystem;
  BASIC::PointCloudType::Ptr merged(new BASIC::PointCloudType());
  const auto cache = g_save_map_dir + "/PCD";
  for (const auto& entry : fs::directory_iterator(cache)) {
    if (entry.path().extension() != ".pcd" ||
        entry.path().filename().string().find("scans_") == std::string::npos) continue;
    BASIC::PointCloudType cloud;
    if (pcl::io::loadPCDFile<BASIC::PointType>(entry.path().string(), cloud) == 0) *merged += cloud;
  }
  pcl::VoxelGrid<BASIC::PointType> filter;
  BASIC::PointCloudType output;
  filter.setInputCloud(merged);
  filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
  if (g_if_filter) filter.filter(output); else output = *merged;
  pcl::io::savePCDFileBinary(g_save_map_dir + "/" + g_map_name, output);
}

void SuperLIO::saveMap() {
  if (!g_save_map) return;
  const auto path = g_save_map_dir + "/" + g_map_name;
  if (pose_graph_) {
    const auto map = pose_graph_->buildMap(g_map_ds_size);
    if (map->empty() || pcl::io::savePCDFileBinary(path, *map) < 0) {
      LOG(ERROR) << RED << " ---> Failed to save pose-graph map: " << path << RESET;
      return;
    }
    LOG(INFO) << GREEN << " ---> Saved pose-graph map: " << path
              << " size: " << map->size() << RESET;
    return;
  }
  if (g_pcd_save_interval > 0) {
    if (!point_map_->empty()) {
      pcl::io::savePCDFileBinary(g_save_map_dir + "/PCD/scans_" +
                                 std::to_string(++pcd_index_) + ".pcd", *point_map_);
      point_map_->clear();
    }
    ProcessCaceMap();
    return;
  }
  pcl::VoxelGrid<BASIC::PointType> filter;
  BASIC::PointCloudType output;
  filter.setInputCloud(point_map_);
  filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
  filter.filter(output);
  pcl::io::savePCDFileBinary(path, output);
}

}  // namespace LI2Sup
