cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(CycloneDDS REQUIRED CONFIG)
find_package(core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HeightMap.msg"
  DEPENDENCIES std_msgs
)

add_executable(height_map_bridge
  src/dds_height_map_publisher.cpp
  src/height_map_bridge.cpp
  src/height_map_bridge_main.cpp
)
ament_target_dependencies(height_map_bridge
  geometry_msgs
  grid_map_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(height_map_bridge_typesupport
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(height_map_bridge ${height_map_bridge_typesupport})
target_include_directories(height_map_bridge PRIVATE include)
idlc_generate(
  TARGET autonomy_light_dds_types
  FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/idl/HeightMap.idl"
)
target_link_libraries(height_map_bridge
  autonomy_light_dds_types
  CycloneDDS::ddsc
)

# The elevation path deliberately owns only the current observation.  Super-LIO
# remains the pose/TF provider; it is never used as an elevation-map backend.
add_executable(observation_merge
  src/observation_merge.cpp
  src/observation_merge_main.cpp
)
ament_target_dependencies(observation_merge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(observation_merge PRIVATE include)

add_executable(camera_frame_mapper
  src/camera_frame_mapper.cpp
  src/camera_frame_mapper_main.cpp
)
ament_target_dependencies(camera_frame_mapper
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(camera_frame_mapper PRIVATE include)

add_executable(gravity_frame_broadcaster
  src/gravity_frame_broadcaster.cpp
  src/gravity_frame_broadcaster_main.cpp
)
ament_target_dependencies(gravity_frame_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(gravity_frame_broadcaster PRIVATE include)

# LiDAR remains in the SLAM/navigation branch.  This node converts Super-LIO's
# registered cloud into the live occupancy map and sensor-frame obstacle clouds
# consumed by Nav2; it is intentionally not an elevation-map input.
add_executable(live_occupancy_mapper
  src/live_occupancy_mapper.cpp
)

add_executable(command_user_bridge
  src/command_user_bridge.cpp
)
ament_target_dependencies(command_user_bridge
  core
  geometry_msgs
  nav_msgs
  rclcpp
)

add_executable(outbound_command_bridge
  src/outbound_command_bridge.cpp
)
ament_target_dependencies(outbound_command_bridge
  core
  rclcpp
)
ament_target_dependencies(live_occupancy_mapper
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

add_executable(precise_elevation_mapping
  src/precise_elevation_mapping.cpp
  src/precise_elevation_mapping_main.cpp
)
ament_target_dependencies(precise_elevation_mapping
  geometry_msgs
  grid_map_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(precise_elevation_mapping PRIVATE include)

install(TARGETS
  height_map_bridge
  camera_frame_mapper
  gravity_frame_broadcaster
  observation_merge
  live_occupancy_mapper
  command_user_bridge
  outbound_command_bridge
  precise_elevation_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# The native map is reconstructed by this package from the current observation.
# The final fixed-grid height map is sent only as a direct DDS sample.
install(DIRECTORY config docs launch resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  build.sh
  launch.sh
  scripts/setup_nav2.sh
  scripts/nav2_wait_and_launch.sh
  scripts/setup_livox_driver.sh
  scripts/monitor_autonomy_state.py
  scripts/visualize_height_map.py
  scripts/generate_height_fov_mask.py
  scripts/show_realsense_usb_ports.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
