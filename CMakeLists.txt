cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(CycloneDDS QUIET CONFIG)

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
target_include_directories(height_map_bridge PRIVATE include)
if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(height_map_bridge "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(height_map_bridge ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

if(CycloneDDS_FOUND)
  idlc_generate(
    TARGET autonomy_light_dds_types
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/idl/HeightMap.idl"
  )
  target_compile_definitions(height_map_bridge PRIVATE AUTONOMY_LIGHT_HAS_CYCLONEDDS=1)
  target_link_libraries(height_map_bridge autonomy_light_dds_types CycloneDDS::ddsc)
else()
  message(STATUS "CycloneDDS not found: height_map_bridge supports ROS 2 only")
  target_compile_definitions(height_map_bridge PRIVATE AUTONOMY_LIGHT_HAS_CYCLONEDDS=0)
endif()

install(TARGETS height_map_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Elevation fusion remains in the vendored ETH/ANYbotics pipeline. This small
# bridge owns the stable autonomy_light height-map output contracts.
install(DIRECTORY config docs resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  build.sh
  launch.sh
  mapping.sh
  scripts/dds_network.py
  scripts/height_map_vis.py
  scripts/telemetry_vis.py
  scripts/setup_elevation_mapping.sh
  scripts/setup_livox_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
