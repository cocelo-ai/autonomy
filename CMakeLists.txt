cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CycloneDDS REQUIRED CONFIG)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HeightMap.msg"
  "msg/HeightMapQuality.msg"
  DEPENDENCIES std_msgs
)

idlc_generate(
  TARGET autonomy_light_dds_types
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/idl/HeightMap.idl"
)

ament_export_dependencies(rosidl_default_runtime)

file(GLOB_RECURSE AUTONOMY_LIGHT_NODE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/autonomy_light_*.cpp"
)

add_executable(autonomy_light_node src/autonomy_light.cpp ${AUTONOMY_LIGHT_NODE_SOURCES})
add_executable(rolling_cloud_merge_node
  src/rolling_cloud_merge_main.cpp
  src/nodes/rolling_cloud_merge_node.cpp
)

# Keep every owned C++ source/header below 500 lines.  The files are ordinary
# .hpp/.cpp units; implementation fragments are intentionally not included.
add_custom_target(autonomy_light_source_line_limit ALL
  COMMAND ${CMAKE_COMMAND}
    -DROOT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src
    -DMAX_LINES=500
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_source_line_limit.cmake
  COMMAND ${CMAKE_COMMAND}
    -DROOT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/include
    -DMAX_LINES=500
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_source_line_limit.cmake
  COMMENT "Checking autonomy-light source-unit line limits"
)
add_dependencies(autonomy_light_node autonomy_light_source_line_limit)
add_dependencies(rolling_cloud_merge_node autonomy_light_source_line_limit)

set_target_properties(autonomy_light_node
  PROPERTIES OUTPUT_NAME autonomy_light
)

ament_target_dependencies(autonomy_light_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_target_dependencies(rolling_cloud_merge_node
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_include_directories(autonomy_light_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_include_directories(rolling_cloud_merge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(autonomy_light_node "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(autonomy_light_node
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
  )
endif()

target_link_libraries(autonomy_light_node
  autonomy_light_dds_types
  CycloneDDS::ddsc
  ${PCL_LIBRARIES}
)
target_link_libraries(rolling_cloud_merge_node ${PCL_LIBRARIES})

install(TARGETS autonomy_light_node rolling_cloud_merge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  build.sh
  launch.sh
  mapping.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/find_mid360_ip.py
  scripts/height_map_vis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/setup_livox_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
