cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features registration search kdtree)
find_package(PythonLibs REQUIRED)
find_package(OpenMP QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HeightMap.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

add_executable(autonomy_light_node
  src/autonomy_light.cpp
)

set_target_properties(autonomy_light_node
  PROPERTIES OUTPUT_NAME autonomy_light
)

ament_target_dependencies(autonomy_light_node
  ament_index_cpp
  geometry_msgs
  livox_ros_driver2
  nav_msgs
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
  ${PCL_LIBRARIES}
)

set(POINT_LIO_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/point_lio_ros2")
add_executable(autonomy_light_pointlio_mapping
  "${POINT_LIO_VENDOR_DIR}/src/laserMapping.cpp"
  "${POINT_LIO_VENDOR_DIR}/include/ikd-Tree/ikd_Tree.cpp"
  "${POINT_LIO_VENDOR_DIR}/src/parameters.cpp"
  "${POINT_LIO_VENDOR_DIR}/src/preprocess.cpp"
  "${POINT_LIO_VENDOR_DIR}/src/Estimator.cpp"
)

target_compile_definitions(autonomy_light_pointlio_mapping
  PRIVATE
    ROOT_DIR="${POINT_LIO_VENDOR_DIR}/"
)

target_compile_options(autonomy_light_pointlio_mapping
  PRIVATE
    -O3
    -pthread
    -fexceptions
)

target_include_directories(autonomy_light_pointlio_mapping
  PRIVATE
    "${POINT_LIO_VENDOR_DIR}/include"
    "${POINT_LIO_VENDOR_DIR}/src"
    ${EIGEN3_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
)

ament_target_dependencies(autonomy_light_pointlio_mapping
  geometry_msgs
  livox_ros_driver2
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

target_link_libraries(autonomy_light_pointlio_mapping
  ${PYTHON_LIBRARIES}
)

install(TARGETS
  autonomy_light_node
  autonomy_light_pointlio_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
