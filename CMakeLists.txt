cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(CycloneDDS REQUIRED CONFIG)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "src/middleware/msg/HeightMap.msg"
)

# These IDL definitions are the stable, direct DDS contracts consumed by the
# control SBC. They are intentionally independent of the internal ROS graph.
idlc_generate(
  TARGET autonomy_light_external_contract_types
  FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/idl/ExternalContracts.idl"
)

rosidl_get_typesupport_target(autonomy_light_typesupport
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

function(configure_autonomy_target target)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE
      $<$<COMPILE_LANGUAGE:CXX>:-Wall>
      $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
      $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
    )
  endif()
endfunction()

# The production runtime has one process entry point. Individual responsibilities
# remain source/include modules and are assembled on one multithreaded executor.
add_executable(autonomy_light_node
  src/main.cpp
  src/algorithm/elevation/elevation_map.cpp
  src/algorithm/planner/live_occupancy_mapper.cpp
  src/middleware/command_user_bridge.cpp
  src/middleware/external_dds_bridge.cpp
  src/sensor/gravity_frame_broadcaster.cpp
)
configure_autonomy_target(autonomy_light_node)
target_include_directories(autonomy_light_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
ament_target_dependencies(autonomy_light_node
  action_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(autonomy_light_node
  autonomy_light_external_contract_types
  CycloneDDS::ddsc
  ${autonomy_light_typesupport}
)
# Some Humble/CMake combinations omit this edge from the typesupport target.
# Keep clean parallel builds deterministic.
add_dependencies(autonomy_light_node ${PROJECT_NAME}__cpp)

install(TARGETS autonomy_light_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY include/parameter/config include/parameter/models
  DESTINATION share/${PROJECT_NAME}/parameter
)
install(DIRECTORY scripts/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(PROGRAMS
  scripts/build.sh
  scripts/launch.sh
  scripts/setup_nav2.sh
  scripts/nav2_wait_and_launch.sh
  scripts/setup_livox_driver.sh
  scripts/goal_picker.py
  scripts/monitor_autonomy_state.py
  scripts/visualize_height_map_3d.py
  scripts/elevation_restorer.py
  scripts/find_livox_ip.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(external_contract_test
    src/test/external_contract_test.cpp
  )
  configure_autonomy_target(external_contract_test)
  target_link_libraries(external_contract_test PRIVATE
    autonomy_light_external_contract_types
    CycloneDDS::ddsc
  )
  add_test(NAME external_contract_test COMMAND external_contract_test)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
