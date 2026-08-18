cmake_minimum_required(VERSION 3.8)

project(core)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EventCore.msg"
  "msg/CommandCore.msg"
  "msg/EventUser.msg"
  "msg/CommandUser.msg"
  "msg/RobotReport.msg"
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
