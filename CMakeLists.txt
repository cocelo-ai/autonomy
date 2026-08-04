cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

find_package(ament_cmake REQUIRED)

# This package is deliberately bringup-only.  Elevation fusion lives in the
# vendored ETH/ANYbotics pipeline (third_party/elevation_mapping).
install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  build.sh
  launch.sh
  mapping.sh
  scripts/height_map_vis.py
  scripts/setup_elevation_mapping.sh
  scripts/setup_livox_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
