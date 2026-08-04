# Vendored source

Source: https://github.com/sanghyunryoo/Elevation_map.git

Pinned revision: `6fa657763be41f8587b39a8dc6bf2cb6660069de`.

Required ROS 2 dependencies are vendored at `../kindr` (ANYbotics/kindr,
`32800890d546e306f73c3ee3091fb6903452f925`) and `../kindr_ros`
(ANYbotics/kindr_ros, `8d60e3f8df5ddd8bcc58db3072edbb651f286b32`).

The source is an ROS 2 port of the ETH/ANYbotics robot-centric elevation
mapping pipeline. Local integration changes are deliberately small:

- add missing `nav_msgs` and `geometry_msgs` package dependencies;
- avoid null timer shutdown when optional timers are disabled;
- treat missing PointCloud2 `confidence_ratio` as neutral confidence so a D435
  point cloud is not assigned infinite variance;
- subscribe to `underlying_map_topic` after parameters are available;
- skip upstream helper/test executables in the production build.
