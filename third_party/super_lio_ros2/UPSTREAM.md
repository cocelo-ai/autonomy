# Super-LIO source provenance

- Upstream: <https://github.com/Liansheng-Wang/Super-LIO>
- Imported branch: `ros2`
- Imported commit: `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2`
- License: GNU GPL version 3 (see `LICENSE`)

`basic/` is an unmodified upstream source snapshot. `super_lio/` has a minimal
local ROS 2 integration patch:

- `lio.ros.global_frame` makes published odometry, paths, clouds, and TF use
  `odom` for normal operation or `map` for relocation instead of a fixed
  upstream frame.
- absolute `lio.map.save_map_dir` paths are preserved so `relocation_node` can
  load the user-selected PCD without copying it under Super-LIO's source tree.
- `pose_graph.hpp/.cpp` adds full mapping SLAM: keyframes, a compact Scan
  Context descriptor, GICP loop verification and iSAM2 pose-graph optimization.
  The optimized PCD is written by Super-LIO when mapping stops.
- mapping and saved-map relocation continuously broadcast `map → odom → imu`.
  The mapping correction is updated after pose-graph optimization; relocation
  refreshes cropped NDT→ICP against the saved PCD at a configurable low rate.
  `lio.ros.odom_frame` configures the middle frame.
- ROS transport is split into `ROSWrapper_params.cpp`, `ROSWrapper_input.cpp`,
  and `ROSWrapper.cpp` (output/TF), keeping each implementation unit below 500
  lines without adding ROS nodes.

The ESKF front-end remains unchanged. Global constraints are isolated in the
new backend; `autonomy_light` does not implement a second SLAM pipeline.
Local integration also lives in `config/super_lio_mid360.yaml`, `build.sh`,
and the `autonomy_light` adapter.
