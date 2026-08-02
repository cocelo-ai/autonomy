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

No LIO estimator, registration, or map-update algorithm was changed. Local
integration also lives in `config/super_lio_mid360.yaml`, `build.sh`, and the
`autonomy_light` adapter.
