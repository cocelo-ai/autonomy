# autonomy_light interface

`autonomy_light`는 elevation runtime node다. 선택적으로 별도
`rolling_cloud_merge` node가 Super-LIO global cloud와 한 대의 D435/D435i cloud를
합친 뒤 rolling elevation input으로 제공한다.

| Direction | Topic | Type | Frame |
|---|---|---|---|
| input | `/lio/odom` | `nav_msgs/msg/Odometry` | global → IMU, calibrated 6×6 pose covariance |
| input | `/lio/cloud_world` | `sensor_msgs/msg/PointCloud2` | `map` |
| optional input | `/camera/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | D435 optical frame |
| intermediate | `/autonomy_light/rolling_cloud` | `sensor_msgs/msg/PointCloud2` | `map` |
| output | `/autonomy_light/odom` | `nav_msgs/msg/Odometry` | global → `base_link` |
| output | `/autonomy_light/path` | `nav_msgs/msg/Path` | global |
| output | `/autonomy_light/height_map_data` | `autonomy_light/msg/HeightMap` | `base_link_gravity` |
| output | `/autonomy_light/height_map_quality` | `autonomy_light/msg/HeightMapQuality` | `base_link_gravity` |
| output | `/autonomy_light/height_map` | `sensor_msgs/msg/PointCloud2` | `base_link_gravity` |
| output | `/autonomy_light/live_map` | `sensor_msgs/msg/PointCloud2` | global |

The resulting TF chain is `map → odom → imu → base_link → base_link_gravity`, plus
static `base_link → lidar_link`. Super-LIO publishes identity `map → odom` before
the first LiDAR update, then updates only that correction for full SLAM or
relocation; it owns dynamic `odom → imu`. `autonomy_light` publishes static
`imu → base_link` and `base_link → lidar_link`. No `world` frame exists.
Before the first odometry, `autonomy_light` publishes identity
`base_link → base_link_gravity`. Once odometry exists, only the dynamic
roll/pitch-removing rotation is updated; its translation always remains zero.
Mapping uses Super-LIO keyframes, Scan Context candidate
search, GICP verification and an iSAM2 pose graph; its corrected `map → odom →
imu` TF remains continuously broadcast. Saved-map localization keeps local LIO
in `odom` and refreshes the NDT→ICP `map → odom` correction at the configured
low rate. `/lio/odom` and `/lio/cloud_world` remain in the selected global
frame; this package does not duplicate the SLAM backend.

`/lio/odom` is the Super-LIO IMU state, not a LiDAR pose. The runtime derives
`T_base_imu = T_base_lidar · inverse(T_imu_lidar)` from
`target_to_lidar_*` and `imu_from_lidar_*`, then converts every odometry sample
with `T_global_base = T_global_imu · inverse(T_base_imu)`. Its managed
Super-LIO command receives the exact same `T_imu_lidar`; when Super-LIO is
started externally, set the matching `lio.extrinsic.lidar_imu` yourself.

## D435 rolling merge

Set height_map.source to rolling and rolling_merge.enabled to true. The launcher
starts rolling_cloud_merge, which publishes rolling_merge.output_topic and the
elevation runtime subscribes to it. The node keeps a short D435 PointCloud2 buffer,
accepts it only inside max_sync_delta_sec of the LiDAR cloud, transforms it to
the LiDAR global frame at the D435 timestamp, applies range and voxel filters,
then appends it. Unavailable TF or an unmatched D435 sample results in a
LiDAR-only output. The D435 optical frame must be connected to base_link in TF.

## Robot-centric uncertainty fusion

Super-LIO exports ESKF covariance in global-frame `[x, y, z, roll, pitch, yaw]`
order. Each cloud uses its nearest odom inside
`rolling_elevation.localization.odom_sync_tolerance_sec`; unmatched clouds are
discarded. The mapper adds sensor range and mounting uncertainty to the
Jacobian-projected pose covariance for every height update. XY uncertainty is a
bounded Gaussian splat across nearby cells, while a pose above the configured
limit rejects the cloud. The merge node preserves `sensor_id` and the D435
world-frame origin, keeping LiDAR and D435 models separate after fusion and
giving visibility cleanup the correct ray source.

`HeightMap.data` is the actual `base_z - terrain_z`, clamped by
`height_map_distance`. A quality mask marks unobserved cells explicitly:
`valid=0` means `data` contains only the configured unknown placeholder.

At startup, `rolling_elevation.initial_prior` seeds only the robot footprint with
a high-variance ground posterior. It publishes as `valid=1` so the controller
receives a finite initial value, while its quality variance explicitly marks it
as low confidence. The first real observation bypasses the innovation gate and
replaces that prior through the usual Kalman update.

## Live 2D debug view

`./launch.sh --vis` starts a live OpenCV view of the numeric `HeightMap.data`
grid only. The monitor keeps just the newest best-effort sample and targets 50
Hz, so it does not display a queued, stale terrain frame after robot motion.
The overlay reports both received `source` Hz and rendered `display` Hz.
Override the input with `AUTONOMY_LIGHT_VIS_TOPIC`.

## Gravity-aligned base frame

`base_link_gravity` has exactly the same origin as `base_link`. Its rotation is
the base orientation with roll and pitch removed, so it remains yaw-only and
gravity-aligned. The terrain PointCloud z coordinate is therefore directly
relative to the base origin; no terrain estimate translates this TF.

## Rolling visibility and overhang handling

`rolling_elevation.visibility` runs per accepted cloud. A ray from the exact
LiDAR/D435 source to every accepted endpoint invalidates an old ground/upper
cell only when it passes sufficiently below that cell, the cell is old enough,
and the ray aligns with the estimated surface normal. This prevents cleanup at
grazing edges while removing stale dynamic walls. `overhang_filter` applies the
paper's distance-ramped upper limit above the sensor before Bayesian fusion.
Both features are intentionally disabled for the static `global` PCD source.

## Cyclone DDS transport

Set `height_map_output.transport` to `cyclone_dds` for a direct DDS output, or
`both` to retain the ROS 2 outputs during migration. The installed IDL is
`resources/idl/HeightMap.idl` and defines exactly:

```idl
module core_dds { struct HeightMap { sequence<float> data; }; };
```

The configurable default is domain `1`, topic `height_map`, type
`core_dds::HeightMap`, best-effort keep-last history depth `1`. Its sole field
has the same row-major cell order, length and unknown convention as ROS
`HeightMap.data`; it intentionally carries no header, geometry or quality
channels. Consumers must share the grid configuration and read the ROS quality
stream when that metadata is required.
