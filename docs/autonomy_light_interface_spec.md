# autonomy_light interface

`autonomy_light`는 elevation runtime node다. 선택적으로 별도
`rolling_cloud_merge` node가 Super-LIO global cloud와 한 대의 D435/D435i cloud를
합친 뒤 rolling elevation input으로 제공한다.

| Direction | Topic | Type | Frame |
|---|---|---|---|
| input | `/lio/odom` | `nav_msgs/msg/Odometry` | `odom` or `map` → LiDAR |
| input | `/lio/cloud_world` | `sensor_msgs/msg/PointCloud2` | `odom` or `map` |
| optional input | `/camera/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | D435 optical frame |
| intermediate | `/autonomy_light/rolling_cloud` | `sensor_msgs/msg/PointCloud2` | `odom` or `map` |
| output | `/autonomy_light/odom` | `nav_msgs/msg/Odometry` | global → `base_link` |
| output | `/autonomy_light/path` | `nav_msgs/msg/Path` | global |
| output | `/autonomy_light/height_map_data` | `autonomy_light/msg/HeightMap` | `base_link_gravity` |
| output | `/autonomy_light/height_map_quality` | `autonomy_light/msg/HeightMapQuality` | `base_link_gravity` |
| output | `/autonomy_light/height_map` | `sensor_msgs/msg/PointCloud2` | `base_link_gravity` |
| output | `/autonomy_light/live_map` | `sensor_msgs/msg/PointCloud2` | global |

`autonomy_light` publishes `odom|map → base_link → base_link_gravity`, plus
static `base_link → lidar_link`. A saved map selects Super-LIO
`relocation_node` and therefore uses `map`; normal operation uses `odom`. No
`world` frame exists. Mapping uses Super-LIO keyframes, Scan Context candidate
search, GICP verification and an iSAM2 pose graph; its corrected `map → odom →
imu` TF remains continuously broadcast. Saved-map localization keeps local LIO
in `odom` and refreshes the NDT→ICP `map → odom` correction at the configured
low rate. `/lio/odom` and `/lio/cloud_world` remain in the selected global
frame; this package does not duplicate the SLAM backend.

## D435 rolling merge

Set height_map.source to rolling and rolling_merge.enabled to true. The launcher
starts rolling_cloud_merge, which publishes rolling_merge.output_topic and the
elevation runtime subscribes to it. The node keeps a short D435 PointCloud2 buffer,
accepts it only inside max_sync_delta_sec of the LiDAR cloud, transforms it to
the LiDAR global frame at the D435 timestamp, applies range and voxel filters,
then appends it. Unavailable TF or an unmatched D435 sample results in a
LiDAR-only output. The D435 optical frame must be connected to base_link in TF.

`HeightMap.data` is `reference_height - terrain_height`, clamped by
`height_map_distance`. A quality mask marks unobserved cells explicitly:
`valid=0` means `data` contains only the configured unknown placeholder.

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
