# autonomy_light interface

`autonomy_light`는 하나의 ROS 2 node다. Super-LIO의 global-frame point cloud와
LiDAR odometry를 소비하고, robot-centric elevation map을 발행한다.

| Direction | Topic | Type | Frame |
|---|---|---|---|
| input | `/lio/odom` | `nav_msgs/msg/Odometry` | `odom` or `map` → LiDAR |
| input | `/lio/cloud_world` | `sensor_msgs/msg/PointCloud2` | `odom` or `map` |
| output | `/autonomy_light/odom` | `nav_msgs/msg/Odometry` | global → `base_link` |
| output | `/autonomy_light/path` | `nav_msgs/msg/Path` | global |
| output | `/autonomy_light/height_map_data` | `autonomy_light/msg/HeightMap` | `base_link_gravity` |
| output | `/autonomy_light/height_map_quality` | `autonomy_light/msg/HeightMapQuality` | `base_link_gravity` |
| output | `/autonomy_light/height_map` | `sensor_msgs/msg/PointCloud2` | `base_link_gravity` |
| output | `/autonomy_light/live_map` | `sensor_msgs/msg/PointCloud2` | global |

TF is `odom|map → base_link → base_link_gravity`, plus static
`base_link → lidar_link`. A saved map selects Super-LIO `relocation_node` and
therefore uses `map`; normal operation uses `odom`. No `world` frame and no
locally generated `map → odom` correction exist.

`HeightMap.data` is `reference_height - terrain_height`, clamped by
`height_map_distance`. A quality mask marks unobserved cells explicitly:
`valid=0` means `data` contains only the configured unknown placeholder.
