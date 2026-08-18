# cocelo-hd-autonomy-light

`launch.sh` starts one simple pipeline:

```text
Livox -> Super-LIO SLAM -> occupancy map -> Nav2
                                  | /nav2/cmd_vel (Twist)
                                  v
              /autonomy_light/command_out (core/CommandCore)
                                  |
                                  v
                     /control_command/autopilot (external ROS domain)

Depth camera -> camera_frame_mapper -> observation_merge -> elevation map
             -> height-map bridge -> DDS height_map
```

LiDAR is used only by Super-LIO and Nav2. On F4 the TF ownership is
`map → odom → lidar_link → base_link → base_link_gravity`, with the active
camera attached directly as `base_link → F_camera_link`. Super-LIO publishes
the first two dynamic edges at 50 Hz; `launch.sh` parses the URDF and publishes
only the required fixed sensor paths, and `gravity_frame_broadcaster` publishes the final
roll/pitch correction at 50 Hz. There is no robot model, joint-state stream,
or URDF publisher. Elevation mapping is camera-only. Each mapper projects
its depth cloud into `map` at the camera timestamp; merge then expresses it in
`base_link_gravity`, so robot tilt cannot appear as terrain-height jitter.
The global TF tree deliberately contains only those navigation edges and the
configured active sensor mounts.
RealSense optical-frame conversion is a mapper-local YAML calibration, not a
published `front_d435_*` TF branch.
With F4's one selected `F_camera_link` source, merge processes each frame
immediately—there is no time-cohort wait or cross-camera drop.  The elevation
grid processes every merged frame and emits its latest completed grid at 50 Hz.
The final height map is published only through DDS as `core_dds::HeightMap`
on DDS domain `1`, topic `height_map`.

All ROS processing runs on the private `internal_ros_domain_id` (default
`10`) in `config/elevation_mapping.yaml`. `outbound_command_bridge` relays
only `/autonomy_light/command_out` to `/control_command/autopilot` on
`external_ros_domain_id` (default `0`). The height map remains the only other
network output, sent directly as DDS on domain `1`. Nav2 costmaps and RViz remain private: start
them with `./launch.sh --rviz` or `ROS_DOMAIN_ID=10 rviz2` on the SBC.

## Run

```bash
./build.sh
./launch.sh
```

The three runtime configuration files are:

- `config/super_lio_mid360.yaml`
- `config/elevation_mapping.yaml`
- `config/nav2_live.yaml`

At runtime the terminal is a 2 Hz redraw dashboard: it shows Super-LIO pose,
roll/pitch/yaw, linear velocity, elevation-grid frame/valid-cell ratio, and
merge/elevation processing time.  Verbose node stdout/stderr is retained per
run under `log/runtime_*` instead of interleaving with that dashboard.

`/nav2/cmd_vel` remains Nav2's native `geometry_msgs/msg/Twist` topic. The
included `command_user_bridge` publishes it as `core/msg/CommandCore` on
the internal `/autonomy_light/command_out`; `outbound_command_bridge` is the
sole external publisher on `/control_command/autopilot`. `linear.x`,
`linear.y`, and `angular.z` map to
`motion_command[0]`, `[1]`, and `[2]`; the remaining slots and all event flags
are zero/false, and `custom_drive_id` is `2`.

The RViz global and local costmap displays subscribe to
`/global_costmap/costmap` and `/local_costmap/costmap`. These are composed
costmaps, so each contains its configured static/obstacle/inflation layers;
the inflation layer does not have a separate ROS topic. The terminal status
table reports Nav2 lifecycle states, costmap freshness, plan size, and the
last Nav2 velocity command.

`config/elevation_mapping.yaml` uses `observation_merge.inputs` as the camera
selection list. F4 currently selects only `front_d435`, whose explicit USB
port is `2-1.3` and whose static mount is `F_camera_link`. Each selected name
must have a same-key `bringup.realsense_cameras` entry and a `sources.<name>`
entry. The launcher verifies the driver topic, mapped topic, USB identity,
and static mount before it starts one mapper process for that camera.

`config/super_lio_mid360.yaml` is also the single source of truth for the
physical Livox transport: edit `bringup.ros__parameters.livox.model`,
`lidar_ip`, and `jetson_ip`.  `model` accepts `MID360` or `MID360S`; the
launcher generates the model-specific Livox-SDK2 JSON only at runtime and
checks that `jetson_ip` is configured on a local NIC.  One-run overrides do
not modify the YAML.

To add rear or side cameras, add its fixed mount to the URDF and a RealSense
definition, then add a source with `raw_topic`, mapped `topic`, and
`mount_frame` before including its name in `inputs`. More than one selected source uses the strict
cohort synchronizer; its policy is therefore isolated from one-camera F4
operation. LiDAR topics must not be added to `inputs`.

Useful options:

```bash
./launch.sh --check
./launch.sh --vis-rate 2.0       # clean SLAM/elevation terminal dashboard
./launch.sh --no-status          # raw node output remains in log/runtime_*/
./launch.sh --rviz
./launch.sh --no-nav2
./launch.sh --map maps/example.pcd
./launch.sh --mapping maps/new_map.pcd
./launch.sh --mapping
./launch.sh --lidar-type mid360s --lidar-ip 192.168.1.190 --jetson-ip 192.168.1.50
```

`--rviz` uses `map` as its fixed frame and includes Nav2's **Navigation 2**
panel, **Nav2 Goal** tool, and **2D Pose Estimate** tool. Use Nav2 Goal to
click and drag a target pose in the map; it publishes the Nav2 navigation goal.
The RViz configuration also displays `/map`, both composed Nav2 costmaps, and
their global/local paths. Inflation appears in the composed costmap displays,
because Nav2 does not publish an independent inflation-layer topic.
Global and local `published_footprint` polygons are overlaid in sky blue and
red respectively, so the footprint remains clear over the costmap gradients.
