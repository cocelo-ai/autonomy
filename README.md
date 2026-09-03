# cocelo-hd-autonomy-light

LiDAR-only ROS 2 Humble autonomy stack. Camera, RealSense, and observation-merge
paths are intentionally removed.

```text
Livox -> Super-LIO global keyframes (internal only)
      -> /lio/elevation_cloud (robot-local crop, 5 Hz)
      -> elevation_mapping (raw grid at 50 Hz)
      -> raw/restored selector (optional height-to-height ONNX)
      -> /autonomy_light/elevation_map (ROS domain 42)
      -> external_dds_bridge (Cyclone DDS, domain 1)
      -> height_map (core_dds::HeightMap, control SBC)

Nav2 /nav2/cmd_vel (geometry_msgs/msg/Twist, ROS domain 42)
      -> command_user_bridge -> /autonomy_light/autopilot_command (geometry_msgs/msg/Twist)
      -> external_dds_bridge (Cyclone DDS, domain 0)
      -> /control_command/autopilot (core/msg/CommandCore, control SBC)
```

Super-LIO retains the complete map as keyframes. It never continuously publishes
that heavy map. Instead, it reconstructs and publishes only a circular local crop
around the current robot pose on `/lio/elevation_cloud`. `elevation_mapping`
transforms that global-map crop into `base_link_gravity`, then samples the
configured robot-attached `grid.x_*` and `grid.y_*` bounds in
`include/parameter/config/elevation_mapping.yaml`. The grid follows robot position and yaw, while
roll and pitch do not tilt its vertical sampling direction.

Each pose-graph keyframe is a voxel-accumulated local surface, not just the
single LiDAR scan captured at the keyframe instant. Scans received between the
configured distance/yaw thresholds are transformed into the latest keyframe and
merged at `lio.slam.accumulation_interval_sec`. This preserves floor and sparse
Livox returns without creating a pose-graph node for every sensor frame.

## Repository layout

The C++ code follows the same domain-first layout as `autonomy_v2`. Public
interfaces live under `include/`, and implementation stays under the matching
`src/` domain. `src/main.cpp` is the sole process entry point: it assembles
the elevation, planner, middleware, and sensor modules on one multithreaded
ROS executor.

```text
include/
  algorithm/elevation/     # elevation-map interface
  algorithm/planner/       # Nav2 occupancy-map interface
  common/                  # shared ROS process runtime
  middleware/              # external DDS contract
  sensor/                  # gravity-frame node interface
src/
  main.cpp                 # the only executable entry point
  algorithm/elevation/     # map extraction
  algorithm/planner/       # Nav2 occupancy-map adapter
  middleware/              # command and external DDS bridges, IDL
  sensor/                  # gravity-frame broadcaster
```

## Run

The elevation-restoration node requires ONNX Runtime for Python. Install the
CPU runtime for the machine architecture before building/running:

```bash
/usr/bin/python3 -m pip install --user onnxruntime
```

```bash
./scripts/build.sh
./scripts/launch.sh
```

## Debian package

`scripts/package_deb.sh` creates a deployable Debian package in `dist/`. It stages a
fresh merged ROS install for `autonomy_light`, Super-LIO, and Livox Driver2,
includes Livox-SDK2 and the Nav2 runtime prefix, and declares the base ROS and
Ubuntu shared-library dependencies in the package metadata.

```bash
./scripts/package_deb.sh
sudo apt install ./dist/cocelo-autonomy-light_0.6.0-1_arm64.deb
autonomy-light --real
autonomy-light-doctor
```

Use `./scripts/package_deb.sh --no-nav2-bundle` only when Nav2 is already installed on
the target. The package is built for the current machine architecture and ROS
distribution; use `--ros-distro` or `--revision` to override those values.

Useful options:

```bash
./scripts/launch.sh --no-nav2
./scripts/launch.sh --map maps/saved_map
./scripts/launch.sh --mapping maps/new_map
./scripts/launch.sh --sim
./scripts/launch.sh --check
./scripts/launch.sh --no-status
```

`launch.sh` starts a terminal status view by default. It shows current
`/lio/odom` pose/velocity, internal elevation-map freshness, Nav2 lifecycle,
`/nav2/cmd_vel`, the staged autopilot command, and `/map`. Use `--status-rate 4` to refresh at
4 Hz, or `--no-status` when running without an interactive terminal.

Use `--vis-height` to add the numeric 18×8 HeightMap grid to this dashboard.
Values are shown directly and coloured from blue (smallest current distance)
through cyan/yellow to red (largest current distance). `--vis-height-3d`
remains the separate 3D point-cloud display.

For RViz-free Nav2 operation, use `./scripts/launch.sh --goal-picker`. This small
OpenCV window (not RViz) renders `/map`, the current robot pose, and `/plan`.
Left-clicking a light free-space cell publishes a `map`-frame `/goal_pose` to
Nav2. Unknown and occupied cells are rejected deliberately.

The local crop is configured in `include/parameter/config/super_lio_mid360.yaml`:

- `lio.elevation.radius`: radius of the small map-frame crop. It should cover
  the diagonal of the elevation grid plus a margin.
- `lio.elevation.leaf_size`: voxel size applied before that crop is published.
- `lio.elevation.publish_rate_hz`: lightweight crop update rate.

The external height-map topic is `height_map` on DDS domain 1. Its
legacy control-SBC type is exactly `core_dds::HeightMap` with one field,
`sequence<float> data`. `external_dds_bridge` subscribes only to the selected
internal ROS map, validates its geometry, and sends its values using that v2
DDS contract. Resolution and map lengths remain internal ROS metadata and are
not serialized to the control network. The internal debug PointCloud2 remains
`/autonomy_light/elevation_map/debug_points` on ROS domain 42.

The direct Cyclone DDS participant is pinned to the SBC Ethernet path
`192.168.20.2 -> 192.168.20.1` on `enxc0a125babe56`. It disables multicast and
uses the SBC as static discovery peers, including its raw domain-1 discovery
port `192.168.20.1:7650`, so the LiDAR NIC cannot be selected instead. These
values are configured as `bringup.external_dds.*` in
`include/parameter/config/elevation_mapping.yaml`; a caller-supplied
`CYCLONEDDS_URI` deliberately overrides the generated wired configuration.

Set `elevation_restorer.output_mode` in `include/parameter/config/elevation_mapping.yaml` to
`raw` or `restored`. In both modes the selected result uses the fixed internal
topic `/autonomy_light/elevation_map` and is sent to external-domain 1 as
`height_map` by the direct DDS egress; consumers keep the v2 topic
and type unchanged.
The mapper-to-selector
staging topic is `/autonomy_light/elevation_map/raw_input` and is not a public
output contract.

In `restored` mode, the network takes the 144 raw height values without
rescaling and returns 144 restored height values. Inference time is published
on `/autonomy_light/elevation_restorer/inference_ms`.
The selected grid is also converted to an internal-domain PointCloud2 on
`/autonomy_light/elevation_map/debug_points`. It uses cell-centre XY,
`z=-selected_distance`, and `intensity=selected_distance` in
`base_link_gravity`. Thus RViz also follows the configured raw/restored mode.

For pre-grid debugging, `/autonomy_light/elevation_map/input_points` publishes
the accumulated global-map samples after transformation into
`base_link_gravity` and target-relative input-Z filtering, but before HeightMap
ROI filtering, aggregation, or output clipping. Its robot-relative XYZ bounds are configured by
`debug.input_cloud.{x,y,z}_{min,max}` and default to `[-3, 3]` m. The topic uses
Reliable QoS on internal ROS domain 42 so it is directly visible in RViz.

`input_filter.target_relative_z.max` is applied before gridding. With the
default `-0.20`, points at or above 0.20 m below `output.target_link` are removed;
only points with `point_z - target_z < -0.20 m` remain.

The same filtered samples are rasterized into a 2.5D debug surface. Use
`/autonomy_light/elevation_map/input_surface_points` as a PointCloud2 with RViz
Squares, or add `/autonomy_light/elevation_map/input_surface_mesh` as a Marker
for the clearest colored surface. The mesh leaves unknown cells and sharp height
discontinuities open instead of drawing false terrain across them.

Each observed cell contains `target_link global z - surface global z` for the
highest supported surface in that robot-relative XY cell. A detached upper
cluster without `intra_cell_max_support_count` points is discarded as noise and
the next lower cluster is considered. This matches the Isaac Lab height-scanner
convention: ground below the target is positive, the grid rotates with robot
yaw, and roll/pitch do not tilt the vertical height rays. An unobserved cell
contains `output.unobserved_value` (normally `0.0`).

On the control SBC, subscribe with its existing v2 `core_dds::HeightMap` IDL:

```idl
module core_dds {
  struct HeightMap {
    sequence<float> data;
  };
};
```

`/autonomy_light/autopilot_command` is an internal `geometry_msgs/msg/Twist`
staging topic. Its only source is `/nav2/cmd_vel`; it retains `linear.x`,
`linear.y`, and `angular.z` after the goal gate and command timeout checks.
The direct domain-0 egress writes the SBC's ROS-compatible
`core::msg::dds_::CommandCore_` type to `rt/control_command/autopilot`, which
is the DDS mapping of SBC ROS topic `/control_command/autopilot`. It maps these
three values to `motion_command[0]`, `[1]`, and `[2]` respectively; all other
motion slots, event flags, and `custom_drive_id` remain zero. Non-finite or
non-float-representable commands are dropped before reaching the SBC.

## Control DDS contract

`height_map` is the legacy raw-DDS control-SBC contract on domain 1. It uses
Best-effort + Volatile QoS and retains 128 samples. The SBC ROS autopilot
contract remains on domain 0, using the ROS 2 graph-name mapping shown below
and Reliable + Volatile QoS with one retained sample.

| Direct DDS topic | DDS type | Source |
| --- | --- | --- |
| `height_map` (domain 1) | `core_dds::HeightMap` (`sequence<float> data`) | selected elevation map |
| `rt/control_command/autopilot` (domain 0) | `core::msg::dds_::CommandCore_` | `/nav2/cmd_vel` adapter |

## LiDAR network identity

The production default is `192.168.1.166` on `enP8p1s0`, verified on this
robot from a reachable `E4:7A:2C:*` Livox neighbour. `launch.sh` validates the
host address and route before starting the driver. On cloned images it calls
`scripts/find_livox_ip.py`, which chooses a single observed Livox neighbour;
an explicit `--lidar-ip` always takes precedence. The helper does not scan a
subnet or change NIC configuration.
