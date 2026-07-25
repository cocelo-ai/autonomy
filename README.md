# cocelo-hd-autonomy-light

Lightweight Jetson runtime for MID360/MID360s + Point-LIO based robot-centric
height map publishing.

The package is intended to run on the perception Jetson. Heavy LiDAR and
mapping topics stay on the internal ROS 2 domain, while control-facing outputs
are republished on the external ROS 2 domain for the control SBC.

## Quick Setup

This repository vendors `livox_ros_driver2` under `third_party/livox_ros_driver2`.
On a new Jetson, prepare the Livox SDK, link the vendored driver into the ROS
workspace, and build it with:

```bash
cd ~/ros2_ws/src/cocelo-hd-autonomy-light
./build.sh
```

If the Jetson already has Livox-SDK2 installed:

```bash
./build.sh --skip-sdk
```

The script creates:

```text
~/ros2_ws/src/livox_ros_driver2 -> ~/ros2_ws/src/cocelo-hd-autonomy-light/third_party/livox_ros_driver2
```

Then run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
./launch.sh --real --mid360
./launch.sh --real --mid360 --vis
# or
./launch.sh --real --mid360s
```

## Saved-map relocalization

Create a PCD with `./mapping.sh`, then start against it with:

```bash
./launch.sh --real --map maps/point_lio_map_YYYYMMDD_HHMMSS.pcd
```

The first 2 seconds of registered Point-LIO scans are accumulated into an odom
submap, globally matched to the PCD with FPFH/RANSAC, and refined with GICP.
Accepted transforms correct the published odometry into `saved_map_frame`
(default: `odom`); GICP then tracks the map pose every 0.5 seconds. Height maps
are withheld until relocalization succeeds and are extracted from the saved PCD
around that corrected pose. Check
`/autonomy_light/heartbeat` for `waiting_for_saved_map_relocalization` or a
`ready:...:relocalization_fitness=...` state.

During initialization, heartbeat reports the active phase and accumulated-point
count, for example `relocalizing:phase=collecting_submap:submap_points=...`.
The console also logs collection, global FPFH/RANSAC matching, and GICP
refinement. The saved PCD is published once with transient-local QoS on
`/autonomy_light/saved_map` in `saved_map_frame`; use that topic in RViz with
the external `ROS_DOMAIN_ID` (the default configuration uses `17`) and set the
PointCloud2 display durability to `Transient Local` when opening RViz after
autonomy-light has already started.

The accumulated live Point-LIO global map is always enabled during a normal
`launch.sh` run and is the only live source for height-map construction.  Its
visualization domain is independent of the control-output domain:

```bash
ROS_DOMAIN_ID=17 rviz2
```

Set `point_lio_global_map.ros_domain_id` in `config/autonomy_light.yaml` to the
same ID used by RViz. It publishes `/point_lio/global_map` with transient-local
QoS at 1 Hz by default, so RViz can display the current accumulated map even if
it starts later. The visual map is voxelized at 5 cm; the separate 1 cm global
terrain store is used for height-map extraction and is not downsampled to the
visualization resolution. Saved PCD operation continues to use that saved global
map, after relocalization, because it is the highest-fidelity global source.

`/point_lio/global_map_refined` is the precision map used by the live height-map
pipeline. Each registered scan receives statistical isolated-point rejection,
then centimetre-level sparse-voxel surfel fusion; this keeps sharp stair edges
because no moving-least-squares fit is allowed across a height discontinuity.
Use this topic in RViz to inspect the terrain input. `refinement.mean_k` and
`refinement.stddev_multiplier` control only isolated-point rejection.

Global scan matching needs stable 3D geometry. Repetitive corridors, featureless
floors, or a map that was made with different sensor extrinsics can produce no
acceptable match; the quality gates deliberately keep height-map output stopped
in that case. Tune `saved_map_localization` in `config/autonomy_light.yaml` for
the site rather than loosening `max_fitness` blindly.

## Debian Runtime Package

For delivery to another Jetson, build a runtime `.deb` instead of handing over
the source tree:

```bash
cd ~/ros2_ws/src/cocelo-hd-autonomy-light
./scripts/package_deb.sh --mid360
# or
./scripts/package_deb.sh --mid360s
```

This creates a package under `dist/` for the current machine architecture.
On a Jetson it creates `arm64`; on an x86_64 workstation it creates `amd64`.
For example:

```text
dist/cocelo-autonomy-light_0.1.0-1_arm64.deb
dist/cocelo-autonomy-light_0.1.0-1_amd64.deb
```

Install and run on a target with the same architecture:

```bash
sudo apt update
sudo apt install "./dist/cocelo-autonomy-light_0.1.0-1_$(dpkg --print-architecture).deb"
sudo nano /etc/cocelo/autonomy-light/autonomy_light.yaml
autonomy-light --real
```

The package installs the binary runtime under `/opt/cocelo/autonomy-light`, the
editable runtime config under `/etc/cocelo/autonomy-light/autonomy_light.yaml`,
and receiver documentation/examples under `/usr/share/doc/cocelo-autonomy-light`.
The target Jetson is assumed to already have ROS 2 Humble installed under
`/opt/ros/humble`. The `.deb` bundles autonomy-light-specific runtime artifacts:
the autonomy-light binaries, generated `HeightMap` message interface, vendored
Livox ROS driver2 install tree, Point-LIO runtime, docs, examples, and the
Livox-SDK2 shared library when it is present at build time.
Run the ROS runtime as the normal login user. The launcher uses `sudo` only for
the Livox MID360/MID360s network interface setup when needed.

Basic checks:

```bash
autonomy-light-doctor
ROS_DOMAIN_ID=0 autonomy-light-heightmap-example
```

Any ROS 2 host that subscribes to `autonomy_light/msg/HeightMap` must have this
package installed and sourced, or must build an equivalent message interface:

```bash
source /opt/ros/humble/setup.bash
source /opt/cocelo/autonomy-light/install/setup.bash
ROS_DOMAIN_ID=0 ros2 interface show autonomy_light/msg/HeightMap
```

## ROS 2 Domains

Default domain split:

```yaml
internal_ros_domain_id: 42
external_ros_domain_id: 0
point_lio_global_map:
  ros_domain_id: 17
```

- Internal domain `42`: Livox driver, Point-LIO, odometry input, and registered scans.
- External domain `0`: control-facing outputs only.
- Global-map debug domain `17`: `/point_lio/global_map` and
  `/point_lio/global_map_refined` for RViz.

Check the external control output:

```bash
ROS_DOMAIN_ID=0 ros2 topic list | grep autonomy_light
```

## Control-Facing Outputs

These topics are published on `external_ros_domain_id`.

| Topic | Type | Purpose |
|---|---|---|
| `/autonomy_light/height_map_data` | `autonomy_light/msg/HeightMap` | Compact sim-to-real height map for control. |
| `/autonomy_light/height_map` | `sensor_msgs/msg/PointCloud2` | Debug point cloud generated from the height grid. |
| `/autonomy_light/odom` | `nav_msgs/msg/Odometry` | Point-LIO odometry remapped to the configured target frame. |
| `/path` | `nav_msgs/msg/Path` | Point-LIO path republished for consumers that need it. |
| `/tf`, `/tf_static` | `tf2_msgs` | `odom -> base_link`, `odom -> base_link_gravity`, and static LiDAR transform. |

The custom height map message is:

```ros
std_msgs/Header header
float32[] data
float32 resolution
float32 x_length
float32 y_length
```

`header.stamp` is the map publication time and `header.frame_id` is the
height-map frame. `data` is row-major and matches the autonomy DDS
`HeightMap.idl` float-array contract:

```text
index = row * width + col
width = ceil(x_length / resolution)
height = ceil(y_length / resolution)
row: y_min -> y_max
col: x_min -> x_max
```

The value is published in Isaac/autonomy height-scanner style:

```text
data[index] = base_height - grid_z
```

With the default `clipping.max_z: 0.48`, flat ground is near `0.48` and
obstacles produce smaller distance values.

For receiver/control debugging without Point-LIO input, enable the manual
height map mode in `/etc/cocelo/autonomy-light/autonomy_light.yaml`:

```yaml
height_map_debug:
  manual_mode: true
  manual_value: 0.48
```

When enabled, `/autonomy_light/height_map_data.data` is filled directly with
`manual_value` at `publish_rate_hz`. Restart `autonomy-light --real` after
editing the YAML. To test only the receiver path without starting Livox or
Point-LIO, run:

```bash
autonomy-light --real --no-drivers
```

The receiving SBC must have this package built and sourced so the custom
message type is available. When using the `.deb`, source the packaged install
tree:

```bash
source /opt/ros/humble/setup.bash
source /opt/cocelo/autonomy-light/install/setup.bash
ROS_DOMAIN_ID=0 ros2 interface show autonomy_light/msg/HeightMap
ROS_DOMAIN_ID=0 ros2 topic echo /autonomy_light/height_map_data
```

Recommended receiver QoS for `/autonomy_light/height_map_data`:

```text
Reliability: Reliable
Durability: Volatile
History: KeepLast
Depth: 2
```

## Internal Inputs

These topics normally stay on `internal_ros_domain_id`.

| Topic | Type | Producer | Purpose |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox ROS Driver2 | MID360/MID360s raw LiDAR input to Point-LIO. |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Livox ROS Driver2 | MID360/MID360s internal IMU input to Point-LIO. |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | Point-LIO | Mapping odometry consumed by autonomy-light. |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Point-LIO | Registered scans fused into `/point_lio/global_map_refined`. |
| `/point_lio/global_map_refined` | `sensor_msgs/msg/PointCloud2` | autonomy-light | Edge-preserving refined global map used for live height extraction. |

## Key Parameters

Commonly changed parameters in `config/autonomy_light.yaml`:

| Parameter | Default | Meaning |
|---|---:|---|
| `target_frame` | `base_link` | Robot control frame. |
| `height_map_frame` | `base_link_gravity` | Yaw-only gravity-aligned frame used by the height map. |
| `elevation_resolution` | `0.05` | Height grid cell size in meters. |
| `elevation_x_length` / `elevation_y_length` | `1.2` / `1.2` | Robot-centric grid size in meters. |
| `publish_rate_hz` | `50.0` | External output publish rate. |
| `livox_model` | `mid360` | Livox model for generated driver config. Override with `--mid360` or `--mid360s`. |
| `algorithm.elevation_backend` | `autonomy_min_z` | Cell selector matching the autonomy min-z style. |
| `algorithm.clipping.min_z` / `max_z` | `0.0` / `0.48` | Output height clamp range. |
| `algorithm.min_z.min_points_per_cell` | `3` | Minimum support for the smooth terrain layer. |
| `algorithm.min_z.obstacle_override_enabled` | `true` | Overlays sparse supported obstacle clusters after terrain filtering. |
| `algorithm.isolated_filter.*` | see config | Removes isolated random cell noise. |

Full input/output and tuning documentation:

- `docs/autonomy_light_interface_spec.md`
- `docs/autonomy_light_tuning_spec.md`
