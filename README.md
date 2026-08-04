# cocelo-autonomy-light

Super-LIO의 full LiDAR–IMU SLAM 출력을 받아 제어용 elevation map을 만드는 ROS 2
런타임이다. loop closure·pose graph·PCD 저장은 Super-LIO가 맡고,
`autonomy_light`는 높이 지도만 맡는다. D435를 켜면 별도 merge node가 registered
LiDAR cloud에 카메라 cloud를 합친다.

```text
Livox CustomMsg + IMU → Super-LIO ────────────┐
                                               ├─ rolling_cloud_merge (optional D435)
D435/D435i PointCloud2 ────────────────────────┘
                                               │
                                               ▼
                         autonomy_light
                         ├─ rolling | global elevation map
                         ├─ /autonomy_light/height_map_data
                         ├─ /autonomy_light/odom, path, TF
                         └─ /autonomy_light/live_map, PCD export
```

노드·토픽·TF 구조는 [software architecture PDF](docs/autonomy_light_software_architecture.pdf)에
있다.

## Build and run

```bash
./build.sh
source install/setup.bash
./launch.sh --real --mid360
```

`--sim`은 Livox `CustomMsg`와 IMU를 내보내는 시뮬레이터용이며, 기본 토픽은
`/f4/livox/lidar`, `/f4/livox/imu`다. 이미 Super-LIO를 실행 중이면
`./launch.sh --no-drivers`를 쓴다.

```bash
./launch.sh --real --map maps/site.pcd
```

저장 지도 모드에서는 Super-LIO `relocation_node`가 PCD를 직접 읽고 재지역화한다.
출력 global frame은 항상 `map`이다. 시작 직후에는 identity `map → odom`을 먼저
발행한다. 초기 NDT→ICP 정합 뒤에는 local LIO를 `odom`에서
유지하고, 저장 PCD와의 NDT→ICP 보정을 기본 1 Hz로만 갱신한다. 마지막 유효 보정은
LiDAR 출력마다 `map → odom → imu`로 계속 broadcast하므로 TF 체인이 끊기지 않는다.
`/lio/odom`과 `/lio/cloud_world`는 `map` 좌표를 사용한다.

`/lio/odom`의 child frame은 LiDAR가 아니라 `imu`다. 따라서
`target_to_lidar_*`와 `imu_from_lidar_*`를 모두 실제 캘리브레이션 값으로 설정해야
한다. 런타임이 Super-LIO를 시작할 때에는 후자를 `lio.extrinsic.lidar_imu`에도 같은
값으로 전달한다.

## Height-map source

```yaml
height_map:
  source: "rolling" # rolling | global
```

- `rolling`은 최근 관측만 확률적으로 융합한다. ground/upper surface, variance,
  age, validity를 유지하므로 제어와 sim-to-real 정책에는 이 모드를 권장한다.
- `global`은 누적·voxelized PCD에서 만든 영속 surface index를 조회한다. 미리 만든
  지도에서 재현 가능한 지형 기준이 필요할 때 사용한다.

두 모드 모두 `/autonomy_light/height_map_quality`의 `valid`, `variance`, `age`를
발행한다. `valid=0`인 `unknown` 값은 평지 관측이 아니므로 Isaac Lab 정책에서는
별도 observation으로 사용해야 한다.

Rolling 시작 시에는 LiDAR/D435가 가리는 `base_link` 아래 footprint에만 평지 prior를
넣는다. `initial_prior.ground_distance_m`만큼 아래의 지면을 가정하지만 분산은 기본
`0.04 m²`로 크게 둔다. 따라서 이 셀도 `valid=1`로 발행되지만 quality variance가
높으며, 첫 실제 관측은 outlier gate 없이 칼만 업데이트되어 prior를 즉시 대체한다.

### Robot-centric uncertainty

Rolling은 cloud timestamp와 일치하는 Super-LIO odometry만 사용한다. Super-LIO의
ESKF 6×6 pose covariance, LiDAR/D435 거리 모델, 각 센서 mounting calibration
uncertainty를 point별 height variance로 전파한다. XY pose uncertainty는 인접 cell에
Gaussian splat으로 나누고, 설정된 한계를 넘는 pose는 cloud 전체를 버린다. D435 merge
output은 `sensor_id`와 관측 원점을 보존하므로 두 센서가 같은 noise model·ray로 섞이지
않는다.

실기 로그의 residual로 `rolling_elevation.sensor.*`와 `localization.*`를 보정해야 한다.
기본값은 과신을 막는 보수적 시작값이며, covariance가 작아도 minimum standard
deviation 아래로는 낮추지 않는다.

`base_link_gravity`는 바닥 frame이 아니다. `base_link`와 같은 원점을 공유하고 roll/pitch만
제거한 yaw-only, gravity-aligned frame이다. 따라서 hanging obstacle이나 지면 추정 실패가 TF
translation을 바꾸지 않는다. Grid z는 이 frame에서의 실제 terrain z이며, `HeightMap.data`는
`base_z - terrain_z`로 발행한다.

### Visibility cleanup and overhang exclusion

Rolling 모드는 [Miki et al., IROS 2022](https://arxiv.org/abs/2204.12876)의 GPU elevation
mapping 방법을 CPU rolling posterior에 맞게 적용한다. 각 유효 측정의 sensor origin → endpoint ray가 오래된 surface보다 충분히 아래를
통과하고 표면 normal과 정렬되면 해당 stale cell을 제거한다. 따라서 움직인 물체나 이전
wall artifact는 다음 관측을 기다리지 않고 갱신된다. `rolling_elevation.visibility.*`가 이
조건을 조절한다.

동시에 `rolling_elevation.overhang_filter.*`는 센서보다 높은 point를 거리 의존 ramp
상한으로 사전 거절한다. 가까운 천장·돌출물은 ground fusion에 들어가지 않지만, 센서 아래의
일반 slope·계단은 유지된다. 이 두 처리는 관측 원점과 시간이 있는 `rolling`에만 적용한다.
정적 global PCD는 ray로 지우지 않는다.

### Live 2D debug view

```bash
./launch.sh --vis
```

`--vis`는 실제 `/autonomy_light/height_map_data`의 숫자 grid 한 장만 50 Hz로 표시한다.
GUI가 잠시 느려도 keep-last 1로 최신 frame만 그려 stale terrain을 표시하지 않는다. 창의
`source`와 `display` Hz가 모두 50에 가까운지 확인한다. `q` 또는 `Esc`로 viewer만
종료할 수 있으며, 사용자 topic은 `AUTONOMY_LIGHT_VIS_TOPIC`으로 지정한다.

### Optional D435/D435i merge

Use the standard realsense2_camera PointCloud2 output; the D435/D435i IMU is
not used by this node. Set height_map.source to rolling and
rolling_merge.enabled to true. The default camera topic is
/camera/camera/depth/color/points.

The launcher starts rolling_cloud_merge automatically. For each LiDAR cloud it
accepts the closest buffered D435 cloud inside max_sync_delta_sec, transforms it into the
LiDAR odom|map frame at the D435 timestamp, range/voxel filters it, and
publishes /autonomy_light/rolling_cloud. It also attaches the D435 world-frame
origin to each camera point; missing TF or an unmatched D435 sample passes the
LiDAR cloud through unchanged. A TF chain from base_link to the D435 optical
frame is required.

## Height-map transport

```yaml
height_map_output:
  transport: "ros2" # ros2 | cyclone_dds | both
  cyclone_dds:
    domain_id: 1
    topic: "height_map"
    type: "core_dds::HeightMap"
    history_depth: 1
```

`ros2`는 기존 ROS 2 `HeightMap`, quality, visualization topic 출력이다.
`cyclone_dds`는 다른 플랫폼을 위해 Cyclone DDS topic `height_map`에
`core_dds::HeightMap { sequence<float> data; }`만 직접 발행한다. payload의 순서,
셀 수와 unknown 값은 ROS `HeightMap.data`와 동일하다. `both`는 두 출력을 함께
발행하므로 플랫폼 전환 중 검증에 사용한다. DDS-only에서는 ROS height-map,
quality, visualization topic을 발행하지 않는다.

빌드에는 Cyclone DDS와 GTSAM이 필요하다. Ubuntu ROS 환경에서는 보통
`sudo apt install ros-$ROS_DISTRO-cyclonedds ros-$ROS_DISTRO-gtsam`으로 설치하며,
다른 DDS participant와 domain/network 설정을 맞추려면 동일한 `CYCLONEDDS_URI`를
사용한다.

## LiDAR rate

`livox_publish_freq: 50.0`은 LiDAR의 물리적 측정 속도가 아니라 드라이버가
수신 포인트를 하나의 ROS cloud로 묶어 발행하는 주기다. 따라서 50 Hz에서는 약
20 ms 길이의 더 작은 cloud가 Super-LIO로 들어가며, 새 elevation 관측의 지연도
줄어든다. 장치의 레이저 firing/scan 패턴과 초당 원시 포인트 수는 바뀌지 않는다.
CPU 여유와 Super-LIO 처리율은 `ros2 topic hz /lio/cloud_world`로 확인한다.

## Mapping

```bash
./mapping.sh --real --output maps/site.pcd
```

`mapping.sh`는 Super-LIO의 full SLAM 모드를 켠다. 1 m/10° keyframe, Scan Context
후보, GICP 검증, iSAM2 pose graph로 loop closure를 최적화하고 Ctrl+C에 loop-closed
PCD를 저장한다. 저장 PCD는 그대로 `--map`의 입력이 된다.

## Interfaces

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| input | `/lio/odom` | `nav_msgs/Odometry` | Super-LIO IMU pose + ESKF covariance |
| input | `/lio/cloud_world` | `sensor_msgs/PointCloud2` | global-frame registered cloud |
| optional input | D435 depth/color/points | `sensor_msgs/PointCloud2` | camera-depth cloud |
| intermediate | `/autonomy_light/rolling_cloud` | `sensor_msgs/PointCloud2` | time-aligned LiDAR+D435 cloud |
| output | `/autonomy_light/height_map_data` | `autonomy_light/HeightMap` | controller grid |
| output | `/autonomy_light/height_map_quality` | `autonomy_light/HeightMapQuality` | validity, variance, age |
| output | `/autonomy_light/height_map` | `sensor_msgs/PointCloud2` | visualization grid |
| output | `/autonomy_light/live_map` | `sensor_msgs/PointCloud2` | sparse voxel PCD |
| optional output | `height_map` | `core_dds::HeightMap` | direct Cyclone DDS custom payload |

모든 토픽은 `ros_domain_id` 하나를 사용한다. `world` frame은 사용하지 않으며,
normal mode·full-SLAM mapping·relocation 모두 `map`이 global frame이다. Normal LIO는
identity `map → odom`으로 시작하고, full SLAM·relocation만 보정값을 저주기로 바꾼다.

## Code layout

- `src/nodes/autonomy_light_node.cpp`, `autonomy_light_io.cpp`: elevation runtime node
- `src/nodes/rolling_cloud_merge_node.cpp`: optional one-D435 merge node
- `src/core/elevation_mapper.cpp`: ROS와 분리된 elevation algorithm
- `src/core/child_processes.cpp`: Livox/Super-LIO child process lifecycle

자체 `.hpp/.cpp` 파일은 모두 500줄 이하이며 CMake가 빌드 때 검증한다.
