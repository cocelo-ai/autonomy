# cocelo-autonomy-light

Super-LIO의 LiDAR–IMU SLAM 출력을 받아 제어용 elevation map을 만드는 단일 ROS 2
노드 런타임이다. 이 패키지는 SLAM·loop closure·GICP를 다시 구현하지 않는다.
그 책임은 외부 Super-LIO에 두고, `autonomy_light`는 높이 지도와 PCD 저장만 맡는다.

```text
Livox CustomMsg + IMU → Super-LIO → /lio/odom, /lio/cloud_world
                                          │
                                          ▼
                         autonomy_light (one ROS node)
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
출력 global frame은 `map`이다. 이 패키지에는 별도 FPFH/GICP 초기화나 `map → odom`
보정이 없으므로 이중 변환이 생기지 않는다.

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

빌드에는 Cyclone DDS가 필요하다. Ubuntu ROS 환경에서는 보통
`sudo apt install ros-$ROS_DISTRO-cyclonedds`로 설치하며, 다른 DDS participant와
domain/network 설정을 맞추려면 동일한 `CYCLONEDDS_URI`를 사용한다.

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

`/lio/cloud_world`를 sparse voxel PCD로 축적하고 Ctrl+C 때 저장한다. 저장 PCD는
그대로 `--map`의 입력이 된다. local pose graph는 의도적으로 제공하지 않는다.

## Interfaces

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| input | `/lio/odom` | `nav_msgs/Odometry` | Super-LIO LiDAR pose |
| input | `/lio/cloud_world` | `sensor_msgs/PointCloud2` | global-frame registered cloud |
| output | `/autonomy_light/height_map_data` | `autonomy_light/HeightMap` | controller grid |
| output | `/autonomy_light/height_map_quality` | `autonomy_light/HeightMapQuality` | validity, variance, age |
| output | `/autonomy_light/height_map` | `sensor_msgs/PointCloud2` | visualization grid |
| output | `/autonomy_light/live_map` | `sensor_msgs/PointCloud2` | sparse voxel PCD |
| optional output | `height_map` | `core_dds::HeightMap` | direct Cyclone DDS custom payload |

모든 토픽은 `ros_domain_id` 하나를 사용한다. `world` frame은 사용하지 않으며,
normal mode는 `odom`, relocation mode는 `map`이 global frame이다.

## Code layout

- `src/nodes/autonomy_light_node.cpp`, `autonomy_light_io.cpp`: 하나의 ROS node
- `src/core/elevation_mapper.cpp`: ROS와 분리된 elevation algorithm
- `src/core/child_processes.cpp`: Livox/Super-LIO child process lifecycle

자체 `.hpp/.cpp` 파일은 모두 500줄 이하이며 CMake가 빌드 때 검증한다.
