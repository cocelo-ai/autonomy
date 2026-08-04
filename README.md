# cocelo-hd-autonomy-light

실기 elevation fusion은 `third_party/elevation_mapping`의 ETH/ANYbotics
robot-centric pipeline을 사용한다. `autonomy_light`는 bringup, calibrated TF,
그리고 controller 호환 `height_map_data` output/transport를 담당한다.

```text
Livox CustomMsg + IMU ──> Super-LIO ──> /lio/cloud_world ─┐
                       map -> odom -> imu, /lio/odom      │
D435/D435i PointCloud2 ───────────────────────────────────┤
                                                           v
                              elevation_mapping (vendored ETH pipeline)
                                      └─ /autonomy_light/elevation_map @ 50 Hz
                                         grid_map_msgs/msg/GridMap
                                                   │
                                                   └─ height_map_bridge @ 50 Hz
                                                      ├─ /autonomy_light/height_map_data (ROS 2)
                                                      └─ height_map (Cyclone DDS)

TF: map → odom → imu → base_link → lidar_link
                              └→ camera_link → camera_depth_optical_frame
```

자세한 노드·TF·topic 구조는 [software architecture PDF](docs/autonomy_light_software_architecture.pdf)에 있다.

## Build and run

```bash
./build.sh
source install/setup.bash
./launch.sh --real --mid360
```

`build.sh`는 Grid Map, PCL, TF 의존성과 Livox SDK를 설치한 뒤 벤더된
`kindr`, `kindr_ros`, `elevation_mapping`과 `livox_ros_driver2`, `super_lio`, bringup을
의존 순서에 맞게
빌드한다. 설치를 건너뛰려면 `--skip-apt`, Livox SDK만 건너뛰려면 `--skip-sdk`를 쓴다.

```bash
./launch.sh --vis                 # native GridMap elevation layer를 2D로 표시
./launch.sh --rviz                # GridMap RViz display
./launch.sh --map maps/site.pcd   # Super-LIO saved-map relocation
./mapping.sh --output maps/site.pcd
```

`mapping.sh`는 Super-LIO full SLAM(키프레임·loop closure·pose graph)을 켜고,
종료 시 최적화된 PCD를 저장한다. `--map`의 PCD는 Super-LIO 재지역화용이다.

설정 책임은 분리되어 있다. `config/autonomy_light.yaml`은 bringup의 frame,
calibration, Livox·Super-LIO 입력과 height-map output transport를 가진다.
`map`/`base_link` frame은 이 공통 설정에서 elevation mapper로 한 번만 전달한다.
`config/elevation_mapping.yaml`은 LiDAR/D435 입력, local-grid 크기·해상도·주기,
융합과 cleanup을 전부 가진다. 다른
mapper 설정은 `./launch.sh --elevation-config FILE`로 교체한다.
기본 출력 topic은 launcher remap이며, 필요하면
`AUTONOMY_LIGHT_ELEVATION_MAP_TOPIC=/my_map ./launch.sh`로 바꾼다.

## Native elevation-map interface

기본 output topic은 `/autonomy_light/elevation_map`, type은
`grid_map_msgs/msg/GridMap`이다. 기본 map은 `1.8 m × 0.8 m`, resolution `0.10 m`,
50 Hz publish다. mapper는 subscriber가 없어도 이 timer에서 fused map을 갱신·publish하며,
기본으로 시작되는 bridge가 구독하므로 native output도 계속 활성 상태다. 주요 layer는 다음과 같다.

| Layer | Meaning |
| --- | --- |
| `elevation` | map frame에서의 terrain height |
| `upper_bound`, `lower_bound` | 융합된 2σ height bound |
| `uncertainty_range` | `upper_bound - lower_bound` |

unknown cell은 `NaN`이다. 이를 평지나 임의의 `0.48` 값으로 대체하지 않는다.
컨트롤러는 `elevation`과 validity(`isfinite`) 및 `uncertainty_range`를 함께 사용해야 한다.

## Legacy height-map output

`height_map_bridge`는 native GridMap을 이전
`autonomy_light/msg/HeightMap` (`header`, `data`, `resolution`, `x_length`,
`y_length`)으로 50 Hz 재발행한다. `data`는 이전과 동일하게 row-major
`data[row * width + column]`이며, base yaw 기준으로 정렬된 `18 × 8` grid다.
Bridge는 근처 terrain의 20% percentile을 local floor로 잡고
`reference_height - relative_height`를 기록한다. unknown cell은 `0.48`이다.

기본 `transport: both`는 두 출력을 함께 보낸다.
`config/autonomy_light.yaml`의 `height_map_output.transport`로 `ros2`,
`cyclone_dds`, `both`를 선택한다. ROS 2 topic은
`/autonomy_light/height_map_data`이고, Cyclone DDS는 기존 IDL contract
`core_dds::HeightMap { sequence<float> data; }`와 `height_map` topic을 사용한다.
두 transport는 같은 배열을 같은 주기로 보낸다.

mapper는 Super-LIO `/lio/odom`의 6×6 pose covariance를 motion prediction과 point
variance에 반영한다. LiDAR와 D435는 서로 다른 sensor model로 바로 구독되므로, 기존의
동기화/병합 노드는 없다. D435가 꺼져 있으면 해당 구독은 유휴 상태이며 LiDAR만 융합한다.

`config/elevation_mapping.yaml`의 `underlying_map_topic`에 표준 `GridMap`(반드시 `elevation` layer 포함)을
지정하면 사전 구축 지형 지도를 underlying map으로 사용할 수 있다. PCD는 elevation-map
format이 아니므로 이 입력에는 사용할 수 없다. 즉 Super-LIO PCD는 localization용, GridMap은
terrain prior용으로 분리한다.

D435를 사용하지 않으려면 같은 파일의 `inputs`를 `['lidar']`로 바꾼다.

## TF and calibration

Super-LIO가 `map → odom → imu`를 발행한다. launcher는 설정의 calibrated extrinsic으로
`imu → base_link → lidar_link`와 `base_link → camera_link`를 `/tf_static`에 발행한다.
`target_to_lidar_*`, `target_to_camera_*`, `imu_from_lidar_*`는 반드시 실측 calibration 값이어야
하며, 후자는 Super-LIO에도 동일하게 전달된다. D435 driver의 camera-internal TF는
`camera_link`에 이어져야 한다. mapper는 gravity-aligned `map` frame을 사용하므로 별도
`base_link_gravity` TF나 자체 frame 변환 노드가 필요하지 않다.
