#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./launch.sh [--real|--sim] [--no-drivers] [--map FILE] [options] [-- <elevation ROS args>]

  --real                  Start Livox and Super-LIO (default).
  --sim                   Use Livox CustomMsg simulation input; do not start Livox.
  --no-drivers            Use externally running Super-LIO and sensor drivers.
  --map FILE              Start Super-LIO relocation against a saved PCD.
  --mapping-output FILE   Enable Super-LIO full SLAM and save its PCD on exit.
  --config FILE           Override autonomy_light.yaml.
  --elevation-config FILE Override elevation_mapping.yaml.
  --raw-lidar-topic TOPIC Override the Super-LIO LiDAR input topic.
  --raw-imu-topic TOPIC   Override the Super-LIO IMU input topic.
  --sim-topic-prefix PFX  Simulation prefix (default: /f4).
  --mid360|--mid360s      Select the bundled Livox launch file.
  --vis                   Show the native GridMap elevation layer in a 2D window.
  --rviz                  Start RViz with the GridMap display.
  --no-rviz               Do not auto-start RViz for --map.
  --no-static-tf          Do not publish imu -> base_link -> lidar_link.
  --ros-distro NAME       ROS distribution (default: $ROS_DISTRO or humble).

Extra ROS arguments after `--` are passed only to elevation_mapping.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="real"
NO_DRIVERS="false"
NO_STATIC_TF="false"
CONFIG_FILE="${AUTONOMY_LIGHT_CONFIG:-${SCRIPT_DIR}/config/autonomy_light.yaml}"
ELEVATION_CONFIG_FILE="${AUTONOMY_LIGHT_ELEVATION_CONFIG:-${SCRIPT_DIR}/config/elevation_mapping.yaml}"
MAP_FILE="${AUTONOMY_LIGHT_MAP_FILE:-}"
MAPPING_OUTPUT=""
RAW_LIDAR_TOPIC=""
RAW_IMU_TOPIC=""
SIM_TOPIC_PREFIX="${AUTONOMY_LIGHT_SIM_TOPIC_PREFIX:-/f4}"
LIVOX_MODEL="${AUTONOMY_LIGHT_LIVOX_MODEL:-mid360}"
VIS="false"
RVIZ="auto"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
VIS_TOPIC="${AUTONOMY_LIGHT_VIS_TOPIC:-}"
VIS_FPS="${AUTONOMY_LIGHT_VIS_FPS:-50}"
VIS_SCALE="${AUTONOMY_LIGHT_VIS_SCALE:-48}"
MAP_TOPIC="${AUTONOMY_LIGHT_ELEVATION_MAP_TOPIC:-/autonomy_light/elevation_map}"
declare -a EXTRA_ELEVATION_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --real) MODE="real"; shift ;;
    --sim) MODE="sim"; shift ;;
    --no-drivers) NO_DRIVERS="true"; shift ;;
    --no-static-tf) NO_STATIC_TF="true"; shift ;;
    --vis) VIS="true"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --config) CONFIG_FILE="${2:?--config requires a file}"; shift 2 ;;
    --config=*) CONFIG_FILE="${1#*=}"; shift ;;
    --elevation-config) ELEVATION_CONFIG_FILE="${2:?--elevation-config requires a file}"; shift 2 ;;
    --elevation-config=*) ELEVATION_CONFIG_FILE="${1#*=}"; shift ;;
    --map|--map-file) MAP_FILE="${2:?--map requires a PCD}"; shift 2 ;;
    --map=*|--map-file=*) MAP_FILE="${1#*=}"; shift ;;
    --mapping-output) MAPPING_OUTPUT="${2:?--mapping-output requires a PCD path}"; shift 2 ;;
    --mapping-output=*) MAPPING_OUTPUT="${1#*=}"; shift ;;
    --raw-lidar-topic) RAW_LIDAR_TOPIC="${2:?--raw-lidar-topic requires a topic}"; shift 2 ;;
    --raw-lidar-topic=*) RAW_LIDAR_TOPIC="${1#*=}"; shift ;;
    --raw-imu-topic) RAW_IMU_TOPIC="${2:?--raw-imu-topic requires a topic}"; shift 2 ;;
    --raw-imu-topic=*) RAW_IMU_TOPIC="${1#*=}"; shift ;;
    --sim-topic-prefix) SIM_TOPIC_PREFIX="${2:?--sim-topic-prefix requires a prefix}"; shift 2 ;;
    --sim-topic-prefix=*) SIM_TOPIC_PREFIX="${1#*=}"; shift ;;
    --mid360) LIVOX_MODEL="mid360"; shift ;;
    --mid360s) LIVOX_MODEL="mid360s"; shift ;;
    --ros-distro) ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"; shift 2 ;;
    --ros-distro=*) ROS_DISTRO_NAME="${1#*=}"; shift ;;
    --) shift; EXTRA_ELEVATION_ARGS=("$@"); break ;;
    *) echo "error: unsupported option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "${CONFIG_FILE}" ]] || { echo "error: config not found: ${CONFIG_FILE}" >&2; exit 1; }
[[ -f "${ELEVATION_CONFIG_FILE}" ]] || {
  echo "error: elevation config not found: ${ELEVATION_CONFIG_FILE}" >&2
  exit 1
}
if [[ -n "${MAP_FILE}" ]]; then
  MAP_FILE="$(realpath -- "${MAP_FILE}")"
  [[ -f "${MAP_FILE}" ]] || { echo "error: map not found: ${MAP_FILE}" >&2; exit 1; }
  [[ "${RVIZ}" == "auto" ]] && RVIZ="true"
fi
if [[ -n "${MAPPING_OUTPUT}" && -n "${MAP_FILE}" ]]; then
  echo "error: --map and --mapping-output cannot be used together" >&2
  exit 2
fi
if [[ -n "${MAPPING_OUTPUT}" ]]; then
  MAPPING_OUTPUT="$(realpath -m -- "${MAPPING_OUTPUT}")"
fi

case "${LIVOX_MODEL,,}" in
  mid360|mid-360) LIVOX_LAUNCH="msg_MID360_launch.py" ;;
  mid360s|mid-360s) LIVOX_LAUNCH="msg_MID360s_launch.py" ;;
  *) echo "error: --mid360 or --mid360s is required" >&2; exit 2 ;;
esac

ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[[ -f "${ROS_SETUP}" ]] || { echo "error: ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
[[ -f "${SCRIPT_DIR}/install/setup.bash" ]] && source "${SCRIPT_DIR}/install/setup.bash"
set -u
command -v ros2 >/dev/null || { echo "error: ros2 is unavailable" >&2; exit 1; }

if [[ "${VIS}" == "true" ]]; then
  /usr/bin/python3 -c 'import cv2, rclpy; from grid_map_msgs.msg import GridMap' || {
    echo "error: --vis requires python3-opencv, rclpy, and grid_map_msgs" >&2
    exit 1
  }
fi

RUNTIME_DIR="$(mktemp -d "/tmp/autonomy_light_$(id -u)_XXXXXX")"
LIO_CONFIG="${RUNTIME_DIR}/super_lio.yaml"
ELEVATION_CONFIG="${RUNTIME_DIR}/elevation_mapping.yaml"
STATIC_TF="${RUNTIME_DIR}/static_tf.txt"
RUNTIME_INFO="${RUNTIME_DIR}/runtime.env"

/usr/bin/python3 - "${CONFIG_FILE}" "${SCRIPT_DIR}/config/super_lio_mid360.yaml" \
  "${ELEVATION_CONFIG_FILE}" "${LIO_CONFIG}" "${ELEVATION_CONFIG}" \
  "${STATIC_TF}" "${RUNTIME_INFO}" "${MODE}" "${MAP_FILE}" "${MAPPING_OUTPUT}" \
  "${RAW_LIDAR_TOPIC}" "${RAW_IMU_TOPIC}" "${SIM_TOPIC_PREFIX}" <<'PY'
import math
import os
import sys
import yaml

(source_path, lio_default, elevation_source, lio_target, elevation_target, tf_target,
 runtime_target, mode, map_file, mapping_output, lidar_override, imu_override, sim_prefix) = sys.argv[1:14]

def read_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}

def params(document, node):
    return document.setdefault(node, {}).setdefault("ros__parameters", {})

def vector(values, name):
    if not isinstance(values, list) or len(values) != 3:
        raise SystemExit(f"error: {name} must contain exactly three numeric values")
    return [float(value) for value in values]

def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)

def multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)

def inverse(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)

def rotate(quaternion, point):
    qpoint = (point[0], point[1], point[2], 0.0)
    result = multiply(multiply(quaternion, qpoint), inverse(quaternion))
    return result[:3]

def matrix_from_quaternion(x, y, z, w):
    return [
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
        2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y),
    ]

root = params(read_yaml(source_path), "autonomy_light")
lidar_topic = lidar_override or root.get("raw_lidar_topic", "/livox/lidar")
imu_topic = imu_override or root.get("raw_imu_topic", "/livox/imu")
if mode == "sim":
    prefix = sim_prefix.rstrip("/")
    lidar_topic = lidar_override or f"{prefix}/livox/lidar"
    imu_topic = imu_override or f"{prefix}/livox/imu"

base_to_lidar = vector(root.get("target_to_lidar_xyz"), "target_to_lidar_xyz")
base_to_lidar_q = quaternion_from_rpy(*vector(root.get("target_to_lidar_rpy"), "target_to_lidar_rpy"))
imu_from_lidar = vector(root.get("imu_from_lidar_xyz"), "imu_from_lidar_xyz")
imu_from_lidar_q = quaternion_from_rpy(*vector(root.get("imu_from_lidar_rpy"), "imu_from_lidar_rpy"))

# T_base_imu = T_base_lidar * inverse(T_imu_lidar); publish its inverse.
lidar_from_imu_q = inverse(imu_from_lidar_q)
lidar_from_imu_t = rotate(lidar_from_imu_q, [-value for value in imu_from_lidar])
base_from_imu_q = multiply(base_to_lidar_q, lidar_from_imu_q)
base_from_imu_t = [base_to_lidar[index] + rotate(base_to_lidar_q, lidar_from_imu_t)[index] for index in range(3)]
imu_to_base_q = inverse(base_from_imu_q)
imu_to_base_t = rotate(imu_to_base_q, [-value for value in base_from_imu_t])

lio_source = root.get("super_lio_config_file") or lio_default
if not os.path.isfile(lio_source):
    raise SystemExit(f"error: Super-LIO config not found: {lio_source}")
lio = read_yaml(lio_source)
lio_params = params(lio, "/**")
lio_params["lio.ros.lidar_topic"] = lidar_topic
lio_params["lio.ros.imu_topic"] = imu_topic
lio_params["lio.ros.global_frame"] = root.get("map_frame", "map")
lio_params["lio.ros.odom_frame"] = root.get("odom_frame", "odom")
lio_params["lio.extrinsic.lidar_imu"] = imu_from_lidar + matrix_from_quaternion(*imu_from_lidar_q)
lio_params["lio.map.save_map"] = bool(mapping_output)
lio_params["lio.slam.enable"] = bool(mapping_output)
lio_params["use_sim_time"] = mode == "sim"
if map_file or mapping_output:
    output = map_file or mapping_output
    lio_params["lio.map.save_map_dir"] = os.path.dirname(output)
    lio_params["lio.map.map_name"] = os.path.basename(output)

mapper = read_yaml(elevation_source)
mapper_params = params(mapper, "elevation_mapping")
mapper_params["use_sim_time"] = mode == "sim"
# The TF contract is platform-wide; keep its names in one canonical config.
mapper_params["map_frame_id"] = root.get("map_frame", "map")
mapper_params["robot_base_frame_id"] = root.get("target_frame", "base_link")
mapper_params["track_point_frame_id"] = root.get("target_frame", "base_link")

with open(lio_target, "w", encoding="utf-8") as stream:
    yaml.safe_dump(lio, stream, default_flow_style=False, sort_keys=False)
with open(elevation_target, "w", encoding="utf-8") as stream:
    yaml.safe_dump(mapper, stream, default_flow_style=False, sort_keys=False)
with open(tf_target, "w", encoding="utf-8") as stream:
    stream.write(" ".join(str(value) for value in (*imu_to_base_t, *imu_to_base_q)) + "\n")
    stream.write(" ".join(str(value) for value in (*base_to_lidar, *base_to_lidar_q)) + "\n")
with open(runtime_target, "w", encoding="utf-8") as stream:
    stream.write(f"{int(root.get('ros_domain_id', 0))}\n")
    stream.write(f"{root.get('imu_frame', 'imu')}\n")
    stream.write(f"{root.get('target_frame', 'base_link')}\n")
    stream.write(f"{root.get('lidar_frame', 'lidar_link')}\n")
PY

mapfile -t RUNTIME_VALUES < "${RUNTIME_INFO}"
ROS_DOMAIN_ID="${RUNTIME_VALUES[0]}"
IMU_FRAME="${RUNTIME_VALUES[1]}"
BASE_FRAME="${RUNTIME_VALUES[2]}"
LIDAR_FRAME="${RUNTIME_VALUES[3]}"
export ROS_DOMAIN_ID
[[ -n "${VIS_TOPIC}" ]] || VIS_TOPIC="${MAP_TOPIC}"
read -r IMU_BASE_X IMU_BASE_Y IMU_BASE_Z IMU_BASE_QX IMU_BASE_QY IMU_BASE_QZ IMU_BASE_QW < "${STATIC_TF}"
read -r BASE_LIDAR_X BASE_LIDAR_Y BASE_LIDAR_Z BASE_LIDAR_QX BASE_LIDAR_QY BASE_LIDAR_QZ BASE_LIDAR_QW < <(sed -n '2p' "${STATIC_TF}")

declare -a PIDS=()
start() {
  echo "autonomy-light: starting $1"
  shift
  "$@" &
  PIDS+=("$!")
}

cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]:-}"; do
    kill -0 "${pid}" 2>/dev/null && kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
  find "${RUNTIME_DIR}" -depth -delete 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "autonomy-light: mode=${MODE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} config=${CONFIG_FILE}"
echo "elevation mapping: native GridMap ${MAP_TOPIC}"
[[ -n "${MAP_FILE}" ]] && echo "relocalization: Super-LIO map=${MAP_FILE}"
[[ -n "${MAPPING_OUTPUT}" ]] && echo "full SLAM: Super-LIO map=${MAPPING_OUTPUT}"

if [[ "${MODE}" == "real" && "${NO_DRIVERS}" != "true" ]]; then
  LIVOX_FREQ="$(/usr/bin/python3 - "${CONFIG_FILE}" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    params = (yaml.safe_load(stream) or {}).get("autonomy_light", {}).get("ros__parameters", {})
print(float(params.get("livox_publish_freq", 50.0)))
PY
)"
  start "Livox driver" ros2 launch livox_ros_driver2 "${LIVOX_LAUNCH}" "publish_freq:=${LIVOX_FREQ}"
fi
if [[ "${NO_DRIVERS}" != "true" ]]; then
  LIO_EXECUTABLE="super_lio_node"
  [[ -n "${MAP_FILE}" ]] && LIO_EXECUTABLE="relocation_node"
  start "Super-LIO" ros2 run super_lio "${LIO_EXECUTABLE}" --ros-args --params-file "${LIO_CONFIG}"
fi
if [[ "${NO_STATIC_TF}" != "true" ]]; then
  start "static imu -> base_link TF" ros2 run tf2_ros static_transform_publisher \
    --x "${IMU_BASE_X}" --y "${IMU_BASE_Y}" --z "${IMU_BASE_Z}" \
    --qx "${IMU_BASE_QX}" --qy "${IMU_BASE_QY}" --qz "${IMU_BASE_QZ}" --qw "${IMU_BASE_QW}" \
    --frame-id "${IMU_FRAME}" --child-frame-id "${BASE_FRAME}"
  start "static base_link -> lidar_link TF" ros2 run tf2_ros static_transform_publisher \
    --x "${BASE_LIDAR_X}" --y "${BASE_LIDAR_Y}" --z "${BASE_LIDAR_Z}" \
    --qx "${BASE_LIDAR_QX}" --qy "${BASE_LIDAR_QY}" --qz "${BASE_LIDAR_QZ}" --qw "${BASE_LIDAR_QW}" \
    --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}"
fi

start "ETH elevation mapping" ros2 run elevation_mapping elevation_mapping --ros-args \
  --params-file "${ELEVATION_CONFIG}" -r "elevation_map:=${MAP_TOPIC}" "${EXTRA_ELEVATION_ARGS[@]}"
MAPPER_PID="${PIDS[$((${#PIDS[@]} - 1))]}"

if [[ "${VIS}" == "true" ]]; then
  start "GridMap viewer" /usr/bin/python3 "${SCRIPT_DIR}/scripts/height_map_vis.py" \
    --topic "${VIS_TOPIC}" --fps "${VIS_FPS}" --scale "${VIS_SCALE}"
fi
if [[ "${RVIZ}" == "true" ]]; then
  RVIZ_CONFIG="${SCRIPT_DIR}/config/elevation_mapping.rviz"
  [[ -f "${RVIZ_CONFIG}" ]] || { echo "error: RViz config missing" >&2; exit 1; }
  start "RViz" rviz2 -d "${RVIZ_CONFIG}"
fi

wait "${MAPPER_PID}"
