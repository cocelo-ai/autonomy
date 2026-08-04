#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./launch.sh [--real|--sim] [--no-drivers] [--map FILE] [options] [-- <ros args>]

  --real                  Start the bundled Livox driver and Super-LIO (default).
  --sim                   Do not start Livox; use Livox CustomMsg simulation input.
  --no-drivers            Consume externally started /lio/odom and /lio/cloud_world.
  --map FILE              Start Super-LIO relocation_node with a saved PCD.
  --config FILE           Override autonomy_light.yaml.
  --raw-lidar-topic TOPIC Override the Super-LIO LiDAR input topic.
  --raw-imu-topic TOPIC   Override the Super-LIO IMU input topic.
  --sim-topic-prefix PFX  Simulation prefix, default /f4.
  --mid360|--mid360s      Select the default Livox launch file.
  --vis                   Start the HeightMap viewer.
  --rviz                  Start RViz (also enabled automatically with --map).
  --no-rviz               Do not auto-start RViz for --map.
  --ros-distro NAME       ROS distribution (default: $ROS_DISTRO or humble).

This launcher intentionally supports one Livox LiDAR/IMU pair. Multi-LiDAR
adapters belong upstream of Super-LIO, not in the elevation-map runtime.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="real"
NO_DRIVERS="false"
CONFIG_FILE="${AUTONOMY_LIGHT_CONFIG:-${SCRIPT_DIR}/config/autonomy_light.yaml}"
MAP_FILE="${AUTONOMY_LIGHT_MAP_FILE:-}"
RAW_LIDAR_TOPIC=""
RAW_IMU_TOPIC=""
SIM_TOPIC_PREFIX="${AUTONOMY_LIGHT_SIM_TOPIC_PREFIX-/f4}"
LIVOX_MODEL="${AUTONOMY_LIGHT_LIVOX_MODEL:-mid360}"
VIS="false"
RVIZ="auto"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
VIS_TOPIC="${AUTONOMY_LIGHT_VIS_TOPIC:-/autonomy_light/height_map_data}"
VIS_FPS="${AUTONOMY_LIGHT_VIS_FPS:-50}"
VIS_SCALE="${AUTONOMY_LIGHT_VIS_SCALE:-48}"
declare -a EXTRA_ROS_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --real) MODE="real"; shift ;;
    --sim) MODE="sim"; shift ;;
    --no-drivers) NO_DRIVERS="true"; shift ;;
    --vis) VIS="true"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --config) CONFIG_FILE="${2:?--config requires a file}"; shift 2 ;;
    --config=*) CONFIG_FILE="${1#*=}"; shift ;;
    --map|--map-file) MAP_FILE="${2:?--map requires a PCD}"; shift 2 ;;
    --map=*|--map-file=*) MAP_FILE="${1#*=}"; shift ;;
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
    --) shift; EXTRA_ROS_ARGS=("$@"); break ;;
    *) echo "error: unsupported option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "${CONFIG_FILE}" ]] || { echo "error: config not found: ${CONFIG_FILE}" >&2; exit 1; }
if [[ -n "${MAP_FILE}" ]]; then
  MAP_FILE="$(realpath -- "${MAP_FILE}")"
  [[ -f "${MAP_FILE}" ]] || { echo "error: map not found: ${MAP_FILE}" >&2; exit 1; }
  [[ "${RVIZ}" == "auto" ]] && RVIZ="true"
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

EFFECTIVE_CONFIG="$(mktemp "/tmp/autonomy_light_$(id -u)_XXXXXX.yaml")"
trap 'rm -f -- "${EFFECTIVE_CONFIG}"' EXIT
/usr/bin/python3 - "${CONFIG_FILE}" "${EFFECTIVE_CONFIG}" "${MODE}" "${NO_DRIVERS}" \
  "${MAP_FILE}" "${RAW_LIDAR_TOPIC}" "${RAW_IMU_TOPIC}" "${SIM_TOPIC_PREFIX}" \
  "${LIVOX_LAUNCH}" <<'PY'
import sys
import yaml

(source, target, mode, no_drivers, map_file, lidar, imu, prefix, launch_file) = sys.argv[1:10]
with open(source, encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}
params = data.setdefault("autonomy_light", {}).setdefault("ros__parameters", {})
params["start_lidar_driver"] = mode == "real" and no_drivers != "true"
params["start_super_lio"] = no_drivers != "true"
params["child_use_sim_time"] = mode == "sim"
params["use_sim_time"] = mode == "sim"
merge_params = data.setdefault("rolling_cloud_merge", {}).setdefault("ros__parameters", {})
merge_params["use_sim_time"] = mode == "sim"
merge_params["lidar_cloud_topic"] = params.get("super_lio_registered_topic", "/lio/cloud_world")
merge_params["output_topic"] = params.get(
    "rolling_merge", {}).get("output_topic", "/autonomy_light/rolling_cloud")
if map_file:
    params["saved_map_file"] = map_file
if lidar:
    params["raw_lidar_topic"] = lidar
if imu:
    params["raw_imu_topic"] = imu
if mode == "sim":
    prefix = prefix.rstrip("/")
    params["raw_lidar_topic"] = lidar or f"{prefix}/livox/lidar"
    params["raw_imu_topic"] = imu or f"{prefix}/livox/imu"
if mode == "real" and no_drivers != "true":
    frequency = float(params.get("livox_publish_freq", 50.0))
    params["lidar_driver_command"] = [
        "ros2", "launch", "livox_ros_driver2", launch_file,
        f"publish_freq:={frequency}"]
with open(target, "w", encoding="utf-8") as stream:
    yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)
PY

ROS_DOMAIN_ID="${AUTONOMY_LIGHT_ROS_DOMAIN_ID:-$(/usr/bin/python3 - "${EFFECTIVE_CONFIG}" <<'PY'
import os
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    params = (yaml.safe_load(stream) or {}).get("autonomy_light", {}).get("ros__parameters", {})
print(params.get("ros_domain_id", os.environ.get("ROS_DOMAIN_ID", "0")))
PY
)}"
export ROS_DOMAIN_ID

COMMAND=(ros2 run autonomy_light autonomy_light --ros-args --params-file "${EFFECTIVE_CONFIG}" "${EXTRA_ROS_ARGS[@]}")
MERGE_ENABLED="$(/usr/bin/python3 - "${EFFECTIVE_CONFIG}" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    params = (yaml.safe_load(stream) or {}).get("autonomy_light", {}).get("ros__parameters", {})
enabled = bool(params.get("rolling_merge", {}).get("enabled", False))
print("true" if enabled and params.get("height_map", {}).get("source") == "rolling" else "false")
PY
)"
MERGE_COMMAND=(ros2 run autonomy_light rolling_cloud_merge --ros-args --params-file "${EFFECTIVE_CONFIG}")
echo "autonomy-light: mode=${MODE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} config=${CONFIG_FILE}"
[[ -n "${MAP_FILE}" ]] && echo "relocalization: Super-LIO relocation_node map=${MAP_FILE}"

cleanup() {
  for pid in "${RUNTIME_PID:-}" "${MERGE_PID:-}" "${VIS_PID:-}" "${RVIZ_PID:-}"; do
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && kill -INT "${pid}" 2>/dev/null || true
  done
}
[[ "${MERGE_ENABLED}" == "true" ]] && echo "rolling merge: D435 + LiDAR enabled"

if [[ "${VIS}" != "true" && "${RVIZ}" != "true" ]]; then
  if [[ "${MERGE_ENABLED}" != "true" ]]; then
    exec "${COMMAND[@]}"
  fi
  trap cleanup EXIT INT TERM
  "${MERGE_COMMAND[@]}" &
  MERGE_PID="$!"
  "${COMMAND[@]}"
  exit $?
fi

trap cleanup EXIT INT TERM
if [[ "${MERGE_ENABLED}" == "true" ]]; then
  "${MERGE_COMMAND[@]}" &
  MERGE_PID="$!"
fi
"${COMMAND[@]}" &
RUNTIME_PID="$!"

if [[ "${VIS}" == "true" ]]; then
  VIS_SCRIPT="${SCRIPT_DIR}/scripts/height_map_vis.py"
  [[ -x "${VIS_SCRIPT}" ]] || VIS_SCRIPT="${SCRIPT_DIR}/height_map_vis.py"
  "${VIS_SCRIPT}" --topic "${VIS_TOPIC}" --fps "${VIS_FPS}" --scale "${VIS_SCALE}" &
  VIS_PID="$!"
fi
if [[ "${RVIZ}" == "true" ]]; then
  RVIZ_CONFIG="${SCRIPT_DIR}/config/autonomy_light_relocalization.rviz"
  [[ -f "${RVIZ_CONFIG}" ]] || { echo "error: RViz config missing" >&2; exit 1; }
  echo "RViz: use 2D Pose Estimate for Super-LIO relocation."
  rviz2 -d "${RVIZ_CONFIG}" &
  RVIZ_PID="$!"
fi
wait "${RUNTIME_PID}"
