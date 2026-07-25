#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  launch.sh [--real|--sim] [--no-drivers] [--vis] [options] [-- <extra ros args>]

Modes:
  --real              Real robot mode. Starts Livox driver and Point-LIO. Default.
  --sim               Simulation mode. Does not start Livox driver, uses sim time,
                      and reads simulated /<prefix>/livox/lidar + /<prefix>/livox/imu.
  --no-drivers        Start autonomy_light only; disables both Livox driver and Point-LIO.
  --vis               Start a lightweight 50Hz OpenCV height-map viewer.

Options:
  --config FILE       Override config yaml.
  --sim-topic-prefix PREFIX
                      Simulation topic prefix. Default: /f4 or AUTONOMY_LIGHT_SIM_TOPIC_PREFIX.
                      Use "" or / for root topics such as /livox/lidar.
  --raw-lidar-topic TOPIC
                      Override raw LiDAR topic.
  --raw-lidar2-topic TOPIC
                      Enable/override optional second raw LiDAR topic.
  --raw-imu-topic TOPIC
                      Override raw IMU topic.
  --map FILE          Load a saved Point-LIO PCD map and build height map from it.
                      Relocalizes registered scans in the saved-map frame before
                      extracting the robot-centric height map.
  --mid360            Use Livox MID360 network/config JSON.
  --mid360s           Use Livox MID360s network/config JSON.
  --livox-model MODEL Livox model: mid360 or mid360s.
  --livox-interface IFACE
                      Livox wired interface. Default: enP8p1s0.
  --livox2-interface IFACE
                      Optional second Livox wired interface.
  --livox-host-ip CIDR
                      Livox host IP/CIDR. Default: 192.168.1.50/24.
  --livox-lidar-ip IP
                      Livox LiDAR IP. Default: 192.168.1.12.
  --livox-lidar2-ip IP
                      Enable second Livox driver for this LiDAR IP.
  --ros-distro NAME   ROS distro. Default: ROS_DISTRO or auto-detected /opt/ros.
  --skip-livox-network
                      Do not configure Livox host network/config.
  --vis-topic TOPIC   HeightMap topic for --vis. Default: /autonomy_light/height_map_data.
  --vis-fps HZ        Viewer refresh rate. Default: 50.
  --vis-scale SCALE   Viewer nearest-neighbor pixel scale. Default: 10.

Examples:
  ./launch.sh --real
  ./launch.sh --real --vis
  ./launch.sh --real --map maps/lab_mapping.pcd
  ROS_DOMAIN_ID=17 rviz2                  # global map (set matching config domain)
  ./launch.sh --real --mid360
  ./launch.sh --real --mid360s
  ./launch.sh --sim
  ./launch.sh --sim --sim-topic-prefix /f16
  ./launch.sh --sim --sim-topic-prefix ""     # /livox/lidar and /livox/imu
  AUTONOMY_LIGHT_SIM_TOPIC_PREFIX= ./launch.sh --sim
  ./launch.sh --no-drivers
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_PREFIX_DIR="$(cd -- "${SCRIPT_DIR}/../../.." 2>/dev/null && pwd || true)"
CALLER_WORKSPACE_DIR="$(pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-}"
MODE="real"
NO_DRIVERS="false"
VIS="false"
VIS_TOPIC="${AUTONOMY_LIGHT_VIS_TOPIC:-/autonomy_light/height_map_data}"
VIS_FPS="${AUTONOMY_LIGHT_VIS_FPS:-50}"
VIS_SCALE="${AUTONOMY_LIGHT_VIS_SCALE:-10}"
VIS_ROS_DOMAIN_ID="${AUTONOMY_LIGHT_VIS_ROS_DOMAIN_ID:-}"
SIM_TOPIC_PREFIX="${AUTONOMY_LIGHT_SIM_TOPIC_PREFIX-/f4}"
RAW_LIDAR_TOPIC=""
RAW_LIDAR2_TOPIC=""
RAW_IMU_TOPIC=""
RAW_IMU2_TOPIC=""
SAVED_MAP_FILE="${AUTONOMY_LIGHT_MAP_FILE:-}"
LIVOX_MODEL="${AUTONOMY_LIGHT_LIVOX_MODEL:-}"
LIVOX_INTERFACE="${AUTONOMY_LIGHT_LIVOX_INTERFACE:-}"
LIVOX2_INTERFACE="${AUTONOMY_LIGHT_LIVOX2_INTERFACE:-}"
LIVOX_HOST_IP="${AUTONOMY_LIGHT_LIVOX_HOST_IP:-}"
LIVOX_LIDAR_IP="${AUTONOMY_LIGHT_LIVOX_LIDAR_IP:-}"
LIVOX_HOST2_IP="${AUTONOMY_LIGHT_LIVOX_HOST2_IP:-}"
LIVOX_LIDAR2_IP="${AUTONOMY_LIGHT_LIVOX_LIDAR2_IP:-}"
LIVOX_CONFIG_PATH="${AUTONOMY_LIGHT_LIVOX_CONFIG_PATH:-}"
LIVOX_CONFIG2_PATH="${AUTONOMY_LIGHT_LIVOX_CONFIG2_PATH:-}"
SKIP_LIVOX_NETWORK="${AUTONOMY_LIGHT_SKIP_LIVOX_NETWORK:-false}"
declare -a LIVOX_DRIVER_COMMAND=()
declare -a LIVOX_DRIVER2_COMMAND=()

if [[ -n "${AUTONOMY_LIGHT_CONFIG:-}" ]]; then
  CONFIG_FILE="${AUTONOMY_LIGHT_CONFIG}"
elif [[ -f "${SCRIPT_DIR}/config/autonomy_light.yaml" ]]; then
  CONFIG_FILE="${SCRIPT_DIR}/config/autonomy_light.yaml"
else
  CONFIG_FILE="${SCRIPT_DIR}/../../share/autonomy_light/config/autonomy_light.yaml"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --real|real)
      MODE="real"
      shift
      ;;
    --sim|sim|simulation:=true|simulation:=1|simulation:=yes|simulation:=on)
      MODE="sim"
      shift
      ;;
    simulation:=false|simulation:=0|simulation:=no|simulation:=off)
      MODE="real"
      shift
      ;;
    --no-drivers)
      NO_DRIVERS="true"
      shift
      ;;
    --vis)
      VIS="true"
      shift
      ;;
    --vis-topic)
      VIS_TOPIC="${2:?--vis-topic requires a topic}"
      shift 2
      ;;
    --vis-topic=*)
      VIS_TOPIC="${1#--vis-topic=}"
      shift
      ;;
    --vis-fps)
      VIS_FPS="${2:?--vis-fps requires a rate}"
      shift 2
      ;;
    --vis-fps=*)
      VIS_FPS="${1#--vis-fps=}"
      shift
      ;;
    --vis-scale)
      VIS_SCALE="${2:?--vis-scale requires a positive integer}"
      shift 2
      ;;
    --vis-scale=*)
      VIS_SCALE="${1#--vis-scale=}"
      shift
      ;;
    --config)
      CONFIG_FILE="${2:?--config requires a file path}"
      shift 2
      ;;
    --config=*)
      CONFIG_FILE="${1#--config=}"
      shift
      ;;
    --sim-topic-prefix)
      SIM_TOPIC_PREFIX="${2:?--sim-topic-prefix requires a prefix}"
      shift 2
      ;;
    --sim-topic-prefix=*)
      SIM_TOPIC_PREFIX="${1#--sim-topic-prefix=}"
      shift
      ;;
    --raw-lidar-topic)
      RAW_LIDAR_TOPIC="${2:?--raw-lidar-topic requires a topic}"
      shift 2
      ;;
    --raw-lidar-topic=*)
      RAW_LIDAR_TOPIC="${1#--raw-lidar-topic=}"
      shift
      ;;
    --raw-lidar2-topic)
      RAW_LIDAR2_TOPIC="${2:?--raw-lidar2-topic requires a topic}"
      shift 2
      ;;
    --raw-lidar2-topic=*)
      RAW_LIDAR2_TOPIC="${1#--raw-lidar2-topic=}"
      shift
      ;;
    --raw-imu-topic)
      RAW_IMU_TOPIC="${2:?--raw-imu-topic requires a topic}"
      shift 2
      ;;
    --raw-imu-topic=*)
      RAW_IMU_TOPIC="${1#--raw-imu-topic=}"
      shift
      ;;
    --map|--map-file)
      SAVED_MAP_FILE="${2:?--map requires a PCD file path}"
      shift 2
      ;;
    --map=*|--map-file=*)
      SAVED_MAP_FILE="${1#*=}"
      shift
      ;;
    --mid360)
      LIVOX_MODEL="mid360"
      shift
      ;;
    --mid360s)
      LIVOX_MODEL="mid360s"
      shift
      ;;
    --livox-model)
      LIVOX_MODEL="${2:?--livox-model requires mid360 or mid360s}"
      shift 2
      ;;
    --livox-model=*)
      LIVOX_MODEL="${1#--livox-model=}"
      shift
      ;;
    --livox-interface)
      LIVOX_INTERFACE="${2:?--livox-interface requires an interface name}"
      shift 2
      ;;
    --livox-interface=*)
      LIVOX_INTERFACE="${1#--livox-interface=}"
      shift
      ;;
    --livox2-interface)
      LIVOX2_INTERFACE="${2:?--livox2-interface requires an interface name}"
      shift 2
      ;;
    --livox2-interface=*)
      LIVOX2_INTERFACE="${1#--livox2-interface=}"
      shift
      ;;
    --livox-host-ip)
      LIVOX_HOST_IP="${2:?--livox-host-ip requires an IP or CIDR}"
      shift 2
      ;;
    --livox-host-ip=*)
      LIVOX_HOST_IP="${1#--livox-host-ip=}"
      shift
      ;;
    --livox-lidar-ip)
      LIVOX_LIDAR_IP="${2:?--livox-lidar-ip requires an IP}"
      shift 2
      ;;
    --livox-lidar2-ip)
      LIVOX_LIDAR2_IP="${2:?--livox-lidar2-ip requires an IP}"
      shift 2
      ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"
      shift 2
      ;;
    --ros-distro=*)
      ROS_DISTRO_NAME="${1#--ros-distro=}"
      shift
      ;;
    --livox-lidar-ip=*)
      LIVOX_LIDAR_IP="${1#--livox-lidar-ip=}"
      shift
      ;;
    --livox-lidar2-ip=*)
      LIVOX_LIDAR2_IP="${1#--livox-lidar2-ip=}"
      shift
      ;;
    --skip-livox-network)
      SKIP_LIVOX_NETWORK="true"
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      break
      ;;
  esac
done

if [[ -n "${SAVED_MAP_FILE}" && ! -f "${SAVED_MAP_FILE}" ]]; then
  echo "error: saved map PCD not found: ${SAVED_MAP_FILE}" >&2
  exit 1
fi

read_ubuntu_version_id() {
  local version_id=""
  if [[ -r /etc/os-release ]]; then
    # shellcheck source=/dev/null
    source /etc/os-release
    version_id="${VERSION_ID:-}"
  fi
  echo "${version_id}"
}

preferred_ros_for_ubuntu() {
  case "$1" in
    20.04)
      echo "foxy"
      ;;
    22.04)
      echo "humble"
      ;;
    24.04)
      echo "jazzy"
      ;;
    *)
      echo ""
      ;;
  esac
}

detect_ros_distro() {
  local requested="${ROS_DISTRO_NAME}"
  local ros_root="/opt/ros"

  if [[ -n "${requested}" ]]; then
    if [[ ! -f "${ros_root}/${requested}/setup.bash" ]]; then
      echo "error: ROS setup file not found: ${ros_root}/${requested}/setup.bash" >&2
      exit 1
    fi
    echo "${requested}"
    return
  fi

  local preferred
  preferred="$(preferred_ros_for_ubuntu "$(read_ubuntu_version_id)")"
  if [[ -n "${preferred}" && -f "${ros_root}/${preferred}/setup.bash" ]]; then
    echo "${preferred}"
    return
  fi

  local candidates=()
  local setup_file
  shopt -s nullglob
  for setup_file in "${ros_root}"/*/setup.bash; do
    candidates+=("$(basename "$(dirname "${setup_file}")")")
  done
  shopt -u nullglob

  if [[ "${#candidates[@]}" -eq 1 ]]; then
    echo "${candidates[0]}"
    return
  fi

  if [[ "${#candidates[@]}" -gt 1 ]]; then
    echo "error: multiple ROS distros found under /opt/ros: ${candidates[*]}" >&2
    echo "hint: choose one with --ros-distro NAME or export ROS_DISTRO." >&2
  else
    echo "error: no ROS setup.bash found under /opt/ros." >&2
  fi
  exit 1
}

ROS_DISTRO_NAME="$(detect_ros_distro)"

normalize_livox_model() {
  local model="${1,,}"
  model="${model//_/-}"
  case "${model}" in
    ""|mid360|mid-360)
      echo "mid360"
      ;;
    mid360s|mid-360s)
      echo "mid360s"
      ;;
    *)
      echo "error: unsupported Livox model: ${1}" >&2
      echo "supported Livox models: mid360, mid360s" >&2
      exit 2
      ;;
  esac
}

if [[ -n "${LIVOX_MODEL}" ]]; then
  LIVOX_MODEL="$(normalize_livox_model "${LIVOX_MODEL}")"
fi

read_livox_network_config() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

params = ((data.get("autonomy_light") or {}).get("ros__parameters")) or {}

def value(name, default=""):
    item = params.get(name, default)
    return "" if item is None else str(item).strip()

print(value("livox_interface", "enP8p1s0"))
print(value("livox_host_ip", "192.168.1.50/24"))
print(value("livox_lidar_ip", "192.168.1.12"))
print(value("lidar_frame", "livox_frame"))
print(value("livox_model", "mid360"))
print(value("livox2_interface", ""))
print(value("livox_host2_ip", ""))
print(value("livox_lidar2_ip", ""))
print(value("raw_lidar_topic", ""))
print(value("raw_lidar2_topic", ""))
print(value("raw_imu_topic", ""))
print(value("raw_imu2_topic", ""))
PY
}

read_ros_domain_config() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import os
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

params = ((data.get("autonomy_light") or {}).get("ros__parameters")) or {}
domain = params.get("internal_ros_domain_id", os.environ.get("ROS_DOMAIN_ID", "0"))
print("" if domain is None else str(domain).strip())
PY
}

read_external_ros_domain_config() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import os
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

params = ((data.get("autonomy_light") or {}).get("ros__parameters")) or {}
fallback = params.get("internal_ros_domain_id", os.environ.get("ROS_DOMAIN_ID", "0"))
domain = params.get("external_ros_domain_id", fallback)
print("" if domain is None else str(domain).strip())
PY
}

height_map_vis_script() {
  if [[ -x "${SCRIPT_DIR}/scripts/height_map_vis.py" ]]; then
    echo "${SCRIPT_DIR}/scripts/height_map_vis.py"
    return
  fi
  if [[ -x "${SCRIPT_DIR}/height_map_vis.py" ]]; then
    echo "${SCRIPT_DIR}/height_map_vis.py"
    return
  fi
  echo "error: height_map_vis.py not found next to launch.sh or in scripts/." >&2
  exit 1
}

effective_config_path() {
  local uid
  uid="${UID:-$(id -u)}"
  echo "/tmp/autonomy_light_${uid}_${MODE}_params.yaml"
}

write_effective_config() {
  local input_file="$1"
  local output_file="$2"
  local lidar_driver_command_json="${3:-}"
  local lidar_driver2_command_json="${4:-}"

  /usr/bin/python3 - \
    "${input_file}" \
    "${output_file}" \
    "${MODE}" \
    "${NO_DRIVERS}" \
    "${RAW_LIDAR_TOPIC}" \
    "${RAW_LIDAR2_TOPIC}" \
    "${RAW_IMU_TOPIC}" \
    "${RAW_IMU2_TOPIC}" \
    "${LIVOX_LIDAR2_IP}" \
    "${SAVED_MAP_FILE}" \
    "${lidar_driver_command_json}" \
    "${lidar_driver2_command_json}" <<'PY'
import json
import sys

import yaml

(
    input_file,
    output_file,
    mode,
    no_drivers,
    raw_lidar_topic,
    raw_lidar2_topic,
    raw_imu_topic,
    raw_imu2_topic,
    livox_lidar2_ip,
    saved_map_file,
    driver_command,
    driver2_command,
) = sys.argv[1:13]
with open(input_file, "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

node = data.setdefault("autonomy_light", {})
params = node.setdefault("ros__parameters", {})

if mode == "sim":
    params["start_lidar_driver"] = False
    params["start_point_lio"] = True
    params["use_sim_time"] = True
    params["child_use_sim_time"] = True
    params["raw_lidar_topic"] = raw_lidar_topic
    params["raw_lidar_msg_type"] = "pointcloud2"
    params["raw_imu_topic"] = raw_imu_topic
else:
    params["start_lidar_driver"] = True
    params["start_point_lio"] = True
    params["use_sim_time"] = False
    params["child_use_sim_time"] = False

if no_drivers in ("true", "1", "yes", "on"):
    params["start_lidar_driver"] = False
    params["start_point_lio"] = False

if raw_lidar_topic:
    params["raw_lidar_topic"] = raw_lidar_topic

if raw_lidar2_topic:
    params["raw_lidar2_topic"] = raw_lidar2_topic

if raw_imu_topic:
    params["raw_imu_topic"] = raw_imu_topic

if raw_imu2_topic:
    params["raw_imu2_topic"] = raw_imu2_topic

if livox_lidar2_ip:
    params["livox_lidar2_ip"] = livox_lidar2_ip

if saved_map_file:
    params["saved_map_file"] = saved_map_file

if driver_command:
    params["lidar_driver_command"] = json.loads(driver_command)

if driver2_command:
    params["lidar_driver2_command"] = json.loads(driver2_command)

with open(output_file, "w", encoding="utf-8") as stream:
    yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)
PY
}

load_livox_defaults() {
  if [[ ! -f "${CONFIG_FILE}" ]]; then
    LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-livox_frame}"
    LIVOX_MODEL="$(normalize_livox_model "${LIVOX_MODEL:-mid360}")"
    return
  fi

  local cfg_iface cfg_host_cidr cfg_lidar_ip cfg_frame_id cfg_model cfg_iface2
  local cfg_host2_cidr cfg_lidar2_ip cfg_raw_lidar_topic cfg_raw_lidar2_topic cfg_raw_imu_topic cfg_raw_imu2_topic
  mapfile -t livox_config < <(read_livox_network_config "${CONFIG_FILE}")
  cfg_iface="${livox_config[0]:-}"
  cfg_host_cidr="${livox_config[1]:-}"
  cfg_lidar_ip="${livox_config[2]:-}"
  cfg_frame_id="${livox_config[3]:-}"
  cfg_model="${livox_config[4]:-}"
  cfg_iface2="${livox_config[5]:-}"
  cfg_host2_cidr="${livox_config[6]:-}"
  cfg_lidar2_ip="${livox_config[7]:-}"
  cfg_raw_lidar_topic="${livox_config[8]:-}"
  cfg_raw_lidar2_topic="${livox_config[9]:-}"
  cfg_raw_imu_topic="${livox_config[10]:-}"
  cfg_raw_imu2_topic="${livox_config[11]:-}"
  LIVOX_INTERFACE="${LIVOX_INTERFACE:-${cfg_iface}}"
  LIVOX2_INTERFACE="${AUTONOMY_LIGHT_LIVOX2_INTERFACE:-${LIVOX2_INTERFACE:-${cfg_iface2}}}"
  LIVOX_HOST_IP="${AUTONOMY_LIGHT_LIVOX_HOST_IP:-${LIVOX_HOST_IP:-${cfg_host_cidr}}}"
  LIVOX_LIDAR_IP="${AUTONOMY_LIGHT_LIVOX_LIDAR_IP:-${LIVOX_LIDAR_IP:-${cfg_lidar_ip}}}"
  LIVOX_HOST2_IP="${AUTONOMY_LIGHT_LIVOX_HOST2_IP:-${LIVOX_HOST2_IP:-${cfg_host2_cidr}}}"
  LIVOX_LIDAR2_IP="${AUTONOMY_LIGHT_LIVOX_LIDAR2_IP:-${LIVOX_LIDAR2_IP:-${cfg_lidar2_ip}}}"
  RAW_LIDAR_TOPIC="${RAW_LIDAR_TOPIC:-${cfg_raw_lidar_topic}}"
  RAW_LIDAR2_TOPIC="${RAW_LIDAR2_TOPIC:-${cfg_raw_lidar2_topic}}"
  RAW_IMU_TOPIC="${RAW_IMU_TOPIC:-${cfg_raw_imu_topic}}"
  RAW_IMU2_TOPIC="${RAW_IMU2_TOPIC:-${cfg_raw_imu2_topic}}"
  LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-${cfg_frame_id:-livox_frame}}"
  LIVOX_MODEL="$(normalize_livox_model "${LIVOX_MODEL:-${cfg_model:-mid360}}")"
}

default_livox_config_path() {
  local model="${1}"
  local suffix="${2:-1}"
  if [[ -n "${XDG_RUNTIME_DIR:-}" && -d "${XDG_RUNTIME_DIR}" && -w "${XDG_RUNTIME_DIR}" ]]; then
    echo "${XDG_RUNTIME_DIR}/autonomy_light_livox_${model}_${suffix}_config.json"
    return
  fi

  local uid
  uid="${UID:-$(id -u)}"
  echo "/tmp/autonomy_light_${uid}_livox_${model}_${suffix}_config.json"
}

interface_has_ip() {
  local iface="$1"
  local local_host="$2"
  ip -o -4 addr show dev "${iface}" | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "${local_host}"
}

interface_has_cidr() {
  local iface="$1"
  local cidr="$2"
  ip -o -4 addr show dev "${iface}" | awk '{print $4}' | grep -Fxq "${cidr}"
}

write_livox_config() {
  local output_path="$1"
  local host_ip="$2"
  local lidar_ip="$3"
  local model="$4"
  local host_port_offset="${5:-0}"
  mkdir -p "$(dirname "${output_path}")"
  /usr/bin/python3 - "${output_path}" "${host_ip}" "${lidar_ip}" "${model}" "${host_port_offset}" <<'PY'
import json
import sys

output_path, host_ip, lidar_ip, model, host_port_offset = sys.argv[1:6]
host_port_offset = int(host_port_offset)
config = {
    "lidar_summary_info": {"lidar_type": 8},
    "lidar_configs": [
        {
            "ip": lidar_ip,
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0,
            },
        }
    ],
}
net_info = {
    "cmd_data_port": 56100,
    "push_msg_port": 56200,
    "point_data_port": 56300,
    "imu_data_port": 56400,
    "log_data_port": 56500,
}
if model == "mid360":
    config["MID360"] = {
        "lidar_net_info": net_info,
        "host_net_info": {
            "cmd_data_ip": host_ip,
            "cmd_data_port": 56101 + host_port_offset,
            "push_msg_ip": host_ip,
            "push_msg_port": 56201 + host_port_offset,
            "point_data_ip": host_ip,
            "point_data_port": 56301 + host_port_offset,
            "imu_data_ip": host_ip,
            "imu_data_port": 56401 + host_port_offset,
            "log_data_ip": "",
            "log_data_port": 56501 + host_port_offset,
        },
    }
elif model == "mid360s":
    config["Mid360s"] = {
        "lidar_net_info": net_info,
        "host_net_info": [
            {
                "host_ip": host_ip,
                "cmd_data_port": 56101 + host_port_offset,
                "push_msg_port": 56201 + host_port_offset,
                "point_data_port": 56301 + host_port_offset,
                "imu_data_port": 56401 + host_port_offset,
                "log_data_port": 56501 + host_port_offset,
            }
        ],
    }
else:
    raise SystemExit(f"unsupported Livox model: {model}")
with open(output_path, "w", encoding="utf-8") as stream:
    json.dump(config, stream, indent=2)
    stream.write("\n")
PY
}

configure_livox_network() {
  if [[ "${MODE}" != "real" || "${NO_DRIVERS}" == "true" ]]; then
    return
  fi
  case "${SKIP_LIVOX_NETWORK}" in
    true|1|yes|on)
      return
      ;;
  esac

  if [[ -n "${LIVOX_CONFIG_PATH}" && ( -z "${LIVOX_LIDAR2_IP}" || -n "${LIVOX_CONFIG2_PATH}" ) ]]; then
    return
  fi

  if ! command -v ip >/dev/null 2>&1; then
    echo "warning: 'ip' command not found; skipping Livox fixed network setup." >&2
    return
  fi

  local host_cidr="${LIVOX_HOST_IP:-192.168.1.50/24}"
  local lidar_ip="${LIVOX_LIDAR_IP:-192.168.1.190}"
  local host_ip="${host_cidr%%/*}"
  [[ "${host_cidr}" == */* ]] || host_cidr="${host_cidr}/24"

  local iface="${LIVOX_INTERFACE:-enP8p1s0}"
  if [[ ! -d "/sys/class/net/${iface}" ]]; then
    echo "warning: Livox fixed network interface does not exist: ${iface}" >&2
    echo "warning: set --livox-interface or livox_interface in the autonomy-light config." >&2
    return
  fi

  sudo ip link set "${iface}" up
  if ! interface_has_cidr "${iface}" "${host_cidr}"; then
    if ! interface_has_ip "${iface}" "${host_ip}"; then
      echo "Configuring Livox ${LIVOX_MODEL} host network: ${iface} ${host_cidr}"
      sudo ip addr add "${host_cidr}" dev "${iface}"
    fi
  fi

  if [[ -z "${LIVOX_CONFIG_PATH}" ]]; then
    LIVOX_CONFIG_PATH="$(default_livox_config_path "${LIVOX_MODEL}" 1)"
    write_livox_config "${LIVOX_CONFIG_PATH}" "${host_ip}" "${lidar_ip}" "${LIVOX_MODEL}" 0
  fi
  echo "Livox ${LIVOX_MODEL} network: iface=${iface} host=${host_ip} lidar=${lidar_ip} config=${LIVOX_CONFIG_PATH}"

  if [[ -n "${LIVOX_LIDAR2_IP}" ]]; then
    local host2_cidr="${LIVOX_HOST2_IP:-${host_cidr}}"
    local host2_ip="${host2_cidr%%/*}"
    [[ "${host2_cidr}" == */* ]] || host2_cidr="${host2_cidr}/24"
    local iface2="${LIVOX2_INTERFACE:-${iface}}"
    if [[ ! -d "/sys/class/net/${iface2}" ]]; then
      echo "warning: second Livox fixed network interface does not exist: ${iface2}" >&2
      echo "warning: set --livox2-interface or livox2_interface in the autonomy-light config." >&2
      return
    fi
    sudo ip link set "${iface2}" up
    if ! interface_has_cidr "${iface2}" "${host2_cidr}"; then
      if ! interface_has_ip "${iface2}" "${host2_ip}"; then
        echo "Configuring Livox ${LIVOX_MODEL} second host network: ${iface2} ${host2_cidr}"
        sudo ip addr add "${host2_cidr}" dev "${iface2}"
      fi
    fi
    if [[ -z "${LIVOX_CONFIG2_PATH}" ]]; then
      LIVOX_CONFIG2_PATH="$(default_livox_config_path "${LIVOX_MODEL}" 2)"
      write_livox_config "${LIVOX_CONFIG2_PATH}" "${host2_ip}" "${LIVOX_LIDAR2_IP}" "${LIVOX_MODEL}" 10
    fi
    echo "Livox ${LIVOX_MODEL} network 2: iface=${iface2} host=${host2_ip} lidar=${LIVOX_LIDAR2_IP} config=${LIVOX_CONFIG2_PATH}"
  fi
}

source_setup_file() {
  local setup_file="$1"
  [[ -f "${setup_file}" ]] || return 1
  case ":${AUTONOMY_LIGHT_SOURCED_SETUPS:-}:" in
    *":${setup_file}:"*)
      return 0
      ;;
  esac
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
  AUTONOMY_LIGHT_SOURCED_SETUPS="${AUTONOMY_LIGHT_SOURCED_SETUPS:-}:${setup_file}"
  echo "sourced: ${setup_file}"
}

source_setup_files() {
  local ros_setup="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  if ! source_setup_file "${ros_setup}"; then
    echo "error: ROS setup file not found: ${ros_setup}" >&2
    exit 1
  fi

  source_setup_file "${SOURCE_WORKSPACE_DIR}/install/setup.bash" || true
  if [[ -n "${INSTALL_PREFIX_DIR}" ]]; then
    source_setup_file "${INSTALL_PREFIX_DIR}/setup.bash" || true
  fi
  source_setup_file "${CALLER_WORKSPACE_DIR}/install/setup.bash" || true

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "error: ros2 command not found after sourcing setup files." >&2
    exit 1
  fi
}

load_livox_defaults
source_setup_files

if [[ -f "${CONFIG_FILE}" ]]; then
  INTERNAL_ROS_DOMAIN_ID="${AUTONOMY_LIGHT_INTERNAL_ROS_DOMAIN_ID:-$(read_ros_domain_config "${CONFIG_FILE}")}"
  if [[ -n "${INTERNAL_ROS_DOMAIN_ID}" ]]; then
    export ROS_DOMAIN_ID="${INTERNAL_ROS_DOMAIN_ID}"
    echo "ROS_DOMAIN_ID internal=${ROS_DOMAIN_ID}"
  fi
fi

if [[ "${MODE}" == "sim" ]]; then
  sim_prefix="${SIM_TOPIC_PREFIX%/}"
  [[ "${sim_prefix}" == "/" ]] && sim_prefix=""
  case "${RAW_LIDAR_TOPIC}" in
    ""|livox/lidar|/livox/lidar|/livox1/lidar)
      RAW_LIDAR_TOPIC="${sim_prefix}/livox/lidar"
      ;;
  esac
  case "${RAW_IMU_TOPIC}" in
    ""|livox/imu|/livox/imu|/livox1/imu)
      RAW_IMU_TOPIC="${sim_prefix}/livox/imu"
      ;;
  esac
fi

configure_livox_network

if [[ "${MODE}" == "real" && -n "${LIVOX_LIDAR2_IP}" ]]; then
  case "${RAW_LIDAR_TOPIC}" in
    ""|livox/lidar|/livox/lidar)
      RAW_LIDAR_TOPIC="/livox1/lidar"
      ;;
  esac
  case "${RAW_IMU_TOPIC}" in
    ""|livox/imu|/livox/imu)
      RAW_IMU_TOPIC="/livox1/imu"
      ;;
  esac
  if [[ -z "${RAW_LIDAR2_TOPIC}" ]]; then
    RAW_LIDAR2_TOPIC="/livox2/lidar"
  fi
  if [[ -z "${RAW_IMU2_TOPIC}" ]]; then
    RAW_IMU2_TOPIC="/livox2/imu"
  fi
fi

if [[ "${MODE}" == "real" && "${NO_DRIVERS}" != "true" ]]; then
  if [[ -n "${LIVOX_CONFIG_PATH}" ]]; then
    LIVOX_DRIVER_COMMAND=(
      "ros2"
      "run"
      "livox_ros_driver2"
      "livox_ros_driver2_node"
      "--ros-args"
      "-r"
      "__node:=livox_lidar_publisher"
      "-r"
      "livox/lidar:=${RAW_LIDAR_TOPIC:-/livox1/lidar}"
      "-r"
      "livox/imu:=${RAW_IMU_TOPIC:-/livox1/imu}"
      "-p"
      "xfer_format:=1"
      "-p"
      "multi_topic:=0"
      "-p"
      "data_src:=0"
      "-p"
      "publish_freq:=10.0"
      "-p"
      "output_data_type:=0"
      "-p"
      "frame_id:=${LIVOX_FRAME_ID:-livox_frame}"
      "-p"
      "lvx_file_path:=/tmp/autonomy_light_livox.lvx"
      "-p"
      "user_config_path:=${LIVOX_CONFIG_PATH}"
      "-p"
      "cmdline_input_bd_code:=livox0000000001"
    )
    if [[ -n "${LIVOX_CONFIG2_PATH}" && -n "${RAW_LIDAR2_TOPIC}" ]]; then
      LIVOX_DRIVER2_COMMAND=(
        "ros2"
        "run"
        "livox_ros_driver2"
        "livox_ros_driver2_node"
        "--ros-args"
        "-r"
        "__node:=livox_lidar_publisher_2"
        "-r"
        "livox/lidar:=${RAW_LIDAR2_TOPIC}"
        "-r"
        "livox/imu:=${RAW_IMU2_TOPIC:-/livox2/imu}"
        "-p"
        "xfer_format:=1"
        "-p"
        "multi_topic:=0"
        "-p"
        "data_src:=0"
        "-p"
        "publish_freq:=10.0"
        "-p"
        "output_data_type:=0"
        "-p"
        "frame_id:=${LIVOX_FRAME_ID:-livox_frame}_2"
        "-p"
        "lvx_file_path:=/tmp/autonomy_light_livox_2.lvx"
        "-p"
        "user_config_path:=${LIVOX_CONFIG2_PATH}"
        "-p"
        "cmdline_input_bd_code:=livox0000000002"
      )
    fi
  else
    case "${LIVOX_MODEL}" in
      mid360)
        livox_launch_file="msg_MID360_launch.py"
        ;;
      mid360s)
        livox_launch_file="msg_MID360s_launch.py"
        ;;
    esac
    LIVOX_DRIVER_COMMAND=(
      "ros2"
      "launch"
      "livox_ros_driver2"
      "${livox_launch_file}"
    )
  fi
  LIVOX_DRIVER_COMMAND_PARAM="$(
    /usr/bin/python3 - "${LIVOX_DRIVER_COMMAND[@]}" <<'PY'
import json
import sys

print(json.dumps(sys.argv[1:]))
PY
  )"
  if [[ "${#LIVOX_DRIVER2_COMMAND[@]}" -gt 0 ]]; then
    LIVOX_DRIVER2_COMMAND_PARAM="$(
      /usr/bin/python3 - "${LIVOX_DRIVER2_COMMAND[@]}" <<'PY'
import json
import sys

print(json.dumps(sys.argv[1:]))
PY
    )"
  fi
fi

EFFECTIVE_CONFIG_FILE="$(effective_config_path)"
write_effective_config \
  "${CONFIG_FILE}" \
  "${EFFECTIVE_CONFIG_FILE}" \
  "${LIVOX_DRIVER_COMMAND_PARAM:-}" \
  "${LIVOX_DRIVER2_COMMAND_PARAM:-}"

echo "autonomy-light mode=${MODE} config=${EFFECTIVE_CONFIG_FILE} source_config=${CONFIG_FILE}"
if [[ -n "${SAVED_MAP_FILE}" ]]; then
  echo "saved map mode: map=${SAVED_MAP_FILE}"
fi
if [[ "${MODE}" == "sim" ]]; then
  echo "simulation topics: lidar=${RAW_LIDAR_TOPIC} imu=${RAW_IMU_TOPIC}"
fi

AUTONOMY_LIGHT_COMMAND=(
  ros2 run autonomy_light autonomy_light
  --ros-args
  --params-file "${EFFECTIVE_CONFIG_FILE}"
  "$@"
)

if [[ "${VIS}" != "true" ]]; then
  exec "${AUTONOMY_LIGHT_COMMAND[@]}"
fi

VIS_SCRIPT="$(height_map_vis_script)"
if [[ -z "${VIS_ROS_DOMAIN_ID}" ]]; then
  VIS_ROS_DOMAIN_ID="$(read_external_ros_domain_config "${EFFECTIVE_CONFIG_FILE}")"
fi

cleanup() {
  local pid
  for pid in "${AUTONOMY_LIGHT_PID:-}" "${VIS_PID:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

"${AUTONOMY_LIGHT_COMMAND[@]}" &
AUTONOMY_LIGHT_PID="$!"

echo "height-map vis: topic=${VIS_TOPIC} fps=${VIS_FPS} scale=${VIS_SCALE} ROS_DOMAIN_ID=${VIS_ROS_DOMAIN_ID}"
(
  export ROS_DOMAIN_ID="${VIS_ROS_DOMAIN_ID}"
  exec "${VIS_SCRIPT}" --topic "${VIS_TOPIC}" --fps "${VIS_FPS}" --scale "${VIS_SCALE}"
) &
VIS_PID="$!"

wait "${AUTONOMY_LIGHT_PID}"
