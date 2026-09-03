#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/launch.sh [--real|--sim] [--no-drivers] [--no-nav2] [--rviz]
                   [--map DIRECTORY] [--mapping DIRECTORY]
                   [--lidar-type mid360|mid360s] [--lidar-interface NAME]
                   [--lidar-ip IPV4] [--jetson-ip IPV4]
                   [--status|--no-status] [--status-rate HZ] [--vis-height|--vis-height-3d]
                   [--goal-picker] [--check]

LiDAR-only bringup: Super-LIO retains the full global map internally and
publishes only a robot-local elevation crop.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_DIR="${ROOT}/include/parameter/config"
MODEL_DIR="${ROOT}/include/parameter/models"
ELEVATION_CONFIG="${CONFIG_DIR}/elevation_mapping.yaml"
LIO_CONFIG="${CONFIG_DIR}/super_lio_mid360.yaml"
NAV2_CONFIG="${CONFIG_DIR}/nav2_live.yaml"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
MODE=real
NO_DRIVERS=false
ENABLE_NAV2=true
ENABLE_RVIZ=false
ENABLE_3D=false
ENABLE_HEIGHT_GRID=false
ENABLE_GOAL_PICKER=false
ENABLE_STATUS=true
STATUS_RATE=2.0
CHECK_ONLY=false
MAP_DIRECTORY=""
MAPPING_DIRECTORY=""
LIDAR_TYPE_OVERRIDE=""
LIDAR_INTERFACE_OVERRIDE=""
LIDAR_IP_OVERRIDE=""
JETSON_IP_OVERRIDE=""
LIDAR_IP_WAS_OVERRIDDEN=false

prepend_runtime_path() {
  local variable_name="$1"
  local path="$2"
  local current_value="${!variable_name-}"
  export "${variable_name}=${path}${current_value:+:${current_value}}"
}

# setup_nav2.sh installs an unprivileged Nav2 runtime below .deps. Debian
# packages use the same layout below runtime/. Make either prefix visible to
# ament and the dynamic/Python loaders before any ROS subprocess is started.
LOCAL_NAV2_PREFIX=""
for candidate in \
  "${ROOT}/.deps/opt/ros/${ROS_DISTRO_NAME}" \
  "${ROOT}/runtime/opt/ros/${ROS_DISTRO_NAME}"; do
  if [[ -x "${candidate}/lib/nav2_controller/controller_server" ]]; then
    LOCAL_NAV2_PREFIX="${candidate}"
    break
  fi
done
if [[ -n "${LOCAL_NAV2_PREFIX}" ]]; then
  LOCAL_DEPENDENCY_ROOT="${LOCAL_NAV2_PREFIX%/opt/ros/${ROS_DISTRO_NAME}}"
  prepend_runtime_path AMENT_PREFIX_PATH "${LOCAL_NAV2_PREFIX}"
  prepend_runtime_path CMAKE_PREFIX_PATH "${LOCAL_NAV2_PREFIX}"
  prepend_runtime_path LD_LIBRARY_PATH "${LOCAL_NAV2_PREFIX}/lib"
  for library_path in \
    "${LOCAL_NAV2_PREFIX}"/lib/*-linux-gnu \
    "${LOCAL_DEPENDENCY_ROOT}/usr/lib" \
    "${LOCAL_DEPENDENCY_ROOT}"/usr/lib/*-linux-gnu; do
    [[ ! -d "${library_path}" ]] || prepend_runtime_path LD_LIBRARY_PATH "${library_path}"
  done
  [[ ! -d "${LOCAL_NAV2_PREFIX}/bin" ]] || \
    prepend_runtime_path PATH "${LOCAL_NAV2_PREFIX}/bin"
  for python_path in \
    "${LOCAL_NAV2_PREFIX}"/local/lib/python*/dist-packages \
    "${LOCAL_NAV2_PREFIX}"/lib/python*/site-packages; do
    [[ ! -d "${python_path}" ]] || prepend_runtime_path PYTHONPATH "${python_path}"
  done
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --real) MODE=real ;;
    --sim) MODE=sim ;;
    --no-drivers) NO_DRIVERS=true ;;
    --no-nav2) ENABLE_NAV2=false ;;
    --rviz) ENABLE_RVIZ=true ;;
    --status) ENABLE_STATUS=true ;;
    --no-status) ENABLE_STATUS=false ;;
    --status-rate) STATUS_RATE="${2:?--status-rate requires a value}"; shift ;;
    --vis-height) ENABLE_HEIGHT_GRID=true ;;
    --vis-height-3d) ENABLE_3D=true ;;
    --goal-picker) ENABLE_GOAL_PICKER=true ;;
    --map) MAP_DIRECTORY="${2:?--map requires a directory}"; shift ;;
    --mapping) MAPPING_DIRECTORY="${2:?--mapping requires a directory}"; shift ;;
    --lidar-type) LIDAR_TYPE_OVERRIDE="${2:?--lidar-type requires a value}"; shift ;;
    --lidar-interface) LIDAR_INTERFACE_OVERRIDE="${2:?--lidar-interface requires a value}"; shift ;;
    --lidar-ip) LIDAR_IP_OVERRIDE="${2:?--lidar-ip requires a value}"; LIDAR_IP_WAS_OVERRIDDEN=true; shift ;;
    --jetson-ip) JETSON_IP_OVERRIDE="${2:?--jetson-ip requires a value}"; shift ;;
    --mid360) LIDAR_TYPE_OVERRIDE=MID360 ;;
    --mid360s) LIDAR_TYPE_OVERRIDE=MID360S ;;
    --check) CHECK_ONLY=true ;;
    *) echo "error: unsupported option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ -z "${MAP_DIRECTORY}" || -z "${MAPPING_DIRECTORY}" ]] || {
  echo "error: --map and --mapping cannot be combined" >&2
  exit 2
}

CONFIG_EXPORTS="$(/usr/bin/python3 - "${ELEVATION_CONFIG}" "${LIO_CONFIG}" <<'PY'
import shlex
import sys
from math import asin, atan2, cos, sin
import yaml

elevation_path, lio_path = sys.argv[1:]
with open(elevation_path, encoding='utf-8') as stream:
    elevation = yaml.safe_load(stream) or {}
with open(lio_path, encoding='utf-8') as stream:
    lio = yaml.safe_load(stream) or {}
bringup = elevation['bringup']['ros__parameters']
mapping = elevation['elevation_mapping']['ros__parameters']
restorer = elevation['elevation_restorer']['ros__parameters']
external_dds_bridge = elevation['external_dds_bridge']['ros__parameters']
livox = lio['bringup']['ros__parameters']['livox']
base_to_lidar_xyz = bringup['base_to_lidar.xyz']
base_to_lidar_rpy = bringup['base_to_lidar.rpy']
if len(base_to_lidar_xyz) != 3 or len(base_to_lidar_rpy) != 3:
    raise ValueError('base_to_lidar.xyz and base_to_lidar.rpy must each contain three values')

# launch uses lidar_link -> base_link because Super-LIO provides map -> lidar_link.
# Invert the configured base_link -> lidar_link pose instead of duplicating the
# mounting values as hand-maintained command-line constants.
roll, pitch, yaw = map(float, base_to_lidar_rpy)
half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
cr, sr = cos(half_roll), sin(half_roll)
cp, sp = cos(half_pitch), sin(half_pitch)
cy, sy = cos(half_yaw), sin(half_yaw)
qw = cr * cp * cy + sr * sp * sy
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy
tx, ty, tz = map(float, base_to_lidar_xyz)
inverse_x = -(1.0 - 2.0 * (qy * qy + qz * qz)) * tx - 2.0 * (qx * qy + qz * qw) * ty - 2.0 * (qx * qz - qy * qw) * tz
inverse_y = -2.0 * (qx * qy - qz * qw) * tx - (1.0 - 2.0 * (qx * qx + qz * qz)) * ty - 2.0 * (qy * qz + qx * qw) * tz
inverse_z = -2.0 * (qx * qz + qy * qw) * tx - 2.0 * (qy * qz - qx * qw) * ty - (1.0 - 2.0 * (qx * qx + qy * qy)) * tz
inverse_qx, inverse_qy, inverse_qz = -qx, -qy, -qz
inverse_roll = atan2(2.0 * (qw * inverse_qx + inverse_qy * inverse_qz),
                    1.0 - 2.0 * (inverse_qx * inverse_qx + inverse_qy * inverse_qy))
inverse_pitch = asin(max(-1.0, min(1.0, 2.0 * (qw * inverse_qy - inverse_qz * inverse_qx))))
inverse_yaw = atan2(2.0 * (qw * inverse_qz + inverse_qx * inverse_qy),
                   1.0 - 2.0 * (inverse_qy * inverse_qy + inverse_qz * inverse_qz))
values = {
    'INTERNAL_ROS_DOMAIN_ID': bringup['internal_ros_domain_id'],
    'EXTERNAL_ROS_DOMAIN_ID': bringup['external_ros_domain_id'],
    'EXTERNAL_HEIGHT_MAP_DDS_DOMAIN_ID': external_dds_bridge['height_map_external_domain_id'],
    'DEBUG_LOCAL_MAP_ROS_DOMAIN_ID': bringup['debug_local_map_ros_domain_id'],
    'EXTERNAL_DDS_INTERFACE': bringup['external_dds.interface'],
    'EXTERNAL_DDS_LOCAL_IP': bringup['external_dds.local_ip'],
    'EXTERNAL_DDS_PEER_IP': bringup['external_dds.peer_ip'],
    'LIDAR_FRAME': bringup['lidar_frame'],
    'BASE_FRAME': bringup['base_frame'],
    'LIDAR_TO_BASE_X': inverse_x,
    'LIDAR_TO_BASE_Y': inverse_y,
    'LIDAR_TO_BASE_Z': inverse_z,
    'LIDAR_TO_BASE_ROLL': inverse_roll,
    'LIDAR_TO_BASE_PITCH': inverse_pitch,
    'LIDAR_TO_BASE_YAW': inverse_yaw,
    'ELEVATION_RATE': mapping['publish_rate_hz'],
    'ELEVATION_RESTORER_MODEL': restorer['model_path'],
    'ELEVATION_OUTPUT_MODE': restorer.get('output_mode', 'restored'),
    'LIVOX_MODEL': livox['model'],
    'LIVOX_INTERFACE': livox['interface'],
    'LIVOX_LIDAR_IP': livox['lidar_ip'],
    'LIVOX_JETSON_IP': livox['jetson_ip'],
    'LIVOX_PUBLISH_FREQUENCY_HZ': livox['publish_frequency_hz'],
    'LIVOX_FRAME_ID': livox['frame_id'],
}
for key, value in values.items():
    print(f'{key}={shlex.quote(str(value))}')
PY
)"
source /dev/stdin <<<"${CONFIG_EXPORTS}"
[[ "${DEBUG_LOCAL_MAP_ROS_DOMAIN_ID}" == "${INTERNAL_ROS_DOMAIN_ID}" ]] || {
  echo "error: debug_local_map_ros_domain_id must equal internal_ros_domain_id" >&2
  exit 2
}
[[ "${ELEVATION_OUTPUT_MODE}" == raw || "${ELEVATION_OUTPUT_MODE}" == restored ]] || {
  echo "error: elevation_restorer.output_mode must be raw or restored" >&2
  exit 2
}
if [[ -n "${LIDAR_TYPE_OVERRIDE}" ]]; then LIVOX_MODEL="${LIDAR_TYPE_OVERRIDE^^}"; fi
if [[ -n "${LIDAR_INTERFACE_OVERRIDE}" ]]; then LIVOX_INTERFACE="${LIDAR_INTERFACE_OVERRIDE}"; fi
if [[ -n "${LIDAR_IP_OVERRIDE}" ]]; then LIVOX_LIDAR_IP="${LIDAR_IP_OVERRIDE}"; fi
if [[ -n "${JETSON_IP_OVERRIDE}" ]]; then LIVOX_JETSON_IP="${JETSON_IP_OVERRIDE}"; fi
[[ "${LIVOX_MODEL}" == MID360 || "${LIVOX_MODEL}" == MID360S ]] || {
  echo "error: lidar type must be MID360 or MID360S" >&2; exit 2;
}

validate_livox_network() {
  local interface_addresses
  local route
  command -v ip >/dev/null 2>&1 || {
    echo "error: iproute2 is required to validate the LiDAR interface" >&2
    return 1
  }
  ip link show dev "${LIVOX_INTERFACE}" >/dev/null 2>&1 || {
    echo "error: LiDAR interface does not exist: ${LIVOX_INTERFACE}" >&2
    return 1
  }
  interface_addresses="$(ip -4 -o address show dev "${LIVOX_INTERFACE}" |
    awk '{sub(/\/.*/, "", $4); print $4}')"
  grep -Fxq "${LIVOX_JETSON_IP}" <<<"${interface_addresses}" || {
    echo "error: ${LIVOX_JETSON_IP} is not assigned to LiDAR interface ${LIVOX_INTERFACE}" >&2
    return 1
  }
  route="$(ip -4 route get "${LIVOX_LIDAR_IP}" 2>/dev/null || true)"
  [[ " ${route} " == *" dev ${LIVOX_INTERFACE} "* ]] || {
    echo "error: route to LiDAR ${LIVOX_LIDAR_IP} does not use ${LIVOX_INTERFACE}" >&2
    return 1
  }
}

validate_external_dds_network() {
  local interface_addresses
  local route
  command -v ip >/dev/null 2>&1 || {
    echo "error: iproute2 is required to validate the control-SBC interface" >&2
    return 1
  }
  ip link show dev "${EXTERNAL_DDS_INTERFACE}" >/dev/null 2>&1 || {
    echo "error: control-SBC interface does not exist: ${EXTERNAL_DDS_INTERFACE}" >&2
    return 1
  }
  interface_addresses="$(ip -4 -o address show dev "${EXTERNAL_DDS_INTERFACE}" |
    awk '{sub(/\/.*/, "", $4); print $4}')"
  grep -Fxq "${EXTERNAL_DDS_LOCAL_IP}" <<<"${interface_addresses}" || {
    echo "error: ${EXTERNAL_DDS_LOCAL_IP} is not assigned to ${EXTERNAL_DDS_INTERFACE}" >&2
    return 1
  }
  route="$(ip -4 route get "${EXTERNAL_DDS_PEER_IP}" 2>/dev/null || true)"
  [[ " ${route} " == *" dev ${EXTERNAL_DDS_INTERFACE} "* &&
     " ${route} " == *" src ${EXTERNAL_DDS_LOCAL_IP} "* ]] || {
    echo "error: route to control SBC ${EXTERNAL_DDS_PEER_IP} must use ${EXTERNAL_DDS_INTERFACE} " \
         "with source ${EXTERNAL_DDS_LOCAL_IP}" >&2
    return 1
  }
}

configure_external_dds_transport() {
  if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    echo "autonomy-light: using caller-supplied CYCLONEDDS_URI=${CYCLONEDDS_URI}"
    return 0
  fi

  EXTERNAL_DDS_RUNTIME_CONFIG="$(mktemp "${TMPDIR:-/tmp}/autonomy_light_cyclonedds.XXXXXX.xml")"
  /usr/bin/python3 - "${EXTERNAL_DDS_RUNTIME_CONFIG}" "${EXTERNAL_DDS_LOCAL_IP}" \
    "${EXTERNAL_DDS_PEER_IP}" <<'PY'
import sys
from xml.sax.saxutils import escape

path, local_ip, peer_ip = sys.argv[1:]
xml = f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="{escape(local_ip)}" priority="default" multicast="false" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <MulticastRecvNetworkInterfaceAddresses>none</MulticastRecvNetworkInterfaceAddresses>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="{escape(peer_ip)}" />
        <!-- The control SBC's raw HeightMap reader uses DDS domain 1 with a
             dynamically assigned unicast locator. Its well-known discovery
             socket is 7650, so seed endpoint discovery there as well. -->
        <Peer address="{escape(peer_ip)}:7650" />
      </Peers>
    </Discovery>
    <SharedMemory>
      <Enable>false</Enable>
      <LogLevel>warn</LogLevel>
    </SharedMemory>
  </Domain>
</CycloneDDS>
'''
with open(path, 'w', encoding='utf-8') as stream:
    stream.write(xml)
PY
  export CYCLONEDDS_URI="file://${EXTERNAL_DDS_RUNTIME_CONFIG}"
}

# A cloned SD image can retain an old LiDAR IP.  Prefer an observed Livox
# neighbour only when the operator did not explicitly supply --lidar-ip.
# The helper performs no subnet scan and never changes NIC configuration.
resolve_livox_ip() {
  [[ "${LIDAR_IP_WAS_OVERRIDDEN}" == false ]] || return 0
  local detected_ip
  detected_ip="$(/usr/bin/python3 "${SCRIPT_DIR}/find_livox_ip.py" \
    --interface "${LIVOX_INTERFACE}" --hint "${LIVOX_LIDAR_IP}" --probe-hint 2>/dev/null || true)"
  [[ -z "${detected_ip}" || "${detected_ip}" == "${LIVOX_LIDAR_IP}" ]] && return 0
  echo "autonomy-light: using observed Livox IP ${detected_ip} instead of configured ${LIVOX_LIDAR_IP}"
  LIVOX_LIDAR_IP="${detected_ip}"
}

validate_nav2_runtime() {
  local package
  command -v ros2 >/dev/null 2>&1 || {
    echo "error: ros2 is not available; source the ROS and workspace setup files" >&2
    return 1
  }
  for package in \
    nav2_behaviors \
    nav2_bt_navigator \
    nav2_controller \
    nav2_lifecycle_manager \
    nav2_mppi_controller \
    nav2_planner \
    nav2_smac_planner \
    nav2_velocity_smoother; do
    ros2 pkg prefix "${package}" >/dev/null 2>&1 || {
      echo "error: required Nav2 package is unavailable: ${package}" >&2
      echo "       run ./scripts/build.sh --setup-only, then retry" >&2
      return 1
    }
  done
}

validate_elevation_restorer_runtime() {
  local model_path="${ELEVATION_RESTORER_MODEL}"
  [[ "${model_path}" == /* ]] || model_path="${MODEL_DIR}/${model_path}"
  [[ -f "${model_path}" ]] || {
    echo "error: elevation-restorer ONNX model not found: ${model_path}" >&2
    return 1
  }
  /usr/bin/python3 - "${model_path}" <<'PY'
import sys
try:
    import numpy
    import onnxruntime as ort
except ImportError as error:
    raise SystemExit("error: elevation restorer requires Python onnxruntime and numpy: " + str(error))
session = ort.InferenceSession(sys.argv[1], providers=['CPUExecutionProvider'])
inputs = {item.name: (item.type, item.shape) for item in session.get_inputs()}
outputs = {item.name: (item.type, item.shape) for item in session.get_outputs()}
expected_inputs = {
    'noisy_elevation': ('tensor(float)', [1, 144]),
}
expected_outputs = {'elevation': ('tensor(float)', [1, 144])}
if inputs != expected_inputs or outputs != expected_outputs:
    raise SystemExit(f'error: unexpected elevation-restorer ONNX contract: inputs={inputs}, outputs={outputs}')
PY
}

if [[ "${CHECK_ONLY}" == true ]]; then
  /usr/bin/python3 - "${ELEVATION_CONFIG}" "${LIO_CONFIG}" "${NAV2_CONFIG}" <<'PY'
import sys, yaml
for path in sys.argv[1:]:
    with open(path, encoding='utf-8') as stream: yaml.safe_load(stream)
print('autonomy-light: LiDAR-only configuration is valid')
PY
  exit 0
fi

if [[ "${ENABLE_NAV2}" == true ]]; then
  validate_nav2_runtime
fi
if [[ "${ELEVATION_OUTPUT_MODE}" == restored ]]; then
  validate_elevation_restorer_runtime
fi

if [[ "${MODE}" == real && "${NO_DRIVERS}" == false ]]; then
  resolve_livox_ip
  validate_livox_network
fi
if [[ "${MODE}" == real ]]; then
  validate_external_dds_network
  EXTERNAL_DDS_RUNTIME_CONFIG=""
  configure_external_dds_transport
fi

export ROS_DOMAIN_ID="${INTERNAL_ROS_DOMAIN_ID}"
RUNTIME_LOG_DIR="${ROOT}/log/runtime_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "${RUNTIME_LOG_DIR}"
declare -a PIDS=()
declare -a AUXILIARY_PIDS=()
LAST_PID=""
LIVOX_RUNTIME_CONFIG=""
# Bash starts non-interactive background jobs with SIGINT ignored.  `setsid`
# creates a new process group but preserves that signal disposition, so reset
# it in a tiny exec wrapper before launching each ROS process.
RESET_INTERRUPT_AND_EXEC=$'import os\nimport signal\nimport sys\n\nsignal.signal(signal.SIGINT, signal.SIG_DFL)\nsignal.signal(signal.SIGQUIT, signal.SIG_DFL)\nos.execvp(sys.argv[1], sys.argv[1:])\n'

start() {
  local label="$1"; shift
  local log_file="${RUNTIME_LOG_DIR}/${#PIDS[@]}_${label//[^a-zA-Z0-9._-]/_}.log"
  echo "autonomy-light: starting ${label}"
  # Preserve a per-process log file, but do not hide ROS output from an
  # interactive launcher.  The `-t` guard keeps detached/service launches
  # quiet and avoids sending a large stream into a redirected manager log.
  if [[ -t 1 ]]; then
    setsid /usr/bin/python3 -c "${RESET_INTERRUPT_AND_EXEC}" "$@" \
      > >(tee -a "${log_file}") 2>&1 &
  else
    setsid /usr/bin/python3 -c "${RESET_INTERRUPT_AND_EXEC}" "$@" \
      >"${log_file}" 2>&1 &
  fi
  LAST_PID=$!
  PIDS+=("${LAST_PID}")
}
start_terminal() {
  local label="$1"; shift
  echo "autonomy-light: starting ${label}"
  if [[ -t 1 ]]; then
    setsid /usr/bin/python3 -c "${RESET_INTERRUPT_AND_EXEC}" "$@" &
  else
    local log_file="${RUNTIME_LOG_DIR}/terminal_${label//[^a-zA-Z0-9._-]/_}.log"
    setsid /usr/bin/python3 -c "${RESET_INTERRUPT_AND_EXEC}" "$@" \
      >"${log_file}" 2>&1 &
  fi
  LAST_PID=$!
  AUXILIARY_PIDS+=("${LAST_PID}")
}

signal_process_groups() {
  local signal="$1"
  local pid
  for pid in "${PIDS[@]}" "${AUXILIARY_PIDS[@]}"; do
    kill -s "${signal}" -- "-${pid}" 2>/dev/null || true
  done
}

process_groups_are_alive() {
  local pid
  for pid in "${PIDS[@]}" "${AUXILIARY_PIDS[@]}"; do
    # `kill -0` also succeeds for an exited, unreaped group leader (a zombie).
    # Only wait for groups that still contain a running or stoppable process.
    if ps -eo pgrp=,stat= | awk -v process_group="${pid}" '
      $1 == process_group && $2 !~ /^Z/ { found = 1; exit }
      END { exit !found }
    '; then
      return 0
    fi
  done
  return 1
}

wait_for_process_groups() {
  local timeout_seconds="$1"
  local deadline=$((SECONDS + timeout_seconds))
  while process_groups_are_alive && (( SECONDS < deadline )); do
    sleep 0.1
  done
}

cleanup() {
  trap - EXIT INT TERM
  signal_process_groups INT
  wait_for_process_groups 5
  if process_groups_are_alive; then
    echo "autonomy-light: processes did not stop after 5 seconds; sending SIGTERM" >&2
    signal_process_groups TERM
    wait_for_process_groups 3
  fi
  if process_groups_are_alive; then
    echo "autonomy-light: forcing remaining processes to stop" >&2
    signal_process_groups KILL
  fi
  local pid
  for pid in "${PIDS[@]}" "${AUXILIARY_PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
  [[ -z "${LIVOX_RUNTIME_CONFIG}" ]] || rm -f -- "${LIVOX_RUNTIME_CONFIG}"
  [[ -z "${EXTERNAL_DDS_RUNTIME_CONFIG}" ]] || rm -f -- "${EXTERNAL_DDS_RUNTIME_CONFIG}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "autonomy-light: LiDAR-only Super-LIO elevation"
if [[ "${MODE}" == real && "${NO_DRIVERS}" == false ]]; then
  echo "  Livox ${LIVOX_LIDAR_IP} via ${LIVOX_INTERFACE} (${LIVOX_JETSON_IP})"
fi
echo "  /lio global keyframes -> local PointCloud2 debug map (domain ${INTERNAL_ROS_DOMAIN_ID})"
echo "  selected height map (${ELEVATION_OUTPUT_MODE}): /autonomy_light/elevation_map (domain ${INTERNAL_ROS_DOMAIN_ID})"
echo "  control DDS egress: height_map (core_dds::HeightMap, domain ${EXTERNAL_HEIGHT_MAP_DDS_DOMAIN_ID}) + /control_command/autopilot (core/msg/CommandCore, domain ${EXTERNAL_ROS_DOMAIN_ID})"
echo "  control DDS transport: ${EXTERNAL_DDS_LOCAL_IP} -> ${EXTERNAL_DDS_PEER_IP} via ${EXTERNAL_DDS_INTERFACE} (unicast)"

LIO_ARGS=(--ros-args --params-file "${LIO_CONFIG}")
if [[ "${MODE}" == sim ]]; then
  LIO_ARGS+=(-p use_sim_time:=true -p lio.ros.lidar_topic:=/f4/lidar/points \
             -p lio.ros.imu_topic:=/f4/lidar/imu -p lio.sensor.lidar_type:=3)
fi
if [[ -n "${MAPPING_DIRECTORY}" ]]; then
  mkdir -p "${MAPPING_DIRECTORY}"
  LIO_ARGS+=(-p lio.map.save_map:=true -p "lio.map.save_map_dir:=${MAPPING_DIRECTORY}" \
             -p lio.map.map_name:=localization.pcd)
fi
if [[ -n "${MAP_DIRECTORY}" ]]; then
  LIO_ARGS+=(-p lio.slam.enable:=false -p "lio.map.save_map_dir:=${MAP_DIRECTORY}" \
             -p lio.map.map_name:=localization.pcd)
fi

if [[ "${MODE}" == real && "${NO_DRIVERS}" == false ]]; then
  LIVOX_RUNTIME_CONFIG="$(mktemp "${TMPDIR:-/tmp}/autonomy_light_livox.XXXXXX.json")"
  /usr/bin/python3 - "${LIVOX_RUNTIME_CONFIG}" "${LIVOX_MODEL}" "${LIVOX_LIDAR_IP}" "${LIVOX_JETSON_IP}" <<'PY'
import json, sys
path, model, lidar_ip, host_ip = sys.argv[1:]
ports = {'cmd_data_port': 56100, 'push_msg_port': 56200, 'point_data_port': 56300,
         'imu_data_port': 56400, 'log_data_port': 56500}
host_ports = {'cmd_data_port': 56101, 'push_msg_port': 56201, 'point_data_port': 56301,
              'imu_data_port': 56401, 'log_data_port': 56501}
if model == 'MID360':
    host = {'cmd_data_ip': host_ip, 'push_msg_ip': host_ip, 'point_data_ip': host_ip,
            'imu_data_ip': host_ip, 'log_data_ip': '', **host_ports}
    device = {'MID360': {'lidar_net_info': ports, 'host_net_info': host}}
else:
    device = {'Mid360s': {'lidar_net_info': ports, 'host_net_info': [{'host_ip': host_ip, **host_ports}]}}
with open(path, 'w', encoding='utf-8') as stream:
    json.dump({'lidar_summary_info': {'lidar_type': 8}, **device,
               'lidar_configs': [{'ip': lidar_ip, 'pcl_data_type': 1, 'pattern_mode': 0,
                                  'extrinsic_parameter': {'roll': 0, 'pitch': 0, 'yaw': 0, 'x': 0, 'y': 0, 'z': 0}}]}, stream)
PY
  start "livox_driver" ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p "publish_freq:=${LIVOX_PUBLISH_FREQUENCY_HZ}" \
    -p output_data_type:=0 -p "frame_id:=${LIVOX_FRAME_ID}" -p "user_config_path:=${LIVOX_RUNTIME_CONFIG}" \
    -p cmdline_input_bd_code:=livox0000000001
fi
if [[ "${NO_DRIVERS}" == false ]]; then
  LIO_EXECUTABLE=super_lio_node
  [[ -z "${MAP_DIRECTORY}" ]] || LIO_EXECUTABLE=relocation_node
  start "super_lio" ros2 run super_lio "${LIO_EXECUTABLE}" "${LIO_ARGS[@]}"
fi

# Super-LIO owns map -> lidar_link. This static inverse edge completes map -> lidar_link -> base_link.
start "lidar_to_base_tf" ros2 run tf2_ros static_transform_publisher \
  --x "${LIDAR_TO_BASE_X}" --y "${LIDAR_TO_BASE_Y}" --z "${LIDAR_TO_BASE_Z}" \
  --roll "${LIDAR_TO_BASE_ROLL}" --pitch "${LIDAR_TO_BASE_PITCH}" --yaw "${LIDAR_TO_BASE_YAW}" \
  --frame-id "${LIDAR_FRAME}" --child-frame-id "${BASE_FRAME}"

ELEVATION_ARGS=(--ros-args --params-file "${ELEVATION_CONFIG}")
[[ "${MODE}" != sim ]] || ELEVATION_ARGS+=(-p use_sim_time:=true)
AUTONOMY_ARGS=(--ros-args --params-file "${ELEVATION_CONFIG}" --params-file "${NAV2_CONFIG}")
[[ "${MODE}" != sim ]] || AUTONOMY_ARGS+=(-p use_sim_time:=true)
start "autonomy_light" ros2 run autonomy_light autonomy_light_node "${AUTONOMY_ARGS[@]}"
start "elevation_restorer" ros2 run autonomy_light elevation_restorer.py "${ELEVATION_ARGS[@]}"
if [[ "${ENABLE_NAV2}" == true ]]; then
  NAV2_USE_SIM_TIME=false
  [[ "${MODE}" != sim ]] || NAV2_USE_SIM_TIME=true
  start "nav2" bash "${SCRIPT_DIR}/nav2_wait_and_launch.sh" "${NAV2_CONFIG}" "${NAV2_USE_SIM_TIME}" /lio/odom 120
fi
if [[ "${ENABLE_STATUS}" == true ]]; then
  STATUS_ARGS=(--rate "${STATUS_RATE}")
  [[ "${ENABLE_HEIGHT_GRID}" == true ]] && STATUS_ARGS+=(--show-height-grid)
  start_terminal "terminal_status" /usr/bin/python3 "${SCRIPT_DIR}/monitor_autonomy_state.py" \
    "${STATUS_ARGS[@]}"
fi
if [[ "${ENABLE_RVIZ}" == true ]]; then start "rviz" rviz2 -d "${CONFIG_DIR}/elevation_mapping.rviz"; fi
if [[ "${ENABLE_3D}" == true ]]; then
  start "height_map_3d" /usr/bin/python3 "${SCRIPT_DIR}/visualize_height_map_3d.py" \
    --rate "${ELEVATION_RATE}" --topic /autonomy_light/elevation_map
fi
if [[ "${ENABLE_GOAL_PICKER}" == true ]]; then
  start_terminal "goal_picker" /usr/bin/python3 "${SCRIPT_DIR}/goal_picker.py"
fi
wait -n "${PIDS[@]}"
