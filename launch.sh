#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./launch.sh [--real|--sim] [--no-drivers] [--no-nav2] [--rviz] [--map FILE] [--mapping [FILE]]
                   [--lidar-type mid360|mid360s] [--lidar-ip IPV4] [--jetson-ip IPV4]
                   [--vis-rate HZ] [--no-status]

Starts full Super-LIO SLAM, camera-only elevation mapping, and Nav2.

  --real                  Start Livox and RealSense drivers (default).
  --sim                   Use /f4/lidar/points and /f4/lidar/imu.
  --no-drivers            Use externally running drivers and Super-LIO.
  --no-nav2               Do not start Nav2 or autopilot command output.
  --rviz                  Start RViz2.
  --map FILE              Start Super-LIO relocation with a saved PCD.
  --mapping [FILE]        Save the full-SLAM PCD on shutdown. Without FILE,
                          writes maps/super_lio_map_<timestamp>.pcd.
  --lidar-type TYPE       Override YAML Livox model: mid360 or mid360s.
  --lidar-ip IPV4         Override bringup.livox.lidar_ip for this run.
  --jetson-ip IPV4        Override bringup.livox.jetson_ip for this run.
  --mid360|--mid360s      Backward-compatible aliases for --lidar-type.
  --vis-rate HZ           Terminal status-table refresh rate (default: 2.0 Hz).
  --no-status             Do not start the terminal status table.
  --check                 Validate the three configuration files and exit.
EOF
}

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ELEVATION_CONFIG="${ROOT}/config/elevation_mapping.yaml"
LIO_CONFIG="${ROOT}/config/super_lio_mid360.yaml"
NAV2_CONFIG="${ROOT}/config/nav2_live.yaml"
MODE="real"
NO_DRIVERS="false"
ENABLE_NAV2="true"
ENABLE_RVIZ="false"
CHECK_ONLY="false"
LIDAR_TYPE_OVERRIDE=""
LIDAR_IP_OVERRIDE=""
JETSON_IP_OVERRIDE=""
LIVOX_RUNTIME_CONFIG=""
VIS_RATE="2.0"
ENABLE_STATUS="true"
RUNTIME_LOG_DIR=""
MAP_FILE=""
MAPPING_OUTPUT=""
MAPPING_TIMESTAMP=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --real) MODE="real" ;;
    --sim) MODE="sim" ;;
    --no-drivers) NO_DRIVERS="true" ;;
    --no-nav2) ENABLE_NAV2="false" ;;
    --rviz) ENABLE_RVIZ="true" ;;
    --check) CHECK_ONLY="true" ;;
    --lidar-type) LIDAR_TYPE_OVERRIDE="${2:?--lidar-type requires mid360 or mid360s}"; shift ;;
    --lidar-type=*) LIDAR_TYPE_OVERRIDE="${1#*=}" ;;
    --lidar-ip) LIDAR_IP_OVERRIDE="${2:?--lidar-ip requires an IPv4 address}"; shift ;;
    --lidar-ip=*) LIDAR_IP_OVERRIDE="${1#*=}" ;;
    --jetson-ip) JETSON_IP_OVERRIDE="${2:?--jetson-ip requires an IPv4 address}"; shift ;;
    --jetson-ip=*) JETSON_IP_OVERRIDE="${1#*=}" ;;
    --mid360) LIDAR_TYPE_OVERRIDE="MID360" ;;
    --mid360s) LIDAR_TYPE_OVERRIDE="MID360S" ;;
    --vis-rate) VIS_RATE="${2:?--vis-rate requires a positive frequency}"; shift ;;
    --vis-rate=*) VIS_RATE="${1#*=}" ;;
    --no-status) ENABLE_STATUS="false" ;;
    --map) MAP_FILE="${2:?--map requires a PCD file}"; shift ;;
    --mapping)
      if [[ $# -gt 1 && "${2}" != --* ]]; then
        MAPPING_OUTPUT="$2"
        shift
      else
        MAPPING_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
        MAPPING_OUTPUT="${PWD}/maps/super_lio_map_${MAPPING_TIMESTAMP}.pcd"
      fi
      ;;
    --mapping=*) MAPPING_OUTPUT="${1#*=}" ;;
    *) echo "error: unsupported option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ -f "${ELEVATION_CONFIG}" && -f "${LIO_CONFIG}" && -f "${NAV2_CONFIG}" ]] || {
  echo "error: config/elevation_mapping.yaml, config/super_lio_mid360.yaml, and config/nav2_live.yaml are required" >&2
  exit 1
}
STATUS_SCRIPT="${ROOT}/scripts/monitor_autonomy_state.py"
[[ -f "${STATUS_SCRIPT}" ]] || { echo "error: terminal status monitor is missing: ${STATUS_SCRIPT}" >&2; exit 1; }
[[ -z "${MAP_FILE}" || -z "${MAPPING_OUTPUT}" ]] || {
  echo "error: --map and --mapping cannot be used together" >&2
  exit 2
}
[[ -z "${MAP_FILE}" || -f "${MAP_FILE}" ]] || { echo "error: map not found: ${MAP_FILE}" >&2; exit 1; }

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[[ -f "${ROS_SETUP}" ]] || { echo "error: ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
[[ -f "${ROOT}/install/setup.bash" ]] && source "${ROOT}/install/setup.bash"
set -u

if [[ "${ENABLE_STATUS}" == "true" ]]; then
  /usr/bin/python3 - "${VIS_RATE}" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError as error:
    raise SystemExit("error: --vis-rate must be a positive finite number") from error
if not math.isfinite(value) or value <= 0.0:
    raise SystemExit("error: --vis-rate must be a positive finite number")
PY
fi

# Read the few bringup values that are not ROS node parameters.  Everything
# else is passed directly from one of the three YAML files above.
source <(/usr/bin/python3 - "${ELEVATION_CONFIG}" "${ROOT}" <<'PY'
from collections import deque
from pathlib import Path
import math
import shlex
import sys
import xml.etree.ElementTree as element_tree
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}
bringup = config.get("bringup", {})
bringup = bringup.get("ros__parameters", {})
merge = config.get("observation_merge", {})
merge = merge.get("ros__parameters", {})
source_names = merge.get("inputs", [])
source_definitions = bringup.get("realsense_cameras", {})

def required(mapping, key):
    value = mapping.get(key)
    if value in (None, ""):
        raise SystemExit(f"error: elevation bringup.{key} is required")
    return value

if not isinstance(source_names, list) or not source_names:
    raise SystemExit("error: observation_merge.inputs must select at least one camera")
if not isinstance(source_definitions, dict):
    raise SystemExit("error: bringup.realsense_cameras must be a mapping")

def source_parameter(source_name, suffix):
    return merge.get(f"sources.{source_name}.{suffix}")

def camera_value(source_name, camera, key):
    value = camera.get(key)
    if value in (None, ""):
        raise SystemExit(
            f"error: selected camera '{source_name}' requires "
            f"bringup.realsense_cameras.{source_name}.{key}"
        )
    return value

def shell_array(name, values):
    print(f"{name}=({' '.join(shlex.quote(str(value)) for value in values)})")

def numeric_vector(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise SystemExit(f"error: {label} must contain exactly three numbers")
    try:
        result = [float(component) for component in value]
    except (TypeError, ValueError) as error:
        raise SystemExit(f"error: {label} must contain exactly three numbers") from error
    if not all(math.isfinite(component) for component in result):
        raise SystemExit(f"error: {label} must be finite")
    return result

def urdf_vector(value, label):
    try:
        result = [float(component) for component in value.split()]
    except (AttributeError, ValueError) as error:
        raise SystemExit(f"error: {label} must contain exactly three numbers") from error
    if len(result) != 3 or not all(math.isfinite(component) for component in result):
        raise SystemExit(f"error: {label} must contain exactly three finite numbers")
    return result

def identity_transform():
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0)

def multiply_matrix(left, right):
    return tuple(tuple(sum(left[row][index] * right[index][column] for index in range(3))
                       for column in range(3)) for row in range(3))

def multiply_vector(matrix, vector):
    return tuple(sum(matrix[row][index] * vector[index] for index in range(3))
                 for row in range(3))

def compose_transform(left, right):
    left_rotation, left_translation = left
    right_rotation, right_translation = right
    rotated_translation = multiply_vector(left_rotation, right_translation)
    return (multiply_matrix(left_rotation, right_rotation),
            tuple(left_translation[index] + rotated_translation[index] for index in range(3)))

def inverse_transform(transform):
    rotation, translation = transform
    inverse_rotation = tuple(tuple(rotation[column][row] for column in range(3)) for row in range(3))
    return inverse_rotation, tuple(-value for value in multiply_vector(inverse_rotation, translation))

def origin_transform(xyz, rpy):
    roll, pitch, yaw = rpy
    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
         cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll),
        (sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
         sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll),
        (-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll),
    )
    return rotation, tuple(xyz)

def transform_rpy(transform):
    rotation, _ = transform
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return roll, pitch, yaw

urdf_path = required(bringup, "urdf")
urdf_file = Path(urdf_path)
if not urdf_file.is_absolute():
    urdf_file = Path(sys.argv[2]) / urdf_file
try:
    urdf_root = element_tree.parse(urdf_file).getroot()
except (OSError, element_tree.ParseError) as error:
    raise SystemExit(f"error: unable to parse URDF '{urdf_file}': {error}") from error

urdf_links = {
    link.attrib["name"] for link in urdf_root.findall("link") if link.attrib.get("name")
}
fixed_adjacency = {link: [] for link in urdf_links}
for joint in urdf_root.findall("joint"):
    if joint.attrib.get("type") != "fixed":
        continue
    parent = joint.find("parent")
    child = joint.find("child")
    if parent is None or child is None:
        raise SystemExit("error: a fixed URDF joint is missing parent or child")
    parent_frame = parent.attrib.get("link", "")
    child_frame = child.attrib.get("link", "")
    if parent_frame not in urdf_links or child_frame not in urdf_links:
        raise SystemExit("error: a fixed URDF joint references an unknown link")
    origin = joint.find("origin")
    xyz = urdf_vector(origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0",
                      f"URDF fixed joint '{joint.attrib.get('name', '')}' xyz")
    rpy = urdf_vector(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0",
                      f"URDF fixed joint '{joint.attrib.get('name', '')}' rpy")
    transform = origin_transform(xyz, rpy)
    fixed_adjacency[parent_frame].append((child_frame, transform))
    fixed_adjacency[child_frame].append((parent_frame, inverse_transform(transform)))

def fixed_transform(source_frame, target_frame):
    if source_frame not in urdf_links or target_frame not in urdf_links:
        raise SystemExit(
            f"error: configured sensor TF '{source_frame}' -> '{target_frame}' is not in the URDF")
    queue = deque([(source_frame, identity_transform())])
    visited = {source_frame}
    while queue:
        current_frame, source_from_current = queue.popleft()
        if current_frame == target_frame:
            return source_from_current
        for next_frame, current_from_next in fixed_adjacency[current_frame]:
            if next_frame not in visited:
                visited.add(next_frame)
                queue.append((next_frame, compose_transform(source_from_current, current_from_next)))
    raise SystemExit(
        f"error: no fixed-joint URDF path from '{source_frame}' to '{target_frame}'")

static_tf_parents = []
static_tf_children = []
static_tf_xyzs = []
static_tf_rpys = []
static_tf_children_set = set()

def append_static_transform(parent_frame, child_frame):
    if child_frame in static_tf_children_set:
        return
    transform = fixed_transform(parent_frame, child_frame)
    rotation, translation = transform
    del rotation
    static_tf_children_set.add(child_frame)
    static_tf_parents.append(parent_frame)
    static_tf_children.append(child_frame)
    static_tf_xyzs.append(" ".join(str(value) for value in translation))
    static_tf_rpys.append(" ".join(str(value) for value in transform_rpy(transform)))

lidar_frame = required(bringup, "lidar_frame").strip()
base_frame = required(bringup, "base_frame").strip()
if not lidar_frame or not base_frame:
    raise SystemExit("error: elevation bringup.lidar_frame and base_frame must be non-empty strings")
append_static_transform(lidar_frame, base_frame)

camera_names = []
camera_namespaces = []
camera_ports = []
camera_driver_frames = []
camera_profiles = []
camera_raw_topics = []
camera_mapped_topics = []
camera_mount_frames = []
camera_source_to_mount_xyzs = []
camera_source_to_mount_rpys = []
camera_noise_stddevs = []
camera_min_ranges = []
camera_max_points = []
seen_sources = set()
for source_name in source_names:
    if not isinstance(source_name, str) or not source_name:
        raise SystemExit("error: observation_merge.inputs contains an invalid source name")
    if source_name in seen_sources:
        raise SystemExit(f"error: observation_merge.inputs contains duplicate '{source_name}'")
    seen_sources.add(source_name)
    if source_parameter(source_name, "type") != "camera":
        raise SystemExit(f"error: selected source '{source_name}' must have type: camera")
    mount_frame = source_parameter(source_name, "mount_frame")
    if not mount_frame or mount_frame not in urdf_links:
        raise SystemExit(
            f"error: selected source '{source_name}' has no URDF mount_frame"
        )
    camera = source_definitions.get(source_name)
    if not isinstance(camera, dict):
        raise SystemExit(
            f"error: selected source '{source_name}' requires a matching "
            "bringup.realsense_cameras entry"
        )
    camera_name = camera_value(source_name, camera, "name")
    camera_namespace = camera_value(source_name, camera, "namespace")
    camera_port = camera_value(source_name, camera, "usb_port_id")
    camera_driver_frame = camera_value(source_name, camera, "driver_base_frame_id")
    camera_profile = camera_value(source_name, camera, "depth_profile")
    if camera_name != source_name:
        raise SystemExit(
            f"error: bringup.realsense_cameras.{source_name}.name must equal '{source_name}'"
        )
    if not isinstance(camera_driver_frame, str) or not camera_driver_frame.strip():
        raise SystemExit(
            f"error: selected camera '{source_name}' driver_base_frame_id must be a non-empty string"
        )
    camera_driver_frame = camera_driver_frame.strip()
    expected_raw_topic = f"/{str(camera_namespace).strip('/')}/{camera_name}/depth/color/points"
    if source_parameter(source_name, "raw_topic") != expected_raw_topic:
        raise SystemExit(
            f"error: source '{source_name}' raw_topic must be '{expected_raw_topic}' to match its driver"
        )
    expected_mapped_topic = f"/autonomy_light/camera_observations/{source_name}_map"
    if source_parameter(source_name, "topic") != expected_mapped_topic:
        raise SystemExit(
            f"error: source '{source_name}' topic must be '{expected_mapped_topic}'"
        )
    if source_parameter(source_name, "frame_override") != "map":
        raise SystemExit(
            f"error: source '{source_name}' must use frame_override: map after camera mapping"
        )
    if source_parameter(source_name, "validate_mount_tf") is not False:
        raise SystemExit(
            f"error: source '{source_name}' must set validate_mount_tf: false after camera mapping"
        )
    for parameter in ("noise_stddev", "min_range", "max_points"):
        if source_parameter(source_name, parameter) in (None, ""):
            raise SystemExit(f"error: source '{source_name}' requires {parameter}")
    source_to_mount_xyz = numeric_vector(
        source_parameter(source_name, "source_to_mount_xyz"),
        f"source '{source_name}' source_to_mount_xyz")
    source_to_mount_rpy = numeric_vector(
        source_parameter(source_name, "source_to_mount_rpy"),
        f"source '{source_name}' source_to_mount_rpy")
    camera_names.append(camera_name)
    camera_namespaces.append(camera_namespace)
    camera_ports.append(camera_port)
    camera_driver_frames.append(camera_driver_frame)
    camera_profiles.append(camera_profile)
    camera_raw_topics.append(expected_raw_topic)
    camera_mapped_topics.append(expected_mapped_topic)
    camera_mount_frames.append(mount_frame)
    camera_source_to_mount_xyzs.append(source_to_mount_xyz)
    camera_source_to_mount_rpys.append(source_to_mount_rpy)
    camera_noise_stddevs.append(source_parameter(source_name, "noise_stddev"))
    camera_min_ranges.append(source_parameter(source_name, "min_range"))
    camera_max_points.append(source_parameter(source_name, "max_points"))
    append_static_transform(base_frame, mount_frame)

values = {
    "INTERNAL_ROS_DOMAIN_ID": bringup.get("internal_ros_domain_id", 10),
    "EXTERNAL_ROS_DOMAIN_ID": bringup.get("external_ros_domain_id", 0),
}
for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
shell_array("CAMERA_SOURCES", source_names)
shell_array("CAMERA_NAMES", camera_names)
shell_array("CAMERA_NAMESPACES", camera_namespaces)
shell_array("CAMERA_USB_PORTS", camera_ports)
shell_array("CAMERA_DRIVER_BASE_FRAMES", camera_driver_frames)
shell_array("CAMERA_DEPTH_PROFILES", camera_profiles)
shell_array("CAMERA_RAW_TOPICS", camera_raw_topics)
shell_array("CAMERA_MAPPED_TOPICS", camera_mapped_topics)
shell_array("CAMERA_MOUNT_FRAMES", camera_mount_frames)
shell_array("CAMERA_SOURCE_TO_MOUNT_XYZS", camera_source_to_mount_xyzs)
shell_array("CAMERA_SOURCE_TO_MOUNT_RPYS", camera_source_to_mount_rpys)
shell_array("CAMERA_NOISE_STDDEVS", camera_noise_stddevs)
shell_array("CAMERA_MIN_RANGES", camera_min_ranges)
shell_array("CAMERA_MAX_POINTS", camera_max_points)
shell_array("STATIC_TF_PARENTS", static_tf_parents)
shell_array("STATIC_TF_CHILDREN", static_tf_children)
shell_array("STATIC_TF_XYZS", static_tf_xyzs)
shell_array("STATIC_TF_RPYS", static_tf_rpys)
PY
)

# The Livox SDK accepts only a JSON device description, while the deployable
# source of truth is this project's Super-LIO YAML.  Read the YAML here and
# produce a small runtime JSON only when the physical driver is started.
source <(/usr/bin/python3 - "${LIO_CONFIG}" "${LIDAR_TYPE_OVERRIDE}" \
    "${LIDAR_IP_OVERRIDE}" "${JETSON_IP_OVERRIDE}" <<'PY'
import ipaddress
import shlex
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}
params = (config.get("bringup") or {}).get("ros__parameters") or {}
livox = params.get("livox") or {}
if not isinstance(livox, dict):
    raise SystemExit("error: bringup.ros__parameters.livox must be a mapping")

model = str(sys.argv[2] or livox.get("model", "")).strip().upper()
lidar_ip = str(sys.argv[3] or livox.get("lidar_ip", "")).strip()
jetson_ip = str(sys.argv[4] or livox.get("jetson_ip", "")).strip()
if model not in {"MID360", "MID360S"}:
    raise SystemExit("error: bringup.livox.model must be MID360 or MID360S")
for label, value in (("lidar_ip", lidar_ip), ("jetson_ip", jetson_ip)):
    try:
        parsed = ipaddress.ip_address(value)
    except ValueError as error:
        raise SystemExit(f"error: bringup.livox.{label} must be an IPv4 address") from error
    if parsed.version != 4 or parsed.is_unspecified or parsed.is_multicast:
        raise SystemExit(f"error: bringup.livox.{label} must be a unicast IPv4 address")
try:
    publish_frequency_hz = float(livox.get("publish_frequency_hz", 60.0))
except (TypeError, ValueError) as error:
    raise SystemExit("error: bringup.livox.publish_frequency_hz must be numeric") from error
if not 0.5 <= publish_frequency_hz <= 100.0:
    raise SystemExit("error: bringup.livox.publish_frequency_hz must be in [0.5, 100.0]")
frame_id = str(livox.get("frame_id", "lidar_link")).strip()
if not frame_id:
    raise SystemExit("error: bringup.livox.frame_id is required")

for key, value in {
    "LIVOX_MODEL": model,
    "LIVOX_LIDAR_IP": lidar_ip,
    "LIVOX_JETSON_IP": jetson_ip,
    "LIVOX_PUBLISH_FREQUENCY_HZ": publish_frequency_hz,
    "LIVOX_FRAME_ID": frame_id,
}.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)
[[ "${INTERNAL_ROS_DOMAIN_ID}" =~ ^[0-9]+$ && "${EXTERNAL_ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] || {
  echo "error: internal_ros_domain_id and external_ros_domain_id must be non-negative integers" >&2
  exit 1
}
(( INTERNAL_ROS_DOMAIN_ID <= 232 && EXTERNAL_ROS_DOMAIN_ID <= 232 )) || {
  echo "error: ROS domain IDs must be in [0, 232]" >&2
  exit 1
}
(( INTERNAL_ROS_DOMAIN_ID != EXTERNAL_ROS_DOMAIN_ID )) || {
  echo "error: internal_ros_domain_id and external_ros_domain_id must differ" >&2
  exit 1
}
ROS_DOMAIN_ID="${INTERNAL_ROS_DOMAIN_ID}"
export ROS_DOMAIN_ID

if [[ "${CHECK_ONLY}" == "true" ]]; then
  /usr/bin/python3 - "${ELEVATION_CONFIG}" "${LIO_CONFIG}" "${NAV2_CONFIG}" <<'PY'
import sys
import yaml
for path in sys.argv[1:]:
    with open(path, encoding="utf-8") as stream:
        yaml.safe_load(stream)
print("autonomy-light: elevation, Super-LIO, and Nav2 configs are valid")
PY
  echo "autonomy-light: Livox ${LIVOX_MODEL}, lidar ${LIVOX_LIDAR_IP}, Jetson ${LIVOX_JETSON_IP}"
  exit 0
fi

if [[ "${MODE}" == "real" && "${NO_DRIVERS}" == "false" ]]; then
  if ! ip -o -4 addr show | awk '{split($4, address, "/"); print address[1]}' | \
      grep -Fqx "${LIVOX_JETSON_IP}"; then
    echo "error: configured bringup.livox.jetson_ip ${LIVOX_JETSON_IP} is not assigned locally" >&2
    echo "       update config/super_lio_mid360.yaml or use --jetson-ip IPV4" >&2
    exit 1
  fi
fi

declare -a PIDS=()
declare -A LABELS=()
LAST_PID=""
LOG_SEQUENCE=0
RUNTIME_LOG_DIR="${ROOT}/log/runtime_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p -- "${RUNTIME_LOG_DIR}"

start() {
  local label="$1"
  shift
  local safe_label log_file
  LOG_SEQUENCE=$((LOG_SEQUENCE + 1))
  safe_label="${label//[^a-zA-Z0-9._-]/_}"
  printf -v log_file "%s/%02d_%s.log" "${RUNTIME_LOG_DIR}" "${LOG_SEQUENCE}" "${safe_label}"
  echo "autonomy-light: starting ${label} (log: $(basename -- "${log_file}"))"
  setsid "$@" >"${log_file}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  LABELS["${pid}"]="${label}"
  LAST_PID="${pid}"
}

start_terminal() {
  local label="$1"
  shift
  echo "autonomy-light: starting ${label}"
  setsid "$@" &
  local pid=$!
  PIDS+=("${pid}")
  LABELS["${pid}"]="${label}"
  LAST_PID="${pid}"
}

cleanup() {
  trap - EXIT INT TERM
  local index pid
  for ((index = ${#PIDS[@]} - 1; index >= 0; --index)); do
    pid="${PIDS[${index}]}"
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  sleep 0.5
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
  if [[ -n "${LIVOX_RUNTIME_CONFIG}" ]]; then
    rm -f -- "${LIVOX_RUNTIME_CONFIG}"
  fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "autonomy-light: SLAM + camera elevation + Nav2"
echo "  ROS domains: internal ${INTERNAL_ROS_DOMAIN_ID}; external ${EXTERNAL_ROS_DOMAIN_ID}"
echo "  LiDAR: Super-LIO -> Nav2 occupancy"
echo "  camera: observation_merge -> elevation map"
echo "  command: /nav2/cmd_vel -> internal CommandCore -> external /control_command/autopilot"
echo "  TF: static sensor extrinsics only"
if [[ "${MODE}" == "real" && "${NO_DRIVERS}" == "false" ]]; then
  echo "  Livox: ${LIVOX_MODEL} ${LIVOX_LIDAR_IP} -> ${LIVOX_JETSON_IP}"
fi
echo "  raw node logs: ${RUNTIME_LOG_DIR}"

declare -a LIO_ARGS=(--ros-args --params-file "${LIO_CONFIG}")
if [[ "${MODE}" == "sim" ]]; then
  LIO_ARGS+=(
    -p use_sim_time:=true
    -p lio.ros.lidar_topic:=/f4/lidar/points
    -p lio.ros.imu_topic:=/f4/lidar/imu
    -p lio.sensor.lidar_type:=3
  )
fi
if [[ -n "${MAPPING_OUTPUT}" ]]; then
  MAPPING_OUTPUT="$(realpath -m -- "${MAPPING_OUTPUT}")"
  mkdir -p -- "$(dirname -- "${MAPPING_OUTPUT}")"
  if [[ -e "${MAPPING_OUTPUT}" ]]; then
    MAPPING_TIMESTAMP="${MAPPING_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
    MAPPING_BACKUP="${MAPPING_OUTPUT}.bak.${MAPPING_TIMESTAMP}"
    mv -- "${MAPPING_OUTPUT}" "${MAPPING_BACKUP}"
    echo "autonomy-light: previous map moved to ${MAPPING_BACKUP}"
  fi
  echo "autonomy-light: full-SLAM map will be saved to ${MAPPING_OUTPUT} on shutdown"
  LIO_ARGS+=(
    -p lio.map.save_map:=true
    -p "lio.map.save_map_dir:=$(dirname -- "${MAPPING_OUTPUT}")"
    -p "lio.map.map_name:=$(basename -- "${MAPPING_OUTPUT}")"
  )
fi
if [[ -n "${MAP_FILE}" ]]; then
  MAP_FILE="$(realpath -- "${MAP_FILE}")"
  LIO_ARGS+=(
    -p lio.slam.enable:=false
    -p "lio.map.save_map_dir:=$(dirname -- "${MAP_FILE}")"
    -p "lio.map.map_name:=$(basename -- "${MAP_FILE}")"
  )
fi

if [[ "${MODE}" == "real" && "${NO_DRIVERS}" == "false" ]]; then
  LIVOX_RUNTIME_CONFIG="$(mktemp "${TMPDIR:-/tmp}/autonomy_light_livox.XXXXXX.json")"
  /usr/bin/python3 - "${LIVOX_RUNTIME_CONFIG}" "${LIVOX_MODEL}" "${LIVOX_LIDAR_IP}" \
      "${LIVOX_JETSON_IP}" <<'PY'
import json
import sys

path, model, lidar_ip, jetson_ip = sys.argv[1:]
lidar_ports = {
    "cmd_data_port": 56100,
    "push_msg_port": 56200,
    "point_data_port": 56300,
    "imu_data_port": 56400,
    "log_data_port": 56500,
}
host_ports = {
    "cmd_data_port": 56101,
    "push_msg_port": 56201,
    "point_data_port": 56301,
    "imu_data_port": 56401,
    "log_data_port": 56501,
}
if model == "MID360":
    host = {
        "cmd_data_ip": jetson_ip,
        "push_msg_ip": jetson_ip,
        "point_data_ip": jetson_ip,
        "imu_data_ip": jetson_ip,
        "log_data_ip": "",
        **host_ports,
    }
    device = {"MID360": {"lidar_net_info": lidar_ports, "host_net_info": host}}
else:
    host = {"host_ip": jetson_ip, **host_ports}
    device = {"Mid360s": {"lidar_net_info": lidar_ports, "host_net_info": [host]}}
payload = {
    "lidar_summary_info": {"lidar_type": 8},
    **device,
    "lidar_configs": [{
        "ip": lidar_ip,
        "pcl_data_type": 1,
        "pattern_mode": 0,
        "extrinsic_parameter": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                                "x": 0, "y": 0, "z": 0},
    }],
}
with open(path, "w", encoding="utf-8") as stream:
    json.dump(payload, stream, indent=2)
    stream.write("\n")
PY
  start "Livox driver (${LIVOX_MODEL})" ros2 run livox_ros_driver2 livox_ros_driver2_node \
    --ros-args \
    -p xfer_format:=1 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p "publish_freq:=${LIVOX_PUBLISH_FREQUENCY_HZ}" \
    -p output_data_type:=0 \
    -p "frame_id:=${LIVOX_FRAME_ID}" \
    -p "user_config_path:=${LIVOX_RUNTIME_CONFIG}" \
    -p cmdline_input_bd_code:=livox0000000001
fi
if [[ "${NO_DRIVERS}" == "false" ]]; then
  LIO_EXECUTABLE="super_lio_node"
  [[ -n "${MAP_FILE}" ]] && LIO_EXECUTABLE="relocation_node"
  start "Super-LIO" ros2 run super_lio "${LIO_EXECUTABLE}" "${LIO_ARGS[@]}"
  LIO_PID="${LAST_PID}"
else
  LIO_PID=""
fi

declare -a STATIC_TF_PIDS=()
for transform_index in "${!STATIC_TF_PARENTS[@]}"; do
  read -r static_x static_y static_z <<<"${STATIC_TF_XYZS[transform_index]}"
  read -r static_roll static_pitch static_yaw <<<"${STATIC_TF_RPYS[transform_index]}"
  start "static TF ${STATIC_TF_PARENTS[transform_index]} -> ${STATIC_TF_CHILDREN[transform_index]}" \
    ros2 run tf2_ros static_transform_publisher \
    --x "${static_x}" --y "${static_y}" --z "${static_z}" \
    --roll "${static_roll}" --pitch "${static_pitch}" --yaw "${static_yaw}" \
    --frame-id "${STATIC_TF_PARENTS[transform_index]}" \
    --child-frame-id "${STATIC_TF_CHILDREN[transform_index]}"
  STATIC_TF_PIDS+=("${LAST_PID}")
done

if [[ "${ENABLE_RVIZ}" == "true" ]]; then
  echo "  RViz2: internal ROS domain ${INTERNAL_ROS_DOMAIN_ID} (Nav2 costmaps stay private)"
  start "RViz2" rviz2 -d "${ROOT}/config/elevation_mapping.rviz"
fi

declare -a CAMERA_PIDS=()
if [[ "${MODE}" == "real" && "${NO_DRIVERS}" == "false" ]]; then
  for camera_index in "${!CAMERA_SOURCES[@]}"; do
    start "RealSense ${CAMERA_SOURCES[camera_index]}" ros2 launch realsense2_camera rs_launch.py \
      "camera_name:=${CAMERA_NAMES[camera_index]}" \
      "camera_namespace:=${CAMERA_NAMESPACES[camera_index]}" \
      "usb_port_id:=${CAMERA_USB_PORTS[camera_index]}" \
      "base_frame_id:=${CAMERA_DRIVER_BASE_FRAMES[camera_index]}" \
      "publish_tf:=false" \
      "enable_depth:=true" "enable_color:=false" \
      "depth_module.depth_profile:=${CAMERA_DEPTH_PROFILES[camera_index]}" \
      "pointcloud.enable:=true" "pointcloud.allow_no_texture_points:=true" \
      "pointcloud.stream_filter:=1" "decimation_filter.enable:=true" \
      "decimation_filter.filter_magnitude:=4"
    CAMERA_PIDS+=("${LAST_PID}")
  done
fi

ELEVATION_ARGS=(--ros-args --params-file "${ELEVATION_CONFIG}")
[[ "${MODE}" == "sim" ]] && ELEVATION_ARGS+=(-p use_sim_time:=true)
start "gravity-frame broadcaster" ros2 run autonomy_light gravity_frame_broadcaster "${ELEVATION_ARGS[@]}"
GRAVITY_PID="${LAST_PID}"
declare -a CAMERA_MAPPER_PIDS=()
for camera_index in "${!CAMERA_SOURCES[@]}"; do
  start "camera mapper ${CAMERA_SOURCES[camera_index]}" \
    ros2 run autonomy_light camera_frame_mapper "${ELEVATION_ARGS[@]}" \
    -r "__node:=camera_frame_mapper_${CAMERA_SOURCES[camera_index]}" \
    -p "input_topic:=${CAMERA_RAW_TOPICS[camera_index]}" \
    -p "output_topic:=${CAMERA_MAPPED_TOPICS[camera_index]}" \
    -p "mount_frame:=${CAMERA_MOUNT_FRAMES[camera_index]}" \
    -p "source_to_mount_xyz:=${CAMERA_SOURCE_TO_MOUNT_XYZS[camera_index]}" \
    -p "source_to_mount_rpy:=${CAMERA_SOURCE_TO_MOUNT_RPYS[camera_index]}" \
    -p "noise_stddev:=${CAMERA_NOISE_STDDEVS[camera_index]}" \
    -p "min_range:=${CAMERA_MIN_RANGES[camera_index]}" \
    -p "max_points:=${CAMERA_MAX_POINTS[camera_index]}" \
    -p tf_timeout_sec:=0.020
  CAMERA_MAPPER_PIDS+=("${LAST_PID}")
done
start "camera observation merge" ros2 run autonomy_light observation_merge "${ELEVATION_ARGS[@]}"
MERGE_PID="${LAST_PID}"
start "elevation mapper" ros2 run autonomy_light precise_elevation_mapping "${ELEVATION_ARGS[@]}"
MAPPER_PID="${LAST_PID}"
start "height-map bridge" ros2 run autonomy_light height_map_bridge "${ELEVATION_ARGS[@]}"
BRIDGE_PID="${LAST_PID}"

NAV2_PID=""
OUTBOUND_COMMAND_BRIDGE_PID=""
start "network boundary bridge" ros2 run autonomy_light outbound_command_bridge \
  --internal-domain "${INTERNAL_ROS_DOMAIN_ID}" \
  --external-domain "${EXTERNAL_ROS_DOMAIN_ID}" \
  --input-topic /autonomy_light/command_out \
  --output-topic /control_command/autopilot
OUTBOUND_COMMAND_BRIDGE_PID="${LAST_PID}"
if [[ "${ENABLE_NAV2}" == "true" ]]; then
  ros2 pkg prefix nav2_controller >/dev/null 2>&1 || {
    echo "error: Nav2 is unavailable; run ./build.sh" >&2
    exit 1
  }
  USE_SIM_TIME="false"
  [[ "${MODE}" == "sim" ]] && USE_SIM_TIME="true"
  start "Nav2 and autopilot bridge" bash "${ROOT}/scripts/nav2_wait_and_launch.sh" \
    "${NAV2_CONFIG}" "${USE_SIM_TIME}" /lio/odom 120
  NAV2_PID="${LAST_PID}"
fi

STATUS_PID=""
if [[ "${ENABLE_STATUS}" == "true" ]]; then
  start_terminal "terminal status monitor (${VIS_RATE} Hz)" \
    /usr/bin/python3 "${STATUS_SCRIPT}" --rate "${VIS_RATE}" --log-dir "${RUNTIME_LOG_DIR}"
  STATUS_PID="${LAST_PID}"
fi

declare -a CRITICAL_PIDS=("${STATIC_TF_PIDS[@]}" "${GRAVITY_PID}" "${CAMERA_MAPPER_PIDS[@]}" "${MERGE_PID}" "${MAPPER_PID}" "${BRIDGE_PID}")
[[ -n "${LIO_PID}" ]] && CRITICAL_PIDS+=("${LIO_PID}")
CRITICAL_PIDS+=("${CAMERA_PIDS[@]}")
[[ -n "${OUTBOUND_COMMAND_BRIDGE_PID}" ]] && CRITICAL_PIDS+=("${OUTBOUND_COMMAND_BRIDGE_PID}")
# Nav2 waits for the first /lio/odom before lifecycle activation.  A missing
# LiDAR packet must not take down the independent SLAM/elevation diagnostics;
# its timeout is reported by nav2_wait_and_launch.sh while the core pipeline
# remains alive for inspection.
wait -n "${CRITICAL_PIDS[@]}"
