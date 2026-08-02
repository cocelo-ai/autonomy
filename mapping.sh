#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./mapping.sh [launch options] [mapping options] [-- <extra autonomy_light ROS args>]

Mapping options:
  --output FILE          Refined saved PCD path. Default: maps/super_lio_map_<timestamp>.pcd
  --output-dir DIR       Directory for the default output file. Default: ./maps
  --dry-run              Print the command without starting mapping.

The map is built from Super-LIO's /lio/cloud_world stream, voxelized by the
single autonomy_light elevation node, and saved on the first Ctrl+C.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${AUTONOMY_LIGHT_MAPPING_OUTPUT_DIR:-${PWD}/maps}"
OUTPUT_FILE="${AUTONOMY_LIGHT_MAPPING_OUTPUT_FILE:-}"
DRY_RUN="false"

declare -a LAUNCH_ARGS=()
declare -a EXTRA_ROS_ARGS=()

expand_user_path() {
  case "$1" in
    '~') printf '%s\n' "${HOME:?HOME is required to expand ~}" ;;
    '~/'*) printf '%s/%s\n' "${HOME:?HOME is required to expand ~/ paths}" "${1:2}" ;;
    '~'*) echo "error: only ~ and ~/ paths are supported: $1" >&2; return 2 ;;
    *) printf '%s\n' "$1" ;;
  esac
}

monitor_save() {
  local pid="$1" output="$2" started
  started="$(date +%s)"
  while kill -0 "${pid}" >/dev/null 2>&1; do
    local size="waiting"
    [[ -e "${output}" ]] && size="$(stat -c '%s bytes' -- "${output}" 2>/dev/null || echo unknown)"
    printf '\rsaving refined map: elapsed=%ss output=%s' "$(( $(date +%s) - started ))" "${size}"
    sleep 1
  done
  printf '\n'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --output) OUTPUT_FILE="${2:?--output requires a file path}"; shift 2 ;;
    --output=*) OUTPUT_FILE="${1#--output=}"; shift ;;
    --output-dir) OUTPUT_DIR="${2:?--output-dir requires a directory}"; shift 2 ;;
    --output-dir=*) OUTPUT_DIR="${1#--output-dir=}"; shift ;;
    --optimize-on-shutdown)
      echo "error: local pose-graph optimization was removed; use Super-LIO's map and --output." >&2
      exit 2
      ;;
    --dry-run) DRY_RUN="true"; shift ;;
    --save-raw-pcd|--no-save-raw-pcd|--raw-output|--pcd-interval|--pcd-source|--save-grace-sec)
      echo "error: $1 was removed with the Point-LIO migration; use --output instead." >&2
      exit 2
      ;;
    --) shift; EXTRA_ROS_ARGS+=("$@"); break ;;
    *) LAUNCH_ARGS+=("$1"); shift ;;
  esac
done

OUTPUT_DIR="$(realpath -m -- "$(expand_user_path "${OUTPUT_DIR}")")"
if [[ -z "${OUTPUT_FILE}" ]]; then
  OUTPUT_FILE="${OUTPUT_DIR}/super_lio_map_${TIMESTAMP}.pcd"
fi
OUTPUT_FILE="$(realpath -m -- "$(expand_user_path "${OUTPUT_FILE}")")"

COMMAND=(
  "${SCRIPT_DIR}/launch.sh"
  "${LAUNCH_ARGS[@]}"
  --
  -p "mapping_only:=true"
  -p "mapping_pcd_file:=${OUTPUT_FILE}"
  "${EXTRA_ROS_ARGS[@]}"
)

if [[ "${DRY_RUN}" == "true" ]]; then
  printf 'dry-run command:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

mkdir -p -- "$(dirname -- "${OUTPUT_FILE}")"
if [[ -e "${OUTPUT_FILE}" ]]; then
  backup="${OUTPUT_FILE}.bak.${TIMESTAMP}"
  mv -- "${OUTPUT_FILE}" "${backup}"
  echo "previous refined map moved to: ${backup}"
fi

echo "mapping mode: Super-LIO global-frame scans -> voxel map=${OUTPUT_FILE}"
echo "stop with Ctrl+C once data collection is complete."

"${COMMAND[@]}" &
MAPPING_PID="$!"
MONITOR_PID=""

cleanup() {
  if ! kill -0 "${MAPPING_PID}" >/dev/null 2>&1; then
    return
  fi
  if [[ -n "${MONITOR_PID}" ]]; then
    echo "second interrupt received; forcing mapping process to stop..."
    kill -TERM "${MAPPING_PID}" >/dev/null 2>&1 || true
    return 130
  fi
  echo "stopping mapping process; saving refined map..."
  monitor_save "${MAPPING_PID}" "${OUTPUT_FILE}" &
  MONITOR_PID="$!"
  kill -INT "${MAPPING_PID}" >/dev/null 2>&1 || true
}
trap cleanup INT TERM

set +e
wait "${MAPPING_PID}"
STATUS="$?"
set -e
trap - INT TERM

if [[ -n "${MONITOR_PID}" ]]; then
  wait "${MONITOR_PID}" >/dev/null 2>&1 || true
fi
if [[ ! -s "${OUTPUT_FILE}" ]]; then
  echo "error: refined map was not saved: ${OUTPUT_FILE}" >&2
  exit 1
fi

META_FILE="${OUTPUT_FILE}.yaml"
{
  echo "created_at: \"${TIMESTAMP}\""
  echo "algorithm: \"Super-LIO\""
  echo "pcd: \"${OUTPUT_FILE}\""
  echo "voxelized_by: \"autonomy_light\""
} > "${META_FILE}"
echo "saved Super-LIO map: ${OUTPUT_FILE}"
echo "saved metadata: ${META_FILE}"
[[ "${STATUS}" == "0" || "${STATUS}" == "130" || "${STATUS}" == "143" ]]
