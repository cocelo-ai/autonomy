#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  autonomy-light-mapping [launch options] [mapping options] [-- <extra autonomy_light ros args>]
  ./mapping.sh [launch options] [mapping options] [-- <extra autonomy_light ros args>]

Mapping options:
  --output FILE          Refined saved PCD path. Default: maps/point_lio_map_<timestamp>.pcd
  --raw-output FILE      Raw Point-LIO PCD path. Default: <output>.raw.pcd
  --output-dir DIR       Directory for the default output file. Default: ./maps
  --pcd-interval N       Point-LIO pcd_save.interval. Default: -1, save one scans.pcd on shutdown.
  --save-grace-sec SEC   Seconds to wait for Point-LIO to flush PCD after Ctrl+C. Default: 120.
  --pcd-source FILE      Internal Point-LIO output path. Default: <raw-output>
  --dry-run              Print the command without starting mapping.

Most launch.sh options are passed through, for example:
  --real, --sim, --config, --livox-interface, --livox-lidar-ip,
  --livox2-interface, --livox-lidar2-ip, --raw-lidar-topic, --raw-lidar2-topic.

Examples:
  autonomy-light-mapping --real
  autonomy-light-mapping --real --output "$PWD/maps/lab_mapping.pcd"
  ./mapping.sh --real
  ./mapping.sh --real --output maps/lab_mapping.pcd
  ./mapping.sh --real --livox-lidar2-ip 192.168.2.166 --livox2-interface enx123

Stop mapping with Ctrl+C. The script then waits for Point-LIO to save the raw
PCD and for the refined-map loop-closure pose graph to finish.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${AUTONOMY_LIGHT_MAPPING_OUTPUT_DIR:-${PWD}/maps}"
OUTPUT_FILE="${AUTONOMY_LIGHT_MAPPING_OUTPUT_FILE:-}"
RAW_OUTPUT_FILE="${AUTONOMY_LIGHT_MAPPING_RAW_OUTPUT_FILE:-}"
PCD_INTERVAL="${AUTONOMY_LIGHT_MAPPING_PCD_INTERVAL:--1}"
SAVE_GRACE_SEC="${AUTONOMY_LIGHT_MAPPING_SAVE_GRACE_SEC:-120}"
PCD_SOURCE="${AUTONOMY_LIGHT_POINT_LIO_PCD_SOURCE:-}"
DRY_RUN="false"

declare -a LAUNCH_ARGS=()
declare -a EXTRA_ROS_ARGS=()

expand_user_path() {
  local path="$1"
  case "${path}" in
    '~')
      printf '%s\n' "${HOME:?HOME is required to expand ~}"
      ;;
    '~/'*)
      printf '%s/%s\n' "${HOME:?HOME is required to expand ~/ paths}" "${path:2}"
      ;;
    '~'*)
      echo "error: only ~ and ~/ paths for the current user are supported: ${path}" >&2
      return 2
      ;;
    *)
      printf '%s\n' "${path}"
      ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --output)
      OUTPUT_FILE="${2:?--output requires a file path}"
      shift 2
      ;;
    --output=*)
      OUTPUT_FILE="${1#--output=}"
      shift
      ;;
    --raw-output)
      RAW_OUTPUT_FILE="${2:?--raw-output requires a file path}"
      shift 2
      ;;
    --raw-output=*)
      RAW_OUTPUT_FILE="${1#--raw-output=}"
      shift
      ;;
    --output-dir)
      OUTPUT_DIR="${2:?--output-dir requires a directory}"
      shift 2
      ;;
    --output-dir=*)
      OUTPUT_DIR="${1#--output-dir=}"
      shift
      ;;
    --pcd-interval)
      PCD_INTERVAL="${2:?--pcd-interval requires an integer}"
      shift 2
      ;;
    --pcd-interval=*)
      PCD_INTERVAL="${1#--pcd-interval=}"
      shift
      ;;
    --save-grace-sec)
      SAVE_GRACE_SEC="${2:?--save-grace-sec requires seconds}"
      shift 2
      ;;
    --save-grace-sec=*)
      SAVE_GRACE_SEC="${1#--save-grace-sec=}"
      shift
      ;;
    --pcd-source)
      PCD_SOURCE="${2:?--pcd-source requires a file path}"
      shift 2
      ;;
    --pcd-source=*)
      PCD_SOURCE="${1#--pcd-source=}"
      shift
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    --)
      shift
      EXTRA_ROS_ARGS+=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

OUTPUT_DIR="$(realpath -m -- "$(expand_user_path "${OUTPUT_DIR}")")"
if [[ -z "${OUTPUT_FILE}" ]]; then
  OUTPUT_FILE="${OUTPUT_DIR}/point_lio_map_${TIMESTAMP}.pcd"
fi
OUTPUT_FILE="$(realpath -m -- "$(expand_user_path "${OUTPUT_FILE}")")"
if [[ -z "${RAW_OUTPUT_FILE}" ]]; then
  if [[ "${OUTPUT_FILE}" == *.pcd ]]; then
    RAW_OUTPUT_FILE="${OUTPUT_FILE%.pcd}.raw.pcd"
  else
    RAW_OUTPUT_FILE="${OUTPUT_FILE}.raw.pcd"
  fi
fi
RAW_OUTPUT_FILE="$(realpath -m -- "$(expand_user_path "${RAW_OUTPUT_FILE}")")"
if [[ -z "${PCD_SOURCE}" ]]; then
  PCD_SOURCE="${RAW_OUTPUT_FILE}"
fi
PCD_SOURCE="$(realpath -m -- "$(expand_user_path "${PCD_SOURCE}")")"

REFINED_OUTPUT_PATH="${OUTPUT_FILE}"
RAW_OUTPUT_PATH="${RAW_OUTPUT_FILE}"
PCD_SOURCE_PATH="${PCD_SOURCE}"
if [[ "${REFINED_OUTPUT_PATH}" == "${RAW_OUTPUT_PATH}" ||
      "${REFINED_OUTPUT_PATH}" == "${PCD_SOURCE_PATH}" ]]; then
  echo "error: --output must differ from --raw-output and --pcd-source" >&2
  exit 2
fi

# ROS 2 infers a bare value such as "120" as an integer.  This parameter is
# declared as double by autonomy_light, so retain or add a decimal point.
if [[ "${SAVE_GRACE_SEC}" =~ ^[+-]?[0-9]+$ ]]; then
  SAVE_GRACE_SEC="${SAVE_GRACE_SEC}.0"
fi

COMMAND=(
  "${SCRIPT_DIR}/launch.sh"
  "${LAUNCH_ARGS[@]}"
  --
  -p
  mapping_only:=true
  -p
  "mapping_refined_pcd_file:=${OUTPUT_FILE}"
  -p
  point_lio_pcd_save_en:=true
  -p
  "point_lio_pcd_save_interval:=${PCD_INTERVAL}"
  -p
  "point_lio_pcd_save_file:=${PCD_SOURCE_PATH}"
  -p
  "child_shutdown_grace_sec:=${SAVE_GRACE_SEC}"
  "${EXTRA_ROS_ARGS[@]}"
)

echo "mapping mode: refined output=${OUTPUT_FILE}"
echo "mapping mode: raw output=${RAW_OUTPUT_FILE}"
echo "mapping mode: Point-LIO internal raw PCD=${PCD_SOURCE}"
echo "mapping mode: stop with Ctrl+C, then wait for raw save and pose-graph optimization"

if [[ "${DRY_RUN}" == "true" ]]; then
  printf 'dry-run command:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

mkdir -p -- "$(dirname -- "${OUTPUT_FILE}")"
mkdir -p -- "$(dirname -- "${RAW_OUTPUT_FILE}")"
mkdir -p -- "$(dirname -- "${PCD_SOURCE}")"

if [[ -e "${PCD_SOURCE}" ]]; then
  BACKUP_SOURCE="${PCD_SOURCE}.bak.${TIMESTAMP}"
  mv -- "${PCD_SOURCE}" "${BACKUP_SOURCE}"
  echo "previous Point-LIO PCD moved to: ${BACKUP_SOURCE}"
fi
if [[ -e "${OUTPUT_FILE}" ]]; then
  BACKUP_REFINED_OUTPUT="${OUTPUT_FILE}.bak.${TIMESTAMP}"
  mv -- "${OUTPUT_FILE}" "${BACKUP_REFINED_OUTPUT}"
  echo "previous refined output moved to: ${BACKUP_REFINED_OUTPUT}"
fi
if [[ "${PCD_SOURCE_PATH}" != "${RAW_OUTPUT_PATH}" && -e "${RAW_OUTPUT_FILE}" ]]; then
  BACKUP_RAW_OUTPUT="${RAW_OUTPUT_FILE}.bak.${TIMESTAMP}"
  mv -- "${RAW_OUTPUT_FILE}" "${BACKUP_RAW_OUTPUT}"
  echo "previous raw output moved to: ${BACKUP_RAW_OUTPUT}"
fi

set +e
"${COMMAND[@]}" &
MAPPING_PID="$!"

cleanup() {
  if [[ -n "${MAPPING_PID:-}" ]] && kill -0 "${MAPPING_PID}" >/dev/null 2>&1; then
    echo "stopping mapping process; waiting for Point-LIO PCD flush..."
    kill -INT "${MAPPING_PID}" >/dev/null 2>&1 || true
    wait "${MAPPING_PID}"
  fi
}
trap cleanup INT TERM

wait "${MAPPING_PID}"
STATUS="$?"
trap - INT TERM
set -e

RAW_SAVED="false"
REFINED_SAVED="false"
if [[ -s "${PCD_SOURCE}" ]]; then
  if [[ "${PCD_SOURCE_PATH}" != "${RAW_OUTPUT_PATH}" ]]; then
    cp -- "${PCD_SOURCE}" "${RAW_OUTPUT_FILE}"
  fi
  RAW_SAVED="true"
else
  echo "warning: Point-LIO did not produce a non-empty raw PCD at ${PCD_SOURCE}" >&2
  echo "hint: collect data, then stop with Ctrl+C so Point-LIO can flush scans.pcd." >&2
fi

if [[ -s "${OUTPUT_FILE}" ]]; then
  REFINED_SAVED="true"
else
  echo "warning: refined global map did not produce a non-empty PCD at ${OUTPUT_FILE}" >&2
  echo "hint: verify /cloud_registered is flowing while mapping." >&2
fi

if [[ "${RAW_SAVED}" == "true" && "${REFINED_SAVED}" == "true" ]]; then
  META_FILE="${OUTPUT_FILE}.yaml"
  {
    echo "created_at: \"${TIMESTAMP}\""
    echo "refined_pcd: \"${OUTPUT_FILE}\""
    echo "raw_source_pcd: \"${PCD_SOURCE}\""
    echo "raw_pcd: \"${RAW_OUTPUT_FILE}\""
    echo "refined_voxel_leaf_size: 0.01"
    echo "pcd_interval: ${PCD_INTERVAL}"
    echo "save_grace_sec: ${SAVE_GRACE_SEC}"
    if [[ "${#LAUNCH_ARGS[@]}" -eq 0 ]]; then
      echo "launch_args: []"
    else
      echo "launch_args:"
      printf '  - "%s"\n' "${LAUNCH_ARGS[@]}"
    fi
  } > "${META_FILE}"
  echo "saved refined mapping PCD: ${OUTPUT_FILE}"
  echo "saved raw Point-LIO PCD: ${RAW_OUTPUT_FILE}"
  echo "saved metadata: ${META_FILE}"
  if [[ "${STATUS}" == "130" || "${STATUS}" == "143" ]]; then
    exit 0
  fi
else
  exit 1
fi

exit "${STATUS}"
