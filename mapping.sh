#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./mapping.sh [launch options] [mapping options]

  --output FILE      Saved loop-closed Super-LIO PCD.
  --output-dir DIR   Default output directory (./maps).
  --dry-run          Print the launch command.

Super-LIO runs full SLAM (keyframes, loop closure, pose graph) and writes the
map after its normal Ctrl+C shutdown.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${AUTONOMY_LIGHT_MAPPING_OUTPUT_DIR:-${PWD}/maps}"
OUTPUT_FILE="${AUTONOMY_LIGHT_MAPPING_OUTPUT_FILE:-}"
DRY_RUN="false"
declare -a LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --output) OUTPUT_FILE="${2:?--output requires a file}"; shift 2 ;;
    --output=*) OUTPUT_FILE="${1#*=}"; shift ;;
    --output-dir) OUTPUT_DIR="${2:?--output-dir requires a directory}"; shift 2 ;;
    --output-dir=*) OUTPUT_DIR="${1#*=}"; shift ;;
    --dry-run) DRY_RUN="true"; shift ;;
    --) echo "error: mapping options must precede --" >&2; exit 2 ;;
    *) LAUNCH_ARGS+=("$1"); shift ;;
  esac
done

OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
[[ -n "${OUTPUT_FILE}" ]] || OUTPUT_FILE="${OUTPUT_DIR}/super_lio_map_${TIMESTAMP}.pcd"
OUTPUT_FILE="$(realpath -m -- "${OUTPUT_FILE}")"
COMMAND=("${SCRIPT_DIR}/launch.sh" "${LAUNCH_ARGS[@]}" --mapping-output "${OUTPUT_FILE}")

if [[ "${DRY_RUN}" == "true" ]]; then
  printf 'dry-run command:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

mkdir -p -- "$(dirname -- "${OUTPUT_FILE}")"
if [[ -e "${OUTPUT_FILE}" ]]; then
  BACKUP="${OUTPUT_FILE}.bak.${TIMESTAMP}"
  mv -- "${OUTPUT_FILE}" "${BACKUP}"
  echo "previous map moved to: ${BACKUP}"
fi

echo "mapping: Super-LIO full SLAM -> ${OUTPUT_FILE}"
echo "stop with Ctrl+C; Super-LIO saves the optimized PCD during shutdown."
set +e
"${COMMAND[@]}"
STATUS="$?"
set -e

[[ -s "${OUTPUT_FILE}" ]] || { echo "error: map was not saved: ${OUTPUT_FILE}" >&2; exit 1; }
printf 'created_at: "%s"\nalgorithm: "Super-LIO pose graph"\npcd: "%s"\n' \
  "${TIMESTAMP}" "${OUTPUT_FILE}" > "${OUTPUT_FILE}.yaml"
echo "saved Super-LIO map: ${OUTPUT_FILE}"
[[ "${STATUS}" == "0" || "${STATUS}" == "130" ]]
