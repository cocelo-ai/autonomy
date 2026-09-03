#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"; shift 2 ;;
    --ros-distro=*) ROS_DISTRO_NAME="${1#*=}"; shift ;;
    -h|--help)
      echo "Usage: setup_nav2.sh [--ros-distro NAME]"
      exit 0
      ;;
    *) echo "error: unsupported option: $1" >&2; exit 2 ;;
  esac
done

LOCAL_ROOT="${PROJECT_DIR}/.deps"
LOCAL_ROS_PREFIX="${LOCAL_ROOT}/opt/ros/${ROS_DISTRO_NAME}"
CACHE_DIR="${LOCAL_ROOT}/cache/nav2-${ROS_DISTRO_NAME}"
READY_MARKER="${LOCAL_ROS_PREFIX}/.autonomy_light_nav2_ready"

required_files=(
  "lib/libdiagnostic_updater.so"
  "lib/nav2_bt_navigator/bt_navigator"
  "lib/nav2_controller/controller_server"
  "lib/libmppi_controller.so"
  "lib/libnav2_smac_planner.so"
  "lib/nav2_map_server/map_server"
  "lib/nav2_planner/planner_server"
  "lib/nav2_velocity_smoother/velocity_smoother"
  "lib/libnav2_regulated_pure_pursuit_controller.so"
  "lib/libnav2_rviz_plugins.so"
  "share/nav2_msgs/action/NavigateToPose.action"
)

ready=true
for relative_path in "${required_files[@]}"; do
  if [[ ! -e "/opt/ros/${ROS_DISTRO_NAME}/${relative_path}" && ! -e "${LOCAL_ROS_PREFIX}/${relative_path}" ]]; then
    ready=false
    break
  fi
done
if ! ldconfig -p 2>/dev/null | grep -F 'libGraphicsMagick++-Q16.so.12' >/dev/null &&
   [[ ! -e "${LOCAL_ROOT}/usr/lib/libGraphicsMagick++-Q16.so.12" ]]; then
  ready=false
fi
if ! ldconfig -p 2>/dev/null | grep -F 'libompl.so.18' >/dev/null &&
   ! compgen -G "${LOCAL_ROS_PREFIX}/lib/*-linux-gnu/libompl.so.18" >/dev/null; then
  ready=false
fi
if [[ "${ready}" == "true" ]]; then
  mkdir -p "${LOCAL_ROS_PREFIX}"
  touch "${READY_MARKER}"
  echo "Nav2 runtime is ready (${ROS_DISTRO_NAME})."
  exit 0
fi

command -v apt-cache >/dev/null || { echo "error: apt-cache is required to resolve Nav2 packages" >&2; exit 1; }
command -v apt-get >/dev/null || { echo "error: apt-get is required to download Nav2 packages" >&2; exit 1; }
command -v dpkg-deb >/dev/null || { echo "error: dpkg-deb is required to unpack Nav2 packages" >&2; exit 1; }

roots=(
  "ros-${ROS_DISTRO_NAME}-nav2-behaviors"
  "ros-${ROS_DISTRO_NAME}-nav2-bt-navigator"
  "ros-${ROS_DISTRO_NAME}-nav2-controller"
  "ros-${ROS_DISTRO_NAME}-nav2-lifecycle-manager"
  "ros-${ROS_DISTRO_NAME}-nav2-map-server"
  "ros-${ROS_DISTRO_NAME}-nav2-mppi-controller"
  "ros-${ROS_DISTRO_NAME}-nav2-navfn-planner"
  "ros-${ROS_DISTRO_NAME}-nav2-planner"
  "ros-${ROS_DISTRO_NAME}-nav2-regulated-pure-pursuit-controller"
  "ros-${ROS_DISTRO_NAME}-nav2-rviz-plugins"
  "ros-${ROS_DISTRO_NAME}-nav2-smoother"
  "ros-${ROS_DISTRO_NAME}-nav2-smac-planner"
  "ros-${ROS_DISTRO_NAME}-nav2-velocity-smoother"
)
local_system_packages=(
  "libgraphicsmagick-q16-3"
  "libgraphicsmagick++-q16-12"
)

mkdir -p "${CACHE_DIR}" "${LOCAL_ROOT}"
package_list="$(mktemp "${CACHE_DIR}/packages_XXXXXX.txt")"
trap 'rm -f "${package_list}"' EXIT

apt-cache depends --recurse --no-recommends --no-suggests \
  --no-conflicts --no-breaks --no-replaces --no-enhances "${roots[@]}" |
  sed -n 's/^[ |]*Depends: //p' |
  grep -E "^ros-${ROS_DISTRO_NAME}-" |
  grep -Ev 'rmw-connext|rmw-cyclonedds|rti-connext|iceoryx|cyclonedds' \
  > "${package_list}"
printf '%s\n' "${roots[@]}" >> "${package_list}"
sort -u -o "${package_list}" "${package_list}"

extract_local_package() {
  local package="$1"
  local deb_file
  local extracted
  local attempt
  extracted=false
  for attempt in 1 2 3; do
    deb_file="$(find "${CACHE_DIR}" -maxdepth 1 -type f -name "${package}_*.deb" -print -quit)"
    if [[ -z "${deb_file}" ]]; then
      echo "Downloading local Nav2 dependency: ${package}"
      (
        cd "${CACHE_DIR}"
        apt-get download "${package}"
      )
      deb_file="$(find "${CACHE_DIR}" -maxdepth 1 -type f -name "${package}_*.deb" -print -quit)"
    fi
    [[ -n "${deb_file}" ]] || continue
    if dpkg-deb -x "${deb_file}" "${LOCAL_ROOT}"; then
      extracted=true
      break
    fi
    echo "warning: corrupt/incomplete ${package} archive; downloading again (${attempt}/3)" >&2
    rm -f "${deb_file}"
  done
  [[ "${extracted}" == "true" ]] || { echo "error: failed to install ${package}" >&2; exit 1; }
}

while IFS= read -r package; do
  [[ -n "${package}" ]] || continue
  # Nav2 packages from the current repository require the shared C++ library.
  # Some older diagnostic_updater installations contain only headers/Python,
  # so always stage the repository candidate in the private prefix.
  if [[ "${package}" != "ros-${ROS_DISTRO_NAME}-diagnostic-updater" ]] &&
     dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q 'install ok installed'; then
    continue
  fi
  extract_local_package "${package}"
done < "${package_list}"

for package in "${local_system_packages[@]}"; do
  extract_local_package "${package}"
done

for relative_path in "${required_files[@]}"; do
  [[ -e "/opt/ros/${ROS_DISTRO_NAME}/${relative_path}" || -e "${LOCAL_ROS_PREFIX}/${relative_path}" ]] || {
    echo "error: Nav2 setup is incomplete; missing ${relative_path}" >&2
    exit 1
  }
done
[[ -e "/usr/lib/libGraphicsMagick++-Q16.so.12" ||
   -e "${LOCAL_ROOT}/usr/lib/libGraphicsMagick++-Q16.so.12" ]] || {
  echo "error: Nav2 setup is incomplete; missing libGraphicsMagick++-Q16.so.12" >&2
  exit 1
}
if ! ldconfig -p 2>/dev/null | grep -F 'libompl.so.18' >/dev/null &&
   ! compgen -G "${LOCAL_ROS_PREFIX}/lib/*-linux-gnu/libompl.so.18" >/dev/null; then
  echo "error: Nav2 setup is incomplete; missing libompl.so.18" >&2
  exit 1
fi

touch "${READY_MARKER}"
echo "Nav2 runtime installed locally at ${LOCAL_ROS_PREFIX}."
