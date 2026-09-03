#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/build.sh [options]

Build Livox, Super-LIO, LiDAR-only elevation, and Nav2.

Options:
  --skip-apt          Do not install Ubuntu/ROS dependencies.
  --skip-sdk          Do not build/install Livox-SDK2.
  --clean             Remove this workspace's build/install/log for these packages first.
  --setup-only        Install runtime/build dependencies only; do not run colcon build.
  --packages PKGS     Packages to build. Default: livox_ros_driver2 super_lio autonomy_light.
  --ros-distro NAME   ROS distro. Default: ROS_DISTRO or humble.
  -h, --help          Show this help.

Examples:
  ./scripts/build.sh
  ./scripts/build.sh --skip-sdk
  ./scripts/build.sh --clean
  ./scripts/build.sh --setup-only
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
SKIP_APT="false"
SKIP_SDK="false"
CLEAN="false"
SETUP_ONLY="false"
PACKAGES=(livox_ros_driver2 super_lio autonomy_light)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --skip-apt)
      SKIP_APT="true"
      shift
      ;;
    --skip-sdk)
      SKIP_SDK="true"
      shift
      ;;
    --clean)
      CLEAN="true"
      shift
      ;;
    --setup-only)
      SETUP_ONLY="true"
      shift
      ;;
    --packages)
      shift
      PACKAGES=()
      while [[ $# -gt 0 && "$1" != --* ]]; do
        PACKAGES+=("$1")
        shift
      done
      if [[ "${#PACKAGES[@]}" -eq 0 ]]; then
        echo "error: --packages requires at least one package name" >&2
        exit 2
      fi
      ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"
      shift 2
      ;;
    --ros-distro=*)
      ROS_DISTRO_NAME="${1#--ros-distro=}"
      shift
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SETUP_ARGS=()
[[ "${SKIP_APT}" == "true" ]] && SETUP_ARGS+=(--skip-apt)
[[ "${SKIP_SDK}" == "true" ]] && SETUP_ARGS+=(--skip-sdk)

detect_build_python() {
  if [[ -x /usr/bin/python3 ]]; then
    printf '%s\n' "/usr/bin/python3"
    return
  fi
  command -v python3
}

BUILD_PYTHON="$(detect_build_python)"

if [[ "${CLEAN}" == "true" ]]; then
  # Super-LIO's vendored `basic` package is built as an implicit dependency.
  # Clear it together with the requested packages so a relocated workspace
  # cannot retain an absolute path in build/basic/CMakeCache.txt.
  CLEAN_PACKAGES=(basic "${PACKAGES[@]}")
  echo "Cleaning build/install/log for: ${CLEAN_PACKAGES[*]}"
  for pkg in "${CLEAN_PACKAGES[@]}"; do
    rm -rf "${WORKSPACE_DIR}/build/${pkg}" \
      "${WORKSPACE_DIR}/install/${pkg}" \
      "${WORKSPACE_DIR}/log"
  done
fi

echo "Preparing vendored Livox driver"
ROS_DISTRO="${ROS_DISTRO_NAME}" "${SCRIPT_DIR}/setup_livox_driver.sh" "${SETUP_ARGS[@]}"
if [[ "${SKIP_APT}" != "true" ]]; then
  echo "Preparing Nav2 runtime"
  ROS_DISTRO="${ROS_DISTRO_NAME}" "${SCRIPT_DIR}/setup_nav2.sh" \
    --ros-distro "${ROS_DISTRO_NAME}"
else
  echo "Skipping Nav2 runtime setup (--skip-apt); an existing Nav2 installation is required."
fi
if [[ "${SETUP_ONLY}" == "true" ]]; then
  echo "Setup complete."
  exit 0
fi

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
else
  echo "error: /opt/ros/${ROS_DISTRO_NAME}/setup.bash not found" >&2
  exit 1
fi

cd "${WORKSPACE_DIR}"
echo "Building packages: ${PACKAGES[*]}"
# This build script uses a normal install tree.  Explicitly reset a cache left
# by a prior manual `colcon build --symlink-install`; otherwise
# ament_cmake_python tries to replace its generated package directory with a
# symlink and the build fails before compiling autonomy_light.
colcon build --packages-up-to "${PACKAGES[@]}" \
  --base-paths \
    "${WORKSPACE_DIR}" \
    "${WORKSPACE_DIR}/src/third_party/livox_ros_driver2" \
    "${WORKSPACE_DIR}/src/third_party/super_lio_ros2" \
  --cmake-args \
    -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="${ROS_DISTRO_NAME}" \
    -DPYTHON_EXECUTABLE="${BUILD_PYTHON}" \
    -DPython3_EXECUTABLE="${BUILD_PYTHON}"

cat <<EOF

Build finished.

Next:
  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
  source ${WORKSPACE_DIR}/install/setup.bash
  ${SCRIPT_DIR}/launch.sh --real --mid360
EOF
