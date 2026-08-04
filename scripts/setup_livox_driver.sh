#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  setup_livox_driver.sh [--build] [--skip-apt] [--skip-sdk]

Prepares the vendored Livox ROS Driver2 package for a ROS 2 workspace.

What it does:
  1. Installs common Ubuntu/ROS build dependencies unless --skip-apt is used.
  2. Installs Livox-SDK2 to /usr/local unless it is already installed or --skip-sdk is used.
  3. Links this repo's third_party/livox_ros_driver2 into <workspace>/src/livox_ros_driver2.
  4. Optionally builds Livox, Super-LIO, and bringup with --build.

Examples:
  scripts/setup_livox_driver.sh
  scripts/setup_livox_driver.sh --build
  scripts/setup_livox_driver.sh --skip-apt --skip-sdk --build
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PACKAGE_DIR}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
LIVOX_SDK2_REPO="${LIVOX_SDK2_REPO:-https://github.com/Livox-SDK/Livox-SDK2.git}"
LIVOX_SDK2_REF="${LIVOX_SDK2_REF:-v1.3.1}"

BUILD="false"
SKIP_APT="false"
SKIP_SDK="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --build)
      BUILD="true"
      shift
      ;;
    --skip-apt)
      SKIP_APT="true"
      shift
      ;;
    --skip-sdk)
      SKIP_SDK="true"
      shift
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_cmd() {
  local command="$1"
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "error: required command not found: ${command}" >&2
    exit 1
  fi
}

detect_build_python() {
  if [[ -x /usr/bin/python3 ]]; then
    printf '%s\n' "/usr/bin/python3"
    return
  fi
  command -v python3
}

livox_sdk2_installed() {
  ldconfig -p 2>/dev/null | grep -q "liblivox_lidar_sdk_shared.so" ||
    [[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]]
}

install_apt_dependencies() {
  if [[ "${SKIP_APT}" == "true" ]]; then
    return
  fi

  require_cmd sudo
  sudo apt-get update
  sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libgoogle-glog-dev \
    libapr1-dev \
    libpcl-dev \
    libtbb-dev \
    python3-catkin-pkg-modules \
    "ros-${ROS_DISTRO_NAME}-ament-cmake-auto" \
    "ros-${ROS_DISTRO_NAME}-cyclonedds" \
    "ros-${ROS_DISTRO_NAME}-gtsam" \
    "ros-${ROS_DISTRO_NAME}-pcl-conversions" \
    "ros-${ROS_DISTRO_NAME}-pcl-ros" \
    "ros-${ROS_DISTRO_NAME}-rclcpp-components" \
    "ros-${ROS_DISTRO_NAME}-rosidl-default-generators"
}

install_livox_sdk2() {
  if [[ "${SKIP_SDK}" == "true" ]]; then
    return
  fi
  if livox_sdk2_installed; then
    echo "Livox-SDK2 already installed."
    return
  fi

  require_cmd git
  require_cmd cmake
  require_cmd make
  require_cmd sudo

  local sdk_dir="${PACKAGE_DIR}/third_party/Livox-SDK2"
  if [[ -d "${sdk_dir}/.git" ]]; then
    git -C "${sdk_dir}" fetch --tags --depth 1 origin "${LIVOX_SDK2_REF}"
    git -C "${sdk_dir}" checkout "${LIVOX_SDK2_REF}"
  elif [[ -d "${sdk_dir}" ]]; then
    echo "error: ${sdk_dir} exists but is not a git checkout." >&2
    exit 1
  else
    git clone --depth 1 --branch "${LIVOX_SDK2_REF}" "${LIVOX_SDK2_REPO}" "${sdk_dir}"
  fi

  cmake -S "${sdk_dir}" -B "${sdk_dir}/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${sdk_dir}/build" --parallel "$(nproc)"
  sudo cmake --install "${sdk_dir}/build"
  sudo ldconfig
}

prepare_vendored_driver() {
  local driver_dir="${PACKAGE_DIR}/third_party/livox_ros_driver2"
  local workspace_driver="${WORKSPACE_DIR}/src/livox_ros_driver2"

  if [[ ! -f "${driver_dir}/CMakeLists.txt" ]]; then
    echo "error: vendored livox_ros_driver2 not found at ${driver_dir}" >&2
    exit 1
  fi

  if [[ ! -f "${driver_dir}/package.xml" ]]; then
    cp "${driver_dir}/package_ROS2.xml" "${driver_dir}/package.xml"
  fi

  if [[ "${WORKSPACE_DIR}" == "${PACKAGE_DIR}" ]]; then
    return
  fi

  mkdir -p "${WORKSPACE_DIR}/src"
  if [[ -L "${workspace_driver}" ]]; then
    local target
    target="$(readlink "${workspace_driver}")"
    if [[ "${target}" != "${driver_dir}" ]]; then
      echo "Replacing Livox driver symlink:"
      echo "  old: ${workspace_driver} -> ${target}"
      echo "  new: ${workspace_driver} -> ${driver_dir}"
      rm "${workspace_driver}"
      ln -s "${driver_dir}" "${workspace_driver}"
    fi
  elif [[ -e "${workspace_driver}" ]]; then
    echo "error: ${workspace_driver} already exists and is not the managed symlink." >&2
    exit 1
  else
    ln -s "${driver_dir}" "${workspace_driver}"
    echo "Linked ${workspace_driver} -> ${driver_dir}"
  fi
}

build_workspace() {
  if [[ "${BUILD}" != "true" ]]; then
    return
  fi

  if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
    set -u
  fi

  cd "${WORKSPACE_DIR}"
  local build_python
  build_python="$(detect_build_python)"
  colcon build --packages-up-to livox_ros_driver2 super_lio autonomy_light \
    --base-paths \
      "${WORKSPACE_DIR}" \
      "${WORKSPACE_DIR}/third_party/livox_ros_driver2" \
      "${WORKSPACE_DIR}/third_party/super_lio_ros2" \
    --cmake-args \
      -DROS_EDITION=ROS2 \
      -DDISTRO_ROS="${ROS_DISTRO_NAME}" \
      -DPYTHON_EXECUTABLE="${build_python}" \
      -DPython3_EXECUTABLE="${build_python}"
}

install_apt_dependencies
install_livox_sdk2
prepare_vendored_driver
build_workspace

cat <<EOF

Livox driver setup finished.

Next:
  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
  source ${WORKSPACE_DIR}/install/setup.bash
  ros2 pkg prefix livox_ros_driver2
  ${PACKAGE_DIR}/launch.sh --real --mid360
EOF
