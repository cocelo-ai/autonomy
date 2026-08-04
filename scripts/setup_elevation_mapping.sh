#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: setup_elevation_mapping.sh [--skip-apt]

Installs the ROS 2 Grid Map, PCL, TF, and RealSense dependencies needed by the
vendored ETH/ANYbotics robot-centric elevation_mapping package. KINDR and
kindr_ros are vendored because they are not released on every ROS 2 Humble target.
EOF
}

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
SKIP_APT="false"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --skip-apt) SKIP_APT="true"; shift ;;
    *) echo "error: unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ "${SKIP_APT}" == "true" ]] && exit 0
command -v sudo >/dev/null || { echo "error: sudo is required (or use --skip-apt)" >&2; exit 1; }

sudo apt-get update
PACKAGES=(
  libboost-system-dev
  libboost-thread-dev
  libeigen3-dev
  python3-opencv
  python3-yaml
  "ros-${ROS_DISTRO_NAME}-grid-map"
  "ros-${ROS_DISTRO_NAME}-pcl-ros"
  "ros-${ROS_DISTRO_NAME}-realsense2-camera"
  "ros-${ROS_DISTRO_NAME}-tf2-eigen"
  "ros-${ROS_DISTRO_NAME}-tf2-geometry-msgs"
)
sudo apt-get install -y "${PACKAGES[@]}"
