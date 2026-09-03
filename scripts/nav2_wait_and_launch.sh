#!/usr/bin/env bash
set -Eeuo pipefail

PARAMS_FILE="${1:?Nav2 parameter file is required}"
USE_SIM_TIME="${2:-false}"
ODOM_TOPIC="${3:-/lio/odom}"
WAIT_TIMEOUT="${4:-120}"
shift 4 || true

echo "autonomy-nav2: waiting for ${ODOM_TOPIC} before lifecycle activation"
## The LIO node creates /lio/odom only after IMU/LiDAR initialisation.  Without
## an explicit type, `ros2 topic echo` can fail immediately during its initial
## type lookup (before the configured timeout has any effect).  Subscribe as
## Odometry from the outset and avoid the ROS CLI daemon cache.
if ! timeout "${WAIT_TIMEOUT}" ros2 topic echo --no-daemon --once \
    "${ODOM_TOPIC}" nav_msgs/msg/Odometry >/dev/null 2>&1; then
  echo "error: no LIO odometry received on ${ODOM_TOPIC} within ${WAIT_TIMEOUT}s" >&2
  exit 1
fi

echo "autonomy-nav2: odometry ready; starting Nav2"
exec ros2 launch autonomy_light nav2_live.launch.py \
  "params_file:=${PARAMS_FILE}" "use_sim_time:=${USE_SIM_TIME}" "$@"
