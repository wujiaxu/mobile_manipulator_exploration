#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TIMEOUT_SECONDS="${FKIE_CHECK_TIMEOUT_SECONDS:-5}"

source_ros_setup() {
  local setup_file="$1"
  if [[ ! -f "${setup_file}" ]]; then
    return
  fi

  # ROS setup scripts can read unset environment variables such as
  # AMENT_TRACE_SETUP_FILES, so source them with nounset disabled.
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source_ros_setup /opt/ros/humble/setup.bash
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source_ros_setup "${ROOT_DIR}/install/setup.bash"
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros-log}"
export ROS2CLI_NO_DAEMON="${ROS2CLI_NO_DAEMON:-1}"

FAILED_CHECKS=0

run_check() {
  local label="$1"
  local timeout_is_success="$2"
  local status=0
  shift
  shift

  printf '\n[%s]\n' "${label}"
  timeout "${TIMEOUT_SECONDS}" "$@" || status=$?

  if [[ "${status}" -eq 0 ]]; then
    return
  fi

  if [[ "${status}" -eq 124 && "${timeout_is_success}" == "yes" ]]; then
    printf '[OK] Timed sample window ended after %s seconds.\n' "${TIMEOUT_SECONDS}"
    return
  fi

  printf '[FAIL] Command exited with status %s.\n' "${status}" >&2
  FAILED_CHECKS=$((FAILED_CHECKS + 1))
}

run_check "OctoMap full map" no ros2 topic echo /octomap_full --once --no-daemon
run_check "Wrist depth compatibility cloud rate" yes ros2 topic hz /realsense/depth/points2
run_check "Wrist camera pose" no ros2 topic echo /camera_pose --once --no-daemon
run_check "FKIE-style robot footprint" no ros2 topic echo /mobile_manipulator_mbf/global_costmap/footprint --once --no-daemon
run_check "Occupied voxel projection rate" yes ros2 topic hz /octomap_occupied_points
run_check "MoveIt planning scene service" no bash -lc 'ros2 service list --no-daemon | grep /apply_planning_scene'

if [[ "${FAILED_CHECKS}" -gt 0 ]]; then
  printf '\n%d FKIE input check(s) failed.\n' "${FAILED_CHECKS}" >&2
  exit 1
fi

printf '\nAll FKIE input checks completed.\n'
