#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source_ros_setup() {
  local setup_file="$1"
  if [[ ! -f "${setup_file}" ]]; then
    return
  fi

  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "${ROOT_DIR}/install/setup.bash"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros-log}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

ACTION_NAME="${FKIE_NBV_ACTION_NAME:-/nbv_rrt}"
FRAME_ID="${FKIE_WORLD_FRAME:-map}"
MIN_X="${FKIE_BOUNDARY_MIN_X:--3.0}"
MAX_X="${FKIE_BOUNDARY_MAX_X:-3.0}"
MIN_Y="${FKIE_BOUNDARY_MIN_Y:--3.0}"
MAX_Y="${FKIE_BOUNDARY_MAX_Y:-3.0}"
MIN_Z="${FKIE_BOUNDARY_MIN_Z:-0.4}"
MAX_Z="${FKIE_BOUNDARY_MAX_Z:-1.4}"
GRID_SIZE="${FKIE_MEASUREMENT_GRID_SIZE:-0.5}"

export FKIE_NBV_ACTION_NAME="${ACTION_NAME}"
export FKIE_WORLD_FRAME="${FRAME_ID}"
export FKIE_BOUNDARY_MIN_X="${MIN_X}"
export FKIE_BOUNDARY_MAX_X="${MAX_X}"
export FKIE_BOUNDARY_MIN_Y="${MIN_Y}"
export FKIE_BOUNDARY_MAX_Y="${MAX_Y}"
export FKIE_BOUNDARY_MIN_Z="${MIN_Z}"
export FKIE_BOUNDARY_MAX_Z="${MAX_Z}"
export FKIE_MEASUREMENT_GRID_SIZE="${GRID_SIZE}"

ros2 run mobile_manipulator_fkie_nbv send_fkie_nbv_goal_client \
  --ros-args \
  -p action_name:="${ACTION_NAME}" \
  -p frame_id:="${FRAME_ID}" \
  -p min_x:="${MIN_X}" \
  -p max_x:="${MAX_X}" \
  -p min_y:="${MIN_Y}" \
  -p max_y:="${MAX_Y}" \
  -p min_z:="${MIN_Z}" \
  -p max_z:="${MAX_Z}" \
  -p measurement_grid_size:="${GRID_SIZE}"
