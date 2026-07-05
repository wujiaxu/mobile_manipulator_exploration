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

ros2 run mobile_manipulator_fkie_nbv nbv_arm_target_adapter \
  --ros-args \
  -p action_name:="${FKIE_NBV_ACTION_NAME:-/nbv_rrt}" \
  -p base_action_name:="${FKIE_NAV2_ACTION_NAME:-/navigate_to_pose}" \
  -p arm_action_name:="${FKIE_ARM_ACTION_NAME:-/move_arm}" \
  -p execution_marker_topic:="${FKIE_EXECUTION_MARKER_TOPIC:-/fkie_nbv/execution_markers}" \
  -p target_topic:="${FKIE_ARM_TARGET_TOPIC:-/arm_target_pose}" \
  -p named_target_topic:="${FKIE_ARM_NAMED_TARGET_TOPIC:-/arm_named_target}" \
  -p frame_id:="${FKIE_WORLD_FRAME:-map}" \
  -p robot_frame:="${FKIE_ROBOT_FRAME:-base_link}" \
  -p end_effector_frame:="${FKIE_EEF_FRAME:-link_eef}" \
  -p camera_frame:="${FKIE_CAMERA_FRAME:-wrist_camera_color_optical_frame}" \
  -p min_x:="${FKIE_BOUNDARY_MIN_X:--3.0}" \
  -p max_x:="${FKIE_BOUNDARY_MAX_X:-3.0}" \
  -p min_y:="${FKIE_BOUNDARY_MIN_Y:--3.0}" \
  -p max_y:="${FKIE_BOUNDARY_MAX_Y:-3.0}" \
  -p min_z:="${FKIE_BOUNDARY_MIN_Z:-0.4}" \
  -p max_z:="${FKIE_BOUNDARY_MAX_Z:-1.4}" \
  -p measurement_grid_size:="${FKIE_MEASUREMENT_GRID_SIZE:-0.5}" \
  -p wait_timeout_s:="${FKIE_WAIT_TIMEOUT_S:-10.0}" \
  -p base_goal_result_timeout_s:="${FKIE_BASE_GOAL_RESULT_TIMEOUT_S:-60.0}" \
  -p auto_explore:="${FKIE_AUTO_EXPLORE:-false}" \
  -p max_exploration_iterations:="${FKIE_MAX_EXPLORATION_ITERATIONS:-20}" \
  -p map_update_wait_s:="${FKIE_MAP_UPDATE_WAIT_S:-2.0}" \
  -p continue_on_motion_failure:="${FKIE_CONTINUE_ON_MOTION_FAILURE:-true}" \
  -p continue_base_on_stow_failure:="${FKIE_CONTINUE_BASE_ON_STOW_FAILURE:-true}" \
  -p stow_arm_before_base_motion:="${FKIE_STOW_ARM_BEFORE_BASE_MOTION:-true}" \
  -p publish_debug_arm_targets:="${FKIE_PUBLISH_DEBUG_ARM_TARGETS:-false}" \
  -p retry_arm_after_base_fallback:="${FKIE_RETRY_ARM_AFTER_BASE_FALLBACK:-true}" \
  -p base_goal_standoff:="${FKIE_BASE_GOAL_STANDOFF:-0.45}" \
  -p arm_named_target_retry_count:="${FKIE_ARM_NAMED_TARGET_RETRY_COUNT:-1}" \
  -p arm_named_target_retry_delay_s:="${FKIE_ARM_NAMED_TARGET_RETRY_DELAY_S:-1.0}" \
  -p stow_arm_wait_s:="${FKIE_STOW_ARM_WAIT_S:-8.0}" \
  -p arm_goal_range:="${FKIE_ARM_GOAL_RANGE:-1.3}" \
  -p arm_goal_publish_delay_s:="${FKIE_ARM_GOAL_PUBLISH_DELAY_S:-8.0}"
