#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
ROS_LOG_DIR="/tmp/ros-log"
ARM_TARGET_WAIT_SECONDS="${ARM_TARGET_WAIT_SECONDS:-10}"

usage() {
  cat <<'EOF'
Usage:
  ./send_arm_target_pose.sh X Y Z QX QY QZ QW [FRAME_ID]

Publishes one geometry_msgs/PoseStamped target to /arm_target_pose.
FRAME_ID defaults to base_link.
ARM_TARGET_WAIT_SECONDS controls how long to wait for a subscriber.

Example:
  ./send_arm_target_pose.sh 0.45 0.00 0.55 0.0 1.0 0.0 0.0 base_link
EOF
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

source_ros_setup() {
  set +u
  source "$ROS_SETUP"
  source "$WORKSPACE_SETUP"
  set -u
}

[[ "${1:-}" != "-h" && "${1:-}" != "--help" ]] || {
  usage
  exit 0
}
[[ $# -eq 7 || $# -eq 8 ]] || {
  usage >&2
  fail "expected 7 pose values plus optional frame_id"
}
[[ -f "$ROS_SETUP" ]] || fail "required file not found: $ROS_SETUP"
[[ -f "$WORKSPACE_SETUP" ]] || fail "required file not found: $WORKSPACE_SETUP"

x="$1"
y="$2"
z="$3"
qx="$4"
qy="$5"
qz="$6"
qw="$7"
frame_id="${8:-base_link}"

mkdir -p "$ROS_LOG_DIR"

source_ros_setup
export ROS_LOG_DIR

if ! timeout "$ARM_TARGET_WAIT_SECONDS" ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped "{
  header: {
    frame_id: '${frame_id}'
  },
  pose: {
    position: {x: ${x}, y: ${y}, z: ${z}},
    orientation: {x: ${qx}, y: ${qy}, z: ${qz}, w: ${qw}}
  }
}"; then
  fail "timed out publishing /arm_target_pose; is pose_goal_planner running?"
fi
