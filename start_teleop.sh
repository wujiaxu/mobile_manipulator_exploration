#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros-log}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: ./start_teleop.sh [--dry-run]

Starts keyboard teleoperation for the mobile base.

Controls:
  i    forward
  ,    backward
  j    rotate left
  l    rotate right
  k    stop
  q/z  increase/decrease max speed
EOF
}

require_file() {
  [[ -f "$1" ]] || fail "required file not found: $1"
}

dry_run=0
while (($#)); do
  case "$1" in
    --dry-run)
      dry_run=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      fail "unknown argument: $1"
      ;;
  esac
  shift
done

require_file "$ROS_SETUP"
require_file "$WORKSPACE_SETUP"
mkdir -p "$ROS_LOG_DIR"

teleop_command="source '${ROS_SETUP}'; source '${WORKSPACE_SETUP}'; export ROS_LOG_DIR='${ROS_LOG_DIR}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'; ros2 pkg prefix teleop_twist_keyboard >/dev/null || { echo 'Error: teleop_twist_keyboard is not installed. Run: sudo apt-get install ros-humble-teleop-twist-keyboard' >&2; exit 1; }; exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"

if ((dry_run)); then
  printf 'Teleop command: bash -lc %q\n' "$teleop_command"
  exit 0
fi

printf 'Starting keyboard teleop on /cmd_vel.\n'
printf 'Keep this terminal focused. Press k to stop, Ctrl+C to exit.\n'
exec bash -lc "$teleop_command"
