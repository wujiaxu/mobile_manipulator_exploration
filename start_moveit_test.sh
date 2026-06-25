#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
ISAAC_PYTHON="/home/user/anaconda3/envs/env_isaaclab/bin/python"
ISAAC_RUNNER="${WORKSPACE}/isaac_sim/scripts/run_mobile_manipulator.py"
ISAAC_ROS_LIB="/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
ROS_LOG_DIR="/tmp/ros-log"
ISAAC_STARTUP_WAIT_SECONDS="${ISAAC_STARTUP_WAIT_SECONDS:-12}"

isaac_pid=""
moveit_pid=""
cleaning_up=0

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || fail "required file not found: $1"
}

require_executable() {
  [[ -x "$1" ]] || fail "required executable not found: $1"
}

stop_process_group() {
  local pid="$1"
  local label="$2"

  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0

  printf 'Stopping %s...\n' "$label"
  kill -INT -- "-${pid}" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done

  printf '%s did not stop after SIGINT; sending SIGTERM.\n' "$label" >&2
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  if ((cleaning_up)); then
    return "$status"
  fi
  cleaning_up=1
  set +e
  stop_process_group "$moveit_pid" "MoveIt/RViz"
  stop_process_group "$isaac_pid" "Isaac Sim"
  wait "$moveit_pid" 2>/dev/null || true
  wait "$isaac_pid" 2>/dev/null || true
  return "$status"
}

trap cleanup INT TERM EXIT

require_file "$ROS_SETUP"
require_file "$WORKSPACE_SETUP"
require_file "$ISAAC_RUNNER"
require_executable "$ISAAC_PYTHON"
mkdir -p "$ROS_LOG_DIR"

isaac_command=(
  env
  ROS_DISTRO=humble
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  "LD_LIBRARY_PATH=${ISAAC_ROS_LIB}:${LD_LIBRARY_PATH:-}"
  PYTHONUNBUFFERED=1
  "$ISAAC_PYTHON"
  "$ISAAC_RUNNER"
  --pose-report-period 1
)

moveit_shell_command="source '${ROS_SETUP}'; source '${WORKSPACE_SETUP}'; export ROS_LOG_DIR='${ROS_LOG_DIR}' RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; exec ros2 launch mobile_manipulator_moveit_config moveit_isaac.launch.py use_rviz:=true use_sim_time:=true"

if [[ "${1:-}" == "--dry-run" ]]; then
  printf 'Isaac command:'
  printf ' %q' "${isaac_command[@]}"
  printf '\nMoveIt command: bash -lc %q\n' "$moveit_shell_command"
  exit 0
fi

[[ $# -eq 0 ]] || fail "usage: $0 [--dry-run]"
[[ "$ISAAC_STARTUP_WAIT_SECONDS" =~ ^[1-9][0-9]*$ ]] || \
  fail "ISAAC_STARTUP_WAIT_SECONDS must be a positive integer"

printf 'Starting Isaac Sim...\n'
setsid "${isaac_command[@]}" &
isaac_pid=$!

for ((second = 1; second <= ISAAC_STARTUP_WAIT_SECONDS; ++second)); do
  if ! kill -0 "$isaac_pid" 2>/dev/null; then
    wait "$isaac_pid" || true
    fail "Isaac Sim exited during startup"
  fi
  sleep 1
done

printf 'Starting MoveIt and RViz2...\n'
printf 'Press Ctrl+C to stop the complete test stack.\n'
setsid bash -lc "$moveit_shell_command" &
moveit_pid=$!

set +e
wait -n "$isaac_pid" "$moveit_pid"
status=$?
set -e

if ! kill -0 "$isaac_pid" 2>/dev/null; then
  printf 'Isaac Sim exited; stopping MoveIt/RViz.\n' >&2
else
  printf 'MoveIt/RViz exited; stopping Isaac Sim.\n' >&2
fi
exit "$status"
