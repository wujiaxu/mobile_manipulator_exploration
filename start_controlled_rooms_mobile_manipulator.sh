#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
ISAAC_PYTHON="/home/user/anaconda3/envs/env_isaaclab/bin/python"
ISAAC_RUNNER="${WORKSPACE}/isaac_sim/scripts/run_factory_navigation.py"
ROOMS_USD="${WORKSPACE}/isaac_sim/assets/environments/controlled_rooms/controlled_rooms.usd"
ROBOT_USD="${WORKSPACE}/isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd"
ISAAC_ROS_LIB="/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
ROS_LOG_DIR="/tmp/ros-log"
ISAAC_STARTUP_WAIT_SECONDS="${ISAAC_STARTUP_WAIT_SECONDS:-15}"
NAV_STARTUP_WAIT_SECONDS="${NAV_STARTUP_WAIT_SECONDS:-8}"
ROBOT_SPAWN=(-4.5 -4.5 0.6 0.0)

isaac_pid=""
navigation_pid=""
moveit_pid=""
octomap_pid=""
cleaning_up=0

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: ./start_controlled_rooms_mobile_manipulator.sh [--use-moveit-rviz] [--dry-run]

Starts the controlled rooms Isaac scene, SLAM Toolbox, Nav2, RViz2, MoveIt,
and the Isaac arm trajectory bridge.

Options:
  --use-moveit-rviz  Also start MoveIt RViz for GUI arm pose targets.
                     This opens a second RViz window in addition to Nav2 RViz.
  --use-wrist-octomap
                     Build a map-frame OctoMap from the wrist depth camera.
  --dry-run          Print the commands without starting processes.
EOF
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
  stop_process_group "$octomap_pid" "Wrist OctoMap"
  stop_process_group "$moveit_pid" "MoveIt"
  stop_process_group "$navigation_pid" "SLAM/Nav2/RViz"
  stop_process_group "$isaac_pid" "Isaac Sim"
  wait "$octomap_pid" 2>/dev/null || true
  wait "$moveit_pid" 2>/dev/null || true
  wait "$navigation_pid" 2>/dev/null || true
  wait "$isaac_pid" 2>/dev/null || true
  return "$status"
}

trap cleanup INT TERM EXIT

dry_run=0
use_moveit_rviz=false
use_wrist_octomap=false
while (($#)); do
  case "$1" in
    --dry-run)
      dry_run=1
      ;;
    --use-moveit-rviz|use_moveit_rviz:=true)
      use_moveit_rviz=true
      ;;
    --use-wrist-octomap|use_wrist_octomap:=true)
      use_wrist_octomap=true
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
require_file "$ISAAC_RUNNER"
require_file "$ROOMS_USD"
require_file "$ROBOT_USD"
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
  --factory-usd "$ROOMS_USD"
  --robot-usd "$ROBOT_USD"
  --spawn "${ROBOT_SPAWN[@]}"
)

navigation_shell_command="source '${ROS_SETUP}'; source '${WORKSPACE_SETUP}'; export ROS_LOG_DIR='${ROS_LOG_DIR}' RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; exec ros2 launch mobile_manipulator_navigation mapping.launch.py use_rviz:=true use_sim_time:=true use_robot_state_publisher:=true"
moveit_shell_command="source '${ROS_SETUP}'; source '${WORKSPACE_SETUP}'; export ROS_LOG_DIR='${ROS_LOG_DIR}' RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; exec ros2 launch mobile_manipulator_moveit_config moveit_isaac.launch.py use_rviz:=${use_moveit_rviz} use_sim_time:=true use_robot_state_publisher:=false"
octomap_shell_command="source '${ROS_SETUP}'; source '${WORKSPACE_SETUP}'; export ROS_LOG_DIR='${ROS_LOG_DIR}' RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; exec ros2 launch mobile_manipulator_navigation wrist_octomap.launch.py use_sim_time:=true"

if ((dry_run)); then
  printf 'Isaac command:'
  printf ' %q' "${isaac_command[@]}"
  printf '\nNavigation command: bash -lc %q\n' "$navigation_shell_command"
  printf 'MoveIt command: bash -lc %q\n' "$moveit_shell_command"
  if [[ "$use_wrist_octomap" == true ]]; then
    printf 'Wrist OctoMap command: bash -lc %q\n' "$octomap_shell_command"
  fi
  exit 0
fi

[[ "$ISAAC_STARTUP_WAIT_SECONDS" =~ ^[1-9][0-9]*$ ]] || \
  fail "ISAAC_STARTUP_WAIT_SECONDS must be a positive integer"
[[ "$NAV_STARTUP_WAIT_SECONDS" =~ ^[1-9][0-9]*$ ]] || \
  fail "NAV_STARTUP_WAIT_SECONDS must be a positive integer"

printf 'Starting Isaac Sim controlled rooms scene...\n'
setsid "${isaac_command[@]}" &
isaac_pid=$!

for ((second = 1; second <= ISAAC_STARTUP_WAIT_SECONDS; ++second)); do
  if ! kill -0 "$isaac_pid" 2>/dev/null; then
    wait "$isaac_pid" || true
    fail "Isaac Sim exited during startup"
  fi
  sleep 1
done

printf 'Starting SLAM Toolbox, Nav2, and RViz2...\n'
setsid bash -lc "$navigation_shell_command" &
navigation_pid=$!

for ((second = 1; second <= NAV_STARTUP_WAIT_SECONDS; ++second)); do
  if ! kill -0 "$isaac_pid" 2>/dev/null; then
    wait "$isaac_pid" || true
    fail "Isaac Sim exited while waiting for Nav2 startup"
  fi
  if ! kill -0 "$navigation_pid" 2>/dev/null; then
    wait "$navigation_pid" || true
    fail "SLAM/Nav2/RViz exited during startup"
  fi
  sleep 1
done

printf 'Starting MoveIt and arm trajectory bridge...\n'
printf 'Use RViz2 Navigation2 Goal for base point-goal navigation.\n'
printf 'Publish geometry_msgs/PoseStamped to /arm_target_pose for arm pose goals.\n'
printf 'Press Ctrl+C to stop the complete stack.\n'
setsid bash -lc "$moveit_shell_command" &
moveit_pid=$!

if [[ "$use_wrist_octomap" == true ]]; then
  printf 'Starting wrist depth OctoMap in map frame...\n'
  setsid bash -lc "$octomap_shell_command" &
  octomap_pid=$!
fi

set +e
if [[ "$use_wrist_octomap" == true ]]; then
  wait -n "$isaac_pid" "$navigation_pid" "$moveit_pid" "$octomap_pid"
else
  wait -n "$isaac_pid" "$navigation_pid" "$moveit_pid"
fi
status=$?
set -e

if ! kill -0 "$isaac_pid" 2>/dev/null; then
  printf 'Isaac Sim exited; stopping ROS stacks.\n' >&2
elif ! kill -0 "$navigation_pid" 2>/dev/null; then
  printf 'SLAM/Nav2/RViz exited; stopping remaining processes.\n' >&2
elif [[ "$use_wrist_octomap" == true ]] && ! kill -0 "$octomap_pid" 2>/dev/null; then
  printf 'Wrist OctoMap exited; stopping remaining processes.\n' >&2
else
  printf 'MoveIt exited; stopping remaining processes.\n' >&2
fi
exit "$status"
