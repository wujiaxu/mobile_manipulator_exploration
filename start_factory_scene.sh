#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_PYTHON="/home/user/anaconda3/envs/env_isaaclab/bin/python"
ISAAC_RUNNER="${WORKSPACE}/isaac_sim/scripts/run_factory_navigation.py"
FACTORY_USD="${WORKSPACE}/isaac_sim/assets/environments/compact_factory/compact_factory.usd"
ROBOT_USD="${WORKSPACE}/isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd"
ISAAC_ROS_LIB="/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"

disable_lidar=0
dry_run=0
isaac_pid=""
cleaning_up=0

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: ./start_factory_scene.sh [--disable-lidar] [--dry-run]

Starts only the Isaac Sim factory scene with the mobile manipulator.
It does not start Nav2, SLAM Toolbox, or RViz2.
EOF
}

require_file() {
  [[ -f "$1" ]] || fail "required file not found: $1"
}

require_executable() {
  [[ -x "$1" ]] || fail "required executable not found: $1"
}

cleanup() {
  local status=$?
  if ((cleaning_up)); then
    return "$status"
  fi
  cleaning_up=1
  set +e
  if [[ -n "$isaac_pid" ]] && kill -0 "$isaac_pid" 2>/dev/null; then
    printf 'Stopping Isaac Sim...\n'
    kill -INT -- "-${isaac_pid}" 2>/dev/null || kill -INT "$isaac_pid" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$isaac_pid" 2>/dev/null || return "$status"
      sleep 0.1
    done
    printf 'Isaac Sim did not stop after SIGINT; sending SIGTERM.\n' >&2
    kill -TERM -- "-${isaac_pid}" 2>/dev/null || kill -TERM "$isaac_pid" 2>/dev/null || true
  fi
  return "$status"
}

trap cleanup INT TERM EXIT

while (($#)); do
  case "$1" in
    --disable-lidar)
      disable_lidar=1
      ;;
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

require_file "$ISAAC_RUNNER"
require_file "$FACTORY_USD"
require_file "$ROBOT_USD"
require_executable "$ISAAC_PYTHON"

isaac_command=(
  env
  ROS_DISTRO=humble
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  "LD_LIBRARY_PATH=${ISAAC_ROS_LIB}:${LD_LIBRARY_PATH:-}"
  PYTHONUNBUFFERED=1
  "$ISAAC_PYTHON"
  "$ISAAC_RUNNER"
  --factory-usd "$FACTORY_USD"
  --robot-usd "$ROBOT_USD"
)

if ((disable_lidar)); then
  isaac_command+=(--disable-lidar)
fi

if ((dry_run)); then
  printf 'Isaac factory scene command:'
  printf ' %q' "${isaac_command[@]}"
  printf '\n'
  exit 0
fi

printf 'Starting Isaac Sim factory scene with mobile manipulator...\n'
printf 'Press Ctrl+C in this terminal to stop Isaac Sim.\n'
setsid "${isaac_command[@]}" &
isaac_pid=$!
wait "$isaac_pid"
