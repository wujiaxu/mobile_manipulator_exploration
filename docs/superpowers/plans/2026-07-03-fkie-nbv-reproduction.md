# FKIE NBV Reproduction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reproduce the FKIE online next-best-view planner behavior on the Isaac Sim mobile manipulator baseline.

**Architecture:** Keep the current ROS 2 Humble Isaac/Nav2/MoveIt stack as the robot execution system. Use Docker only where it reduces risk for the original ROS Noetic/Catkin FKIE code; prefer a ROS 2 native port once the planner interfaces are fully understood.

**Tech Stack:** ROS 2 Humble, Isaac Sim, Nav2, MoveIt 2, OctoMap, Docker, optional ROS Noetic container for FKIE source inspection/build.

---

## Current Baseline

- Branch: `feature/fkie-nbv-reproduction`
- Implemented FKIE-like input topics:
  - `/octomap_full`
  - `/realsense/depth/points2`
  - `/camera_pose`
  - `/mobile_manipulator_mbf/global_costmap/footprint`
- Implemented execution pieces:
  - Nav2 point-goal navigation
  - MoveIt arm pose execution
  - MoveIt collision boxes from `/octomap_occupied_points`
- Local reference:
  - `docs/references/fkie_nbv_planner_summary.md`
  - `docs/references/fkie_nbv_repo_interface.md`
  - `docs/references/fkie_reproduction_runtime.md`
- First ROS 2 FKIE reproduction packages:
  - `mobile_manipulator_fkie_msgs`
  - `mobile_manipulator_fkie_nbv`

## Reproduction Strategy

Use three milestones:

1. **Interface Discovery**
   - Clone and inspect FKIE repo.
   - Record action definitions, parameters, frames, namespace assumptions, and planner outputs.
   - Decide ROS 2 port vs ROS 1 Docker bridge from evidence.

2. **Minimal Planner Execution**
   - Get FKIE planner to compute a next best view using our current topics.
   - First target is planner output only, not full autonomous execution.

3. **Robot Execution Integration**
   - Convert selected NBV result into Nav2 base goal plus MoveIt wrist camera pose.
   - Run repeated plan-execute-update cycles in the controlled rooms environment.

## Task 1: Clone and Freeze FKIE Source Snapshot

**Files:**

- Create: `third_party/fkie-nbv-planner/`
- Create: `docs/references/fkie_nbv_repo_interface.md`

- [x] Clone the repository:

```bash
mkdir -p third_party
git clone https://github.com/fkie/fkie-nbv-planner.git third_party/fkie-nbv-planner
```

- [x] Record commit hash:

```bash
git -C third_party/fkie-nbv-planner rev-parse HEAD
```

- [x] Inspect files:

```bash
find third_party/fkie-nbv-planner -maxdepth 3 -type f | sort
```

- [x] Save repo interface notes in `docs/references/fkie_nbv_repo_interface.md`.

## Task 2: Inspect FKIE Interfaces

**Files:**

- Modify: `docs/references/fkie_nbv_repo_interface.md`

- [x] Inspect action definitions:

```bash
find third_party/fkie-nbv-planner -type f \( -name '*.action' -o -name '*.srv' -o -name '*.msg' \) -print -exec sed -n '1,220p' {} \;
```

- [x] Inspect launch/config:

```bash
find third_party/fkie-nbv-planner -type f \( -name '*.launch' -o -name '*.yaml' -o -name '*.xml' \) -print -exec sed -n '1,220p' {} \;
```

- [x] Record:
  - action server name,
  - goal fields,
  - result fields,
  - feedback fields,
  - required topics,
  - output topics/actions,
  - expected frames,
  - costmap/MBF assumptions.

## Task 3: Decide Port vs Docker Bridge

**Files:**

- Modify: `docs/references/fkie_nbv_repo_interface.md`
- Create one of:
  - `docs/superpowers/plans/2026-07-03-fkie-nbv-ros2-port.md`
  - `docs/superpowers/plans/2026-07-03-fkie-nbv-docker-bridge.md`

- [x] Try dependency inventory:

```bash
rg -n "find_package|catkin|roscpp|rospy|move_base|mbf|octomap|pcl|tf|actionlib" third_party/fkie-nbv-planner -S
```

- [x] Choose ROS 2 port if dependencies are mostly standard C++ ROS interfaces and OctoMap/PCL.

- [x] Choose Docker bridge only if direct ROS 2 port is too large for the first reproduction pass.

Decision: use a ROS 2 native port path first. The FKIE package is mostly C++ planner logic plus standard ROS messages/actions, OctoMap, PCL, and TF dependencies. A Docker/ROS 1 bridge would add topic/action bridging complexity before the planner interface is validated.

## Task 4: Build Reproduction Harness

**Files:**

- Create: `scripts/check_fkie_inputs.sh`
- Create: `docs/references/fkie_reproduction_runtime.md`

- [x] Add a shell script that checks:

```bash
ros2 topic echo /octomap_full --once
ros2 topic hz /realsense/depth/points2
ros2 topic echo /camera_pose --once
ros2 topic echo /mobile_manipulator_mbf/global_costmap/footprint --once
ros2 topic hz /octomap_occupied_points
ros2 service list | grep apply_planning_scene
```

- [ ] Run the harness in the controlled rooms environment.

- [x] Save expected output examples in `docs/references/fkie_reproduction_runtime.md`.

## Task 5: Minimal Planner Invocation

**Files:**

- `mobile_manipulator_fkie_msgs/`
- `mobile_manipulator_fkie_nbv/`

- [x] Create a ROS 2 wrapper or port package that can receive one FKIE-style planner request.

- [x] Use fixed exploration boundary matching the controlled rooms environment.

- [x] Verify planner returns a candidate next view without commanding the robot.

Current status: the action contract is available as
`mobile_manipulator_fkie_msgs/action/NbvPlanner`, and the planner action
server starts as `/fkie_nbv_planner` with action name `nbv_rrt`. It validates
that camera pose, full OctoMap, and FKIE-style footprint inputs are present,
then samples deterministic camera-pose candidates inside the request boundary
and scores them by unknown/free-space ray gain in `/octomap_full`.

This is the first ROS 2 native planner kernel. It ports FKIE's core
information-gain idea, but not the full upstream RRT tree expansion,
frontier-cache management, path optimizer, or RViz marker set yet.

Verified build command:

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_EXE \
  -u CONDA_PYTHON_EXE -u CONDA_PROMPT_MODIFIER -u CONDA_SHLVL \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin \
  bash -lc 'source /opt/ros/humble/setup.bash; colcon build --packages-select mobile_manipulator_fkie_msgs mobile_manipulator_fkie_nbv --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so'
```

Live planner request:

```bash
./scripts/send_fkie_nbv_goal.sh
```

## Task 6: Execution Adapter

**Files:**

- Expected create path:
  - `mobile_manipulator_fkie_nbv/src/nbv_execution_adapter.cpp`

- [ ] Convert selected base pose into Nav2 goal.

- [ ] Convert selected camera pose into MoveIt target for `link_eef` or camera frame.

- [ ] Execute in sequence:
  - base navigation,
  - arm/camera positioning,
  - map update wait,
  - next planner call.

## Task 7: Reproduction Scenario and Metrics

**Files:**

- Create: `docs/references/fkie_reproduction_metrics.md`
- Optional create: `mobile_manipulator_experiments/`

- [ ] Use the controlled rooms environment as the first reproducible test scene.

- [ ] Record metrics:
  - explored volume or known voxel count,
  - occupied voxel count,
  - number of NBV iterations,
  - traveled base distance,
  - arm planning success/failure,
  - runtime per iteration.

## Immediate Next Step

Validate `./scripts/send_fkie_nbv_goal.sh` in the live Isaac session. If it
returns a useful camera pose, add the execution adapter that sends the selected
view to MoveIt first. Add Nav2 base movement only after the arm-only NBV loop is
stable.
