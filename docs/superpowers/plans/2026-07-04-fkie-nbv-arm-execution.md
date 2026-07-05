# FKIE NBV Arm/Base Execution Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute FKIE NBV candidates with the paper-style policy: try the manipulator along the branch toward the target, and if the final EEF target is not reachable, move the mobile base by projecting the farthest reachable EEF node on that RRT branch onto the ground plane.

**Architecture:** Keep the FKIE planner responsible for scoring camera candidates and, next, preserving the branch/path of EEF poses leading to a selected target. Keep the existing MoveIt bridge responsible for arm planning/execution from `/arm_target_pose`. Add an execution adapter layer that converts NBV camera poses to desired `link_eef` poses, selects the farthest branch node that is still arm-attemptable, and otherwise sends a Nav2 `NavigateToPose` goal generated from that branch node's ground projection.

**Tech Stack:** ROS 2 Humble, `rclcpp_action`, `tf2_ros`, `nav2_msgs/action/NavigateToPose`, MoveIt 2 pose target bridge, existing `mobile_manipulator_fkie_nbv` package.

---

## Evidence

- `/nbv_rrt` returns a non-empty candidate, for example `frame=map x=-3.429 y=-5.143 z=0.567 gain=0.288000`.
- `nbv_arm_target_adapter` publishes to `/arm_target_pose`.
- `pose_goal_planner` accepts the request and forwards it to MoveIt group `xarm7`.
- MoveIt fails in OMPL with `Unable to sample any valid states for goal tree`, so the rejected object is the converted `link_eef` target, not the action request.
- Correction against the paper: the base fallback should not project the unreachable final target directly. It should project the farthest node on the RRT branch toward that target that represents the closest/farthest reachable EEF pose before the unreachable segment.
- Current ROS 2 port limitation: `fkie_nbv_planner_node.cpp` does not yet expand or retain an RRT tree/branch. It samples independent candidates and returns one `goals[0]`. Reproducing this paper behavior requires adding a branch representation before implementing the final fallback policy.

## Files

- Modify: `mobile_manipulator_fkie_nbv/src/nbv_arm_target_adapter.cpp`
- Modify: `mobile_manipulator_fkie_nbv/src/fkie_nbv_planner_node.cpp`
- Modify: `mobile_manipulator_fkie_msgs/action/NbvPlanner.action`
- Modify: `scripts/execute_fkie_nbv_arm_target.sh`
- Modify: `isaac_sim/tests/test_fkie_reproduction_contract.py`
- Modify: `docs/references/fkie_reproduction_runtime.md`
- Build/test: `mobile_manipulator_fkie_nbv`

## Task 0: Add Branch Data to the Planner Result

- [ ] Extend `mobile_manipulator_fkie_msgs/action/NbvPlanner.action` result with a branch/path field that can preserve EEF or camera poses leading to the chosen target. Use a simple field first:

```text
geometry_msgs/PoseStamped[] branch_goals
```

- [ ] In the current simplified planner, fill `branch_goals` with a deterministic straight-line interpolation from the current camera pose to the best NBV candidate. This is not yet full RRT, but it creates the same downstream contract: an ordered branch toward a target.
- [ ] Later, when real RRT expansion is implemented, replace the interpolation source with actual parent-chain extraction from the RRT tree.
- [ ] Contract-test that `branch_goals` exists in the action definition and that `fkie_nbv_planner_node.cpp` populates it before succeeding.

## Task 1: Add Arm/Base Branch Execution Contract

- [ ] Add a contract test that checks `nbv_arm_target_adapter.cpp` declares and uses these arm-reachability parameters:
  - `base_frame`, default `base_link`
  - `max_eef_step`, default conservative value around `0.25`
  - `keep_current_orientation`, default `true`
  - `min_base_z` and `max_base_z` for diagnostics/rejection
- [ ] Add a contract test that checks the adapter can talk to Nav2:
  - includes `nav2_msgs/action/navigate_to_pose.hpp`
  - creates a `rclcpp_action::Client<nav2_msgs::action::NavigateToPose>`
  - has a `navigate_action_name` parameter defaulting to `/navigate_to_pose`
  - has `base_goal_z`, `base_goal_standoff`, and `base_goal_timeout_s` parameters
- [ ] Add test expectations for:
  - current `map -> link_eef` lookup
  - step limiting before publish
  - target logging with current, desired, and published pose
  - iterating over `branch_goals`
  - selecting the farthest arm-attemptable branch node
  - Nav2 fallback using the ground projection of the selected branch node, not the final unreachable target
- [ ] Run:

```bash
python3 -m unittest isaac_sim.tests.test_fkie_reproduction_contract -v
```

Expected before implementation: fail on the new arm/base branch execution expectations.

## Task 2: Implement Arm-First Branch Reachability Decision

- [ ] In `nbv_arm_target_adapter.cpp`, read `result->branch_goals` when available. If it is empty, fall back to the existing `result->goals` behavior for compatibility.
- [ ] Convert each branch camera pose to a desired `link_eef` pose.
- [ ] Lookup the current `map -> link_eef` transform.
- [ ] For each branch EEF pose, transform it into `base_link`.
- [ ] Treat a branch node as arm-attemptable only when:
  - base-frame z is inside `[min_base_z, max_base_z]`
  - target distance from the current EEF is inside the configured direct arm reach/step threshold
- [ ] Select the farthest arm-attemptable node along the branch.
- [ ] If an arm-attemptable node exists, optionally preserve current orientation and publish that branch node to `/arm_target_pose`.
- [ ] Log enough data to debug one run:

```text
NBV branch nodes: count=...
Branch node i camera target: frame=map x=... y=... z=...
Branch node i link_eef target: frame=map x=... y=... z=... arm_attemptable=true/false
Selected farthest arm-attemptable branch node: index=...
Published branch link_eef target: frame=map x=... y=... z=...
```

## Task 3: Implement Paper-Style Branch Base Fallback

- [ ] Add a Nav2 action client using `nav2_msgs/action/NavigateToPose`.
- [ ] When the final target is not arm-attemptable, create a base goal from the selected fallback branch node, not from the final unreachable target:
  - `goal.pose.position.x = fallback_branch_eef_map_x - base_goal_standoff * cos(yaw)`
  - `goal.pose.position.y = fallback_branch_eef_map_y - base_goal_standoff * sin(yaw)`
  - `goal.pose.position.z = base_goal_z`
  - `goal.pose.orientation` yaw comes from the fallback branch node orientation, so the platform faces along the branch direction.
- [ ] Define fallback branch node selection explicitly:
  - Preferred: farthest branch node that is still reachable by the manipulator but is not the final target.
  - If no reachable branch node exists: use the first branch node after the current pose as a conservative base target, and log this degraded behavior.
- [ ] Keep `base_goal_standoff` configurable. A default of `0.0` follows the ground projection behavior. Increase it later if the base should stop behind the projected branch node instead of directly underneath it.
- [ ] Wait for the Nav2 result and log accepted/succeeded/failed.
- [ ] Do not publish `/arm_target_pose` in the same run after sending the base goal. The next NBV cycle should re-evaluate from the new base pose.

## Task 4: Expose Runtime Parameters in the Script

- [ ] Update `scripts/execute_fkie_nbv_arm_target.sh` to pass environment-overridable parameters:
  - `FKIE_EEF_MAX_STEP`
  - `FKIE_KEEP_CURRENT_ORIENTATION`
  - `FKIE_BASE_FRAME`
  - `FKIE_MIN_BASE_Z`
  - `FKIE_MAX_BASE_Z`
  - `FKIE_NAVIGATE_ACTION_NAME`
  - `FKIE_BASE_GOAL_Z`
  - `FKIE_BASE_GOAL_STANDOFF`
  - `FKIE_BASE_GOAL_TIMEOUT_S`
- [ ] Keep defaults conservative so the first test is a small arm move.
- [ ] Run:

```bash
bash -n scripts/execute_fkie_nbv_arm_target.sh
```

Expected: no output.

## Task 5: Rebuild and Run Static Verification

- [ ] Build only the FKIE planner package with system Python:

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_EXE -u CONDA_PYTHON_EXE -u CONDA_PROMPT_MODIFIER -u CONDA_SHLVL PATH=/usr/bin:/bin:/usr/sbin:/sbin bash -lc 'source /opt/ros/humble/setup.bash; colcon build --packages-select mobile_manipulator_fkie_nbv --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
```

- [ ] Run:

```bash
python3 -m unittest isaac_sim.tests.test_fkie_reproduction_contract -v
bash -n scripts/execute_fkie_nbv_arm_target.sh
python3 -m py_compile isaac_sim/tests/test_fkie_reproduction_contract.py mobile_manipulator_fkie_nbv/launch/fkie_nbv_planner.launch.py
```

Expected: all pass.

## Task 6: Live Test

- [ ] With Isaac, MoveIt, wrist OctoMap, Nav2, and `/nbv_rrt` running, run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/execute_fkie_nbv_arm_target.sh
```

- [ ] Expected first success criterion: MoveIt plans and executes a small step toward the NBV target.
- [ ] Expected fallback criterion: if the final target is outside arm reach, the adapter sends a `NavigateToPose` goal generated from the ground projection of the farthest usable branch EEF node.
- [ ] If MoveIt still fails after the adapter judged the target arm-attemptable, use the new logs to decide whether the blocker is collision, z workspace, orientation, or target distance.

## Next Decision After Success

After one arm-first/base-fallback cycle works, the next feature is replacing the straight-line branch placeholder with actual RRT expansion and parent-chain extraction, then adding a loop runner: request NBV, execute arm or base action, wait for the updated OctoMap, then request the next NBV until the planner returns `complete_exploration`.
