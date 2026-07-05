# FKIE RRT ROS 2 Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current simplified FKIE candidate sampler with a ROS 2 port of the original FKIE RRT-based NBV planner.

**Architecture:** Keep the existing ROS 2 action, topic mapping, Isaac inputs, and execution adapters. Refactor the planner internals to match the original ROS 1 implementation: RRT nodes, KD-tree nearest-neighbor search, OctoMap collision checks, free-space gain cubature, measurement/visited sparse grids, cached frontiers, best-branch extraction, and original result semantics.

**Tech Stack:** ROS 2 Humble, `rclcpp`, `rclcpp_action`, `tf2_ros`, `octomap`, `octomap_msgs`, `Eigen`, bundled `nanoflann.hpp`, existing `mobile_manipulator_fkie_msgs`.

---

## Guiding Rule

Do not add synthetic interpolation or behavior that moves away from the FKIE implementation. If RRT behavior is needed, port it from `third_party/fkie-nbv-planner` and adapt only the ROS API layer.

## Source References

- Algorithm summary: `docs/references/fkie_original_algorithm_summary.md`
- Original planner: `third_party/fkie-nbv-planner/src/NBVPlanner.cpp`
- Original node model: `third_party/fkie-nbv-planner/include/fkie_nbv_planner/RRTNode.h`
- Original parameters: `third_party/fkie-nbv-planner/include/fkie_nbv_planner/NBVParameters.hpp`
- Current ROS 2 simplified planner: `mobile_manipulator_fkie_nbv/src/fkie_nbv_planner_node.cpp`

## Task 1: Contract Tests for Real RRT Port

**Files:**

- Modify: `isaac_sim/tests/test_fkie_reproduction_contract.py`

- [x] Add contract expectations that `mobile_manipulator_fkie_nbv` contains:
  - `include/mobile_manipulator_fkie_nbv/rrt_node.hpp`
  - `include/mobile_manipulator_fkie_nbv/tree_nanoflann_adapter.hpp`
  - `include/mobile_manipulator_fkie_nbv/nbv_parameters.hpp`
  - `include/mobile_manipulator_fkie_nbv/nbv_utils.hpp`
- [x] Update the planner-node contract so it expects original-algorithm functions:
  - `expand_rrt`
  - `initialize_root`
  - `find_closest_neighbor`
  - `find_new_rrt_node`
  - `collision_line`
  - `gain_cubature`
  - `update_gain`
  - `compute_yaw`
  - `get_best_node_branch`
  - `extract_nbv_poses`
  - `cached_nodes_`
  - `best_node_`
- [x] Remove or relax expectations that force the current simplified functions:
  - `compute_best_view_candidate`
  - `score_candidate`

Run:

```bash
python3 -m unittest isaac_sim.tests.test_fkie_reproduction_contract -v
```

Observed before implementation: failed on missing RRT port files/functions and missing FKIE parameter names.

## Task 2: Port Parameters and Utility Functions

**Files:**

- Create: `mobile_manipulator_fkie_nbv/include/mobile_manipulator_fkie_nbv/nbv_parameters.hpp`
- Create: `mobile_manipulator_fkie_nbv/include/mobile_manipulator_fkie_nbv/nbv_utils.hpp`
- Modify: `mobile_manipulator_fkie_nbv/config/fkie_nbv_planner.yaml`

- [x] Port the original parameter set as ROS 2 node parameters, preserving names where practical:
  - `num_frontiers`
  - `frontiers_N_max_tries`
  - `max_num_cached_points_to_keep`
  - `min_gain_frontier`
  - `free_space_frontier`
  - `arm_height_min`
  - `arm_height_max`
  - `self_collision_height`
  - `arm_goal_range`
  - `raycast_dr`
  - `raycast_dphi`
  - `raycast_dtheta`
  - `gain_r_min`
  - `gain_r_max`
  - `camera_hfov`
  - `camera_vfov`
  - `visited_cells_grid_size`
  - `min_distance_to_obstacle`
  - `compute_yaw_from_free_space`
  - `compute_yaw_from_measurement`
  - `rrt_tree_length`
  - `rrt_N_max_tries`
  - `rrt_sampling_radius`
  - `rrt_collision_cyl_radius`
  - `rrt_sample_threshold_distance`
  - `rrt_step_size`
  - `sample_in_unknown`
  - `do_rrt_star`
  - utility weights and normalization values
  - `robot_sampling_frame`
  - `world_frame`
- [x] Port `isSampleInPolygon`, `generateRandomSample`, `distancePoseStamped`, `poseToEigenVector3d`, and cylinder collision helper.
- [x] Use deterministic RNG with a configurable seed for repeatable tests; runtime can set seed from wall time later if needed.

## Task 3: Port RRT Node and KD-Tree Adapter

**Files:**

- Create: `mobile_manipulator_fkie_nbv/include/mobile_manipulator_fkie_nbv/rrt_node.hpp`
- Create: `mobile_manipulator_fkie_nbv/src/rrt_node.cpp`
- Create: `mobile_manipulator_fkie_nbv/include/mobile_manipulator_fkie_nbv/tree_nanoflann_adapter.hpp`
- Modify: `mobile_manipulator_fkie_nbv/CMakeLists.txt`

- [x] Port `RRTNode` fields and methods:
  - node id
  - parent weak pointer
  - children shared pointers
  - pose
  - gain components
  - cost fields
  - score
  - cubature best yaw
- [x] Port weighted-sum gain and score behavior.
- [x] Port the nanoflann adapter around `std::vector<std::shared_ptr<RRTNode>>`.
- [x] Build `rrt_node.cpp` into the planner executable.

## Task 4: Refactor Planner Node Around Original State

**Files:**

- Modify: `mobile_manipulator_fkie_nbv/src/fkie_nbv_planner_node.cpp`

- [x] Replace independent grid-candidate state with original-style planner state:
  - `cached_nodes_`
  - `best_node_`
  - `best_frontier_`
  - `best_branch_`
  - `tree_adapter_`
  - `kdtree_`
  - `node_count_`
  - utility normalization state
  - visited grid and measurement grid equivalents
- [x] Preserve existing ROS 2 subscriptions and flat-boundary action goal.
- [x] Convert flat `boundary_x`/`boundary_y` arrays into the internal polygon used by the original algorithm.
- [x] Keep the current `/octomap_full`, `/camera_pose`, and footprint topic names parameterized.

## Task 5: Port RRT Expansion

**Files:**

- Modify: `mobile_manipulator_fkie_nbv/src/fkie_nbv_planner_node.cpp`

- [x] Implement:
  - `reset_tree`
  - `initialize_root`
  - `find_closest_neighbor`
  - `find_new_rrt_node`
  - `is_sample_close_to_rrt_node`
  - `collision_line`
  - `expand_tree_towards_frontiers`
  - `expand_rrt`
- [x] Use the original acceptance conditions:
  - known/unknown policy from `sample_in_unknown`
  - collision-free parent-child cylinder
  - inside boundary and height limits
  - outside robot footprint/self-collision height
  - not too close to existing nodes
- [x] Keep RRT* rewiring optional. It can be ported after base RRT because the original default disables it.

## Task 6: Port Gain, Yaw, and Frontier Cache

**Files:**

- Modify: `mobile_manipulator_fkie_nbv/src/fkie_nbv_planner_node.cpp`

- [x] Implement `gain_cubature` with the original spherical/FOV ray sampling and unknown-volume gain.
- [x] Implement `update_gain` using:
  - free-space gain,
  - measurement estimation grid,
  - visited cell penalty.
- [x] Implement `compute_yaw` using:
  - free-space cubature yaw when configured,
  - measurement-gradient yaw when configured.
- [x] Implement cached-node update, trimming, and frontier extraction.
- [x] Implement `find_high_utility_frontier`.

## Task 7: Port Branch Extraction and Original Result Semantics

**Files:**

- Modify: `mobile_manipulator_fkie_nbv/src/fkie_nbv_planner_node.cpp`

- [x] Implement `get_best_node_branch` by following parent pointers from `best_node_` to root and reversing the list.
- [x] Implement `extract_nbv_poses`:
  - filter branch nodes by `min_gain_`,
  - transform each node into `robot_sampling_frame`,
  - keep nodes within `arm_goal_range`,
  - return them in `result.goals`.
- [x] Preserve original fallback:
  - if best branch exists, set `request_base_pose=false` and fill `goals`;
  - if no branch and no frontiers, set `complete_exploration=true`;
  - if no branch but a best frontier exists, set `request_base_pose=true` and `goal_pose_3d=best_frontier`.
- [x] Do not add `branch_goals` or straight-line interpolation.

## Task 8: Execution Adapter Mapping

**Files:**

- Modify: `mobile_manipulator_fkie_nbv/src/nbv_arm_target_adapter.cpp`
- Modify: `scripts/execute_fkie_nbv_arm_target.sh`

- [x] If `result.goals` is non-empty, send those arm goals to the existing MoveIt path.
- [x] If `result.request_base_pose=true`, convert `goal_pose_3d` to a Nav2 goal by projecting it to the floor and preserving yaw.
- [x] Keep this adapter thin. It should execute planner output, not decide planner algorithm behavior.

## Task 9: Verification

- [ ] Run static contracts:

```bash
python3 -m unittest isaac_sim.tests.test_fkie_reproduction_contract -v
bash -n scripts/check_fkie_inputs.sh
bash -n scripts/send_fkie_nbv_goal.sh
bash -n scripts/execute_fkie_nbv_arm_target.sh
```

- [ ] Build:

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_EXE -u CONDA_PYTHON_EXE -u CONDA_PROMPT_MODIFIER -u CONDA_SHLVL PATH=/usr/bin:/bin:/usr/sbin:/sbin bash -lc 'source /opt/ros/humble/setup.bash; colcon build --packages-select mobile_manipulator_fkie_msgs mobile_manipulator_fkie_nbv --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
```

- [ ] Runtime check:
  - launch Isaac + mapping + wrist OctoMap,
  - launch `mobile_manipulator_fkie_nbv fkie_nbv_planner.launch.py`,
  - send one FKIE goal,
  - verify logs show RRT size, best node, branch size, arm goals count, and base fallback flag.

## Current Implementation Boundary

Tasks 1-8 compile, and static contracts pass. The next step is Task 9 runtime verification in a live Isaac/ROS session: launch mapping, wrist OctoMap, FKIE planner, then run the adapter and verify RRT size, best node/branch behavior, arm-goal publication, and Nav2 fallback when requested.
