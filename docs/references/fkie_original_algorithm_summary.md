# FKIE Original NBV Planner Algorithm Summary

Source inspected locally: `third_party/fkie-nbv-planner` at commit `91091f8a58301a69bbf240bce073059092e6f606`.

## Runtime Inputs

The ROS 1 planner action server is `nbv_rrt`. On each request, `NBVPlanner::executePlan()`:

- Requires an OctoMap and current camera pose before planning.
- Initializes the exploration boundary from the action goal, or from YAML defaults if the goal boundary is empty.
- Loads measurement-estimation values into a sparse 3D grid.
- Publishes the visited grid and boundary visualization.
- Locks the OctoMap while planning so the tree is built against a stable map snapshot.

Original code reference: `third_party/fkie-nbv-planner/src/NBVPlanner.cpp:149`.

## Persistent Frontier Cache

The planner keeps `cached_nodes_` across planning requests. Before each new RRT expansion it:

- Recomputes gain for cached nodes using the current OctoMap and measurement grid.
- Sorts/keeps only the best cached nodes.
- Converts high-gain cached nodes into `curr_frontiers`.
- Selects a `best_frontier_` that is outside the arm reach and closest to the robot sampling frame.

Original code references:

- Cached-node update and trimming: `NBVPlanner.cpp:204`
- Frontier extraction in `executePlan()`: `NBVPlanner.cpp:212`
- Best-frontier selection: `NBVPlanner.cpp:393`

## RRT Node Data

Each `RRTNode` stores:

- Pose.
- Parent pointer and child pointers.
- Free-space gain, measurement gain, visited-cell penalty, total gain.
- Cost to parent, cost to root, and final score.
- Best yaw from free-space cubature.

The current scoring path is weighted-sum scoring:

```text
score = gain - utility_weight_euclidean_cost * cost_to_root
```

with gain computed from configured weights over free space, measurement, and visited-cell terms.

Original code references:

- Node fields: `third_party/fkie-nbv-planner/include/fkie_nbv_planner/RRTNode.h`
- Gain composition: `third_party/fkie-nbv-planner/src/RRTNode.cpp:120`
- Weighted score: `third_party/fkie-nbv-planner/src/RRTNode.cpp:237`

## RRT Expansion

`getNbvBranch()` resets the current tree, calls `expandRRT()`, then extracts poses from the best branch if a valid `best_node_` exists.

`expandRRT()`:

1. Initializes the root from the current camera pose.
2. Samples random points around `robot_sampling_frame` within `rrt_sampling_radius`, respecting `arm_height_min` and `arm_height_max`.
3. Finds the nearest existing RRT node with nanoflann.
4. Extends from nearest node toward the sample by `rrt_step_size`.
5. Rejects invalid samples:
   - unknown/known occupancy depending on `sample_in_unknown`
   - collision along the parent-child cylinder
   - outside the exploration polygon
   - inside robot footprint/self-collision region
   - too close to an existing node
6. Computes gain and yaw for accepted nodes.
7. Adds parent/child links.
8. Optionally does RRT* rewiring.
9. Computes node score and updates `best_node_`.
10. Caches the accepted node for future frontier use.
11. Biases continued expansion toward frontiers when cached frontiers are better than the current best node.

Original code references:

- `getNbvBranch()`: `NBVPlanner.cpp:943`
- RRT root: `NBVPlanner.cpp:1389`
- Main expansion loop: `NBVPlanner.cpp:1005`
- Nearest-neighbor lookup: `NBVPlanner.cpp:1353`
- Step extension: `NBVPlanner.cpp:1379`
- Collision cylinder check: `NBVPlanner.cpp:1464`
- Frontier-biased expansion: `NBVPlanner.cpp:1223`

## Gain and Yaw

`gainCubature()` evaluates information gain by ray-casting over camera FOV:

- It samples yaw/theta, vertical angle/phi, and range/r.
- It stops a ray when it leaves the boundary, enters the robot footprint, or hits occupied OctoMap space.
- Unknown cells add volumetric free-space gain.
- It accumulates gain per yaw and chooses the yaw with best summed FOV gain.

`updateGain()` combines:

- free-space gain from cubature,
- measurement-grid gain,
- visited-cell penalty.

`computeYaw()` uses either:

- free-space cubature yaw, or
- measurement-gradient yaw from neighboring measurement-grid values.

Original code references:

- Gain cubature: `NBVPlanner.cpp:512`
- Gain normalization/composition: `NBVPlanner.cpp:722`
- Yaw computation: `NBVPlanner.cpp:781`

## Branch Extraction and Arm Reach Filtering

The branch is not interpolated. It is extracted from the real RRT tree:

1. Start from `best_node_`.
2. Follow parent pointers back to the root.
3. Reverse the chain so it is ordered from root toward target.

Then `extractNBVPoses()` filters this branch:

- Keeps only nodes with gain above `min_gain_`.
- Transforms each node into `robot_sampling_frame`.
- Keeps only nodes within `arm_goal_range`.
- Returns these reachable branch nodes as `result.goals`.

Original code references:

- Parent-chain extraction: `NBVPlanner.cpp:617`
- Reachable goal filtering: `NBVPlanner.cpp:433`

## Base Fallback

In the inspected code path, if no reachable best branch is produced:

- If no cached frontiers exist, exploration is marked complete.
- If a high-utility frontier exists, `result.request_base_pose = true` and `result.goal_pose_3d = best_frontier_.toPoseStamped()`.

There is also a `generateGoalsToSubFrontier()` function that iterates along `best_branch_`, keeps reachable goals until the branch leaves arm range, and identifies a sub-frontier. In the inspected code, the actual `request_base_pose` assignment inside that function is disabled/commented, so the active action callback uses the cached-frontier fallback path.

Original code references:

- Active fallback in `executePlan()`: `NBVPlanner.cpp:329`
- Sub-frontier helper: `NBVPlanner.cpp:642`

## Current ROS 2 Port Gap

Our current ROS 2 node is not a port of this algorithm yet. It is a simplified independent candidate sampler:

- no `RRTNode`,
- no KD-tree,
- no parent/child tree,
- no cached frontier reuse,
- no branch extraction from parent pointers,
- no `arm_goal_range` filtering inside the planner.

The next implementation should replace the simplified candidate sampler with ROS 2 versions of the original RRT data structures and planning flow.
