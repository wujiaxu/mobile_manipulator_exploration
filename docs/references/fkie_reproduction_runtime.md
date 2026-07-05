# FKIE Reproduction Runtime Checks

Run this after the Isaac controlled-rooms simulation, navigation/mapping stack,
wrist OctoMap pipeline, and MoveIt are already running:

```bash
./scripts/check_fkie_inputs.sh
```

The script checks the inputs needed before the FKIE NBV planner can be tested:

```text
/octomap_full
/realsense/depth/points2
/camera_pose
/mobile_manipulator_mbf/global_costmap/footprint
/octomap_occupied_points
/apply_planning_scene
```

Expected behavior:

- `/octomap_full` publishes `octomap_msgs/msg/Octomap` and is the
  probabilistic map for information gain.
- `/realsense/depth/points2` publishes the wrist depth cloud compatibility
  alias.
- `/camera_pose` publishes the wrist camera pose in `map`.
- `/mobile_manipulator_mbf/global_costmap/footprint` publishes the passive
  FKIE-style base footprint in `map`.
- `/octomap_occupied_points` publishes occupied voxel centers for MoveIt
  collision projection.
- `/apply_planning_scene` exists when MoveIt is running.

Use `FKIE_CHECK_TIMEOUT_SECONDS=10 ./scripts/check_fkie_inputs.sh` if the
simulation is slow to produce the first messages.

## Send One NBV Request

After the FKIE planner node is running, send a fixed rectangular boundary:

```bash
./scripts/send_fkie_nbv_goal.sh
```

The wrapper runs the C++ client:

```bash
ros2 run mobile_manipulator_fkie_nbv send_fkie_nbv_goal_client
```

The C++ client is used instead of `ros2 action send_goal` YAML or the Python
client because it constructs the custom action goal with generated C++ message
classes, matching the C++ planner server.

The ROS 2 action goal intentionally uses flat boundary fields:

```text
boundary_x: float64[]
boundary_y: float64[]
boundary_min_z: float64
boundary_max_z: float64
```

This avoids the nested FKIE ROS 1 shape
`BoundaryPolygon -> PolygonStamped -> Polygon -> Point32[]`, which repeatedly
arrived empty through the ROS 2 action transport on this host.

Default action and boundary:

```text
/nbv_rrt
FKIE_BOUNDARY_MIN_X=-3.0
FKIE_BOUNDARY_MAX_X=3.0
FKIE_BOUNDARY_MIN_Y=-3.0
FKIE_BOUNDARY_MAX_Y=3.0
FKIE_BOUNDARY_MIN_Z=0.4
FKIE_BOUNDARY_MAX_Z=1.4
```

For the full controlled-rooms environment, use a larger boundary if the map
origin is centered near the room layout:

```bash
FKIE_BOUNDARY_MIN_X=-6.0 FKIE_BOUNDARY_MAX_X=6.0 \
FKIE_BOUNDARY_MIN_Y=-6.0 FKIE_BOUNDARY_MAX_Y=6.0 \
./scripts/send_fkie_nbv_goal.sh
```

Equivalent direct C++ client command:

```bash
ros2 run mobile_manipulator_fkie_nbv send_fkie_nbv_goal_client --ros-args \
  -p min_x:=-6.0 -p max_x:=6.0 \
  -p min_y:=-6.0 -p max_y:=6.0 \
  -p min_z:=0.4 -p max_z:=1.4
```

Expected output:

- `complete_exploration: false`
- at least one pose under `goals`
- `request_base_pose: false`

If `complete_exploration` is true, first check that `/octomap_full`,
`/camera_pose`, and `/mobile_manipulator_mbf/global_costmap/footprint` are
publishing, then lower `min_candidate_gain` in
`mobile_manipulator_fkie_nbv/config/fkie_nbv_planner.yaml` for debugging.

## Publish NBV To MoveIt

After the FKIE planner, MoveIt, and `pose_goal_planner` are running, publish
one NBV result to the existing MoveIt pose-control topic:

```bash
./scripts/execute_fkie_nbv_arm_target.sh
```

This runs:

```bash
ros2 run mobile_manipulator_fkie_nbv nbv_arm_target_adapter
```

Behavior:

- sends one NBV request to `/nbv_rrt`
- takes the first returned camera target pose
- looks up `link_eef -> wrist_camera_color_optical_frame`
- converts the camera target into a `link_eef` target pose
- publishes the target to `/arm_target_pose`

The existing `mobile_manipulator_moveit_bridge/pose_goal_planner` then plans and
executes the arm motion.
