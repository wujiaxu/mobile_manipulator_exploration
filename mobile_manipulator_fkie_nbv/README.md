# Mobile Manipulator FKIE NBV Planner

This package is the ROS 2 reproduction layer for the FKIE next-best-view planner workflow. It expects the Isaac Sim mobile manipulator stack to already provide:

- SLAM/Nav2 frames, including `map`, `odom`, and `base_link`
- Wrist depth OctoMap on `/octomap_full`
- Wrist camera pose on `/camera_pose`
- FKIE-style mobile footprint on `/mobile_manipulator_mbf/global_costmap/footprint`
- MoveIt arm target input on `/arm_target_pose`
- Nav2 `NavigateToPose` action on `/navigate_to_pose`

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_manipulator_fkie_msgs mobile_manipulator_fkie_nbv
source install/setup.bash
```

If a conda environment is active and ROS Python tools fail, run `conda deactivate` first.

## Start Simulation, Mapping, Nav2, MoveIt, and Wrist OctoMap

Recommended controlled-room test environment:

```bash
./start_controlled_rooms_mobile_manipulator.sh --use-wrist-octomap --use-moveit-rviz
```

This starts:

- Isaac Sim controlled rooms scene
- mobile manipulator USD with LiDAR, wrist RGBD camera, and ROS bridge
- SLAM Toolbox, Nav2, and RViz2
- MoveIt and the Isaac trajectory bridge
- wrist depth OctoMap in `map`

Use `Ctrl+C` in this terminal to stop the complete stack.

For a lighter navigation-only factory stack:

```bash
./start_navigation_test.sh --use-wrist-octomap
```

## Start the FKIE Planner

Open a new terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_manipulator_fkie_nbv fkie_nbv_planner.launch.py use_sim_time:=true
```

The planner starts an action server named `/nbv_rrt`.

## Check Required Runtime Inputs

Open another terminal after Isaac, mapping, wrist OctoMap, MoveIt, and the planner are running:

```bash
./scripts/check_fkie_inputs.sh
```

Expected final line:

```text
All FKIE input checks completed.
```

If this fails, fix the missing topic/service before testing FKIE goals.

## Send One FKIE NBV Goal Only

This sends a fixed boundary request to `/nbv_rrt` and prints the planner result. It does not move the robot.

```bash
./scripts/send_fkie_nbv_goal.sh
```

Useful boundary overrides:

```bash
FKIE_BOUNDARY_MIN_X=-3.0 \
FKIE_BOUNDARY_MAX_X=3.0 \
FKIE_BOUNDARY_MIN_Y=-3.0 \
FKIE_BOUNDARY_MAX_Y=3.0 \
FKIE_BOUNDARY_MIN_Z=0.4 \
FKIE_BOUNDARY_MAX_Z=1.4 \
./scripts/send_fkie_nbv_goal.sh
```

## Run FKIE and Execute the Result

This requests `/nbv_rrt`, then dispatches the result:

- If the planner returns arm goals, it publishes `link_eef` targets to `/arm_target_pose`.
- If the planner requests a base fallback, it sends a Nav2 goal to `/navigate_to_pose`.
- If exploration is complete, it exits without commanding motion.

```bash
./scripts/execute_fkie_nbv_arm_target.sh
```

Useful execution overrides:

```bash
FKIE_NAV2_ACTION_NAME=/navigate_to_pose \
FKIE_ARM_TARGET_TOPIC=/arm_target_pose \
FKIE_ARM_GOAL_PUBLISH_DELAY_S=0.5 \
./scripts/execute_fkie_nbv_arm_target.sh
```

## Typical Terminal Layout

Terminal 1:

```bash
./start_controlled_rooms_mobile_manipulator.sh --use-wrist-octomap --use-moveit-rviz
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_manipulator_fkie_nbv fkie_nbv_planner.launch.py use_sim_time:=true
```

Terminal 3:

```bash
./scripts/check_fkie_inputs.sh
./scripts/send_fkie_nbv_goal.sh
./scripts/execute_fkie_nbv_arm_target.sh
```

## Important Topics and Actions

Planner inputs:

- `/octomap_full`: probabilistic wrist depth OctoMap in `map`
- `/camera_pose`: wrist camera pose in `map`
- `/mobile_manipulator_mbf/global_costmap/footprint`: mobile base footprint in `map`

Planner action:

- `/nbv_rrt`: `mobile_manipulator_fkie_msgs/action/NbvPlanner`

Execution outputs:

- `/arm_target_pose`: `geometry_msgs/msg/PoseStamped` target for the MoveIt pose-goal adapter
- `/navigate_to_pose`: Nav2 base fallback action

## Notes

- The planner assumes `use_sim_time:=true` for Isaac Sim runtime.
- The default boundary is `x=[-3, 3]`, `y=[-3, 3]`, `z=[0.4, 1.4]` in `map`.
- The execution adapter is intentionally thin. It executes planner output; it does not choose new goals or modify the FKIE algorithm.
- For live debugging, watch the planner terminal for RRT size, branch result, `request_base_pose`, and `complete_exploration`.
