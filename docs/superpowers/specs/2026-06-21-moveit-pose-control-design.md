# MoveIt Pose Control Design

## Goal

Add collision-aware end-effector pose control for the unified mobile manipulator in Isaac Sim. Operators can provide a target through RViz's MotionPlanning panel or publish a `geometry_msgs/msg/PoseStamped` message. MoveIt plans the xArm7 motion and executes it through the existing Isaac Sim arm command topic.

## Scope

This milestone includes:

- MoveIt 2 configuration for the seven-joint xArm7 chain in the unified robot.
- RViz MotionPlanning support.
- A `/arm_target_pose` pose-command interface.
- Execution of MoveIt joint trajectories in Isaac Sim.
- Cancellation, validation, and measurable final-pose verification.

MoveIt Servo will be the following milestone and will reuse this configuration. Nav2, coordinated base-and-arm planning, gripper control, and wrist-camera calibration are outside this milestone.

## Packages

### `mobile_manipulator_moveit_config`

An `ament_cmake` MoveIt configuration package containing:

- `config/mobile_manipulator.srdf`: `xarm7` chain group from `link_base` to tip link `link_eef`; no separate gripper group is declared.
- `config/kinematics.yaml`: KDL numerical inverse kinematics for the `xarm7` group.
- `config/joint_limits.yaml`: limits copied from the unified description for `joint1` through `joint7`.
- `config/ompl_planning.yaml`: OMPL planning configuration.
- `config/moveit_controllers.yaml`: maps the `xarm7` joints to `/arm_controller/follow_joint_trajectory` using MoveIt's simple controller manager.
- `config/moveit.rviz`: RViz MotionPlanning configuration.
- Launch files for `move_group`, RViz, `robot_state_publisher`, and the execution bridge.

The planning group controls only the arm. The mobile base remains part of the robot model and collision scene but is not a variable in arm plans.

### `mobile_manipulator_moveit_bridge`

An `ament_cmake` C++ package containing two nodes.

`isaac_trajectory_bridge` provides a `control_msgs/action/FollowJointTrajectory` server at `/arm_controller/follow_joint_trajectory`. It validates that each goal contains exactly the supported arm joints, interpolates the trajectory against ROS time, and publishes `sensor_msgs/msg/JointState` position commands on `/arm_joint_commands`. It uses `/joint_states` for initial state and terminal error checks.

`pose_goal_planner` subscribes to `/arm_target_pose` as `geometry_msgs/msg/PoseStamped`. It transforms the target into the MoveIt planning frame, sets `link_eef` as the pose target, plans for the `xarm7` group, and executes successful plans through MoveIt's configured trajectory controller.

## Data Flow

RViz execution follows:

```text
RViz MotionPlanning goal
  -> move_group
  -> /arm_controller/follow_joint_trajectory
  -> isaac_trajectory_bridge
  -> /arm_joint_commands
  -> Isaac Sim
  -> /joint_states
```

Topic execution follows:

```text
/arm_target_pose (PoseStamped)
  -> pose_goal_planner
  -> move_group planning and execution
  -> /arm_controller/follow_joint_trajectory
  -> isaac_trajectory_bridge
  -> /arm_joint_commands
  -> Isaac Sim
```

`robot_state_publisher` consumes Isaac's `/joint_states` and remains the sole authority for robot TF. Final pose verification reads `base_link` to `link_eef` from TF after execution.

## Frames And Robot Model

- Unified robot root: `base_link`.
- Arm planning group: `xarm7`.
- Arm joints: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`, `joint7`.
- Arm base link: `link_base`.
- Pose target link: `link_eef`.
- Default pose reference frame: `base_link`.

The recorded transforms `frame_base -> frame_1` and `frame_base -> livox_frame` remain unchanged. No wrist-camera frame is added.

## Trajectory Execution

The bridge accepts trajectories whose joint-name set is exactly the seven arm joints. Incoming order may differ; the bridge reorders every point into `joint1` through `joint7` order before publishing.

For each trajectory segment, the bridge linearly interpolates joint position using `time_from_start` and publishes at 100 Hz. It rejects goals with missing positions, duplicate or unknown joints, non-increasing timestamps, non-finite values, or values outside URDF joint limits.

The bridge reports:

- `SUCCESSFUL` after the final command is published and `/joint_states` reaches the configured position tolerance before timeout.
- `INVALID_JOINTS` or `INVALID_GOAL` for malformed input.
- `PATH_TOLERANCE_VIOLATED` if feedback departs from configured path tolerance.
- `GOAL_TOLERANCE_VIOLATED` if final feedback does not converge before timeout.

Cancellation stops interpolation immediately, publishes the most recent measured arm positions as a hold command, and returns a canceled action result.

## Pose-Goal Behavior

The pose node accepts one target at a time. A new target replaces any target that has not started execution; a target received during execution is rejected with an error log so execution behavior remains deterministic.

The node rejects:

- Empty or unknown frame IDs.
- Non-finite positions or orientations.
- Zero-length quaternions.
- Targets that cannot be transformed into `base_link`.
- Planning failures or trajectories rejected by the controller.

Quaternion inputs are normalized before planning. Planning success and execution success are logged separately.

## Dependencies

The host requires ROS 2 Humble MoveIt packages, including `moveit`, `moveit_ros_move_group`, `moveit_ros_planning_interface`, `moveit_simple_controller_manager`, and `moveit_kinematics`. The host currently lacks MoveIt 2 and Xacro, so dependencies must be installed before building or running this milestone.

The workspace continues to force `/usr/bin/python3` for CMake because the Anaconda Python environment lacks ROS build dependencies.

## Testing

### Static contracts

- SRDF contains the `xarm7` chain from `link_base` to `link_eef` and does not declare a nonexistent gripper group.
- Controller YAML contains all seven joints in deterministic order.
- Joint-limit YAML matches the unified description.
- Launch descriptions load with the expected robot description and controller action.

### Bridge unit tests

- Joint-name reordering.
- Rejection of missing, duplicate, unknown, non-finite, and out-of-limit values.
- Rejection of non-increasing trajectory timestamps.
- Interpolation at segment boundaries and intermediate timestamps.
- Cancellation produces a measured-position hold command.

### Live integration

1. Start Isaac Sim, robot state publishing, MoveIt, the bridge, and RViz.
2. Plan and execute a small reachable `link_eef` pose through RViz.
3. Publish the same target to `/arm_target_pose` and execute it.
4. Confirm the action reports success and all seven joints converge within tolerance.
5. Confirm the final `base_link -> link_eef` TF position and orientation match the target within configured tolerances.
6. Send an unreachable pose and verify planning fails without commanding Isaac Sim.

## Success Criteria

- Both RViz and `/arm_target_pose` produce collision-aware MoveIt plans.
- Successful plans execute on the Isaac Sim arm through the existing `/arm_joint_commands` interface.
- `/joint_states` reflects all seven commanded joints and action feedback reaches the terminal tolerance.
- Final TF agrees with the requested reachable pose.
- Invalid or unreachable targets do not move the arm.
- Existing base control, calibrated transforms, support collision, and source-model integrity regressions continue to pass.
