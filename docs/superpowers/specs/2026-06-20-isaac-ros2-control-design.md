# Isaac ROS2 Control Design

## Goal

Complete Isaac Sim integration by making the unified mobile manipulator controllable through standard ROS2 topics, using the shortest path toward later Nav2 and MoveIt 2 integration.

## Scope

This milestone adds:

- Differential-drive velocity control for the mobile base.
- Position command control for the seven manipulator joints.
- Automated USD contract checks and live motion verification.

It does not add odometry, Nav2, MoveIt 2, `ros2_control`, sensors, or a command-timeout watchdog.

## Architecture

The ROS-enabled USD owns the control bridge in its existing `/ActionGraph`. This keeps the imported base USD reusable while making the ROS overlay directly runnable from `run_mobile_manipulator.py`.

The existing publishers remain authoritative:

- Isaac Sim publishes `/clock` and `/joint_states`.
- `robot_state_publisher` consumes `/joint_states` and publishes `/tf` and `/tf_static`.

Two subscriber branches are added to the Isaac action graph.

## Mobile Base Control

Topic contract:

```text
Topic: /cmd_vel
Type: geometry_msgs/msg/Twist
Used fields: linear.x, angular.z
```

Data flow:

```text
ROS2SubscribeTwist
  -> BreakVector3 for linear velocity
  -> BreakVector3 for angular velocity
  -> DifferentialController
  -> IsaacArticulationController
  -> left_wheel, right_wheel
```

Only the two main drive-wheel joints are commanded. The four caster assemblies and their wheel joints remain passive.

The wheel separation starts from the URDF joint positions: `0.34 m`. The effective drive-wheel radius is verified through live straight-line motion rather than assumed solely from mesh comments. The selected radius and its evidence are recorded in `CODEX_TASKS.md`.

The first version has no command-timeout watchdog. Every test and operator procedure must send a zero `Twist` after motion, including failure cleanup.

## Manipulator Control

Topic contract:

```text
Topic: /arm_joint_commands
Type: sensor_msgs/msg/JointState
Commanded joints: joint1, joint2, joint3, joint4, joint5, joint6, joint7
Command mode: position
Units: radians
```

Data flow:

```text
ROS2SubscribeJointState
  -> jointNames
  -> positionCommand
  -> IsaacArticulationController
  -> xArm7 articulation joints
```

Arm commands must include exactly seven matching names and seven finite position values. Test poses remain within the URDF joint limits. Velocity and effort arrays are omitted for this milestone so position control is unambiguous.

## Generated Assets

`isaac_sim/scripts/import_mobile_manipulator.py` remains the source of the ROS overlay graph. Re-running it regenerates:

```text
isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd
```

The original mobile and manipulator URDF files remain unchanged.

## Error Handling

The import script fails if the expected articulation root or required control nodes cannot be created. Contract tests fail if topic names, control connections, target articulation, or commanded joint sets differ from this design.

Live verification always publishes zero `/cmd_vel` during cleanup. If the simulator or ROS command process exits unexpectedly, the operator stops the Isaac timeline before restarting because no watchdog exists yet.

## Verification

Static checks:

- The ROS USD contains both subscriber nodes and both articulation controllers.
- `/cmd_vel` connects through the differential controller to `left_wheel` and `right_wheel`.
- `/arm_joint_commands` connects position commands to the articulation controller.
- Existing clock and joint-state publishers remain present.

Live checks:

1. Publish positive `linear.x`; confirm forward base displacement and drive-wheel motion.
2. Publish zero `Twist`; confirm the base stops.
3. Publish positive `angular.z`; confirm in-place rotation direction.
4. Publish zero `Twist`; confirm the base stops again.
5. Publish a conservative seven-joint arm pose; confirm `/joint_states` converges toward it.
6. Confirm `/clock`, `/joint_states`, `/tf`, and `/tf_static` remain available throughout.

## Acceptance Criteria

- The robot responds to `/cmd_vel` in Isaac Sim.
- The arm responds to `/arm_joint_commands` in Isaac Sim.
- Base direction and scale are measured and documented.
- Arm feedback matches the commanded pose within a stated tolerance.
- All static USD contract tests pass.
- The simulator is left stopped with no nonzero velocity command active.
