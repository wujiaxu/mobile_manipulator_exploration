# MoveIt Pose Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Control the unified xArm7 in Isaac Sim from collision-aware end-effector pose goals supplied through RViz or `/arm_target_pose`.

**Architecture:** A MoveIt configuration package models the arm group and sends `FollowJointTrajectory` goals to a dedicated bridge. The bridge validates and interpolates trajectories into the existing `/arm_joint_commands` JointState topic, while a separate MoveGroupInterface node converts `PoseStamped` requests into planned and executed trajectories.

**Tech Stack:** ROS 2 Humble, MoveIt 2, C++17, `rclcpp_action`, `moveit_ros_planning_interface`, KDL, OMPL, Python `unittest`, GoogleTest, Isaac Sim 4.5.

**Repository note:** This workspace is not a Git repository. Replace commit checkpoints with fresh tests and updates to `progress.md`; do not initialize Git as part of this work.

**Scope boundary:** Do not add Nav2, coordinated mobile-manipulator planning, gripper control, or wrist-camera frames in this plan. MoveIt Servo is dependency-only here and remains the next implementation milestone.

---

## File Map

Create `mobile_manipulator_moveit_config` for declarative robot planning configuration and launch orchestration.

Create `mobile_manipulator_moveit_bridge` for executable behavior. Keep validation/interpolation in `trajectory_sampler.*`, action execution in `isaac_trajectory_bridge.cpp`, and pose planning in `pose_goal_planner.cpp`.

Extend `isaac_sim/tests/test_moveit_config_contract.py` for dependency-free XML/YAML/package contracts.

## Task 1: Install And Verify ROS Dependencies

**Files:**
- Modify: `task_plan.md`
- Modify: `progress.md`

- [x] **Step 1: Confirm the packages are still absent**

Run:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix moveit_ros_move_group
command -v xacro
```

Expected before installation: MoveIt package not found and no Xacro executable.

- [x] **Step 2: Install the required Humble packages**

Run:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-xacro
```

Servo is installed now to avoid a second dependency installation, but no Servo configuration is implemented in this milestone.

- [x] **Step 3: Verify the installed interfaces**

Run:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_ros_planning_interface
ros2 pkg prefix moveit_simple_controller_manager
ros2 pkg prefix moveit_kinematics
command -v xacro
```

Expected: every command prints a valid path.

- [x] **Step 4: Record dependency versions and test result**

Add the installed package versions and commands to `progress.md`. If installation fails, add the exact error to `task_plan.md` before changing approach.

## Task 2: Add Failing MoveIt Configuration Contracts

**Files:**
- Create: `isaac_sim/tests/test_moveit_config_contract.py`

- [x] **Step 1: Write package and semantic contracts**

Create tests using only `unittest`, `xml.etree.ElementTree`, and `yaml.safe_load`. Require:

```python
ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]

def test_srdf_defines_xarm7_chain(self):
    group = self.srdf.find("./group[@name='xarm7']")
    chain = group.find("chain")
    self.assertEqual(chain.attrib["base_link"], "link_base")
    self.assertEqual(chain.attrib["tip_link"], "link_eef")

def test_controller_uses_expected_action_and_joint_order(self):
    controller = self.controllers["moveit_simple_controller_manager"]["arm_controller"]
    self.assertEqual(controller["action_ns"], "follow_joint_trajectory")
    self.assertEqual(controller["type"], "FollowJointTrajectory")
    self.assertEqual(controller["joints"], ARM_JOINTS)

def test_kinematics_uses_kdl(self):
    self.assertEqual(
        self.kinematics["xarm7"]["kinematics_solver"],
        "kdl_kinematics_plugin/KDLKinematicsPlugin",
    )
```

Also assert that the configuration package manifest declares MoveIt and description-package dependencies; joint-limit values match the seven limits in `xarm7.urdf.xacro`; and all configuration files exist. Bridge-specific control message and TF dependencies are added to this contract in Task 4 after the bridge package exists.

- [x] **Step 2: Run the contracts and verify they fail**

Run:

```bash
/usr/bin/python3 -m unittest isaac_sim/tests/test_moveit_config_contract.py -v
```

Expected: FAIL because both new packages and configuration files are absent.

## Task 3: Create The MoveIt Configuration Package

**Files:**
- Create: `mobile_manipulator_moveit_config/CMakeLists.txt`
- Create: `mobile_manipulator_moveit_config/package.xml`
- Create: `mobile_manipulator_moveit_config/config/mobile_manipulator.urdf.xacro`
- Create: `mobile_manipulator_moveit_config/config/mobile_manipulator.srdf`
- Create: `mobile_manipulator_moveit_config/config/kinematics.yaml`
- Create: `mobile_manipulator_moveit_config/config/joint_limits.yaml`
- Create: `mobile_manipulator_moveit_config/config/ompl_planning.yaml`
- Create: `mobile_manipulator_moveit_config/config/moveit_controllers.yaml`
- Create: `mobile_manipulator_moveit_config/config/initial_positions.yaml`
- Create: `mobile_manipulator_moveit_config/config/moveit.rviz`

- [x] **Step 1: Create package metadata**

Use `ament_cmake`. Install `config` and `launch` directories. Declare runtime dependencies on:

```xml
<exec_depend>mobile_manipulator_description</exec_depend>
<exec_depend>mobile_manipulator_moveit_bridge</exec_depend>
<exec_depend>moveit_configs_utils</exec_depend>
<exec_depend>moveit_kinematics</exec_depend>
<exec_depend>moveit_planners_ompl</exec_depend>
<exec_depend>moveit_ros_move_group</exec_depend>
<exec_depend>moveit_ros_visualization</exec_depend>
<exec_depend>moveit_simple_controller_manager</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>xacro</exec_depend>
```

- [x] **Step 2: Add the robot-description wrapper**

Create `config/mobile_manipulator.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="mobile_manipulator" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find mobile_manipulator_description)/urdf/mobile_manipulator.urdf.xacro"/>
</robot>
```

- [x] **Step 3: Add the SRDF**

Define:

```xml
<robot name="mobile_manipulator">
  <group name="xarm7">
    <chain base_link="link_base" tip_link="link_eef"/>
  </group>
  <group_state name="home" group="xarm7">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0"/>
    <joint name="joint3" value="0"/>
    <joint name="joint4" value="0"/>
    <joint name="joint5" value="0"/>
    <joint name="joint6" value="0"/>
    <joint name="joint7" value="0"/>
  </group_state>
</robot>
```

Do not declare a gripper group or wrist-camera link. Disable collision checking only for these physically adjacent or fixed-connected pairs:

```xml
<disable_collisions link1="link_base" link2="link1" reason="Adjacent"/>
<disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
<disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
<disable_collisions link1="link3" link2="link4" reason="Adjacent"/>
<disable_collisions link1="link4" link2="link5" reason="Adjacent"/>
<disable_collisions link1="link5" link2="link6" reason="Adjacent"/>
<disable_collisions link1="link6" link2="link7" reason="Adjacent"/>
<disable_collisions link1="link7" link2="link_eef" reason="Adjacent"/>
<disable_collisions link1="arm_support_link" link2="link_base" reason="Never"/>
```

Do not disable non-adjacent arm collisions without a reproducible collision-sampling result.

- [x] **Step 4: Add planning and controller YAML**

Use KDL with `kinematics_solver_timeout: 0.05` and `kinematics_solver_search_resolution: 0.005`. Configure OMPL's default planner as `RRTConnectkConfigDefault`.

Configure the controller exactly as:

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7
```

Copy position, velocity, and acceleration constraints explicitly into `joint_limits.yaml`; use `has_position_limits: true` for all seven revolute joints and preserve the URDF lower/upper values.

- [x] **Step 5: Add initial positions and RViz configuration**

Set every arm joint to `0.0` in `initial_positions.yaml`. Configure RViz with Fixed Frame `base_link`, RobotModel, TF, and MotionPlanning displays; select group `xarm7` and end-effector link `link_eef`.

- [x] **Step 6: Run static contracts**

Run:

```bash
/usr/bin/python3 -m unittest isaac_sim/tests/test_moveit_config_contract.py -v
```

Expected: every configuration-only test from Task 2 passes. Bridge and launch contracts are not added to the test module until Tasks 5 through 7.

## Task 4: Implement And Test Trajectory Validation/Interpolation

**Files:**
- Create: `mobile_manipulator_moveit_bridge/CMakeLists.txt`
- Create: `mobile_manipulator_moveit_bridge/package.xml`
- Create: `mobile_manipulator_moveit_bridge/include/mobile_manipulator_moveit_bridge/trajectory_sampler.hpp`
- Create: `mobile_manipulator_moveit_bridge/src/trajectory_sampler.cpp`
- Create: `mobile_manipulator_moveit_bridge/test/test_trajectory_sampler.cpp`

- [x] **Step 1: Create bridge package metadata**

Use `ament_cmake`, C++17, and dependencies:

```text
control_msgs
geometry_msgs
moveit_ros_planning_interface
rclcpp
rclcpp_action
sensor_msgs
tf2_geometry_msgs
tf2_ros
trajectory_msgs
```

Enable `ament_cmake_gtest` under `BUILD_TESTING`.

- [x] **Step 2: Write failing sampler tests**

Define `TrajectorySampler` with deterministic arm order and `ValidationResult validate(const trajectory_msgs::msg::JointTrajectory&)`. Tests must cover:

```cpp
EXPECT_TRUE(sampler.validate(valid_trajectory()).ok);
EXPECT_FALSE(sampler.validate(missing_joint_trajectory()).ok);
EXPECT_FALSE(sampler.validate(duplicate_joint_trajectory()).ok);
EXPECT_FALSE(sampler.validate(unknown_joint_trajectory()).ok);
EXPECT_FALSE(sampler.validate(non_finite_trajectory()).ok);
EXPECT_FALSE(sampler.validate(non_increasing_time_trajectory()).ok);
EXPECT_FALSE(sampler.validate(out_of_limit_trajectory()).ok);
```

Add interpolation tests at the first point, midpoint, segment boundary, and final point. Verify input joint order is converted to `joint1` through `joint7`.

- [x] **Step 3: Run tests red**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_manipulator_moveit_bridge \
  --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Expected: build or tests fail because sampler implementation is absent.

- [x] **Step 4: Implement the sampler**

Use constants:

```cpp
inline constexpr std::array<std::string_view, 7> kArmJoints = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
```

Store the seven URDF lower/upper limits in the same order. Validate exact joint set, finite positions, complete point dimensions, and strictly increasing `time_from_start`. Reorder once during goal preparation, then linearly interpolate positions with clamped segment ratio.

- [x] **Step 5: Build and run sampler tests green**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_manipulator_moveit_bridge \
  --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon test --packages-select mobile_manipulator_moveit_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Expected: sampler tests pass with zero failures.

## Task 5: Implement The FollowJointTrajectory Bridge

**Files:**
- Create: `mobile_manipulator_moveit_bridge/src/isaac_trajectory_bridge.cpp`
- Modify: `mobile_manipulator_moveit_bridge/CMakeLists.txt`
- Modify: `isaac_sim/tests/test_moveit_config_contract.py`

- [x] **Step 1: Add failing executable/interface contracts**

Require the package to install an `isaac_trajectory_bridge` executable and source to contain:

```text
/arm_controller/follow_joint_trajectory
/arm_joint_commands
/joint_states
goal_position_tolerance
path_position_tolerance
goal_time_tolerance
```

Run the Python contract and confirm it fails before adding the node.

- [x] **Step 2: Implement action and topic interfaces**

Create a `FollowJointTrajectory` server at `/arm_controller/follow_joint_trajectory`, publisher at `/arm_joint_commands`, and subscriber at `/joint_states`. Declare parameters:

```yaml
publish_rate: 100.0
goal_position_tolerance: 0.02
path_position_tolerance: 0.25
goal_time_tolerance: 2.0
```

Reject malformed goals in `handle_goal`. Keep one active goal; reject concurrent goals.

- [x] **Step 3: Implement execution and cancellation**

Execute accepted goals in a joined worker thread owned by the node. Compute elapsed time from the node's ROS clock and publish interpolated `JointState` positions at 100 Hz. Publish action feedback using measured `/joint_states` positions and error.

On cancellation, publish the latest measured arm positions once as a hold command and return canceled. On completion, wait up to `goal_time_tolerance` for all measured joint errors to fall below `goal_position_tolerance`, then return `SUCCESSFUL` or `GOAL_TOLERANCE_VIOLATED`. Abort with `PATH_TOLERANCE_VIOLATED` when measured error exceeds the path tolerance after the first commanded sample.

- [x] **Step 4: Add an action smoke test**

Launch the bridge without Isaac, publish a complete synthetic `/joint_states` sample, and use `ros2 action info /arm_controller/follow_joint_trajectory`. Expected: one action server with the correct type. Send an invalid goal and verify it is rejected without publishing `/arm_joint_commands`.

- [x] **Step 5: Run bridge and regression tests**

Run the bridge package tests plus existing `test_description_contract.py`, `test_run_contract.py`, and `test_import_contract.py`. Expected: all pass.

## Task 6: Implement The Pose Goal Planner

**Files:**
- Create: `mobile_manipulator_moveit_bridge/src/pose_goal_planner.cpp`
- Modify: `mobile_manipulator_moveit_bridge/CMakeLists.txt`
- Modify: `isaac_sim/tests/test_moveit_config_contract.py`

- [x] **Step 1: Add failing interface contracts**

Require an installed `pose_goal_planner` executable and these literals/interfaces:

```text
/arm_target_pose
xarm7
link_eef
base_link
```

Require dependencies on `geometry_msgs`, `moveit_ros_planning_interface`, `tf2_geometry_msgs`, and `tf2_ros`. Run red before implementation.

- [x] **Step 2: Implement target validation and transformation**

Subscribe to `/arm_target_pose`. Reject empty frames, non-finite values, and quaternions with norm below `1e-9`. Normalize valid quaternions and transform targets to `base_link` with a bounded 0.5-second TF timeout.

- [x] **Step 3: Implement planning and execution**

Construct `MoveGroupInterface` for `xarm7`, set pose reference frame `base_link`, end-effector link `link_eef`, planning time `5.0`, and planning attempts `5`. Use an atomic busy flag; reject new targets during planning or execution. Log planning and execution outcomes separately and clear pose targets after every attempt.

- [ ] **Step 4: Build and validate interfaces**

Run package build/tests and load the node with MoveIt parameters. Publish invalid targets for empty frame, zero quaternion, and unknown frame; verify each is rejected without an action goal.

## Task 7: Add Integrated MoveIt Launch

**Files:**
- Create: `mobile_manipulator_moveit_config/launch/moveit_isaac.launch.py`
- Modify: `isaac_sim/tests/test_moveit_config_contract.py`

- [x] **Step 1: Add failing launch contract**

Require launch arguments `use_sim_time` and `use_rviz`, and nodes for `robot_state_publisher`, `move_group`, `isaac_trajectory_bridge`, `pose_goal_planner`, and optional `rviz2`.

- [x] **Step 2: Build MoveIt parameters**

Use:

```python
moveit_config = (
    MoveItConfigsBuilder(
        "mobile_manipulator",
        package_name="mobile_manipulator_moveit_config",
    )
    .robot_description(file_path="config/mobile_manipulator.urdf.xacro")
    .robot_description_semantic(file_path="config/mobile_manipulator.srdf")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_pipelines(pipelines=["ompl"])
    .to_moveit_configs()
)
```

Pass `use_sim_time` to every node. Run `robot_state_publisher` as the only TF publisher and do not launch a joint-state GUI.

- [x] **Step 3: Add nodes and RViz**

Start `move_group` with `moveit_config.to_dict()`, both bridge executables, and RViz with `config/moveit.rviz` when `use_rviz` is true. Pass robot description, semantic description, kinematics, planning pipeline, and joint limits to the pose node.

- [x] **Step 4: Validate launch description**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch mobile_manipulator_moveit_config \
  moveit_isaac.launch.py --show-args
```

Then load the launch description in a bounded test without Isaac. Expected: nodes initialize; controller action exists; MoveIt waits for `/joint_states` without crashing.

## Task 8: Live Isaac Integration

**Files:**
- Modify: `CODEX_TASKS.md`
- Modify: `progress.md`
- Modify: `task_plan.md`

- [x] **Step 1: Start Isaac Sim cleanly**

Restart the host ROS process state first because prior CLI discovery was wedged. Run Isaac with explicit Fast DDS and the bridge library path:

```bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}
/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_mobile_manipulator.py --pose-report-period 1
```

- [x] **Step 2: Start MoveIt and verify interfaces**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mobile_manipulator_moveit_config moveit_isaac.launch.py
```

Verify `/joint_states`, `/tf`, `/arm_target_pose`, and `/arm_controller/follow_joint_trajectory`.

- [ ] **Step 3: Execute a reachable RViz pose**

Choose a small displacement from the current `link_eef` pose, plan, inspect the collision-free trajectory, and execute. Record final joint errors and `base_link -> link_eef` TF error.

- [x] **Step 4: Execute the same target through the topic**

Publish a normalized `PoseStamped` in `base_link`. Confirm planning succeeds, the action completes, and TF reaches the target within `0.02 m` position and `0.05 rad` orientation error.

- [x] **Step 5: Verify failure and cancellation behavior**

Publish an unreachable target and confirm no `/arm_joint_commands` motion. Execute a longer reachable trajectory, cancel its action, and verify the measured-position hold command stops motion.

- [x] **Step 6: Run final regressions**

Run:

```bash
/usr/bin/python3 -m unittest \
  isaac_sim/tests/test_description_contract.py \
  isaac_sim/tests/test_run_contract.py \
  isaac_sim/tests/test_moveit_config_contract.py -v

source /opt/ros/humble/setup.bash
colcon test --packages-select \
  mobile_manipulator_description \
  mobile_manipulator_moveit_bridge \
  mobile_manipulator_moveit_config \
  --event-handlers console_direct+
colcon test-result --verbose
```

Run `isaac_sim/tests/test_import_contract.py` with Isaac's Python and confirm all generated-USD contracts remain green. Re-hash both original source URDFs and compare with the values recorded in `CODEX_TASKS.md`.

- [x] **Step 7: Update project memory**

Record created packages, exact run commands, tested target poses, tolerances, final errors, and the next MoveIt Servo milestone in `CODEX_TASKS.md`, `progress.md`, and `task_plan.md`.
