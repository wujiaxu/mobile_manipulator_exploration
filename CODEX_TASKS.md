# Codex Development Memory

## Rule for Codex

Before doing any coding task:

1. Read `PROJECT_OVERVIEW.md`
2. Read this file `CODEX_TASKS.md`
3. Summarize current project status
4. Then propose the next concrete action

After finishing any task:

1. Update this file
2. Record what was changed
3. Record remaining issues
4. Record next recommended command

---

# Existing Robot Assets

## Mobile Robot URDF

Package:
mobile_platform_description

Path:

```bash
/home/user/wu_ws/isaac_crowd_navi/isaac_crowd_navi/source/isaac_crowd_navi/assets/tracer_description/urdf/tracer_v1.urdf
```

Root frame:

```text
frame_base
```

---

## Manipulator URDF

Package:
xarm7_description

Path:

```bash
/home/user/wu_ws/isaac_crowd_navi/isaac_crowd_navi/source/isaac_crowd_navi/assets/xarm_isaac/xarm7_clean.urdf
```

Root frame:

```text
frame_1
```

There is a block between the manipulator and the mobile robot, which can be simulated as a premitive shape in isaacsim

# Robot Calibration / Extrinsic Parameters

All transforms below are fixed transforms and should be treated as ground truth when generating the unified mobile manipulator URDF/Xacro.

Coordinate convention:

- Translation unit: meters
- Rotation format: quaternion `x y z w`
- Quaternion is the authoritative rotation format
- RPY is only for readability

---

## 1. Mobile Base → LiDAR

TF source:

```bash
ros2 run tf2_ros tf2_echo frame_base livox_frame
```

Parent frame:

```text
frame_base
```

Child frame:

```text
livox_frame
```

Translation:

```yaml
x: 0.149
y: 0.000
z: 0.444
```

Rotation quaternion:

```yaml
x: 0.000
y: 0.000
z: 0.000
w: 1.000
```

Rotation RPY:

```yaml
roll: 0.000
pitch: 0.000
yaw: 0.000
```

Homogeneous matrix:

```text
1.000  0.000  0.000  0.149
0.000  1.000  0.000  0.000
0.000  0.000  1.000  0.444
0.000  0.000  0.000  1.000
```

---

## 2. Mobile Base → Manipulator Base

TF source:

```bash
ros2 run tf2_ros tf2_echo frame_base frame_1
```

Parent frame:

```text
frame_base
```

Child frame:

```text
frame_1
```

Translation:

```yaml
x: -0.076
y: 0.000
z: 0.519
```

Rotation quaternion:

```yaml
x: 0.000
y: 0.000
z: 0.000
w: 1.000
```

Rotation RPY:

```yaml
roll: 0.000
pitch: 0.000
yaw: 0.000
```

Homogeneous matrix:

```text
1.000  0.000  0.000 -0.076
0.000  1.000  0.000  0.000
0.000  0.000  1.000  0.519
0.000  0.000  0.000  1.000
```

---

## Notes

`tf2_echo` printed:

```text
Invalid frame ID "frame_base" passed to canTransform argument target_frame - frame does not exist
```

but it later returned valid transforms from the bag playback. Therefore, the extracted numerical transforms above should be kept, but Codex should verify the TF tree again during integration.

Possible reason:

- `tf2_echo` started before `/tf` or `/tf_static` was fully available
- bag playback was not yet publishing the required frame
- the transform was only available at a specific timestamp

Verification command after integration:

```bash
ros2 run tf2_ros tf2_echo frame_base livox_frame
ros2 run tf2_ros tf2_echo frame_base frame_1
ros2 run tf2_tools view_frames
```

# 2026-06-19: Unified ROS 2 Description Package

## What Changed

Created the ROS 2 `ament_cmake` package:

```text
mobile_manipulator_description
```

Package contents:

- `urdf/mobile_manipulator.urdf.xacro`: unified robot and calibrated fixed joints
- `urdf/tracer_base.urdf.xacro`: package-local macro adapted from `tracer_v1.urdf`
- `urdf/xarm7.urdf.xacro`: package-local macro adapted from `xarm7_clean.urdf`
- `meshes/tracer/`: referenced Tracer meshes
- `meshes/xarm7/`: referenced xArm7 meshes
- `launch/display.launch.py`: joint state publisher GUI, robot state publisher, and RViz2
- `rviz/display.rviz`: robot model and TF display configuration

The two original robot files were not modified. Their SHA-256 hashes after integration are:

```text
19bf5eec287ceabe679359b5e331cb329e87ce846ba1dec301a4ca7cd3259cbc  tracer_v1.urdf
a4577332d8cd8d3f1610606c48f2cec52ee2ddbe811adc8c977d0550fd860ab2  xarm7_clean.urdf
```

## Root Links And Unified TF Tree

- Mobile model root: `base_link`
- Manipulator source root: `world`
- Manipulator physical root used in the unified model: `link_base`

The source arm's standalone `world -> link_base` fixture was excluded from the package-local adaptation. The calibrated frame aliases absent from the source models were added explicitly:

```text
base_link
└── frame_base                     identity
    ├── frame_1                    xyz=-0.076 0 0.519, rpy=0 0 0
    │   └── link_base              identity
    │       └── link1 ... link7 -> link_eef
    └── livox_frame                xyz=0.149 0 0.444, rpy=0 0 0
```

`check_urdf` confirmed that `base_link` is the single root and the complete mobile and manipulator chains are connected.

Final validation results:

- `colcon build`: 1 package finished, 0 failed
- Expanded URDF: 24 links and 23 joints
- Mesh validation: all 34 active mesh references resolve to package files
- Launch-description generation: passed with the `use_sim_time` argument

## Arm Support Block

Added `arm_support_link` as a fixed child of `frame_base` to provide visible and collision support between the Tracer chassis and xArm7 base.

Support properties:

```yaml
size: [0.300, 0.300, 0.49692302]
center_xyz: [-0.076, 0.000, 0.27053849]
bottom_z: 0.02207698
top_z: 0.519
mass: 10.0
```

The visual and collision elements use the same primitive box. Box inertia is:

```yaml
ixx: 0.2807770732
iyy: 0.2807770732
izz: 0.15
ixy: 0.0
ixz: 0.0
iyz: 0.0
```

The calibrated `frame_base -> frame_1` transform remains unchanged. Validation passed with four support-description assertions, `check_urdf`, and a package build.

## Remaining Issues

- The wrist camera frame was not added because no calibrated `link_eef -> camera_link` transform or optical-frame convention is available.
- The host currently lacks `ros-humble-xacro` and `ros-humble-joint-state-publisher-gui`. They are declared as package dependencies and must be installed before display launch.
- The workspace shell prioritizes Anaconda Python. CMake must use `/usr/bin/python3`, which contains the ROS Python dependencies.
- Source-asset licensing is not documented; the package manifest retains an unknown license marker until ownership is confirmed.
- Live TF was verified on 2026-06-20; both calibrated transforms match exactly. The first missing-frame line is only a publisher-discovery startup race.

## Next Commands

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
source /opt/ros/humble/setup.bash

# Install declared dependencies, including Xacro and the joint-state GUI.
rosdep install --from-paths mobile_manipulator_description --ignore-src -r -y

# Force system Python because Anaconda Python lacks catkin_pkg.
colcon build \
  --packages-select mobile_manipulator_description \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
ros2 launch mobile_manipulator_description display.launch.py
```

In another sourced terminal:

```bash
ros2 run tf2_ros tf2_echo frame_base frame_1
ros2 run tf2_ros tf2_echo frame_base livox_frame
ros2 run tf2_tools view_frames
```

---

# 2026-06-20: Isaac Sim ROS State Verification

## Verified Result

The ROS-enabled USD was run successfully in Isaac Sim 4.5 with physics enabled:

```text
isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd
```

The live ROS graph contained `/clock`, `/joint_states`, `/tf`, `/tf_static`, and `/robot_description`. Isaac Sim published all 17 expected movable joints. `robot_state_publisher` used the generated plain URDF and was the only TF authority for the robot tree.

The calibrated transforms matched the recorded ground truth exactly:

```text
frame_base -> frame_1:     xyz=-0.076 0.000 0.519, quaternion=0 0 0 1
frame_base -> livox_frame: xyz= 0.149 0.000 0.444, quaternion=0 0 0 1
```

The initial missing-frame line from `tf2_echo` was a startup race; both static transforms were then continuously available.

Final verification:

- Isaac USD contract: 6 tests passed
- Plain URDF: parsed successfully with `base_link` as the single root
- Support block: 10 kg mass and collision geometry present in the imported USD

## Run Commands

Terminal 1, Isaac Sim with GUI:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
export ROS_DISTRO=humble
export LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}
/home/user/anaconda3/envs/env_isaaclab/bin/python isaac_sim/scripts/run_mobile_manipulator.py
```

Terminal 2, robot state publisher using the already generated import URDF:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  build/isaac_import/mobile_manipulator.urdf \
  --ros-args -p use_sim_time:=true
```

Terminal 3, verification:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo frame_base frame_1
ros2 run tf2_ros tf2_echo frame_base livox_frame
```

## Remaining Issue

`ros-humble-xacro` is still absent from the host. Install the declared dependencies before using `isaac_state.launch.py` or `display.launch.py` directly:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
source /opt/ros/humble/setup.bash
rosdep install --from-paths mobile_manipulator_description --ignore-src -r -y
```

---

# 2026-06-21: Isaac Sim ROS2 Control

## Implemented

- `/cmd_vel` (`geometry_msgs/msg/Twist`) drives `left_wheel` and `right_wheel` through an Isaac differential controller.
- `/arm_joint_commands` (`sensor_msgs/msg/JointState`) sends position targets to `joint1` through `joint7`.
- `/joint_states` and `/clock` remain published by Isaac Sim.
- The ROS overlay now explicitly preserves `Z`-up and `1.0` meters/unit from the base USD.
- Drive wheels use velocity gains `stiffness=0`, `damping=10000` after import.
- The package-local left wheel axis is `0 -1 0`, compensating for its pi-roll joint frame so both drive axes align in the robot frame.
- Runtime diagnostics support root-pose reporting and disabling controllers or the complete action graph.

The original mobile and manipulator description files were not modified.

## Verification

- Generated USD contract: 10 tests passed.
- Lightweight runner and description contracts: 10 tests passed.
- Stationary active-graph physics remained stable at approximately `z=0.142517 m`.
- A `0.2 m/s` command produced wheel velocities `3.2915` and `3.3099 rad/s`; expected is `3.3058 rad/s`.
- Arm command `[0.2, -0.2, 0, 0, 0, 0, 0]` reached `joint1=0.1995`, `joint2=-0.1988`.

One final forward-motion pose check is pending after restarting the host ROS process state. During the last run, ROS CLI discovery and host process inspection became wedged, although Isaac Sim and all deterministic tests remained healthy.

## Next Commands

After restarting the host session, terminal 1:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}
/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_mobile_manipulator.py --pose-report-period 1
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
sleep 3
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
ros2 topic pub --once /arm_joint_commands sensor_msgs/msg/JointState \
  '{name: [joint1, joint2, joint3, joint4, joint5, joint6, joint7], position: [0.15, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]}'
ros2 topic echo /joint_states --once
```

# 2026-06-21: MoveIt Pose Control In Isaac Sim

## What Changed

Created `mobile_manipulator_moveit_config` with the xArm7 SRDF, KDL/OMPL settings,
joint limits, controller mapping, RViz MotionPlanning configuration, and the
integrated `moveit_isaac.launch.py` launch file.

Created `mobile_manipulator_moveit_bridge` with:

- `/arm_controller/follow_joint_trajectory` trajectory execution
- 100 Hz interpolation to `/arm_joint_commands`
- measured `/joint_states` feedback, tolerances, and cancellation hold support
- `/arm_target_pose` validation, TF conversion, MoveIt planning, and execution

## Live Result

A `PoseStamped` target in `base_link` at:

```text
position:    [0.12920498, 0.00005203, 0.65890382]
orientation: [0.99999670, -0.00029999, -0.00255000, -0.00000230]  # xyzw
```

planned and executed successfully in Isaac Sim. The final measured transform was:

```text
position:    [0.129067652, 0.000393706, 0.658323246]
orientation: [0.999992936, 0.000061845, -0.003412262, -0.001575234]  # xyzw
```

The translation error was approximately `0.00069 m`. An unreachable target at
`[5, 0, 5]` failed planning and did not change the arm joints.

Final checks passed: 21 lightweight contracts, 4 trajectory sampler tests, 10
Isaac USD contracts, all three package builds, and both original URDF hashes.

RViz loaded the unified model and connected its MotionPlanning panel to group
`xarm7`. A 10-second action trajectory canceled after two seconds with action
status `5` (canceled); the hold command matched the measured arm position within
`0.0033 rad`. The only remaining manual check is one Plan/Execute operation from
the RViz panel. The small Cartesian test selected a distant IK branch, so
joint-space continuity constraints are the next control-quality improvement.
Forced SIGINT teardown also triggered a MoveIt Humble class-loader cleanup
segfault after successful execution.

## Next Commands

Terminal 1:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}
/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_mobile_manipulator.py --pose-report-period 1
```

Terminal 2:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch mobile_manipulator_moveit_config moveit_isaac.launch.py \
  use_rviz:=true use_sim_time:=true
```

The next implementation milestone is MoveIt Servo for low-latency Cartesian
commands, after adding a joint-continuity policy and completing the remaining
RViz Plan/Execute check above.

# 2026-06-22: One-Command MoveIt Test Launcher

Run the complete Isaac Sim, MoveIt, and RViz test stack from the workspace root:

```bash
./start_moveit_test.sh
```

The terminal remains attached. Wait for RViz to load, use the MotionPlanning
panel, and press `Ctrl+C` in the terminal to stop both the ROS launch and Isaac
Sim. The launcher uses Fast DDS for Isaac and Cyclone DDS for the host MoveIt
stack, matching the verified configuration on this machine.

To validate paths and inspect the resolved commands without starting Isaac or
RViz:

```bash
./start_moveit_test.sh --dry-run
```

# 2026-06-22: Nav2 Factory SLAM Bootstrap

## What Changed

- Added the collision-enabled 10 m by 8 m factory at
  `isaac_sim/assets/environments/compact_factory/compact_factory.usd`.
- Added Isaac `/odom`, dynamic `odom -> base_link`, and a temporary 360-degree
  RTX `/scan` sensor attached to `livox_frame`.
- Added `mobile_manipulator_navigation` with SLAM Toolbox, Nav2, and RViz
  configuration. Nav2 uses `base_link`; `base_link -> frame_base` remains the
  fixed identity calibration alias.
- Added `start_navigation_test.sh`, which supervises Isaac and the complete ROS
  navigation stack and cleans both process groups on exit.

Deterministic verification passes: 36 lightweight contracts, 4 factory USD
contracts, and 11 robot USD contracts. Live DDS validation is still pending:
Kit reached `Simulation App Startup Complete`, but the existing host ROS/process
state caused direct DDS probes and bounded process teardown to hang.

## Next Commands

After restarting the host session:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
./start_navigation_test.sh --dry-run
./start_navigation_test.sh
```

Wait for Isaac Sim and RViz2. In RViz2, confirm `/scan`, the growing `/map`, and
the frame chain `map -> odom -> base_link -> frame_base -> livox_frame`. Use the
Nav2 Goal tool only after the map around the selected goal is observed. Press
`Ctrl+C` in the launcher terminal to stop the complete stack.

## 2026-06-23 Runner Fix

`run_factory_navigation.py` now edits the imported `/ActionGraph` through its
existing OmniGraph handle and connects the scan publisher using the absolute
playback-tick attribute path. This fixes the prior `A graph already exists at
this path` failure and the subsequent relative-attribute connection error. A
five-second headless Isaac run with LiDAR enabled completed successfully.

## 2026-06-23 Isaac Floor-Clearance Fix

The imported chassis collision mesh extends lower than the previous `0.242 m`
robot spawn height, which placed the mobile base collision partly inside the
ground plane and could make the robot settle tilted under physics. The Isaac
factory layout now uses `ROBOT_CLEARANCE_Z = 0.305`, and the standalone mobile
manipulator runner uses the same value when placing `/mobile_manipulator`.

Regression coverage:

```bash
python3 -m unittest isaac_sim.tests.test_factory_layout_contract -v
python3 -m py_compile \
  isaac_sim/scripts/factory_layout.py \
  isaac_sim/scripts/run_mobile_manipulator.py \
  isaac_sim/tests/test_factory_layout_contract.py
```

Next command for the Nav2 point-goal check:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
./start_navigation_test.sh
```

In RViz2, wait until `/scan`, `/odom`, and the local map are visible, then use
the Nav2 Goal tool on a nearby reachable point in the observed aisle.

## 2026-06-23 Isaac Articulation Spawn Fix

The robot still appeared below the floor after the clearance change because the
runner was lifting the parent `/mobile_manipulator` namespace prim. For PhysX
articulations the reliable spawn pose is the articulation root rigid body,
`/mobile_manipulator/base_link`.

Updated both Isaac runners:

- `isaac_sim/scripts/run_mobile_manipulator.py`
- `isaac_sim/scripts/run_factory_navigation.py`

The parent prim now provides only the container placement/yaw, while
`/mobile_manipulator/base_link` receives the Z clearance directly.

Regression coverage:

```bash
python3 -m unittest isaac_sim.tests.test_run_contract -v
python3 -m py_compile \
  isaac_sim/scripts/run_mobile_manipulator.py \
  isaac_sim/scripts/run_factory_navigation.py \
  isaac_sim/tests/test_run_contract.py
./start_navigation_test.sh --dry-run
```

Next live check:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
./start_navigation_test.sh
```

In Isaac Sim, first confirm that the mobile base is above the factory floor and
not tilted. Then continue the Nav2 point-goal check in RViz2.

## 2026-06-23 Factory Pipe Height and Stage Composition Fix

Regenerated the compact factory USD after lowering every horizontal pipe to the
`0.7 m` to `1.4 m` range:

```bash
/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/generate_factory_environment.py
```

`run_factory_navigation.py` now builds one composed Isaac session stage by
loading the factory USD first and the robot USD second through root-layer
sublayers. This avoids replacing the factory stage with `open_stage(robot_usd)`
and fixes the stale-stage `Stage.GetPrimAtPath(Stage, str)` failure path.

The runner uses explicit `Sdf.Path(...)` lookups for `/Factory`,
`/mobile_manipulator`, and `/mobile_manipulator/base_link`, then places:

- `/mobile_manipulator` at the configured XY/yaw spawn
- `/mobile_manipulator/base_link` at the configured spawn Z

Regression coverage:

```bash
python3 -m unittest \
  isaac_sim.tests.test_factory_layout_contract \
  isaac_sim.tests.test_navigation_contract \
  isaac_sim.tests.test_run_contract -v

/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/tests/test_factory_usd_contract.py
```

The generated USD contract also now asserts that `/Factory/Ground` has
`UsdPhysics.CollisionAPI`.

Next visual check:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
./start_factory_scene.sh
```

The factory scene launcher now always starts the LiDAR and publishes `/scan`;
there is no no-LiDAR mode.

## 2026-06-23 Factory Lighting Fix

The generated factory USD now includes scene lighting so the environment is
visible when opened directly in Isaac Sim:

- `/Factory/Lights/Ambient`: dome light
- `/Factory/Lights/OverheadSouth`: large rectangular overhead light
- `/Factory/Lights/OverheadNorth`: large rectangular overhead light

Regenerated:

```bash
/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/generate_factory_environment.py
```

Verified:

```bash
/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/tests/test_factory_usd_contract.py
```

Next visual check:

```bash
./start_factory_scene.sh
```

## 2026-06-23 Factory Robot Sinking Follow-Up

Rechecked `isaac_sim/scripts/run_factory_navigation.py` after the robot was
still reported below the floor. The factory runner had regressed to using the
spawn Z on the parent `/mobile_manipulator` prim and no longer applied the
height directly to the PhysX articulation root link.

Updated behavior:

- `/mobile_manipulator` is placed at the configured X/Y/yaw only, with Z set to
  `0.0`
- `/mobile_manipulator/base_link` is placed at `ROBOT_SPAWN[2]`
- after playback starts, the PhysX articulation root body is explicitly reset
  to the same spawn pose with zero linear and angular velocity

Verified:

```bash
python3 -m unittest \
  isaac_sim.tests.test_run_contract \
  isaac_sim.tests.test_navigation_contract -v

python3 -m py_compile \
  isaac_sim/scripts/run_factory_navigation.py \
  isaac_sim/tests/test_run_contract.py \
  isaac_sim/tests/test_navigation_contract.py

./start_factory_scene.sh --dry-run
```

Live Isaac startup in this Codex environment did not reach the script body
because Kit failed during GPU/display initialization before scene loading. The
code path is therefore contract-verified here, but the visual floor-contact
check still needs to be run on the Isaac GUI session.

Next visual check:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
./start_factory_scene.sh
```

If the base still sinks after this change, the next likely cause is the imported
robot collision setup rather than the factory spawn transform.

## 2026-06-23 Nav2 Map Frame Debug

Reported symptom: RViz2 says the `map` frame does not exist during the Nav2
test.

Root-cause findings from the live ROS logs:

- `map` is produced by SLAM Toolbox as `map -> odom`; Nav2 does not create it.
- The first failed launch had `controller_server` crash immediately because the
  local costmap `height` parameter was a YAML double while this Nav2 Humble
  build declares it as an integer.
- SLAM Toolbox received `/scan`, but dropped scan messages because the required
  TF chain to `odom` was not available at the scan timestamps.

Changed:

- `mobile_manipulator_navigation/config/nav2_params.yaml`
  - local costmap `width: 4`
  - local costmap `height: 4`
- `isaac_sim/tests/test_navigation_contract.py`
  - added a regression check that local costmap width/height remain integers

Installed the updated navigation package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_manipulator_navigation
```

Verified:

```bash
python3 -m unittest isaac_sim.tests.test_navigation_contract -v
python3 -m py_compile isaac_sim/tests/test_navigation_contract.py
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
timeout 8 ros2 launch mobile_manipulator_navigation mapping.launch.py \
  use_rviz:=false use_sim_time:=true
```

The bounded launch now gets past controller creation/configuration and waits for
`odom -> base_link` instead of crashing.

Next live check from a clean terminal:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
./start_navigation_test.sh
```

Before using the RViz2 Nav2 Goal tool, confirm:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS2CLI_NO_DAEMON=1 ROS_LOG_DIR=/tmp/ros-log
ros2 topic info /scan --no-daemon
ros2 topic info /odom --no-daemon
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Only after `/scan`, `/odom`, `odom -> base_link`, and `base_link -> livox_frame`
exist should RViz2 show `map`.

## 2026-06-23 Keyboard Teleop Script

Added:

```bash
./start_teleop.sh
```

Purpose:

- starts `teleop_twist_keyboard`
- sources `/opt/ros/humble/setup.bash`
- sources the workspace `install/setup.bash`
- uses `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` by default
- remaps `cmd_vel` to `/cmd_vel`

Run:

```bash
cd /home/user/wu_ws/mobile_manipulator_exploration
./start_teleop.sh
```

Dry-run check:

```bash
./start_teleop.sh --dry-run
```

## 2026-06-23 Slam Toolbox Base Frame Fix

Observation:

- Slam Toolbox's default config at
  `/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml`
  uses `base_frame: base_footprint`.
- This robot currently uses `base_link` as the Nav2 base frame.

The project config already had:

```yaml
base_frame: base_link
```

But to remove ambiguity, `mobile_manipulator_navigation/launch/mapping.launch.py`
now starts `slam_toolbox/async_slam_toolbox_node` directly with the project
`slam_toolbox.yaml` instead of including Slam Toolbox's default
`online_async_launch.py`.

Changed:

- `mobile_manipulator_navigation/launch/mapping.launch.py`
- `isaac_sim/tests/test_navigation_contract.py`

Verified:

```bash
python3 -m unittest isaac_sim.tests.test_navigation_contract -v
python3 -m py_compile \
  mobile_manipulator_navigation/launch/mapping.launch.py \
  isaac_sim/tests/test_navigation_contract.py
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_manipulator_navigation
```

Runtime note:

In a bounded launch without Isaac odometry active, Nav2 and SLAM start but Nav2
waits for `odom -> base_link`. `map -> odom` will not appear until Isaac is
publishing `/odom` and `odom -> base_link`, and SLAM can consume `/scan`.

## 2026-06-24 Slam Toolbox Scan Queue Fix

Live graph status:

- `/slam_toolbox` is running.
- live parameters confirm:
  - `mode: mapping`
  - `map_frame: map`
  - `odom_frame: odom`
  - `base_frame: base_link`
  - `scan_topic: /scan`
- `/scan` is publishing at about 50 Hz.
- `odom -> base_link` exists.
- `base_link -> livox_frame` exists.
- `/map` has a publisher.

Root-cause evidence:

The Slam Toolbox log repeatedly reports:

```text
Message Filter dropping message: frame 'livox_frame' ... reason 'discarding message because the queue is full'
```

Changed `mobile_manipulator_navigation/config/slam_toolbox.yaml` to reduce scan
pressure and allow more TF lookup time:

```yaml
minimum_time_interval: 0.3
transform_timeout: 1.0
throttle_scans: 5
scan_queue_size: 200
```

Added regression coverage in:

```text
isaac_sim/tests/test_navigation_contract.py
```

Verified and installed:

```bash
python3 -m unittest isaac_sim.tests.test_navigation_contract -v
python3 -m py_compile isaac_sim/tests/test_navigation_contract.py
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_manipulator_navigation
```

Required next step:

Restart `./start_navigation_test.sh` so the running Slam Toolbox process loads
the new installed parameters.

## 2026-06-25 Controlled Rooms SLAM Test Environment

Added a less cluttered Isaac test environment for controlled SLAM/Nav2 checks:

- New source layout:
  - `isaac_sim/scripts/controlled_rooms_layout.py`
- New USD generator:
  - `isaac_sim/scripts/generate_controlled_rooms_environment.py`
- Generated USD:
  - `isaac_sim/assets/environments/controlled_rooms/controlled_rooms.usd`
- New launch helpers:
  - `./start_controlled_rooms_scene.sh`
  - `./start_controlled_rooms_navigation.sh`

Environment structure:

- Four 6 m x 6 m rooms in a 2x2 layout, total 12 m x 12 m.
- Door gaps between adjacent rooms allow the robot to pass room to room.
- Room 1 has two 1 m vertical pipes in the middle, each with a horizontal pipe
  from its top to the west wall.
- Room 2 has four vertical pipes in one line, 0.5 m from the south wall.
- Room 3 has one tank with 1 m diameter and 0.7 m height.
- Room 4 has two vertical-pipe lines near different walls, and one line has a
  horizontal pipe connecting all pipes.

Runner change:

- `isaac_sim/scripts/run_factory_navigation.py` now accepts:

```bash
--spawn X Y Z YAW
```

This allows different test environments to use different robot start poses
without changing the old compact factory layout.

Verified:

```bash
python3 -m py_compile \
  isaac_sim/scripts/controlled_rooms_layout.py \
  isaac_sim/scripts/generate_controlled_rooms_environment.py \
  isaac_sim/scripts/run_factory_navigation.py
bash -n start_controlled_rooms_scene.sh start_controlled_rooms_navigation.sh
/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/generate_controlled_rooms_environment.py
./start_controlled_rooms_scene.sh --dry-run
./start_controlled_rooms_navigation.sh --dry-run
env ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-} \
  PYTHONUNBUFFERED=1 \
  /home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_factory_navigation.py \
  --factory-usd isaac_sim/assets/environments/controlled_rooms/controlled_rooms.usd \
  --robot-usd isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd \
  --spawn -4.5 -4.5 0.6 0.0 \
  --headless --duration 1
```

Next commands:

```bash
# Scene only
./start_controlled_rooms_scene.sh

# Full Isaac + SLAM/Nav2/RViz test
./start_controlled_rooms_navigation.sh
```

## 2026-07-02 Wrist RGB-D Camera on End Effector

Added a RealSense-D455-style wrist camera to the unified robot description and Isaac ROS overlay.

Description changes:

- `mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro`
- Added fixed frames:
  - `wrist_camera_link`
  - `wrist_camera_color_optical_frame`
- Parent joint:
  - `link_eef_to_wrist_camera`
  - parent: `link_eef`
  - child: `wrist_camera_link`
- Current mount transform is a placeholder, not calibrated:
  - `wrist_camera_xyz="0.035 0 0.025"`
  - `wrist_camera_rpy="0 0 0"`

Isaac USD/ROS overlay changes:

- `isaac_sim/scripts/import_mobile_manipulator.py`
- Added USD camera prim:
  - `/mobile_manipulator/wrist_camera_color_optical_frame/D455Camera`
- Added ROS2 publishers:
  - `/wrist_camera/color/image_raw`
  - `/wrist_camera/depth/image_rect_raw`
  - `/wrist_camera/color/camera_info`
- Frame ID:
  - `wrist_camera_color_optical_frame`
- Resolution:
  - `640x480`
- Clipping range:
  - `0.15 m` to `8.0 m`
- Camera info now uses Isaac 4.5's dedicated:
  - `isaacsim.ros2.bridge.ROS2CameraInfoHelper`

Regenerated:

- `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator.usd`
- `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd`

Runner changes:

- `isaac_sim/scripts/run_factory_navigation.py`
- The scene runner now verifies the baked wrist camera prim and RGB/depth/camera-info graph nodes before starting simulation.
- Startup print now includes the wrist camera topics.

Verified:

```bash
python3 -m py_compile \
  isaac_sim/scripts/run_factory_navigation.py \
  isaac_sim/scripts/import_mobile_manipulator.py \
  isaac_sim/tests/test_navigation_contract.py \
  isaac_sim/tests/test_description_contract.py \
  isaac_sim/tests/test_import_contract.py

python3 -m unittest \
  isaac_sim.tests.test_description_contract \
  isaac_sim.tests.test_navigation_contract -v

env ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-} \
  PYTHONUNBUFFERED=1 \
  /home/user/anaconda3/envs/env_isaaclab/bin/python \
  -m unittest \
  isaac_sim.tests.test_import_contract.GeneratedAssetTest \
  isaac_sim.tests.test_import_contract.ImportedRobotContractTest.test_calibration_and_support_links_are_preserved \
  isaac_sim.tests.test_import_contract.ImportedRobotContractTest.test_ros_overlay_has_wrist_rgbd_camera_publishers -v

env ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-} \
  PYTHONUNBUFFERED=1 \
  /home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_factory_navigation.py \
  --factory-usd isaac_sim/assets/environments/controlled_rooms/controlled_rooms.usd \
  --robot-usd isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd \
  --spawn -4.5 -4.5 0.6 0.0 \
  --headless --duration 1
```

Next commands:

```bash
./start_controlled_rooms_mobile_manipulator.sh --use-moveit-rviz

ros2 topic list | rg wrist_camera
ros2 topic echo /wrist_camera/color/camera_info --once
ros2 topic hz /wrist_camera/color/image_raw
ros2 topic hz /wrist_camera/depth/image_rect_raw
```

## 2026-07-02 Wrist Camera Render Direction Fix

Corrected the USD camera prim orientation while keeping the ROS optical frame unchanged.

Reason:

- ROS optical convention uses:
  - `+Z` forward into the camera FOV
  - `+X` image right
  - `+Y` image down
- USD cameras render along local `-Z`.

Change:

- `isaac_sim/scripts/import_mobile_manipulator.py`
- The USD camera prim now has:
  - `xformOp:rotateXYZ = (180.0, 0.0, 0.0)`
- This maps USD camera render direction `-Z` to ROS optical `+Z`.
- The published ROS frame remains:
  - `wrist_camera_color_optical_frame`

Regenerated:

- `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator.usd`
- `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd`

Verified:

```bash
python3 -m py_compile \
  isaac_sim/scripts/import_mobile_manipulator.py \
  isaac_sim/tests/test_import_contract.py

USD_LIB=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310
PYTHONPATH=$USD_LIB LD_LIBRARY_PATH=$USD_LIB/bin \
  /home/user/anaconda3/envs/env_isaaclab/bin/python - <<'PY'
from pxr import Usd
stage = Usd.Stage.Open('isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd')
prim = stage.GetPrimAtPath('/mobile_manipulator/wrist_camera_color_optical_frame/D455Camera')
print('camera_valid', prim.IsValid())
attr = prim.GetAttribute('xformOp:rotateXYZ')
print('rotate_valid', attr.IsValid())
print('rotate', tuple(attr.Get()))
PY

env ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-} \
  PYTHONUNBUFFERED=1 \
  /home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_factory_navigation.py \
  --factory-usd isaac_sim/assets/environments/controlled_rooms/controlled_rooms.usd \
  --robot-usd isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd \
  --spawn -4.5 -4.5 0.6 0.0 \
  --headless --duration 1
```

## 2026-07-02 Wrist Camera Axis Alignment with End Effector

Aligned the wrist camera frame with the manipulator end-effector frame.

Requirement:

- `wrist_camera_color_optical_frame` `+Z` should align with `link_eef` `+Z`.

Description change:

- `mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro`
- `link_eef_to_wrist_camera` keeps:
  - `wrist_camera_rpy="0 0 0"`
- `wrist_camera_link_to_color_optical_frame` now uses an identity transform:
  - `xyz="0 0 0"`
  - `rpy="0 0 0"`

Isaac convention:

- Keep the USD camera prim rotation:
  - `xformOp:rotateXYZ = (180.0, 0.0, 0.0)`
- Reason: USD cameras render along local `-Z`, so this makes the rendered FOV
  point along the ROS camera frame `+Z`, which is now aligned with `link_eef`
  `+Z`.

Regenerated:

- `build/isaac_import/mobile_manipulator.urdf`
- `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator.usd`
- `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd`

Verified:

```bash
python3 -m py_compile \
  isaac_sim/scripts/import_mobile_manipulator.py \
  isaac_sim/scripts/run_factory_navigation.py \
  isaac_sim/tests/test_description_contract.py \
  isaac_sim/tests/test_import_contract.py

python3 -m unittest isaac_sim.tests.test_description_contract -v

sed -n '980,996p' build/isaac_import/mobile_manipulator.urdf

USD_LIB=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310
PYTHONPATH=$USD_LIB LD_LIBRARY_PATH=$USD_LIB/bin \
  /home/user/anaconda3/envs/env_isaaclab/bin/python -c \
  "from pxr import Usd; stage = Usd.Stage.Open('isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd'); prim = stage.GetPrimAtPath('/mobile_manipulator/wrist_camera_color_optical_frame/D455Camera'); print('camera_valid', prim.IsValid()); attr = prim.GetAttribute('xformOp:rotateXYZ'); print('rotate_valid', attr.IsValid()); print('rotate', tuple(attr.Get()))"
```

## 2026-07-02 Point Goal Navigation and Manipulator Integration Branch

Created working branch:

```bash
feature/point-goal-navigation-manipulator-control
```

Readiness:

- Point-goal Nav2 navigation is ready to test with the existing controlled rooms
  stack.
- Manipulator pose-goal control already exists through MoveIt:
  - target topic: `/arm_target_pose`
  - message type: `geometry_msgs/msg/PoseStamped`
  - planning frame: `base_link`
  - end-effector link: `link_eef`
- The remaining integration work was to start Nav2 and MoveIt together without
  duplicate `robot_state_publisher` or duplicate RViz2 instances.

Changes:

- `mobile_manipulator_navigation/launch/mapping.launch.py`
  - Added `use_robot_state_publisher` launch argument.
- `mobile_manipulator_moveit_config/launch/moveit_isaac.launch.py`
  - Added `use_robot_state_publisher` launch argument.
- `start_controlled_rooms_mobile_manipulator.sh`
  - Starts Isaac controlled rooms scene.
  - Starts SLAM Toolbox, Nav2, and RViz2.
  - Starts MoveIt, `isaac_trajectory_bridge`, and `pose_goal_planner`.
  - Runs only one `robot_state_publisher`.
  - Runs only the Nav2 RViz instance.
- `send_arm_target_pose.sh`
  - Publishes one `PoseStamped` target to `/arm_target_pose`.

Rebuilt:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  mobile_manipulator_navigation \
  mobile_manipulator_moveit_config
```

Verified:

```bash
python3 -m py_compile \
  mobile_manipulator_navigation/launch/mapping.launch.py \
  mobile_manipulator_moveit_config/launch/moveit_isaac.launch.py

bash -n \
  start_controlled_rooms_mobile_manipulator.sh \
  send_arm_target_pose.sh

./start_controlled_rooms_mobile_manipulator.sh --dry-run

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log
ros2 launch mobile_manipulator_navigation mapping.launch.py --show-args
ros2 launch mobile_manipulator_moveit_config moveit_isaac.launch.py --show-args

timeout 8 ros2 launch mobile_manipulator_navigation mapping.launch.py \
  use_rviz:=false \
  use_sim_time:=true \
  use_robot_state_publisher:=true

timeout 8 ros2 launch mobile_manipulator_moveit_config moveit_isaac.launch.py \
  use_rviz:=false \
  use_sim_time:=true \
  use_robot_state_publisher:=false
```

The timeout launch checks started the nodes successfully. Nav2 showed expected
missing `odom` TF warnings when Isaac was not running. MoveIt reached:

```text
You can start planning now!
```

Next operator test:

```bash
./start_controlled_rooms_mobile_manipulator.sh
```

In RViz2:

1. Drive/build the map if needed.
2. Use `Nav2 Goal` to send a base point goal.
3. After the base reaches the area, test arm pose control:

```bash
./send_arm_target_pose.sh 0.45 0.00 0.55 0.0 1.0 0.0 0.0 base_link
```

## 2026-07-02 MoveIt RViz GUI Option and Arm Target Script Fix

Fixed:

- `send_arm_target_pose.sh` no longer fails on ROS setup with:
  - `AMENT_TRACE_SETUP_FILES: unbound variable`
- Root cause was `set -u` in the helper while sourcing ROS setup files that
  reference optional unset variables.
- The script now temporarily disables nounset while sourcing:
  - `/opt/ros/humble/setup.bash`
  - `install/setup.bash`
- The script also bounds the wait for a `/arm_target_pose` subscriber with:
  - `ARM_TARGET_WAIT_SECONDS`, default `10`

Added:

- `start_controlled_rooms_mobile_manipulator.sh --use-moveit-rviz`

This starts the same integrated stack, but also launches MoveIt RViz for GUI arm
target control. It does not start a duplicate MoveIt backend; it only changes the
MoveIt launch argument:

```bash
use_rviz:=true
```

Next GUI manipulation test:

```bash
./start_controlled_rooms_mobile_manipulator.sh --use-moveit-rviz
```

Expected windows:

- Nav2 RViz for map and base point-goal navigation.
- MoveIt RViz for interactive manipulator pose targets.

Command-line pose target test remains:

```bash
./send_arm_target_pose.sh 0.45 0.00 0.55 0.0 1.0 0.0 0.0 base_link
```

Verified:

```bash
bash -n send_arm_target_pose.sh start_controlled_rooms_mobile_manipulator.sh
./start_controlled_rooms_mobile_manipulator.sh --use-moveit-rviz --dry-run
ARM_TARGET_WAIT_SECONDS=2 ./send_arm_target_pose.sh \
  0.45 0.00 0.55 0.0 1.0 0.0 0.0 base_link
```

The last command was run without the integrated stack; it no longer fails during
ROS setup and exits after the bounded wait because no `pose_goal_planner`
subscriber is running in that isolated check.

## 2026-07-01 Baked LiDAR USD Render Product Fix

Current robot USD behavior:

- The RTX LiDAR prim is baked into the robot USD at:
  - `/mobile_manipulator/livox_frame/NavLidar`
- The ROS2 scan publisher is baked into the robot USD ActionGraph as:
  - `ScanPublisher`
- The scan publisher no longer stores a stale fixed render product path like:
  - `/Render/OmniverseKit/HydraTextures/Replicator`
- Instead, the baked ActionGraph creates the LiDAR render product at runtime with:
  - `CreateLidarRenderProduct`
  - node type: `isaacsim.core.nodes.IsaacCreateRenderProduct`
  - camera prim: `/mobile_manipulator/livox_frame/NavLidar`

Files changed:

- `isaac_sim/scripts/import_mobile_manipulator.py`
  - Creates the RTX LiDAR prim directly with `IsaacSensorCreateRtxLidar`.
  - Adds `CreateLidarRenderProduct` and connects its output path to
    `ScanPublisher.inputs:renderProductPath`.
- `isaac_sim/scripts/run_factory_navigation.py`
  - Accepts either `/ActionGraph` or `/mobile_manipulator/ActionGraph`, because
    Isaac composition may place the imported graph at either path.
- `isaac_sim/tests/test_navigation_contract.py`
  - Locks the baked-LiDAR and runtime-render-product contract.

Regenerated robot USD:

```text
isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator.usd
isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd
```

Minimum LiDAR range remains here:

```text
isaac_sim/config/lidar/MobileManipulator_Nav2D.json
```

Important values:

```json
"nearRangeM": 0.15,
"minDistBetweenEchos": 0.15
```

Verified:

```bash
python3 -m py_compile \
  isaac_sim/scripts/import_mobile_manipulator.py \
  isaac_sim/scripts/run_factory_navigation.py \
  isaac_sim/tests/test_navigation_contract.py

python3 -m unittest \
  isaac_sim.tests.test_navigation_contract.FactoryNavigationRunnerContract -v

env ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-} \
  PYTHONUNBUFFERED=1 \
  /home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/import_mobile_manipulator.py

env ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-} \
  PYTHONUNBUFFERED=1 \
  /home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_factory_navigation.py \
  --factory-usd isaac_sim/assets/environments/controlled_rooms/controlled_rooms.usd \
  --robot-usd isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd \
  --spawn -4.5 -4.5 0.6 0.0 \
  --headless --duration 1
```

Next commands:

```bash
# Scene only
./start_controlled_rooms_scene.sh

# Full Isaac + SLAM/Nav2/RViz test
./start_controlled_rooms_navigation.sh
```

## 2026-06-25 Robot USD Regeneration and Required LiDAR Scan

Regenerated the Isaac robot USD from the unified Xacro/URDF path:

- Expanded Xacro:
  - `build/isaac_import/mobile_manipulator.urdf`
- Regenerated Isaac USD:
  - `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator.usd`
  - `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd`

Removed the old no-LiDAR mode from the Isaac scene launchers:

- `./start_factory_scene.sh`
- `./start_controlled_rooms_scene.sh`

LiDAR/scan behavior:

- The scene runner always creates the RTX LiDAR at:
  - `/mobile_manipulator/livox_frame/NavLidar`
- It publishes ROS2 `LaserScan` on:
  - topic: `/scan`
  - frame: `livox_frame`
- The runner loads the project-local LiDAR profile directory via Isaac setting:
  - `/app/sensors/nv/lidar/profileBaseFolder`

Minimum LiDAR range is set here:

```text
isaac_sim/config/lidar/MobileManipulator_Nav2D.json
```

Important profile values:

```json
"nearRangeM": 0.15,
"minDistBetweenEchos": 0.15,
"elevationDeg": [0.0]
```

The runner selects that profile here:

```text
isaac_sim/scripts/run_factory_navigation.py
config_file_name="MobileManipulator_Nav2D"
```

Verified:

```bash
python3 -m py_compile \
  isaac_sim/scripts/run_factory_navigation.py \
  isaac_sim/scripts/import_mobile_manipulator.py \
  isaac_sim/tests/test_navigation_contract.py

bash -n \
  start_factory_scene.sh \
  start_controlled_rooms_scene.sh \
  start_navigation_test.sh \
  start_controlled_rooms_navigation.sh

python3 -m json.tool isaac_sim/config/lidar/MobileManipulator_Nav2D.json
python3 -m unittest isaac_sim.tests.test_navigation_contract.FactoryNavigationRunnerContract

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xacro xacro \
  mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro \
  > build/isaac_import/mobile_manipulator.urdf

/home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/import_mobile_manipulator.py

env ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/user/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-} \
  PYTHONUNBUFFERED=1 \
  /home/user/anaconda3/envs/env_isaaclab/bin/python \
  isaac_sim/scripts/run_factory_navigation.py \
  --factory-usd isaac_sim/assets/environments/controlled_rooms/controlled_rooms.usd \
  --robot-usd isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd \
  --spawn -4.5 -4.5 0.6 0.0 \
  --headless --duration 1
```

Next commands:

```bash
# Scene only, with LiDAR and /scan
./start_controlled_rooms_scene.sh

# Full Isaac + SLAM/Nav2/RViz test
./start_controlled_rooms_navigation.sh
```
