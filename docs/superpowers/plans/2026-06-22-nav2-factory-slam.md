# Nav2 Factory SLAM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a compact pipe-dense factory, publish Isaac odometry and a 360-degree scan, and navigate the mapped environment with ROS 2 Humble Nav2 and SLAM Toolbox.

**Architecture:** A dependency-free layout module describes the factory and an Isaac generator writes a reusable collision-enabled USD. The existing robot ROS overlay gains `/odom` and `odom -> base_link`; a factory runner composes the environment and creates an RTX 2D LiDAR under `livox_frame`. A new ROS package owns SLAM Toolbox, Nav2, RViz, and navigation launch configuration.

**Tech Stack:** Python 3.10, Isaac Sim 4.5 USD/OmniGraph/RTX LiDAR, ROS 2 Humble, Nav2, SLAM Toolbox, `ament_cmake`, Python `unittest`

**Repository note:** `.git` is not usable in this workspace. Replace commit checkpoints with fresh tests and `progress.md` updates; do not initialize Git.

---

## File Map

- `isaac_sim/scripts/factory_layout.py`: dependency-free dimensions and primitive placement.
- `isaac_sim/scripts/generate_factory_environment.py`: collision-enabled USD generator.
- `isaac_sim/scripts/run_factory_navigation.py`: factory composition, RTX LiDAR, and simulation runner.
- `isaac_sim/assets/environments/compact_factory/compact_factory.usd`: generated factory asset.
- `isaac_sim/tests/test_factory_layout_contract.py`: geometry, count, and clearance contracts.
- `isaac_sim/tests/test_navigation_contract.py`: sensor, odometry, package, launch, and script contracts.
- `isaac_sim/tests/test_factory_usd_contract.py`: Isaac/PXR generated-stage contracts.
- `mobile_manipulator_navigation/`: SLAM Toolbox, Nav2, RViz, and launch package.
- `start_navigation_test.sh`: foreground supervisor for Isaac and ROS navigation.

## Task 1: Install Navigation Dependencies

**Files:** Modify `progress.md`, `task_plan.md`

- [x] Confirm `nav2_bringup`, `nav2_controller`, and `slam_toolbox` are absent with `ros2 pkg prefix`.
- [x] Install `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-slam-toolbox`, and `ros-humble-teleop-twist-keyboard` using apt.
- [x] Verify every package prefix resolves under `/opt/ros/humble` and record `dpkg-query -W` versions in `progress.md`.
- [x] If apt fails, record the exact error in `task_plan.md` before changing approach. (No apt error occurred.)

## Task 2: Define Factory Layout Test-First

**Files:** Create `isaac_sim/tests/test_factory_layout_contract.py`, `isaac_sim/scripts/factory_layout.py`

- [x] Write a failing dependency-free test requiring `ARENA_SIZE == (10.0, 8.0)`, main aisle at least `1.2`, service aisle at least `0.8`, three navigation goals, unique paths, positive dimensions, in-bounds geometry, and an unoccupied spawn footprint.
- [x] Require category minimums: 6 columns, 4 cabinets, 3 tanks, 2 pumps, 4 pipe racks, 12 vertical pipes, 12 horizontal pipes, perimeter and partition walls, and guardrails.
- [x] Run `/usr/bin/python3 -m unittest isaac_sim/tests/test_factory_layout_contract.py -v`; expected failure is missing `factory_layout.py`.
- [x] Implement immutable constants, `ROBOT_SPAWN = (-3.8, -2.8, 0.242, 0.0)`, three reachable goal poses, explicit `CLEARANCE_ZONES`, and primitive dictionaries containing path, category, shape, pose, dimensions, color, and collision.
- [x] Use boxes for walls, cabinets, pumps, racks, and rails; use cylinders for columns, tanks, and pipes; encode horizontal-cylinder rotations explicitly.
- [x] Run the layout test again; expected result is all green.

## Task 3: Generate Factory USD Test-First

**Files:** Create `isaac_sim/scripts/generate_factory_environment.py`, `isaac_sim/tests/test_factory_usd_contract.py`; generate `isaac_sim/assets/environments/compact_factory/compact_factory.usd`

- [x] Add failing contracts requiring `Usd.Stage.CreateNew`, Z-up, 1 meter/unit, default prim `/Factory`, a ground slab, hidden spawn/goal Xforms, and `UsdPhysics.CollisionAPI` on every collision primitive.
- [x] Run the static tests and confirm failure because the generator and asset are absent.
- [x] Implement `create_material`, `add_box`, `add_cylinder`, and `generate` helpers. Scale cubes in meters, orient cylinders from layout rotations, bind simple industrial materials, and apply collision exactly when requested.
- [x] Generate with `/home/user/anaconda3/envs/env_isaaclab/bin/python isaac_sim/scripts/generate_factory_environment.py`.
- [x] Run `/home/user/anaconda3/envs/env_isaaclab/bin/python isaac_sim/tests/test_factory_usd_contract.py -v`; require correct metadata, paths, counts, and collision schemas.

## Task 4: Add Isaac Odometry And Dynamic Base TF

**Files:** Modify `isaac_sim/tests/test_import_contract.py`, `isaac_sim/scripts/import_mobile_manipulator.py`; regenerate `mobile_manipulator_ros.usd`

- [x] Add failing USD contracts for `ComputeOdometry`, `PublishOdometry`, and `PublishOdomTF` in `/ActionGraph`.
- [x] Require `/odom` with `odomFrameId=odom`, `chassisFrameId=base_link`, and raw TF with `parentFrameId=odom`, `childFrameId=base_link`, `topicName=tf`, `staticPublisher=False`.
- [x] Require the compute node to target the unique articulation root and require tick, simulation-time, position, orientation, and velocity connections matching NVIDIA's installed differential-base example.
- [x] Run the Isaac import contracts red; existing robot contracts must remain green.
- [x] Add `isaacsim.core.nodes.IsaacComputeOdometry`, `isaacsim.ros2.bridge.ROS2PublishOdometry`, and `isaacsim.ros2.bridge.ROS2PublishRawTransformTree` to the generated graph with the explicit frames above.
- [x] Regenerate the robot assets and rerun the Isaac import contracts; expected result is all green.

## Task 5: Compose Factory And Publish RTX Scan

**Files:** Create `isaac_sim/scripts/run_factory_navigation.py`; extend `isaac_sim/tests/test_navigation_contract.py`

- [x] Write failing CLI contracts for `--factory-usd`, `--robot-usd`, `--headless`, `--duration`, and `--disable-lidar`.
- [x] Require `/Factory`, `/mobile_manipulator/livox_frame/NavLidar`, `Example_Rotary_2D`, `ROS2RtxLidarHelper`, topic `scan`, frame `livox_frame`, type `laser_scan`, and simulation timestamps.
- [x] Run the contract red because the runner does not exist.
- [x] Implement `SimulationApp` startup, enable ROS 2 and RTX sensor extensions, open the robot ROS USD, reference the factory default prim at `/Factory`, and place the robot from `ROBOT_SPAWN` without changing articulation internals.
- [x] Create `LidarRtx` under `livox_frame` at identity local pose with `Example_Rotary_2D` and useful range about `0.15` to `12.0` m. Feed its render-product path into a playback-tick-triggered `ROS2RtxLidarHelper` configured for `/scan` and `livox_frame`.
- [x] Run navigation and existing runner contracts green.

## Task 6: Create Navigation Package Test-First

**Files:** Create `mobile_manipulator_navigation/CMakeLists.txt`, `package.xml`, `config/slam_toolbox.yaml`, `config/nav2_params.yaml`, `config/navigation.rviz`, `launch/mapping.launch.py`; extend `test_navigation_contract.py`

- [x] Add failing manifest/YAML contracts requiring `ament_cmake`, `mobile_manipulator_description`, `nav2_bringup`, `robot_state_publisher`, `rviz2`, `slam_toolbox`, and `xacro`.
- [x] Require consistent `map`, `odom`, `base_link`, `/scan`, `/odom`, and `use_sim_time=true` settings. Require obstacle and inflation layers in both costmaps, a conservative mobile-base polygon footprint, and maximum speeds no greater than `0.35 m/s` linear and `0.8 rad/s` angular.
- [x] Run contracts red because the package is absent.
- [x] Create package metadata and install `config` and `launch`.
- [x] Configure asynchronous SLAM Toolbox with 0.05 m resolution, 0.02 s transform publication, 2 s map updates, and 0.1 m/rad minimum motion thresholds.
- [x] Base Nav2 parameters on installed Humble bringup defaults. Use NavFn, DWB, BT Navigator, behavior server, waypoint follower, velocity smoother, global costmap in `map`, and rolling local costmap in `odom`.
- [x] Configure RViz fixed frame `map` with RobotModel, TF, `/map`, `/scan`, costmaps, plans, and Nav2 Goal.
- [x] Create `mapping.launch.py` that publishes the unified robot description, includes SLAM Toolbox online async launch and Nav2 `navigation_launch.py`, and conditionally starts RViz. It must not start AMCL, map_server, MoveIt, a joint-state GUI, or another odometry publisher.
- [x] Build description plus navigation packages, rerun contracts, and load `ros2 launch mobile_manipulator_navigation mapping.launch.py --show-args` with `ROS_LOG_DIR=/tmp/ros-log`.

## Task 7: Add One-Command Navigation Launcher

**Files:** Create `start_navigation_test.sh`; extend `test_navigation_contract.py`

- [x] Add a failing contract requiring executable status, `--dry-run`, Fast DDS for Isaac, Cyclone DDS for ROS, `run_factory_navigation.py`, `mapping.launch.py`, separate `setsid` process groups, and `trap cleanup INT TERM EXIT`.
- [x] Run red because the launcher is absent.
- [x] Adapt the verified `start_moveit_test.sh` supervisor: validate factory/robot assets and setup files, start Isaac first, wait a configurable bounded interval, start navigation/RViz second, and stop both groups on `Ctrl+C` or child exit.
- [x] Run `chmod +x`, `bash -n`, `./start_navigation_test.sh --dry-run`, and the navigation contracts; require no processes during dry run and all tests green.

## Task 8: GPU Sensor And Odometry Integration

**Files:** Modify `isaac_sim/tests/test_factory_usd_contract.py`, `progress.md`, `task_plan.md`

- [ ] Start a 120-second headless factory run with Isaac's Humble ROS library path and Fast DDS.
- [ ] Use a bounded Cyclone diagnostic node if Fast DDS CLI participants hang. Require nonempty `/scan`, advancing simulation timestamps, `/odom` with `frame_id=odom` and `child_frame_id=base_link`, and connected `odom -> base_link -> frame_base -> livox_frame`.
- [ ] Publish `0.2 m/s` for two seconds, stop, and require odometry forward displacement over `0.2 m` with bounded lateral drift. Confirm scan values change and the articulation remains upright.
- [ ] Encode only stable USD/graph facts in GPU contracts; do not encode DDS timing.

## Task 9: Live SLAM And Three Nav2 Goals

**Files:** Modify `CODEX_TASKS.md`, `progress.md`, `task_plan.md`

- [ ] Start `./start_navigation_test.sh`; require the factory and robot in Isaac, visible `/scan` in RViz, fixed frame `map`, and a growing map around spawn.
- [ ] Drive conservatively through the open bay and both corridor exits using teleop; require map dimensions and known-cell count to grow without TF breaks.
- [ ] Send the three authored goal poses through `NavigateToPose`. Require success, no collision, final position error below 0.20 m, and yaw error below 0.20 rad; one route must use the 0.8 m service corridor.
- [ ] Run lightweight factory/navigation/description/runner/MoveIt tests, then Isaac factory USD and robot import contracts.
- [ ] Record dependency versions, asset paths, commands, frame ownership, mapping metrics, goal results, limitations, and deferred FAST-LIO2 work in project memory.

## Final Verification Commands

```bash
/usr/bin/python3 -m unittest isaac_sim/tests/test_factory_layout_contract.py isaac_sim/tests/test_navigation_contract.py isaac_sim/tests/test_description_contract.py isaac_sim/tests/test_run_contract.py isaac_sim/tests/test_moveit_config_contract.py -v
/home/user/anaconda3/envs/env_isaaclab/bin/python isaac_sim/tests/test_factory_usd_contract.py -v
/home/user/anaconda3/envs/env_isaaclab/bin/python isaac_sim/tests/test_import_contract.py -v
```

Expected: all new and existing tests pass with zero failures, both original URDF hashes remain unchanged, and no Isaac/ROS/RViz processes remain after cleanup.
