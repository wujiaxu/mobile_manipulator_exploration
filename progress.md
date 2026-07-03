# Progress

## 2026-06-19

- Read `PROJECT_OVERVIEW.md` and `CODEX_TASKS.md`.
- Inspected both source robot models, meshes, package metadata, and available tooling.
- Presented and received approval for the calibrated connection design.
- Wrote the approved design and implementation plan.
- Created the `ament_cmake` package skeleton and runtime dependency manifest.
- Copied referenced meshes and adapted both source model bodies into package-local Xacro macros.
- Removed the copied arm's standalone `world` fixture and replaced absolute mesh paths.
- Added the unified Xacro with explicit calibration aliases and fixed joints.
- Initial build failed because Anaconda Python shadowed the ROS system Python; root cause was confirmed from `CMakeCache.txt` and import checks.
- Rebuilt successfully using `/usr/bin/python3`.
- Expanded the unified Xacro with a temporarily extracted ROS Xacro package; `check_urdf` confirmed `base_link` as the single root and both calibrated transforms.
- Added the display launch file and RViz configuration.
- Parsed the RViz configuration and loaded the installed launch description successfully with sandbox-safe logging.
- Confirmed source-model hashes are unchanged.
- Updated `CODEX_TASKS.md` with changes, remaining issues, and next commands.
- Final verification passed: package build, XML/Xacro expansion, URDF graph, 34 mesh references, launch-description loading, RViz YAML, calibration values, and unchanged source hashes.
- Approved a 300 mm square support that fills the measured chassis-to-arm gap.
- Recorded the support design and implementation plan.
- Added a regression test and observed it fail because the support link and joint were absent.
- Added `arm_support_link` with matching primitive visual/collision geometry, fixed placement, mass, and box inertia.
- Rebuilt the package; all four support-description tests and `check_urdf` passed, including an explicit expanded collision check.
- Updated `CODEX_TASKS.md` with support geometry, inertia, and validation results.
- Relaunched RViz2 with the rebuilt support model.
- Final verification passed: package build, four support tests, Xacro expansion, and `check_urdf`.
- Located Isaac Sim 4.5.0 and inspected its bundled URDF importer and ROS 2 bridge examples.
- Recorded the executable Isaac import and state-publication plan.
- Generated the plain import URDF; `check_urdf` and support regression tests passed before a separate summary command hit a quoting-only syntax error.
- Generated the base Isaac USD on the GPU; diagnosed articulation-root placement and the ROS bridge library-path requirement from the first import run.
- Generated both USD stages; the integration contract passed articulation, joint, frame, support-collider, and ROS graph checks.
- Observed the missing ROS state launch, then added `isaac_state.launch.py` with robot_state_publisher as the sole descendant-TF authority.

## 2026-06-20

- Recovered the previous session and confirmed no Isaac, ROS, or RViz processes were left running.
- Started `mobile_manipulator_ros.usd` in Isaac Sim 4.5 on the host GPU and ran robot_state_publisher against the generated plain URDF.
- Confirmed the live ROS graph contains `/clock`, `/joint_states`, `/tf`, `/tf_static`, and `/robot_description`.
- Sampled all 17 expected movable joints from Isaac Sim with simulation timestamps.
- Verified both calibrated TFs exactly: `frame_base -> frame_1` and `frame_base -> livox_frame`.
- Re-ran the Isaac asset contract: all 6 tests passed, including articulation, joints, calibrated frames, support mass/collider, and ROS publishers.
- Re-ran `check_urdf`: the generated URDF parsed successfully with `base_link` as its single root.
- The source-level support regression test remains unavailable until `ros-humble-xacro` is installed; no model files were changed in this verification session.
- Approved standard ROS2 topic control: `/cmd_vel` for the base and `/arm_joint_commands` for the arm.
- Wrote and self-reviewed the Isaac ROS2 control design specification.
- Confirmed the main wheel mesh radius is 0.0605 m and drive-wheel separation is 0.34 m.
- Wrote the test-first Isaac ROS2 control implementation plan; production code has not been changed yet.
- Added generated-USD contracts for `/cmd_vel` differential drive and `/arm_joint_commands` arm position control.
- Generated ROS2 Twist and JointState subscriber branches with message-gated articulation controllers.
- Added PhysX root-pose telemetry and diagnostic switches for disabling controllers or the complete action graph.
- Diagnosed severe ROS-overlay physics instability by comparing the stable base USD against progressively disabled graph configurations.
- Confirmed the overlay root had incorrect defaults (`Y`-up, `0.01` meters/unit), added an explicit metadata regression test, and fixed the importer to preserve `Z`-up meter-scale metadata.
- Regenerated both assets; all 9 generated-USD contracts passed and a 2.5-second run with the normal active graph remained stationary at the expected base height.

## 2026-06-21

- Resumed the control milestone and synchronized stale planning files from the prior session.
- Confirmed both ROS command subscriptions and the joint-state publisher on the live graph.
- Diagnosed a one-message base delay caused by parallel subscriber-triggered differential and articulation controllers; adopted NVIDIA's supported tick-driven articulation pattern.
- Verified arm position control reached `joint1=0.1995` and `joint2=-0.1988`.
- Diagnosed imported wheel drives fighting velocity commands with position stiffness; added a generated-USD gain contract and scoped wheel drive overrides.
- Verified a `0.2 m/s` command yields `left=3.2915` and `right=3.3099 rad/s`, matching the expected `3.3058 rad/s`.
- Diagnosed opposite robot-frame wheel axes from the copied left joint's pi roll; corrected the package-local left axis and added source/generated URDF regressions without modifying the original model.
- Regenerated the base and ROS USD assets. Final checks passed: 10 lightweight tests and all 10 generated-USD contracts.
- Final post-axis ROS motion verification remains pending because the host ROS CLI/process table became wedged; daemon-free Fast DDS publication also hung while Isaac itself remained stable.
- Selected MoveIt planned trajectories as the first end-effector pose-control milestone, with both RViz MotionPlanning and `/arm_target_pose` interfaces; MoveIt Servo follows on the same configuration.
- Confirmed the host has no MoveIt 2 packages installed and still lacks Xacro.
- Wrote and self-reviewed `docs/superpowers/specs/2026-06-21-moveit-pose-control-design.md`; no placeholders or interface inconsistencies remain. Git commit is unavailable because the workspace is not a repository.
- Received approval for the written MoveIt pose-control specification.
- Wrote and self-reviewed the test-first implementation plan at `docs/superpowers/plans/2026-06-21-moveit-pose-control.md`, covering dependency installation, MoveIt configuration, trajectory validation, action bridging, pose planning, launch integration, and live Isaac verification.
- Installed and verified ROS Humble MoveIt 2 `2.5.9`, MoveIt Servo `2.5.9`, and Xacro `2.1.1`; all required MoveIt package prefixes resolve under `/opt/ros/humble`.
- Added seven failing MoveIt configuration contracts, then created `mobile_manipulator_moveit_config` with SRDF, KDL, OMPL, controller, joint-limit, initial-position, Xacro-wrapper, and RViz configuration.
- Verified all seven configuration contracts, rebuilt the description package, expanded the MoveIt wrapper with Xacro, and parsed the unified robot successfully with `base_link` as root and `link_eef` as the arm tip.
- Created the C++ bridge package and implemented exact xArm7 trajectory validation, canonical joint reordering, URDF limit checks, strict timing checks, and clamped linear interpolation. All four GoogleTest cases pass through colcon.
- Implemented the `FollowJointTrajectory` bridge with 100 Hz interpolation, measured feedback, cancellation hold, and path/goal tolerance results.
- Implemented `/arm_target_pose` planning with validation, quaternion normalization, bounded TF transformation, MoveGroup planning, and execution for `link_eef`.
- Removed Anaconda native libraries from the bridge CMake cache and verified both ROS executables are indexed; ten MoveIt configuration/bridge contracts pass.
- Added and validated `moveit_isaac.launch.py`. A bounded runtime launch loaded the unified model, KDL, OMPL, the xArm7 controller, MoveGroup capabilities, trajectory bridge, and pose planner; MoveGroup reported ready for planning.
- Completed live Isaac Sim pose-control integration through `/arm_target_pose`. A target at `[0.12920498, 0.00005203, 0.65890382]` in `base_link` planned and executed successfully through MoveIt and `/arm_controller/follow_joint_trajectory`.
- Measured final `base_link -> link_eef` translation `[0.129067652, 0.000393706, 0.658323246]`, giving approximately `0.00069 m` position error; quaternion error was also below the `0.05 rad` acceptance threshold.
- Verified an unreachable target at `[5, 0, 5]` fails planning after five seconds and leaves all seven arm joints unchanged.
- Final regressions passed: 21 Python contract tests, 4 trajectory sampler GoogleTests, 10 Isaac generated-USD tests, clean builds of the description/bridge/config packages, and unchanged source-model SHA-256 hashes.
- At that checkpoint, RViz MotionPlanning execution and action cancellation/hold behavior remained. Forced SIGINT shutdown also exposed a MoveIt Humble class-loader cleanup segfault after successful operation.
- Relaunched the integrated stack with RViz2 using Cyclone DDS on the host. RViz loaded the unified robot, connected its interactive markers, and initialized MotionPlanning group `xarm7`.
- Verified action cancellation with a 10-second in-limit trajectory canceled after two seconds. The action reported canceled (`status=5`), and the measured-position hold command differed from the next joint-state sample by at most `0.0033 rad`.
- Only the user-driven Plan/Execute click in RViz remains; Isaac Sim and RViz were left running for that check.
- Added executable `start_moveit_test.sh` as a foreground supervisor for Isaac Sim, MoveIt, and RViz. It validates paths, supports `--dry-run`, checks Isaac startup, and cleans both process groups on `Ctrl+C` or child exit.
- Added three launcher contracts and verified the required commands, executable bit, middleware split, cleanup trap, and dry-run behavior.
- Started the Nav2 prerequisite milestone and confirmed Nav2/SLAM packages, odometry, and range-sensor ROS outputs are absent.
- Reviewed two supplied factory reference slides and selected a 10 m by 8 m procedural arena with 1.2 m main aisles, 0.8 m service corridors, and dense mixed-height pipework.
- Approved online SLAM Toolbox, Isaac odometry, and a temporary 360-degree RTX `/scan` sensor at the calibrated `livox_frame`; FAST-LIO2 is deferred.
- Wrote the approved design at `docs/superpowers/specs/2026-06-22-nav2-factory-slam-design.md`.
- Clarified that Nav2 uses `base_link`; Isaac owns `odom -> base_link`, and robot_state_publisher preserves the existing identity `base_link -> frame_base` calibration alias.
- Wrote and self-reviewed the test-first implementation plan at `docs/superpowers/plans/2026-06-22-nav2-factory-slam.md`, covering dependencies, factory generation, odometry, RTX scan, Nav2/SLAM configuration, launch supervision, and live goals.
- Installed ROS Humble Navigation2/Nav2 Bringup `1.1.20`, SLAM Toolbox `2.6.10`, and verified Teleop Twist Keyboard `2.4.1`; all required package prefixes resolve under `/opt/ros/humble`.
- Added a deterministic 10 m by 8 m factory layout with explicit route clearances, three goal poses, and the approved minimum counts for walls, columns, equipment, racks, pipes, and guardrails. All four dependency-free layout contracts pass.
- Generated `compact_factory.usd` with Z-up meter-scale metadata, industrial materials, navigation markers, and collision-enabled primitive geometry. All four factory generator/USD contracts pass under Isaac Python.
- Extended the generated robot ROS graph with Isaac odometry, `/odom`, and one dynamic `odom -> base_link` raw TF publisher. All eleven robot USD contracts pass; target-input validation was corrected to inspect USD relationships rather than attributes.
- Added `run_factory_navigation.py` to compose factory and robot in a session layer, place the robot at the authored spawn, and publish an `Example_Rotary_2D` RTX scan from `livox_frame`. All runner contracts pass.
- Created `mobile_manipulator_navigation` with SLAM Toolbox, conservative DWB/Nav2, scan costmaps, map-frame RViz, and mapping launch orchestration. Eight package/runner contracts pass; both description and navigation packages build and launch arguments load.
- Added executable `start_navigation_test.sh` as a foreground supervisor for the factory Isaac runner and SLAM/Nav2/RViz. It validates both USD assets, preserves the Fast DDS/Cyclone DDS split, supports `--dry-run`, and cleans both process groups on exit; all eleven navigation contracts pass.
- Attempted bounded live factory integration. Isaac's Kit log reached `Simulation App Startup Complete` without a GPU/CUDA error, but the pre-existing host ROS/process state caused direct Cyclone DDS probes, process enumeration, and bounded teardown to hang before any live topic sample could be accepted.
- Final deterministic checks passed: 36 lightweight contracts, 4 factory USD contracts, and 11 generated robot USD contracts. Live `/scan`, `/odom`, TF, mapping, and navigation-goal acceptance remain the next clean-session checkpoint.

## 2026-06-23

- Fixed the factory runner's LiDAR graph integration. Passing a graph descriptor dictionary to `Controller.edit` always attempted to create `/ActionGraph` even though the imported robot already owned it; the runner now resolves and edits the existing graph handle.
- Fixed the existing playback-tick connection to use the absolute `/ActionGraph/OnPlaybackTick.outputs:tick` path. Relative lookup only covers nodes created in the same edit operation.
- Added a regression contract for both conditions. All twelve navigation contracts pass, and a live five-second Isaac run with RTX LiDAR enabled reached the publication loop and exited normally.
- Diagnosed the Isaac floor-intersection/tilt issue as a spawn-height mismatch: the imported chassis collision mesh extends to about `-0.285 m`, while the previous Isaac spawn used `z=0.242`.
- Raised the shared Isaac robot spawn clearance to `0.305 m` and reused it in both the factory and standalone robot runners. The factory layout contract now guards against spawn heights below `0.30 m`.
- Follow-up diagnosis found the runners were lifting the parent `/mobile_manipulator` namespace prim, not the PhysX articulation root body. Updated both standalone and factory runners to place `/mobile_manipulator/base_link` at the clearance height directly, with tests guarding against parent-only spawning.
- Regenerated `compact_factory.usd` after lowering all horizontal pipes to the `0.7 m` to `1.4 m` range. Added source and generated-USD regressions for pipe height and ground collision.
- Fixed `run_factory_navigation.py` to compose a new session stage with the factory USD loaded before the robot USD, instead of opening the robot USD over the factory stage. The runner now uses explicit `Sdf.Path(...)` lookups and places `/mobile_manipulator/base_link` at the configured spawn Z.
- Added factory scene lighting to the generated USD: one ambient dome light and two overhead rectangular lights. Regenerated `compact_factory.usd` and added generated-USD coverage for the light prims.
- Rechecked the factory runner for the remaining sinking report and restored the correct articulation-root spawn path: `/mobile_manipulator` now gets only X/Y/yaw, `/mobile_manipulator/base_link` gets the spawn Z, and the PhysX root body is reset to the same pose with zero velocity after playback starts.
- Re-ran the runner and navigation contracts; all 22 checks pass. A live Isaac run from this Codex environment could not reach scene loading because Kit failed during GPU/display initialization, so the next required check is visual confirmation in the user's Isaac GUI session.
- Debugged the missing RViz `map` frame. The first failed navigation launch had `controller_server` crash because Nav2 Humble declares local costmap `height`/`width` as integers; changed the YAML values to integer `4`, added a regression check, rebuilt `mobile_manipulator_navigation`, and verified the bounded launch now reaches activation and waits for `odom -> base_link` instead of crashing.
- Confirmed the remaining `map` prerequisite is live synchronized `/scan`, `/odom`, and TF. SLAM Toolbox only publishes `map -> odom` after it can transform scans from `livox_frame` into the odom tree.
- Added executable `start_teleop.sh` for keyboard driving through `teleop_twist_keyboard` on `/cmd_vel`; syntax and dry-run checks pass.
- Removed Slam Toolbox launch ambiguity by changing `mapping.launch.py` to start `slam_toolbox/async_slam_toolbox_node` directly with the project `slam_toolbox.yaml`, which uses `base_frame: base_link`; rebuilt `mobile_manipulator_navigation` and updated the navigation contract to reject fallback to the default `mapper_params_online_async.yaml`.
- Confirmed the live Slam Toolbox process uses `base_link` and `/scan`, while `/scan` runs around 50 Hz and required odom/LiDAR TFs exist. The remaining map-TF blocker is Slam Toolbox dropping every `livox_frame` scan because its message filter queue fills. Tuned SLAM parameters with `scan_queue_size: 200`, `throttle_scans: 5`, `transform_timeout: 1.0`, and rebuilt `mobile_manipulator_navigation`.

## 2026-07-02

- Added a map-frame wrist depth OctoMap pipeline for exploration.
- Created `mobile_manipulator_navigation/src/wrist_depth_octomap_node.cpp`, which subscribes to `/wrist_camera/depth/points`, transforms clouds into `map`, inserts them into an `octomap::OcTree`, and publishes `/octomap_binary`.
- Added `mobile_manipulator_navigation/launch/wrist_octomap.launch.py` to convert `/wrist_camera/depth/image_rect_raw` plus `/wrist_camera/color/camera_info` into `/wrist_camera/depth/points` using `depth_image_proc/point_cloud_xyz_node`, then start the OctoMap node.
- Added `mobile_manipulator_navigation/config/wrist_octomap.yaml` with `map_frame: map`, `resolution: 0.05`, and `max_range: 3.0`.
- Added optional `--use-wrist-octomap` support to `start_navigation_test.sh` and `start_controlled_rooms_mobile_manipulator.sh`.
- Verified the new contracts, package build, launch argument loading, shell syntax, dry-runs, and bounded ROS startup outside the sandbox.
- Updated the baked Isaac wrist camera to `848x480` with USD intrinsics derived from `fx=429`, `fy=427`, `cx=425`, `cy=240`; regenerated both robot USD assets and verified the Isaac import contract.
- Added RViz visualization for the wrist depth image, z-colored wrist depth point cloud, and a z-colored occupied-voxel cloud derived from the map-frame OctoMap.
- Added `/octomap_occupied_points` because this host has `octomap_msgs` but not `octomap_rviz_plugins`; RViz can show the occupied voxels with the default `PointCloud2` display while `/octomap_binary` remains available for algorithms.
- Changed the wrist OctoMap pipeline to publish the probabilistic full map on `/octomap_full` for NBV information-gain work, while keeping `/octomap_binary` and `/octomap_occupied_points` for compatibility and RViz.
- Added FKIE-prep outputs from the same node: `/camera_pose` as `geometry_msgs/msg/PoseStamped` in `map`, and `/realsense/depth/points2` as a compatibility alias for the wrist depth cloud.
- Added a passive FKIE-style footprint publisher on `/mobile_manipulator_mbf/global_costmap/footprint`, transforming the existing Nav2 base footprint from `base_link` into `map` without modifying Isaac, SLAM, Nav2 costmap params, or TF.
- Added a MoveIt PlanningScene bridge that converts `/octomap_occupied_points` into cropped voxel collision boxes in `base_link`, publishes `/planning_scene` diffs, and calls `/apply_planning_scene` when MoveIt is available.
