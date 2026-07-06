# Task Plan

## Goal

Add MoveIt 2 end-effector pose control for the unified mobile manipulator in Isaac Sim 4.5.

## Current Milestone: FKIE NBV Candidate Execution Through MoveIt

- [x] Create ROS 2 FKIE-style action messages and a native `/nbv_rrt` action server.
- [x] Feed the planner with `/camera_pose`, `/octomap_full`, and the passive map-frame footprint.
- [x] Fix ROS 2 action transport by flattening the boundary polygon into `boundary_x`/`boundary_y` arrays.
- [x] Produce a non-empty OctoMap gain-scored NBV candidate from a live Isaac map.
- [x] Add a one-shot adapter that requests one NBV candidate and publishes a converted `link_eef` target to `/arm_target_pose`.
- [ ] Replace the current simplified candidate sampler with a ROS 2 port of the original FKIE RRT planner: RRT nodes, KD-tree nearest-neighbor expansion, gain cubature, cached frontiers, parent-chain branch extraction, and original `goals`/`request_base_pose` result semantics.
- [ ] Keep execution adapters thin: if planner returns `goals`, send arm goals; if planner returns `request_base_pose`, send a Nav2 goal from `goal_pose_3d`.
- [ ] Live-test the original-algorithm ROS 2 planner with Isaac, MoveIt, OctoMap, Nav2, and RViz running.

## Previous Milestone: Map-Frame Wrist OctoMap for Exploration

- [x] Inspect current Isaac ROS topics, installed Nav2 packages, and sensor extensions.
- [x] Select online SLAM Toolbox with a temporary 360-degree 2D LiDAR.
- [x] Write and approve the Nav2 prerequisite design.
- [ ] Implement odometry and range sensing in Isaac Sim (USD contracts and runner complete; live topic verification pending).
- [x] Add and validate Nav2 configuration.
- [x] Add wrist depth image to `PointCloud2` conversion.
- [x] Add a project-owned OctoMap node that integrates wrist depth clouds in `map`.
- [x] Add RViz displays for wrist depth image, z-colored depth points, and z-colored occupied OctoMap voxels.
- [x] Publish full probabilistic `/octomap_full`, `/camera_pose`, and `/realsense/depth/points2` compatibility outputs for the future FKIE NBV planner branch.
- [x] Publish passive FKIE-style `/mobile_manipulator_mbf/global_costmap/footprint` in `map`.
- [x] Add MoveIt PlanningScene collision boxes from cropped `/octomap_occupied_points` so arm planning can avoid observed occupied voxels.
- [ ] Live-verify `/octomap_binary` while Isaac, SLAM Toolbox, and wrist depth topics are running.

## Phases

- [x] Inspect the current description, command interfaces, and host MoveIt installation.
- [x] Approve planned-trajectory pose control through RViz and `/arm_target_pose`.
- [x] Write and self-review the MoveIt pose-control design.
- [x] Receive written-spec approval.
- [x] Write the test-first implementation plan.
- [x] Implement MoveIt configuration and Isaac trajectory bridge.
- [ ] Verify RViz and topic pose goals in Isaac Sim (topic execution and RViz connection verified; manual RViz execution remains).
- [ ] Add MoveIt Servo in a follow-up milestone.

## Constraints

- Do not modify either original robot file.
- Preserve the calibrated transforms exactly.
- Treat the wrist-camera extrinsic as a placeholder until measured/calibrated.
- Exploration 3D occupancy must use fixed frame `map`, not `base_link`.

## Errors Encountered

- The inventory path for `using-superpowers` did not exist; the installed path under `~/.codex/superpowers/skills` was used.
- `.git` is an empty read-only directory, so repository status, worktrees, and commits are unavailable.
- The first `colcon build` selected `/home/user/anaconda3/bin/python3`, which lacks `catkin_pkg`. `/usr/bin/python3` has the ROS dependency; rebuild with a clean CMake cache and `Python3_EXECUTABLE=/usr/bin/python3`.
- Xacro and joint-state GUI packages are absent from the host. Xacro was temporarily extracted under `/tmp` for validation; declared dependencies and installation commands are recorded in `CODEX_TASKS.md`.
- ROS launch initially tried to write under read-only `~/.ros`; setting `ROS_LOG_DIR=/tmp/ros-log` allowed sandboxed launch-description validation.
- The first import-URDF summary assertion used conflicting quotes inside an inline f-string. URDF generation and prior tests passed; rerun the summary with precomputed counts and non-conflicting quoting.
- Sandboxed Isaac startup could not access Vulkan/CUDA and exited with code 139; GPU execution must run with approved host access.
- The first GPU import created the USD but found `ArticulationRootAPI` below the robot default prim rather than on `/mobile_manipulator`; discover and target the unique articulation root below the robot.
- Isaac's bundled ROS 2 bridge requires its `humble/lib` directory in `LD_LIBRARY_PATH` before startup.
- A fresh shell still lacks `ros-humble-xacro`; use the generated plain URDF for live verification or install the declared dependency before using `isaac_state.launch.py`.
- The first final support-test attempts omitted the ROS and workspace setup, then confirmed the remaining failure is specifically the absent `xacro` executable. The generated URDF and Isaac USD contract checks passed independently.
- The ROS overlay initially launched the articulation rapidly along `-Y`, even with both controllers or the complete action graph inactive. Its root layer had fallen back to `Y`-up and `0.01` meters/unit instead of the base asset's `Z`-up and `1.0` meters/unit. The importer now authors both metrics explicitly; the regenerated active-graph robot remains stationary.
- A direct `DifferentialController.outputs:execOut` ordering edge failed because that Isaac 4.5 node has no execution output, leaving the newly created overlay without a saved action graph. NVIDIA's bundled ROS2 differential-base test uses playback tick for both the differential and articulation controllers; generation was changed to that supported pattern.
- The first wheel-axis patch matched the identical right-wheel line instead of the intended left-wheel line. The new axis regression failed on the right joint; both joint blocks were then patched with explicit joint-name context.
- After repeated ROS CLI integration runs, `ros2 doctor`, `ros2 daemon stop`, direct daemon-free topic publication, and even host process enumeration began hanging. Deterministic USD and physics tests still run; one final post-axis ROS pulse requires a fresh host ROS process state.
- The first trajectory bridge build used `rclcpp::Duration::to_msg()`, which is unavailable in ROS 2 Humble. Duration feedback now converts nanoseconds explicitly into the message fields.
- The first bridge install contract assumed `isaac_trajectory_bridge` was the first CMake install target. The executable installed correctly; the test now checks membership anywhere in the `install(TARGETS ...)` block.
- The bridge initially installed its node under generic `bin`, so `ros2 run` reported `No executable found`. ROS executables now install under `lib/${PROJECT_NAME}`, and the contract asserts that runtime destination.
- The trajectory bridge host-network smoke test remains blocked by the previously wedged ROS middleware state: normal and daemon-free CLI discovery both hang. The executable itself builds, installs, and starts; interface discovery is deferred until the required host restart before live integration.
- The first pose-planner link mixed Anaconda Boost/fmt with system CURL/OpenSSL because the active Conda environment influenced CMake. A clean configure with Conda variables removed, system-only PATH, `/usr/bin/cc`, `/usr/bin/c++`, and `/usr/bin/python3` eliminated all Anaconda paths from the bridge cache.
- New Fast DDS CLI participants remained unreliable on this host. Cyclone DDS diagnostic publishers/subscribers interoperated with the Fast DDS Isaac and MoveIt processes and were used for bounded live checks.
- A 20 mm Cartesian target selected a distant valid IK branch and took about 22 seconds to execute. The final pose was accurate, but joint-space continuity constraints should be added before routine operation.
- On forced SIGINT teardown after the live test, MoveIt Humble `move_group` segfaulted inside class-loader cleanup. Planning and execution had already completed successfully; normal runtime behavior was unaffected.
- The first synthetic cancellation goal used `joint4=-0.2`, below its URDF lower limit `-0.19198`, and was correctly rejected. The corrected in-limit goal canceled successfully and held within `0.0033 rad` of measured state.
- A diagnostic `xacro | check_urdf | head` pipeline caused Xacro to report `BrokenPipeError` when `head` closed stdout. The frame definition was already obtained directly from source; this is not a URDF failure.
- The first Nav2 implementation-plan patch was rejected because a multiline `dpkg-query` line lacked an `apply_patch` addition prefix. No plan file was created; retry with simpler fenced commands.
- The isolated Isaac unittest invocation stalled for more than four minutes during startup/output buffering and was terminated. Diagnose the authored USD directly with PXR instead of repeating the same full-app test.
- The first live factory run reached Isaac's `Simulation App Startup Complete`, but the already stale host ROS/process state wedged direct Cyclone DDS probes, `pgrep`, and even external timeout teardown. The deterministic factory and robot USD suites still pass; retry live sensor/odometry validation after a clean host restart.
- The factory runner initially passed a graph descriptor dictionary to `Controller.edit`, which always creates a graph and failed because `/ActionGraph` came from the robot USD. After switching to the existing graph handle, the pre-existing tick node also required its absolute attribute path. Both issues are covered by a regression contract and a live bounded Isaac run.
- The installed MoveIt occupancy map monitor package provides `moveit_ros_occupancy_map_server`, but this host does not expose the normal point-cloud updater plugin XML. A project-owned OctoMap node is used for the exploration map instead of depending on MoveIt-only map monitoring.
- The host does not have `octomap_rviz_plugins`, so RViz cannot natively display `octomap_msgs/msg/Octomap`. The project publishes `/octomap_occupied_points` as an RViz-compatible occupied-voxel `PointCloud2` visualization topic.
- The first FKIE NBV arm execution reached MoveIt through `/arm_target_pose`, but OMPL reported `Unable to sample any valid states for goal tree`. This confirmed action transport and execution plumbing, but the project direction is now to reproduce the original FKIE RRT algorithm first instead of adding workaround target filtering around the simplified sampler.
