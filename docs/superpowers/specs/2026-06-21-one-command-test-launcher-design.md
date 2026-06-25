# One-Command Test Launcher Design

## Goal

Provide one foreground Bash command that starts the existing Isaac Sim robot,
MoveIt planning stack, and RViz MotionPlanning interface for manual testing.

## Interface

The user runs an executable script from anywhere. The script resolves the
workspace from its own location, validates required executables and setup files,
then starts the test. The terminal remains attached until the user presses
`Ctrl+C` or a required process exits.

## Process Model

The script starts Isaac Sim as a background process with Fast DDS and the Isaac
ROS 2 bridge library path. It waits for a bounded startup period, checking that
Isaac remains alive. It then starts the existing
`mobile_manipulator_moveit_config/moveit_isaac.launch.py` launch in the foreground
with RViz enabled and Cyclone DDS, which is the reliable host-side middleware on
this machine.

Isaac and the ROS launch are placed in separate process groups. A Bash trap for
`INT`, `TERM`, and `EXIT` sends `SIGINT` to both groups, waits briefly, and uses
`SIGTERM` only for a process group that did not exit. Cleanup is idempotent so an
early startup failure follows the same path.

## Configuration

The launcher uses the established paths and settings:

- Workspace: derived from the script location
- ROS distribution: `/opt/ros/humble`
- Isaac Python: `/home/user/anaconda3/envs/env_isaaclab/bin/python`
- Isaac runner: `isaac_sim/scripts/run_mobile_manipulator.py`
- Isaac middleware: `rmw_fastrtps_cpp`
- MoveIt middleware: `rmw_cyclonedds_cpp`
- MoveIt launch: `moveit_isaac.launch.py use_rviz:=true use_sim_time:=true`
- ROS logs: `/tmp/ros-log`

No robot, USD, launch, or package source file is modified.

## Output And Failure Handling

Both child processes inherit the terminal output. The script prints short phase
messages so the user can distinguish Isaac startup, MoveIt/RViz startup, and
shutdown. Missing paths produce a direct error before any process starts.

If Isaac exits during the startup wait, the launcher exits nonzero and does not
start MoveIt. If the ROS launch exits, the launcher cleans up Isaac and returns
the launch exit status. The known MoveIt Humble shutdown-only class-loader crash
may still appear after `Ctrl+C`; cleanup continues and does not leave Isaac
running.

## Verification

A dependency-free shell contract test will inspect syntax and required commands.
It will fail before the launcher exists, then pass after implementation. A dry
run option will validate paths and print the two resolved commands without
starting GPU or GUI processes. Final verification will run `bash -n`, the
contract test, and the dry run. The live Isaac/RViz workflow has already been
validated separately.
