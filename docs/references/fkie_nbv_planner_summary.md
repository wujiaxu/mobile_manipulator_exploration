# FKIE NBV Planner Reference Summary

Last checked: 2026-07-03

Sources:

- Paper/arXiv: https://arxiv.org/abs/2203.10113
- Paper DOI: https://doi.org/10.1109/LRA.2022.3146558
- Repository: https://github.com/fkie/fkie-nbv-planner

## Paper

Title:

- Online Next-Best-View Planner for 3D-Exploration and Inspection With a Mobile Manipulator Robot

Authors:

- Menaka Naazare
- Francisco Garcia Rosas
- Dirk Schulz

Publication:

- IEEE Robotics and Automation Letters, 2022
- arXiv:2203.10113
- DOI: 10.1109/LRA.2022.3146558

Core idea:

- The paper proposes an online next-best-view planner for a mobile manipulator
  with an RGB-D camera mounted on the arm.
- The planner supports:
  - full 3D exploration,
  - user-oriented exploration,
  - inspection of regions of interest.
- It formulates exploration plus inspection as a multi-objective optimization
  problem.
- It uses a weighted-sum information gain function to score candidate views.
- The target sensor is the manipulator-mounted RGB-D camera, not only a base
  sensor.

Important implication for this project:

- We need a map representation that preserves occupancy probability, not only a
  binary occupied/free map.
- The authoritative 3D map for NBV should therefore be a full probabilistic
  OctoMap.
- The arm camera pose must be available in the global/world planning frame.
- The mobile-base footprint must be available so sampled base poses can be
  rejected when invalid.

## Repository

Repository:

- `fkie/fkie-nbv-planner`

Package:

- `fkie_nbv_planner`

Original environment:

- Ubuntu 20.04
- ROS Noetic
- Catkin

This means direct use in our ROS 2 Humble workspace requires either:

- a ROS 2 port/adaptation branch, or
- a bridge/wrapper around the original ROS 1 package.

The repo README says `run_planner.launch` starts the planner. The planner is an
action server. Requests include an exploration boundary and measured readings of
the region of interest.

## Required Input Topics From Repo README

The README lists these required planner inputs:

```text
octomap_full
octomap_msgs/Octomap
```

```text
/realsense/depth/points2
sensor_msgs/PointCloud2
```

```text
camera_pose
geometry_msgs/PoseStamped
```

```text
${robot_ns}_mbf/global_costmap/footprint
geometry_msgs/PolygonStamped
```

Notes:

- The original stack uses `octomap_server` to subscribe to `/clock`, `/tf`,
  `/tf_static`, and `/realsense/depth/points2`, then publish `octomap_full`.
- The original depth cloud comes from `depth_image_proc` using depth image plus
  camera info.
- The original footprint topic comes from Move Base Flex.

## Mapping To This Project

Our current ROS 2 equivalents:

```text
FKIE expected: octomap_full
Current:       /octomap_full
Type:          octomap_msgs/msg/Octomap
Frame:         map
Producer:      mobile_manipulator_navigation/wrist_depth_octomap_node
Status:        implemented
```

```text
FKIE expected: /realsense/depth/points2
Current:       /realsense/depth/points2
Type:          sensor_msgs/msg/PointCloud2
Source:        alias of /wrist_camera/depth/points
Status:        implemented
```

```text
FKIE expected: camera_pose
Current:       /camera_pose
Type:          geometry_msgs/msg/PoseStamped
Frame:         map
Pose source:   TF lookup map -> wrist_camera_color_optical_frame
Status:        implemented
```

```text
FKIE expected: ${robot_ns}_mbf/global_costmap/footprint
Current:       /mobile_manipulator_mbf/global_costmap/footprint
Type:          geometry_msgs/msg/PolygonStamped
Frame:         map
Source:        Nav2 footprint transformed from base_link to map
Status:        implemented
```

## Current Project Components

`mobile_manipulator_navigation/src/wrist_depth_octomap_node.cpp`

- Subscribes:
  - `/wrist_camera/depth/points`
- Publishes:
  - `/octomap_full`
  - `/octomap_binary`
  - `/octomap_occupied_points`
  - `/realsense/depth/points2`
  - `/camera_pose`

`mobile_manipulator_navigation/src/nbv_footprint_publisher_node.cpp`

- Reads TF:
  - `map -> base_link`
- Publishes:
  - `/mobile_manipulator_mbf/global_costmap/footprint`
- Passive only:
  - does not command `/cmd_vel`,
  - does not publish TF,
  - does not modify Isaac,
  - does not modify SLAM,
  - does not modify Nav2 costmap parameters.

## Runtime Checks

Use these after starting the simulation with wrist OctoMap enabled:

```bash
ros2 topic echo /octomap_full --once
ros2 topic hz /realsense/depth/points2
ros2 topic echo /camera_pose --once
ros2 topic echo /mobile_manipulator_mbf/global_costmap/footprint --once
```

Also useful:

```bash
ros2 param get /wrist_depth_octomap full_octomap_topic
ros2 param get /wrist_depth_octomap point_cloud_alias_topic
ros2 param get /nbv_footprint_publisher footprint_topic
ros2 run tf2_ros tf2_echo map wrist_camera_color_optical_frame
ros2 run tf2_ros tf2_echo map base_link
```

## Next Engineering Step

Before implementing a ROS 2 FKIE-compatible branch:

1. Inspect the repo action definition and planner parameters.
2. Decide whether to port the planner to ROS 2 or wrap/bridge the ROS 1 package.
3. Keep the current branch as the stable perception/interface baseline.
4. In the new branch, map FKIE action/service APIs to our Nav2 and MoveIt
   execution stack.

