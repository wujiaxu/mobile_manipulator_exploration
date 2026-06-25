# Mobile Manipulator Description Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build an installable ROS 2 package containing one calibrated mobile-manipulator robot description and an RViz display launch file.

**Architecture:** Package-local Xacro macros hold adapted copies of the two source link trees and use package-relative mesh URIs. A small top-level Xacro instantiates both macros and owns all integration frames and calibrated fixed joints.

**Tech Stack:** ROS 2 Humble, ament_cmake, Xacro, robot_state_publisher, joint_state_publisher_gui, RViz2

---

### Task 1: Scaffold The Package

**Files:**
- Create: `mobile_manipulator_description/package.xml`
- Create: `mobile_manipulator_description/CMakeLists.txt`

- [x] Add an `ament_cmake` package manifest with runtime dependencies on `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.
- [x] Install the `launch`, `meshes`, `rviz`, and `urdf` directories from CMake.
- [x] Validate the manifest with `xmllint --noout mobile_manipulator_description/package.xml`.

### Task 2: Adapt The Source Models

**Files:**
- Create: `mobile_manipulator_description/urdf/tracer_base.urdf.xacro`
- Create: `mobile_manipulator_description/urdf/xarm7.urdf.xacro`
- Create: `mobile_manipulator_description/meshes/tracer/*`
- Create: `mobile_manipulator_description/meshes/xarm7/*`

- [x] Convert the source model bodies into callable Xacro macros while preserving link, joint, inertial, visual, collision, transmission, and Gazebo content needed by the description.
- [x] Exclude the arm's standalone `world` link and `world_joint`.
- [x] Rewrite every mesh path as `package://mobile_manipulator_description/meshes/...` and copy each referenced source mesh.
- [x] Confirm the two original files have not changed by comparing hashes captured before and after adaptation.

### Task 3: Compose The Unified Robot

**Files:**
- Create: `mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro`

- [x] Include and instantiate both model macros.
- [x] Add `base_link -> frame_base` and `frame_1 -> link_base` identity fixed joints.
- [x] Add calibrated `frame_base -> frame_1` and `frame_base -> livox_frame` fixed joints.
- [x] Expand with `xacro` and verify that every child link has one parent and `base_link` is the only root.

### Task 4: Add Display Support

**Files:**
- Create: `mobile_manipulator_description/launch/display.launch.py`
- Create: `mobile_manipulator_description/rviz/display.rviz`

- [x] Resolve the package-share Xacro and RViz paths in the launch description.
- [x] Start `joint_state_publisher_gui`, `robot_state_publisher`, and RViz2.
- [x] Compile-check the launch file with `python3 -m py_compile` and validate the RViz YAML structure by launching when ROS 2 is available.

### Task 5: Verify And Document

**Files:**
- Modify: `CODEX_TASKS.md`

- [x] Build with `colcon build --packages-select mobile_manipulator_description` when the ROS 2 environment is available.
- [x] Record created files, validation results, the missing wrist-camera calibration, and exact build, launch, and TF inspection commands in `CODEX_TASKS.md`.

