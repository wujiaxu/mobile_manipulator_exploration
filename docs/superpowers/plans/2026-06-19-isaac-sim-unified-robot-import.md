# Isaac Sim Unified Robot Import Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import the validated unified mobile-manipulator URDF into Isaac Sim 4.5, preserve its calibrated frames and support collider, publish articulation state to ROS 2, and verify the resulting USD and ROS topics.

**Architecture:** A generated plain URDF is a disposable import input under `build/isaac_import`. A standalone Isaac Sim script imports and validates a reusable physics asset at `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator.usd`, then creates a ROS overlay USD containing a joint-state and clock Action Graph. ROS `robot_state_publisher` consumes those joint states and owns descendant TF, avoiding duplicate articulation TF publishers.

**Tech Stack:** Isaac Sim 4.5.0, USD/PhysX, Isaac ROS 2 Bridge, ROS 2 Humble, Xacro, Python

---

### Task 1: Generate The Import URDF

**Files:**
- Generate: `build/isaac_import/mobile_manipulator.urdf`

- [ ] Expand `mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro` from the sourced workspace.
- [ ] Run `check_urdf` and the existing support-description regression test.
- [ ] Assert one root link, calibrated transforms, support collision, and package-resolved mesh files.

### Task 2: Import And Validate The Base USD

**Files:**
- Create: `isaac_sim/scripts/import_mobile_manipulator.py`
- Create: `isaac_sim/tests/test_import_contract.py`
- Generate: `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator.usd`

- [ ] Write a failing source contract for movable base, unmerged fixed joints, inertia import, and required USD prim names.
- [ ] Implement the Isaac Sim 4.5 `URDFParseAndImportFile` workflow using `fix_base=False`, `merge_fixed_joints=False`, and `import_inertia_tensor=True`.
- [ ] Validate articulation root, arm/wheel joints, calibration links, support rigid body/collider, movable root, stage units, and physics startup under gravity.

### Task 3: Add ROS State Publishing

**Files:**
- Modify: `isaac_sim/scripts/import_mobile_manipulator.py`
- Generate: `isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd`
- Create: `mobile_manipulator_description/launch/isaac_state.launch.py`

- [ ] Add an Isaac Sim 4.5 Action Graph with playback tick, simulation time, ROS 2 context, `/clock`, and `/joint_states` publisher targeting the imported articulation.
- [ ] Add a ROS launch file that expands the unified Xacro and starts `robot_state_publisher` with `use_sim_time=true`.
- [ ] Keep full articulation TF publishing disabled in Isaac Sim; reserve `odom -> base_link` for the later odometry graph.

### Task 4: End-To-End Verification

**Files:**
- Modify: `CODEX_TASKS.md`

- [ ] Start the ROS state launch and Isaac Sim overlay.
- [ ] Verify `/clock`, `/joint_states`, `/tf`, and `/tf_static`, including `frame_base -> frame_1` and `frame_base -> livox_frame`.
- [ ] Open the ROS overlay USD in Isaac Sim for visual inspection and document exact commands and remaining physics-tuning issues.

