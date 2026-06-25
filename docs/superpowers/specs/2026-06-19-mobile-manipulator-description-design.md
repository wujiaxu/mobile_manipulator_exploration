# Mobile Manipulator Description Design

## Goal

Create the first ROS 2 Humble description package for a unified Tracer mobile base and xArm7 manipulator without modifying either source model.

## Source Models

- Mobile source: `/home/user/wu_ws/isaac_crowd_navi/isaac_crowd_navi/source/isaac_crowd_navi/assets/tracer_description/urdf/tracer_v1.urdf`
- Manipulator source: `/home/user/wu_ws/isaac_crowd_navi/isaac_crowd_navi/source/isaac_crowd_navi/assets/xarm_isaac/xarm7_clean.urdf`
- Mobile graph root: `base_link`
- Manipulator graph root: `world`; physical root after removing its standalone-world fixture: `link_base`

The calibration frame names `frame_base` and `frame_1` are absent from the source files. The unified model therefore introduces them as explicit links.

## Composition

The package will contain package-local Xacro adaptations and mesh assets. This makes the description installable and avoids depending on incomplete source-package installation rules or absolute arm mesh paths.

The fixed-frame topology is:

```text
base_link
└── frame_base                     identity
    ├── frame_1                    xyz=-0.076 0 0.519, rpy=0 0 0
    │   └── link_base              identity
    │       └── xArm7 chain
    └── livox_frame                xyz=0.149 0 0.444, rpy=0 0 0
```

`frame_base -> frame_1` and `frame_base -> livox_frame` exactly preserve the calibrated transforms in `CODEX_TASKS.md`. The source arm's artificial `world` link and `world_joint` are excluded from the local adaptation.

## Sensors

`livox_frame` is represented as a fixed frame. No uncalibrated LiDAR geometry is invented. A wrist camera frame is not added because the only known information is the likely parent `link_eef`; no calibrated translation, rotation, or optical-frame convention is available.

## Package And Display

`mobile_manipulator_description` will be an `ament_cmake` package with `urdf`, `meshes`, `launch`, and `rviz` directories. `display.launch.py` will expand the unified Xacro, start `robot_state_publisher`, publish editable display joint states with `joint_state_publisher_gui`, and launch RViz2 with a package-local configuration.

## Validation

Validation will cover XML syntax, Xacro expansion, a single-root URDF tree, calibrated joint values, launch-file Python syntax, package build when ROS 2 tooling is available, and confirmation that the original assets remain unchanged.

