# Findings

- The mobile source graph root is `base_link`; `frame_base` is absent.
- The manipulator graph root is `world`, fixed at identity to physical root `link_base`; `frame_1` is absent.
- The arm end-effector link is `link_eef`.
- The mobile Xacro expands at top level and is not a reusable macro.
- The arm URDF uses absolute `/home/user/xarm_isaac/meshes/...` paths.
- The source Tracer package does not install its URDF, RViz, or mesh directories.
- Both source URDF files are well-formed XML.
- No wrist-camera extrinsic is recorded, so a camera frame cannot be calibrated yet.
- Isaac Sim 4.5.0 is installed at `/home/user/anaconda3/envs/env_isaaclab` with URDF importer 2.3.10 and ROS 2 bridge 4.1.15.
- Isaac Sim 4.5 publishes joint states directly from `ROS2PublishJointState.inputs:targetPrim`; the 6.0 `Isaac Read Joint State` migration does not apply.
- The bundled Carter importer confirms `fix_base=False` and `merge_fixed_joints=False` for a movable mobile base.
- Live Isaac Sim 4.5 verification publishes `/clock` and 17 joints on `/joint_states`; robot_state_publisher publishes `/tf` and `/tf_static` from the same generated URDF.
- Live `frame_base -> frame_1` is `xyz=-0.076 0 0.519`, identity rotation, and `frame_base -> livox_frame` is `xyz=0.149 0 0.444`, identity rotation.
- The initial `tf2_echo` missing-frame message is a startup race: both transforms become available immediately afterward and remain stable.
- A USD overlay root layer does not automatically preserve the imported base stage's authored up-axis and unit metrics for this workflow. The ROS overlay must explicitly author `Z`-up and `1.0` meters/unit; otherwise PhysX interprets the robot as Y-up at centimeter scale and launches it along `-Y`.
- With corrected overlay metrics and the complete ROS action graph active, the root pose settles at approximately `[0.000903, -0.000012, 0.142517]` and remains unchanged through a 2.5-second physics run.
- Isaac's URDF importer gave both continuous drive wheels position-drive gains (`stiffness=625`, `damping=0`), which held them near zero under velocity commands. The generated USD now overrides only `left_wheel` and `right_wheel` to `stiffness=0`, `damping=10000`.
- With corrected gains, a `0.2 m/s` command produced measured wheel velocities of `3.2915` and `3.3099 rad/s`, matching `0.2 / 0.0605 = 3.3058 rad/s`.
- The copied Tracer model's left joint frame has a pi roll, so its local axis must be `0 -1 0` to align with the right wheel's robot-frame axis. Equal positive wheel commands otherwise spin the base.
- Arm topic control reached `joint1=0.1995` and `joint2=-0.1988` for a `[0.2, -0.2, ...]` position command.
# 2026-06-22 Nav2 Prerequisite Discovery

- No `nav2_*`, `slam_toolbox`, or `pointcloud_to_laserscan` ROS packages are currently installed.
- The active Isaac ROS action graph publishes `/clock` and `/joint_states` and subscribes to `/cmd_vel` and `/arm_joint_commands`; it does not publish `/odom`, `odom -> base_link`, `/scan`, or a point cloud.
- `tracer_base.urdf.xacro` contains legacy Gazebo odometry plugin tags, but those do not configure Isaac Sim.
- Isaac Sim 4.5 includes RTX and physics sensor extensions plus the ROS 2 bridge, so an Isaac-native LiDAR publisher is available in principle.
- The unified description already has the calibrated `livox_frame`; no sensor measurement model is attached to it yet.
- No FAST-LIO2, Livox ROS driver, Nav2, SLAM Toolbox, or pointcloud-to-laserscan package was found in `/home/user/wu_ws` or the installed ROS packages.
- A FAST-LIO2 design therefore requires selecting and adding an external ROS 2 implementation plus defining whether Isaac publishes standard `sensor_msgs/PointCloud2` or Livox custom messages.
- The current Isaac runner adds only a flat ground plane; there are no walls or obstacles for LiDAR-based SLAM.
- `PROJECT_OVERVIEW.md` explicitly permits a first procedural environment made from boxes and cylinders, including walls, pipes, and columns.
- The fastest self-contained SLAM validation therefore needs a small primitive test arena in addition to odometry and a 2D LiDAR.

## Factory Reference Images

- `/home/user/Downloads/image (1).png` shows an experimental plant bay with partial-height partition walls, red guardrails, cylindrical vessels, repeated vertical pipe loops, wall-mounted pipe racks, and a much denser pool-local equipment room.
- `/home/user/Downloads/image.png` shows four aged plant service areas: narrow corridors, concrete columns, overhead pipe networks, vertical risers, pumps, tanks, electrical cabinets, grating/fences, and hanging cables.
- The useful simulation abstraction is a compact mixed-layout factory: one open bay, one narrow service corridor, repeated vertical/horizontal pipe banks, tanks/pumps/cabinets, and columns that create occlusions and alternate routes.
- Photorealistic corrosion and texture aging are not required for the first Nav2/SLAM benchmark; collision geometry, LiDAR-visible structure, scale, and traversable clearance are the priority.
- Isaac Sim 4.5 locally provides `IsaacComputeOdometry`, `ROS2PublishOdometry`, and the `ROS2RtxLidarHelper` flat-scan pipeline, matching the planned `/odom` and `/scan` interfaces without a custom ROS publisher.
- The RTX flat-scan node accumulates the lowest elevation emitter into a complete scan; a single-channel 360-degree RTX LiDAR configuration is appropriate for the temporary 2D `livox_frame` sensor.
- The unified URDF already defines `base_link -> frame_base` as a fixed identity transform. `base_link` is the single URDF and Isaac articulation root; `frame_base` is the calibrated alias used as parent for the arm and LiDAR transforms.
- NVIDIA's installed differential-base test connects `IsaacComputeOdometry` position/orientation/velocities to `ROS2PublishOdometry` and position/orientation to `ROS2PublishRawTransformTree`; the raw TF defaults match `odom -> base_link` and will be set explicitly.
- `LidarRtx` can create `Example_Rotary_2D`, a one-emitter rotary sensor, and provides a render-product path consumed by `ROS2RtxLidarHelper` with `type=laser_scan`, `topicName=scan`, and simulation time.
