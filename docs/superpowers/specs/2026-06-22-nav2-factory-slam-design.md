# Nav2 Factory SLAM Design

## Goal

Add the first reproducible factory environment and the ROS interfaces required
to map and navigate it with Nav2, while preserving the verified MoveIt and Isaac
arm-control workflows.

FAST-LIO2 and realistic Livox scan patterns are deferred. This milestone uses a
temporary 360-degree 2D LiDAR so the Nav2 architecture can be validated first.

## Factory Environment

A deterministic Python generator creates a standalone USD stage for a compact
10 m by 8 m industrial arena. The environment uses primitive geometry with
authored collision and simple industrial materials. Runtime code composes this
factory USD with the existing mobile-manipulator ROS USD rather than rebuilding
the environment every launch.

The layout contains:

- Four perimeter walls and two internal partition walls
- One open experimental bay and one narrow service corridor with two exits
- At least six structural columns
- At least four cabinets or control enclosures
- At least three cylindrical tanks or vessels
- At least two pump-like assemblies
- At least four pipe racks
- At least twelve vertical pipe sections and twelve horizontal pipe sections
- Guardrails around one equipment zone
- A clear robot spawn zone and at least three reachable navigation goal areas

Main routes are at least 1.2 m wide. Deliberately narrow service sections are at
least 0.8 m wide. Overhead pipes create the dense visual character shown in the
reference slides, while selected waist-height and vertical pipe geometry remains
visible in the 2D scan plane. No route requires the robot to pass through a
collision clearance smaller than 0.8 m.

The first version prioritizes scale, collision, occlusion, and LiDAR-visible
structure. Photorealistic corrosion, cables, labels, and aged textures are out of
scope.

## Isaac Composition And Sensors

The existing generated robot USD remains the source of robot articulation and
ROS control. A new navigation runner opens the factory stage, composes the robot
at the authored spawn pose, enables the ROS 2 bridge, and starts simulation.

A single-channel 360-degree RTX LiDAR is rigidly attached to the existing
calibrated `livox_frame`. It publishes `sensor_msgs/msg/LaserScan` on `/scan` with
frame ID `livox_frame`. The sensor uses a bounded useful range for the 10 m by 8 m
arena and publishes using simulation time. This is explicitly a Nav2 bootstrap
sensor and does not claim to reproduce a Livox measurement pattern.

The Isaac action graph adds:

- `IsaacComputeOdometry` for the mobile articulation root
- `ROS2PublishOdometry` on `/odom`
- `odom -> base_link` TF publication from the same odometry result
- `ROS2RtxLidarHelper` flat-scan publication on `/scan`

Existing `/clock`, `/joint_states`, `/cmd_vel`, and `/arm_joint_commands`
interfaces remain unchanged.

## Frame Ownership

Frame ownership is exclusive:

- Isaac publishes `odom -> base_link` and `/odom`.
- `robot_state_publisher` publishes the existing fixed identity
  `base_link -> frame_base` edge and all descendants, including the calibrated
  `frame_base -> livox_frame` path.
- SLAM Toolbox publishes `map -> odom` and `/map`.
- Nav2 consumes the tree and does not publish a duplicate base transform.

The resulting navigation chain is `map -> odom -> base_link -> ... ->
livox_frame`.

`base_link` remains the canonical Nav2 robot base because it is the URDF root
and Isaac articulation root. `frame_base` remains an identity-offset calibration
alias; the joint is not reversed and neither frame is removed.

## ROS Packages And Launch

Install ROS Humble Nav2, Nav2 bringup, and SLAM Toolbox. Add a new
`mobile_manipulator_navigation` package containing SLAM Toolbox configuration,
Nav2 parameters, RViz navigation configuration, and launch orchestration.

The initial workflow has two modes:

1. Mapping mode starts Isaac factory simulation, robot state publication, SLAM
   Toolbox, Nav2, and RViz. The user can drive manually or send goals while the
   map grows.
2. Saved-map mode is deferred until a useful map has been produced and validated.

MoveIt remains a separate launch boundary during this milestone. A later combined
launcher may start both navigation and manipulation after each stack is stable.

## Nav2 Configuration

Nav2 uses the differential base through the existing `/cmd_vel` subscription.
The robot footprint is derived conservatively from the mobile base dimensions,
not from the arm sweep. Local and global costmaps use `/scan` obstacle data,
simulation time, and the standard `map`, `odom`, and `base_link` frames.

Initial speeds are deliberately conservative for the dense environment. Main
success criteria are stable TF, collision-free planning, and repeatable movement;
aggressive tuning is out of scope.

## Error Handling

Generation fails if required dimensions, clearances, or collision schemas are
missing. The navigation launch fails early when the generated factory asset,
robot asset, ROS installation, or configuration files are absent.

Live verification treats missing or duplicated TF authorities, stale simulation
timestamps, empty scans, and an odometry pose that does not follow base motion as
blocking failures.

## Testing

Dependency-free contracts verify factory dimensions, object counts, spawn and
goal markers, collision schemas, sensor topic/frame configuration, odometry
topic/frame configuration, package manifests, and Nav2/SLAM parameters.

Isaac GPU tests generate and open the factory stage, verify the composed robot,
sample nonempty `/scan` and `/odom`, and confirm forward motion changes odometry
without breaking the robot articulation.

Live integration verifies:

1. `map -> odom -> base_link -> livox_frame` is connected and has one authority
   per dynamic edge.
2. Driving through the arena grows a nonempty SLAM occupancy grid.
3. Nav2 plans and executes at least three collision-free goals, including one
   through the 0.8 m service corridor.
4. Existing MoveIt contracts, arm topic control, unified-description hashes, and
   generated robot USD contracts remain green.

## Deferred Work

- FAST-LIO2, IMU simulation, and 3D Livox point-cloud modeling
- Saved-map localization with AMCL
- Photorealistic materials and CAD assets
- Dynamic obstacles and randomized factory layouts
- Combined coordinated base-and-arm planning
