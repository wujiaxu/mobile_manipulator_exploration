# Arm Support Block Design

## Goal

Add a rigid, collision-enabled support block between the Tracer chassis and the calibrated xArm7 base for URDF, RViz2, and Isaac Sim import.

## Geometry

The support is a primitive box centered beneath the arm mounting point:

- Width: `0.300 m`
- Depth: `0.300 m`
- Height: `0.49692302 m`
- Center: `x=-0.076 m`, `y=0 m`, `z=0.27053849 m` in `frame_base`
- Bottom: measured Tracer chassis mesh top at `z=0.02207698 m`
- Top: calibrated arm base plane at `z=0.519 m`

The xArm calibration remains exactly `frame_base -> frame_1: xyz=-0.076 0 0.519, rpy=0 0 0`.

## Physical Model

`arm_support_link` is attached to `frame_base` by `frame_base_to_arm_support`, a fixed joint at the box center. The link contains matching box visual and collision elements. A nominal `10 kg` hollow-structure mass is used with box inertia:

- `ixx = iyy = 0.2807770732 kg m^2`
- `izz = 0.15 kg m^2`
- Cross terms are zero

The existing fixed arm mounting joints provide the rigid mechanical constraint under gravity. The support block adds the corresponding visible and collision geometry for simulation.

## Validation

Expand the Xacro and assert the support link, fixed joint, box dimensions, collision element, mass, inertia, and unchanged arm calibration. Then run `check_urdf`, build the package, and reload the RViz2 display.

