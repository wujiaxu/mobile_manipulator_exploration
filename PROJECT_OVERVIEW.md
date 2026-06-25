# Mobile Manipulator Exploration Benchmark Platform

## Project Goal

Develop a reusable simulation platform for evaluating exploration and inspection algorithms using a mobile manipulator in industrial environments.

The platform will be based on:

* Isaac Sim
* ROS 2 Humble
* MoveIt 2
* Navigation2 (Nav2)

The platform should support multiple future exploration algorithms, including:

* Next-Best-View (NBV)
* Frontier-based exploration
* Active perception
* Active SLAM
* Reinforcement Learning based exploration

The initial target algorithm to reproduce is:

"Online Next-Best-View Planner for 3D-Exploration and Inspection With a Mobile Manipulator Robot"

Original implementation:

https://github.com/fkie/fkie-nbv-planner

The original implementation uses ROS1 and Gazebo.

The goal of this project is NOT to port the entire ROS1 repository, but to build a modern ROS2 + Isaac Sim benchmark platform and later integrate the planner's core algorithm.

---

# System Architecture

## Robot

The robot consists of:

### Mobile Platform

* Existing URDF available

### Manipulator

* Existing URDF available

### Mounting Transform

Known fixed transform:

mobile_base_link -> arm_base_link

The final robot model should be represented as a single robot.

---

## Sensors

### LiDAR

Mounted on the mobile platform.

Known transform:

mobile_base_link -> lidar_link

Expected outputs:

* LaserScan
* PointCloud2

### Wrist Camera

Mounted on the manipulator end-effector.

Known transform:

ee_link -> camera_link

Expected outputs:

* RGB image
* Depth image
* Camera info

---

# Target Environment

The target application is inspection and exploration inside a nuclear power plant.

A realistic CAD model is NOT required initially.

The first version should use primitive geometry.

Environment elements:

* Floor
* Walls
* Vertical pipes
* Horizontal pipes
* Structural columns

Primitive shapes:

* Boxes
* Cylinders

Typical horizontal pipe height:

0.7 m above the floor

The environment should be generated procedurally to support future randomization.

---

# Desired Robot Capabilities

## Navigation

Given:

geometry_msgs/PoseStamped

The mobile base should navigate to the target pose using Nav2.

---

## Manipulation

Given:

geometry_msgs/PoseStamped

The manipulator should move the wrist camera to the target pose using MoveIt 2.

---

## Combined Operation

Future exploration algorithms should be able to command:

1. Mobile base goal
2. End-effector goal

independently.

---

# Software Stack

## Simulation

Isaac Sim

## Middleware

ROS 2 Humble

## Navigation

Nav2

## Manipulation

MoveIt 2

## Mapping

Candidate options:

* OctoMap
* Voxblox
* nvblox

The first implementation can start with OctoMap.

---

# Development Phases

## Phase 1: Workspace Setup

Goal:

Create project structure and repository.

Deliverables:

* Git repository
* ROS2 workspace
* Isaac Sim project directory
* Documentation

Success criteria:

Project builds and launches successfully.

---

## Phase 2: Robot Integration

Goal:

Create unified mobile manipulator model.

Tasks:

* Merge mobile base and arm URDFs
* Add fixed mounting joint
* Add sensor frames
* Validate TF tree

Deliverables:

* Unified URDF
* Unified USD

Success criteria:

Robot loads correctly in Isaac Sim.

---

## Phase 3: Isaac Sim Integration

Goal:

Import robot into Isaac Sim.

Tasks:

* URDF import
* Physics validation
* Joint control validation

Deliverables:

* robot.usd

Success criteria:

Robot can be controlled in Isaac Sim.

---

## Phase 4: Sensor Integration

Goal:

Integrate LiDAR and wrist camera.

Tasks:

* Add RTX LiDAR
* Add RGB-D camera
* ROS2 bridge setup

Deliverables:

* Sensor topics available in ROS2

Success criteria:

Sensor data visible from ROS2.

---

## Phase 5: MoveIt 2 Integration

Goal:

Control manipulator through MoveIt 2.

Tasks:

* Generate MoveIt configuration
* Validate IK
* Execute pose goals

Success criteria:

Manipulator reaches requested end-effector pose.

---

## Phase 6: Nav2 Integration

Goal:

Control mobile base through Nav2.

Tasks:

* Odometry
* Localization
* Costmaps
* Path planning

Success criteria:

Robot reaches navigation goals.

---

## Phase 7: Nuclear Plant Environment

Goal:

Create procedural industrial inspection environment.

Tasks:

* Pipe generation
* Obstacle generation
* Environment randomization

Success criteria:

Environment suitable for exploration experiments.

---

## Phase 8: Exploration API

Goal:

Provide a common interface for all exploration algorithms.

Inputs:

* Sensor observations
* Robot state
* Map

Outputs:

* Mobile base goal
* End-effector goal

Success criteria:

Algorithms can be swapped without modifying the simulator.

---

## Future Work

* Port FKIE NBV planner core to ROS2
* Integrate OctoMap/Voxblox/Nvblox
* Multi-floor environments
* Radiation field simulation
* Isaac Lab integration
* Reinforcement learning exploration policies

---

# Instructions for Codex

When assisting this project:

1. Prefer ROS2 Humble solutions.
2. Prefer Isaac Sim over Gazebo.
3. Prefer clean ROS2-native implementations instead of ROS1 bridges.
4. Minimize technical debt.
5. Keep components modular and reusable.
6. Explain architecture decisions before generating large amounts of code.
7. When uncertain, propose multiple design options with trade-offs.
