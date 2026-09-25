# SAFE-Drive

ROS 2 / Gazebo based autonomous driving safety evaluation and risk scenario exploration platform.

The graduation-critical goal is to build a reproducible research core:

1. Launch Gazebo.
2. Spawn one ego vehicle.
3. Follow predefined waypoints.
4. Execute one configurable hazardous scenario.
5. Calculate safety metrics such as TTC and collision.
6. Run automated experiments.
7. Compare Random Sampling with Risk-Guided Adaptive Grid Refinement under the same budget.

This project does not initially depend on YOLO, lane detection, LLMs, voice input, or a large custom city model.

## Environment

- ROS 2: Humble
- Simulator: Gazebo Classic 11.10.2

## Current Status

- Implemented: initial project structure, `safe_drive_bringup`, the
  simulator-independent ego vehicle URDF/Xacro, and minimal Gazebo road world
  with ego-vehicle spawning
- In progress: MVP-01 ROS 2 vehicle control and odometry
- Planned next: `safe_drive_control` with a simple command interface

## MVP-01 Target

One ego vehicle should drive through predefined waypoints in Gazebo and stop at the final waypoint.

Required behavior:

- Gazebo launches. — Implemented
- Ego vehicle spawns. — Implemented
- ROS 2 control command works. — Planned
- Odometry is published. — Planned
- Vehicle follows several waypoints. — Planned
- Vehicle stops at the final waypoint. — Planned
