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
  simulator-independent ego vehicle URDF/Xacro, minimal Gazebo road world,
  ego-vehicle spawning, ROS 2 velocity commands, odometry, waypoint following,
  and destination stop
- Completed: MVP-01 vehicle baseline
- Planned next: MVP-02 configurable sudden-pedestrian scenario

## MVP-01 Target

One ego vehicle should drive through predefined waypoints in Gazebo and stop at the final waypoint.

Required behavior:

- Gazebo launches. — Implemented
- Ego vehicle spawns. — Implemented
- ROS 2 control command works. — Implemented
- Odometry is published. — Implemented
- Vehicle follows several waypoints. — Implemented
- Vehicle stops at the final waypoint. — Implemented

Run the MVP-01 baseline:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch safe_drive_bringup vehicle.launch.py
```
