# SAFE-Drive

ROS 2 / Gazebo based autonomous-driving simulation environment and traffic-scenario validation project.

The primary goal is to build a visually credible environment whose vehicle,
road, crossing, traffic signal, pedestrian, and sensors work together. Defined
traffic situations then verify the behavior and limitations of that environment.

The core target is one repeatable demonstration with route following, red-signal
stopping, pedestrian interaction, camera and LiDAR ROS data, and saved validation
results. Radar is a desirable extension to evaluate after the camera and LiDAR
work. Random vs. Adaptive scenario search is an optional follow-up study, not
a graduation-critical dependency.

The project does not depend on YOLO, LLMs, voice input, or a large custom city.

## Environment

- ROS 2: Humble
- Simulator: Gazebo Classic 11.10.2

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Current Status

- Implemented: initial project structure, `safe_drive_bringup`, the
  simulator-independent ego vehicle URDF/Xacro, minimal Gazebo road world,
  ego-vehicle spawning, ROS 2 velocity commands, odometry, waypoint following,
  destination stop, one configurable sudden-pedestrian scenario, and
  ground-truth safety measurement
- Completed: MVP-01 vehicle baseline
- Completed: MVP-02 sudden-pedestrian execution baseline
- Implemented: MVP-03 live minimum center distance, line-of-sight TTC,
  pedestrian contact, commanded emergency braking, and completion result
- Implemented: a Prius exterior and wheel meshes, MCity-inspired textured road,
  sidewalks and verges, a revised zebra crossing at `x=-5 m`, and a more
  human-shaped pedestrian. These are visual changes; the collision shapes and
  driving model remain simple. Third-party files are listed in
  [the asset inventory](docs/asset-inventory.md).
- Implemented and headless-verified: a separate compact intersection, a timed
  roadside three-aspect vehicle signal and curbside pedestrian signal with separate ROS 2
  states, red-signal stop before the line, and green-signal route completion.
  Both road worlds now have bars aligned with vehicle travel and a stop line
  ahead of the crossing. The original vehicle and pedestrian launches remain
  separate; see [road layout](docs/road-layout.md).
- Planned: connect pedestrian motion to the signal phase; add camera and LiDAR;
  save and repeat validation results. Radar remains optional.
- Not implemented: signal-aware pedestrian crossing, camera, LiDAR, radar,
  sensor-based obstacle avoidance, and result-file persistence

## Existing vehicle baseline

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

Run the MVP-02 scenario:

```bash
ros2 launch safe_drive_bringup scenario.launch.py
```

Run the compact signal intersection (red, green, yellow cycle):

```bash
ros2 launch safe_drive_bringup signal_intersection.launch.py
```

For a repeatable phase check, add `fixed_state:=RED` or
`fixed_state:=GREEN`. The vehicle and pedestrian indications are separate
`std_msgs/msg/String` topics: `/safe_drive/vehicle_signal/state` and
`/safe_drive/pedestrian_signal/state`. The controller subscribes only to the
vehicle topic and limits speed before the `x=-9.25 m` stop line. It still
follows odometry and configured waypoints, without camera or LiDAR perception.

The default scenario definition is stored in
`ros2_ws/src/safe_drive_scenario/config/sudden_pedestrian.yaml`. The
`ego_speed` value is also passed to the waypoint follower so that scenario and
vehicle configuration remain consistent. The current default starts the crossing
at `trigger_distance=8.0 m` with `ego_speed=2.0 m/s`. The vehicle does not
yet actively brake or steer for pedestrians; changing trigger timing can
produce collision cases. The painted crossing is fixed at `x=-5 m` and
matches the default pedestrian spawn distance.

The scenario launch also starts `safe_drive_safety`. Inspect the latest result:

```bash
ros2 topic echo --once --full-length --qos-reliability reliable \
  --qos-durability transient_local /safe_drive/safety/result
```

`min_distance` is the actor-center distance in meters. TTC uses the current
line-of-sight closing speed and is `null` when no valid closing observation
exists. Collision comes from Gazebo pedestrian contact, while emergency braking
currently means the commanded speed dropped at least the configured rate.
