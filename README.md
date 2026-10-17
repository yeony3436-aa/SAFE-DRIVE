# SAFE-Drive

ROS 2 Humble and Gazebo Classic 11 driving simulation for repeatable traffic
scenario validation. The current scenes include a Prius visual model, road and
crossing, timed vehicle and pedestrian signals, mapped routes, and a separate
distance-triggered pedestrian scenario.

## Build

```bash
cd /home/suyeon/workspace/SAFE_DRIVE/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run one launch at a time from a shell with the workspace sourced:

| Check | Command |
| --- | --- |
| Straight route and destination stop | `ros2 launch safe_drive_bringup vehicle.launch.py` |
| Pedestrian crossing and live safety result | `ros2 launch safe_drive_bringup scenario.launch.py` |
| Timed signal and straight route | `ros2 launch safe_drive_bringup signal_intersection.launch.py` |
| Fixed red stop | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=RED` |
| Fixed green right turn | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=right_turn.yaml result_file:=/tmp/right_turn.json` |
| Fixed green left turn | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=left_turn.yaml result_file:=/tmp/left_turn.json` |
| Gentle bend | `ros2 launch safe_drive_bringup curve_course.launch.py result_file:=/tmp/curve.json` |

Add `gui:=false` to a launch for a headless run. The intersection route monitor
writes JSON after the vehicle settles at the goal; its default path is
`/tmp/safe_drive_route_result.json`. The bend uses
`/tmp/safe_drive_curve_result.json` by default. A fixed red run stops before
the line and does not produce a completed route result.

In another sourced shell, inspect the vehicle signal during the intersection
run or the safety result after the pedestrian scenario:

```bash
ros2 topic echo --once /safe_drive/vehicle_signal/state
ros2 topic echo --once --full-length --qos-reliability reliable \
  --qos-durability transient_local /safe_drive/safety/result
```

## Status

- **Implemented and verified headless:** waypoint driving and stop, Ackermann
  steering, fixed red stop, fixed green straight/left/right routes, a separate
  bend, and JSON route measurements. Stored runs and criteria are in
  [validation results](docs/validation/results.md).
- **Implemented and verified separately:** distance-triggered pedestrian
  crossing with live ground-truth distance, TTC, and contact reporting. The
  current vehicle controller does not react to the pedestrian.
- **Planned:** signal-aware pedestrian motion in the intersection, camera and
  LiDAR topics, integrated traffic-case result files, and repeatability runs.
  Radar is optional.

The controller uses configured map waypoints, odometry, and the ROS vehicle
signal state. It does not use camera or LiDAR perception. The corridor result
is a geometric check against the configured route, not lane detection.

See [architecture](docs/architecture.md), [scenarios](docs/scenarios.md),
[road layout](docs/road-layout.md), [asset licenses](docs/asset-inventory.md),
and the [development log](docs/development-log.md) for implementation details
and limitations.
