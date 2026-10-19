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
| Timed signal and right-turn route | `ros2 launch safe_drive_bringup signal_intersection.launch.py` |
| Fixed red stop with JSON result | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=RED route_config:=fast_straight.yaml red_result_file:=/tmp/red_stop.json` |
| Fixed green right turn on extended south road | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=right_turn.yaml result_file:=/tmp/right_turn.json` |
| Fixed green left turn | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=left_turn.yaml result_file:=/tmp/left_turn.json` |
| Measured 60 km/h straight run | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=fast_straight.yaml result_file:=/tmp/fast_straight.json` |
| Signal-coordinated pedestrian and red stop | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=RED spawn_x:=-30 pedestrian:=true route_config:=fast_straight.yaml` |
| Pedestrian crossing, then green-signal departure | `ros2 launch safe_drive_bringup signal_intersection.launch.py spawn_x:=-30 pedestrian:=true route_config:=fast_straight.yaml` |
| Gentle bend | `ros2 launch safe_drive_bringup curve_course.launch.py result_file:=/tmp/curve.json` |

Add `gui:=false` to a launch for a headless run. The intersection route monitor
writes JSON after the vehicle settles at the goal; its default path is
`/tmp/safe_drive_route_result.json`. The bend uses
`/tmp/safe_drive_curve_result.json` by default. A fixed red run writes a
separate stop-line result to `red_result_file` after the vehicle holds still
near the line for five simulated seconds. It does not produce a completed
route result.
With `pedestrian:=true`, the fixed-red integration also saves crossing and
ground-truth safety JSON to `/tmp/safe_drive_signal_crossing.json` and
`/tmp/safe_drive_signal_safety.json`. The pedestrian starts on the pedestrian
`WALK` topic, while the car stops on the vehicle `RED` topic.
With `fixed_state:=RED`, the light never turns green, so the car stays stopped
after the pedestrian finishes. Omit `fixed_state` for a timed RED-to-GREEN
transition. The default intersection launch follows `right_turn.yaml` and
does not spawn a pedestrian unless `pedestrian:=true` is set.

In another sourced shell, inspect the vehicle signal during the intersection
run or the safety result after the pedestrian scenario:

```bash
ros2 topic echo --once /safe_drive/vehicle_signal/state
ros2 topic echo --once --full-length --qos-reliability reliable \
  --qos-durability transient_local /safe_drive/safety/result
```

## Status

- **Implemented and verified headless:** waypoint driving and stop, Ackermann
  steering, fixed red stop, faster green right/left routes, signal-coordinated
  pedestrian crossing with a red-stop vehicle response, a separate bend, and
  JSON measurements. Stored runs and criteria are in
  [validation results](docs/validation/results.md).
- **Measured but outside the lane-corridor criterion:** the fast straight
  route reached 17.07 m/s (about 61 km/h) and stopped at its goal. Its
  maximum chassis-corner route error was 2.46 m against a 1.75 m limit.
- **Implemented and verified separately:** distance-triggered pedestrian
  crossing with live ground-truth distance, TTC, and contact reporting. The
  current vehicle controller does not react to the pedestrian.
- **Planned:** camera and LiDAR topics, repeated integrated validation runs,
  and sensor-based pedestrian response.
  Radar is optional.
- **Optional follow-up:** scenario exploration with a live results dashboard
  after the integrated scene and repeatable validation are stable; see
  [validation and optional research](docs/experiments.md).

The controller uses configured map waypoints, odometry, and the ROS vehicle
signal state. It does not use camera or LiDAR perception. The corridor result
is a geometric check against the configured route, not lane detection.

See [architecture](docs/architecture.md), [scenarios](docs/scenarios.md),
[road layout](docs/road-layout.md), [asset licenses](docs/asset-inventory.md),
and the [development log](docs/development-log.md) for implementation details
and limitations.
