# SAFE-Drive

ROS 2 Humble and Gazebo Classic 11 project for autonomous-driving risk-scenario
exploration. The first implementation phase builds and validates a repeatable
traffic simulation. The current scenes include a Prius visual model, road and
crossing, timed vehicle and pedestrian signals, mapped routes, and a separate
distance-triggered pedestrian scenario. Automated search is planned.

## Build

```bash
cd /home/suyeon/workspace/SAFE_DRIVE/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run one launch at a time from a shell with the workspace sourced:

The simulator now stops at startup with a clear error if an earlier Gazebo
server still owns `GAZEBO_MASTER_URI` (port 11345 by default). A stale server
can show an old world or vehicle even when a new `ros2 launch` command is run.
Close the previous Gazebo run before retrying; `ss -ltnp '( sport = :11345 )'`
identifies the server using the default port.
The GUI opens after the ego model has spawned so its camera can acquire the
vehicle. Closing the Gazebo window also stops the launched server.

| Check | Command |
| --- | --- |
| Straight route and destination stop | `ros2 launch safe_drive_bringup vehicle.launch.py` |
| Pedestrian crossing and live safety result | `ros2 launch safe_drive_bringup scenario.launch.py` |
| Timed signal and right-turn route | `ros2 launch safe_drive_bringup signal_intersection.launch.py` |
| Fixed red stop with JSON result | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=RED route_config:=fast_straight.yaml red_result_file:=/tmp/red_stop.json` |
| Fixed green right turn on extended south road | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=right_turn.yaml result_file:=/tmp/right_turn.json` |
| Fixed green left turn from nearby start | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN spawn_x:=-30 route_config:=left_turn.yaml result_file:=/tmp/left_turn.json` |
| Measured 50 km/h straight run | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=fast_straight.yaml result_file:=/tmp/fast_straight.json` |
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
With `spawn_x:=-30`, the tracked car starts about 25 m before the crossing,
so the intersection enters the camera view only as the car approaches.
The configured pedestrian crossing speed is now 1.0 m/s; the model still has
a fixed pose without animated steps. At the signalized crossing it now walks
onto the north sidewalk before the pedestrian WALK phase ends.

In another sourced shell, inspect the vehicle signal during the intersection
run or the safety result after the pedestrian scenario:

```bash
ros2 topic echo --once /safe_drive/vehicle_signal/state
ros2 topic echo --once --full-length --qos-reliability reliable \
  --qos-durability transient_local /safe_drive/safety/result
```

## Status

- **Implemented and verified headless:** waypoint driving and stop, Ackermann
  steering, fixed red stop, a green right turn and a short-start left turn,
  signal-coordinated pedestrian crossing with a red-stop vehicle response,
  a separate bend, and JSON measurements. Stored runs and criteria are in
  [validation results](docs/validation/results.md).
- **Verified in two headless runs:** the current fast straight route reached
  about 14.0 m/s (50 km/h), stopped at its goal, and kept every measured
  chassis corner within the 1.75 m route corridor. The earlier 61 km/h
  setting exceeded that corridor and remains a historical failed result.
- **Implemented and verified separately:** distance-triggered pedestrian
  crossing with live ground-truth distance, TTC, and contact reporting. The
  current vehicle controller does not react to the pedestrian.
- **Planned core study:** a repeat runner for the sudden-pedestrian parameter
  space, predefined risk criteria, and equal-budget Random Sampling versus
  Risk-Guided Adaptive Grid Refinement. No automatic search result exists yet;
  see [risk-scenario exploration plan](docs/experiments.md).
- **Planned environment extensions:** camera and LiDAR topics, repeated
  integrated validation runs, and sensor-based pedestrian response.
  Radar is optional.
- **Optional presentation aid:** a live results dashboard after the experiment
  loop works. It would display saved measurements, not create risk scores.

The controller uses configured map waypoints, odometry, and the ROS vehicle
signal state. It does not use camera or LiDAR perception. The corridor result
is a geometric check against the configured route, not lane detection.

See [architecture](docs/architecture.md), [scenarios](docs/scenarios.md),
[road layout](docs/road-layout.md), [asset licenses](docs/asset-inventory.md),
and the [development log](docs/development-log.md) for implementation details
and limitations.
