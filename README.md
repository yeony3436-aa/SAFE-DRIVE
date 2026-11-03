# SAFE-Drive

ROS 2 Humble and Gazebo Classic 11 project for autonomous-driving risk-scenario
exploration. The first implementation phase builds and validates a repeatable
traffic simulation. The current scenes include a Prius visual model, road and
crossing, timed vehicle and pedestrian signals, mapped routes, and a separate
distance-triggered pedestrian scenario. A small measured Random/Adaptive
scenario-selection loop and forward camera and planar LiDAR are implemented.

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
| Connected road, right-turn return route | `ros2 launch safe_drive_bringup signal_intersection.launch.py world_file:=connected_road.world spawn_x:=-27 route_config:=connected_right_loop.yaml` |
| Connected road, left-turn return route | `ros2 launch safe_drive_bringup signal_intersection.launch.py world_file:=connected_road.world spawn_x:=-30 route_config:=connected_left_loop.yaml` |
| Connected road, straight-through return route | `ros2 launch safe_drive_bringup signal_intersection.launch.py world_file:=connected_road.world spawn_x:=-30 route_config:=connected_straight_loop.yaml` |
| Unsignalized right turn yielding to a pedestrian | `ros2 launch safe_drive_bringup unsignalized_right_turn.launch.py` |
| Fixed red stop with JSON result | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=RED route_config:=fast_straight.yaml red_result_file:=/tmp/red_stop.json` |
| Fixed green right turn on extended south road | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=right_turn.yaml result_file:=/tmp/right_turn.json` |
| Fixed green left turn from nearby start | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN spawn_x:=-30 route_config:=left_turn.yaml result_file:=/tmp/left_turn.json` |
| Measured 50 km/h straight run | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=GREEN route_config:=fast_straight.yaml result_file:=/tmp/fast_straight.json` |
| Signal-coordinated pedestrian and red stop | `ros2 launch safe_drive_bringup signal_intersection.launch.py fixed_state:=RED spawn_x:=-30 pedestrian:=true route_config:=fast_straight.yaml` |
| Pedestrian crossing, then green-signal departure | `ros2 launch safe_drive_bringup signal_intersection.launch.py spawn_x:=-30 pedestrian:=true route_config:=fast_straight.yaml` |
| Gentle bend | `ros2 launch safe_drive_bringup curve_course.launch.py result_file:=/tmp/curve.json` |

To rerun a connected-scene check from the sourced `ros2_ws` shell, use a new
output directory for each invocation:

```bash
python3 ../scripts/run_connected_validation.py --case red_crossing \
  --output-dir /tmp/safe_drive_connected_check_01
```

Cases are defined in `config/connected_validation.yaml`. Omit `--case` to run
all six cases sequentially, each in a fresh Gazebo process. The runner saves
raw results, a launch log,
settings, input-file hashes, Git revision, and `summary.json`. It exits
nonzero for a failed case. The right loop now starts and returns near
`(-27,-1.75) m`; its 1.2 m/s configured run passed twice after a curve
tracking correction. This runner validates
the scene and does not implement risk-scenario search.

The `continuous_right` case is one uninterrupted drive with a cycling signal:
the ego stops on red while a pedestrian crosses, proceeds on green, turns
right, and completes the connected return route. Run it alone with
`--case continuous_right`, or watch it in Gazebo with:

```bash
ros2 launch safe_drive_bringup signal_intersection.launch.py \
  world_file:=connected_road.world spawn_x:=-16 \
  route_config:=connected_right_loop.yaml pedestrian:=true \
  monitor_red_stop:=true
```

This is a safe baseline; the pedestrian clears the crossing before the car
continues. The continuous mission currently has one controlled intersection
and the `continuous_late_crossing` case separately triggers a pedestrian
against DONT_WALK after the vehicle begins moving on green. That case ends
after the crossing outcome is measured; a recorded collision is a valid risk
outcome, not a safe-driving pass or a completed return route.

To repeat that late-crossing event across requested speeds and pedestrian
entry distances, run this from a sourced `ros2_ws` shell:

```bash
python3 ../scripts/run_late_crossing_sweep.py \
  --output-dir /tmp/safe_drive_late_crossing_matrix_01
```

The matrix command runs headless by default. To watch **one** condition in
the Gazebo window, use a new output directory and add `--gui`:

```bash
python3 ../scripts/run_late_crossing_sweep.py --gui \
  --speed 1.2 --distance 2.0 \
  --output-dir /tmp/safe_drive_late_preview_01
```

The window opens after the ego spawns and closes when that case is measured.
`Ctrl+C` stops the current case and the entire matrix, while retaining a
partial summary. The output directory must be new for each invocation.
Close any earlier Gazebo run using the same master port before starting this
preview; a port conflict is reported directly in the terminal.

`config/late_crossing_sweep.yaml` defines the small 2×3 matrix. Each fresh
Gazebo run saves its route and crossing variants, actual ego speed at entry,
stop gap, contact, minimum actor-center distance, painted-line overlap, and
a separate event-valid/risk label in JSON and CSV. This deterministic matrix
has one run per condition and does not establish a real-world risk rate.
The late-entry 2 m case deliberately creates a close pedestrian conflict;
the current follower does not avoid the pedestrian. Gazebo contact is recorded
as a risk event. The pose-driven pedestrian now has contact force disabled so
it cannot push the ego sideways, but the scene does not model realistic impact
physics or injury. Earlier late-crossing matrix measurements used the old
contact response and should be treated as a separate simulator configuration.

For the focused visual demonstration, use the configured braking and contact
cutoff cases from a sourced `ros2_ws` shell. Use a fresh output directory each
time, and set a free `GAZEBO_MASTER_URI` if the default port is occupied:

```bash
python3 ../scripts/run_connected_validation.py --gui \
  --config ../config/quality_demo.yaml --case brake_before_crossing \
  --output-dir /tmp/safe_drive_brake_demo_01
python3 ../scripts/run_connected_validation.py --gui \
  --config ../config/quality_demo.yaml --case contact_cut \
  --output-dir /tmp/safe_drive_contact_demo_01
```

The first case applies a scenario-state-based crossing stop and checks measured
front gap and speed. The second ends when contact is detected so the actor does
not continue walking through the ego. This demonstration controller uses the
scenario state and configured crossing position, not camera or LiDAR perception.
The contact demo uses a narrower collision proxy than the experiment matrix;
its GUI appearance is still under review after a reported visible gap. Use the
braking case as the verified visual behavior for presentations for now.

To vary requested route speed and retain independent route or red-stop
measurements, run a selected case and speed from a sourced workspace:

```bash
python3 ../scripts/run_speed_validation.py --case green_right \
  --speed 1.2 --speed 1.6 --output-dir /tmp/safe_drive_speed_check_01
```

The full low-speed case list is in `config/speed_validation.yaml`. Each run
uses a fresh Gazebo process and saves its generated route YAML, actual maximum
speed, raw result, and JSON/CSV summary. The 1.75 m route-corridor check and
the connected road's painted-centerline/lane-edge check are separate results.
The latter checks chassis corners on painted straight arms; unpainted junctions
are outside its scope. A speed request is not proof of the speed reached.
For a faster connected-map check, select `green_straight`, `red_crossing`, or
`continuous_right` with `--speed 4.0` or `--speed 6.0` and a new output path.
The straight-loop controller now brakes using the remaining route distance
and limits bend speed. In the stored single-run 6.0 m/s requests, the straight
loop reached 4.921 m/s and the continuous right mission reached 4.674 m/s;
both passed their route and painted-line checks. The fixed-red approach reached
only 2.535 m/s before stopping, so it does not validate a 6 m/s red approach.
`continuous_right` records both whole-route maximum speed and red-approach
maximum speed. See [measured speed results](docs/validation/results.md).

To repeat two configured sudden-pedestrian conditions twice each and save
the measured outcomes and predefined risk labels:

```bash
python3 ../scripts/run_pedestrian_sweep.py --repeats 2 \
  --output-dir /tmp/safe_drive_pedestrian_sweep_01
```

The conditions and 3.0 m actor-center threshold are in
`config/pedestrian_sweep.yaml`. The current seed is recorded, but the scenario
has no random draw. This fixed sweep does not compare search methods.

To run the measured search-method comparison with equal budgets, use a fresh
output directory:

```bash
python3 ../scripts/run_risk_exploration.py \
  --output-dir /tmp/safe_drive_risk_exploration_01
```

`config/risk_exploration.yaml` fixes the parameter ranges, risk rule, seeds,
and four runs per method and seed. Each run saves its generated scenario,
launch log, raw safety result, and record; `summary.json` and `runs.csv`
compare the measured outcomes. The first center point is shared. Random uses
a seeded uniform draw for later points; Adaptive subdivides measured regions
into octants and prioritizes collision or smaller clearance. The scenario
itself makes no random draw, and Adaptive currently uses deterministic tie
order. This small default budget is a functional demonstration, not a
performance claim.

To validate actual camera and LiDAR messages against the roadside bollard:

```bash
python3 ../scripts/validate_sensors.py \
  --output-dir /tmp/safe_drive_sensors_01
```

See [sensor topics, frames, rates, and sample](docs/sensors.md).

Add `gui:=false` to a launch for a headless run. The intersection route monitor
writes JSON after the vehicle settles at the goal; its default path is
`/tmp/safe_drive_route_result.json`. The bend uses
`/tmp/safe_drive_curve_result.json` by default. A fixed red run writes a
separate stop-line result to `red_result_file` after the vehicle holds still
near the line for five simulated seconds. It does not produce a completed
route result.
The connected road joins the central signalized intersection to an outer grid;
all four approaches reconnect. Its return routes are initial map-validation
routes, and only the original eastbound central approach has signal control.
Use `fixed_state:=GREEN` to isolate steering during a route check. Geometry and
verification status are in [road layout](docs/road-layout.md). All three
configured loops returned to their starts in headless Gazebo runs. The first
right loop exceeded the strict lane-corridor criterion; its revised route
passed at configured 1.2 and 1.6 m/s with curve-aware speed limiting. A later
painted-line check found a right-turn edge overlap; the first-turn radius was
adjusted to 4.0 m and separate 1.2 and 1.6 m/s runs then passed both checks.
The painted-line result covers marked straight arms, not unmarked junctions.
The separate unsignalized right-turn launch starts at `x=10 m` and uses the
east-side junction at `x=36 m`. It stops before the painted crossing while a
configured pedestrian is active, then turns after the crossing completes. Its
yield decision uses the scenario's ROS state, not camera or LiDAR perception.
The launch saves independent route, yield-stop, and ground-truth safety JSON.
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
- **Implemented and verified for the connected scene:** a configuration-driven
  validation runner; the fixed-red signal and pedestrian case passed in two
  fresh headless runs with recorded raw results.
- **Implemented and verified headless:** an unsignalized right turn that yields
  to the configured pedestrian, then completes its route. Other unsignalized
  cross-traffic conflicts are not yet modeled.
- **Implemented small measured search loop:** a fixed two-condition sweep and
  an equal-budget Random/Adaptive selector record configurations, selection
  seeds, ground-truth outcomes, and predefined risk labels. The 16-run check
  validates execution and accounting; it does not establish a method
  advantage. See [risk-scenario exploration](docs/experiments.md).
- **Implemented and headless-verified sensors:** front camera and planar
  LiDAR publish ROS 2 data with a visible/returned roadside target.
  [Sensor validation](docs/sensors.md) records frames, rates, and Gazebo
  performance samples. Driving still does not use these sensor messages.
- **Planned environment extensions:** broader integrated validation and
  sensor-based pedestrian response.
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
