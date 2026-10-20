# Scenarios

The first supported scenario is `sudden_pedestrian`.

Scenario definitions must be configuration data, not hardcoded ROS node behavior.

Example:

```yaml
scenario:
  id: SCENARIO-0001
  type: sudden_pedestrian
  seed: 42

  parameters:
    ego_speed: 2.0
    pedestrian_speed: 1.0
    pedestrian_spawn_distance: 10.0
    trigger_distance: 8.0

  environment:
    pedestrian_start_y: -4.25
    pedestrian_end_y: 4.25
    update_rate_hz: 20.0

  termination:
    timeout: 30.0
```

`pedestrian_spawn_distance` is measured forward from the first observed ego
position. The resulting x coordinate is the fixed pedestrian crossing line.
The pedestrian starts moving when the remaining longitudinal distance from ego
to that line is at most `trigger_distance`.

The default `trigger_distance: 8.0` starts the crossing before the full-size
car arrives in the current 2 m/s demo. The pedestrian now moves at 1.0 m/s;
recheck the saved outcome when changing either speed or trigger distance.
The waypoint follower does not detect or avoid pedestrians; a non-collision
result at this setting is due to timing. Smaller trigger distances
can still produce collision cases for safety evaluation. This value is not a
universal safety threshold. The scenario manager commands the pedestrian through
`/safe_drive/gazebo/set_entity_state` and pauses if the service is unavailable.
Other trigger distances can be supplied through a separate scenario YAML file.

The road model has a zebra crossing centered at `x=-5 m`, aligned with the
default spawn position `x=-15 m` and default
`pedestrian_spawn_distance: 10 m`. Changing the spawn distance moves the
pedestrian crossing point but does not yet move the road paint. Align those
before using a different spawn distance for visual demonstrations.
The pedestrian is rendered as a fixed walking pose facing its +y travel
direction. Its gait is not animated; the scenario still moves the model pose
with the Gazebo entity-state service. The collision cylinder is unchanged.

Scenario states are published on `/safe_drive/scenario/state`:

```text
WAITING_FOR_EGO -> ARMED -> ACTIVE -> COMPLETED
                                  -> TIMED_OUT
```

The active scenario definition is published as JSON on
`/safe_drive/scenario/current`. Both topics use transient-local durability so a
late subscriber can receive the latest state.

Run the scenario with:

```bash
ros2 launch safe_drive_bringup scenario.launch.py
```

The separate `signal_intersection.launch.py` scene has a vehicle signal with
configurable red, green, and yellow durations in
`safe_drive_scenario/config/traffic_signal.yaml`. Set `fixed_state:=RED` or
`fixed_state:=GREEN` for a repeatable phase check. The node publishes
`std_msgs/msg/String` separately on `/safe_drive/vehicle_signal/state` and
`/safe_drive/pedestrian_signal/state`. The eastbound waypoint follower reads
only the vehicle topic, stops its front bumper before the `x=-9.25 m` line on
red or yellow, then proceeds on green. In cyclic mode, the pedestrian signal
stays `DONT_WALK` while the ego has no odometry; the signal cycle starts with
the first ego odometry message. This prevents the crossing from starting while
Gazebo is still spawning the car. Once the ego is ready, the pedestrian signal
shows WALK during the first 10.5 seconds of vehicle RED, then DONT_WALK during
the 1.5-second clearance and all vehicle GREEN/YELLOW time. Signal decisions
use ROS state and odometry, not a simulated camera. With `pedestrian:=true`,
the intersection spawns a pedestrian at `x=-5 m`, starts crossing only after
the published pedestrian signal is `WALK`, and continues to clear the road if
the indication changes to `DONT_WALK` while crossing. This is separate from
the distance-triggered `scenario.launch.py` behavior. The pedestrian signal
face at the south curb points north across the crosswalk; its passive lenses
and active lamps share that orientation. The signal crossing uses 1.0 m/s
from `y=-4.25 m` to `y=5.2 m` on the north sidewalk, giving a
nominal 9.45 s crossing. The separate distance-triggered crossing retains its
own configured endpoints. In the measured timed intersection, the pedestrian
reached the sidewalk before WALK ended and before vehicle GREEN.

A repeatable integrated test is `signal_intersection.launch.py
fixed_state:=RED spawn_x:=-30 pedestrian:=true
route_config:=fast_straight.yaml`. It saves crossing, safety, and red-stop
results through `pedestrian_result_file`, `pedestrian_safety_file`, and
`red_result_file`. The vehicle stops because of the vehicle RED topic; it
does not detect the pedestrian. With fixed GREEN, the pedestrian remains
`ARMED` on `DONT_WALK` and does not enter the crossing.

## Mapped steering routes

`signal_intersection.launch.py` accepts `route_config:=fast_straight.yaml`,
`right_turn.yaml`, or `left_turn.yaml`. The default is the right-turn route.
For a turning test, set `fixed_state:=GREEN`. `curve_course.launch.py` tests
a separate gentle road bend. These routes use configured map waypoints and
odometry. They do not detect painted lanes or pedestrians from sensors.
The right-turn route continues south to `(0.25, -30)` on the extended road.
The eastbound approach is long enough to increase speed before the compact
intersection. Right and left turns limit speed to 0.8 m/s at the corner.
`fast_straight.yaml` commands 17.0 m/s and reached about 14.0 m/s (50 km/h)
in two measured fixed-GREEN runs. Both passed the geometric lane-corridor
criterion after the steering-joint and straight-tracking changes. The earlier
20.8 m/s command reached 17.07 m/s (61 km/h) but failed that criterion.
Use saved results rather than treating the command value as achieved speed.
The eastbound light controls entry at `x=-9.25`; after crossing the line on
green, the vehicle completes the intersection even if that light turns red.

The route monitor writes a JSON result at `result_file`. It reports the
settled destination position and speed, centreline error, and the largest
distance from any of four chassis corners to the configured path. A sample
outside the 1.75 m half-width sets `corridor_clear: false`. This corridor is
an approximate lane-boundary test; markings at the intersection and road-edge
collisions are not used for the calculation. Stored samples and limitations are
listed in [validation/results.md](validation/results.md).

With `fixed_state:=RED`, the separate red-stop monitor writes JSON to
`red_result_file` after five simulated seconds of standing near the line.
It uses the vehicle-signal ROS topic and odometry; a front crossing of the
line produces an immediate failure result. Acceptance requires a front gap
of at least 0.6 m and speed below 0.15 m/s. This check is intended for a
fresh fixed-red launch, not a vehicle that entered on green before a phase
change.
