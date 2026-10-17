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
    pedestrian_speed: 1.5
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

The default `trigger_distance: 8.0` starts the crossing early enough for the
pedestrian to clear the ego lane before the full-size car arrives in the current
2 m/s demo. The waypoint follower does not detect or avoid pedestrians; a
non-collision result at this setting is due to timing. Smaller trigger distances
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
shows WALK during the first seven seconds of vehicle RED, then DONT_WALK during
the five-second clearance and all vehicle GREEN/YELLOW time. Signal decisions
use ROS state and odometry, not a simulated camera. The pedestrian actor
scenario remains distance-triggered and is not yet coordinated with this phase.

## Mapped steering routes

`signal_intersection.launch.py` accepts `route_config:=controller.yaml`,
`right_turn.yaml`, or `left_turn.yaml`. The default is the straight route.
For a turning test, set `fixed_state:=GREEN`. `curve_course.launch.py` tests
a separate gentle road bend. These routes use configured map waypoints and
odometry. They do not detect painted lanes or pedestrians from sensors.

The route monitor writes a JSON result at `result_file`. It reports the
settled destination position and speed, centreline error, and the largest
distance from any of four chassis corners to the configured path. A sample
outside the 1.75 m half-width sets `corridor_clear: false`. This corridor is
an approximate lane-boundary test; markings at the intersection and road-edge
collisions are not used for the calculation. Stored samples and limitations are
listed in [validation/results.md](validation/results.md).
