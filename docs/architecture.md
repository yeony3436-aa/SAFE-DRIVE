# Architecture

SAFE-Drive's research topic is autonomous-driving risk-scenario exploration.
The first deliverable is a working ROS 2 / Gazebo simulation environment that
can run defined traffic conditions and measure outcomes reproducibly. The scene
connects a recognizable ego vehicle, road and crossing, traffic signal, and
pedestrian, a simulated camera and LiDAR. A small measured search
loop now runs over the sudden-pedestrian scene.

## Current implementation

```text
Gazebo Classic road + crossing + visual roadside props
  ├── ego vehicle (Prius visual mesh, front-steered Ackermann model)
  │   └── odometry → waypoint follower → velocity command
  │   └── forward camera + planar LiDAR → ROS sensor topics
  ├── sudden pedestrian (configured trigger and movement)
  ├── model states + contact → safety monitor → live JSON result
  └── odometry + route geometry → route monitor → saved JSON result

Separate compact intersection launch:
  traffic-signal timer → ROS String state → waypoint speed limit
                       └→ Gazebo visible bulb positions
                       └→ signal crossing pedestrian on WALK
  RED state + ego odometry → red-stop monitor → saved JSON result
```

- `safe_drive_description`: simulator-independent vehicle model.
- `safe_drive_sim`: world, Gazebo vehicle plugins, and spawning.
- `safe_drive_control`: route following and destination stop.
- `safe_drive_scenario`: structured sudden-pedestrian definition, signal
  crossing actor, and configurable traffic-signal timer.
- `safe_drive_safety`: center distance, line-of-sight TTC, contact, route
  completion, and fixed-red stop-line measurement.
- `safe_drive_bringup`: launch composition.

The vehicle's `base_footprint` is the ground projection; `base_link` is 0.75 m
above it. The four wheel links carry the visual and collision wheels, and the
front wheels have steering pivot links in the Ackermann model. The default
Prius meshes are visual; the body collision remains a simple box. The earlier
skid-steer layout is selectable with matching model and controller settings.
With the Gazebo Ackermann plugin, commanded `Twist.angular.z` is tyre steering
angle, while odometry `twist.angular.z` is measured yaw rate. The follower
converts between them using the configured 2.5 m wheelbase.
The steering pivot links have added inertia and damping, and the Gazebo
steering-joint PID gains were tuned against measured wheel angles. This
reduced lateral drift in the stored 50 km/h straight runs; it is still a
simulation-specific vehicle model, not a calibrated real-car dynamics model.

The revised connected right loop uses an optional path-heading correction
and curvature-based speed profile. The profile limits speed in bends using a
configured lateral-acceleration bound and propagates braking limits backward
along waypoints. Both settings default to disabled on other routes. The
speed-validation runner copies a route YAML per requested speed and records
the actual maximum speed plus independent route, red-stop, and connected-road
painted-line measurements. The line monitor uses the generated marking
inventory and odometry-derived chassis corners. It reports line overlap on
painted straight arms; junction gaps have no painted line to check.

The Prius exterior and selected MCity surface textures are visual-only assets;
their source and license are recorded in [the asset inventory](asset-inventory.md).
The road contact surface and pedestrian collision cylinder are unchanged. The
pedestrian has a fixed walking pose without articulated gait animation.

The Gazebo vehicle carries a 320×240 forward camera and a 360-sample planar
LiDAR, both at 10 Hz by default. ROS topics, fixed frames, launch parameters,
and a static target validation are documented in [sensors](sensors.md).
Gazebo publishes a real-time-factor sample on `/performance_metrics`; the
sensor validation captures it. Neither sensor currently feeds the controller.

The scenario manager moves the pedestrian through
`/safe_drive/gazebo/set_entity_state` and pauses progression if that service
is unavailable. The Ackermann plugin reports Gazebo world coordinates in
odometry so vehicle and actor coordinates match. `scenario.launch.py` passes
the configured `ego_speed`
to the waypoint follower. Pedestrian safety results are published on
`/safe_drive/safety/result`; the separate sudden-pedestrian launch can save
them through `safety_result_file`, as can the integrated signal-crossing
launch. The route monitor
stores separate JSON files for map-based path and chassis-corner checks.
The top-level `scripts/run_connected_validation.py` launches one connected
scene case per fresh Gazebo process and collects result files into a per-case
record with input hashes and a summary. Its `continuous_right` case keeps one
process running through an initial cyclic-signal red stop, pedestrian
crossing, green departure, right turn, and route completion. The opt-in
red-stop monitor records the red hold without forcing the signal to stay red;
the route, safety, and painted-line monitors record independent outcomes in
that same process. Its YAML case list is
`config/connected_validation.yaml`. It does not select risk-scenario parameters
or implement either search method.

`scripts/run_risk_exploration.py` launches a fresh scenario for each selected
parameter set. It uses the same measured risk classifier and Gazebo launch path
for seeded uniform Random Sampling and a deterministic, collision/clearance
prioritized octree search. The top-level config fixes both methods' parameter
bounds, equal budgets, seeds, and risk criterion. Raw safety JSON, scenario
YAML, per-run records, and aggregate JSON/CSV are retained. The controller
does not react to pedestrian observations, so these measurements characterize
that fixed controller in this scene.

`signal_intersection.launch.py` uses a separate four-way road world. One phase
clock drives the roadside three-aspect vehicle signal and curbside two-aspect
pedestrian signal. It publishes `/safe_drive/vehicle_signal/state` and
`/safe_drive/pedestrian_signal/state` separately. The waypoint follower
subscribes only to the vehicle topic, and limits speed using odometry, the
`x=-9.25 m` stop line, and the vehicle's 2.1 m front offset. Missing or stale
vehicle signal data commands a stop before the line. Straight, right-turn, and
left-turn waypoint files use the same intersection world. The independent
`curve_course.world` exercises a gentle bend. The optional intersection
pedestrian starts only from the published `WALK` indication and continues
through clearance after entry. Geometry is documented in
[road layout](road-layout.md).
The `world_file:=connected_road.world` launch option selects a 3×3 connected
road grid that reuses this central signal and crossing. Three configured
routes exercise a right turn, left turn, or straight movement and return to
the start. Only the central eastbound approach has controlled stop/go logic;
the other junctions are unsignalized. The route monitor arms loop completion
after the vehicle first leaves the goal area.
At the outer east-side junction, `unsignalized_right_turn.launch.py` runs the
pedestrian scenario without a vehicle signal node. `unsignalized_yield` maps
scenario state to `/safe_drive/ego/yield_required` (`std_msgs/msg/Bool`). The
waypoint follower applies a configured speed cap before the yield line, and
`unsignalized_yield_monitor` independently records stop duration, minimum
front gap, and entry after the crossing clears. This is a limited
scenario-state-driven response, not camera or LiDAR perception.
For a fixed-red run, the red-stop monitor subscribes independently to the
vehicle signal and odometry, then saves a stop-line JSON result after a stable
stop. It does not control the vehicle.
The pedestrian signal crossing node saves its start signal and completion
time; the safety monitor can save independent actor-center distance, TTC,
and contact results. The ego responds only to the vehicle signal, not to
observed pedestrian position or camera/LiDAR data.
The bend world is generated from
`ros2_ws/src/safe_drive_sim/scripts/generate_curve_course.py` and uses simple
road collision segments. The simulator launch preserves Gazebo's built-in
model and shader paths alongside the local materials.

## Target integration

```text
Scenario configuration ───────────────┐
Traffic-signal state ─→ visible signal ├─→ vehicle behavior
Pedestrian state ──────→ visible actor ┘
Camera + LiDAR ────────────────────────→ ROS 2 sensor topics
Gazebo ground truth + contact ─────────→ validation metrics
Vehicle and scenario outcomes ─────────→ saved validation results
Configured parameter set + seed ───────→ repeat runner
Saved outcomes ─────────────────────────→ risk criterion + next search point
```

The signal must have one state source for its appearance, ROS output, and
controller decisions. A crossing scenario should explicitly define whether it
is triggered by distance, signal phase, or both. The controller's use of sensors
versus simulator ground truth must be stated precisely. The intersection
pedestrian is signal-aware; the separate sudden-pedestrian scenario remains
distance-triggered. The waypoint follower uses the vehicle signal topic but
is not sensor-based.

Keep current packages stable and add packages only when implementation warrants
them. Evaluate radar separately: a ground-truth
range/velocity approximation is not a physical radar simulation.

## Validation and search boundary

Use simulator ground truth to check behavior and calculate metrics. Report
route completion, red stop-line compliance, collision, pedestrian clearance,
sensor availability, and simulator performance from actual runs. Keep raw
measurements separate from pass/fail results. A dashboard may later display
these results, but does not calculate them.
The fixed-signal and crossing records validate scene components. The separate
small search loop varies a sudden-pedestrian parameter space and passes
measured results to Random Sampling or an adaptive selector. Its default
budget is a functional check; a research comparison needs larger budgets,
more independent seeds, and simulator performance measurement.

Current platform: ROS 2 Humble and Gazebo Classic 11. Gazebo migration is a
separate decision and should preserve the working baseline.
