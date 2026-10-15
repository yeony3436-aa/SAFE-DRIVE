# Architecture

SAFE-Drive's primary deliverable is a working ROS 2 / Gazebo autonomous-driving
simulation environment. The scene should connect a recognizable ego vehicle,
road and crossing, traffic signal, pedestrian, and simulated sensors. Defined
traffic situations validate its behavior; scenario-search research is optional.

## Current implementation

```text
Gazebo Classic road + crossing + visual roadside props
  ├── ego vehicle (Prius visual mesh, skid-steer diff-drive)
  │   └── odometry → waypoint follower → velocity command
  ├── sudden pedestrian (configured trigger and movement)
  └── model states + contact → safety monitor → live JSON result

Separate compact intersection launch:
  traffic-signal timer → ROS String state → waypoint speed limit
                       └→ Gazebo visible bulb positions
```

- `safe_drive_description`: simulator-independent vehicle model.
- `safe_drive_sim`: world, Gazebo vehicle plugins, and spawning.
- `safe_drive_control`: route following and destination stop.
- `safe_drive_scenario`: structured pedestrian definition, ROS scenario manager,
  and configurable traffic-signal timer.
- `safe_drive_safety`: center distance, line-of-sight TTC, contact, and completion.
- `safe_drive_bringup`: launch composition.

The Prius exterior and selected MCity surface textures are visual-only assets;
their source and license are recorded in [the asset inventory](asset-inventory.md).
The ego keeps its simple body and wheel collisions. The road contact surface,
crossing axis, and pedestrian collision cylinder are unchanged. The pedestrian
now has a human-shaped fixed walking pose but no articulated gait animation.

The scenario manager moves the pedestrian through
`/safe_drive/gazebo/set_entity_state` and pauses progression if that service
is unavailable. The diff-drive odometry source is WORLD so vehicle and actor
coordinates match. `scenario.launch.py` passes the configured `ego_speed`
to the waypoint follower. Safety results are published on
`/safe_drive/safety/result`; they are not yet saved to files.

`signal_intersection.launch.py` uses a separate four-way road world. One phase
clock drives the roadside three-aspect vehicle signal and curbside two-aspect
pedestrian signal. It publishes `/safe_drive/vehicle_signal/state` and
`/safe_drive/pedestrian_signal/state` separately. The waypoint follower
subscribes only to the vehicle topic, and limits speed using odometry, the
`x=-9.25 m` stop line, and the vehicle's 2.1 m front offset. Missing or stale
vehicle signal data commands a stop before the line. The route is straight
through the intersection; pedestrian actor timing is not linked to the
pedestrian signal yet. Geometry is documented in [road layout](road-layout.md).

## Target integration

```text
Scenario configuration ───────────────┐
Traffic-signal state ─→ visible signal ├─→ vehicle behavior
Pedestrian state ──────→ visible actor ┘
Camera + LiDAR ────────────────────────→ ROS 2 sensor topics
Gazebo ground truth + contact ─────────→ validation metrics
Vehicle and scenario outcomes ─────────→ saved validation results
```

The signal must have one state source for its appearance, ROS output, and
controller decisions. A crossing scenario should explicitly define whether it
is triggered by distance, signal phase, or both. The controller's use of sensors
versus simulator ground truth must be stated precisely. The current pedestrian
controller is not signal-aware. The waypoint follower uses the signal topic
but is not sensor-based.

Keep current packages stable and add packages only when implementation warrants
them. Camera and LiDAR belong on the ego model with documented frames, ROS
topics, message types, and rates. Evaluate radar separately: a ground-truth
range/velocity approximation is not a physical radar simulation.

## Validation boundary

Use simulator ground truth to check behavior and calculate metrics. Report
route completion, red stop-line compliance, collision, pedestrian clearance,
sensor availability, and simulator performance from actual runs. Keep raw
measurements separate from pass/fail results. A dashboard may later display
these results, but does not calculate them.

Current platform: ROS 2 Humble and Gazebo Classic 11. Gazebo migration is a
separate decision and should preserve the working baseline.
