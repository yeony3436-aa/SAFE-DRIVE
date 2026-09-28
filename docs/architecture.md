# Architecture

SAFE-Drive should grow from a small reproducible research core.

Initial architecture:

```text
Gazebo
  -> Ego Vehicle
  -> ROS 2 Control
  -> Waypoint Follower
  -> Vehicle State / Odometry
```

Later Tier 1 architecture:

```text
Scenario Config
  -> Scenario Manager
  -> Gazebo Actors
  -> Safety Monitor
  -> Experiment Manager
  -> Result Files
```

The dashboard, heatmap, replay, perception, and natural-language command features are presentation or optional extensions. They should not block the Tier 1 experiment pipeline.

## Initial ROS 2 Packages

Create packages only when they are needed.

Recommended order:

1. `safe_drive_bringup`: launch files
2. `safe_drive_description`: ego vehicle description
3. `safe_drive_sim`: Gazebo worlds and spawn logic
4. `safe_drive_control`: waypoint following and speed control
5. `safe_drive_scenario`: sudden pedestrian scenario execution
6. `safe_drive_safety`: TTC, distance, collision, risk score
7. `safe_drive_experiment`: random and adaptive experiment execution

## Current MVP-01 Boundary

`safe_drive_description` contains the ego model only: links, wheel joints,
inertial values, visual geometry, and collision geometry. It has no Gazebo
plugins or control logic. This keeps the model reusable while the next package,
`safe_drive_sim`, owns the Gazebo world and spawning integration.

`safe_drive_bringup/simulation.launch.py` is the current one-command entry
point. It delegates to `safe_drive_sim`, which starts the minimal road world,
publishes the ego vehicle description, and spawns `safe_drive_ego`.

`safe_drive_bringup/vehicle.launch.py` adds `safe_drive_control`. The waypoint
follower consumes `/safe_drive/ego/odom`, publishes `/safe_drive/ego/cmd_vel`,
and commands a persistent stop after reaching the final configured waypoint.
