# Validation and Optional Research

The primary goal is to validate the constructed ROS 2 / Gazebo driving scene.
The experiment is evidence that traffic elements, vehicle behavior, and sensors
work together under defined, repeatable conditions.

## Core validation cases

| Case | Intended observation |
|---|---|
| Open road | Ego follows the configured route and stops at its goal. |
| Red signal | Ego stops before the configured stop line. |
| Green signal | Ego proceeds when permitted. |
| Pedestrian crossing | Pedestrian moves visibly; collision and clearance are measured; vehicle response is recorded. |
| Sensor target | Camera and LiDAR ROS topics publish timestamped data with correct frames and a visible target. |

Fixed red and green vehicle-signal behavior has been observed in Gazebo. Four
stored green route runs and their predefined geometric acceptance criteria are
in [validation/results.md](validation/results.md). Signal-aware pedestrian
motion and camera/LiDAR cases remain planned. Record
configuration, model/world version or revision, ROS/Gazebo versions, seed when
used, timestamp, measured outcomes, and simulator real-time factor. Save JSON
or CSV directly from runs; do not transcribe or fabricate results.

Possible result fields include `scenario_id`, `case`, `seed`,
`route_completed`, `stop_line_violation`, `collision`,
`min_distance_m`, `min_ttc_s`, `camera_available`,
`lidar_available`, `real_time_factor`, and `result`.
The existing minimum distance measures actor centers, and existing TTC uses
line-of-sight closing speed; preserve those definitions in results.

These tests validate behavior in a defined simulator scene. They do not
estimate real-world accident probability or real-vehicle sensor performance.

## Optional follow-up: scenario exploration

The original research proposal compared Random Scenario Sampling with
Risk-Guided Adaptive Grid Refinement on a defined pedestrian scenario space:

```text
S = (ego_speed, pedestrian_spawn_distance, pedestrian_speed)
```

This comparison is no longer a core milestone. If explicitly resumed, give
both methods the same ranges, simulation budget, controller, environment,
metrics, initialization, and recorded seeds. Store raw measurements apart from
derived risk scores and make claims only from saved experiment data. Do not
build an experiment framework or risk heatmap ahead of the integrated traffic
scene and its basic validation.
