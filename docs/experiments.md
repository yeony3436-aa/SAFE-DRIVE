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

Fixed red and green vehicle-signal behavior, faster turn routes, and one
signal-coordinated pedestrian crossing have stored results in
[validation/results.md](validation/results.md). The fast straight run reached
about 61 km/h but failed its geometric lane-corridor threshold. Camera/LiDAR
cases and repeated integrated runs remain planned. Record
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

### Optional live dashboard for a scenario-search demonstration

The scenario-search backend is not implemented yet. If that follow-up study is
started, repeated runs may look similar in Gazebo even while the search covers
different parameter combinations. Show the simulator beside a live dashboard
so an audience can see what is being tested and what the measurements show.

The dashboard could show:

- A grid heatmap of two selected scenario parameters, with the third fixed or
  filtered. Distinguish untested cells from measured cells and display the
  recorded outcome or risk score for each visited cell.
- The current run's measured TTC over simulation time. Mark periods when TTC
  is undefined rather than drawing an invented value; label it as the existing
  line-of-sight TTC, not a collision prediction.
- The cumulative count of scenarios meeting a risk threshold defined before
  the run, alongside completed runs and the total search budget.

Each update must come from a saved or streamed run record containing the
configuration, seed, timestamp, raw TTC and collision measurements, and
derived outcome. The search backend computes metrics and risk; the dashboard
only displays them. Plotly or Streamlit are possible implementation choices
after the repeatable validation and search backend exist. A saved-run replay
can support a reliable demo, but it must be labeled as replay rather than live
simulation.
