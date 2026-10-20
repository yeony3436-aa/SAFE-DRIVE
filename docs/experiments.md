# Risk-Scenario Exploration and Validation

The project topic is risk-scenario exploration. The current implementation
priority is to validate the ROS 2 / Gazebo scene so each later search run has
repeatable conditions and measured outcomes. Scene-validation results are
prerequisites, not evidence that a search algorithm already works.

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
[validation/results.md](validation/results.md). The earlier fast straight run
reached about 61 km/h but failed its geometric lane-corridor threshold; two
later runs at about 50 km/h passed after steering changes. Camera/LiDAR cases
and repeated integrated runs remain planned. Record
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

## Planned core study: scenario exploration

The planned comparison is Random Scenario Sampling versus Risk-Guided Adaptive
Grid Refinement. Start with one sudden-pedestrian scenario space whose current
YAML already exposes the following tunable values:

```text
S = (ego_speed, trigger_distance, pedestrian_speed)
```

Keep the crossing point (`pedestrian_spawn_distance`) fixed for the first
comparison so the physical road geometry is unchanged. The trigger distance
controls when the pedestrian enters relative to the ego's approach. Define
parameter ranges and a risk criterion before running the comparison. Collision
and measured actor-center minimum separation are primary candidates. The
existing TTC is a line-of-sight value, not a path-intersection predictor, so report it as a
separate diagnostic rather than an unqualified danger score.

The current waypoint controller does not react to pedestrian observations.
Initial search results therefore describe the risk of this fixed controller
under the configured pedestrian conditions, not pedestrian-aware avoidance.

First add a repeat runner that launches one configuration per run and stores
the parameter set, seed, timestamp, simulator revision, raw measurements, and
outcome. Then implement Random Sampling as a baseline and Adaptive Grid
Refinement as the proposed method. Give both methods the same ranges,
simulation budget, controller, environment, metrics, initialization rules,
and recorded seeds. Compare risky conditions discovered per run and the number
of runs needed to find the first one. Repeat with multiple seeds before making
an efficiency claim. None of this experiment loop or comparison is implemented
yet; current JSON files validate individual scene behaviors.

### Optional live dashboard for a scenario-search demonstration

The scenario-search backend is not implemented yet. After the planned study's
repeat runner is working, runs may look similar in Gazebo even while the search
covers different parameter combinations. Show the simulator beside a live dashboard
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
