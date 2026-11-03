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
later runs at about 50 km/h passed after steering changes. A stationary
camera/LiDAR target case passed; broader repeated integrated runs remain
planned. See [sensors.md](sensors.md). Record
configuration, model/world version or revision, ROS/Gazebo versions, seed when
used, timestamp, measured outcomes, and simulator real-time factor. Save JSON
or CSV directly from runs; do not transcribe or fabricate results.

A connected-scene validation runner is implemented in
`scripts/run_connected_validation.py`, with cases in
`config/connected_validation.yaml`. It starts a fresh Gazebo process for each
case and stores actual JSON outcomes, launch logs, selected settings, input
file hashes, Git revision, and wall time. Two separate fixed-RED
signal-plus-pedestrian runs passed on 2026-11-03. That runner checks fixed
scene cases.

The `continuous_right` case adds one uninterrupted cyclic-signal mission:
red stop and pedestrian crossing first, then green departure and right-loop
completion. It records the independent stop, crossing, safety, route, and
painted-line outcomes from one Gazebo process. The current pedestrian clears
the crossing before green, so this is a safe integrated baseline rather than
a new close-conflict condition or a search-performance result.

The `continuous_late_crossing` case is a separate event-driven risk check.
During one run, the vehicle first holds at the red signal, then a pedestrian
enters against DONT_WALK when the green-moving ego is within a configured
3 m of the crossing. The case stops after the crossing outcome is measured;
its validity pass is distinct from the collision/risk classification and
does not imply that the route finished.

`scripts/run_late_crossing_sweep.py` uses
`config/late_crossing_sweep.yaml` to repeat this event at requested ego speeds
of 1.2 and 1.6 m/s and entry-distance thresholds of 2, 4, and 8 m. Each
combination starts a fresh Gazebo process and saves the exact generated route
and crossing YAML, input hashes, raw outcomes, actual ego speed at entry,
and a CSV row. The crossing starts only after the green-moving ego exceeds
0.3 m/s. The validity flag checks a moving GREEN/DONT_WALK entry and measured
outcomes; the risk flag uses contact or actor-center distance below 1.5 m.
Only one deterministic run per condition has been stored, so the matrix
illustrates parameter sensitivity in this simulator, not a probability.
The original matrix used a pose-driven static pedestrian with physical
contact force; this could shove the ego after contact and distort subsequent
distance and lane measurements. The revised collision surface keeps Gazebo
contact detection but disables contact force. Treat any new matrix as a
different simulator configuration and compare only like-for-like runs.
The opt-in `config/quality_demo.yaml` exercises a measured crossing stop and
a first-contact cutoff. Its pass criteria validate the demonstration behavior;
these cases are separate from the six-condition risk matrix and do not measure
search-method performance.

`scripts/run_speed_validation.py` and `config/speed_validation.yaml` now
repeat selected connected-map cases at configured requested speeds. The runner
copies route YAML per speed, changing target and maximum speed together, and
records the actual maximum speed plus route-corridor or fixed-red stop-line
measurements. Initial 1.2 and 1.6 m/s right-loop checks and a 1.6 m/s red
stop check have stored results. A speed request is not evidence that the car
actually reached that speed. The current 1.75 m corridor is distance from
chassis corners to a route polyline. Connected-road runs now additionally
save independent centerline and lane-edge overlap measurements from the
generated map's painted intervals. The checker samples chassis corners from
odometry on painted straight arms and excludes unpainted junction gaps.
For a continuous mission, the summary reports whole-route maximum speed and
separate maximum speed during the initial red approach. The fixed-red case has
only the latter. Recorded single runs at requested 4 and 6 m/s passed for the
straight loop, fixed-red stop, and continuous right mission after route-length
braking and straight-loop bend limiting. Achieved straight-loop maxima were
3.321 and 4.921 m/s; the 6 m/s fixed-red approach achieved only 2.535 m/s.
These are controller and scene checks, not search-method comparisons.

The fixed sudden-pedestrian sweep in
`scripts/run_pedestrian_sweep.py` varies the sudden-pedestrian trigger
distance and stores each run's YAML, seed, raw safety JSON, JSON summary, and
CSV. Its predefined exploratory label is collision or measured actor-center
distance below 3.0 m. The current scenario draws no random values, so the
seed is recorded but has no behavioral effect. Neither validation runner
measures a true simulator real-time factor.

The separate measured comparison is implemented by
`scripts/run_risk_exploration.py` and `config/risk_exploration.yaml`. The
configuration fixes three parameter ranges, a crossing position, a collision
or actor-center distance risk criterion, two selection seeds, and four runs
per method per seed. Both selectors begin at the same center point. Random
uses seeded continuous uniform draws; Adaptive divides a measured region
into eight cells and gives priority to children of collision or smaller
clearance. Each run starts a fresh Gazebo process and saves its exact scenario
YAML, launch log, raw safety JSON, seed, selected point, selection order,
classification, wall time, and Gazebo real-time-factor samples. A missing
measurement remains `unknown` and
consumes a run budget. This is an executable small-budget demonstration, not
evidence of which method is generally better. The Adaptive tie order is
deterministic; the current scenario seed affects selection only for Random.
The real-time-factor median is a per-run simulator performance diagnostic,
not a risk score. The first 16-run comparison predates its collection; the
sensor-equipped repeat collected it for every run. See
[validation/results.md](validation/results.md).

Possible result fields include `scenario_id`, `case`, `seed`,
`route_completed`, `stop_line_violation`, `collision`,
`min_distance_m`, `min_ttc_s`, `camera_available`,
`lidar_available`, `real_time_factor`, and `result`.
The existing minimum distance measures actor centers, and existing TTC uses
line-of-sight closing speed; preserve those definitions in results.

These tests validate behavior in a defined simulator scene. They do not
estimate real-world accident probability or real-vehicle sensor performance.

## Small implemented comparison and planned larger study

The implemented small comparison is Random Scenario Sampling versus
Risk-Guided Adaptive Grid Refinement. It uses one sudden-pedestrian scenario
space whose current
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

The fixed sweep validates the scene and measurement pipeline. The new
comparison records risky conditions discovered per run and the first risky
run under equal budgets, ranges, controller, environment, metrics, and
initialization. Larger budgets and independent scenario variation are needed
before an efficiency claim; the present pedestrian scenario has no random
draw and the default budget is only four runs per method and seed.

### Optional live dashboard for a scenario-search demonstration

The search backend is implemented. Runs may look similar in Gazebo while search
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
