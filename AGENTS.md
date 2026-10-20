# SAFE-Drive — Codex Development Instructions

## 1. Project direction

SAFE-Drive is a ROS 2 / Gazebo graduation project about **risk-scenario exploration for autonomous driving**. Its first implementation priority is a working, visually credible simulation environment that can reproduce traffic situations and measure vehicle outcomes.

Working title: **ROS 2/Gazebo-based Autonomous Driving Risk-Scenario Exploration**.

The environment is the prerequisite; the research goal is to vary defined scenario parameters, measure outcomes, and find risky conditions under a fixed simulation budget. Random Sampling is the baseline and Risk-Guided Adaptive Grid Refinement is the planned comparison in `docs/experiments.md`. Neither search method nor a comparative result is implemented yet. Do not present environment validation as search performance.

Keep the project demonstrable at each milestone. Prefer a small road scene whose vehicle, crossing, signal, pedestrian, and sensors work together over a large decorative city.

## 2. Graduation-critical scope

The core demonstration should provide:

1. One command that reliably launches the simulator and required ROS 2 nodes.
2. A recognizable ego vehicle on a road with lane markings, sidewalk, crossing, and traffic signal. Visual and collision geometry must both work.
3. A vehicle that follows a predefined route, stops before a red signal, and proceeds when permitted.
4. A pedestrian that visibly crosses under a configured trigger or crossing phase, with at least one verified vehicle response.
5. Camera and LiDAR data published to ROS 2 with documented frames, rates, and usable sample output.
6. Odometry and independently measured route completion, signal compliance, collision, and minimum pedestrian distance. Retain the existing TTC metric with its limitations.
7. At least one repeatable, configuration-driven validation run that saves results in a machine-readable file.
8. A repeatable experiment loop that records each parameter set, seed, measured outcome, and risk classification, then compares search methods under equal budgets.

These are targets, not claims of current completion. `README.md` and `docs/development-log.md` distinguish implemented, verified, and planned work.

**Radar:** desirable if feasible, but not a prerequisite. Define its intended output, verify simulator support, and label an approximation as such. Do not present ground-truth-derived range and radial velocity as a physically faithful radar return.

## 3. Current baseline and priorities

The repository currently includes a Gazebo Classic road world, simplified vehicle and crossing visuals, waypoint following and destination stop, a configurable sudden-pedestrian scenario, and ground-truth distance/TTC/contact reporting. Verify actual behavior before changing it; some older log results were superseded by bug fixes.

Work in this order unless the user requests otherwise:

1. Preserve reliable launch, vehicle motion, odometry, and scenario behavior.
2. Improve the existing vehicle, road, crossing, and pedestrian models for a credible demonstration. Check third-party asset license, format, scale, and performance.
3. Add a visible traffic signal with ROS 2 state output; implement stop/go and pedestrian timing.
4. Define a small, measurable risk-scenario parameter space and pass its values through configuration without breaking the integrated scene.
5. Record and repeat scenario runs, saving raw outcomes, seeds, configuration, and simulation performance.
6. Implement and compare Random Sampling and the planned adaptive search with equal budgets and a predefined risk criterion.
7. Add and validate a forward camera, then LiDAR, with ROS 2 topics and frames. Connect sensor observations to one limited behavior where feasible, distinguishing them from simulator ground truth.
8. Consider an explicitly modeled radar approximation, dashboard, richer roads, or perception algorithms after the experiment loop is stable.

Avoid YOLO, LLM, voice input, large custom city maps, and complex planners as prerequisites. Keep the initial adaptive search small and measurable.

## 4. Milestones

- **MVP-01 — Vehicle baseline (implemented):** Gazebo launch, ego spawn, ROS 2 command and odometry, waypoint following, destination stop.
- **MVP-02 — Pedestrian baseline (implemented):** configured crossing and reproducible state transitions.
- **MVP-03 — Safety observation (partly implemented):** center distance, line-of-sight TTC, contact, and completion. Selected validation runs save JSON results; a general repeat runner remains pending.
- **MVP-04 — Integrated traffic scene (in progress):** road users and traffic signal run together; selected stop-line, signal transition, and pedestrian cases have been verified. Continue checking visual and behavioral consistency.
- **MVP-05 — Repeatable experiments:** run defined parameter sets from configuration, save actual outcomes and seeds, and summarize pass/fail and limitations.
- **MVP-06 — Risk exploration:** define a risk threshold before runs, implement random and adaptive condition selection, and compare discoveries under equal budgets.
- **MVP-07 — Sensor-equipped scene:** camera and LiDAR publish usable ROS 2 data; frames and rates are verified. Evaluate radar separately.

A realistic exterior alone does not complete MVP-04; behavior must be verified.

## 5. Simulator and model policy

The current environment is ROS 2 Humble with Gazebo Classic 11. Do not change versions just for appearance. Gazebo Classic has reached end of life; evaluate migration separately when a concrete compatibility or maintenance need arises, preserving the working baseline.

Prefer compatible existing worlds and reusable models, then targeted edits to the current world. Keep simple collision meshes even when visual meshes are detailed. Verify wheel contact, vehicle footprint, pedestrian collision, and signal placement after visual changes. Maintain an asset inventory with source and license when adding third-party assets.

The initial scene needs one road segment, crossing, stop line, signal, ego vehicle, and pedestrian. Expansion to a city is optional. Keep crossing geometry aligned with scenario positions; the current painted crossing is fixed near `x=-5 m`.

The default vehicle now uses front-wheel Ackermann steering with rear-wheel drive through the Gazebo Classic Ackermann plugin. The same Prius visual meshes remain, and the old skid-steer plugin is selectable for comparison. Preserve verified straight, signal-stop, pedestrian, and turning behavior when changing vehicle dynamics.

## 6. Traffic behavior and scenarios

Represent scenarios and tunable values as YAML or ROS parameters, separate from execution logic. Keep the existing `sudden_pedestrian` scenario working. Do not create hardcoded nodes for each parameter combination.

Traffic-signal state must have one source of truth for visible lamps, ROS output, and vehicle decisions. Define transitions, crossing permissions, startup, and reset explicitly. Test green progression, red approach, and pedestrian presence separately before combining them. Stop before the configured stop line on red.

Record configuration, seed where applicable, timestamps, and outcome. Do not describe a timed crossing as signal-aware until it actually responds to signal state.

## 7. Sensors and measurements

Add camera and LiDAR as actual simulated sensors. Verify message type, topic, frame, rate, timestamps, orientation, and visibility of a known target. Profile real-time factor before raising resolution or update rates.

Keep Gazebo ground truth for validation and collision/TTC measurement. Document whether each driving decision uses camera, LiDAR, a radar approximation, or ground truth. Do not claim autonomous perception if the controller reads actor poses directly.

Use SI units internally: meters, meters/second, radians, seconds. Document transforms between `map`, `odom`, `base_link`, `camera_link`, and `lidar_link` as applicable. Convert km/h and degrees explicitly.

The existing TTC is line-of-sight distance divided by positive closing speed. It is not a path-intersection predictor or body-surface clearance. Preserve raw measurements separately from derived pass/fail or risk scores.

## 8. Validation and research claims

The research question is: **Under the same simulation budget, which search method finds more predefined risky traffic conditions?** First validate that the constructed environment reproducibly exercises and measures vehicle behavior; search results must use measured outcomes, not screenshots alone.

Initial cases: open-road route completion, red-signal stop, green-signal progress, pedestrian crossing interaction, and sensor publication with a visible target. Save configuration and actual results in JSON or CSV. Useful measures include route completion, stop-line violation, collision, minimum distance, TTC, sensor availability/rate, and simulator real-time factor. Define acceptance thresholds before reporting outcomes.

Do not infer real-world accident probability or real-vehicle sensor performance from these runs. Compare Random and Adaptive using equal budgets, ranges, controller, environment, metrics, and recorded seeds. The integrated scene and repeatable measurements must work before making search-performance claims.

## 9. Architecture and implementation

Keep ROS 2 packages focused. Existing responsibilities: `safe_drive_description` (vehicle model), `safe_drive_sim` (world and Gazebo integration), `safe_drive_control` (waypoint follower), `safe_drive_scenario` (pedestrian), `safe_drive_safety` (measurements), and `safe_drive_bringup` (launch). Add packages only when needed; avoid placeholders and giant nodes.

Use ROS messages or services across modules. Keep calculations and state machines separate from callbacks where practical. Prefer clear names under `/safe_drive/`; document actual topic types and frames. Put speeds, positions, sensor settings, thresholds, and signal timing in configuration.

Dashboard, event replay, heatmap, YOLO, natural-language commands, and voice input are optional. A dashboard displays measured results; it does not calculate safety metrics. An LLM must never directly produce steering, throttle, or brake commands.

## 10. Verification workflow

Inspect the repository and installed ROS/Gazebo versions first. Preserve working behavior and uncommitted changes. Build affected packages, run focused tests, launch the relevant scene where possible, inspect errors and topics, and verify simulated behavior. A successful parse or build alone does not prove functionality.

Typical commands from `ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <affected_packages>
source install/setup.bash
colcon test --packages-select <affected_packages>
colcon test-result --verbose
```

Avoid destructive cleanup of `build/`, `install/`, or `log/` without a concrete reason. Add an integrated `demo.launch.py` only after its parts work separately.

## 11. Coding and documentation

Use descriptive English identifiers, Python type hints for important interfaces, small ROS callbacks, and maintainable C++ ownership. Avoid unnecessary dependencies. Throttle per-tick logs; make signal changes, scenario starts, violations, contacts, and completion easy to find.

Maintain `README.md`, `docs/architecture.md`, `docs/scenarios.md`, `docs/experiments.md`, and `docs/development-log.md`. Distinguish **implemented**, **verified**, **in progress**, and **planned**. Preserve historical log entries; append corrections when past measurements are invalidated. Generate claims and charts only from stored data.

For development requests: read this file, inspect relevant code and status, implement the smallest useful milestone, build or test affected code, then report changes, actual verification, and unresolved issues. Existing working code takes precedence over a speculative redesign. Choose reasonable defaults for reversible details.
