# SAFE-Drive — Codex Development Instructions

## 1. Project direction

SAFE-Drive is a ROS 2 / Gazebo graduation project. Its primary goal is to build a **working, visually credible autonomous-driving simulation environment** and use it to verify vehicle behavior in reproducible traffic situations.

Working title: **ROS 2/Gazebo-based Autonomous Driving Simulation Environment and Traffic-Scenario Validation**.

The environment is the main deliverable. Experiments provide evidence that its components work; a novel scenario-search algorithm is not required for the core project. The previous Random Sampling vs. Risk-Guided Adaptive Grid Refinement proposal remains an optional follow-up study in `docs/experiments.md`.

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

These are targets, not claims of current completion. `README.md` and `docs/development-log.md` distinguish implemented, verified, and planned work.

**Radar:** desirable if feasible, but not a prerequisite. Define its intended output, verify simulator support, and label an approximation as such. Do not present ground-truth-derived range and radial velocity as a physically faithful radar return.

## 3. Current baseline and priorities

The repository currently includes a Gazebo Classic road world, simplified vehicle and crossing visuals, waypoint following and destination stop, a configurable sudden-pedestrian scenario, and ground-truth distance/TTC/contact reporting. Verify actual behavior before changing it; some older log results were superseded by bug fixes.

Work in this order unless the user requests otherwise:

1. Preserve reliable launch, vehicle motion, odometry, and scenario behavior.
2. Improve the existing vehicle, road, crossing, and pedestrian models for a credible demonstration. Check third-party asset license, format, scale, and performance.
3. Add a visible traffic signal with ROS 2 state output; implement stop/go and pedestrian timing.
4. Add and validate a forward camera, then LiDAR, with ROS 2 topics and frames.
5. Connect sensor observations to one limited driving behavior where feasible. Distinguish sensor-based decisions from simulator ground truth.
6. Record and repeat validation scenarios; measure behavior and simulation performance.
7. Consider an explicitly modeled radar approximation.
8. Consider a dashboard, richer roads, perception algorithms, or scenario-search research after the integrated scene is stable.

Avoid YOLO, LLM, voice input, large custom city maps, complex planners, and adaptive search as prerequisites.

## 4. Milestones

- **MVP-01 — Vehicle baseline (implemented):** Gazebo launch, ego spawn, ROS 2 command and odometry, waypoint following, destination stop.
- **MVP-02 — Pedestrian baseline (implemented):** configured crossing and reproducible state transitions.
- **MVP-03 — Safety observation (partly implemented):** center distance, line-of-sight TTC, contact, and completion. File persistence and repeat runs remain pending.
- **MVP-04 — Integrated traffic scene (next):** credible road users and traffic signal; state changes, stop-line compliance, and pedestrian interaction in one launch.
- **MVP-05 — Sensor-equipped scene:** camera and LiDAR publish usable ROS 2 data; frames and rates are verified. Evaluate radar separately.
- **MVP-06 — Validation:** repeat defined traffic cases from configuration, save actual outcomes, and summarize pass/fail and limitations.

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

The validation question is: **Does the constructed environment reproducibly exercise and measure intended vehicle behavior in selected traffic situations?** Experiments support environment construction and must use measured outcomes, not screenshots alone.

Initial cases: open-road route completion, red-signal stop, green-signal progress, pedestrian crossing interaction, and sensor publication with a visible target. Save configuration and actual results in JSON or CSV. Useful measures include route completion, stop-line violation, collision, minimum distance, TTC, sensor availability/rate, and simulator real-time factor. Define acceptance thresholds before reporting outcomes.

Do not infer real-world accident probability or real-vehicle sensor performance from these runs. If Random vs. Adaptive is revived, use equal budgets, ranges, controller, environment, metrics, and recorded seeds. It must not block the integrated scene.

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
