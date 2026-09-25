# SAFE-Drive — Codex Development Instructions

## 1. Project Overview

This repository contains **SAFE-Drive**, a graduation project based on ROS 2 and Gazebo.

**Full project title:**
> ROS2/Gazebo-based Autonomous Driving Safety Evaluation and Risk Scenario Exploration Platform

The project goal is **not** merely to make an autonomous vehicle drive.

**The primary goal is to:**
> Automatically generate hazardous driving scenarios, evaluate vehicle safety, and efficiently discover high-risk scenarios through simulation.

**The final vision may include:**

- ROS 2 autonomous driving
- Gazebo simulation
- Dynamic hazardous scenario generation
- Safety evaluation
- Automated experiment execution
- Risk-guided adaptive scenario exploration
- Web monitoring dashboard
- Event recording and replay
- Risk heatmap
- Existing lane-detection integration
- YOLO-based perception
- Natural-language destination commands
- Optional voice input

However:

> **The entire vision is NOT required for the graduation-critical core system.**
> Do not attempt to implement everything at once.

---

## 2. Primary Engineering Principle

The highest priority is:

> Build a reliable research core first, then add presentation features.

When choosing between:

- a technically complicated architecture
- a simple and reproducible implementation

prefer the:

> **simple, reliable, reproducible implementation**

The graduation project must remain demonstrable even if all optional features are unfinished.

The core project must work **without**:

- YOLO
- LLM
- voice recognition
- custom smart-city modeling
- advanced perception
- complex machine learning

---

## 3. Core Research Question

The primary research question is:

> Under the same simulation budget, can a risk-guided adaptive scenario exploration method discover hazardous autonomous-driving scenarios more efficiently than random sampling?

**The baseline comparison is:**

```
Random Scenario Sampling

        VS

Risk-Guided Adaptive Grid Refinement
```

This comparison is the main research component.

Do not replace this research direction with unrelated AI or perception tasks unless explicitly requested.

### Scope of the Research Claim

The project focuses on **failure-oriented scenario discovery**.

The objective is **not** to estimate the complete real-world accident probability of an autonomous vehicle.

Instead, the objective is to compare how efficiently different sampling strategies discover safety-critical or high-risk regions **inside a defined simulation scenario space**.

> Real-world accident probability estimation — ❌ out of scope
> Efficiency of discovering high-risk conditions inside a defined simulation scenario space — ✅ in scope

This distinction must be stated explicitly whenever the project's results are presented, so that the comparison (Random vs. Adaptive) is not mistaken for a claim about real-world safety statistics.

---

## 4. Scope Hierarchy

Development is divided into three strict tiers.

### Tier 1 — Graduation-Critical Core

These features **MUST** be completed first.

- Gazebo simulation launches reliably.
- One ego vehicle can be spawned.
- The ego vehicle can follow a predefined route.
- Vehicle state and odometry are available.
- At least one hazardous scenario can be triggered automatically.
- Scenario parameters can be changed through configuration.
- TTC can be calculated.
- Collision can be detected.
- Scenario results can be stored.
- Multiple scenarios can run automatically.
- Random Sampling experiments can be executed.
- Risk-Guided Adaptive Grid Refinement can be executed.
- Random and Adaptive experiments can be compared.

> The graduation project must remain valid and demonstrable if development stops at Tier 1.

### Tier 2 — Presentation Layer

Implement only after Tier 1 is stable.

**Possible features:**

- basic web dashboard
- real-time vehicle state visualization
- scenario progress visualization
- experiment result charts
- risk heatmap
- event recording
- event replay

> Tier 2 should improve demonstration quality without changing the core research question.

### Tier 3 — Optional Extensions

Implement only if sufficient development time remains.

**Possible extensions:**

- existing lane-detection integration
- YOLO pedestrian detection
- YOLO vehicle detection
- natural-language destination commands
- LLM-based intent parsing
- voice command input
- additional smart-city visual effects

> Tier 3 features must NEVER delay or destabilize Tier 1.

---

## 5. Current Development Priority

Unless explicitly requested otherwise, follow this order:

1. Gazebo launches
2. Ego vehicle spawns
3. ROS 2 vehicle control works
4. Odometry / vehicle state works
5. Predefined route following works
6. Vehicle reaches and stops at destination
7. One pedestrian hazard scenario works
8. TTC calculation works
9. Collision detection works
10. Scenario configuration works
11. Scenario result logging works
12. Automated experiment execution works
13. Random Sampling works
14. Risk-Guided Adaptive Grid Refinement works
15. Random vs Adaptive comparison works
16. Basic dashboard
17. Risk heatmap
18. Event recording
19. Event replay
20. Existing lane detection
21. YOLO
22. Natural-language commands
23. Voice input

> When uncertain whether to improve the core experiment or add a new feature: **Always improve the core experiment.**

---

## 6. Immediate Milestones

Development should progress through small, demonstrable milestones.

### MVP-01 — Vehicle Baseline

**Goal:** One vehicle drives through a predefined route in Gazebo.

**Required behavior:**

```
Gazebo
  ↓
Ego Vehicle
  ↓
ROS 2 Control
  ↓
Predefined Waypoints
  ↓
Vehicle Movement
  ↓
Destination Stop
```

**Required:**

- Gazebo launches
- ego vehicle spawns
- ROS 2 control command works
- odometry is published
- vehicle follows several waypoints
- vehicle stops at final waypoint

**Do not implement** until this baseline works: YOLO, LLM, dashboard, adaptive sampling, pedestrian scenarios.

### MVP-02 — Hazard Scenario

**Goal:** A configurable hazardous scenario can be reproduced.

Start with only one scenario: **Sudden Pedestrian**

```
Ego Vehicle

    🚗 ──────────────▶

                  🚶
                  ↑
            pedestrian enters road
```

**Required parameters may include:**

- `ego_speed`
- `pedestrian_speed`
- `pedestrian_spawn_distance`
- `trigger_distance`

All parameters must be configurable.

### MVP-03 — Safety Evaluation

**Goal:** Automatically evaluate each scenario.

**Initially calculate:**

- collision
- minimum distance
- minimum TTC
- emergency braking occurrence
- scenario completion

**Example:**

```
Scenario: SCENARIO-0037

ego_speed: 8.0 m/s
pedestrian_speed: 1.5 m/s
spawn_distance: 10.0 m

minimum_ttc: 1.31 s
minimum_distance: 1.24 m
collision: false
emergency_braking: true

result: NEAR_MISS
```

### MVP-04 — Automated Experiments

**Goal:** Execute many scenarios without manual intervention.

```
Experiment Start

Scenario 001
Scenario 002
Scenario 003
...
Scenario 100

Experiment Complete
```

Experiment results must be written to **CSV**, **JSON**, or another machine-readable format.

> Manual result transcription is not acceptable.

### MVP-05 — Adaptive Scenario Exploration

**Goal:** Compare Random Sampling vs. Risk-Guided Adaptive Grid Refinement.

Both strategies must receive the **same simulation budget**.

**Example:**

```
Random:
100 scenarios

Adaptive:
100 scenarios
```

The comparison should evaluate how efficiently hazardous scenarios are discovered.

---

## 7. Recommended Repository Structure

Prefer a standard ROS 2 workspace.

```
safe_drive/
├── AGENTS.md
├── README.md
├── docs/
│   ├── architecture.md
│   ├── experiments.md
│   └── scenarios.md
├── config/
└── ros2_ws/
    └── src/
        ├── safe_drive_description/
        ├── safe_drive_sim/
        ├── safe_drive_bringup/
        ├── safe_drive_control/
        ├── safe_drive_scenario/
        ├── safe_drive_safety/
        ├── safe_drive_experiment/
        ├── safe_drive_msgs/
        └── safe_drive_dashboard/
```

> Do NOT create all packages immediately. Create packages only when they are actually needed. Avoid empty placeholder packages created only to make the project appear larger.

---

## 8. Simulation Environment Policy

Do NOT build a complete smart-city environment from scratch unless explicitly requested.

**Prefer, in order:**

1. existing compatible Gazebo worlds
2. reusable Gazebo models
3. modification of an existing road environment
4. custom modeling only when required

**The minimum environment needed for the research core is:**

- one usable road segment or intersection
- ego vehicle
- one pedestrian or NPC hazard
- enough space to reproduce the scenario

**Additional elements such as:**

large city maps, many buildings, parking lots, hospitals, convenience stores, traffic lights, construction zones, decorative objects

are **presentation enhancements**. They are **NOT research-critical**.

> Prioritize: scenario reproducibility over visual complexity.

---

## 9. Autonomous Driving Baseline

The first controller should be simple.

**Recommended progression:**

```
Predefined Waypoints
        ↓
Pure Pursuit
        ↓
PID Speed Control
```

Do not introduce complex global planners before basic route following works reliably.

The vehicle does not initially need to solve general autonomous navigation. A fixed test route is acceptable for the core experiment.

---

## 10. Perception Scope

Perception is **NOT required** for the core safety-testing experiment.

During core development, Gazebo simulator ground-truth data may be used for:

- ego vehicle position
- pedestrian position
- NPC vehicle position
- relative distance
- relative velocity
- collision state

This is acceptable because the primary research question concerns **scenario exploration and safety evaluation**, not perception accuracy.

Lane detection and YOLO are optional demonstration extensions.

**Do NOT make the following dependencies:**

- YOLO → Safety Evaluation
- Lane Detection → Scenario Testing

The core system must operate when perception modules are disabled.

---

## 11. Dynamic Scenario Generator

### Scenario Representation Policy

A scenario must be represented as **structured configuration data** rather than hardcoded simulator behavior.

Conceptually:

```
Scenario = Environment + Actors + Parameters + Trigger + Termination Conditions
```

For the initial Sudden Pedestrian scenario:

- **Environment**: predefined road or intersection
- **Ego actor**: ego vehicle
- **Hazard actor**: pedestrian
- **Parameters**: `ego_speed`, `pedestrian_speed`, `pedestrian_spawn_distance`, `trigger_distance`
- **Trigger**: ego vehicle reaches configured distance or location
- **Termination**: collision / hazard passed / vehicle stopped / timeout

```yaml
scenario:
  id: SCENARIO-0001
  type: sudden_pedestrian

  parameters:
    ego_speed: 8.0
    pedestrian_speed: 1.5
    pedestrian_spawn_distance: 10.0
    trigger_distance: 15.0

  seed: 42

  termination:
    timeout: 30.0
```

Scenario definitions must remain separate from scenario execution logic.

> Do not create a different hardcoded ROS node for every parameter combination.

This mirrors the definition/execution separation used by tools such as CARLA's ScenarioRunner and Scenic, and can be described in the final presentation as: *"시나리오 정의와 실행을 분리하는 구조를 참고했다."*

The Scenario Generator creates reproducible hazardous situations.

**Initial scenario:** Sudden Pedestrian — a pedestrian enters the ego vehicle's driving path.

**Later scenarios may include:**

- sudden braking NPC vehicle
- occluded pedestrian
- parked vehicle obstruction

> Do not implement multiple scenarios until the first scenario works reliably.

Each scenario must have a unique ID (e.g. `SCENARIO-0001`, `SCENARIO-0002`, `SCENARIO-0003`).

Scenario configuration should use YAML or ROS parameters:

```yaml
scenario:
  type: sudden_pedestrian

  ego_speed: 8.0
  pedestrian_speed: 1.5

  pedestrian_spawn_distance: 12.0
  trigger_distance: 15.0

  seed: 42
```

> Avoid hardcoded experiment values inside ROS nodes.

---

## 12. Initial Experiment Variables

### Scenario Parameter Space

The safety-testing problem should be treated as a **search problem over a scenario parameter space**.

Initial scenario vector:

```
S = (
    ego_speed,
    pedestrian_spawn_distance,
    pedestrian_speed
)
```

Every executed experiment corresponds to **one point** in this parameter space. The sampling strategy determines which point should be tested next.

> Do not increase the dimensionality of the scenario space until the complete Random vs. Adaptive experiment pipeline works reliably.

Framing the problem this way keeps the project from reading as "a script that repeats scenarios," and makes it explicit that this is a **search problem**, not a fixed simulation loop.

Do NOT begin with six or more scenario variables.

The first adaptive experiment should use approximately **2–3 independent variables**.

**Recommended initial variables:**

| Variable | Range |
|---|---|
| `ego_speed` | 10–30 km/h |
| `pedestrian_spawn_distance` | 5–20 m |
| `pedestrian_speed` | 0.5–2.0 m/s |

> Only increase dimensionality after the first experiment pipeline works.

---

## 13. Safety Evaluation

Implement safety evaluation **independently** from visualization.

**Initial metrics:**

- collision
- minimum obstacle distance
- minimum TTC
- emergency braking
- scenario completion

**Possible later metrics:**

- near miss
- lane departure
- route completion time
- stopping distance

> The dashboard must display results. The dashboard must NOT calculate core safety metrics itself.

---

## 14. TTC (Time to Collision)

TTC should initially use a simple, interpretable model.

For an object closing toward the ego vehicle:

```
TTC = relative_distance / closing_speed
```

only when `closing_speed > 0`.

Handle invalid and non-closing situations explicitly. Do not silently return misleading TTC values.

Keep the TTC calculation isolated from ROS plumbing when possible so that it can be unit tested.

---

## 15. Initial Risk Score

Start with an interpretable, rule-based risk score.

**Example baseline:**

| Condition | Score |
|---|---|
| Collision | +100 |
| TTC < 1.0 s | +50 |
| TTC < 2.0 s | +30 |
| Minimum Distance < 1.0 m | +30 |
| Emergency Braking | +20 |

> These values are initial engineering defaults. They are NOT scientifically validated universal safety thresholds.

All thresholds and weights must be configurable. Do not spread hardcoded threshold values throughout the codebase.

**Example configuration:**

```yaml
risk:
  collision_score: 100

  ttc_critical_threshold: 1.0
  ttc_critical_score: 50

  ttc_warning_threshold: 2.0
  ttc_warning_score: 30

  minimum_distance_threshold: 1.0
  minimum_distance_score: 30

  emergency_braking_score: 20
```

### Safety Metrics vs. Search Score

Do not confuse **measured safety metrics** with the **adaptive search score**.

Measured safety metrics include:

- collision
- minimum TTC
- minimum distance
- emergency braking

The risk score is a **derived heuristic** used by the adaptive sampling algorithm to decide which regions should receive more testing.

```
Simulator Measurements
        ↓
TTC / Distance / Collision
        ↓
Risk Score
        ↓
Adaptive Sampling Decision
```

> The risk score must not replace or modify the original measured safety data. Always store the raw metrics separately.

This separation matters because the risk-score formula may be revised later, but the raw TTC/collision data must remain intact and independently re-analyzable.

---

## 16. Core Adaptive Algorithm

The initial adaptive scenario exploration algorithm is: **Risk-Guided Adaptive Grid Refinement**.

**Do NOT replace this with:**

- reinforcement learning
- deep learning
- genetic algorithms
- Bayesian Optimization
- neural networks

unless explicitly requested.

The initial algorithm should remain simple and explainable.

**Algorithm:**

1. Define a coarse grid over the scenario parameter space.
2. Select or execute scenario samples from the coarse grid.
3. Execute each scenario.
4. Calculate its safety metrics.
5. Calculate a risk score.
6. Identify high-risk grid cells or neighborhoods.
7. Subdivide those regions into finer grids.
8. Allocate additional experiment budget to those refined regions.
9. Repeat until the experiment budget is exhausted.

**Conceptually:**

```
Scenario Parameter Space
          ↓
     Coarse Sampling
          ↓
   Scenario Execution
          ↓
   Safety Evaluation
          ↓
   Risk Score Calculation
          ↓
High-Risk Region Detection
          ↓
      Grid Refinement
          ↓
      Focused Sampling
```

The implementation should be **deterministic** when the same configuration, random seed, and simulation conditions are used.

---

## 17. Random Sampling Baseline

Random Sampling is the control baseline.

**Random Sampling must:**

- use the same parameter ranges as Adaptive Sampling
- use the same experiment budget
- use reproducible random seeds
- store results in the same output format

> Do not give either method an unfair experiment budget.

**Example:**

```
Random Sampling:
100 scenarios

Adaptive Grid Refinement:
100 scenarios
```

### Fair Sampling Comparison

Random Sampling and Risk-Guided Adaptive Grid Refinement must be compared under the **same experiment conditions**.

**Both methods must use:**

- the same scenario parameter ranges
- the same maximum simulation budget
- the same vehicle controller
- the same safety metric implementation
- the same simulator environment
- equivalent simulator initialization
- stored random seeds where randomness is involved

> The sampling algorithm is the primary independent variable. Do not change vehicle behavior or safety thresholds between Random and Adaptive experiments.

Comparison results should be derived strictly from stored experiment data.

This section exists specifically to defend against the question: *"Adaptive 쪽에 유리한 조건을 준 것 아닌가?"* — every other variable besides the sampling strategy must be held identical.

---

## 18. Experiment Comparison

**Initial comparison metrics may include:**

- total scenarios executed
- collision scenarios discovered
- near-miss scenarios discovered
- high-risk scenarios discovered
- minimum TTC discovered
- minimum distance discovered
- number of simulations required before first collision is discovered
- number of simulations required before first high-risk scenario is discovered

> Do not fabricate experimental results. All charts and claims must be generated from actual stored experiment data.

---

## 19. Experiment Reproducibility

Experiments must be reproducible. Store experiment configuration separately from source code.

**Recommended:**

```
config/
├── vehicle.yaml
├── controller.yaml
├── safety.yaml
├── scenarios.yaml
├── random_experiment.yaml
└── adaptive_experiment.yaml
```

**Record important metadata:**

experiment ID, scenario ID, sampling method, random seed, parameter values, timestamp, result, risk score, minimum TTC, minimum distance, collision status

---

## 20. Experiment Output

Prefer a tabular structure.

```csv
scenario_id,method,ego_speed,pedestrian_speed,spawn_distance,min_ttc,min_distance,collision,risk_score
SCENARIO-0001,random,6.0,1.2,15.0,3.21,5.3,false,0
SCENARIO-0002,random,8.0,1.5,8.0,1.42,1.4,false,30
SCENARIO-0003,adaptive,9.0,1.7,6.0,0.82,0.7,false,100
```

The experiment pipeline should allow later visualization without manually editing results.

---

## 21. ROS 2 Architecture

Use ROS 2 nodes with clear responsibilities. Avoid creating one giant node.

**Possible nodes:**

- `ego_vehicle_controller`
- `waypoint_follower`
- `scenario_manager`
- `pedestrian_controller`
- `npc_vehicle_controller`
- `safety_monitor`
- `experiment_manager`
- `adaptive_sampler`
- `event_recorder`
- `dashboard_bridge`

> Not all nodes are required immediately. Create them incrementally.

Use ROS messages or services for communication between unrelated modules. Keep algorithm code separate from ROS callback code where practical.

---

## 22. Suggested ROS Topics

Use consistent topic naming.

```
/safe_drive/ego/odom
/safe_drive/ego/cmd_vel
/safe_drive/ego/path

/safe_drive/scenario/current
/safe_drive/scenario/state

/safe_drive/safety/ttc
/safe_drive/safety/risk
/safe_drive/safety/event

/safe_drive/experiment/state
/safe_drive/experiment/result
```

If simulator plugins require different topic names, prefer adapters when useful rather than spreading simulator-specific naming across the project.

---

## 23. Coordinate Frames and Units

Follow ROS conventions. Use SI units internally.

- distance: meters
- velocity: meters/second
- angle: radians
- time: seconds

**Typical TF frames:** `map`, `odom`, `base_link`, `camera_link`, `lidar_link`

> Do not silently mix km/h and m/s, or degrees and radians. Conversions should be explicit.

---

## 24. Configuration Policy

Avoid unexplained hardcoded values. Put tunable values into YAML or ROS parameters.

**Examples:** PID gains, Pure Pursuit lookahead distance, target speed, TTC thresholds, risk-score weights, scenario trigger distance, pedestrian speed, experiment budget, random seed.

Configuration should support repeatable experiments.

---

## 25. Dashboard Scope

The dashboard is a **Tier 2** feature. Do not implement the full dashboard before the experiment pipeline works.

**The first dashboard version only needs to show:**

- ego vehicle state
- current scenario ID
- current scenario parameters
- TTC
- collision state
- experiment progress
- latest scenario result

**Possible architecture:**

```
ROS 2
  ↓
Dashboard Bridge
  ↓
WebSocket
  ↓
Web Backend
  ↓
Frontend
```

**A reasonable default stack is:**

- Backend: Python + FastAPI
- Frontend: React + Vite

> If another stack already exists and works, preserve it unless there is a strong reason to replace it.

---

## 26. Event Recording and Replay

Event replay is **Tier 2**. Do not build replay before scenario execution, safety events, and result logging all work.

**Possible event triggers:** collision, TTC below threshold, near miss, emergency braking.

> Prefer a simple implementation first.

**Possible recording data:** camera frames, vehicle speed, TTC, vehicle position, scenario parameters, event timestamp.

> Perfect physical simulator replay is NOT required for the first version. A synchronized event review is sufficient.

---

## 27. Risk Heatmap

The heatmap is a visualization feature. It should consume actual experiment data.

**Possible heatmap axes:**

- X-axis: `ego_speed`
- Y-axis: `pedestrian_spawn_distance`
- Value: `risk_score`

> Do not manually color or fabricate risk regions. Generate the heatmap from stored experiment results.

---

## 28. Lane Detection

Existing lane-detection code may later be integrated.

**Potential components:** HSV filtering, perspective transformation, sliding-window lane detection.

> Prefer reusing working existing code rather than rewriting it unnecessarily.

Lane detection is NOT required for the initial experiment pipeline.

---

## 29. YOLO

YOLO is an optional **Tier 3** extension.

**Potential uses:** pedestrian detection, vehicle detection, visually impressive camera overlay.

**Do not use YOLO as a prerequisite for:**

- TTC calculation
- collision detection
- scenario generation
- experiment execution

The simulator must remain functional when YOLO is disabled.

---

## 30. Natural-Language Destination Commands

Natural-language commands are optional.

If implemented, do **NOT** allow an LLM to directly generate steering, throttle, or brake commands.

**Preferred architecture:**

```
User Command

"편의점 앞에 세워줘"

        ↓

Intent Parser / LLM

        ↓

POI Identifier

"convenience_store"

        ↓

Local POI Database

        ↓

Known Destination Coordinate

        ↓

ROS 2 Navigation Goal
```

**Preferred structured output:**

```json
{
  "intent": "navigate",
  "destination": "convenience_store"
}
```

> Do not allow arbitrary free-form LLM output to directly control the vehicle.

---

## 31. Voice Input

Voice input is one of the lowest-priority features.

**If implemented:**

```
Voice
  ↓
Speech-to-Text
  ↓
Same text command pipeline
```

> Do not create a separate navigation system for voice commands. Voice should only be another input method.

---

## 32. Logging

Use clear structured logs.

**Good examples:**

```
[scenario_manager] Started SCENARIO-0012 type=sudden_pedestrian seed=42
[safety_monitor] TTC warning: 1.42 s
[experiment_manager] Completed 34/100 scenarios
[adaptive_sampler] Refining high-risk grid region
```

> Avoid printing logs every simulation tick. Throttle high-frequency debug logs. Important events should be easy to find during the final demonstration.

---

## 33. Coding Style

Write maintainable code.

**Python**

- follow PEP 8 where practical
- use type hints for important interfaces
- prefer dataclasses when useful
- use descriptive names
- avoid deeply nested logic

**C++**

- prefer modern C++
- use RAII
- avoid unsafe raw ownership
- keep ROS callbacks small
- separate algorithms from ROS plumbing where possible

**General**

- code identifiers should be English
- comments may be English or Korean
- avoid premature abstraction
- avoid unnecessary design patterns
- avoid adding dependencies without clear benefit

---

## 34. Testing Rules

Every important algorithm should have a basic verification method.

**Good unit-test candidates:**

- TTC calculation
- risk-score calculation
- grid-refinement logic
- scenario parameter generation
- experiment result serialization

**For ROS integration:**

- build affected packages
- run relevant launch files where possible
- inspect startup errors
- verify expected topics
- verify message types

> Do not claim something works unless it was reasonably verified.

---

## 35. Standard Build Workflow

First inspect the current environment. Determine: ROS 2 distribution, Gazebo version, existing packages, simulator plugins, current launch structure.

> Do NOT change ROS or Gazebo versions without explicit reason.

**Typical build workflow:**

```bash
cd ros2_ws

source /opt/ros/$ROS_DISTRO/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

**Tests:**

```bash
colcon test
colcon test-result --verbose
```

**Single-package build:**

```bash
colcon build --symlink-install --packages-select <package_name>
```

> Avoid destructive cleanup unless required. Do not blindly delete `build/`, `install/`, `log/` without a concrete reason.

---

## 36. Launch Philosophy

The final project should eventually support a simple demo command:

```bash
ros2 launch safe_drive_bringup demo.launch.py
```

During development, use smaller launch files. Possible structure:

- `simulation.launch.py`
- `vehicle.launch.py`
- `scenario.launch.py`
- `experiment.launch.py`
- `dashboard.launch.py`
- `demo.launch.py`

> The final demo launcher should only combine modules that are already individually stable.

---

## 37. Documentation

**Maintain:**

- `README.md`
- `docs/architecture.md`
- `docs/scenarios.md`
- `docs/experiments.md`

**README should contain:**

- project purpose
- supported environment
- build instructions
- launch instructions
- implementation status

**Clearly distinguish:** Implemented / In Progress / Planned / Optional

> Do not document unfinished features as completed.

---

## 38. Git and Change Management

Prefer small, coherent changes.

Do not combine unrelated refactors, new features, and dependency migrations in the same change unless necessary.

**Before making a large architectural change:**

1. inspect the current repository
2. identify working behavior
3. explain why the change is needed
4. preserve working behavior
5. make the smallest reasonable change

> Never remove working features solely because a different architecture looks cleaner.

---

## 39. Codex Behavior

When receiving a development request:

1. Read this `AGENTS.md`.
2. Inspect the relevant repository files.
3. Determine the current implementation state.
4. Identify the smallest next milestone.
5. Implement only what is required for that milestone.
6. Build or test affected code.
7. Report what changed.
8. Report what was actually verified.
9. Mention unresolved issues clearly.

Do not assume the repository perfectly matches this document. Existing working code takes precedence over speculative architecture.

Do not repeatedly ask for minor reversible implementation decisions. Choose a reasonable default when the choice can easily be changed later. Ask only when a decision would significantly alter project direction or destroy existing work.

---

## 40. Things Codex Must Avoid

**Do NOT:**

- attempt the entire SAFE-Drive vision in one change
- add YOLO before basic vehicle driving works
- treat YOLO as a prerequisite for safety evaluation
- treat lane detection as a prerequisite for scenario testing
- add LLM integration before navigation basics work
- add voice input before text commands work
- build a large custom Gazebo city before the core experiment works
- implement many scenario types before one scenario is reliable
- introduce more than 3 variables in the first adaptive experiment
- replace Adaptive Grid Refinement with a more complicated algorithm without explicit approval
- introduce reinforcement learning just to make the project look more advanced
- introduce deep learning when a deterministic algorithm is sufficient
- introduce Kubernetes, microservices, or unnecessary infrastructure
- build optional features while Tier 1 contains broken or incomplete items
- manually fabricate experiment results
- manually fabricate safety metrics
- hardcode final experiment results
- hide runtime errors
- claim untested functionality works
- change ROS or Gazebo versions unnecessarily
- create large numbers of empty placeholder files
- redesign working components solely for architectural elegance

> When uncertain whether to improve the core experiment or add a new feature: **Always improve the core experiment.**

---

## 41. Minimum Graduation Definition of Done

The core project is considered successful when all of the following work:

1. One command launches the simulation.
2. The ego vehicle follows a predefined route.
3. At least one parameterized hazardous scenario can be executed.
4. TTC is calculated automatically.
5. Collision status is calculated automatically.
6. At least 100 scenarios can be executed without manual intervention.
7. Scenario results are stored in a machine-readable format.
8. Random Sampling works under a fixed experiment budget.
9. Risk-Guided Adaptive Grid Refinement works under the same budget.
10. Results from both methods can be compared.
11. The entire experiment can be reproduced from configuration and seed values.

> Everything beyond this is an enhancement.

---

## 42. Final Vision

After the core research platform is stable, the final demonstration may evolve toward:

```
                    User
                     │
          Natural Language / Voice
                     │
                     ▼
              Command Interface
                     │
                     ▼
        ┌────────────────────────┐
        │    Gazebo Simulation   │
        │                        │
        │ Ego Vehicle            │
        │ Pedestrian             │
        │ NPC Vehicle            │
        │ Road Environment       │
        └───────────┬────────────┘
                    │
                  ROS 2
                    │
       ┌────────────┴────────────┐
       │                         │
       ▼                         ▼
Vehicle Control          Scenario Generator
       │                         │
       └────────────┬────────────┘
                    ▼
             Safety Evaluator
                    │
                    ▼
            Experiment Manager
                    │
             ┌──────┴──────┐
             ▼             ▼
          Random        Adaptive
                        Refinement
             │             │
             └──────┬──────┘
                    ▼
              Result Dataset
                    │
          ┌─────────┴─────────┐
          ▼                   ▼
     Dashboard             Replay
          │
          ▼
      Risk Heatmap
```

> This architecture represents the final vision. It does NOT mean every component must be implemented before the core research system is considered successful.

---

## Appendix — Recommended First Codex Prompt

After adding this `AGENTS.md`, start with a small, scoped first prompt:

```text
Read AGENTS.md and inspect the current repository.

Work only on MVP-01.

First identify:
- ROS 2 distribution
- Gazebo version
- existing ROS packages
- existing vehicle model
- existing launch files
- available control and odometry topics

Then implement the smallest working baseline where:
1. Gazebo launches,
2. one ego vehicle spawns,
3. ROS 2 can command the vehicle,
4. odometry or vehicle state is published.

Do not implement scenario generation, dashboard, YOLO, LLM, or adaptive sampling yet.

Preserve existing working code.
Build and verify every change you make.
```
