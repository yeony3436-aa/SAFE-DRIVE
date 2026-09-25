# Experiments

The core research comparison is:

```text
Random Scenario Sampling
vs.
Risk-Guided Adaptive Grid Refinement
```

Both methods must use:

- the same parameter ranges
- the same simulation budget
- the same vehicle controller
- the same safety metric implementation
- the same simulator environment
- reproducible seeds

The research claim is limited to discovering high-risk scenarios inside a defined simulation scenario space. It is not a real-world accident probability estimate.

## Initial Scenario Vector

```text
S = (ego_speed, pedestrian_spawn_distance, pedestrian_speed)
```

## Result Fields

Recommended machine-readable fields:

```csv
scenario_id,method,seed,ego_speed,pedestrian_speed,pedestrian_spawn_distance,min_ttc,min_distance,collision,emergency_braking,scenario_completed,risk_score
```

