# Scenarios

The first supported scenario should be `sudden_pedestrian`.

Scenario definitions must be configuration data, not hardcoded ROS node behavior.

Example:

```yaml
scenario:
  id: SCENARIO-0001
  type: sudden_pedestrian
  seed: 42

  parameters:
    ego_speed: 8.0
    pedestrian_speed: 1.5
    pedestrian_spawn_distance: 10.0
    trigger_distance: 15.0

  termination:
    timeout: 30.0
```

Do not add multiple scenario types until the sudden pedestrian scenario is reliable.

