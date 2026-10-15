# Launch Files

This package provides launch entry points for SAFE-Drive.

Available now:

- `simulation.launch.py`: start the Gazebo world and spawn the ego vehicle
- `vehicle.launch.py`: start the simulation and follow the configured route
- `signal_intersection.launch.py`: compact intersection, timed vehicle signal,
  and a signal-responsive straight route

Run the complete vehicle baseline with:

```bash
ros2 launch safe_drive_bringup vehicle.launch.py
```

The intersection launch accepts `gui:=false` and `fixed_state:=RED` or
`fixed_state:=GREEN` for repeatable checks. The pedestrian launch remains
separate until its motion follows the crossing phase.
