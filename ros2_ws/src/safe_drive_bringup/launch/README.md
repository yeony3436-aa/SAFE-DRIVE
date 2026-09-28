# Launch Files

This package provides launch entry points for SAFE-Drive.

Available now:

- `simulation.launch.py`: start the Gazebo world and spawn the ego vehicle
- `vehicle.launch.py`: start the simulation and follow the configured route

Run the complete vehicle baseline with:

```bash
ros2 launch safe_drive_bringup vehicle.launch.py
```

`demo.launch.py` will be added after the parameterized hazard scenario is
stable.
