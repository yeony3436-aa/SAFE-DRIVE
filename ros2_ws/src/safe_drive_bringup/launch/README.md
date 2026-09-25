# Launch Files

This package provides launch entry points for SAFE-Drive.

Available now:

- `simulation.launch.py`: start the Gazebo world and spawn the ego vehicle

Run it with:

```bash
ros2 launch safe_drive_bringup simulation.launch.py
```

Future launch files will be added after their corresponding packages are
implemented: `vehicle.launch.py` for control, then `demo.launch.py` once the
end-to-end MVP is stable.
