# safe_drive_control

This package owns the MVP-01 waypoint follower. It reads ego odometry from
`/safe_drive/ego/odom` and publishes velocity commands to
`/safe_drive/ego/cmd_vel`.

The controller follows the configured waypoints in order and continuously
publishes a zero-velocity command after reaching the final waypoint. Controller
limits and route points come from the repository-level `config/controller.yaml`.

Run the complete vehicle baseline with:

```bash
ros2 launch safe_drive_bringup vehicle.launch.py
```
