# safe_drive_description

This package owns the simulator-independent URDF/Xacro model of the SAFE-Drive
ego vehicle. It intentionally contains only the vehicle's links, joints,
geometry, collision shapes, and inertial values.

`ego_vehicle.urdf.xacro` defines these frames:

- `base_footprint`: vehicle ground projection
- `base_link`: vehicle body frame, 0.25 m above `base_footprint`
- `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`,
  `rear_right_wheel`: rotating wheel links

Gazebo-specific plugins, world files, and spawning logic belong in
`safe_drive_sim`. ROS 2 control and waypoint following belong in
`safe_drive_control` when those packages are introduced.

To inspect the expanded URDF:

```bash
source /opt/ros/humble/setup.bash
xacro $(ros2 pkg prefix safe_drive_description)/share/safe_drive_description/urdf/ego_vehicle.urdf.xacro
```
