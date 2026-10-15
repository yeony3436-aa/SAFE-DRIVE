# safe_drive_description

This package owns the simulator-independent URDF/Xacro model of the SAFE-Drive
ego vehicle. It intentionally contains only the vehicle's links, joints,
geometry, collision shapes, and inertial values.

`ego_vehicle.urdf.xacro` defines these frames:

- `base_footprint`: vehicle ground projection
- `base_link`: vehicle body frame, 0.75 m above `base_footprint`
- `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`,
  `rear_right_wheel`: rotating wheel links

The default exterior uses the Prius body and wheel meshes from `osrf/car_demo`;
see the repository's `docs/asset-inventory.md` for attribution. The previous
primitive exterior can be expanded with `use_reference_mesh:=false` in the
`safe_drive_ego_vehicle` Xacro macro. The simple collision box and wheel
cylinders remain independent of either visual style. Gazebo-specific plugins,
world files, and spawning logic belong in `safe_drive_sim`; route control
belongs in `safe_drive_control`.

To inspect the expanded URDF:

```bash
source /opt/ros/humble/setup.bash
xacro $(ros2 pkg prefix safe_drive_description)/share/safe_drive_description/urdf/ego_vehicle.urdf.xacro
```
