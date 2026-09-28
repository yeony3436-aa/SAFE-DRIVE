# safe_drive_sim

This package provides the minimal Gazebo Classic 11 world for MVP-01. It owns
the road world and vehicle spawning, while the vehicle definition remains in
`safe_drive_description`.

Run the simulation:

```bash
cd ~/workspace/SAFE_DRIVE/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch safe_drive_sim simulation.launch.py
```

For a headless run, use `gui:=false`.

The launch file starts Gazebo, publishes `robot_description`, and spawns the
model as `safe_drive_ego` at `(-15.0, -1.75, 0.0)` meters. Spawn coordinates
are launch arguments so later scenarios can reproduce their initial state.

The Gazebo model accepts `geometry_msgs/msg/Twist` commands on
`/safe_drive/ego/cmd_vel` and publishes `nav_msgs/msg/Odometry` on
`/safe_drive/ego/odom`. The first MVP uses a four-wheel skid-steer plugin so
the controller can be developed against a small, deterministic ROS interface.

The GUI camera automatically follows `safe_drive_ego` from a rear elevated
view. Right-click the vehicle and toggle `Follow` to leave or restore tracking
during a run.
