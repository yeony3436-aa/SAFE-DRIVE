# Ego camera and LiDAR

The ego vehicle now carries two Gazebo Classic simulated sensors. Their
parameters are passed through `safe_drive_sim/launch/simulation.launch.py`.
The controller still uses waypoints, odometry, signal state, and the configured
yield state. It does not use either sensor for driving decisions.

| Sensor | ROS 2 topic and type | Frame | Default |
| --- | --- | --- | --- |
| Front camera | `/safe_drive/ego/camera/image_raw`, `sensor_msgs/msg/Image` | `camera_optical_frame` | 320×240 RGB, 10 Hz |
| Calibration | `/safe_drive/ego/camera/camera_info`, `sensor_msgs/msg/CameraInfo` | `camera_optical_frame` | Camera calibration message |
| Planar LiDAR | `/safe_drive/ego/lidar/scan`, `sensor_msgs/msg/LaserScan` | `lidar_link` | 360 samples, 360°, 0.2–30 m, 10 Hz |

`base_footprint` is the road-ground projection. `base_link` is 0.75 m
above it. `camera_link` is 1.3 m forward and 0.55 m above `base_link`.
`camera_optical_frame` is colocated with `camera_link`; its +z points
forward, +x right, and +y down. `lidar_link` is 0.95 m above `base_link`.
`robot_state_publisher` publishes the fixed transforms. Gazebo odometry
publishes `odom → base_footprint`; there is no separate `map → odom`
transform in the current scene, and odometry coordinates match the Gazebo
world coordinates used by scenarios.

The sensor rates, image size, and scan samples can be changed through launch
arguments: `camera_rate_hz`, `camera_width`, `camera_height`,
`lidar_rate_hz`, and `lidar_samples`. Keep rates modest until profiling a
larger scene.

The minimal road contains a static orange bollard outside the drivable
corridor at world `(-3.0, -4.65)`. The validation script launches a stationary
ego at `(-15.0, -1.75)`, waits for sensor messages, checks frames, simulated
timestamps and rates, and checks the camera's orange/red pixels and the
LiDAR range in the bollard's expected direction. It also records Gazebo's
`/performance_metrics` real-time-factor samples. Run it from a sourced
workspace with a new output directory:

```bash
python3 ../scripts/validate_sensors.py --output-dir /tmp/safe_drive_sensors_01
```

The [stored validation](validation/sensors_20261103/result.json) passed:
12 camera images and 12 scans; 320×240 `rgb8`; 360 scan samples;
approximately 10.40 Hz camera and 10.00 Hz LiDAR from simulation stamps;
22 red/orange pixels in the saved camera sample; and 11.99 m range in the
bollard sector. The [camera sample](validation/sensors_20261103/camera_sample.png)
visibly contains the road and bollard. Startup performance samples varied;
they do not establish sustained real-time performance. The sensor-target
check demonstrates publication and one visible/returned target, not
object detection or autonomous pedestrian avoidance.
