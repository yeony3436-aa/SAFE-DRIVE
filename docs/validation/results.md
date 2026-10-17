# Steering route validation — 2026-10-16

These are measured Gazebo Classic 11 / ROS 2 Humble headless results from the
current working tree. The route YAML parameters are embedded in each JSON file;
`seed: null` means the run did not use a random seed. Simulation timestamps
come from ego odometry. The tests used the Prius exterior with the new
front-steering Ackermann model and fixed GREEN for the intersection routes.

| Route | Result | Goal distance | Settled speed | Maximum chassis-corner error |
|---|---|---:|---:|---:|
| [Straight](straight_green.json) | pass | 0.690 m | 0.014 m/s | 0.942 m |
| [Right turn](right_turn_green.json) | pass | 0.705 m | 0.0004 m/s | 1.656 m |
| [Left turn](left_turn_green.json) | pass | 0.698 m | 0.0024 m/s | 1.541 m |
| [Gentle bend](curve_green.json) | pass | 0.686 m | 0.0015 m/s | 1.274 m |

Acceptance was set before these final runs: goal position within 0.75 m,
settled speed below 0.15 m/s, and every sampled chassis corner within 1.75 m
of the configured route polyline. The result is recorded after 10 simulated
seconds in the goal region. The corner distance is a map-based corridor proxy;
it does not measure camera-detected lane markings, tire position, road contact,
or collision. It uses a 4.2 m by 1.8 m rectangular vehicle footprint.

The bend is a separate small course. The three intersection routes share one
eastbound signal approach. These runs do not establish safe motion for all
turn radii, speeds, surfaces, or signal phases. A separate fixed RED check
stopped the vehicle centre near `x=-12.04 m`; with the 2.1 m front offset,
the front remained about 0.69 m before the `x=-9.25 m` stop line. The existing
pedestrian scenario was also rechecked after the drive change: it reached
`COMPLETED` with no reported contact and a 3.65 m minimum actor-centre
distance. Those two checks are observational and do not yet have saved run
JSON in this directory.

Run a route with `ros2 launch safe_drive_bringup signal_intersection.launch.py
fixed_state:=GREEN route_config:=right_turn.yaml
result_file:=/tmp/right_turn.json`, substituting another intersection YAML as
needed. For the bend, use `ros2 launch safe_drive_bringup
curve_course.launch.py result_file:=/tmp/curve.json`. Keep each result file
from a fresh launch; route monitor output is a single JSON result per run.

Simulator real-time factor was not recorded for these four runs. Camera and
LiDAR availability are outside this milestone.
