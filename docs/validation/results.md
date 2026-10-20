# Steering route validation — 2026-10-16

These are measured Gazebo Classic 11 / ROS 2 Humble headless results from the
2026-10-16 working tree. The route YAML parameters are embedded in each JSON file;
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

## 2026-10-20 — extended right turn and fixed red stop

The extended [right-turn run](right_turn_extended_green.json) used fixed GREEN.
It reached `(0.178, -29.356) m` on the extended south arm, settled at
`0.0013 m/s`, and had maximum chassis-corner route error `1.610 m`. It passed
the established goal and 1.75 m corridor criteria. The south-arm curbs have
visual geometry only, so this run demonstrates route completion on the road
surface but cannot measure physical curb contact.

The independent [fixed-red run](fixed_red_stop.json) used the published RED
vehicle-signal state and odometry. Its front remained at least `0.664 m`
behind the configured `x=-9.25 m` stop line, exceeding the required `0.6 m`
gap. After five simulated seconds near the line, speed was `0.0066 m/s`,
below the `0.15 m/s` threshold. It passed. The front position is an odometry
center plus configured 2.1 m offset, not a direct mesh measurement. The
monitor records the first stable stop; it does not monitor for later motion
after saving the result.

Both files came from separate fresh headless Gazebo Classic runs. The
right-turn file includes its route configuration and raw extrema; the red
file includes its thresholds, timestamps, and sampled minimum gap. These
checks do not yet measure pedestrian crossing behavior or sensor output.

## 2026-10-20 — extended approach and signal crossing

The east-west road was extended to `x=-240..150 m`, with the ego spawned at
`x=-210 m` for the long approach. The existing compact turn geometry and
crossing at `x=-5 m` remain. All runs below were fresh headless launches on
ROS 2 Humble / Gazebo Classic 11. Results include their configured route or
scenario values and simulation timestamps. The route acceptance is goal
distance at most 0.75 m, final speed below 0.15 m/s, and no chassis-corner
sample farther than 1.75 m from the configured path.

| Run | Peak ego speed | Goal error | Maximum corner error | Result |
| --- | ---: | ---: | ---: | --- |
| [Fast straight, fixed GREEN](fast_straight_60_green.json) | 17.07 m/s (61.4 km/h) | 0.238 m | 2.460 m | **Fail:** corridor exceeded |
| [Faster right turn, fixed GREEN](right_turn_faster_green.json) | 6.48 m/s (23.3 km/h) | 0.651 m | 1.739 m | Pass |
| [Faster left turn, fixed GREEN](left_turn_faster_green.json) | 6.48 m/s (23.3 km/h) | 0.700 m | 1.739 m | Pass |

The fast straight route uses a 20.8 m/s controller command to obtain the
measured 17.07 m/s peak from this Gazebo vehicle model. It reached and stopped
at its goal, but 262 samples exceeded the 1.75 m geometric corridor.
Thus 60 km/h motion is available for a driving demonstration but high-speed
lane keeping has **not** passed validation. The turn routes accelerate on
the long approach and slow to a configured 0.8 m/s at the tight corner;
their measured peak is below 60 km/h. The result files contain 1 Hz odometry
traces and raw extrema for review.
The fast straight JSON was captured immediately before the long route was
renamed from `controller.yaml` to `fast_straight.yaml`; the embedded parameters
match the separate fast route. The short `controller.yaml` used by
`vehicle.launch.py` was restored afterward.

The [long-approach fixed RED run](long_approach_red.json) used the fast straight
route. It peaked at 9.28 m/s while the red-stop speed limit applied, then
stopped with a minimum 0.857 m front gap and 0.010 m/s final speed. The
configured brake target margin was 1.3 m; acceptance required at least
0.6 m measured gap after a five-second stable stop.

The integrated fixed-RED run spawned the pedestrian on the displayed
crosswalk. The [crossing result](signal_crossing_pedestrian.json) records
`start_signal: WALK` and completion at `y=4.25 m`. The independent
[red-stop result](signal_crossing_red_stop.json) passed with a 0.801 m
minimum front gap. The [safety result](signal_crossing_safety.json) reports
no pedestrian contact and a 14.82 m minimum actor-center distance. This
vehicle response comes from the vehicle RED topic; it is not detection of
the pedestrian. A separate fixed-GREEN observation found the pedestrian
state remained `ARMED` on `DONT_WALK`, with no crossing-completion file.
Signal phase change during an active crossing and repeated-run variability
remain unverified in Gazebo. These runs did not record simulator real-time
factor or camera/LiDAR output.

## 2026-10-20 — timed pedestrian crossing followed by green departure

An additional headless run used `spawn_x:=-30 pedestrian:=true
route_config:=fast_straight.yaml` with cycling signals. The
[pedestrian result](timed_signal_pedestrian.json) records a `WALK` start and
completed crossing, and the [safety result](timed_signal_safety.json) records
no contact. In the [vehicle trace](timed_signal_straight.json), the ego center
was about `x=-12.2 m` at the RED-to-GREEN change near simulation time 12 s,
putting its configured front about 0.85 m behind the stop line. It then
departed and reached the route goal. This demonstrates phase progression in
one run; the vehicle reacted to the signal state, not the pedestrian pose.

The straight run **failed** its geometric corridor criterion: maximum
chassis-corner route error was 2.320 m, with 291 samples outside 1.75 m.
Its peak speed was 13.56 m/s (48.8 km/h), below the separate fixed-GREEN
60 km/h trial because it began 30 m from the crossing and stopped on RED.
The sampled trace is not a continuous stop-line compliance monitor. Simulator
real-time factor and camera/LiDAR output were not recorded.

## 2026-10-20 — steadier 50 km/h straight driving and slower crossing

The vehicle steering pivot inertia and damping were increased, and its Gazebo
steering PID was changed to `500 0 20`. The current straight route commands
17.0 m/s rather than the earlier 20.8 m/s. Its lane-centering gains were
tuned to lateral `0.06` and heading `0.7`. The two fresh fixed-GREEN results
below use the same 1.75 m chassis-corner corridor and destination criteria as
the earlier runs. All results below came from headless Gazebo Classic launches
on ROS 2 Humble; the measurement JSON stores the route settings and trace.

| Run | Peak speed | Maximum center error | Maximum corner error | Outside samples | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| [Straight run 1](steady_straight_50_tuned_run1.json) | 13.96 m/s (50.3 km/h) | 0.381 m | 1.284 m | 0 | Pass |
| [Straight run 2](steady_straight_50_tuned_run2.json) | 13.99 m/s (50.3 km/h) | 0.443 m | 1.357 m | 0 | Pass |
| [Right turn](right_turn_steady_steering_green.json) | 6.56 m/s (23.6 km/h) | 0.200 m | 1.523 m | 0 | Pass |
| [Left turn from -30 m](left_turn_steady_steering_green.json) | 3.85 m/s (13.9 km/h) | 0.144 m | 1.595 m | 0 | Pass |

The first 50 km/h tuning step also passed twice, but with larger center errors
of 0.680 m and 0.570 m in [run 1](steady_straight_50_green_run1.json) and
[run 2](steady_straight_50_green_run2.json). These are retained as tuning
measurements, not substituted for the final settings. The new straight results
show less lane wander in this scene. A 1 Hz trace alone cannot establish tyre
slip or real-car handling. Simulator real-time factor was not recorded.
As a simple visual-motion check, the largest difference between each 1 s
displacement direction and the average body yaw while moving over 5 m was
about 0.37° and 0.44° in the tuned runs, versus 2.40° in the earlier 61 km/h
run. This sampled direction difference is not a tyre-slip measurement.
The left-turn regression started at `x=-30 m` to focus on the intersection;
its final approach was adjusted to a 0.35 m controller goal tolerance and
0.2 m/s² configured deceleration. It settled 0.338 m from the goal. This
short-start result does not measure the full westbound approach.

The final integrated timed run used a 1.0 m/s pedestrian instead of 1.5 m/s.
The [crossing record](timed_crossing_1mps_tuned_pedestrian.json) shows an
8.5 s crossing, ending before vehicle GREEN at about 12 s. The
[safety record](timed_crossing_1mps_tuned_safety.json) reports no contact and
10.39 m minimum actor-center distance. The
[vehicle result](timed_crossing_1mps_tuned_route.json) reached its goal with
maximum corner error 1.090 m and zero outside samples. Its 1 Hz trace puts
the configured vehicle front about 0.82 m before the stop
line at the RED-to-GREEN change. The vehicle still uses the signal topic for
the stop and departure, not pedestrian detection. This is one integrated run;
repeatability of the crossing and continuous stop-line compliance remain to
be measured.

The earlier 1.0 m/s integrated trial with the first steering gains is retained
as [route](timed_crossing_1mps_route.json),
[crossing](timed_crossing_1mps_pedestrian.json), and
[safety](timed_crossing_1mps_safety.json) results. It also passed the route
criterion, with a 1.218 m maximum corner error.

## 2026-10-21 — signal cycle after launch sequencing fix

A fresh headless run of `signal_intersection.launch.py gui:=false
spawn_x:=-30 pedestrian:=true route_config:=fast_straight.yaml` used the
12 s RED, 16 s GREEN, 3 s YELLOW cycle. The [crossing record](signal_cycle_after_gui_fix_pedestrian.json)
shows the pedestrian walking from simulation time 0.5 to 9.0 s. The signal
turned GREEN at about 12 s after ego odometry became available. The
[route trace](signal_cycle_after_gui_fix_route.json) samples the ego near
`x=-12.174 m` just before GREEN, placing its configured front about 0.82 m
behind the stop line. It then departed, reached the goal, and passed the
1.75 m chassis-corner corridor criterion with a maximum error of 1.301 m and
zero outside samples. The [safety record](signal_cycle_after_gui_fix_safety.json)
reports no pedestrian contact and a minimum actor-center distance of
10.30 m. This headless result does not verify the GUI camera view, and the
1 Hz route trace is not a continuous stop-line violation measurement.

## 2026-10-21 — pedestrian clears sidewalk before WALK ends

After a GUI observation showed the pedestrian standing at the crossing edge
after WALK, the configured endpoint moved from `y=4.25` to `y=5.2 m` on the
north sidewalk. WALK now lasts 10.5 s of the 12 s vehicle RED phase. At the
configured 1.0 m/s, the nominal trip to the new endpoint is 9.45 s. In a
fresh headless run, the [pedestrian record](signal_clearance_20261021_pedestrian.json)
shows completion at simulation time 9.9 s and final `y=5.2 m`; the signal
changed to DONT_WALK at about 10.6 s and vehicle GREEN at 12 s. The
[safety record](signal_clearance_20261021_safety.json) reports no contact.
In a separate live Gazebo model-state probe after crossing, the pedestrian
model pose was `(-5.0, 5.2) m`, matching the configured sidewalk endpoint.
The [route result](signal_clearance_20261021_route.json) reached its goal
with a 1.136 m maximum chassis-corner error and zero corridor violations.
The safety monitor marked `emergency_braking: true` during this run; that
metric has not been attributed to a pedestrian response. This headless run
does not verify the visual sidewalk position in Gazebo GUI.
