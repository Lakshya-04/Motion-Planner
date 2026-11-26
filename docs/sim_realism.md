# Sim realism: odometry and depth noise

The simulators hand the stack a perfect `/odom` and a clean depth cloud ([navigation_and_perception.md](navigation_and_perception.md),
[sim_vs_real.md](sim_vs_real.md)). `sim_realism` (`src/simulation/sim_realism`) puts two ROS nodes between the simulator and its
consumers so the base controllers and the wall detector see something closer to a robot. Both work the same for MuJoCo and Gazebo
because they only look at topics. Everything is off by default: with no argument set, no node starts and no topic moves.

Not calibrated against any robot or sensor. The presets are round numbers picked to sit in the usual range for wheel
odometry and for stereo / structured-light depth (reasons below); take your own from a measurement before you trust a result.

## Switching it on

```bash
# MuJoCo
ros2 launch sim_mujoco sim_mujoco.launch.py base_motion:=true depth_sensor:=true odom_realism:=typical depth_realism:=typical
# Gazebo
ros2 launch sim_gazebo sim.launch.py base_motion:=true depth_sensor:=true odom_realism:=typical depth_realism:=typical
# either one: realism_seed:=7 (default -1: a new seed each run, logged), realism_overrides:="odom.scale_lin_sigma=0.02 depth.sigma_a=0.002"
```

`odom_realism` needs `base_motion:=true` and `depth_realism` needs `depth_sensor:=true`; without them it does nothing.
The values are `off`, `good`, `typical`, `poor`.

| | off (default) | on |
|---|---|---|
| `/odom` | the simulator's base pose | `odom_realism_node` output |
| `/odom_truth` | not published | the simulator's base pose, never touched; use it for scoring |
| TF `odom -> base_footprint` | MuJoCo `base_odom.py` / gz `OdometryPublisher` | published by `odom_realism_node` from the noisy pose (the simulators' own TF is switched off) |
| `/depth_sensor/points` | the simulator's cloud | `depth_realism_node` output |
| `/depth_sensor/points_truth` | not published | the simulator's clean cloud |

How the simulators are redirected: MuJoCo `base_odom.py` gets `odom_topic:=odom_truth publish_tf:=false` and the `ros2_control_node`
a remap of the plugin's `depth_sensor/points` to `points_truth`; Gazebo's `ros_gz_bridge` remaps `/base/odom` to `/odom_truth`,
`/base/tf` to `/base/tf_truth` (nothing listens) and `/depth_sensor/points` to `/depth_sensor/points_truth`.

Standalone, against a simulator started with the same two arguments (it publishes the truth topics): `ros2 launch sim_realism realism.launch.py odom_realism:=typical depth_realism:=good seed:=7`
(this launch defaults both to `typical`; `use_sim_time:=true` for Gazebo). The nodes also run on their own: `ros2 run sim_realism odom_realism_node --ros-args -p preset:=poor -p seed:=3 -p scale_lin_sigma:=0.05`.
Every model field is a parameter; NaN (and -1 for `latency_periods`) keeps the preset value.

## Odometry model

`/odom_truth` -> body-frame increment per message -> corrupted increment -> integrated into the reported pose. So the error
accumulates the way real odometry does, and the pose starts at the truth pose.

- Per run, drawn once: linear scale error N(0, `scale_lin_sigma`), yaw scale error N(0, `scale_yaw_sigma`), initial yaw-rate bias N(0, `yaw_bias_init_sigma`).
- While running: the bias random-walks (`yaw_bias_walk`, rad/s per sqrt(s)); white noise on every increment, `trans_noise` (m per sqrt(m driven), both body axes) and `yaw_noise` / `yaw_noise_rot` (rad per sqrt(m driven) / per sqrt(rad turned)).
- Slip: with probability `slip_rate_per_m` per metre a spell of `slip_duration_s` starts in which both scales get an extra N(0, `slip_scale_sigma`).
- Latency: `latency_periods` whole truth periods (50 Hz in Gazebo, the plugin rate in MuJoCo). The output keeps the original header stamp, so a stamp-based age check sees the delay; the navigator's own watchdog uses the arrival time.
- Dropout: each output message is lost with probability `dropout` (the TF with it).
- Covariance: pose x, y and yaw variances and twist vx, vy, wz variances from the model (accumulated distance and turn, scale sigmas, bias walk integral); z, roll, pitch are 1e-9 (planar base). The twist is the truth twist with the scale errors and the bias on wz.
- Drift figure: `~/drift` (`std_msgs/Float64MultiArray`: position error m, yaw error rad, truth path m) at 1 Hz, and one log line at shutdown, for example
  `odom_realism drift: preset=typical seed=1 position error 0.003 m (0.44 % of 0.72 m driven), yaw error +2.19 deg, slips 0, messages in 1371 out 1363`.
- `off` forwards the truth message object itself (bit exact).

### Odometry presets

| field | off | good | typical | poor |
|---|---|---|---|---|
| `scale_lin_sigma` | 0 | 0.5 % | 1.5 % | 3 % |
| `scale_yaw_sigma` | 0 | 1 % | 2 % | 4 % |
| `yaw_bias_init_sigma` (deg/s) | 0 | 0.01 | 0.03 | 0.1 |
| `yaw_bias_walk` (deg/s per sqrt(s)) | 0 | 0.003 | 0.01 | 0.03 |
| `trans_noise` (m per sqrt(m)) | 0 | 0.001 | 0.003 | 0.01 |
| `yaw_noise` (rad per sqrt(m)) | 0 | 0.002 | 0.004 | 0.01 |
| `yaw_noise_rot` (rad per sqrt(rad)) | 0 | 0.004 | 0.01 | 0.02 |
| slip (per m, duration s, extra sigma) | none | none | 0.05, 1.0, 0.15 | 0.2, 1.5, 0.3 |
| `latency_periods` | 0 | 0 | 1 | 2 |
| `dropout` | 0 | 0 | 0.5 % | 3 % |

Why these: `typical` is a differential or omni base on wheel encoders over a smooth indoor floor with no other sensor.
A per-run scale sigma of 1.5 % gives a median distance error of about 1 % and a 90th percentile near 2.5 %, which is the 1 to 2 percent
usual for uncorrected wheel odometry; the heading terms add up to roughly 1 to 3 degrees per 10 m at 0.3 m/s (1 sigma about 1.7 degrees,
unit test `test_typical_preset_error_budget` checks the median lies in 0.5 to 4 degrees). `good` is a calibrated base or one
with a gyro in the loop; `poor` is carpet, worn wheels or a caster-heavy drive, with slip spells every 5 m. These are order-of-magnitude
choices from the common literature on wheel odometry (systematic errors of a few percent on scale, wheel-base and heading), not data
from a robot.

## Depth model

Organised `PointCloud2` in (as both simulators publish it, NaN for no return) -> same message out: same size, fields, frame and stamp,
only the `x`, `y`, `z` values change (other fields and padding are copied). It works on the range `r` along each ray, so it does not
care which way the sensor frame points. Order: range limits, dropouts, flying pixels, range noise, outliers, quantisation.

- Range noise N(0, sigma) with sigma = `sigma_a` + `sigma_b` r^2 (stereo and structured light: error grows with the square of range). Points move along their own ray.
- Dropouts: base probability `dropout`; on depth discontinuities (a neighbour in the grid differs by more than 5 cm + 2 % of r) `edge_dropout` more; at grazing incidence `grazing_dropout` times a ramp from 0 at 60 degrees to 1 at 85 degrees, with the incidence angle estimated from the surface normal of the grid (cross product of the central differences).
- Flying pixels: at a discontinuity, with probability `flying_pixel_prob` the range is replaced by one between the two surfaces on the same ray.
- Outliers: per pixel probability `outlier_prob` of a uniformly random range between the minimum and maximum range.
- Quantisation of the range to `quantum_m`; cuts at `min_range_m` and `max_range_m` (0 = off).
- An unorganised cloud (height 1) gets everything except the edge and grazing terms, which need the grid.

### Depth presets

| field | off | good | typical | poor |
|---|---|---|---|---|
| `sigma_a` (m) / `sigma_b` (1/m) | 0 | 0.0005 / 0.0008 | 0.001 / 0.002 | 0.002 / 0.005 |
| sigma at 1, 2, 4 m | 0 | 1.3, 3.7, 13 mm | 3, 9, 33 mm | 7, 22, 82 mm |
| `dropout` | 0 | 0.2 % | 1 % | 5 % |
| `edge_dropout` / `grazing_dropout` | 0 | 0.3 / 0.3 | 0.5 / 0.5 | 0.8 / 0.8 |
| `flying_pixel_prob` | 0 | 0.1 | 0.3 | 0.6 |
| `outlier_prob` | 0 | 0.01 % | 0.05 % | 0.2 % |
| `quantum_m` | 0 | 1 mm | 1 mm | 4 mm |
| `min_range_m` / `max_range_m` | 0 | 0.15 / 6 | 0.2 / 5 | 0.3 / 3.5 |

Why these: `typical` follows the shape of a consumer stereo or time-of-flight depth camera: millimetres at 1 m, a few centimetres at
4 m (the quadratic term follows the Kinect-class fit of about 4 cm at 4 to 5 m), 1 mm depth units, a 0.2 m blind zone, a 5 m useful range,
holes and flying pixels at edges. `poor` is a cheap sensor in sunlight or on shiny surfaces. Not taken from a data sheet.

## Self filter in the passes

`robot_self_filter` was a library and a node that no pass started. With `MULTI_STAND_SELF_FILTER=1` both multi-stand pass scripts
(`sim_mujoco/scripts/run_multi_stand_pass.sh`, `sim_gazebo/scripts/run_multi_stand_pass.sh`) add `depth_sensor:=true`, start the
self filter on `/depth_sensor/points` -> `/depth_sensor/points_filtered` and a second `wall_detector` (`wall_detector_sf`, target frame
`odom`, output in `$OUT/self_filter`) on the filtered cloud. Default off, and the plan does not read what it detects: it is an observer
that shows the filtered chain running through a pass. Not run through a full pass (see the measurement below).

Measured on Gazebo (`sim_gazebo/scripts/run_self_filter_check.sh`: smoke wall world, `depth_sensor:=true`, self filter and one
`wall_detector` on each cloud, `self_filter_check.py` pairs raw and filtered clouds by stamp; 96 x 64 cloud, mean of 5 pairs):

| arm and sensor | raw finite points | wall points | removed | wall points removed | wall fit raw vs filtered |
|---|---|---|---|---|---|
| start pose, sensor 1.15 m up (the default) | 6144 | 6144 | 0 | 0 | normal 0.0000 deg, offset 0.00 mm |
| start pose, sensor at z 1.46 | 6144 | 6113 (31 other) | 0 | 0 | normal 0.0000 deg, offset 0.00 mm |
| ready-pose arm (lift 0), sensor at (0.1, 0.15, 1.46) | 376 | 331 (45 other) | 45 | 0 | normal 0.0000 deg, offset 0.00 mm, inliers 1640 vs 1654 |

The 45 removed points sit at x 0.49 to 0.56, y -0.02 to 0.06, z 1.60 to 1.63 m in `base_footprint`: the wrist and forearm.
So the filter does remove the robot's own links from the Gazebo cloud and leaves the wall alone, but the honest findings are:
with the default sensor mount and the arm poses tried (the manager's ready pose and four others) the arm never enters the 60 degree,
96 x 64 view, so there the cloud contains no robot points at all (the old statement that the Gazebo cloud "sees the robot's own links" holds
only when a link is in view, which the default mount does not give); the 31 non-wall points of the raised-sensor start pose are not removed (not
identified; likely the tool, see next); and the wall fit is identical to the 0.01 mm printed. Two problems found on the way:
the shipped `config/self_filter.yaml` had `links: []` and `padding_overrides: []`, which crash the node on start (untyped empty lists), fixed by dropping them;
and `tool_tip_link`, where the Gazebo description puts the tool's collision, hangs off a joint nobody publishes in TF, so the node drops every cloud
("tool_tip_link does not exist"). The Gazebo scripts therefore pass `links:=` without it, which means the tool itself is not filtered there.
Not run: a full multi-stand pass with `MULTI_STAND_SELF_FILTER=1` (the script wiring passes `bash -n` and mirrors the check script, nothing more), the MuJoCo pass variant.

## What is not modelled

- No wheel physics, floor or contact: the base is a kinematic velocity in both simulators, the model corrupts its reported pose only. Real slip also changes where the base actually goes; here the truth is unaffected.
- No lidar, no IMU, no visual odometry, no localisation filter. Nothing here is a model of a particular robot or camera.
- Odometry noise is independent per axis in the body frame; no systematic wheel-base or wheel-diameter asymmetry beyond the two scale errors; no correlation between slip and the commanded acceleration.
- Depth: no multipath, no texture or colour dependent holes, no motion blur, no per-pixel fixed pattern, no temporal flicker model beyond independent draws per frame, and the sim's own cloud is already free of robot self hits in MuJoCo.
- The dropout and latency are drawn per message on the truth rate, not per wall-clock time; jitter is not modelled.

## Tests and checks

- `src/simulation/sim_realism/test` (37 pytest): the models (zero-mean scale and bias, sigma against the configured value, drift growing with distance, latency of exactly N periods, dropout rate, determinism per seed, `off` bit exact, covariance filled), the depth model on synthetic organised grids, and the two nodes' callbacks (`rclpy` present only).
- `src/simulation/sim_realism/scripts/run_realism_check.sh`: stand navigation, wall docking and the depth wall check with the nodes on.

Results, MuJoCo, one run per line, seeds as given (`odom_realism:=typical`, the existing 30 mm / 0.03 rad tolerances):

| stand navigation, 0.7 m path | against `/odom` (what the navigator sees) | against `/odom_truth` |
|---|---|---|
| seed 1 | 11.3 mm, 0.012 rad pass | 8.8 mm, 0.048 rad FAIL |
| seed 2 (one slip) | 7.4 mm, 0.012 rad pass | 27.9 mm, 0.237 rad FAIL |
| seed 3 | 6.5 mm, 0.012 rad pass | 5.7 mm, 0.053 rad FAIL |
| seed 4 | 8.1 mm, 0.008 rad pass | 11.8 mm, 0.003 rad pass |
| seed 5 | 8.0 mm, 0.011 rad pass | 10.5 mm, 0.021 rad pass |
| seed 6 | 9.6 mm, 0.015 rad pass | 14.9 mm, 0.088 rad FAIL |
| `poor`, seed 1 | 8.6 mm, 0.008 rad pass | 5.6 mm, 0.084 rad FAIL |

The navigator always meets its tolerance against the odometry it reads, since it closes the loop on it. Against the truth, position stays
within about 3 cm but the heading misses the 0.03 rad limit in 4 of 6 `typical` runs (and on `poor`): the staging move turns the base about 90 degrees and
the yaw scale error (about 2 %), the yaw noise and, in seed 2, a slip spell (-14 degrees) are read as truth. This is a result, not a threshold to loosen:
a navigator that must be right in the world needs a wall or map reference, not odometry alone. The reported-versus-truth drift lines were
for example `position error 0.007 m (1.00 % of 0.70 m driven), yaw error -3.61 deg` (seed 3).

Wall docking, `typical` odometry, seed 1: start standoff 0.953 m, yaw 14.5 degrees; docked in 5.1 s; final standoff 0.5548 m against `/odom` and
0.5550 m against `/odom_truth` (goal 0.55, tolerance 20 mm: 4.8 mm and 5.0 mm), yaw -0.79 deg against `/odom` and -0.02 deg against truth; the two
odometries differ by 0.2 mm at the end. Docking passes because it regulates on the detected wall plane, not on odometry; one run, one seed. Drift of
that run: 7 mm over 0.43 m driven, yaw -0.77 deg.

Depth wall (`run_depth_wall.sh`, MuJoCo, `depth_sensor_noise_m:=0` for both runs, seed 1), plane fit against the known wall:

| | clean (`depth_realism:=off`) | `typical` |
|---|---|---|
| points per scan | 4464 | 4420 |
| median sensor x vs wall 0.8800 m | 0.8800 | 0.8799 |
| normal error | 0.000 deg | 0.002 deg |
| plane distance error at corners and centre | 0.00 mm | 0.05 mm |
| fit rms | 0.00 mm | 2.62 mm (the model says 2.5 mm at 0.88 m) |
| inliers | 22320 | 22081 |

All depth wall checks pass in both runs, including the coverage plan on the detected scene.

Gazebo, same checks: depth wall clean versus `typical` (seed 1): normal error 0.000 versus 0.002 deg, plane distance error 0.00 versus 0.02 mm, fit rms
0.00 versus 2.36 mm, 6144 versus 6086 points, both pass. `run_base_motion.sh odom_realism:=typical` passes (0.300 m travelled, TF matches `/odom`, node drift
0.001 m over 0.30 m, yaw -0.04 deg), which shows the bridge remap to `/odom_truth` and the node's TF work with `use_sim_time`.

Not run: `poor` depth, `good` anything, stand navigation or docking on Gazebo, a full stand pass or multi-stand pass with the odometry noise on, any hardware.
