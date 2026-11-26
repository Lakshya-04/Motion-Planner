# Base localisation: EKF over odometry, IMU and lidar scan matching

Until now the stack ran on `/odom`, the simulated base pose itself, and `stand_navigator` "does not localise: it trusts `/odom`"
([stand_navigation.md](stand_navigation.md)). `base_localisation` (`src/perception/base_localisation`) is an opt-in layer between the
sensors and the consumers: `robot_localization`'s EKF fuses wheel odometry, an IMU and, with a lidar, scan matching into
`/odometry/filtered` and the TF `odom -> base_footprint`. Nothing changes unless you ask for it.

It is odometry fusion, not a map localiser: the pose still lives in the `odom` frame and drifts, only more slowly than the wheels alone.
There is no map, no loop closure and no wall reference in it ([ROADMAP.md](ROADMAP.md) lists those).

## Switching it on

```bash
# Gazebo: noisy odometry + IMU + lidar, EKF with scan matching
ros2 launch sim_gazebo sim.launch.py base_motion:=true world:=$(ros2 pkg prefix sim_gazebo)/share/sim_gazebo/worlds/room.sdf \
  odom_realism:=typical imu:=true lidar:=true localisation:=true localisation_scan_match:=true
# MuJoCo: the IMU is derived from the base twist, no lidar scan matching in this run
ros2 launch sim_mujoco sim_mujoco.launch.py base_motion:=true odom_realism:=typical imu:=true localisation:=true
# a real base: standalone
ros2 launch base_localisation localisation.launch.py platform:=diff odom_topic:=/wheel/odom imu_topic:=/imu/data publish_tf:=true
```

- `localisation:=true` needs `base_motion:=true` and `odom_realism:=good|typical|poor`. With truth odometry there is nothing to filter, and the
  EKF would fight the simulator for the TF. With it on, the realism node's own TF is switched off (`publish_tf:=false`) and the EKF publishes
  `odom -> base_footprint`.
- `localisation_scan_match:=true` needs `lidar:=true`. `localisation_platform:=diff|omni|ackermann` (default `omni`, which is what the simulated base is).
- `imu:=true` is separate and works without the EKF (the Gazebo `imu` sensor, or `sim_imu_node` in MuJoCo).
- Consumers read `/odometry/filtered`: `stand_navigator` with `-p odom_topic:=/odometry/filtered`. Nothing in the existing passes is switched over.
- Scoring uses `/odom_truth` (published by the realism node) as before.

## The IMU in the simulators

Off by default (`imu:=false`): nothing is added to the description, the world or the bridge.

| | Gazebo | MuJoCo |
|---|---|---|
| source | gz-sim `imu` sensor on `imu_link` (on `base_footprint` at `imu_pose`, default `0 0 0.1 0 0 0`), `gz-sim-imu-system` added to the world, bridged to `sensor_msgs/Imu` on `/imu` at `imu_rate_hz` (100) | `sim_imu_node`: derived from the twist of `/odom_truth` (or `/odom` when realism is off); the plugin's own gyro and accelerometer sensors are not used |
| angular velocity | the sensor's | yaw rate of the twist on z, 0 on x and y |
| linear acceleration | the sensor's (specific force, gravity on z) | derivative of the body twist (smoothed, factor 0.3) plus `w x v`, plus 9.81 on z; crude |
| orientation | published by gz (world referenced) | not provided (`orientation_covariance[0] = -1`) |
| covariance | 0 (gz does not fill it) | from the noise parameters, never below 1e-6 |

Noise and bias are off. `imu_gyro_noise` (rad/s) and `imu_accel_noise` (m/s^2) are the white std per sample; `imu_gyro_bias` and
`imu_accel_bias` the std of a constant offset drawn once per axis and run (the same meaning in Gazebo's `<noise>` and in the MuJoCo node).
Nothing is calibrated to a part. The EKF reads only the yaw rate, not the orientation, so a 6-axis IMU is enough and the
world-referenced gz orientation does not leak the truth into it.

## EKF config per platform class

All three configs: `two_d_mode`, world frame `odom`, base frame `base_footprint`, velocities only. The wheel odometry pose is not fused, because
that pose is the integrated drift; the filter integrates the fused velocities from the origin of the odom frame instead, so it
starts at zero (both simulators start the base at the odom origin). Sources that are off are removed from the parameters, so the filter never waits for them.

| class (`platform:=`) | bases | wheel odom fused | gyro | scan matching | rate / timeout | note |
|---|---|---|---|---|---|---|
| `omni` (`config/ekf_omni.yaml`) | mecanum, swerve, the simulated base | vx, vy, yaw rate | yaw rate | vx, vy, yaw rate | 30 Hz / 0.2 s | the only class run in a simulator |
| `diff` (`config/ekf_diff.yaml`) | differential, skid-steer | vx, yaw rate | yaw rate | vx, yaw rate | 30 Hz / 0.2 s | vy has no source and a tiny process noise (it stays 0); on a skid-steer base the wheel yaw rate is the weak term |
| `ackermann` (`config/ekf_ackermann.yaml`) | car-like | vx, yaw rate | yaw rate | vx, yaw rate | 20 Hz / 0.3 s | wheel yaw rate comes from the steering angle and is poor at low speed |

Only `omni` was run. The other two differ in the `*_config` rows, the process noise, the rate and the timeout, and are checked for shape
by the tests, not against any simulated or real base of that kind; the process-noise numbers are round and untuned. Edit the YAML or pass
`params_file:=` (its top key must be the node name, `ekf_filter_node` by default; `name:=` changes it) to change them.
The order of every 15-value row is `x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az`; `process_noise_diag` in the YAML is expanded
to the 15 x 15 matrix by `ekf_params.build`.

The EKF uses each message's own covariance. The realism node's twist covariance includes the scale error sigma and the white noise,
the scan matcher's comes from `sigma_trans_m` and `sigma_rot_rad` (0.002 m, 0.002 rad per match, round numbers, divided by the scan period), and
the Gazebo IMU's is 0, which the filter replaces with its own small floor: a Gazebo IMU with large noise is therefore over-trusted (see the noisy-IMU run below).

## Scan matching

`scan_match_node` matches each `/scan` against the previous accepted one with point-to-line ICP (numpy and scipy, normals from a local line fit,
the worst fifth of the pairs trimmed), takes the motion over the scan period as a body velocity and publishes it on `/scan_odom`. The pose
fields stay zero with a huge covariance, so only the velocity is fused. A match is dropped when ICP does not converge, the line residual is
above 5 cm, fewer than half the points find a partner, or the step is above 0.5 m. Limits:

- It needs structure in all directions. In a corridor or against a single wall the motion along the wall is not observable and the matcher
  still returns a number. The check world `worlds/room.sdf` (8 x 8 m, three obstacles) was made for this; the stock empty world has no walls at all.
- Scan-to-scan, so its own error is a drift too; it helps because that error is independent of the wheel error, not because it is absolute.
- Dynamic objects and the robot's own arm in the scan are not handled (the arm is not in the lidar's plane in the tested pose).

## What was measured

Gazebo, `scripts/run_localisation_check.sh`: room world, UR10e profile, `odom_realism:=typical`, lidar and IMU on (the IMU ideal unless said),
the base driven open loop on two laps of a 2 m square at 0.3 m/s with 90 degree turns at 0.5 rad/s (16 m of path). Four estimates of
the same run are scored against `/odom_truth` after aligning each to the truth at the first sample: `raw` is `/odom`, `ekf_odom` an EKF on the
same `/odom` alone, `ekf_imu` odometry plus gyro, `ekf_full` odometry, gyro and scan matching (this one owns the TF).

Final position error in metres, rms in brackets, final yaw error in degrees (ideal IMU, one run per row, 16 m of path):

| run | raw `/odom` | `ekf_odom` | `ekf_imu` | `ekf_full` |
|---|---|---|---|---|
| typical, seed 1 | 0.222 (0.130), 8.7 | 0.042 (0.052), 3.2 | 0.026 (0.019), 0.4 | 0.029 (0.021), 0.4 |
| typical, seed 2 | 0.045 (0.048), 0.3 | 0.294 (0.179), -10.5 | 0.028 (0.019), 0.1 | 0.026 (0.017), 0.1 |
| typical, seed 3 | 0.633 (0.376), -21.7 | 0.947 (0.527), -34.6 | 0.009 (0.058), 0.3 | 0.012 (0.054), 0.3 |
| poor, seed 1 | 1.074 (0.486), 33.9 | 0.582 (0.261), 18.6 | 0.119 (0.066), -0.3 | 0.089 (0.050), -0.3 |
| typical, seed 1, noisy IMU (gyro noise 0.005 rad/s, bias std 0.003 rad/s) | 0.202 (0.128), 8.4 | 0.024 (0.039), 2.8 | 0.058 (0.050), 4.0 | 0.055 (0.049), 4.0 |

What it says, and what it does not:

- With an ideal gyro, odometry plus IMU beats raw odometry in every run: rms 0.017 to 0.058 m against 0.048 to 0.486 m, and heading within half a degree. The gyro does the work; the heading error of the wheels (a few degrees to tens of degrees on these seeds) is what ruins raw odometry.
- Seeds 1 (typical) and 1 (poor) were run twice over the session (the first typical run gave raw 0.257 m, `ekf_imu` 0.029 m): the runs are not bit-reproducible, because the sim timing and the realism latency differ.
- The EKF on wheel odometry alone (`ekf_odom`) is not reliably better: it is better on seeds 1 and poor, worse than raw on seeds 2 and 3. It fuses the odometry twist, whose yaw rate carries the yaw scale error and bias, so it inherits them; and the realism node's twist lacks the white noise and slip that its pose has, which makes the two not directly comparable.
- Scan matching (`ekf_full` against `ekf_imu`) changed little: 0.029 against 0.026, 0.026 against 0.028, 0.012 against 0.009, 0.089 against 0.119 m. Only the `poor` run, with the 3 % scale error, shows a gain, and it is one run. With 835 of 842 matches used.
- With a noisy IMU the result reverses: odometry plus IMU (0.058 m, 4 deg) is worse than the odometry-only filter (0.024 m) in that run. The Gazebo IMU reports covariance 0, the filter treats it as nearly exact, and a 0.003 rad/s bias is integrated straight into heading. Set a real covariance (or use `params_file`/a covariance relay) before using a noisy IMU; this layer does not do it.
- Five runs, one seed each, one world, one path, open-loop driving, omni config. The truth is the simulator's own pose. No claim beyond "with an ideal gyro it tracks truth better than the simulated wheel odometry on these runs". The `poor` run's shutdown exited 134 after the data was written (cause not investigated).

## Not done

- No MuJoCo run of the EKF or of the lidar scan matcher; the MuJoCo IMU node and launch path are unit-tested or untested only.
- `diff` and `ackermann` configs have not driven a base of that kind, simulated or real. Nothing here ran on hardware.
- The consumers (`stand_navigator`, the passes) were not switched to `/odometry/filtered`; they still read `/odom`.
- No absolute reference (map, wall, fiducial), so the pose still drifts without bound.
