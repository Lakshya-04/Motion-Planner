# Calibration procedures

One place for every calibration the force path and the wall work depend on. Nothing here has been run on
hardware; the steps are derived from the code, and each says what to record and what a bad result means.
[hardware_bringup.md](hardware_bringup.md) is the ordered commissioning procedure that calls these (driver selection
for `hardware:=real` is its section 0a; run the calibrations below after the sensor-alive check and before any contact);
[ft_sensing.md](ft_sensing.md) has the conditioning maths.

Order matters: sign, then TCP, then payload (payload gravity compensation and the lever arm both use the
TCP), then wall plane, then base-to-wall.

| What | Tool | Output goes to |
|---|---|---|
| F/T sign and bias check | by hand + `ros2 topic echo` | `sign`, `bias` of `ft_conditioner` |
| Tool TCP position | `core_types::pivotCalibration` (ROS-free) | `tcp_xyz` of the tool profile, `tool_in_sensor_xyz` |
| Tool TCP from touches on a plane | `core_types::planeTouchCalibration` (ROS-free) | `tcp_xyz`, plane normal |
| Tool tilt from a measured normal | `core_types::fitToolTilt` (ROS-free) | `tcp_rpy` / `tilt_*` |
| Calibration record | `tool_calibration::ToolCalibrationRecord` (JSON) | the loader gate for contact tools |
| Payload (mass, centre of gravity, bias) | `ft_conditioner` services; `force_core` excitation planner | `payload_mass_kg`, `payload_cog_xyz`, `bias` |
| Wall plane | `wall_detector` node | `wall_scene.json` for the coverage planner |
| Base-to-wall | `wall_detector` in `base_footprint` + a touch check | stand/base pose, `target_frame` |

## 1. F/T sign and bias check

Node `wrench_conditioner` (package `ft_conditioner`, executable `wrench_conditioner_node`). Parameters:
`input_topic` (default `wrench_raw`), `output_topic` (`wrench_conditioned`), `sign` (1.0), `bias`
(6 values, sensor frame), `world_frame`, `tool_frame`, `sensor_frame`, `max_tf_age`.

1. Arm still, tool free: `ros2 topic echo /<input_topic>` and confirm the sensor reads the tool weight in the
   direction of gravity in its own axes.
2. Push the tool tip by hand towards the robot along the tool +Z (the way a wall would push back). The
   conditioned output (`ros2 topic echo /<output_topic>`) must show negative `force_z`.
3. Wrong sign: set `sign: -1.0` (the node applies it to the raw wrench first). A negative identified mass in
   section 3 is the second sign indicator.
4. Bias is not set by hand: it is fitted by `identify` (section 3). Check it afterwards by holding still in a
   pose not used in the fit: the conditioned wrench must be near zero within the noise measured at rest.

## 2. Tool TCP by pivot calibration

`tool_profile.tcp_xyz` (flange to tool tip, metres, see `core_types/tool_profile.hpp`) can be measured or
taken from CAD. To measure it, pivot about a fixed point:

1. Fix a cone, dimple or nail in the workspace. It must not move during the procedure.
2. Place the tool tip in it and reorient the flange about it in at least six poses (use 10 or more).
   Rotate about at least two non-parallel axes by 20 to 60 degrees each; rotations about a single axis are
   rejected as degenerate. Keep the tip seated: no sliding.
3. In each pose record the flange pose in the base frame (`ros2 run tf2_ros tf2_echo base_footprint
   <flange_link>`, or the robot controller's own pose output). Use the flange frame that `tcp_xyz` is
   relative to.
4. Solve with the ROS-free core (header only, `#include "core_types/pivot_calibration.hpp"`):

   ```cpp
   std::vector<core_types::Pose> poses = /* one per pose, base frame */;
   const auto r = core_types::pivotCalibration(poses);   // min_poses defaults to 4
   if (r.ok) { /* r.tcp_xyz -> tool profile, r.pivot_xyz, r.rms_m, r.max_residual_m */ }
   ```

   It solves `R_i t + p_i = q` for the tip offset `t` and the fixed point `q` by linear least squares, and
   returns `ok = false` with a reason for too few poses, a zero quaternion or degenerate rotations.
5. Accept when `rms_m` is at the level of the arm's pose accuracy and the repeatability you saw seating the
   tip (a few tenths of a millimetre to about 1 mm for a collaborative arm); `max_residual_m` far above the rms
   points to one slipped pose, drop it and repeat. In the simulation test, 0.5 mm of position noise on 12
   poses recovers `t` to within 1.5 mm.
6. Limits: this gives the position of the tip only. The tool orientation (`tcp_rpy`) is not observable from a
   point contact; measure or take it from CAD. There is no ROS node for it yet, so the solve is run from
   your own small program or test.
7. Put the result in `tcp_xyz` of the profile, and in the conditioner's `tool_in_sensor_xyz` (or let
   `tool_in_sensor_from_tf: true` take it from TF once the URDF has the new TCP). A wrong TCP shows up as a
   pose-dependent torque residual in section 3's check.

## 2b. Tool TCP from touches on a plane, tilt, and the calibration record

Simulation-only maths: no node, no manager stage, nothing run on hardware. Industry practice is a fixed
pointer, plate or cone (section 2); the wall touch below is the field option when no fixture is available.

**Plane touch** (`core_types/plane_touch_calibration.hpp`). Press the tool tip on one flat surface at 12 or
more flange poses (40 or more for the accuracy below) and record the flange pose at each touch together with a flag that
the contact force confirmed the touch. `planeTouchCalibration(poses, forces_ok, options)` drops the unconfirmed
touches and solves `n . (p_i + R_i t) + d = 0` for the tip `t` and the plane (unknown offset `d`, normal `n`
estimated, or `options.known_normal` when you have it from a detector). It reuses the Gauss-Jordan solver
extracted from `pivotCalibration`. It refuses a set whose flange-frame normals `R_i^T n` vary by less than
20 degrees (`min_orientation_variation_deg`) or whose Jacobian is ill-conditioned (rotation about one axis
only), and reports the condition number, the RMS and the maximum residual. The residual includes the arm's
own kinematic error. In the seeded simulation (0.5 mm noise on each position axis, tool tilted up to 50
degrees about both axes, 40 touches) the tip is within 1.5 mm for the headline seed, the mean over 30 seeds is
under 1 mm and the worst seed about 2 mm; with 14 touches and 30 degrees the worst seed was 8 mm. The
offset along the surface normal is the weakest direction: use wide tilts and many touches, and accept the
result only with a small residual and a condition number well below the limit. The normal columns use the tips
relative to their centroid, so the condition number does not depend on where the base origin is. `options.max_residual_m`
(default infinity, off) refuses the fit when the worst touch is further than that from the plane; the error and
`worst_touch_index` (an index into the poses given) name the touch to drop or redo, and the index is set on every
result, so a slipped touch (a +10 mm slip gives a residual of several millimetres against a fraction of a
millimetre for clean data) can also be found without a limit.

**Tilt** (`fitToolTilt`). Given touches with the commanded tool axis and the surface normal measured by a
detector (or the normal fitted from the touches above), it returns the rotation that aligns the commanded
axis with the inward normal, averaged over the touches, in the tool frame when the tool poses are given (a fixed
mounting error; `tool_pose` is the CONTACT-frame pose, the flange pose composed with `tcp_rpy`, and the roll along
the commanded axis in that frame is removed, so an angled mount gives the true error) and in the base frame otherwise (e.g. a wall-normal error). It also gives the spread between
touches. Roll about the tool axis does not move the axis and is not observable; `roll_observable` is always
false. A tilt estimated from the contact force alone is not offered: friction confounds the force direction
with the tilt, and it needs validation on a real pad. In simulation 2 and 10 degree errors are recovered within
0.3 degrees with 0.3 degrees of normal noise.

**Record** (`tool_calibration/calibration_record.hpp`, package `tool_calibration`). `ToolCalibrationRecord`
holds `tcp_xyz/rpy`, payload mass and centre of gravity, F/T bias, method, ISO 8601 UTC date, operator and robot
id strings, residuals (RMS, maximum, tilt spread) and validity: `max_age_hours` and the mounting-event counter
at calibration time. JSON with `toJson` / `fromJson` / `parseRecord` (non-finite numbers and missing fields
are errors). `isValid(now_epoch_s, current_mounting_events)` is false after `max_age_hours`, for a date in the
future, for a date that does not exist (2026-02-31, leap years respected), or when the counter differs (the tool was re-mounted).
`validityReport(...)` returns `valid`, `mounting_checked` (false = no counter was given, the re-mount check was
skipped) and the error. `requireValidCalibration(tool, record, now,
mounting_events)` throws for a contact tool with no record, a record of another tool, or an invalid one, and
also throws when the current mounting counter is not given, and accepts a non-contact tool without a record. Not done: nothing calls it yet (no manager stage, no file
location convention, no comparison with the profile's `tcp_xyz`), wear tracking, payload and bias are stored
but not produced by these methods.

Hand-eye and sensor-extrinsic calibration is not written here: use MoveIt Calibration, `easy_handeye2` or
`robot_calibration` and consume the result (licences to verify, see the reuse map in the roadmap).

## 3. Payload (mass, centre of gravity, bias)

Already implemented as three `std_srvs/srv/Trigger` services on the conditioner node (`wrench_conditioner`).
Everything with nothing touching the tool and the arm still, low accelerations.

```bash
ros2 service call /wrench_conditioner/capture_pose std_srvs/srv/Trigger   # once per pose
ros2 service call /wrench_conditioner/identify std_srvs/srv/Trigger       # fit, apply, print YAML
ros2 service call /wrench_conditioner/clear_samples std_srvs/srv/Trigger  # start over
```

- `capture_pose` averages the last `capture_window_s` (0.5 s) and refuses on stale input
  (`capture_max_age_s` 0.25), too few samples (`capture_min_samples` 5), force spread above
  `capture_max_spread_n` (1 N), or orientation drift above `capture_max_rotation_rad` (0.02).
- Capture at least four poses (six used in simulation) where gravity points in clearly different directions in
  the sensor frame, wrist rotated about at least two non-parallel axes.
- `identify` fits mass, centre of gravity (sensor frame) and the six bias values, applies them at once and
  returns a YAML snippet for `payload_mass_kg`, `payload_cog_xyz` and `bias`. Degenerate pose sets are rejected
  with a reason.
- Verify: mass close to the weighed tool + plate + adapter; conditioned force near zero in poses not used in
  the fit. Repeat after warm-up and after any tool change.
- `identify` also reports the regressor condition number of the pose set and rejects a set above 1000
  (`identifyPayload(..., max_condition)`). The planner's 8 to 10 well-spread poses give about 1.4; six poses within 0.5 rad of one axis give tens
  to hundreds. Noise on the readings is amplified by about this number.

### Choosing the poses: safe excitation planner (ROS-free)

`force_core/payload_excitation.hpp` picks the poses and times the moves. You supply forward kinematics (sensor
orientation from joints), the joint limits and velocity / acceleration bounds, and optionally a safety proxy
(`std::function<bool(q)>`, true = safe: your self-collision spheres, a workspace box, a keep-out on the wrist).

```cpp
force_core::ExcitationLimits lim;  // q_min, q_max, v_max, a_max, margin_rad (0.1), velocity_scale (0.5)
force_core::ExcitationOptions opt; // num_poses (8), num_candidates (400), dwell_s (1.5), seed
auto plan = force_core::generateExcitationPoses(q_now, lim, fk, is_safe, gravity_world, opt);
if (plan.ok) {                      // plan.waypoints[k].q, t_arrive_s, t_leave_s; plan.condition_number
  auto q = force_core::sampleExcitationPlan(plan, t);   // joint reference at time t
}
```

Candidates are random joint configurations inside the limits minus the margin that pass the proxy; poses are
chosen farthest-point first on the gravity direction seen in the sensor frame, and each move from the previous
pose must pass the proxy along the straight joint-space path (checked every `path_step_rad`). Moves are
rest-to-rest quintics, slowed until peak velocity and acceleration are within `velocity_scale` of the bounds.
It returns `ok = false` with a reason if fewer than six reachable, safe, well-spread poses exist, and
warns (still `ok`) when the condition number is above `max_condition` (20). Send the waypoints to the arm
through your normal trajectory interface, wait `dwell_s` at each pose and call `capture_pose` there. No ROS
service runs the plan yet, and the proxy is yours: the planner is only as safe as it is. This is the simple,
offline form of the idea in arXiv 2504.21486 (bounded excitation); it has no online set-based safety
guarantee, and its code was not used (no licence).

Simulated numbers (gtest, 2 kg payload, 0.1 N force and 0.01 N m torque noise, 10 poses, 50 trials): worst
mass error 0.0096 kg, worst centre-of-gravity error 0.68 mm, worst force-bias error 0.09 N, condition 1.4. The
8-pose plan in that test spans 69 s at half the velocity and acceleration bounds.

## 3b. Bias and temperature drift

`force_core/bias_tracker.hpp` (`BiasTracker`) tracks the six bias values slowly while the tool is free. A
per-axis Kalman filter (random-walk bias) updates only when the caller says the tool is free of contact and
the arm is still. It rejects an innovation above 1 N / 0.1 N m (a missed contact or a payload change must not
be absorbed), caps the total correction at 2 N / 0.2 N m from the starting bias, and sets `needsRecalibration()`
after 20 rejections in a row (re-run section 3).

### In the conditioner node (opt-in)

`bias_tracking.enabled:=true` (default false: the node behaves exactly as before) feeds the tracker with the
sign-applied raw wrench minus the payload's gravity (what the bias should equal at rest, sensor frame). The two
gates:

- Free of contact: a `std_msgs/Bool` on `bias_tracking.contact_free_topic` (default `contact_free`), for example
  from a node that watches the force state machine or the conditioned force. Fail safe: no message within
  `bias_tracking.contact_free_timeout_s` (1 s) counts as in contact.
- Arm still: `sensor_msgs/JointState` on `bias_tracking.joint_states_topic` (default `joint_states`); the largest
  |joint velocity| must be below `bias_tracking.still_velocity_rad_s` (0.01) and the message at most
  `bias_tracking.joint_state_timeout_s` (0.5 s) old. A message without velocities counts as moving.

Outputs:

- `~/tracked_bias` (`WrenchStamped`, sensor frame, latched): the tracked six values, same convention as the
  `bias` parameter. With `bias_tracking.apply:=false` (default) it is only published and the conditioner keeps
  the static `bias`; with `apply:=true` the conditioner uses it.
- `~/recalibrate_needed` (`std_msgs/Bool`, latched): true after `bias_tracking.max_consecutive_rejects` (20)
  rejected samples in a row, false again after an accepted one or a reset. Re-run `identify`.
- `~/reset_bias_tracker` (`Trigger`): restart from the `bias` parameter (also undoes an applied correction).
  Setting `bias`, `payload_mass_kg`, `payload_cog_xyz`, `sign` or `gravity_world` (so also `identify`) restarts the
  tracker as well, because the tracked residual depends on them.

Filter tuning parameters: `bias_tracking.process_noise_force` / `_torque`, `measurement_noise_force` / `_torque`,
`reject_threshold_force` / `_torque`, `max_total_correction_force` / `_torque`. The defaults assume a 0.1 N noise
sigma and very slow drift; raise the process noise if the bias moves faster (the node test uses 1e-4).

Tests (`ft_conditioner`, fake wrench stream, no simulator): a bias ramp of 0.1 to 0.5 N over 3 s of stream is
tracked to within 0.06 N when free and still; applying it zeroes the conditioned output (below 0.12 N summed);
nothing is learned while the flag says contact, while a joint moves, or once the flag goes stale; a 4 N push the
flag missed is rejected, `recalibrate_needed` goes true and the reset service clears it. Not run on hardware;
nothing in the repo publishes `contact_free` yet, so the node waits for one you supply.

### Running the excitation plan: `~/plan_excitation`

`ros2 service call /wrench_conditioner/plan_excitation std_srvs/srv/Trigger` plans the static poses of section 3
and answers with the waypoint times and joint values (YAML), the regressor condition number of the set and the
smallest angle between gravity directions. It only plans: it does not move the arm, so follow it by hand or with
your own trajectory sender, then `capture_pose` at each waypoint. There is no URDF forward kinematics inside the
node; the input is joint-space parameters:

| Parameter | Meaning |
|---|---|
| `excitation.q_start`, `q_min`, `q_max`, `v_max`, `a_max` | n values each: current joints, limits [rad], velocity and acceleration bounds |
| `excitation.joint_axes` | 3 n values: unit axis of each revolute joint at q = 0, in the base frame |
| `excitation.home_rpy` | sensor orientation at q = 0 in the base frame (roll, pitch, yaw) |
| `excitation.margin_rad` (0.1), `velocity_scale` (0.5), `num_poses` (8), `num_candidates` (400), `seed` (1), `dwell_s` (1.5), `max_condition` (20) | planner options (`ExcitationLimits` / `ExcitationOptions`) |

The node builds the rotation-only forward kinematics of a serial revolute chain from these
(`R(q) = exp(a1 q1) ... exp(an qn) R_home`); only the sensor orientation matters for gravity directions. Axes
and `home_rpy` come from the URDF at the zero configuration; getting them wrong gives a good-looking condition
number for the wrong poses. No self-collision or workspace proxy is applied (poses are inside the joint limits
with the margin only): review every pose before moving. The base frame must be the world frame in which
`gravity_world` is expressed (a mobile base on level ground).

## 4. Wall plane

Node `wall_detector` (launch `ros2 launch wall_detector wall_detector.launch.py`). It accumulates
`accumulate_frames` (5) point clouds, transforms them to `target_frame` and fits the dominant plane (RANSAC then
PCA). Relevant parameters: `cloud_topic` (`/depth_sensor/points`), `target_frame` (`base_footprint`),
`inlier_threshold` (0.01 m), `min_inliers` (200), `min_inlier_fraction` (0.3), `max_range_m` (0 = unlimited),
`output_dir` (`/tmp/wall_detector`), `wall_id`, `publish_collision_object`.

```bash
ros2 launch wall_detector wall_detector.launch.py cloud_topic:=/depth_sensor/points \
    target_frame:=base_footprint output_dir:=/tmp/wall_detector max_range_m:=4.0
ros2 topic echo /wall_detector/wall_scene --once      # transient local String with the same JSON
```

The log line per fit prints the normal `n`, offset `d`, inlier count and `rms` in mm. Check:

1. `rms` is at the depth sensor's noise at that range, not much larger (a large rms means a curved or cluttered
   surface, or a bad cloud-to-base TF).
2. Fit from two or three base positions: the normals must agree within about a degree and the offsets within
   the depth noise. A systematic difference is a sensor extrinsic or TF error, fix that first.
3. `wall_scene.json` and `walls.pcd` are written in `output_dir`; the coverage planner reads the former.

## 5. Base-to-wall

There is no separate estimator: the wall pose in the base frame is the wall-plane fit expressed in
`base_footprint` (section 4, `target_frame`), so its accuracy is the accuracy of the depth sensor extrinsic and
the base-to-sensor TF. Verify it against the force path, which is the only independent reference:

1. With the wall detected, command a slow approach with JTAC (force not engaged, see
   [hardware_bringup.md](hardware_bringup.md) section 5-6) towards the fitted plane with a stand-off of a few
   centimetres, then close the distance until the conditioned force first rises above the noise.
2. The difference between the commanded distance to the plane at first contact and zero is the base-to-wall
   (plus TCP) error along the normal. Repeat at three or more points of the wall; a constant offset is a TCP or
   sensor extrinsic error (redo section 2), an offset varying with the point is a tilted fitted normal.
3. Apply constant corrections to the sensor TF, not to the wall scene, so every consumer sees one geometry.
4. The in-plane position of the base relative to the wall boundaries is not checked by this procedure; the
   stand navigator takes it from the same detected scene.
