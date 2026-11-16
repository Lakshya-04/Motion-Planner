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
  (`identifyPayload(..., max_condition)`). Well-spread sets of 8 to 10 poses give 3 to 6; a planar sweep of
  0.6 rad gives hundreds. Noise on the readings is amplified by about this number.

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

Simulated numbers (gtest, 2 kg payload, 0.1 N force and 0.01 N m torque noise, 10 poses, 50 trials): see
the CHANGELOG entry.

## 3b. Bias and temperature drift

`force_core/bias_tracker.hpp` (`BiasTracker`) tracks the six bias values slowly while the tool is free. Feed it
`raw - payload gravity` (sign applied) at its own rate together with two flags from the caller: tool free of
contact (force FSM) and arm still (joint velocity). A per-axis Kalman filter (random-walk bias) updates only
then. It rejects an innovation above 1 N / 0.1 N m (a missed contact or a payload change must not be absorbed),
caps the total correction at 2 N / 0.2 N m from the starting bias, and sets `needsRecalibration()` after 20
rejections in a row (re-run section 3). Not wired into the conditioner node: the node's `bias` parameter is
static until a node uses the tracker. In the gtest 0.1 N of drift over 60 s with 0.1 N noise is followed to
within 0.05 N.

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
