# Stand navigation

Gets the base from where it is to a stand returned by reachability `~/plan_stands`. Milestone M3 of
`ROADMAP.md`, first half (the base reaches the stand; running the pass from there is still open).

## What is there

`stand_navigator` (`src/manipulation/stand_navigator`) and `stand_navigator_msgs`:

- `NavigateToStand` action (`navigate_to_stand`), shaped like Nav2's `nav2_msgs/NavigateToPose`: a `PoseStamped`
  goal, `distance_remaining` feedback, an error code in the result. The goal is where the **arm base** goes
  (`Stand.x`, `Stand.y`) and the base yaw, in the odom frame (the node does not transform frames; it rejects a
  goal whose `frame_id` differs from the odom frame).
- `core.py` (ROS-free, `test/test_core.py`, 21 tests): the arm-base to `base_footprint` conversion with the profile's
  `arm_mount_xy` rotated by the yaw, the base profile loader (the `base:` section reachability reads: drive type,
  max linear and angular velocity and acceleration), and the controller.
- Controller: proportional with a square-root braking ramp so the base stops at the goal, speed and acceleration
  limits from the profile. Omni (and 4ws): translate and turn together. Diff and skid: turn toward the goal,
  drive forward steering on the heading, reverse when the goal is close and behind, then turn to the final yaw; never a
  lateral velocity. Arrival is position and yaw within tolerance (defaults 2 cm, 0.02 rad) with the base nearly
  at rest.
- Node parameters: `base_profile` (required), `tolerance_xy_m`, `tolerance_yaw_rad`, `timeout_s`, `odom_topic`,
  `cmd_vel_topic`, `rate_hz`, `odom_timeout_s`. It publishes `Twist` on `cmd_vel` and stops on cancel, timeout
  and stale odometry.

## Check in the MuJoCo sim

`sim_mujoco/scripts/run_stand_navigation.sh` (builds a UR10e reachability map, starts the sim with `base_motion:=true`
and the smoke wall, `reachability_node` with the base profile, and `stand_navigator_node`) then
`stand_navigation_test.py`: drive to a staging pose, `plan_stands` for 18 wall poses from there, drive the arm base to the
first stand, check the arrival error against `/odom` and TF, the footprint gap to the wall, the arm and lift joints
(held within 0.02) and the raw F/T (within 2 N of rest).

Result (UR10e on the generic omni base, wall at x = 0.98 m): staging drive 19.6 mm; one stand covers 18 of 18 poses (arm base at
(-0.12, -0.25), yaw 1.57, standoff 1.10 m, footprint clearance 0.80 m); arrival error 9.0 mm and 0.013 rad, TF
agrees, wall gap 0.803 m, arm deviation 0.0016, F/T change 0.49 N, no drift at the stand. A run is one drive of 0.27 m.

## Not done

- Obstacle avoidance. The controller drives a straight line; there is no costmap and no planning. The wall is
  avoided only because the stand keeps the footprint clear and the approach is parallel to it.
- Diff drive in the sim (the controller is unit-tested for it; the sim run used the omni profile).
- The manager pass from the stand. The manager works in `base_footprint` and docks at a fixed standoff, assuming the base
  where the wall scene was authored; it would need the coverage poses re-expressed for the new base pose (the
  manager's `base_check` already compares the stand against the origin). Next step: send the poses in the odom frame
  or shift the plan by the base pose, then run `run_manager_pass.sh` after the drive.
- `NavigateThroughPoses`, a wall-plane servo for the last centimetres, localisation (odom only; the odom frame is the
  world frame, the base starts at the origin).
- A reminder found on the way: `manipulation_manager`'s default `base_check_service` is `/reachability_node/plan_stands`,
  but the node is named `reachability` (service `/reachability/plan_stands`); the script renames the node.

## Nav2 swap path

Nav2 (`nav2_bringup`, `nav2_msgs`, `opennav_docking`) is not installed in the `build`, `full` or `sim` images, and adding it
is a large dependency set, so it was not added. To switch:

1. Install `ros-jazzy-navigation2` and `ros-jazzy-nav2-bringup` in the image.
2. Costmap: a local and global costmap from the profile's `footprint` (the same polygon reachability uses), an
   obstacle layer from the sim lidar once one exists (none yet), `odom` as the global frame until localisation is added.
   Controller: MPPI with the `Omni` motion model for omni profiles, `DiffDrive` for diff; velocity and acceleration
   limits from the profile.
3. Replace the node with a small bridge that keeps the `NavigateToStand` interface: convert the arm-base pose to a
   `base_footprint` pose with `core.arm_to_footprint`, send it as `NavigateToPose` (`nav2_msgs`, same fields), and map the result.
   Callers do not change.
4. For the last metres to the wall use `opennav_docking` with a custom `NonChargingDock` plugin (see the reuse map in `ROADMAP.md`).

`core.py` stays useful for the frame conversion and as a fallback controller where Nav2 is not wanted.
