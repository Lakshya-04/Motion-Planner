# Nav2 (opt-in)

Nav2 is not part of any default image. The `nav2` Docker stage adds it on top of `sim-gazebo`, and `sim_gazebo` ships a
small parameter set and launch file for the omni base. `stand_navigator` has a `backend:=nav2` adapter
(`nav2_adapter.py`); results of running it against real Nav2 are at the bottom.

## Image

```bash
docker build -f docker/Dockerfile --target nav2 -t motion-planner:nav2 .
```

`nav2` is `sim-gazebo` plus `ros-<distro>-navigation2` and `nav2-bringup`. Mount `src/`, build `sim_gazebo` and
`stand_navigator` inside as described in [sim_gazebo.md](sim_gazebo.md), then:

```bash
ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic base_motion:=true lidar:=true [depth_sensor:=true]
ros2 launch sim_gazebo nav2.launch.py          # in a second shell
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_nav2_goal.sh     # starts both, sends a goal past a box
```

`run_nav2_goal.sh` takes `NAV2_MODE=nav2` (NavigateToPose direct, default) or `adapter` (`navigate_to_stand` with
`backend:=nav2`), and `NAV2_DEPTH=1` to add the depth camera.

## Parameters

`src/simulation/sim_gazebo/config/nav2/nav2_omni_params.yaml`, started by `launch/nav2.launch.py`. Only the controller,
planner, behavior and bt_navigator servers run (no velocity smoother, collision monitor, docking, map server or AMCL), and
the controller publishes `/cmd_vel` directly.

- Frame: everything is in `odom` (rolling-window costmaps, no static map). `odom -> base_footprint` comes from the sim.
- Controller: MPPI with the `Omni` motion model, 0.5 m/s and 1.0 rad/s as in the base profile. Footprint is the profile
  rectangle, 0.9 x 0.6 m.
- Local costmap (5 m, 5 cm): voxel layer with `/scan` (mark and clear) and `/depth_sensor/points` (mark only, floor points
  filtered by `min_obstacle_height`), then inflation 0.65 m.
- Global costmap (24 m): obstacle layer on `/scan`, inflation, NavFn A* with `allow_unknown`.

The lidar is mounted on the front edge and covers 270 degrees, so it is blind behind the base. The depth source is only
as good as the camera pose (default 1.15 m high); it is there to show the voxel layer taking a cloud.

## Run results (2026-11-26, Gazebo, one machine)

Empty world, box 0.3 x 0.6 x 0.6 m centred at (2, 0) on the straight line, goal (4, 0, yaw 0) in `odom`, `run_nav2_goal.sh`.
Nav2 Jazzy (apt), MPPI Omni.

| Case | Result |
|---|---|
| NavigateToPose, `/scan` (3 runs) | succeeded 3 of 3, 11.6 to 11.7 s, position error 0.020 m, yaw error 0.09 rad, footprint 0.155 to 0.196 m from the box, swung out to y 1.04 to 1.06 m |
| Same, `lidar:=false` (control) | no avoidance: the base drove into the box (gap 0.000), no arrival in 120 s |
| `/scan` plus depth cloud | succeeded, 11.8 s, error 0.021 m, gap 0.182 m |
| Depth cloud only (`lidar:=false`, both layers on `/depth_sensor/points`) | succeeded, 11.4 s, error 0.024 m, gap 0.084 m |
| `navigate_to_stand` with `backend:=nav2` | succeeded, 11.6 s, error 0.022 m, gap 0.179 m |

Footprint gap is computed from `/odom` and the footprint rectangle against the box, not from Nav2's own costmap. The yaw error is
the goal checker's tolerance (0.10) at work: Nav2 reports success once inside it.

### The existing adapter against real Nav2

Works: goal conversion (checked with `arm_mount_xy` [0.2, 0.1], goal (4, -1, 0.5): the arm base ended 0.09 m from the goal, inside Nav2's
0.10 m tolerance), success mapping, `NO_NAV2` when no server answers (after 5 s), timeout (goal inside the box: Nav2 kept
replanning and the adapter ended `TIMEOUT` at 120 s; it did not abort on its own).

Gaps found, not fixed here:
- The adapter's `tolerance_xy_m` / `tolerance_yaw_rad` do nothing with Nav2; the goal checker in the Nav2 params decides (10 cm, 0.1 rad here
  against 2 cm and 0.02 rad built in).
- The result's `position_error_m` and `yaw_error_rad` stay 0 and the feedback has no `yaw_remaining` or `current_pose`.
- `obstacle_scan_topic` and the safety monitor (obstacle stop, stuck detection) are bypassed in this backend; Nav2's costmaps are the
  only obstacle protection.
- `stand_navigator_node` prints an `RCLError: publisher's context is invalid` traceback on Ctrl-C (`node._stop()` in `main` after shutdown).
  Same code path for the builtin backend, harmless but noisy.

### Not done

- No map, localisation or SLAM: odom frame only, the robot starts at the origin. Drift is not a factor in a short sim run.
- One obstacle, one goal, a few runs, no moving obstacles, no tuning pass. The MPPI weights are the Nav2 defaults, so the 0.15 to 0.2 m
  gap is what it chose, not a safety margin guarantee.
- Depth source is a floor-level-filtered cloud from a camera 1.15 m up, which sees the 0.6 m box face; a low obstacle below the camera's
  view cone would not show up there.
- Arm, lift and F/T were not exercised during the drive.
- Not tried: the Nav2 `velocity_smoother`, `collision_monitor`, `NavigateThroughPoses`, DWB, real hardware.
- Domain ID: Fast DDS cannot use IDs above 232, so the run used `ROS_DOMAIN_ID=91` with `ROS_LOCALHOST_ONLY=1` in a container on its own Docker network.
