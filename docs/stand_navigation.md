# Stand navigation

Gets the base from where it is to a stand returned by reachability `~/plan_stands`. Milestone M3 of
`ROADMAP.md`. The base reaches the stand, and the manager pass runs from there (see "The pass from the stand").

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
- Several stands in a loop is done in the manager's `execute_coverage` action (see "Several stands in a loop" below); not
  done there: replanning a failed column, choosing another stand when navigation fails.
- `NavigateThroughPoses`, a wall-plane servo for the last centimetres, localisation (odom only; the odom frame is the
  world frame, the base starts at the origin).
- Found on the way and fixed: `manipulation_manager`'s default `base_check_service` was `/reachability_node/plan_stands`, but the
  node is named `reachability` (service `/reachability/plan_stands`); the default now matches. The script still renames the node.

### IK-confirmed stands in a mission

`base_check_service:=/reachability/plan_verified_stands` makes the multi-stand mission ask for stands that IK and collision
checking confirmed (`reachability_verified_node`, `docs/reachability.md`) instead of RM4D's promise alone. That service takes
minutes on a wall, so raise `stand_service_timeout_s` (default 60 s, the wait for the stand planner's reply) to match. Only
the parameter is new and the manager build passes; a mission has not been run this way.

## Wall docking: standoff and yaw from the fitted plane

For the last metres after `NavigateToStand`. `stand_navigator/wall_docking.py` (ROS-free, `test/test_wall_docking.py`, 21 tests)
and `wall_docking_node` serve the `WallDock` action (`standoff_m`, `timeout_s`; typed result `NONE`, `TIMEOUT`, `NO_ODOM`,
`NO_WALL`, `CANCELED`, with the final standoff and yaw error; feedback with the two errors).

- Input: the plane `wall_detector` publishes on `~/wall_scene` (`plane_abcd`, `normal_outward`) and `/odom`. Run the detector
  with `target_frame:=odom`: the plane is then a fixed world plane, so the base moving while the scans accumulate does not
  smear it, and the node re-expresses it in the base frame from the current odometry every cycle (20 Hz) between scans. A plane
  in another frame is taken as a base-frame plane of the moment it arrived (stop-and-look quality).
- Two numbers: the standoff (from a reference point, default the profile's `arm_mount_xy`, `ref_xy`, to the plane, along the
  horizontal normal) and the yaw error (the base axis `facing_rad`, default +x, against the wall normal). A plane whose normal is
  more than 0.5 rad from horizontal, or with the reference point behind it, is no measurement.
- Control: proportional with square-root braking and a small creep floor, speed and acceleration limits from the profile
  (docking caps `max_linear_mps` 0.15, `max_angular_radps` 0.3). Omni and 4ws: translate along the normal and turn together.
  Diff and skid: no lateral velocity; turn until the yaw error is under 0.15 rad, then drive along the facing axis (reversing if
  too close, backward when `facing_rad` is pi). Docked is standoff within 5 mm and yaw within 0.02 rad with the base at rest;
  `timeout_s`, and `NO_WALL` when no fresh plane (`plane_max_age_s`) arrives for `wall_timeout_s`.
- Not done: lateral position along the wall (the plane does not fix it), a wall with obstacles in front, non-vertical walls,
  hardware. The sensor must see the wall from the start pose.

Check: `sim_mujoco/scripts/run_wall_docking.sh` (sim with `base_motion` and `depth_sensor`, `wall_detector` on odom, the node,
then `wall_docking_test.py`): the base is driven to a start pose 0.4 m too far and 15 degrees yawed, the action is sent for a
0.55 m standoff (footprint to wall), and the final standoff and yaw are read from `/odom` against the known wall at x = 0.98 m.
Three runs (UR10e on the omni base, 3 mm range noise): start standoff 0.953 m and yaw 14.4 degrees, docked in 5.3, 5.2, 5.2 s,
final standoff error 4.9, 4.8, 5.2 mm, yaw error 0.00, 0.01, 0.00 degrees, lateral drift 1.3-1.5 mm. (A first run with a 10 mm
tolerance stopped 9.7 mm off, which is why the default is 5 mm.) Drive type awareness is covered by gtests only; the sim base is
omni (`DOCK_PROFILE=` another profile runs the diff logic on it, not run).

## The pass from the stand

`manipulation_manager` takes `world_frame` (default empty = off: the poses are used as they are in `frame_id`, the base at
the origin, byte-identical to before). With `world_frame: odom` the coverage plan stays in the odom frame, where the
coverage planner and `plan_stands` authored it, and **each column is transformed into `base_footprint` with the TF of the
moment it starts** (`FrameTransform`, ROS-free, `test/frame_transform_test.cpp`). `wall_direction` is a direction in the
world frame and is rotated with the column, so a base that stopped at another yaw (the planner picks side-on stands) still
docks, presses and strokes along the wall's real normal.

Why this and not re-authoring the poses from the arrival pose: the poses are produced once, by the coverage planner, in
one frame; the base stops within tolerance of the stand (about 1 cm), and a pose set re-authored from the goal would carry
that error into the contact. With a TF lookup per column the poses are right for where the base actually is, and
drift between columns (a pushed base) is absorbed. `base_check` also works in the world frame: it starts the planner at the
arm base's pose from TF and compares the stand with it.

`sim_mujoco/scripts/run_stand_pass.sh` closes the loop in MuJoCo (`base_motion:=true`, JTAC, F/T conditioner, MoveIt):
the wall is moved to x = 0.58 m and y +0.3 (`shift_wall_scene.py`, `STAND_PASS_WALL_DX/DY`) so the first stand is not
where the base starts; the coverage planner plans it in odom; `stand_drive.py` calls `plan_stands` (start = the base pose)
and drives to stand 0 through `navigate_to_stand`; the wall goes into MoveIt's scene as an octomap in the odom frame; the
manager runs with `world_frame: odom`; the recorder logs the tool point in odom (`record_run.py --frame odom`) and
`score_run` scores it against the moved wall.

## Several stands in a loop (the manager task)

`manipulation_manager` is also an action server, `execute_coverage` (`coverage_task_msgs/ExecuteCoverage`). Started with
`run_on_start:=false` it waits for goals; with the default `true` it runs the plan of its parameters once and exits, as before
(`run_manager_pass.sh` is unchanged: coverage 0.931, peak 19.97 N, RMS 3.61 N on this build).

| Goal | Meaning |
|---|---|
| `coverage_json_path` | empty = the `coverage_json_path` parameter |
| `multi_stand` | false: the whole plan from where the base is; true: the stand loop below |
| `tool_mode` | `TOOL_PARAMETER` (the `contact` parameter), `TOOL_CONTACT`, `TOOL_NON_CONTACT` |
| `columns` | only these columns (indices into the plan, file order); empty = all |

Feedback (on each state change): `state` (`planning_stands`, `navigating`, `stowing`, `priming`, `docking`, `pressing`,
`stroking`, `retracting`, `done`, `canceled`), `stand_index` / `stand_count`, `column`, `columns_done`, `columns_failed`
(failed + partial), `mission_state` (the mission FSM state, `IDLE`/`ACTIVE`/`PAUSED`/`STOPPED`/`RECOVERY`, as the `mission_command` service reports it), `peak_force_n` (largest magnitude of the conditioned wrench, `force_topic` parameter, counted only while
the tool is pressing or stroking; -1 when not read). Result: `success`, `message`, `columns_done` / `_partial` / `_failed`,
`stands[]` (`StandOutcome`: `reached`, message, the columns of that stand by outcome), `peak_force_n`. A run that ran to its end
is `succeeded` with `success` telling whether every column was done; `aborted` is for a run that could not start (plan not
loadable, no stand); `canceled` for a cancel. One goal at a time (a second is rejected).

**Verify result.** With the parameter `verify_result_path` set, the manager writes a JSON file when a task ends (also a canceled or
not-started one), with the keys of coverage_planner's `VerifyResult` (`passed`, `coverage_percent`, `failed_segment_ids`,
`report_path`, `source: "manager"`, `failures`) plus `coverage_basis: "columns"` and `peak_force_n`. What it can compute: the share of the
requested columns that finished done (`coverage_percent`; this is NOT the geometric coverage, the manager has no tool-position
telemetry or wall model: `score_run` on a recording gives that), the columns that ended partial or failed as `failed_segment_ids`,
and, when `verify_max_peak_n` > 0 and the force topic is read, columns and the run whose force peak passed it. `passed` needs a
loaded, not canceled run in which every column and stand succeeded and no peak limit was broken; `report_path` is the mission
ledger path (`ledger_path`) when set. Not computed: standoff, in-band fraction, thickness, obstacle contacts, controller switch
under load. `core/mission_verify.hpp`, gtests; not run in a sim.

**The loop** (`multi_stand`): `plan_stands` for the plan's poses (world frame, start = the arm base from TF) -> each column
goes to the stand that reaches most of its poses (`assignColumnsToStands`, ties to the earlier stand; a column no stand reaches
is reported failed, not run) -> per stand in visiting order: stow the arm to its travel pose (`travel_joint_state`, default the
pose when the task started) if it had been working, `navigate_to_stand` (`stand_nav_attempts` tries, `stand_nav_timeout_s`; if
the stand is never reached its columns are reported failed and the loop goes on to the next stand), prime the joint state, run
the stand's columns in the world frame (the per-column flow of the pass from the stand, a failed column is recorded and the
next one runs). Stands that run no column are not visited. The task logic that does not touch ROS (column grouping, the state
machine `idle -> planning -> navigating -> running -> done`, the outcome record) is `core/coverage_task.hpp`, unit-tested
(`coverage_task_test`).

`stand_max_span_m` (default 0 = off): the widest stretch of wall one stand is trusted with. The RM4D map does not know the
arm's self-collision or how awkward the pose is at the edge of reach, so asking `plan_stands` for the whole plan returns one
stand "reaching" 54 poses that the arm cannot press in practice. With a span the columns are asked for in groups of at most
that width along the wall (nearest end first, each group starting from the previous group's last stand).

**Cancel**: the trajectory in flight is cancelled (`JtacExecutionAdapter::cancelCurrent`), a navigation in flight is
cancelled, the force target goes to 0, the admittance is disengaged (after the 1.5 s unload ramp the normal flow also gives it,
only when it was engaged) and the valve is closed; the arm stays where it stopped, and the result is `canceled`. Latency: the
node is spun in every wait of the task, so a cancel is heard within about 50 ms, except while MoveIt/OMPL is planning (up to
a few seconds, the planners block).

**Why not BehaviorTree.CPP** (what Nav2 uses for task flow; `ros-jazzy-behaviortree-cpp` is in the image): the flow is a
fixed sequence with two loops and one failure rule, which is 200 lines of state machine with the unit tests above; a tree
would add a node library, XML and ROS action nodes for no behaviour this task has (no parallel branches, no runtime-edited
flow). It is the swap if the task grows retries with different recoveries, conditions on sensor state or several tools: the
`TaskStateMachine` states map to a `Sequence` of `NavigateToStand`, `RunColumns` and `Stow` actions with a `RetryUntilSuccessful`
around the navigation, and the `TaskTracker` is the blackboard.

Not done: replanning a failed column from another stand, a different stand chosen on the fly when navigation fails, pause and
resume, a durable ledger (a restart reruns everything), `columns` given as a wall region instead of indices.

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
