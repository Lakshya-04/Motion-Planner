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

Overview of what is real versus stubbed across navigation and perception: [navigation_and_perception.md](navigation_and_perception.md).

- Obstacle avoidance from live sensing. The controller drives a straight line unless a wall scene is given, then a base
  path planner can go around the scene's walls and obstacles ("Path planning" below); there is no costmap from a sensor
  and no dynamic obstacle handling.
- Diff drive in the sim (the controller is unit-tested for it; the sim run used the omni profile).
- Several stands in a loop is done in the manager's `execute_coverage` action (see "Several stands in a loop" below); not
  done there: replanning a failed column, choosing another stand when navigation fails.
- `NavigateThroughPoses`, a wall-plane servo for the last centimetres, localisation (odom only; the odom frame is the
  world frame, the base starts at the origin).
- Found on the way and fixed: `manipulation_manager`'s default `base_check_service` was `/reachability_node/plan_stands`, but the
  node is named `reachability` (service `/reachability/plan_stands`); the default now matches. The script still renames the node.

### Whole columns, approach and retract (analysis, not yet in the loop)

`assignColumnsToStands` gives a column to the stand that reaches most of its poses, so a column whose poses are split between stands
runs once, from the better stand, and may come out partial. The reachability module now has the layer that avoids this before the
mission: `planColumnStands` (`docs/reachability.md`, "Column pose sequences") picks stands over *whole* columns, where a column
counts as whole only if the approach (`dock_approach_extra_m`, `dock_standoff_m`), the contact pose, every stroke and the retract
are usable at one lift height with a margin from the map, and reports each unavoidable split with its pieces and each failing column
with the limiting pose. `analyzeColumnCoverage` plus `column_reach_tool` give the numbers (columns whole / split / unreachable,
covered fraction, stands). The manager does not call it: `manipulation_manager_node.cpp` still uses `plan_stands` and
`assignColumnsToStands`; the hook is `CoverageFeasibilityAnalyzer::setColumnReachability` (`ColumnReachabilityPort`), unset by
default. Not verified in a sim; RM4D estimates only.

### IK-confirmed stands in a mission

`base_check_service:=/reachability/plan_verified_stands` makes the multi-stand mission ask for stands that IK and collision
checking confirmed (`reachability_verified_node`, `docs/reachability.md`) instead of RM4D's promise alone. That service takes
minutes on a wall, so raise `stand_service_timeout_s` (default 60 s, the wait for the stand planner's reply) to match. Only
the parameter is new and the manager build passes; a mission has not been run this way.

## Path planning (opt-in, scene based)

`stand_navigator/path_planner.py` is a ROS-free planner for the base footprint (the `base.footprint` rectangle of the profile).
It builds a 2D grid from a wall scene (the wall_detector JSON: `walls`, `obstacles`; vertical planes become 2 cm thick floor
segments, obstacles become boxes protruding from their wall; heights are ignored, so anything blocks), inflates it by the
footprint's circumscribed radius plus `planner_margin_m`, runs 8-connected A*, shortcuts the path by line of sight and
follows it with `StandController` aimed at a carrot point `planner_lookahead_m` ahead (waypoint following, no stop at corners;
`PathFollower`). It is plain grid A* plus shortcutting, not hybrid-A*: the diff base turns in place. `time_parametrize`
turns a path into a speed- and acceleration-limited pose sequence (trapezoidal speed profile, yaw rate capped).
The cells near the start and the goal, which the inflation would close (a stand is near the wall by design), are opened
only where the exact footprint at that endpoint's yaw is collision-free; away from them the guarantee holds for every heading.

Behaviour of `navigate_to_stand` (action definition unchanged):

- No scene (`wall_scene_json` and `wall_scene_topic` empty): exactly the old straight-line drive.
- With a scene, the straight drive is checked with the exact footprint (yaw interpolated for omni, along the line for diff).
  If it is free the old controller runs unchanged; if it is blocked the planned path is followed, and if there is no path the goal
  is rejected (`INVALID_GOAL`, reason in `error_msg`). `use_path_planner:=true` plans even when the straight line is free
  (and falls back to the straight line if no path is found).
- The result's `error_msg` carries the length, e.g. `arrived (planned path 3.10 m vs straight 2.40 m)` or
  `arrived (straight line 2.40 m)`.

Parameters: `use_path_planner` (false), `wall_scene_json` (file), `wall_scene_topic` (String JSON, transient local, e.g.
`/wall_detector/wall_scene`), `planner_resolution_m` (0.05), `planner_margin_m` (0.02), `planner_lookahead_m` (0 = from the
profile). The scene must be in the odom frame (nothing is transformed). Limits: host-tested on a kinematic base only
(`test/test_path_planner.py`), not run in a simulator; no dynamic obstacles; one scene snapshot per goal.

MuJoCo multi-stand pass switches (`run_multi_stand_pass.sh`): `MULTI_STAND_PATH_PLANNER=1` gives the navigator the pass's wall scene
and `use_path_planner:=true`; `MULTI_STAND_WALL_DOCK=1` also starts `wall_docking_node` and latches the pass's scene
(frame set to odom) on `/wall_detector/wall_scene` (the pass runs no wall_detector). Both default off. The manager does not send
`WallDock` goals yet, so with the flag only the server is up (send a goal by hand, see the script header).

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
- Lateral position along the wall: the plane does not fix it; an optional term for it is below (core only, off by default, not
  in the node or the action). Not done: a wall with obstacles in front, non-vertical walls,
  hardware. The sensor must see the wall from the start pose.

Check: `sim_mujoco/scripts/run_wall_docking.sh` (sim with `base_motion` and `depth_sensor`, `wall_detector` on odom, the node,
then `wall_docking_test.py`): the base is driven to a start pose 0.4 m too far and 15 degrees yawed, the action is sent for a
0.55 m standoff (footprint to wall), and the final standoff and yaw are read from `/odom` against the known wall at x = 0.98 m.
Three runs (UR10e on the omni base, 3 mm range noise): start standoff 0.953 m and yaw 14.4 degrees, docked in 5.3, 5.2, 5.2 s,
final standoff error 4.9, 4.8, 5.2 mm, yaw error 0.00, 0.01, 0.00 degrees, lateral drift 1.3-1.5 mm. (A first run with a 10 mm
tolerance stopped 9.7 mm off, which is why the default is 5 mm.) Drive type awareness is covered by gtests only; the sim base is
omni (`DOCK_PROFILE=` another profile runs the diff logic on it, not run).

### Lateral term, typed outcomes and the noise Monte Carlo (core only)

`wall_docking.py`, off by default; with `LateralConfig.enabled` False nothing changes (the 21 original tests pass unmodified; a
test also checks the command is identical whatever `WallMeasurement.lateral` holds). The node and the `WallDock` action do not use
it yet: the action's result has no lateral field, so the follow-up is to add `lateral_error_m` and an outcome code to
`WallDock.action` and the parameters to `wall_docking_node`.

- Lateral error: the signed slide along the wall tangent still needed, `+` toward the left when facing the wall (the tangent is
  90 degrees left of the direction into the wall). Config: tolerance (10 mm), speed cap (0.05 m/s), gain, `max_correctable` (1 m).
  Speed is `min(kp*|e|, cap, sqrt(2*a*|e|) + creep)` with a dead band of half the tolerance, added to the standoff velocity.
- Two sources behind `LateralSource.error(LateralContext)` (None = cannot say; then the dock is not declared):
  (a) `PlannedStandLateral`: (planned stand - odom reference point) projected on the tangent; blind to odom drift by construction;
  (b) `WallEdgeLateral`: the visible wall extent `u_range = (u_min, u_max)` along the tangent from the reference point (the fitted
  plane's inlier u-range, when the detector provides it; `wall_detector` does not publish it yet) against a wanted distance to the
  left or right edge, or to the centre (`edge_lateral_error`, plain numbers). Independent of odom, but a view cut by occlusion or the
  field of view gives a wrong edge, not a missing one.
- Docked = standoff and yaw within tolerance, the base at rest, and (if enabled) lateral within tolerance.
- Base without lateral velocity (diff, skid): no sliding. Once square and at rest with the lateral error over tolerance, the status is
  `NEEDS_REAPPROACH` (outcome `lateral_unreachable`, `DockStep.reapproach`), and `plan_lateral_manoeuvre` returns the primitives
  back off, turn 90 degrees, drive the error, turn back, approach (`[('drive', m), ('turn', rad), ...]`, mirrored when the rear faces
  the wall). The caller runs them on odom and docks again. An error above `max_correctable` gives the same status on any base.
- Typed outcomes `DockOutcome`: `docked`, `timeout`, `lost_wall`, `lateral_unreachable`, `blocked` (lower_snake like the manager's
  stand_check reason codes); `outcome_of(status)`; `DockStep` carries `lateral_error`, `outcome`, `reapproach`; `WallDockController.result()`
  gives `DockResult(outcome, standoff_error, yaw_error, lateral_error, elapsed_s)` (outcome None while running; a non-finite lateral reading counts as no reading; `LateralConfig` rejects non-positive or non-finite values). `blocked` (commanding motion, measured speed under
  a quarter of it for `blocked_timeout_s`) is off by default (0). Relation to the manager's stand check (#169, which compares the
  arrival error with a tolerance from the reachability margin): that gate decides go, re-dock or abort after the base arrives; this term
  works inside the dock. A sensible tolerance for the lateral term is the same margin-derived value (`stand_check` position tolerance),
  not wired.

**Monte Carlo** (`python3 -m stand_navigator.dock_montecarlo --runs 200 --seed 1 [--base diff] [--lateral off|planned|edge|all] [--sweep]`,
`test/test_dock_montecarlo.py`): `StandController` drives to a planned stand on a noisy odometry estimate, then `WallDockController`
docks on a noisy plane. Noise: odometry Gaussian per step (0.3 mm, 0.3 mrad), per-run scale error (1 % sigma), a drift velocity with a
random walk (per axis 2 mm/s sigma, applied while the base moves), plane standoff 2 mm and yaw 3 mrad, wall edge 5 mm, dropouts (0.5 %
per step, 1 to 20 steps). Random starts 0.8 to 2 m before the stand, 0.8 m sideways, 0.5 rad yaw; wall 3 m wide, plane at x = 3 m,
standoff 0.55 m. Seeded: three random streams per run, so a sweep reuses the same starts. Output: docked rate, failure reasons by
outcome, `stand_timeouts` (the approach did not settle in 90 s) and mean/p50/p95/max of the TRUE final position, standoff, lateral and
yaw error (all runs, docked or not). Read the lateral row against the 10 mm tolerance and the standoff row against 5 mm plus the
2 mm plane noise (the base stops on a noisy reading, so the true standoff error exceeds the tolerance in the tail). **It measures the
controller logic against synthetic noise; the base moves exactly as commanded, the wall is a perfect plane, the noise levels are
assumptions. It is not a simulator or hardware result.**

Run of `--runs 200 --seed 1 --sweep` (Python 3.10, under a minute), final error in mm (yaw in degrees), mean / p95 / max:

| base | lateral | docked | failures | position | standoff | lateral | yaw |
|---|---|---|---|---|---|---|---|
| omni | off | 100 % | none | 14.2 / 31.9 / 36.8 | 5.9 / 8.5 / 9.5 | 12.1 / 31.3 / 36.0 | 0.20 / 0.71 / 1.17 |
| omni | planned | 100 % | none | 14.9 / 32.6 / 43.9 | 5.9 / 8.5 / 9.5 | 12.8 / 32.1 / 43.5 | 0.20 / 0.71 / 1.02 |
| omni | edge | 100 % | none | 8.2 / 14.0 / 18.6 | 5.5 / 8.4 / 9.6 | 4.9 / 13.2 / 18.5 | 0.16 / 0.52 / 0.71 |
| diff | off | 99.5 % | lost_wall 1 | 64.7 / 316.6 / 910.1 | 9.0 / 8.6 / 667.1 | 60.7 / 276.2 / 910.1 | 0.30 / 0.84 / 4.19 |
| diff | planned | 65.5 % | lateral_unreachable 68, lost_wall 1 | 95.4 / 454.7 / 1259.1 | 9.0 / 8.2 / 667.1 | 91.7 / 447.6 / 1259.0 | 0.31 / 0.84 / 4.19 |
| diff | edge | 95.5 % | lateral_unreachable 8, lost_wall 1 | 11.8 / 14.5 / 700.9 | 9.0 / 8.3 / 667.1 | 6.4 / 12.3 / 215.0 | 0.33 / 0.87 / 4.19 |

Drift sweep (per-axis drift sigma in mm/s against mean lateral error in mm, p95 in brackets; docked rate in %):

| drift | omni off | omni planned | omni edge | diff off | diff planned | diff edge |
|---|---|---|---|---|---|---|
| 0 | 8.9 (22) 100 | 8.3 (21) 100 | 4.0 (11) 100 | 14.8 (52) 100 | 25.4 (91) 82.5 | 5.5 (12) 95.5 |
| 1 | 9.6 (27) 100 | 9.5 (26) 100 | 4.0 (10) 100 | 24.3 (78) 100 | 40.1 (143) 76.5 | 5.2 (11) 97.5 |
| 2 | 12.1 (31) 100 | 12.8 (32) 100 | 4.9 (13) 100 | 60.7 (276) 99.5 | 91.7 (448) 65.5 | 6.4 (12) 95.5 |
| 5 | 22.5 (54) 100 | 48.1 (82) 99.5 | 6.1 (15) 100 | 256 (958) 92.5 | 343 (1277) 40 | 49.4 (425) 78 |
| 10 | 174.5 (1303) 95.5 | 379 (1888) 86.5 | 74.0 (18) 95.5 | 706 (1759) 82.5 | 907 (2285) 19 | 178 (1168) 51.5 |

What it shows, and what it does not: (1) the wall edge source holds the lateral error near the plane noise on the omni base and
cuts it by a factor of 2.5 at the default drift; the planned-vs-odom source does not help on omni (it cannot see drift, and by the
dock the approach has already put the odometry within the stand tolerance) and on the differential base it lowers the docked rate
from 99.5 % to 65.5 % (its odom-based error is not brought under 10 mm by the turns), so enabling it alone is counterproductive;
(2) lateral error grows with drift without a wall feature (about linearly to 5 mm/s, then the approach itself fails);
(3) the differential base shows 10 of 200 approaches where `StandController` did not settle in 90 s on a drifting odometry
(stand_timeouts, 5 %): the truth then wanders up to 0.9 m and the dock recovers standoff and yaw but not the lateral error; that is a
finding about the approach controller under this noise model, not fixed here; (4) on the differential base with the edge source
123 of 200 runs do at least one back-off, turn and approach manoeuvre (162 manoeuvres, they begin square: the plan turns out the
yaw error first) because the stand tolerance (20 mm) is looser than the lateral one (10 mm); it brings the lateral error down but
costs time (mean 35 s against 20 s), and 9 of 200 runs (4.5 %) fail: 8 `lateral_unreachable` (out of the 90 s dock budget or the 2
re-approaches) and 1 `lost_wall`; (5) at 10 mm/s drift the scheme is out of its range. The "docked" call is made on one noisy
reading, so some "docked" runs end with a true lateral error above 10 mm. The numbers are a property of the assumed noise. A
review of the first version found a stall in the harness's open-loop manoeuvre (zero-noise runs) and the harness plane noise not
reaching the wall direction; both are fixed, and these tables are from the fixed code (the shift against the first run is small: the
diff edge docked rate went from 94.5 % to 95.5 %).

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

**Stand check** (`stand_check`, default off; `core/stand_check.hpp`, `StandCheck`): after `navigate_to_stand` reports the arrival error
(the "arrived at the stand (error .. mm, .. rad)" line) and before the arm moves, the error is compared with a tolerance
`clamp(stand_check_margin_scale * reach_margin, stand_check_min_tol_m, stand_check_max_tol_m)` and `stand_check_yaw_tol_rad`.
Within: go. Outside: drive to the stand again (`stand_check_max_redocks`, default 1), then abort: the stand is failed and its columns
are not run (reasons `position_out_of_tolerance`, `yaw_out_of_tolerance`, `position_and_yaw_out_of_tolerance`, `redocks_exhausted`,
`invalid_measurement`). The reach margin is the parameter `stand_check_reach_margin_m` (a length; negative = unknown, then
`stand_check_fallback_tol_m`); the planner's per-stand margin (`offset_margin` of a verified plan is a share, not a length) is not passed
through yet, and the defaults are unmeasured. gtests use fake values (`stand_check_test`); not built in the ROS image, not run in a sim.

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
