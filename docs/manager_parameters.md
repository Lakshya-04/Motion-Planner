# manipulation_manager: parameters that are easy to miss

`manipulation_manager_node` declares about a hundred parameters (`declare_parameter` calls in `src/ros/manipulation_manager_node.cpp` and
`src/ros/execution/retiming_wiring.cpp`). Most are explained next to the feature they belong to ([sim_mujoco.md](sim_mujoco.md),
[sim_gazebo.md](sim_gazebo.md), [stand_navigation.md](stand_navigation.md), [reachability.md](reachability.md)). The ones below were
not described anywhere; the table gives the type, the default in the code and what the code does with the value. Set them in a params
file (`params_file:=` of `manipulation_manager.launch.py`) or with `--ros-args -p name:=value`.
`docker/tools/check_params.py` fails when a declared parameter is in no doc, config or launch file, so a new one must be added here or
on its feature's page.

## Coverage plan resampling

| Parameter | Default | Meaning |
|---|---|---|
| `coverage_z_step_m` | 0.1 | Step in z (metres, in `frame_id`) between the points a wall column is resampled to. |
| `coverage_z_max_m` | 1.60 | Upper z limit of a column in `frame_id`. |
| `coverage_eef_speed_mps` | 0.02 | Target tool speed (m/s) of the contact segments, handed to the planner as `target_eef_speed` (2 cm/s: "a controlled press, not a crash"). |
| `max_wall_penetration_m` | 0.12 | Largest distance a contact waypoint may lie behind the wall reference plane. A waypoint beyond it fails the segment with `wall_penetration_exceeded` (the check is in the OMPL planner's contact mode, `ompl_planner.cpp`). |

(`coverage_z_min_m` is described in [sim_mujoco.md](sim_mujoco.md); it is the lower limit paired with `coverage_z_max_m`.)

## Dock IK selection (`dock_ik_continuity:=true`)

These shape the candidate filter of `core/dock_ik_selector.hpp`. They have no effect while `dock_ik_continuity` is false (its default).

| Parameter | Default | Meaning |
|---|---|---|
| `dock_ik_linear_joints` | [0] | Indices (in the planning group, lift first) of prismatic joints. Their joint change is not wrapped to [-pi, pi] when the cost is computed. |
| `dock_ik_max_dq_rad` | 3.2 | A candidate whose cost (largest joint change from the reference configuration) is above this is rejected. |

### Reach preference (opt-in, `dock_ik_reach_fraction` > 0)

With the lift free the IK tends to dock with the lift near its start and the tool far out on the arm (tall-wall MuJoCo run: lift docked range -0.002..0.099 m of a 1.5 m stroke, tool up to 87% of reach). With the preference on, every seed is also tried with its lift moved to the value nearest the reference lift that keeps the tool at or under the target fraction, and a candidate at or under the target beats one above it whatever its cost. Needs `dock_ik_continuity:=true` and the TF `frame_id` <- `arm_base_frame`. No candidate under the target: the cheapest one as without the preference (logged as "over the target"). Gtested on the host with a fake IK; not run in a simulation.

| Parameter | Default | Meaning |
|---|---|---|
| `dock_ik_reach_fraction` | 0.0 | Wanted upper bound of tool distance from the `arm_base_frame` origin over `dock_ik_reach_m`. 0 = off. |
| `dock_ik_lift_cost_weight` | 2.0 | Multiplier of the lift (linear joint) delta in the dock IK candidate cost, so arm motion is preferred over lift motion. 1.0 restores the unweighted ordering. The `dock_ik_max_dq_rad` rejection stays on the raw delta. |
| `dock_ik_reach_m` | 0.0 | Arm reach (m) from the `arm_base_frame` origin. Must be > 0 for the preference to act. |
| `dock_ik_lift_joint` | 0 | Index of the lift joint in the planning group. |
| `dock_ik_lift_min_m`, `dock_ik_lift_max_m` | 0.0, 1.5 | Stroke the proposed lift is clamped into. |

## Multi-stand requests: near-duplicate stands and clearance

With `stand_max_span_m` > 0 the manager asks `plan_stands` for the columns group by group. Each group's search grid is anchored on that group's own targets, so neighbouring groups can return stands a few cm apart (a 3 x 2 m wall gave arm-base stands at y = -0.61, -0.57, -0.53 m). Two parameters of `manipulation_manager`:

- `stand_merge_tol_m` (default 0 = off; behaviour change when set: extra blocking `plan_stands` call per close pair, yaw is ignored): consecutive stands closer than this (plan view) are merged when `plan_stands`, asked for the union of their targets alone, returns exactly one stand with nothing unreachable or unplanned. Coverage is therefore unchanged; a pair the planner cannot serve from one stand stays as it was. Costs one extra service call per close pair.
- `stand_min_clearance_m` (default 0 = the planner's own default): written into the `min_clearance` field of every request the manager sends. The reachability node's `stand_min_clearance_m` is only a floor on that field, and applies on the node that answers `base_check_service`; `stand_strategy` is ignored by `~/plan_verified_stands` (selectStands, a weighted greedy cover), so `set_cover` has no effect when the manager points at the verified service.

The reachability node has a matching opt-in for stand planning, `stand_min_clearance_m` (default 0 = off), described in [reachability.md](reachability.md): a floor for the chassis-to-wall clearance of every `~/plan_stands` request.

## Column retry pass (`retry_failed_columns:=true`)

Off by default. After the main pass, every column that ended failed or partial gets up to `max_column_retries` more attempts. The policy (what, in which order, which variant, when to stop, what is recorded) is `core/column_retry.hpp`, gtested on the host with a fake attempt function; the node function `retryFailedColumns` only moves the robot. Each attempt: stow the arm and prime the continuity from the live state with the dock reference cleared (a clean pose); dock with the next-best dock IK solution that differs from the ones the earlier attempts docked with (`DockIkExclusion` in `core/dock_ik_selector.hpp`; when none is left the planner samples its own goal); for a navigation failure, or a dock failure on the second attempt, optionally a stand re-planned with the `plan_stands` service from that column's cells (`retry_replan_stand`). Columns are retried by stand (fewest drives), then by id. A contact overload is never retried. The pass stops on cancel, an unrecoverable stow, or `retry_stop_after_failed` columns in a row that did not recover. The verify result gets a `column_retries` block: per attempt the column, stand, attempt number, variant (for example `fresh_stow+next_ik+restand`), the failure before and after, the outcome and a note; per column the attempts and the final outcome; the columns not retried and why. Not run in a simulation yet.

| Parameter | Default | Meaning |
|---|---|---|
| `retry_failed_columns` | false | Enable the second pass. |
| `max_column_retries` | 1 | Extra attempts per failed or partial column (0 = none). |
| `retry_replan_stand` | false | Allow a re-planned nearby stand (multi-stand runs): at once after a navigation failure, from the second attempt after a dock failure. |
| `retry_allow_branch_change` | false | From the second attempt on, the different dock solution may be on the other IK branch (the wrist and jump limits still apply). |
| `retry_stop_after_failed` | 3 | Stop the pass after this many columns in a row used all attempts without recovering (0 = never stop). |

## Recovery jog

The jog moves the arm a few centimetres of joint space back toward the last good configuration before a retract or stow is retried.

| Parameter | Default | Meaning |
|---|---|---|
| `recovery_jog_speed` | 0.15 | Fastest joint speed along the jog (rad/s, m/s for the lift); the jog lasts at least 1.5 times the largest joint change divided by this (a smoothstep ramp has a peak slope of 1.5). |
| `recovery_joint_lower`, `recovery_joint_upper` | empty | Joint limits (planning-group order) the jog target is clamped into. When both are empty and the base check has a lift range (`base_check_lift_min_m` < `base_check_lift_max_m`), only the lift is limited, to that range. |

## Time parameterisation (opt-in, `retime:=true`)

Declared in `retiming_wiring.cpp`. Only `retime` is read when it is false. Gtested; not run in a simulation (see [ROADMAP.md](ROADMAP.md)).

| Parameter | Default | Meaning |
|---|---|---|
| `retime_tcp_link` | "" | Link whose speed is held constant on contact seams. Empty: the child link of the last active joint of the planning group. |
| `retime_accel` | 2.0 | Acceleration limit used for both seams and transitions (the limits are the model's velocity and acceleration limits; this value is only the fallback for a joint that has no acceleration limit in the model). |
| `retime_vel_scale` | 1.0 | Scale (0.01 to 1) on the joint velocity limits of both seams and transitions. It can only slow things down: 1.0 is already the model's limit, so there is no faster free-space setting here. It is not a free-space-only knob; see [tcp_speed.md](tcp_speed.md). |
| `retime_seam_jerk` | 20.0 | Jerk limit of contact seams (Ruckig smoothing). |
| `retime_transition_jerk` | 0.0 | Jerk limit of transitions; 0 leaves the transition time-optimal without a jerk limit. |
| `retime_transition_jerk_ratio` | 0.0 | Opt-in Ruckig smoothing of free-space transitions: jerk limit = ratio x each joint's acceleration limit. 0 = off (plain TOTG). Off by default: with 4.0 Ruckig failed on short moves in the MuJoCo run (66-72 planning failures per run). |

## Verify result: geometric coverage and path quality

All need `verify_result_path` (the file written when a task ends); without it they are switched off. Both are off by default and report only.

| Parameter | Default | Meaning |
|---|---|---|
| `verify_geometric_coverage` | false | Samples the tool's TF position while a stroke runs and compares it with the planned patch (`core/geometric_coverage.hpp`). |
| `verify_swath_m` | 0.10 | Width of the swath the tool covers; it is not read from the tool. |
| `verify_cell_m` | 0.01 | Grid cell size of the coverage grid (it may be coarsened for a large patch). |
| `verify_contact_force_n` | 1.0 | A sample counts as working when the tool force is at least this (contact tools; a spray tool has no contact, so all stroking samples work). |
| `verify_max_gap_m` | 0.05 | Working samples farther apart than this are not joined into a polyline, so a lift-off breaks the line. |
| `verify_max_standoff_m` | 0.04 | A sample works only within this distance of the plane through the plan's first pose (plan poses press past the wall by `contact_y_overshoot_m`, hence 4 cm). |
| `verify_min_geometric_percent` | 0.0 | The run fails below this geometric coverage percent, or when it could not be computed; 0 means not checked. |
| `verify_trace_hz` | 20.0 | Rate of the TF sampling timer (raised to at least 1 Hz) that feeds both checks; it runs as a spin of the node's wait loops. |
| `verify_path_quality` | false | Reports how close the executed joint path came to a singularity or a joint limit, as warnings in the verify result; it never fails a run. Needs `verify_path_chain_file`. |
| `verify_path_chain_file` | "" | Chain file of the arm joints (made by `reachability/scripts/urdf_to_chain.py`). |
| `verify_path_skip_joints` | 1 | Leading joints of the planning group that are not in the chain file (1 for a lift plus arm group, 0 for a bare arm). |
| `verify_path_min_sigma` | 0.02 | Smallest Jacobian singular value along the path below which a column is warned about. |
| `verify_path_min_limit_margin` | 0.02 | Smallest joint-limit margin along the path (a share of the joint's range, 0 to 0.5) below which a column is warned about. |

## Wall dock after navigate (`wall_dock_after_navigate:=true`)

Off by default. After a successful `navigate_to_stand` (and the `stand_check` gate when on) and before priming or docking, the manager sends the
stand navigator's `WallDock` goal (servo to a standoff from, and square to, the fitted wall plane). The measured standoff error (measured minus
goal) and yaw error go through the stand-check logic (`core/wall_dock_gate.hpp`, built on `StandCheck`): within tolerance goes on, outside sends
the goal again until `wall_dock_max_attempts` goals are used, then the stand is given up (columns not run, as for `stand_check`). A goal that
fails (no fresh plane, timeout) is retried inside the same budget and then counts as a failed navigation attempt (the normal `stand_nav_attempts`
retry applies). The log line is "wall dock: standoff error .. mm, yaw error .. rad", and with `verify_result_path` the result carries a
`wall_dock` list (per stand: `ok`, `abort`, `attempts`, `wall_gap_error_m`, `wall_yaw_error_rad`, `message`). The action has no tolerance
fields of its own (the dock node's `tolerance_standoff_m` and `tolerance_yaw_rad` decide when it reports success); the two tolerances below are the manager's.
Host gtest: `wall_dock_gate_test`. The node's `facing_rad` has to be the base axis that faces the wall.

| Parameter | Default | Meaning |
|---|---|---|
| `wall_dock_after_navigate` | false | Enables the step. |
| `wall_dock_action` | "wall_dock" | Name of the WallDock action. |
| `wall_dock_standoff_m` | 0.30 | Goal standoff from the reference point to the plane. |
| `wall_dock_planned_standoff` | false | Use each stand's planned standoff (arm base to the plan's wall) instead of `wall_dock_standoff_m`, so the dock does not pull the base off the stand. |
| `wall_dock_timeout_s` | 0.0 | Goal timeout; 0 uses the dock node's default. |
| `wall_dock_max_attempts` | 2 | WallDock goals sent per stand arrival at most (minimum 1). |
| `wall_dock_standoff_tol_m` | 0.02 | Largest accepted absolute standoff error. |
| `wall_dock_yaw_tol_rad` | 0.05 | Largest accepted absolute yaw error. |

## Column IK check (`column_ik_check:=true`)

The check itself is described in [reachability.md](reachability.md); these select the joints of the planning group (lift first).

| Parameter | Default | Meaning |
|---|---|---|
| `column_ik_prismatic_joints` | [0] | Indices of prismatic joints (the lift). |
| `column_ik_continuous_joints` | [] | Indices of unbounded revolute joints; their step is wrapped to [-pi, pi]. All other joints count as limited revolutes. |

## Chained columns (`chain_columns:=true`)

Between two columns of one stand the manager retracts and then plans an approach to the next column with OMPL and a joint goal, a randomized swing that differs every time even when the neighbour is 10 cm away. With `chain_columns` the approach is first tried as one straight Cartesian move from the previous column's dock pose; a failed or short move falls back to the normal approach. Needs `dock_approach_extra_m` > 0. Rule: `core/column_chain.hpp`.

| Parameter | Default | Meaning |
|---|---|---|
| `chain_columns` | false | Try the straight chained approach between columns of one stand. |
| `chain_max_shift_m` | 0.4 | Longest distance (m) between the previous dock pose and the next approach pose that is still chained. |
| `serpentine_columns` | false | Keep the plan's up/down direction per column, retract (straight, fsp fallback) at the stroke END, lower the tool before a stow from the top; use with `double_pass:=false`. Mock playback 3 x 2 m: 16/19 columns, 1090 s vs 15/19, 1179 s. |
