# How to read a run

What a MuJoCo manager pass leaves behind, what each field means and where in the code it comes from. The worked example is the two-stand
pass with the MoveIt planners (`run_multi_stand_pass.sh`, scenario `two_panels_plain`, 2026-11-22; [README](../README.md#two-stand-pass-with-moveit-scored-on-the-tool-pose-mujoco),
[sim_mujoco.md](sim_mujoco.md#two-stand-pass-with-the-moveit-planners-2026-11-22)). Every number below is one the repository already
reports for that run. The run directory was not kept, so the example has no raw `segment_exec` lines or CSV rows: the formats are
shown without invented values.

## The files

All are written under the pass's output directory (`MULTI_STAND_OUT`, default `/tmp/multi_stand_pass`; single-stand passes use
`manager_pass_common.sh`, same names).

| File | Written by | What it is |
|---|---|---|
| `manager.log` | the manager launch, stdout and stderr | Everything the manager node logged: stand and column progress, `segment_exec` lines, the run summary |
| `run.csv` | `record_run.py` (`ContactTest.write_recording`) | One row per ground-truth force message: tool point, force, valve, tilt |
| `run.csv.wall.json` | same | The wall plane and extent the score is made against |
| score card | `ros2 run coverage_planner score_run` | A JSON card **printed to the terminal**; the pass scripts do not save it. Add `--report-out card.json` (through `MANAGER_PASS_SCORE_ARGS` or `MULTI_STAND_SCORE_ARGS`) to keep it |
| `run_film.npz` | `render_run.py record`, only with `MULTI_STAND_FILM=1` | Joint states, base pose and force for rendering a video |
| `client.log`, `record.log`, `film.log`, `world_model.log` | the pass script | Action client output (feedback and the result), recorder and loader messages |

## manager.log

**Per segment** (`MotionExecReporter::logSegmentRecord`, `motion_exec_reporter.cpp`), one line each:

```
segment_exec id=<n> label=<name> outcome=<state> mode=<mode> planner=<name> fsp_backend=<plugin or -> frame=<frame>
  wp=[<first>,<last>] attempted=<n> completed=<n> ratio=<c/a> path_fraction=<0..1> reason=<code> stage=<stage>
  planning_ms=<ms> execution_ms=<ms> fallback=<bool> mode_switch=<bool> traj_pts=<n> traj_s=<s>
```

- `outcome`: `planned`, `attempted`, `completed`, `partially_completed`, `skipped_unreachable`, `skipped_collision`, `skipped_no_ik` or `failed_runtime`.
- `mode`, `planner`: the planning mode and legacy planner name. `planner=` is `ompl` whichever plugin did the work; `fsp_backend` names the free-space plugin ([sim_mujoco.md](sim_mujoco.md)).
- `wp`, `attempted`, `completed`, `ratio`: the waypoint range of the segment, how many were attempted, how many completed, and their quotient.
- `path_fraction`: the planner's achieved fraction of the requested path (0 to 1).
- `reason`, `stage`: failure code and the stage it failed in; `none` when nothing failed.
- `planning_ms`, `execution_ms`: wall-clock time to plan and to execute, in milliseconds.
- `fallback`, `mode_switch`: whether a fallback planner or a mode switch was used.
- `traj_pts`, `traj_s`: point count and duration (`time_from_start` of the last point) of the trajectory handed to the executor; 0 when the planner gave no timing. A short `execution_ms` with a full `traj_s` and a truncated one are told apart this way.

**Run summary** (`logRunSummary`): `motion_exec_summary segments(total attempted completed partial skipped failed) waypoints(total completed ratio) timing(planning_ms execution_ms) switches(mode fallback) failures(recoverable terminal)`, the sums of the per-segment fields.

**Progress lines** (`manipulation_manager_node.cpp`): `<label>: contact success=<0|1> completed=<n>` after each column's contact segment;
`stand <s>: arm base (x, y) yaw <rad> reaches <n> pose(s)`; `stand <s> not reached (...): its columns are not run`; `<label>: retract failed`;
`Coverage plan complete: done=<n> partial=<n> failed=<n>` (single-stand run, from the column tracker). The multi-stand scripts grep these
lines after the run (`stand N:`, `arrived`, `not reached`, `contact success`, `dock failed`, `retract failed`, `stow`).

## run.csv

Header `t,x,y,z,force,active,tilt_deg` (`contact_test.py`):

| Column | Meaning |
|---|---|
| `t` | Seconds of the ground-truth message's stamp, minus the first row's. Simulated time, not wall time |
| `x`, `y`, `z` | `tcp_link` position from TF in the recording frame (`odom` for the multi-stand pass, `base_footprint` otherwise), metres |
| `force` | `abs(Fz)` of `/sim_mujoco/ft_ground_truth` in newtons: the true contact force, not the conditioned wrench the controller uses |
| `active` | Last `/tool/valve` state (0 or 1); only meaningful for non-contact tools |
| `tilt_deg` | Angle between the tool's +Z axis and the wall's inward normal (+x of the frame), 0 = square to the wall |

A row is written only when the TF lookup works, so a missing transform drops that sample.

## The score card

`score_run` (`score_run_main.cpp`, `run_scorecard.cpp`, `run_scorecard_io.cpp`) prints JSON. A sample is *working* when `force` is at
least `--engaged-force` (default 1 N). With `--max-wall-gap` or `--max-tilt` a working sample that is off the wall or tilted is refused
(`bad_pose`) and does not count as coverage.

| Field | Meaning |
|---|---|
| `passed`, `failures` | All limits given (`--min-coverage`, `--max-rms`, `--max-peak`, ...) met; the failed ones as text |
| `coverage.covered_fraction`, `covered_cells`, `target_cells` | Grid cells (`--resolution`, swath `--swath`) the working samples swept, over the cells of the wall minus obstacle keep-outs |
| `coverage.missed_cells`, `mean_overlap`, `outside_cells` | Cells never swept; mean sweeps per covered cell; cells swept outside the target |
| `force.rms_error_n` | RMS of (`force` - `--target-force`) over working samples (approach and release ramps count) |
| `force.peak_n` | Largest `force` over **all** samples, working or not |
| `force.in_band_fraction` | Working samples with the error within `--band` |
| `time.duration_s`, `time.engaged_s` | First to last sample; summed time steps ending on a working sample |
| `strokes` | Runs of consecutive working samples (a lift-off ends one) |
| `obstacle.contacts`, `contact_samples` | Episodes and samples with the tool point inside an obstacle footprint |
| `pose.tilt_mean_deg`, `tilt_max_deg` | Over would-be-working samples, refused ones included |
| `pose.wall_gap_mean_m`, `wall_gap_max_abs_m` | Signed tool-point height over the wall (+ in front), same samples |
| `pose.bad_pose_samples`, `off_wall_samples`, `tilted_samples`, `bad_pose_fraction` | Refused samples and why; only counted when the matching gate was given |

Two manager figures differ from the card by design. The manager's `peak_force_n` (action feedback and result, `client.log`) is tracked
only while a contact segment runs (`tracking_peak_`) and from the conditioned force topic; the card's peak is over the whole recording of the ground-truth force.

## Worked example: the two-stand pass

Two runs of the same pass are on record. The filmed one (B: live viewer on, host busy) and an earlier one (A: no viewer, host quiet):

| Quantity | Run B (filmed) | Run A | Where it comes from |
|---|---|---|---|
| columns done / partial / failed | 10 / 0 / 0 (stand 0: 6, stand 1: 4) | 10 of 10 | manager log, column tracker |
| coverage | 100 % (2790 of 2790 cells) | 1.0 | `coverage` |
| peak force, scored | 142.2 N | 22.1 N | `force.peak_n` |
| peak force, feedback | 20.9 N | not recorded | manager `peak_force_n` |
| RMS force error | 6.7 N (target 10 N) | not recorded | `force.rms_error_n` |
| samples in the 1.5 N band | 34 % | not recorded | `force.in_band_fraction` |
| tool tilt max | 31.8 deg | 0.15 deg | `pose.tilt_max_deg` |
| wall gap max | 15.8 mm | 0.9 mm | `pose.wall_gap_max_abs_m` |
| engaged time | 277 s | not recorded | `time.engaged_s` |
| simulated time | 551 s | not recorded | `time.duration_s`; the video is this at 12x, 46 s |

How to read B:

- **142.2 N versus 20.9 N.** Not a contradiction. The feedback peak only covers contact segments, and the scored peak covers every sample. The 142 N is one spike between the two stands, outside any tracked contact; its cause is not found. Run A had no such spike, so one run cannot say how often it happens.
- **34 % in band with 100 % coverage.** Coverage only asks that the tool was working (at least 1 N) over each cell. In-band asks that the force stayed within 1.5 N of 10 N, and the RMS error of 6.7 N shows it often did not. Coverage is not contact quality.
- **Tilt 31.8 deg but 0 tilted samples.** `tilted_samples` is only counted when `--max-tilt` was given, and `tilt_max_deg` includes every working sample. The docs call the tilt a transient and report 0 tilted and 0 bad-pose samples by the scorer's rule; which gates the filmed run used is not recorded, so treat 31.8 deg as measured and the zero count as depending on the gates.
- **277 s engaged of 551 s.** Most of the rest is approach, docking, the stow and the drive between stands, not working time.
- **Run A against run B.** Same pass, same code family, different load and viewer; both 10 of 10. Single runs; no statistics.

## Related

[sim_mujoco.md](sim_mujoco.md) (scripts, gates, other passes), [coverage_patterns.md](coverage_patterns.md) (the scorer), [manager_parameters.md](manager_parameters.md).
