# TCP speed: what is held constant and what is not

Only the tool speed over the **contact strokes** (the wall columns) is a requirement here. Free-space moves, approach, retract,
stow and base moves have no speed target; they only have to be safe and finish.

## Where the stroke speed comes from

| Step | Where | What it does |
|---|---|---|
| Target | manager parameter `coverage_eef_speed_mps` ([manager_parameters.md](manager_parameters.md)), default 0.02 m/s | Copied into every stroke segment as `target_eef_speed` (`coverage_output_json_source.cpp`, `manipulation_manager_node.cpp`). |
| Pass scripts | `MULTI_STAND_SPEED` in `sim_mujoco/scripts/run_multi_stand_pass.sh` and the Gazebo twin, default **0.01** m/s | Writes `coverage_eef_speed_mps: ${MULTI_STAND_SPEED:-0.01}` into the pass's `manager.yaml`, so a scripted multi-stand pass runs at 1 cm/s, not the node default of 2 cm/s. |
| Time stamps, `retime:=false` (default) | `CartesianPlanner::applyConstantSpeedTimeParameterization` (`cartesian_planner.cpp`) | Stamps every waypoint with `cumulative TCP path length / target_eef_speed`: constant speed by construction, no acceleration ramps at the ends (the first and last point carry zero velocity). A failure falls back to `applyFallbackTimeParameterization`, which is not constant speed. |
| Time stamps, `retime:=true` | `MoveItTimeParameterizer::retime`, role `kContactSeam` | The same constant-speed stamps, then Ruckig smoothing under `retime_seam_jerk`; a joint limit that cannot be met is a failure, not a silent slow-down. |

What the stamps do not guarantee: the controller has to follow them. The measured speed (below) is what the tool did, not what was commanded.

## What is held and what is not

| Motion | Speed held? | How it is timed |
|---|---|---|
| Contact stroke (`wall_column_N_contact`, spray strokes) | **Yes**, `coverage_eef_speed_mps` | Constant-speed stamps as above. |
| Press-down lowering before the stow (`stow_lower`) | Yes, same target (it reuses `coverage_eef_speed_`) | Cartesian segment with `target_eef_speed`. |
| Approach, unload and retract Cartesian segments | No | MoveIt Cartesian path with the planner's own default timing (`applyFallbackTimeParameterization`); `coverage_pass_demo_node` says outright that `target_eef_speed` is not set for the retract. |
| Free-space moves (stand to stand, stow, `prime_continuity`) planned by OMPL, MoveIt Cartesian, VAMP or cuRobo | No | Without `retime`: whatever the planner stamps (VAMP stamps at its plugin parameter `nominal_joint_speed`, 1.0 rad/s; cuRobo's trajectory goes through `joint_limit_guard`). With `retime:=true`: role `kTransition`, time-optimal TOTG within the model's joint velocity and acceleration limits, optional Ruckig jerk smoothing (`retime_transition_jerk_ratio`). |
| Base moves | No | `stand_navigator` `time_parametrize` against the base profile limits (`max_lin`, `max_lin_acc`, `max_ang`). |

## Making free-space moves faster

Speed does not matter in free space, so shorter wall-clock is the only interest. What exists:

- `retime:=true` puts transitions on time-optimal TOTG at the model's limits. That is already the fastest this code produces; `retime_vel_scale` (1.0) can only slow it down, and it also scales the contact seams' velocity limits, so it is not a free-space-only knob.
- `retime_accel` is only the fallback for a joint with no acceleration limit in the model; it does not raise a limit that exists.
- The remaining lever is the robot's own limits (the description's `joint_limits.yaml`, or VAMP's `nominal_joint_speed`), which is a robot-safety decision, not a pass knob.

So no new `free_space_velocity_scale` was added: there is no free-space-only scaling to expose, and adding one on top of the model limits would be a way to exceed them. Nothing in this change touches the stroke timing.

## Measuring it: TCP speed over the contact strokes

`score_run` (`run_scorecard.cpp`, `computeTcpSpeedStats`) adds a `tcp_speed` object to the card, from `run.csv` only (ROS-free, gtested in `run_scorecard_test.cpp`):

```bash
score_run --samples run.csv --wall run.csv.wall.json --target-speed 0.01
# "tcp_speed": {"windows", "mean_mps", "median_mps", "std_mps", "p5_mps", "p95_mps", "target_mps", "median_rel_dev"}
```

Windowing, so the numbers can be reproduced:

- Contact = a run of consecutive samples with `force > 3 N` (`--speed-force`). A gap of more than the window between two samples ends a run.
- Speed = straight-line distance of the tool point over at least 0.5 s (`--speed-window`) inside the run, divided by the elapsed time, one value per sample (so the windows overlap and are not independent). A single 10 ms step is not used: `run.csv` is rounded to 10 um, which is about 10 % noise on one step at 1 cm/s.
- A turn at the end of a column inside a window shortens the chord and lowers that value; so do pauses and the first moments of contact. This is why the mean sits below the median.
- `median_rel_dev` = (median - target) / target. The statistics are reported only; they are not a pass criterion.
- The base moves between stands but not during a stroke; positions are not corrected for it, so a base creep during contact would count as tool speed.
