# cuRobo optimal free-space mode

A free-space plan that is longer than necessary executes just as slowly, so the cuRobo backend has an opt-in
quality mode that spends planning time to get the shortest/fastest path it can find. Default: off (unchanged
behaviour).

## Parameters (node `curobo_planner`, Python `CuroboPlanner` keyword arguments of the same names)

| Parameter | Default | Meaning |
|---|---|---|
| `free_space_optimize` | `false` | Switch the joint-goal plan (`plan_joint`, the manager's free-space moves) to the optimal mode. |
| `free_space_time_budget_s` | `20.0` | No new candidate starts after this many seconds. One cuRobo call cannot be interrupted, so the plan can overrun by one candidate. |
| `free_space_num_candidates` | `6` | Candidates planned with cuRobo (the straight line is always one more). |
| `free_space_objective` | `length` | `length`: weighted joint path length. `duration`: duration of the time-optimal trajectory (needs cuRobo's velocity and acceleration limits; the straight line is skipped without them). |

`run_curobo_node.sh start --optimize [--time-budget S] [--candidates N] [--objective length|duration]` sets them.
Raise the manager's `curobo` `timeout_sec` (`planning_curobo.yaml`) above `free_space_time_budget_s` plus the time
of one candidate, or the client gives up before the best plan returns.

Python-only tuning (constructor): `optimal_trajopt_seeds` (32), `optimal_num_knots` (24),
`optimal_interpolation_dt` (0.01 s), `optimal_finetune_attempts` (8, +1 per candidate), `optimal_max_attempts`
(60), `optimal_seed` (1234).

## What it does

1. The planner is built for quality: more trajectory-optimisation seeds, more B-spline knots, a finer
   interpolation step, time-optimal retiming on (`MotionPlannerCfg.create(num_trajopt_seeds, num_knots,
   interpolation_dt, enable_timeopt)` in the installed cuRobo; `plan_cspace` takes `finetune_attempts`,
   `max_attempts`, `enable_graph_attempt`).
2. The straight joint-space line start to goal is built with a trapezoidal time-optimal profile under cuRobo's
   velocity/acceleration limits and checked with cuRobo's collision/limit checker (`check_motion`, 0.02 step).
   It is a candidate only if the whole line is valid, so an unobstructed straight line always wins on length, and
   wins ties.
3. N cuRobo candidates run with different seeding (linear seed first, or graph seed from the first attempt) and a
   different random seed; each returns the first valid optimised trajectory.
4. `curobo_bridge/path_quality.py` (numpy only, no ROS/torch/cuRobo) scores each: weighted joint path length (each
   joint divided by its range, so lift metres and arm radians are comparable fractions of travel) and duration.
   `select_best` picks the lowest objective, ties broken by the other metric, then by order.
5. One line per plan: `free-space plan: candidates N, best <objective>=X (baseline straight Y, efficiency Z)`.
   Efficiency is baseline/best, at most 1. When the straight line is blocked, the baseline is the unobstructed
   line (the lower bound), so efficiency says how far the detour is from ideal.

The result's `route` is `straight` when the line won. `plan_pose` is unchanged (its goal is a pose, not a joint
state). The C++ `joint_limit_guard` only rescales time uniformly and never changes the path, so nothing needs
mirroring there; a straight candidate is timed under cuRobo's own limits, and the guard still checks every
returned trajectory against the configured limits.

## Measuring

`docker/tools/curobo_path_quality_bench.py` draws M random valid start/goal pairs, plans them in default and in
optimal mode and prints per-pair and summary tables of length, duration, planning time and the ratio. Needs a GPU
(see its docstring for the docker command). Status: written, host-tested for the scoring and the candidate
selection with a fake planner; the cuRobo calls are NOT yet run on a GPU, so the actual gain is unmeasured.

## Limits

- cuRobo's trajectory optimiser minimises its own cost (smoothness, time), not geometric path length, and the
  installed version exposes no direct path-length cost weight through `MotionPlannerCfg.create`; the length gain
  comes from more seeds, finetune passes and picking the best of N plus the straight line.
- The budget is checked between candidates only.
