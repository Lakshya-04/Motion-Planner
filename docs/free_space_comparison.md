# Free-space transitions: Cartesian, direct joint line, OMPL, cuRobo compared

Question (user): the path between two contact strokes should be about as long as the Euclidean distance between the
two tool positions. Is the free-space transition worth doing as a Cartesian line, and how does cuRobo compare?
This page holds what is measured, what is only reasoned, why the Cartesian attempt did not run where it was
expected to, the comparison harness, and the recommended routing. Anything not measured is marked **unmeasured**.

## 1. Measured so far (MuJoCo, two stands, tall wall, UR10e + lift, old double-pass plan, OMPL RRT-Connect)

| Transition | Joint path / straight joint line | TCP path | Euclidean chord | TCP path / chord |
|---|---|---|---|---|
| gap between two contact strokes (typical) | 1.6 to 1.7 | 1.66 m | 1.36 m | 1.22 |
| stand change (retract to next stand's approach) | 16 | 14.2 m | 1.6 m | 8.9 (lift travelled 6.3 m for 0.3 m net) |
| stow | trajectory of 375 points, 37 s | | | |
| "approach" between adjacent columns | 70 to 108 points, 7 to 11 s | | | |

The stand-change number is the joint-space problem, not a Cartesian one: the planner is free to wander because
nothing asks for a short path. Not measured: any of the same numbers for the direct joint line, the Cartesian line,
the optimal OMPL mode or cuRobo. The harness below produces them.

## 2. Why the Cartesian attempt does not run, or fails, for "approach" and "stow" today

`MotionPlanner::planSegmentWithStartState` already tries `CartesianPlanner` first for every `fsp` segment and keeps it
when `success` and `path_fraction >= request.min_path_fraction`. The log shows `mode_switch=true` where that did
not hold. Reading the code, there are two separate causes; the first one means Cartesian is never even tried.

1. **Joint-goal segments are refused outright (the main cause).** `CartesianPlanner::plan` starts with
   `if (request.target_poses.empty()) fail "missing_target_poses"`. A joint goal carries `target_joint_state` and no
   poses. The manager builds exactly these requests:
   - `stowAttempts()` (`target_joint_state = travel_state_`, `mode_hint=fsp`): every stow;
   - the approach of a column when `dock_ik_seeded_` or `dock_ik_continuity_` is on: `goal_q` is the seeded IK of the
     approach pose and the attempt uses it as a joint goal on attempt 0, on every even attempt and on every
     attempt after `recovery_keeps_branch` pinned it (`use_q`).
   So for stow and for most approaches the preflight returns in microseconds and OMPL runs (`mode_switch=true`).
   The preflight rejection was also silent. It is now logged:
   `segment label=...: Cartesian preflight rejected (failed, reason=missing_target_poses, truncation=..., path_fraction=..., joint_goal=yes)`.
2. **Pose-goal approaches (the odd attempts) have to pass four gates**, all in `cartesian_planner.cpp`, with
   `dock_seg.min_path_fraction = 0.99` (set on purpose: a partial 61 % line left the arm short of the pose):
   - jump threshold, defaults 0.3 rad per revolute joint and 0.05 m for the lift between two 0.01 m samples. A line
     whose IK flips the wrist or elbow branch is cut there (`truncation = jump`). Starting from a retract pose on
     another branch than the approach pose makes this likely;
   - collision of the whole robot at every sample (base, lift column and arm against the scene and the wall octomap):
     the code comment at `dock_seg` records a live 61.5 % truncation by base/lift self collision;
   - IK failure (no solution within the bounds at a sample);
   - the start state is clamped into the joint bounds first (this is not a failure cause).
   Which of the three hit each logged approach is **unmeasured**: the old log has no per-segment reason. The new
   preflight line prints `truncation=` and `path_fraction`, so one run gives the split.

What was changed for this (all opt-in, default behaviour unchanged):
- `moveit_cartesian` param `cartesian_joint_goal: true` (tolerance `cartesian_joint_goal_tolerance`, default 0.05):
  a joint goal gets the Cartesian attempt too. The goal's tool pose comes from forward kinematics, the line is walked
  with the usual jump, collision and IK checks, and the result is accepted only if the whole line (fraction 1,
  whatever the segment's `min_path_fraction`) ends within the tolerance of the goal joint state, i.e. on the goal's own
  IK branch. Otherwise it fails (`joint_goal_line_blocked` or `joint_goal_branch_mismatch`) and OMPL runs as before.
  Skipped when the request has a wall reference (contact segments).
- The preflight rejection log line above.

## 3. Cartesian line as a candidate of the optimal mode

`OmplPlanner::planOptimized` now builds, besides the straight joint line and the optimal OMPL runs, the tool's
straight line from the start pose to the goal pose (`tryCartesianLine`: MoveIt `CartesianInterpolator`, IK seeded
from the previous sample, every sample through `PlanningScene::isStateValid`, jump thresholds as above, accepted only
at fraction 1 and on the goal's branch, then TOTG). `selectBestCandidate` picks the winner by `free_space_objective`
(`length`: weighted joint path, `duration`: TOTG time); candidate order is direct, cartesian, OMPL, so on equal value
the cheaper one wins. Params: `free_space_cartesian_candidate` (default true inside the opt-in optimal mode),
`free_space_cartesian_eef_step` (0.01), `free_space_cartesian_goal_tolerance` (0.05), `free_space_cartesian_revolute_jump`
(0.3), `free_space_cartesian_prismatic_jump` (0.05). Note the consequence for the `length` objective: a valid straight
joint line is by definition the shortest in weighted joint space and always wins it; the Cartesian candidate wins when
the joint line is blocked, or under `duration` when it is faster. For the shortest TCP path use the harness numbers
to decide, there is no TCP-length objective yet. Host tests: `path_quality_test` (selection with a Cartesian
candidate, ties, duration objective, branch check). The MoveIt code is not built here (**unbuilt**).

## 4. The comparison harness

One command (in the sim image, with `motion_planner_moveit` and `sim_mujoco` installed and the repo mounted at
`/repo`):

```
FSC_LOG=/path/to/run/manager.log FSC_OUT=/tmp/fsc \
  src/simulation/sim_mujoco/scripts/run_free_space_compare.sh
```

It starts the sim with move_group and the wall in the planning scene (as `run_manager_pass.sh`), turns the ARMSTATE
lines of the log into transitions, runs `free_space_compare_node`, and writes `$FSC_OUT/results.csv`, `results_table.markdown`
and `results.png`. A hand list instead of a log: `python3 docker/tools/free_space_compare.py template --out t.json`,
edit the joint states (retract to next-column approach, stow to dock, dock to stow, stand change), then
`FSC_TRANSITIONS=t.json`. Optional: `FSC_MODES`, `FSC_BUDGET` (optimal time budget, 10 s), `FSC_REPEATS`.

Modes per transition (all on the same goal joint state, each from the same start):

| mode | what runs |
|---|---|
| `straight_joint` | baseline only: the straight joint line, not collision checked |
| `ompl_default` | the unchanged OMPL call |
| `ompl_optimal` | optimal mode (straight joint line + optimal planners + shortcut/smoothing), no Cartesian candidate |
| `ompl_optimal_cart` | the same plus the Cartesian candidate |
| `direct` | `free_space_direct_first`: straight joint line, OMPL when it is blocked |
| `cartesian` | Cartesian line to the goal pose, continuity IK, any branch |
| `cartesian_joint` | Cartesian line accepted only on the goal's own branch (the new joint-goal mode) |
| `curobo_default`, `curobo_optimal` | with `FSC_CUROBO_ROBOT=/abs/robot.yml` (needs the GPU python env; otherwise run `docker/tools/free_space_compare_curobo.py` in the gpu image on `moveit_results.jsonl` and pass both result files to `free_space_compare.py analyze`). The cuRobo world is empty unless `FSC_CUROBO_WORLD` (an N x 3 numpy cloud in the cuRobo base frame) is given. |

Recorded per (transition, mode): success and reason, route taken (`ompl`, `direct`, `optimal:<candidate>`), planning
time, duration after TOTG, points, joint path length weighted by 1/range, ratio to the straight joint line, TCP path
length (forward kinematics), ratio to the Euclidean chord. The chart has three panels per transition (TCP / chord,
joint / straight joint, duration). A transition taken from a log spans the free-space moves between two ARMSTATE
lines (retract plus approach, possibly two segments), and the log has no stored goal pose, so the goal is the next
ARMSTATE joint state. The analyser and the log parser are host-tested (`docker/tools/tests/test_free_space_compare.py`).
Packages to build: `motion_planner` (core), `motion_planner_moveit`, `sim_mujoco` (installs the script).

## 5. Recommendation (reasoned; to be confirmed or overturned by the harness numbers)

Analytic argument. For a short move (adjacent columns, TCP chord about 0.1 m, same tool orientation) the Jacobian
is nearly constant over the move, so the joint-space image of the straight tool line is nearly the straight joint
line: joint path ratio close to 1 (error of second order in the move length) and TCP ratio 1 by construction. A
10-sample line is also a few milliseconds to plan, against RRT-Connect's measured 70 to 108 points and 7 to 11 s.
The straight joint line has the same joint cost but a tool path that bows by the same second-order amount, so for
0.1 m the two are equivalent and Cartesian has the extra guarantee of a predictable tool path near the wall. For a
long move (stand change, 1.6 m chord, folded-arm start) the tool line crosses the robot's own lift column and base
or needs another IK branch: the jump and collision gates will reject it (**unmeasured**), while the straight joint
line is the shortest joint path and the lift moves 0.3 m instead of 6.3 m. Its TCP path is an arc through
the swept volume, whose length is **unmeasured**.

Proposed default routing for an `fsp` segment (cost order, first valid wins; each step falls back to the next):
1. `column_change` and other short moves (chord up to about 0.5 m, a number to confirm with the harness):
   Cartesian first (`cartesian_joint_goal: true` so joint-goal approaches take part).
2. Everything else, and anything Cartesian rejected: the direct joint line (`free_space_direct_first: true`),
   collision checked.
3. Direct line blocked: optimal OMPL (`free_space_optimize: true`) or, on the GPU lane, cuRobo optimal; plain RRT-Connect
   only as the last resort. With the optimal mode on, step 1 to 3 are candidates of one selection and the objective
   decides.
Not recommended: Cartesian-first for stow and stand change. The evidence needed to move this is the `tcp_ratio` and
`joint_ratio` of `cartesian` and `direct` for those classes, and the `truncation=` reasons now logged. cuRobo: its
trajectory optimiser minimises smoothness and time, not geometric length (`docs/curobo_free_space_optimal.md`), so
whether it beats the direct line on a clear transition is **unmeasured**; the harness gives the first numbers.
