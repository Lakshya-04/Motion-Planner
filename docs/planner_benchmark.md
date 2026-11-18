# Planner benchmark across backends

One set of joint-space free-space transitions, planned by MoveIt/OMPL, VAMP and cuRobo v0.9.0, on the same
scenes, judged by one planner-independent model. It complements
[spherization_benchmark.md](spherization_benchmark.md), which compares sphere sets on random start/goal
pairs; this one uses the moves the manager actually makes between stands: stand to stow, stand to dock.

Script: `src/manipulation/motion_planner_moveit/scripts/planner_benchmark.py` (stages `prepare`,
`run-vamp`, `run-moveit`, `run-curobo`, `report`); OMPL helper `planner_benchmark` in the same package.
It reuses `docker/tools/bench_spheres.py` (numpy FK, sphere model, VAMP and cuRobo runners).

## Problems

The UR10e profile `ur10e_nolift` (6 joints, arm-only URDF from `docker/tools/generate_robot_assets.sh`).
Named configurations: four stand-like (`stand_home` is the profile home; left, right and high
variations), two stow-like (folded over the base, folded to the side) and four dock-like (tool tip
8 to 15 cm from the wall face, tool axis within 20 degrees of the wall normal, sampled and free under the
reference model). Every ordered pair of configurations that is free in a scene is one problem.

| scene | content | problems |
|---|---|---|
| empty | nothing | 72 (9 valid configurations) |
| wall | slab 0.06 x 2.4 x 2.2 m, face 0.95 m from the arm root | 56 (8) |
| wall_box | the slab plus a 0.25 x 0.25 x 0.4 m box in front of it | 30 (6) |

The same start/goal pairs and the same boxes go to every backend (`vamp_problems.txt`, `problems.json`).
Validity of the configurations is decided once, by the reference model (FOAM spheres of the arm,
self-collision of links more than 2 movable joints apart ignored, boxes as world).

## Metrics

- success: a path (cuRobo: a trajectory) was returned by that backend, in that backend's own model. A backend
  whose own model rejects an endpoint counts a failure; the second column leaves those problems out (cuRobo
  accepted all 158 endpoint pairs, so that column equals the first for every row). What "success" means differs per
  backend, see the list below the results.
- time: wall clock around the call that plans, successful plans only, one run per problem per pass, three passes
  per backend (see Isolation). VAMP: RRT-Connect plus simplification. OMPL: RRT-Connect plus simplification, no
  `move_group`. cuRobo: `CuroboPlanner.plan_joint`, i.e. the pre-check, host to device copy, `plan_cspace` and
  the trajectory copied back to the host; the planner's own `total_time` is within 2 to 12 ms of it (median 2.2 ms
  in session A, 4.2 ms in B for `plan_cspace`). Problems cuRobo fails are not in its time figures (a failure takes
  20 attempts, tens of seconds, and is counted in the success column only).
- **Planning time is not trajectory time.** VAMP and OMPL return a geometric path; cuRobo returns a
  time-parameterised trajectory (B-spline knots, retimed under velocity and acceleration limits). The "trajectory
  s" column is that trajectory's duration (median 1.25 to 1.6 s): how long the arm would take, not how long the
  planner took. Only the planning is timed.
- path length: joint-space polyline length of the returned path or trajectory, radians (median over that backend's
  successful problems; the sets differ, so lengths are comparable only where the success column is the same).
- clearance: the smallest gap between any reference sphere and any box along the path densified to 0.05 rad
  (median and minimum over paths; negative is a penetration of the reference model). Empty scene: none.
- hits in reference model: paths whose clearance is negative.
- first call: cuRobo only. `construct + warmup` is `CuroboPlanner(..., warmup=True)` (Warp kernel compile or
  cache load, torch compile, CUDA graph capture); `first call` is the first `plan_joint` after that, timed alone.
- VRAM: device-wide used memory (`torch.cuda.mem_get_info`, polled every 20 ms, the counter `nvidia-smi` shows) minus the
  value before the planner was built, over the whole process (construct, warmup, all passes), as in
  `docker/tools/eval_curobo_mapper.py`. Warp allocates outside torch, so torch's own peak (also given) is lower.

## Results

Two complete sessions (A: 2026-11-18 22:05 to 23:08, node run until 23:14, B: 2026-11-18 23:14 to 2026-11-19 00:51), same problems, same
binaries, each = VAMP and OMPL three times (once before each cuRobo mode) and each cuRobo mode as three consecutive
passes in one process, so every row of a session comes from one window of one to one and a half hours. Machine: Intel i5-10500H
(6 cores, 12 threads), RTX 3050 Laptop 4 GB (driver 595.99), cuRobo `nvidia-curobo 0.9.0`
(cuRoboV2-Fast) in `motion-planner:gpu` / `full`, MoveIt and OMPL as shipped in `motion-planner:build` (Jazzy),
VAMP from the pinned Cricket header with 50 FOAM spheres. Every container ran with `--cpuset-cpus=10-11`.
**The host was shared and loaded (see Isolation): these are not times of an idle machine, and the two sessions
differ by up to 1.5x on the cuRobo rows for that reason.** Success counts are the same in both sessions unless two
numbers are shown. In the time columns: the median of the three per-pass medians, with the lowest and highest pass median in brackets
(p95 the same way); "all" pools the three scenes.

| backend | scene | success (A, B) | median ms, session A (3 pass medians min-max) | p95 ms, A | median ms, session B | p95 ms, B | path rad | clearance cm (median / min), A / B | hits in ref. model, A / B | trajectory s (median) |
|---|---|---|---|---|---|---|---|---|---|---|
| vamp | empty | 72/72 | 0.04 (0.04-0.07) | 0.06 (0.04-0.15) | 0.05 (0.04-0.07) | 0.12 (0.06-0.22) | 4.55 | - | 0 / 0 | - |
| vamp | wall | 56/56 | 0.05 (0.04-0.12) | 0.15 (0.13-0.45) | 0.06 (0.04-0.10) | 0.18 (0.13-0.49) | 4.65 | 13.4 / 0.1, 13.4 / 0.1 | 0 / 0 | - |
| vamp | wall_box | 30/30 | 0.07 (0.05-0.11) | 0.2 (0.2-0.3) | 0.08 (0.05-0.12) | 0.2 (0.2-0.5) | 4.52 | 12.7 / 0.1, 12.7 / 0.1 | 0 / 0 | - |
| vamp | all | 158/158 | 0.05 (0.04-0.10) | 0.15 (0.13-0.36) | 0.06 (0.04-0.08) | 0.18 (0.13-0.49) | 4.58 | - | - | - |
| moveit_ompl | empty | 72/72 | 32.9 (32.4-35.4) | 58.1 (54.4-65.5) | 44.4 (38.0-45.1) | 79.4 (70.8-87.2) | 4.55 | - | 0 / 0 | - |
| moveit_ompl | wall | 56/56 | 29.9 (28.2-45.8) | 61.3 (59.7-76.2) | 41.5 (39.7-52.5) | 85.7 (78.4-101.3) | 4.67 | 14.2 / -0.2, 13.4 / -2.9 | 2 / 3 | - |
| moveit_ompl | wall_box | 30/30 | 35.5 (32.0-44.5) | 62.5 (59.9-84.8) | 47.1 (42.7-47.3) | 99.8 (69.3-123.7) | 4.52 | 12.7 / -0.2, 12.7 / -0.2 | 2 / 2 | - |
| moveit_ompl | all | 158/158 | 31.9 (31.6-39.1) | 62.2 (59.8-74.6) | 43.5 (39.1-47.0) | 91.9 (74.6-97.8) | 4.58 | - | - | - |
| cuRobo plan_cspace | empty | 48/72 | 203.8 (199.7-278.6) | 395.7 (341.8-505.1) | 224.3 (151.9-415.3) | 455.6 (276.1-529.1) | 3.69 | - | 0 / 0 | 1.25 |
| cuRobo plan_cspace | wall | 36/56 | 192.2 (183.5-314.3) | 391.6 (333.6-558.7) | 232.8 (207.0-553.3) | 522.8 (419.0-574.5) | 3.77 | 11.1 / -0.2, 11.1 / -0.2 | 2 / 2 | 1.38 |
| cuRobo plan_cspace | wall_box | 22/30 | 268.5 (242.7-303.7) | 367.9 (300.5-544.6) | 325.0 (180.9-464.6) | 591.0 (329.2-598.3) | 2.71 | 10.6 / -0.2, 10.6 / -0.2 | 2 / 2 | 1.38 |
| cuRobo plan_cspace | all | 106/158 | 231.3 (203.0-247.5) | 485.5 (374.1-487.8) | 223.1 (189.8-442.4) | 563.4 (442.4-579.5) | 3.75 | - | - | 1.38 |
| cuRobo bi_rrt_trajopt | empty | 48/72 | 231.7 (166.1-232.2) | 237.7 (198.0-238.9) | 344.1 (338.6-344.3) | 451.3 (354.0-461.9) | 3.69 | - | 0 / 0 | 1.25 |
| cuRobo bi_rrt_trajopt | wall | 37/56 | 252.9 (216.3-258.6) | 302.7 (301.2-466.8) | 475.7 (386.9-497.3) | 553.4 (526.8-596.6) | 3.79 | 10.6 / -0.2, 10.6 / -0.2 | 2 / 2 | 1.38 |
| cuRobo bi_rrt_trajopt | wall_box | 22/30 | 263.1 (256.9-264.4) | 310.7 (305.5-457.1) | 394.0 (391.0-403.3) | 460.1 (457.7-472.8) | 2.72 | 10.6 / -0.2, 10.6 / -0.2 | 2 / 2 | 1.38 |
| cuRobo bi_rrt_trajopt | all | 107/158 | 248.6 (190.5-249.9) | 341.5 (273.3-378.3) | 386.3 (361.9-391.8) | 509.3 (503.0-515.3) | 3.75 | - | - | 1.38 |
| cuRobo bi_rrt_seed | empty | 72/72 | 23.6 (23.2-24.1) | 47.0 (41.5-49.3) | 29.8 (26.7-30.1) | 61.3 (54.0-67.4) | 4.56 | - | 0 / 0 | 1.60 |
| cuRobo bi_rrt_seed | wall | 56/56 | 28.5 (28.4-34.3) | 58.5 (50.1-76.9) | 32.2 (28.7-37.0) | 75.2 (57.7-93.5) | 4.62 | 11.5 / -0.7, 11.5 / -0.7 | 4 / 4 | 1.60 |
| cuRobo bi_rrt_seed | wall_box | 30/30 | 29.9 (24.4-31.9) | 62.2 (53.4-63.3) | 30.2 (29.7-33.4) | 74.5 (64.6-81.7) | 4.53 | 11.7 / -0.7, 11.7 / -0.7 | 4 / 4 | 1.60 |
| cuRobo bi_rrt_seed | all | 158/158 | 27.9 (26.8-28.2) | 55.8 (51.7-63.5) | 32.0 (28.4-32.7) | 64.7 (61.2-79.4) | 4.59 | - | - | 1.60 |

What "success" and the path judged here mean per backend (this is not a ranking, see below):

- **vamp**: RRT-Connect found a path in the 50-sphere Cricket model, 158/158.
- **moveit_ompl**: OMPL RRT-Connect (5 s limit) found a path that FCL accepts on the URDF collision meshes with the
  hops-2 matrix, 158/158; 2 to 3 of the paths touch a box in the reference model in each session.
- **cuRobo plan_cspace**: `MotionPlanner.plan_cspace` with cuRobo's defaults (attempt 0 a straight-line seed refined by trajectory
  optimisation, PRM graph seeds from attempt 1 on, up to 20 attempts) returned a trajectory that satisfies cuRobo's constraints in its own
  sphere model (644 spheres from `RobotBuilder`). 106/158. cuRobo accepted every start and goal as collision free
  (`check_states`), yet fails **52 problems, every one with `dock_1`, `dock_2` or `dock_3` as an endpoint (`dock_3` in 30 of them, `dock_2` in 22,
  `dock_1` in 8; dock-to-dock pairs count twice), in the empty scene too**; the failing set is the same in all passes. The seed-only route below finds paths for the same problems, so the graph search is
  not what fails; the trajectory-optimisation step is. I did not find out why (candidates: a collision-activation margin that makes
  the dock poses costly although the binary check passes, or the endpoint pose tolerance); it is a result of this cuRobo model and
  configuration, not a statement about cuRobo.
- **cuRobo bi_rrt_trajopt**: the GPU bidirectional RRT seeds every attempt, then trajectory optimisation. 107/158, the
  same failing set except one pair.
- **cuRobo bi_rrt_seed**: the RRT path is fitted to a B-spline, retimed and densely validated in cuRobo's model, trajectory
  optimisation is skipped. 158/158, but the path is cuRobo's own validated seed, not an optimised trajectory; 4 of its paths in each
  box scene touch a box in the reference model (up to 0.7 cm).

Path lengths: only the rows with the same success counts are comparable. VAMP, OMPL and `bi_rrt_seed` solve the same 158
problems (4.52 to 4.67 rad medians); `plan_cspace` and `bi_rrt_trajopt` solve the 106/107 problems that do not involve the
failing docks, so their medians (2.7 to 3.8 rad) are over easier problems.

### cuRobo: first call, warmup, VRAM

| mode | construct + warmup, s (session A / B, cold) | first `plan_joint`, s | VRAM peak, MiB device (above base) | torch max allocated, MiB |
|---|---|---|---|---|
| plan_cspace | 346 / 300 (cold kernel cache, quiet: 199) | 0.14 / 0.18 (0.12 cold) | 1070 (978) | 601 |
| bi_rrt_trajopt | 118 / 426 | 0.60 / 3.24 | 3390 (3298) | 2805 |
| bi_rrt_seed | 117 / 199 | 0.51 / 0.50 | 3280 (3188) | 2805 |

- **The first call is the construction, not the first plan.** Warmup (inside `CuroboPlanner(...)`) takes 2 to 7 minutes on cores 10-11,
  then the first `plan_joint` costs only 0.1 to 0.6 s (one 3.2 s outlier in session B on a loaded host). The one run with an empty Warp
  kernel cache (a fresh cache directory, host quiet, load average 1.9) took 199 s; runs with a warm cache took 117 to 426 s depending on
  host load, so the warmup is not mainly kernel compilation: it includes torch compile workers and CUDA graph capture, both CPU-bound, and
  is as variable as the load. A service that must answer within seconds has to build the planner at start and not on the first request.
- **Steady state** is every problem after the warmup and first call: the medians and p95s in the table above.
- **VRAM**: `plan_cspace` peaks at about 1 GB of the 4 GB card; both RRT modes at 3.3 GB (the RRT planner's buffers, 2.8 GB in torch
  alone), which leaves under 0.8 GB on a 4 GB GPU: the cuRobo `Mapper` evaluation in [curobo_v2_notes.md](curobo_v2_notes.md)
  needs 0.17 to 1.4 GB, so RRT planning and mapping do not fit together on this card. The peaks are over the whole process; I did not split
  warmup from the passes.

### cuRobo through the ROS node and the C++ client

`curobo_planner_benchmark` (new, `src/manipulation/curobo_planner/src/planner_benchmark.cpp`) drives
`curobo_planner::CuroboClientPlanner` against a running `curobo_planner_node` (cuRobo `plan_cspace` defaults, same robot config, boxes
through `setWorld`), one container, node in the background, cores 10-11. 21 problems (every third of the first 21 of each scene), three
client passes; 48 of the 63 plans succeeded (the others are the same dock failures). Wall clock around `CuroboClientPlanner::plan()`:

| session | node + client median / p95 ms | node's own `planning_time_ms` (whole ms, cuRobo `total_time`) median | service overhead per call (client wall minus node time) median (min-max) | in-process on the same ids, `plan_cspace` pass medians |
|---|---|---|---|---|
| A (quiet host, load average 2) | 188 / 356 | 184.5 | 3.6 ms (2.9-9.6) | 247 / 252 / 211 |
| B (loaded host) | 301 / 496 | 294.5 | 5.7 ms (3.3-10.7) | 189 / 200 / 438 |

The ROS layer adds 3 to 6 ms to a 200 to 300 ms plan (about 2 percent): about 1.5 ms over the in-process wrapper (which itself adds 2.2 to 4.2 ms
over `total_time`: the pre-check, tensor copies and the trajectory to the host). The in-process and node medians above differ by more
than that overhead, in both directions, because the host load changed between the runs (in B the third in-process pass ran while the load
average was above 10); the overhead column is the reliable number. The node's first plan took 135 ms (A) and 204 ms (B) after its
warmup, which the client waits for (up to 300 s) before the first call.

### Isolation: what was and was not controlled

- Every container of the benchmark ran with `--cpuset-cpus=10-11`; no other containers of mine ran, one at a time, `MAKEFLAGS=-j2` for the builds.
  A MuJoCo simulation container (`mfs_sim`) stayed up on the host on CPU cores 0-7, other users' browsers and jobs ran throughout.
- **The pinning does not isolate the cores.** `thread_siblings_list` says CPU 10 shares a physical core with CPU 4, and CPU 11 with CPU 5, which
  are in the simulation's range: its threads and mine ran on the two hyperthreads of the same cores. Pinning only stopped the scheduler from
  moving my processes onto busier cores.
- The host had about 1.8 GB of RAM available and 4.8 GB of swap in use during the runs.
- Reference workload (`planner_benchmark.py refload`: 40 chained 384 x 384 float64 matmuls, one thread, `taskset -c 10-11`, 15 repeats, run
  before and after every stage). Median of the repeats, ms: **session A** 406 at the start, 443 to 468 through VAMP, OMPL and the start of
  the `plan_cspace` passes, 86 to 90 from 22:52 until the end (23:14, range 86-101), i.e. a **5.4x** swing within one session;
  **session B** 436 at the start, 356 to 450 until 00:12, then 155 to 221 from 00:13 to 00:43, 95 to 107 at 00:43 (185 again after the node run): a **4.6x** swing. Within one
  measurement, min to max spanned about 50 percent of the median when loaded and 15 to 20 percent when quiet. Load average (1 min) ranged from 1.7 to 14.8.
- So: the cuRobo `plan_cspace` passes of A ran from 443 ms of the yardstick at the start down to 86 ms at the end, B's `plan_cspace` passes at about 410 and the
  `bi_rrt_trajopt` passes at 220 down to 170. The same mode differs 1.4x (OMPL: 32 against 44 ms) to 1.5x (cuRobo `bi_rrt_trajopt`: 249 ms quiet in A, 386 ms
  loaded in B) between sessions, one `plan_cspace` pass in B was 2x slower than its siblings (442 ms against 190 and 200), VAMP's
  medians moved between 0.04 and 0.12 ms. Three passes bound the spread inside one session (the parenthesised ranges); they do not make a quiet
  machine. One stray, unpinned `docker run` of the GPU image (listing package files, under a minute) happened during session A's `plan_cspace` passes (about 22:21).
- Only the large ratios survive this: VAMP (0.04 to 0.12 ms) against everything else, and cuRobo with trajectory optimisation
  (190 to 440 ms) against the rest. OMPL (32 to 44 ms) and the cuRobo seed route (28 to 32 ms) are not distinguishable here; absolute milliseconds are not
  to be trusted beyond the load they were taken under.
- OMPL here (32 to 44 ms median) is 3 to 4 times slower than the 9.8 ms of the previous version of this document, which ran on a different
  build of the same package and a different load; I did not find out which of the two it is. VAMP's median is unchanged (under 0.1 ms).

## How to read this, and what it does not show

- **The numbers are not a ranking of the algorithms.** Each backend plans in its own collision model:
  OMPL checks the URDF collision meshes through FCL (the SRDF matrix is the same hops-2 rigid-group one
  VAMP was generated with), VAMP checks 50 FOAM spheres (33 before the carriage fix below) through the Cricket header, and the reference
  model used for clearance is a third (FOAM spheres, numpy). They disagree.
- **VAMP and the endpoints (fixed).** The first run of this benchmark had VAMP reject 60 of 72 starts or
  goals in the empty scene (`stand_left`, `stand_right`, `dock_1` to `dock_3`) and fail every pair with
  `stow_side`. Cause: one link pair, `arm_upper_arm_link` against `lift_carriage_link`. The carriage is the
  0.04 m thick plate the arm sits on; FOAM gave it 6 spheres of radius 0.08 (its sphere count is capped by
  how well one bounding sphere fits the link), which bulge 6 cm above the plate. At those configurations the meshes accept the pose (OMPL does) but the
  upper-arm and plate spheres overlap by a few mm (-4.6 mm at `stand_left`). `rigid_pairs_srdf.py --hops 2` leaves that pair checked on purpose (a rigid platform must not
  hide the upper arm from the chassis; `test_rigid_pairs_srdf.py` pins it), and the reference model of this
  benchmark does not check it only because it counts movable joints for the hops rule. Joint order, joint
  limits and the other pairs were not the problem: the header's order equals the URDF's, its bounds equal the
  URDF's (elbow +-pi, the others +-2 pi), and the only other pair that touches in the ten configurations is
  `arm_upper_arm_link` / `tool_mount_link` at `stow_fold`, which really folds into itself and was already
  dropped as invalid. The `stow_side` failures were not investigated separately: with the plate spheres fixed they are gone,
  so the "narrow pocket" was the same bulge (valid start, but the paths into the pose are swept over the plate). Fix: `docker/tools/generate_vamp_robot.sh` now passes
  `--lift_carriage_link=4` to FOAM by default (23 spheres of radius at most 0.044 m on the plate, 50 spheres in
  total instead of 33), the ACM is unchanged. Before and after, same 158 problems, same seed:

| VAMP | scene | success | problems with valid endpoints |
|---|---|---|---|
| 33 spheres (before) | empty | 6/72 | 12/72 |
| 50 spheres (after) | empty | 72/72 | 72/72 |
| 33 spheres (before) | wall | 2/56 | 6/56 |
| 50 spheres (after) | wall | 56/56 | 56/56 |
| 33 spheres (before) | wall_box | 2/30 | 6/30 |
| 50 spheres (after) | wall_box | 30/30 | 30/30 |

  After the fix VAMP succeeds on every problem (0 reference-model hits). The numbers are from the same `problems.json`, the reference model is still the 33-sphere set the problems were chosen with. The moves in this set are free-space
  joint-space moves between named poses, so almost all of them are a straight line (median under 0.1 ms,
  p95 0.1 to 0.2 ms): this checks that the VAMP model agrees with the others on the endpoints and on
  the straight segments, not that VAMP's planner is tested on hard problems. VAMP's paths keep at least 1 mm
  from the boxes in the reference spheres (OMPL's go up to 20 cm into them).
- **OMPL solved everything the meshes allow, but 3 of 56 and 3 of 30 paths touch a box in the reference
  sphere model** (up to 20 cm deep). The meshes are tighter than the FOAM spheres, so this is the
  coarser model flagging the finer one, not proof of a collision with the real mesh; it shows that paths
  from one model are not safe in another, the same finding as the sphere benchmark.
- VAMP's numbers beat OMPL's by almost three orders of magnitude on the problems both solve, which is its
  design (SIMD collision kernel) and is consistent with the sphere benchmark. cuRobo's `plan_cspace` takes 0.2 to 0.3 s on the
  problems it solves and fails a third of the set; its RRT-seed route is in OMPL's range (28 to 32 ms) with a time-parameterised
  trajectory and 158/158 success, at 3.3 GB of VRAM. These are three different kinds of output (a geometric path, a
  retimed validated seed, an optimised trajectory), in three collision models.
- The OMPL runner is the OMPL planner on a MoveIt `PlanningScene`, not a `move_group` round trip: no
  request adapters, no time parameterisation, no ROS messages. Time through `move_group` is higher.
- One run per problem per pass, three passes (VAMP's sampler is a fixed Halton sequence, so its passes differ only in timing; OMPL
  and cuRobo's RRT are randomised): small sample (158 problems, 10 configurations), no confidence intervals beyond the pass ranges.
- The OMPL runner and `CuroboPlanner` both include costs the other does not (the latter copies tensors and the trajectory
  across PCIe, the former does not touch ROS messages); they are described in Metrics, not corrected for.
- Pose goals, Cartesian and contact planning are not part of this benchmark (VAMP and cuRobo-pose do not do
  them either; see the table in [backend_selection.md](backend_selection.md)).

## cuRobo v0.9.0: which modes can be called

Read from the installed `nvidia-curobo 0.9.0` package (`curobo/_src/motion/motion_planner.py`, `motion_planner_cfg.py`, the
`examples/getting_started/motion_planning.py` example; the wheel carries no CHANGELOG, so this is from the code, not the release notes):

- `MotionPlanner.plan_cspace(goal_state, current_state, max_attempts=20, enable_graph_attempt=1, finetune_attempts=1)`: joint-space
  goal. Attempt 0 refines a straight-line seed with trajectory optimisation; from attempt `enable_graph_attempt` on, a graph
  planner's path seeds it. This is what `CuroboPlanner.plan_joint` (and so the ROS node and the C++ client) calls, **mode `plan_cspace`**.
- The graph planner is chosen at construction: `MotionPlannerCfg.create(graph_planner_algorithm="prm" | "rrt", ...)`. `"rrt"` is the new GPU
  bidirectional RRT (`BiRRTPlanner`). With `enable_graph_attempt=0` it seeds the first attempt, then trajectory optimisation runs
  (**mode `bi_rrt_trajopt`**).
- `enable_trajopt=False` returns the RRT seed fitted, retimed and densely validated, with no trajectory optimisation (**mode
  `bi_rrt_seed`**, the "graph search + timeopt" route of the `enable_timeopt` docstring).
- Not exposed here: `plan_pose` and the goal-set / batch planners (this benchmark has joint goals), and `always_run_trajopt` /
  `run_trajopt_on_graph_seed` variants (only the three routes above were run).

`CuroboPlanner` takes `graph_planner_algorithm`, `enable_trajopt` and `enable_graph_attempt` for this (defaults are cuRobo's, nothing changes for the
node). Two bridge findings on the way: `check_states` called `graph_planner.check_samples_feasibility`, which the RRT planner does not have (it
lives on its `resources`), so it raised `AttributeError` in the RRT modes: fixed; and every cuRobo planner is one process, one planner (see
[curobo_v2_notes.md](curobo_v2_notes.md)), so each mode ran in its own container.

## Reproduce

All commands from the repository root; `$S` is the script. Assets are generated once (needs `CRICKET_SRC`, a Cricket checkout at the pinned
ref with `resources/templates`, and the GPU for the cuRobo config, about 2 minutes):

```bash
S=src/manipulation/motion_planner_moveit/scripts/planner_benchmark.py
export CRICKET_SRC=...   # Cricket checkout, pinned ref (docker/tools/build_robot_tools.sh)
docker/tools/generate_robot_assets.sh src/description/mobile_manipulator_description/config/profiles/ur10e_nolift.yaml \
  --out /tmp/bench_assets --only urdf,srdf,spheres,vamp,curobo      # URDF, FOAM spheres, VAMP header, cuRobo config (644 spheres)
python3 $S prepare --urdf /tmp/bench_assets/arm_only/resolved/robot.urdf --spheres /tmp/bench_assets/vamp/spherized.urdf --work /tmp/bench
# OMPL inputs: the URDF with mesh paths as file:///robot/... (the directory is mounted at /robot), and an SRDF with an `arm` group plus the hops-2 matrix
sed 's#filename="ur_description/#filename="file:///robot/arm_only/resolved/ur_description/#' /tmp/bench_assets/arm_only/resolved/robot.urdf > /tmp/bench_assets/ompl.urdf
sed 's#<robot name="mobile_manipulator">#&<group name="arm"><chain base_link="arm_base_link" tip_link="tcp_link"/></group>#' /tmp/bench_assets/vamp/rigid.srdf > /tmp/bench_assets/ompl.srdf
# build motion_planner_moveit once (about 8 minutes at -j2) and curobo_planner + curobo_bridge in the full image
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -e MAKEFLAGS=-j2 -v $PWD:/ws:ro -v /tmp/bench_ws:/out motion-planner:build bash -c \
  'source /opt/ros/jazzy/setup.bash; cd /out; colcon build --base-paths /ws/src --packages-up-to motion_planner_moveit --build-base /out/build --install-base /out/install --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release'
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -e MAKEFLAGS=-j2 -v $PWD:/ws:ro -v /tmp/bench_ws_full:/out motion-planner:full bash -c \
  'source /opt/ros/jazzy/setup.bash; cd /out; colcon build --base-paths /ws/src --packages-up-to curobo_planner curobo_bridge --build-base /out/build --install-base /out/install --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release'

# the yardstick, before and after every stage (host, same cores)
taskset -c 10-11 python3 $S refload --label "session start" --out /tmp/out/refload.jsonl
# VAMP and OMPL, one pass each (repeat 3 times; --skip-compile after the first VAMP pass)
python3 $S run-vamp --work /tmp/bench --header /tmp/bench_assets/vamp/Ur10eNolift.hh --struct Ur10eNolift --out /tmp/out/vamp_1.jsonl --image motion-planner:full --cpuset 10-11
python3 $S run-moveit --work /tmp/bench --urdf /tmp/bench_assets/ompl.urdf --srdf /tmp/bench_assets/ompl.srdf --install /tmp/bench_ws/install --out /tmp/out/moveit_1.jsonl --cpuset 10-11
# cuRobo, one mode per container, three passes in it (mode: plan_cspace | bi_rrt_trajopt | bi_rrt_seed). The cuRobo config stores absolute
# paths, so /tmp/bench_assets is mounted at the same path; a persistent HOME keeps the Warp kernel cache.
docker run --rm --gpus all --cpuset-cpus=10-11 --user $(id -u):$(id -g) -e HOME=/tmp/warp_home -e OMP_NUM_THREADS=1 \
  -v $PWD:/ws:ro -v /tmp/bench_assets:/tmp/bench_assets:ro -v /tmp/bench:/work -v /tmp/out:/out -v /tmp/bench_warp:/tmp/warp_home motion-planner:gpu \
  /opt/venv/bin/python /ws/$S run-curobo --work /work --cfg /tmp/bench_assets/curobo/Ur10eNolift.yml --mode plan_cspace --passes 3 --pass-index 1 --out /out/curobo_plan_cspace_1.json
# node + C++ client: node in the background, 3 client passes of 21 problems (limit per scene 7, stride 3)
docker run --rm --gpus all --cpuset-cpus=10-11 --user $(id -u):$(id -g) -e HOME=/tmp/warp_home -v $PWD:/ws:ro -v /tmp/bench_assets:/tmp/bench_assets:ro \
  -v /tmp/bench:/work -v /tmp/out:/o -v /tmp/bench_warp:/tmp/warp_home -v /tmp/bench_ws_full:/out:ro motion-planner:full bash -c '
  source /opt/ros/jazzy/setup.bash; source /out/install/setup.bash
  /opt/venv/bin/python -m curobo_bridge.ros_node --ros-args -p robot:=/tmp/bench_assets/curobo/Ur10eNolift.yml -p max_boxes:=16 > /o/node_log.txt 2>&1 &
  for p in 1 2 3; do /out/install/curobo_planner/lib/curobo_planner/curobo_planner_benchmark /work/vamp_problems.txt /o/node_$p.jsonl plan_cspace_node 7 3; done; kill %1'
# the table (results.txt, results.json in --work)
python3 $S report --work /tmp/bench --urdf /tmp/bench_assets/arm_only/resolved/robot.urdf --spheres /tmp/bench_assets/vamp/spherized.urdf \
  /tmp/out/vamp_?.jsonl /tmp/out/moveit_?.jsonl /tmp/out/curobo_plan_cspace_1_p?.json /tmp/out/curobo_bi_rrt_trajopt_1_p?.json \
  /tmp/out/curobo_bi_rrt_seed_1_p?.json /tmp/out/node_?.jsonl
```

`run-curobo --verify-invalid N` plans N problems per scene whose endpoints cuRobo's own model rejects (normally counted as failures
without calling the planner); it was not needed here (no endpoint was rejected) and `--limit N` is a smoke test.

## Backend routing test

`manipulation_manager/test/backend_routing_test.cpp` checks the routing half of the story: configs with
`free_space: vamp` or `free_space: curobo` and `segments: {contact: cartesian, transition: free_space}`
are parsed, built through `PlanningPipeline` over the pluginlib registry, and one transition and one contact
segment are planned. Transitions reach the selected backend (after the Cartesian attempt falls short, as
documented), contact segments stay on MoveIt Cartesian and never reach it. Where the real plugin loads and
initializes it is the one called; otherwise a fake stands in, and the test prints which it was. MoveIt
plugins are always fakes there.

Run result: in `motion-planner:build` VAMP is a fake and cuRobo is the real plugin (it initializes with
a node and would answer `unavailable` without the cuRobo node, which is irrelevant to routing); in
`motion-planner:full` both are real plugins. All three tests pass in both.
