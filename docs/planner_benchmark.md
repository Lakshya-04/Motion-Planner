# Planner benchmark across backends

One set of joint-space free-space transitions, planned by MoveIt/OMPL, VAMP and (where it can run)
cuRobo, on the same scenes, judged by one planner-independent model. It complements
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

- success: a path was returned (a backend whose own model rejects an endpoint counts a failure; the
  second column leaves those out).
- time: wall clock around solve plus simplification, successful plans only, one run per problem. VAMP and
  OMPL run on the CPU, single thread, same machine under load from other jobs (read times as order of
  magnitude).
- path length: joint-space polyline length of the returned path, radians (median).
- clearance: the smallest gap between any reference sphere and any box along the path densified to 0.05 rad
  (median and minimum over paths; negative is a penetration of the reference model). Empty scene: none.
- hits in reference model: paths whose clearance is negative.

## Results

Machine: the development host (CPU, shared, loaded); VAMP and OMPL with default settings; OMPL
RRT-Connect, 5 s limit, validity resolution 0.1 percent of the extent, then path simplification.

| backend | scene | success | success, valid endpoints | median ms | p95 ms | path rad (median) | clearance cm (median / min) | hits in reference model |
|---|---|---|---|---|---|---|---|---|
| vamp | empty | 6/72 | 6/12 | 0.1 | 0.1 | 4.50 | - | 0 |
| vamp | wall | 2/56 | 2/6 | 0.1 | 0.1 | 4.50 | 10.6 / 10.6 | 0 |
| vamp | wall_box | 2/30 | 2/6 | 0.1 | 0.1 | 4.50 | 10.6 / 10.6 | 0 |
| moveit_ompl | empty | 72/72 | 72/72 | 9.8 | 16.5 | 4.55 | - | 0 |
| moveit_ompl | wall | 56/56 | 56/56 | 9.7 | 17.2 | 4.76 | 13.4 / -3.9 | 3 |
| moveit_ompl | wall_box | 30/30 | 30/30 | 11.2 | 26.0 | 4.52 | 14.4 / -20.3 | 3 |
| cuRobo | all | not run | - | - | - | - | - | see below |

## How to read this, and what it does not show

- **The numbers are not a ranking of the algorithms.** Each backend plans in its own collision model:
  OMPL checks the URDF collision meshes through FCL (the SRDF matrix is the same hops-2 rigid-group one
  VAMP was generated with), VAMP checks 21-ish FOAM spheres through the Cricket header, and the reference
  model used for clearance is a third (FOAM spheres, numpy). They disagree.
- **VAMP rejects most of the endpoints** (60 of 72 starts or goals in the empty scene: `stand_left`,
  `stand_right`, `dock_1` to `dock_3` are in collision in its model although the reference model
  and the meshes accept them). Cause not isolated; the likely one is the self-collision pairs that the
  hops-2 rigid-group SRDF leaves enabled for the generated header against the simple hops rule of the
  reference model. Of the problems VAMP accepts, every pair with `stow_side` failed (start valid, no path
  within 100000 iterations, about 65 ms): the stow pose is in a narrow pocket of VAMP's configuration
  space. The VAMP times are those of the successes, which are the trivial stand-to-stand and
  stand-to-dock moves (a straight line), hence 0.1 ms.
- **OMPL solved everything the meshes allow, but 3 of 56 and 3 of 30 paths touch a box in the reference
  sphere model** (up to 20 cm deep). The meshes are tighter than the FOAM spheres, so this is the
  coarser model flagging the finer one, not proof of a collision with the real mesh; it shows that paths
  from one model are not safe in another, the same finding as the sphere benchmark.
- VAMP's numbers beat OMPL's by two orders of magnitude on the problems both solve, which is its
  design (SIMD collision kernel) and is consistent with the sphere benchmark.
- The OMPL runner is the OMPL planner on a MoveIt `PlanningScene`, not a `move_group` round trip: no
  request adapters, no time parameterisation, no ROS messages. Time through `move_group` is higher.
- One run per problem, one seed (OMPL and VAMP are randomised): no confidence intervals, small sample (158
  problems, 10 configurations).
- Pose goals, Cartesian and contact planning are not part of this benchmark (VAMP and cuRobo-pose do not do
  them either; see the table in [backend_selection.md](backend_selection.md)).

## cuRobo: not run

The GPU image could not start on the benchmark host: `nvidia-smi` fails with "Driver/library version
mismatch" (NVML 595.99) and `docker run --gpus all` fails creating the container. The host needs a reboot
or a matching driver. The runner is in place (`run-curobo`, the same one as the sphere benchmark, from
`motion-planner:gpu` with `--gpus all`; it needs a cuRobo robot config from `generate_robot_assets.sh`'s
`curobo` stage, which also needs the GPU) and its output goes into the same `report` stage, but no cuRobo
number is claimed here. The earlier sphere benchmark measured 0.2 to 4 s per plan on random pairs on the
4 GB RTX 3050 (see [spherization_benchmark.md](spherization_benchmark.md)); that is a different problem set.

## Reproduce

```bash
# assets for the profile (FOAM spheres, VAMP header); CRICKET_SRC is a Cricket checkout at the pinned ref
docker/tools/generate_robot_assets.sh src/description/mobile_manipulator_description/config/profiles/ur10e_nolift.yaml \
  --out /tmp/assets --only urdf,srdf,spheres,vamp
S=src/manipulation/motion_planner_moveit/scripts/planner_benchmark.py
python3 $S prepare --urdf /tmp/assets/arm_only/resolved/robot.urdf --spheres /tmp/assets/vamp/spherized.urdf --work /tmp/bench
python3 $S run-vamp --work /tmp/bench --header /tmp/assets/vamp/Ur10eNolift.hh --struct Ur10eNolift --out /tmp/bench/vamp.jsonl
# OMPL: build motion_planner_moveit, then run the `planner_benchmark` executable (see its header) with the
# arm-only URDF (mesh paths as file:// URIs) and an SRDF with an `arm` group plus the hops-2 matrix
python3 $S report --work /tmp/bench --urdf /tmp/assets/arm_only/resolved/robot.urdf \
  --spheres /tmp/assets/vamp/spherized.urdf /tmp/bench/vamp.jsonl /tmp/bench/moveit.jsonl
```

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
