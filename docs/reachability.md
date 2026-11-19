# Reachability analysis module (RM4D)

Answers, for a given robot arm: **can the tool reach this pose** (forward query) and **where
should the base be so that it can** (inverse query), from one compact precomputed map.
Package: [`src/manipulation/reachability`](../src/manipulation/reachability) (ROS-free core,
28 tests). Implemented here from the paper's idea; the other pieces below describe how it fits. Method: **RM4D**, Rudorfer, *"RM4D: A Combined Reachability and
Inverse Reachability Map for Common 6-/7-Axis Robot Arms by Dimensionality Reduction to 4D"*,
ICRA 2025, [arXiv:2410.06968](https://arxiv.org/abs/2410.06968) (project page
mrudorfer.github.io/rm4d). Per the paper's abstract, the map is far more compact and builds
about an order of magnitude faster than 6D maps with no loss of accuracy.

## Idea (from the paper)

A classic 6D reachability map discretizes position and orientation. For common 6- and 7-axis
arms the last wrist joint rotates fully and the base can rotate 360 degrees about its vertical
axis, which removes two dimensions: the map is indexed by four values (pz, theta, x*, y*)
around a canonical base position. One structure then serves both the forward query and the
inverse query, with no inversion cost. The paper reports accuracy holds even when those
assumptions are only partly satisfied.

## Why we need it

| Use | How |
|---|---|
| **Base stationing** | inverse query for the coverage poses of a wall region gives candidate base poses; pick stations that cover the most poses ([cleaning / stationing work in the roadmap]) |
| **Lift band design** | build the map for the **6-DOF arm only**; the lift shifts pz, so the map tells which wall rows each lift height covers. Choose the fewest bands, then index the lift between bands with it locked (see [design_principles.md](design_principles.md) section 4) |
| **Feasibility pre-filter** | reject unreachable coverage waypoints in microseconds before IK / planning (the manager's coverage feasibility analyzer) |
| **Recovery targets** | prefer known-state targets that are reachable from the current station ([learned_recovery.md](learned_recovery.md)) |
| **Arm / lift selection** | compare candidate arms and lift strokes on the same wall layout before buying hardware |

Assumptions to check for our robot: last wrist joint full rotation (UR family: yes), vertical
base axis (arm on a vertical lift: yes), base free to rotate (mobile base: yes). Contact work
adds an orientation constraint (tool normal to the wall); query with the constrained pose set.

## What exists

| Piece | Where |
|---|---|
| `Map4D`: canonical key (pz, theta, x*, y*), `record`, forward `isReachable` / `cellCount`, inverse `inverseQuery` (base positions; ring of candidates for a vertical approach axis), `mergeFrom` | `core/map4d.hpp` |
| `ArmModel` port (joint limits + FK/validity per configuration) and `ChainModel` (revolute chain, optional validity callback, fingerprint, `loadChainYaml`) | `core/arm_model.hpp` |
| `buildMap`: uniform joint sampling on N threads, deterministic for a seed, stats (valid, in window, saturated cells) | `core/builder.hpp` |
| `planLiftBands`: greedy set cover of target poses over candidate lift heights for a base station; reports unreachable targets | `core/lift_bands.hpp` |
| `chooseHorizontalStation` / `planHorizontalStands`: stands for a ceiling or floor patch (vertical tool axis): grid over the patch footprint plus a margin, lift bands per stand, greedy multi-stand cover, targets outside the lift + arm height window reported as `out_of_lift_range` | `core/horizontal_stands.hpp` |
| Versioned binary map file with a robot fingerprint (a map for another robot is rejected) | `core/io.hpp` |
| `scripts/urdf_to_chain.py`: URDF -> chain YAML (folds fixed joints, records axes and limits) | `scripts/` |
| `rm4d_tool build / info / bands` | `apps/rm4d_tool.cpp` |

Backends other than the built-in chain (Pinocchio, VAMP, cuRobo, MoveIt, with self-collision)
implement `ArmModel`; none is written yet, so the built-in chain only checks joint limits.

## Try it

```bash
xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e -o /tmp/robot.urdf
urdf_to_chain.py /tmp/robot.urdf --base arm_base_link --tip tcp_link -o ur10e_chain.yaml
rm4d_tool build ur10e_chain.yaml ur10e.rm4d --samples 3000000 --resolution 0.05 --extent 2.0 --zmin -1.8 --zmax 2.0
rm4d_tool bands ur10e.rm4d --wall-x 0.8 --z-min 0.1 --z-max 3.0 --lift-max 1.5 --mount-z 0.2
```

Measured on this machine (8 threads): 3 million samples in 0.5 s. UR10e, a 3 m wall 0.8 m in
front of the base: **2 bands** (lift 0.95 m covers 25 of 30 targets, 1.45 m the other 5). The same
wall 3 m away: all targets unreachable. Generic arm: 3 bands.

## Base-aware stationing

The map is indexed by the **arm base** position, but what must stay clear of a wall is the whole
**footprint**. `BaseModel` (`core/base_model.hpp`) carries the footprint polygon, where the arm sits on
it (`arm_mount_xy`, default the footprint origin), the drive type and the velocity limits, and loads from
the `base:` section of a robot profile (`loadBaseProfile`; every profile in
`mobile_manipulator_description/config/profiles` parses, a test checks it).

- **Clearance.** With `StationRequest::base` set, `min_clearance` applies to the nearest footprint
  point, not the arm base. `chooseStation` picks the base yaw that leaves the most clearance (ties go to
  facing the wall), skips stands where the footprint would not fit, and reports `yaw` and
  `footprint_clearance`. `standoff` and `max_standoff` still refer to the arm base. Without a base model
  nothing changes.
- **Travel.** `travelTime(base, from, to)` models the drive type: an omni base
  translates and turns together, a four-wheel-steer base crabs and spins one after the other; diff and skid-steer bases turn toward the goal, drive
  and turn to the final yaw, and reverse when that is quicker. `orderStands` visits stands greedily by that
  time, so a wall covered from several stands is ordered by what the base can actually do. Only the velocity limits are used; the profile's
  acceleration limits are not modelled, so short hops are underestimated (fine for ordering).
- **Try it.**

```bash
rm4d_tool station ur10e.rm4d --wall-x 3.0 --z-min 0.3 --z-max 2.2 --width 2.0 \
  --base-profile config/profiles/ur10e_husky_telescopic.yaml
```

With `--base-profile` the arm base height at lift zero comes from the profile's
`lift_mount_height_m` plus the lift column and arm mount (`armBaseHeightAtLiftZero`: 1.08 m on the
Ranger Mini, 1.12 m MPO-500, 1.17 m MiR100, 1.21 m Husky); `--mount-z` overrides it, and without a
base profile the default stays `--mount-z 0.2`.

Same 2 m x 1.9 m wall 3 m ahead, UR10e map, `min_clearance` 0.3 m: with no base model the arm base
stands 0.50 m from the wall; with a base model 0.60 m (Ranger Mini, MiR100, MPO-500) or 0.70 m (Husky),
every base turned side-on (yaw 90 degrees, short side to the wall), footprint clearance 0.31 to 0.36 m.
Footprints are nominal rectangles from the profiles. The mount height changes the bands: the lift
only goes up from 1.1 to 1.2 m, so the lowest targets of the wall (z = 0.3 m) are out of reach, and
the tool reports 4 (Ranger Mini, MPO-500), 6 (MiR100) or 9 (Husky) of the 110 targets unreachable
in 2 bands; the old `--mount-z 0.2` default reached all of them in 3 bands for every base.

### Several stands for a wide wall

`planStands(map, request, start)` (`core/stands.hpp`) covers a wall wider than the arm's reach: greedy
cover, each round taking the station that reaches the most remaining targets (`chooseStation`) and, among
equally good ones, the one the base reaches soonest from where it is (`StationRequest::from`, drive type
and footprint through `travelTime`). It returns the stands in visiting order, the targets and lift bands
of each, the driving time between them and the targets nothing reaches.

```bash
rm4d_tool station ur10e.rm4d --wall-x 3.0 --z-min 0.3 --z-max 2.2 --width 7.0 --multi 1 --start-y -3.5 \
  --base-profile config/profiles/ur10e_husky_telescopic.yaml
```

A 7 m wide, 1.9 m high wall (360 targets), UR10e map, start at one end, 0.3 m clearance:

| Base | Stands | Driving | Notes |
|---|---|---|---|
| Ranger Mini (strafes) | 3 | 5.1 s | 130 / 130 / 100 targets, 3, 3 and 2 lift bands |
| MiR100 (diff) | 3 | 6.6 s | same stands, turn-drive-turn between them |
| Husky (skid-steer) | 4 | 11.1 s | its 0.99 m footprint puts the arm 0.7 m from the wall: less reach, 7 bands per stand, and a fourth stand for 4 leftover targets |

Limits: greedy cover can leave a few stragglers that cost an extra stand and a backtrack (the Husky
row). Folding such a stand into a neighbour does not help there: its targets lie outside what the
neighbours reach. A cover chosen for total time instead of count, or a finer stand grid near the gaps,
might. Each stand is chosen on its own, so the order is "nearest next", not the best
tour. Not done: a yaw preference for contact work.

### Set-cover stand selection (`StandStrategy::SetCover`)

Greedy-by-panel takes the busiest window first, which can strand two lone ends 4 m apart and cost a third
stand. `planStands(map, request, start, max_stands, StandStrategy::SetCover, options)`
(`core/stand_cover.cpp`) formulates it as a set cover over station-to-target feasibility:

1. Every station of the `chooseStation` search grid (same free-side, footprint and clearance rules) is scored
   once: `planLiftBands` over all targets gives the set of targets it reaches at some lift height. The RM4D map
   is the prefilter; a station reaching nothing is dropped. This is one `chooseStation` round of work, then no
   more map queries per round.
2. Weighted greedy cover: new targets per `1 + travel_weight * travel_s` from the previous stand
   (`SetCoverOptions::travel_weight`, 0.05 per second), travel from the drive type through `travelTime`.
3. Fewer stands: while a cover one stand smaller exists, a depth-first search finds it (branch on the target
   the fewest stations reach, only stations no other station dominates, `node_budget` 200000 nodes per try; 0
   skips this step). The result is never more stands than step 2.
4. Redundant stands are dropped, the rest ordered nearest-next then 2-opt on the whole path cost (with a start
   pose it is part of the path; without a base model the order is along the wall and the travel stays 0), and each stand is moved to
   the cheapest station that still reaches the targets only it reaches.
5. Targets go to the first stand in visiting order that reaches them; the lift bands are rebuilt per stand.

The default stays `Greedy` and its behaviour is unchanged. Select the new one with `rm4d_tool station ...
--multi 1 --strategy set_cover` or with the `stand_strategy` parameter of `reachability_node`
(`greedy` default, `set_cover`; anything else logs a warning and uses greedy); the `~/plan_stands`
request and response are unchanged. `max_stands` truncates the visiting order and reports the rest in
`not_planned`. Not done: confirming stations with batched IK (the map is a prefilter only, as in `chooseStation`),
whole-path base refinement, and proof of optimality (the search is bounded).

Gtests (`stands_test`, generic arm, 0.8 x 0.5 m box base, strafing, start 0.5 m before the first target,
arm reaches about 2.4 m along the wall). A wall of four lone targets at y = -2.2, -0.9, 0.9, 2.2 plus a dense
column at y = 0 (3 targets): greedy 3 stands, 7.0 s driving; set cover 2 stands, 4.2 s (inner targets at
+-1.1 m: 7.4 s against 4.2 s). Other walls, never worse: 5 columns x 2 rows (10 targets) 3 / 6.8 s greedy,
3 / 5.8 s set cover; 6 columns 2 / 3.8 s both; 7 columns 3 / 6.4 s against 3 / 5.5 s (differential base:
8.3 -> 7.5, 5.6 -> 5.6, 8.0 -> 7.1 s). Same stand count where greedy was already minimal, the saving
there is the ordering and the choice of station among equals. Synthetic walls on one coarse generic-arm map,
no Gazebo or MuJoCo run.

## Horizontal surfaces (ceiling, floor)

`chooseStation` needs a wall plane and a free side, so it refuses a horizontal one. `planHorizontalStands(map,
request)` (`core/horizontal_stands.hpp`) is the counterpart for a ceiling (tool pointing up) or a floor (tool
pointing down). The tool axis is vertical, so the map has no azimuth: a cell depends only on the horizontal
distance and the height between base and target, and one stand reaches a ring around itself.

- Search grid: the patch footprint (bounding box of the targets) plus max(`margin`, the map's `xy_extent`), step
  `grid_step`: a station reaches a ring up to `xy_extent` around itself, so a target only reachable at 0.9 m needs
  stations that far out. Stands go under or over the patch, there is no free side. Ranking per station: fewest unreachable targets, then fewest lift bands,
  then base travel time (with a base model and `from`) or distance to the middle of the patch. Scan order is fixed,
  so the result is deterministic.
- Lift: `planLiftBands` over `lift_min..lift_max` gives the lift height per stand. A target whose world height no lift
  position brings into the map's vertical window (`heightWithinLiftReach`; the window is `z_min` up to
  `z_min + nz * resolution`, the map rounds `z_max` up to whole cells) goes to `out_of_lift_range`, which is
  also part of `plan.unreachable`, so "too high for lift plus arm" is told apart from "no stand within horizontal reach".
- A patch wider than one stand's reach is covered by the same greedy loop as `planStands` (most remaining targets
  per round, `max_stands`, `not_planned` when capped); indices are into the request.
- Not done: base footprint or obstacle clearance (a stand is any arm base position the map accepts; with a base model and a start the yaw is the arrival
  heading, without them 0), the
  set-cover strategy (greedy only), a check that the tool axis really is vertical (an inclined ceiling goes through
  the same map query but is not the case this was written for), a ROS service, use by `manipulation_manager`.
  Tests: `test/core/horizontal_stands_test.cpp` (9: one-stand ceiling, wide patch with several stands, ceiling above
  reach, mixed patch, floor, floor below the window, determinism, empty, single target).

### Global stand selection over explicit candidates (`selectStands`)

`selectStands(map, request, candidates, start, options)` (`core/global_stands.hpp`) takes all wall targets
and a candidate set (`makeCandidateStands(request)` gives the `chooseStation` grid: free side, footprint
clearance for the base model; any list of `CandidateStand` works) and returns a `StandSelection`:
`success`, a `code` (`ok`, `no_targets`, `invalid_request`, `no_candidates`, `unreachable_targets`) and a
`message`, the stands in tour order (targets, lift bands, mean margin, leg cost), `unreachable`, `tour_cost`,
`input_order_cost`, and whether the budget cut the tour short.

1. Each candidate is scored once with `reachableByHeight`; the share of lift heights that reach a target is
   the reachability margin of that pair.
2. Weighted greedy set cover: lowest `cost / newly covered`, with a fixed per-stand `cost = 1 +
   margin_penalty * (1 - mean margin)` (default penalty 0.5; 0 is plain greedy by count). With fixed costs the
   textbook bound holds: cost within H(n) <= 1 + ln(n) of the cheapest cover in those costs, so the stand count
   is within (1 + penalty) * H(n) of the fewest stands. Stands made redundant are dropped; each target goes to
   the chosen stand where its margin is highest (ties to the earlier pick).
3. Open tour from `start` (with a base model: `travelTime`; without one: metres, the nearest-neighbour seed starts at
   the lowest stand along the wall and 2-opt may reorder): nearest neighbour then 2-opt, and the cover's own order as a second 2-opt seed, so the tour is
   never longer than the cover's order. `time_budget_s` (default 2 s, 0 disables the clock; the clock starts with the 2-opt step, not with scoring) and
   `max_two_opt_passes` bound it; `tour_budget_hit` says it stopped early.

Targets no candidate reaches are never dropped: they come back in `unreachable`, `success` is false with code
`unreachable_targets`, and the rest are still planned. No random numbers; ties go to the lower index; the result
is identical across runs unless the wall-clock budget trips. `rm4d_tool station ... --multi 1 --strategy global
[--margin-penalty w --time-budget s]` prints it.

This overlaps `StandStrategy::SetCover` above, which stays the one `planStands` and the node use (it adds a
bounded exact search for fewer stands and moves stands to cheaper stations; this one takes caller-supplied
candidates, weights by margin and returns a coded result). Not done: wiring into `reachability_node` or the
manager, a comparison with the exact search on real walls; tested on synthetic generic-arm
walls (`global_stands_test`) only, no simulator.

### Confirming stands by IK and collision checking (`StandVerifier`)

RM4D is a prefilter: it ignores self-collision and obstacles and says only that a tool pose is kinematically
reachable from some base position. `core/stand_verifier.hpp` is the port that confirms it, and an opt-in second
pass of `selectStands` uses it.

- **Port.** `StandVerifier::verify(stand, lift_height, targets_world)` returns one `TargetVerdict` per target:
  `reason` (`ok`, `ik_failed`, `in_collision`, `out_of_limits`, `error`) and the joint solution when ok. The
  scene belongs to the implementation. `FunctionStandVerifier` is the fake for tests (a function of one target).
  A backend that returns fewer verdicts than targets gets `error` for the rest, never `ok`.
- **`verifyStand`** takes a stand, targets and per-target lift heights (those RM4D reaches, at most
  `max_lift_probes`, default 4, picked evenly) and reports per target: `confirmed` (some height solves at the stand
  itself), the failure that got furthest (in_collision > out_of_limits > ik_failed > error), `lift_margin` (share of
  heights that solve), `offset_margin` (with `jitter_m` > 0 the best height is retried with the base shifted by
  +-jitter_m along and across its heading; 1 without), `margin = lift_margin * offset_margin` (greater than 0 exactly
  when confirmed), the best height and its joints, plus greedy lift bands over the confirmed heights.
- **Second pass.** `StandSelectionOptions::verifier` (null = RM4D only, as before): `selectStands` verifies the
  stands its cover chose, drops every (stand, target) pair the verifier refuses, takes the IK margins in place of
  the RM4D ones for the re-ranking, and reruns the cover and tour until every chosen stand is fully confirmed or
  `verify.max_rounds` (default 64; a round verifies the chosen stands, so a wall whose every candidate is refused
  needs about one round per candidate; `verification.converged` is false when the limit stopped it). Each pair is
  verified once. Targets no candidate confirms land in `unreachable` and `rejected_by_verifier` (code
  `unreachable_targets`); `verification.refused` lists the pairs RM4D promised and the verifier refused with their
  reasons, and `verification.optimism()` = refused / verified pairs measures how optimistic RM4D is. Each
  `SelectedStand` carries its `verification` and its lift bands come from the confirmed heights. Deterministic given a
  deterministic verifier.
- **Real implementation.** `reachability_moveit::PortStandVerifier` (package `reachability_moveit`, ROS-free) works
  over the `motion_planner` `IkPort`, `CollisionPort` and `ScenePort`, so MoveIt (TRAC-IK or KDL, whatever the
  config sets) works today and any other backend that implements the ports works the same. The stand is
  expressed by moving the target: world target -> the stand's `base_footprint` -> an IK target for the `arm` group
  with the lift at 0 (shifted down by the lift height); the collision check runs on the `lift_arm` group with the
  lift joint set to the height, so lift parts are checked where they are. World boxes and wall points are re-posed into
  the stand's base frame and pushed through the `ScenePort` whenever the stand changes. A lift outside its travel
  is `out_of_limits` without a backend call; joint solutions outside the configured limits are `out_of_limits` too
  (backends that do not enforce limits); extra IK seeds can be tried when the first solution collides.
  `PortVerifierConfig` holds the group names, the tcp link, the arm base pose in the planning frame at lift 0 (mount
  x, y and height), limits, seeds and the world geometry.
- **Not wired.** `planStands` (and so `~/plan_stands` and the manager's `base_check`) is unchanged: it needs a MoveIt
  backend in `reachability_node`, which is not a small change, and its stations are not candidates in the sense of
  `selectStands`. The cuRobo client (`ik()` / `check_states`) is not wrapped; it would be a second
  `StandVerifier` (or a second `IkPort`).

`stand_verify_probe` (in `reachability_moveit`) measures RM4D against IK on a wall: it builds the candidates and
targets of a `StationRequest`, takes a deterministic sample of the (stand, target, lift height) triples RM4D
reaches and of those it does not, sends them to MoveIt and prints the agreement, then runs `selectStands` with and
without the verifier (`ros2 launch reachability_moveit stand_verify_probe.launch.py map_file:=ur10e.rm4d wall_x:=1.4
extra:=max_triples=100,seed_set=true,run_selection=false`, with the MoveIt demo running).

Measured once (UR10e profile on the telescopic lift, MoveIt demo on mock hardware, TRAC-IK, tcp_link; wall at x = 1.4 m, 25 tool poses y -0.6..0.6 and z 0.5..2.1 with the tool axis into the wall, 77 candidate stands, lift 0..1.5 m in 0.25 m steps; map 3 M samples at 5 cm; no obstacles, self-collision only; samples are a fixed stride through the triples, 102 of each): of the (stand, target, lift height) triples RM4D calls reachable, IK from the backend's default seed confirms 45.1% (46 ok, 51 in collision, 5 IK failed). The same triples with three UR-style folded seeds (`seed_set`, first collision-free solution wins) are confirmed 85.3% (87 ok, 10 in collision, 5 IK failed). So RM4D itself is right about kinematics for about 95% of the triples sampled; most of the single-seed refusals are IK solutions on a branch that hits the lift column, which another seed avoids. Of the triples RM4D calls unreachable, IK finds a collision-free solution for 1 of 102 (1.0%, seeded run). `selectStands` on that wall with the single-seed verifier: RM4D alone picks 1 stand with 0 unreachable; verified, it needed 2 stands, hit the 64-round limit (not converged: 1278 pairs verified, 467 refused = 36.5% optimism, 701 IK batches, about 5 minutes) and left 5 targets rejected by IK. One run each, one wall, one arm, no seed tuning beyond those three seeds, sampled not exhaustive.

Two things this shows for use: give the verifier several IK seeds (`PortVerifierConfig::seeds`), or a single
solver branch will refuse many stands RM4D rightly likes; and the selection pass costs one verifier batch per
(stand, height) per round, so against a real backend bound it with `max_lift_probes`, fewer candidates or a smaller
`max_rounds`.

### The verified service (`reachability_verified_node`)

`reachability_moveit` ships `reachability_verified_node`: the plain `ReachabilityNode` plus the MoveIt IK and collision
checks as a `StandVerifier`. It adds `~/plan_verified_stands`, which takes the same `PlanStands` request as `~/plan_stands`
and answers it with `selectStands` and the verifier, so a stand RM4D promises but IK or the scene refuses is dropped
and the cover is recomputed. The response carries the IK-confirmed lift bands and a message such as
`verified: 3 round(s), 12 of 80 RM4D pairs refused by IK or collision`. It needs a running `move_group`
(`docs/moveit_config.md`), and a request takes minutes on a real wall (about 5 minutes on the one measured, not converged
at 64 rounds), so use a long client timeout. Parameters of the verifier (`ik_group`, `ik_link`, `collision_group`,
`arm_mount_offset_z`, `arm_base_x/y`, `lift_max`, `seed_set`, `max_lift_probes`, `jitter_m`, `max_rounds`) are on the node
`reachability_verified_config`. Without a verifier (the plain `reachability_node`) the service answers with a message and no
stands. Status: unit-tested through the service with a fake verifier (accepting, and refusing a target everywhere);
the node itself has not been run against MoveIt, the stand cap and `stand_strategy` do not apply to it, the scene is
whatever `move_group` holds (stand-relative obstacle posing is not wired), and the manager does not call it yet.

## Limits and next steps

- **Request validation.** Search grids are checked before they are enumerated: `grid_step` must be finite and
  positive, standoff, margin and lift range finite, and a grid may have at most 250000 points (`kMaxGridPoints`),
  at most 100000 lift heights. `selectStands(request)` answers `invalid_request`, `planHorizontalStands` sets
  `code = kInvalidRequest` with a message (all targets in `not_planned`); `chooseStation` and
  `chooseHorizontalStation` return `found = false`. A vertical tool axis is stored at y* = 0 exactly, so inverse
  queries and forward queries agree for every azimuth.
- **Sampling density.** Uniform joint sampling leaves cells empty at fine resolution: 3 M samples
  set 11% of a 5 cm map. Use at least ten samples per reachable cell, or a coarser map; the
  saturated-cell count in the build stats shows where more samples add nothing.
- **Self-collision** is not checked by the built-in chain; confirm stands with a `StandVerifier` (above) before
  trusting them, or plug a collision-aware `ArmModel` in.
- **Joint range assumption.** The reduction needs the first joint to rotate a full turn and the
  last joint to rotate fully (UR family: yes). Check a new arm against brute force.
- **Done since the first version:** the ROS shell (`reachability_node`, see below: `~/is_reachable`,
  `~/get_base_candidates`, `~/plan_lift_bands`, `~/plan_stands`) and selection over several stands
  (`planStands`, `orderStands`).
- **Not done:** RViz slices of the map, a validation harness against brute-force 6D sampling,
  orientation-constrained wall queries, a yaw preference for contact work, and a caller that
  acts on the result: the manipulation manager only runs a verify-only stance check
  (`base_check`, see above) and never asks this module for a different base pose.

## File format

Own versioned format: header (format version, robot profile hash, `MapParams`, joint-limit
snapshot) + the 4D cell array. A map is invalid for a different robot or limits (hash check).
Maps are build artifacts: stored outside git, reproducible from the profile.

## Evaluation

- **Accuracy** against brute-force sampling of the 6D reachable set: precision / recall per
  workspace region and per resolution.
- **Build time** and memory versus resolution (parallel shards, OpenMP / GPU).
- **Query latency** for forward and inverse queries.
- **Assumption stress:** limited wrist range, restricted base rotation (e.g. docking), tool
  orientation constraints.
- **End-to-end:** share of coverage waypoints on a real wall that the map says are reachable
  versus what the planner actually reaches.

## Provenance

The method and its public code come from the paper above. An earlier production stack had its
own C++ implementation (map, builder with Pinocchio and VAMP backends, numerical IK, file
format, apps for building and querying, paper-accuracy tests); it informed the module list and
test ideas here, but its code is not used. Our implementation is written from the paper;
check the license of any public reference code before reusing it.

## ROS shell

`reachability_node` (package `reachability`, interfaces in `reachability_msgs`) loads a map file
built with `rm4d_tool` and serves it:

| Service | Request | Answer |
|---|---|---|
| `~/is_reachable` | tool pose in the arm base frame | reachable, sample count |
| `~/get_base_candidates` | tool pose in the world, arm base height | base (x, y) candidates |
| `~/plan_lift_bands` | wall targets, base station, lift range and step | bands + unreachable targets |
| `~/plan_stands` | wall targets and plane, lift range, optional start stand | stands in visiting order (arm base x, y, base yaw, targets, lift bands, travel time) + unreachable targets |

Parameters: `map_file` (required), `stand_strategy` (`greedy` default, or `set_cover`, see the section above), `robot_fingerprint` (rejects a map built for another robot), `base_profile` (optional robot profile YAML with a `base:` section; `~/plan_stands` then keeps the footprint clear of the wall and reports driving time for the drive type, otherwise the footprint is ignored; a set `base_profile` that cannot be loaded leaves the node not ready, like a missing `map_file`).

`~/plan_stands` rejects `grid_step` below 0.02 m, `lift_step` below 0.01 m, `lift_max` (when set) below `lift_min`, and searches whose work (stations x lift heights x targets) exceeds 5e7 per stand, so one request cannot keep the node's callbacks busy for minutes. All positions (targets, wall point, returned stands) are in the same world frame the caller chose; the service does not transform.
The map does not know which side of a wall is free, so base candidates come back on both sides of
the target plane; the caller (stationing logic) picks the free side.

```
ros2 launch reachability reachability.launch.py map_file:=/path/ur10e.rm4d
```

## Checking the base stance from the manager

`manipulation_manager` can ask `~/plan_stands` whether the base, where it stands, reaches the whole coverage plan
(`base_check:=off|warn|enforce`, default `off`). It sends the plan's poses in the base frame with the wall in the
direction of `wall_direction` (a vector in the planning frame, default `[0, 1, 0]` = +y; the MuJoCo and Gazebo scenes
have the wall at `[1, 0, 0]`; the dock standoff, the contact overshoot and the wall reference orientation use the
same direction) and accepts the stance only when exactly one stand covers
every target and that stand is within `base_check_tol_xy_m` (0.15) and `base_check_tol_yaw_rad` (0.1) of the
origin (or, with `world_frame` set, of the arm base's pose in that frame, from TF). Otherwise it reports where the base would have to be, or that several stands are needed. `warn` logs and
runs the plan; `enforce` does not run it, and also refuses when the service is unreachable. Set
`base_check_lift_min_m`, `base_check_lift_max_m` and `base_check_arm_mount_z_m` to match the robot (0 selects the
service defaults). This only verifies; it never moves the base (that waits for base motion in the simulators).

