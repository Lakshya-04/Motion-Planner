# Reachability analysis module (RM4D)

Answers, for a given robot arm: **can the tool reach this pose** (forward query) and **where
should the base be so that it can** (inverse query), from one compact precomputed map.
Package: [`src/manipulation/reachability`](../src/manipulation/reachability) (ROS-free core,
28 tests plus `redundancy_test`). Implemented here from the paper's idea; the other pieces below describe how it fits. Method: **RM4D**, Rudorfer, *"RM4D: A Combined Reachability and
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
| **Lift band design** | build the map for the **arm only** (6 or 7 joints, see "Redundant arms"); the lift shifts pz, so the map tells which wall rows each lift height covers. Choose the fewest bands, then index the lift between bands with it locked (see [design_principles.md](design_principles.md) section 4) |
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
| Column pose sequences: approach, contact, strokes, retract checked against the map per stand and lift height; per-pose score (`scorePose`), singular value and joint-limit margin of a verifier's solution; stands that keep columns whole (`planColumnStands`); the "how much can we do" analysis and its JSON; `column_reach_tool` | `core/column_sequence.hpp`, `pose_quality.hpp`, `column_stands.hpp`, `column_analysis.hpp`, `apps/column_reach_tool.cpp` |

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

## Redundant arms (7 or more joints)

The builder, the chain model and the file format take any number of revolute joints; the joint count
comes from the chain (`urdf_to_chain.py` on the profile's arm, one entry per `joints.arm`), nothing in the
map depends on six. A 7-axis arm reaches a tool pose with a one-parameter family of configurations (the
elbow swivel), so a map has to say what "reachable" means for it. Policy, stored in the map header:

- **Union (default).** Joints are sampled uniformly over their limits, so a cell is set when **some**
  null-space value reaches it. The map answers "is there any arm configuration (within joint limits)
  that puts the tool here". This is also exactly what a 6-DOF map is (no freedom to choose), and for a 6-DOF
  chain the sampling is unchanged: a gtest replays the previous builder and compares every cell for a seed.
- **Restricted (`BuildOptions::redundancy`).** One joint (`joint`, usually joint 3, the upper-arm roll of
  the 7-axis arms) is drawn only from `values` (+- `tolerance`), the others uniformly. The map then
  answers "reachable with that joint near one of these values", which is the right question when the
  controller or IK keeps the elbow near a preferred setting. `rm4d_tool build chain.yaml out.rm4d
  --redundant-joint 3 --redundant-values -0.5,0,0.5 --redundant-tol 0.05` (joint index is 1-based on the
  command line).
- **Header.** Map format version 2 stores the arm's joint count and a text describing the policy
  (`rm4d_tool info` prints it; `MapInfo` / `loadMap(path, fingerprint, &info)` in C++). Version 1 files
  still load and report "not recorded".

Limits, stated plainly: the union is optimistic for a planner that holds the swivel fixed, and a cell
says nothing about how many elbow angles reach it beyond the sample count; joint 3 is a proxy for the
null-space parameter, not the swivel angle itself (they coincide only for a spherical shoulder with no
offsets, which none of the shipped arms has exactly); restricting one joint does not guarantee the
remaining six can realise the pose away from singularities; the 4D reduction still assumes joint 1 and the
tool-axis joint rotate fully (Gen3 and xArm 7 do; iiwa 7 and the Panda family stop short of a full turn, which the
paper reports as a small accuracy loss). Self-collision and obstacles are not in the map, as before. Tested on
a Panda-kinematics chain at 0.1 m resolution (`redundancy_test`: fresh samples reachable, out-of-reach poses
not, inverse query finds the true base, restricted map smaller than the union and consistent with it);
not compared against IK or a simulator.

**Stands on a 7-DOF map.** `planStands`, `selectStands`, `planColumnStands`, the stand verifier and the node read only
the map and tool poses; none of them assumes six joints (the verifier's joint solutions are plain vectors, any length),
so no code change was needed there. `stands_redundant_test` runs them on the Panda-kinematics map: a wall patch gets
stands whose every target is reachable in the map from that stand and lift height, columns stay whole under the
union, set cover is no worse than greedy, and a fake IK (7-joint solutions) that refuses a stand makes the plan route
around it, or report the targets as `rejected_by_verifier`. What the policy changes: pinning joint 3 at 0 (+- 0.05)
left 22101 of 22216 non-zero cells (0.1 m, 20 degree cells, 2 million samples), and about 1 percent of poses reached
with the elbow swung out are missing from it, so at this resolution the restriction is a small subset of the union.
Cell counts do change (the pinned map concentrates the same samples), so `count_ref` of `MarginConfig` is per map.
`~/plan_stands` and `~/plan_verified_stands` now return `arm_dof` and `redundancy` (the map header text) and the
node logs them at start and per request, so a result says which null-space samples it rests on. Unverified against
IK of a real 7-axis arm; the union stays optimistic for a planner that holds the elbow fixed.

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

`chooseStation` also accepts a horizontal plane now (see "Plane-generic stands and base free space" below); `planHorizontalStands` is the older, separate planner. `planHorizontalStands(map,
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
- **`planStands` with confirmation.** `planStandsVerified(map, request, start, max_stands, strategy, cover, options)`
  (`core/stands.hpp`) runs `planStands` (greedy or set cover), verifies every stand's targets with the same
  `verifyStand` (heights from RM4D), keeps the confirmed ones (lift bands become the confirmed bands) and puts the
  refused targets back into a pool. The next round plans only the pool with the refusing stands avoided
  (`StationRequest::avoid_stations` / `avoid_radius`, honoured by `chooseStation` and the set-cover grid), up to
  `max_rounds` (8). What no stand confirms is `rejected_by_verifier`, never covered, and `refused` lists every
  (stand, target, reason). With a base model the travel legs are recomputed for the final order. Without a verifier it is
  `planStands`. Tested with fake verifiers (`stands_verified_test`: accepting verifier changes nothing, a refused stand is
  replaced, targets refused everywhere are reported, everything refused terminates, set cover, travel). Wired behind
  `~/plan_stands` as opt-in (below); `~/plan_verified_stands` is still `selectStands` and the manager's `base_check` does not
  call it, and it has not run against MoveIt. A refused stand is avoided as a point (radius 5 cm by default), not as a region, so a
  verifier that refuses a whole neighbourhood needs several rounds.
- **cuRobo verifier: not built.** The bridge (`curobo_bridge_msgs`) has `SolveIk` and `CheckMotion` (state validity)
  but no batched IK, no way to fix the lift height of the 7-joint group, and a world loaded separately (`SetWorld`).
  A cuRobo `StandVerifier` (or `IkPort` + `CollisionPort` + `ScenePort` over the bridge) therefore needs bridge changes
  and measurements on the GPU image, which cannot be checked here without the GPU job. Left open.

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

#### Where the verified cost goes, and what bounds it

No profile was taken against MoveIt (that needs the live demo); the cost model below is read from the code and from the
measured counts above (1278 pairs, 701 batches, about 5 minutes, so roughly 0.4 s a batch of a few poses). Per
(stand, target) pair the cost is one IK-plus-collision probe per lift height tried (up to `max_lift_probes`), and `selectStands`
repeats the pass once per round, one new stand per round when whole stands are refused. The RM4D recomputation per round
(CPU, no IK) is cheap by comparison. What the code now does about it:

- **Verdict cache across rounds (already there, now tested).** `selectStands` keeps a verdict per (candidate stand, target) pair,
  so a later round sends only pairs it has not seen; `SelectStandsCost.EveryPairIsVerifiedAtMostOnceAcrossRounds` fails if a pair
  is ever sent twice. A pair confirmed in one round is never re-sent. `planStandsVerified` needs no cache: a stand that
  refused or only partly confirmed its targets goes on the avoid list, so the same stand is not planned again.
- **Rounds that learn nothing stop.** A round that sends no new pair ends `selectStands`; a round that plans nothing and
  refuses nothing ends `planStandsVerified`. The RM4D reachability query of a stand is no longer computed when all its pairs
  are already cached.
- **`VerifyOptions::stop_at_first_ok`** (node parameter `stop_at_first_ok`, default true on `reachability_verified_node`,
  false in the core): a target's lift heights are probed lowest first and probing stops at the first height that solves, so a
  confirmed pair costs its position in the probe list instead of all of them. The confirmed set and each target's best
  height (the lowest ok one) are the same as with full probing; refused targets still cost every height. The price: the
  margin of a confirmed target becomes 1 / (heights tried), a lowest-first ordinal rather than the share of ok heights, and
  with `jitter_m` > 0 the offset probes run at that first ok height only. The ranking among stands therefore can differ
  in `selectStands`, which weighs margins; the gtests below compare covered and rejected targets, not stand identity there.
- **`VerifyOptions::max_verifier_calls`** (node parameter, 0 = unlimited): a hard bound on batches. When reached the run stops
  verifying, reports `converged = false` (`budget_hit` in the summary), and in `planStandsVerified` the targets of
  stands not yet verified go to `not_planned`, never reported as covered.
- `verifier_poses` next to `verifier_calls` in `StandVerification`, `VerificationSummary` and `VerifiedStandPlan`: poses sent are
  the IK cost, batches only the call overhead.

Measured with counting fake verifiers in `stands_verified_test` (one run; the fake refuses a neighbourhood of the preferred
stand and every lift height under 0.2 m; a call-count measure, not wall time, and the fake costs nothing per pose):
`planStandsVerified` 72 poses in 12 batches without `stop_at_first_ok` against 60 poses in 10 batches with it (-17% poses), same stands,
same targets, same refusals, same lift-band heights; `selectStands` (several rounds because a stand is refused) 720 poses
against 399 (-45%), same covered and rejected targets. The two gains differ because they depend on how many probe heights a
pair has and how early the first one solves; they are properties of this fake, not predictions for a real wall.
Wall-clock gain on a real backend is expected to follow the pose count but was not measured. Not done: refusing a
whole neighbourhood of a refused stand at once (needs a policy; today a refused stand is avoided as a point, so a
verifier that refuses a region still costs about one round per candidate), batching several stands per verifier call, and
a measured run on the MoveIt demo.

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
stands. Cost bounds: `stop_at_first_ok` (default true here) and `max_verifier_calls` (default 0 = none), see "Where the
verified cost goes".

**Verified or not in `~/plan_stands`.** `PlanStands.srv` has a `bool verified` (default false = RM4D only, exactly as
before) and the node a parameter `verify_by_default` (default false; true verifies requests that leave the flag false). A
verified `~/plan_stands` request runs `planStandsVerified`, so the request's `max_stands`, the node's `stand_strategy` and
`max_rounds` apply (unlike `~/plan_verified_stands`, which runs `selectStands` with its tour). Targets no stand confirms
land in `unreachable` (the message counts them separately), `success` is false for them, and the message reports rounds, verifier
batches and refusals. The flag on a node without a verifier answers `success = false` and says so; it never falls back
silently. Tested through the service with fake verifiers (default path never calls the verifier; flag; flag without verifier;
a target refused everywhere; the parameter). Status: unit-tested through the service with a fake verifier (accepting, and refusing a target everywhere);
the node itself has not been run against MoveIt, the stand cap and `stand_strategy` do not apply to it, the scene is
whatever `move_group` holds (stand-relative obstacle posing is not wired), and the manager does not call it yet.

## Column pose sequences: approach, contact, retract, margins, splits

A column of a wall plan is more than its stroke poses. The manager docks at an approach pose (the dock pose pulled back by
`dock_approach_extra_m`), closes to the dock pose (`dock_standoff_m` from the plan surface), presses, strokes and retracts.
A column whose strokes are in reach but whose approach is not cannot be run, so `buildColumnSequence` makes the sequence
(approach, contact when `press_depth_m > 0`, strokes, retract; `SequenceParams`, the wall normal pointing into the free side as in
`StationRequest`) and `evaluateColumn` checks all of it against the map for one stand and one lift height (the lift is locked while
a column runs; the best of the offered heights is taken: a height where everything is usable, else usable end poses and the most
usable strokes). Per (stand, column) the answer is **whole** (every pose usable), **split** (approach and retract fine, only some
strokes usable: the column would be broken into pieces) or **unreachable** (no stroke usable, or an end pose is not). The pose that
limits it is reported with its role and reason.

**Margins.** The map stores a sample count per cell, not a manipulability index. `scorePose` turns it into a score in [0, 1]:
the smaller of `min(count, count_ref) / count_ref` and the share of the 8 neighbouring cells (one step in x*, y*, z, theta) that are
reachable, so a pose near the edge of the workspace (arm stretched or folded: the reach limit and the usual singular configurations)
scores low. `MarginConfig::min_score` (default 0.15) makes such a pose not usable even though its cell is reachable. This is an
estimate: counts depend on the sampler and the build's sample count, so `count_ref` and `min_score` have to be tuned against the map in
use; nothing here was calibrated against a real arm. When a `StandVerifier` returns a joint solution, `minSingularValue` (6 x dof
Jacobian by finite differences of `ArmModel::evaluate`, any dof, unscaled: metres and radians share one matrix) and
`jointLimitMargin` (share of the range to the nearest limit, continuous joints ignored) can be held against
`min_singular_value` / `min_limit_margin` (off by default) in `confirmColumnWithVerifier`.

**Stands that keep columns whole.** `planColumnStands(map, station, columns, start, options)`: every candidate stand of
`makeCandidateStands` is evaluated per column; a weighted greedy cover over *whole* columns picks the stands (cost 1 +
`margin_penalty` * (1 - mean margin)), redundant stands are dropped, the stands are ordered by `travelTime` (with a base model) or
distance. Only the columns no candidate does whole are cut: into the fewest pieces by greedy cover, each piece with its own approach
and retract built from its own strokes, preferring stands already chosen, and every split is listed with its pieces
(`allow_splits = false` leaves them out, `skipped_to_avoid_split`). One lift height per column visit: a column taller than the lift
plus arm window is cut only as far as one height reaches, the rest is reported as not covered. `stands_if_splitting_allowed`
is the stand count when strokes may be taken one by one (end poses ignored), i.e. the price of keeping columns whole. An optional
verifier re-checks each chosen (stand, column) and the totals follow; a refused column is reported, not moved to another stand.

**How much can we do.** `analyzeColumnCoverage` runs that and `toJson` writes: per stand (position, lift heights, columns whole or
as pieces, points covered, travel) and overall (columns whole / split / unreachable, covered fraction of plan points, stands, the
splitting-allowed stand count), and for every column that is not whole its pieces and the limiting pose
(`approach`, `contact`, `stroke` with its index, `retract`; `out_of_reach`, `low_margin`, `refused`, `singular`, `joint_limit`).
`plan.json` format and example: the header of `apps/column_reach_tool.cpp`:

```
column_reach_tool map.rm4d plan.json [--out report.json] [--base-profile profile.yaml] [--start-x X --start-y Y --start-yaw A]
```

**Manager hook.** `manipulation_manager/core/column_reach_port.hpp` defines `ColumnReachabilityPort` (columns in, a verdict per column
out) without depending on this package. `CoverageFeasibilityAnalyzer::setColumnReachability(port)` makes the analyzer send every
contact column to it before any IK: an unreachable column is excluded whole (reason `approach_out_of_reach`, ...), a split one loses its
unusable strokes (`stroke_low_margin`), and the verdicts come back in `FeasibilityReport::column_reach`. `ReachabilityColumnPort`
(`manipulation_manager/adapters/reachability_column_port.hpp`, library `manipulation_manager_column_reach`) implements the port over a
loaded `Map4D`: per column it runs `buildColumnSequence` (from `dock_standoff_m`, `dock_approach_extra_m`, the press depth,
`dock_at_stroke_start`, the negated `wall_direction`) and `evaluateColumn` for the arm base position and the lift heights it is given.
It sits in the manager package, outside `manipulation_manager_core`, so the core stays free of this package; `reachability` does not
depend on `manipulation_manager`, so the dependency is one-way and there is no cycle.

The node builds it only when `column_reachability_map_file` (an RM4D `.rm4d` file, default empty = off) is set; `world_frame` must be
set too, the arm base position comes from TF (`world_frame` <- `arm_base_frame`). Other parameters: `column_reachability_min_score`
(0.15), `column_reachability_count_ref` (8), `column_reachability_lift_step_m` (0.05, steps between `base_check_lift_min_m` and
`base_check_lift_max_m`), `column_reachability_press_depth_m` (negative = `contact_y_overshoot_m`; 0 for a non-contact tool);
`base_check_arm_mount_z_m` is the arm mount height. With a map loaded the port is handed to `ManipulationManager::setColumnReachability`
(used by `analyzeFeasibleSegments`) and, in a single-stand `execute_coverage` run, every contact column is checked from where the arm
base stands before the run and split / unreachable ones are logged with the limiting pose and reason. That log is advisory: nothing is
skipped or refused. The multi-stand loop does not use it (the stand is not known before the drive; `plan_stands` already decides there).
Poses go to the map as the plan has them (as in the base check), z measured from the floor.

Status: gtests with a synthetic map and fake verifiers (approach unreachable with reachable strokes, retract, low-margin pose,
singular joint solution, split versus whole, totals and JSON, verifier downgrade, analyzer hook). All of it is RM4D-level (no IK,
no collision, no obstacles) unless the verifier is given; the thresholds are starting values; no real arm map, sim run or hardware
run was involved. The adapter has gtests against a hand-made map (whole, approach / retract / low-margin limits, split, lift heights,
unknown stand); the node wiring (parameters, TF stand provider, the log) was written without a build and has not been run.

### How splits are avoided

A column that is "whole" in the map can still have no continuous joint path: IK solved pose by pose may land on another branch
(elbow or wrist flip, a lift jump) for the next pose. Implemented (ROS-free, `core/column_ik_chain.hpp`, `ColumnIkChain`): it
solves a column's pose sequence in order through an injected IK callable, seeding each pose from the previous solution and then
from caller-supplied folded seeds; per step it computes the continuity ratio `max |dq_i| / jump` (jump is the Cartesian distance
of the two poses), compares each joint with the limit of its type (continuous and revolute in rad per m, prismatic lift in m per
m; continuous joints wrapped) and a branch signature (signs of chosen joints, a deadband around zero). A step that stays
discontinuous after reseeding is `PoseFailure::kBranchBreak`. It reports `split_count`, `branch_changes`, `max_continuity_ratio`,
`reseeds` and `lift_travel_m`. Tested only against a fake 2-link planar IK; no real arm, no adapter, no build run yet.

Planned, not implemented: an IK adapter over the stand verifier, the whole-column check before a column is committed, the
escalation (change lift height, then re-stand, then re-order columns, split only last), and the split numbers in the column
analysis, scorecard, verify result and the execution-time count. See `docs/development_plan.md` section 4.

### Split avoidance, measured

`reachability/apps/split_benchmark.cpp` (model and runner in `apps/split_benchmark.hpp`, gtest `split_benchmark_test` with N = 50) is a Monte Carlo
check of the claim above. Per column it draws a wall standoff (0.40 to 0.95 m), a stroke start and length (0.3 to 0.9 m, 3 cm steps, an approach and
a retract pose 0.15 m off the wall) and, for 3 links, a tool angle (+-0.4 rad), and solves it with a planar arm: a vertical lift (it tracks half the pose
height, so lift travel is the same for every method) plus 2 revolute links (0.55, 0.50 m) or 3 (0.45, 0.40, 0.20 m, tool angle held), closed-form IK
returning both elbow branches within the joint limits ("loose" or "tight" limits). Columns with a pose no branch reaches are discarded (86 to 1924
per setting), so every kept column is whole in the reachability sense and only the joint path can break. All methods are scored by the same
`ColumnIkChain` yardstick (break = normalised continuity ratio over 1, limit 6 rad/m, or an elbow sign change). Methods: per-pose IK taking the
first solution (`first_fixed`: elbow + first, always the same order; `first_random`: random order per pose, a solver that lands in either basin),
per-pose lowest cost (`lowest_cost`: sum of |q|), and the chain (previous-solution seeding plus two folded seeds). 1000 kept columns per setting,
fixed seeds (`--seed 20240611`), `g++ -O2`, under a second on the host:

| setting | method | whole | breaks | branch changes | mean / p95 ratio | near singular | near limit | columns worse than chain |
|---|---|---|---|---|---|---|---|---|
| 2link_loose | first_fixed | 996 | 4 | 0 | 2.58 / 3.67 | 1 | 0 | 0 |
| 2link_loose | first_random | 0 | 10582 | 10580 | 112.6 / 150.0 | 1 | 0 | 1000 |
| 2link_loose | lowest_cost | 414 | 586 | 582 | 65.7 / 148.3 | 1 | 0 | 582 |
| 2link_loose | chain | 996 | 4 | 0 | 2.58 / 3.67 | 1 | 0 | - |
| 2link_tight | first_fixed | 648 | 352 | 343 | 18.7 / 118.2 | 1 | 148 | 343 |
| 2link_tight | first_random | 0 | 10529 | 10525 | 96.8 / 122.6 | 1 | 114 | 1000 |
| 2link_tight | lowest_cost | 374 | 626 | 617 | 60.4 / 121.9 | 1 | 0 | 617 |
| 2link_tight | chain | 991 | 9 | 0 | 2.81 / 3.74 | 1 | 0 | - |
| 3link_loose | first_fixed | 962 | 46 | 7 | 3.52 / 5.23 | 1 | 0 | 6 |
| 3link_loose | first_random | 0 | 10536 | 10514 | 114.5 / 147.6 | 1 | 0 | 1000 |
| 3link_loose | lowest_cost | 890 | 136 | 98 | 9.25 / 69.9 | 1 | 0 | 88 |
| 3link_loose | chain | 966 | 40 | 1 | 3.36 / 4.99 | 1 | 0 | - |
| 3link_tight | first_fixed | 518 | 495 | 471 | 20.1 / 111.0 | 1 | 150 | 451 |
| 3link_tight | first_random | 0 | 10165 | 10154 | 94.9 / 117.7 | 1 | 113 | 1000 |
| 3link_tight | lowest_cost | 638 | 406 | 382 | 18.1 / 107.0 | 1 | 110 | 347 |
| 3link_tight | chain | 969 | 34 | 10 | 3.64 / 5.05 | 1 | 0 | - |

What it shows, and what it does not:

- Where the IK order is fixed and the limits are loose (`2link_loose`) the chain gives **no gain**: it is identical to `first_fixed` (996 whole, 4 breaks).
  Always taking the elbow + solution already stays on one branch, so only the few columns that stretch the arm (ratio blow-up near the reach limit) break, and
  the chain cannot fix those: a break there is geometry, not a branch choice. In `3link_loose` the gain is small (966 against 962 whole columns, 40 against 46
  breaks; the chain was never worse than `first_fixed` and better on 6 columns).
- With limits that take one branch away for part of the workspace (`*_tight`) a fixed order is forced onto the other branch mid-column: 52 to 65 % whole
  against 97 to 99 % for the chain. A chain break that remains is mostly a branch change the limits force (10 in `3link_tight`), which no seeding can avoid.
- A random order per pose or the lowest-cost pick breaks almost every column. `first_random` is a worst case, not a realistic solver; `lowest_cost` is a cost
  that has no continuity term, so its poor result says "do not pick per pose by a pose-only cost", not that it is a typical planner. Because the per-pose
  methods here are scored by the chain's own break rule, the comparison is only as honest as that rule.
- `lowest_cost` beat the chain on 1 column in `3link_loose` and 10 in `3link_tight` (per-column `better_than_chain` in the CSV): the chain is not dominant per column.
- Lift travel (0.26 m mean) does not discriminate: the lift policy ignores the branch. Near-singular strokes do not either (1 column in 1000 in every
  method): the singular value of a planar arm does not depend on the elbow side, so it is a property of the geometry, not of the method. Near-limit counts
  are zero for the chain because it was never started near a limit here, not because it avoids limits.

Limits of the model: planar fake arms with a closed-form IK that returns exactly two branches, a lift tied to the height, hand-picked link lengths and
joint limits, one break rule, tool position (and angle for 3 links) only, no collision, no obstacles, no real IK solver basin behaviour, no wrist flips,
no redundancy resolution. It is not a real arm and not hardware; the numbers say that the chain logic behaves as designed on a fake with real branches,
not how many columns of a real wall would split. Run: `split_benchmark [--n 1000] [--seed S] [--csv out.csv]`. Written and run with host `g++` and gtest only;
the colcon build, the new CMake targets and the ROS install were not built here.

### Singularities and joint limits

Implemented (`reachability/core/pose_quality.hpp`, `path_quality_audit.hpp`; ROS-free, gtested with planar 2- and 3-link fakes, not run against a real arm or a sim):

- `minSingularValue`, `conditionNumber` and `manipulability` (Yoshikawa product of singular values) of a Jacobian, and of an `ArmModel` at a joint vector (`armJacobian`, finite differences); `jointLimitMargin` as before.
- `auditPathQuality`: takes stroke joint trajectories plus injected `JacobianFn` / `LimitMarginFn` (`armJacobianFn`, `armLimitMarginFn`, `limitMarginFromBounds` build them) and returns per stroke the min sigma, min limit margin, the worst waypoint index and a reason `near_singular` / `near_limit` / `ok`, with aggregate counts. `PathQualityConfig` defaults are `min_sigma = 0.02` and `min_limit_margin = 0.02` (share of the joint range); 0 switches a check off. The Jacobian is unscaled, so these are starting values, not calibrated. It only reports; it does not change a path.

Planned, not built: calling the audit from the Cartesian planner and a `manipulation_manager` gate (opt-in parameter, default off, report and flag only); profile-level thresholds into the stand verifier and the cuRobo guard (which checks velocity and acceleration only); a damped least-squares / null-space IK decorator; an opt-in monitor that slows and then faults; per-pass reporting and an A/B on one wall.

## Plane-generic stands and base free space

The stand search no longer assumes a vertical wall. `core/surface_frame.hpp` builds the in-plane axes for any normal
(`surfaceFrame`): for a vertical or tilted plane `u = normalize(UnitZ x n)` exactly as before (every wall result is
unchanged, tested bit for bit), for a horizontal plane `u` is +x projected into the plane and `v = n x u`.
`chooseStation`, `makeCandidateStands`, the set-cover enumeration and the column-stand ordering all go through one
grid generator, `stationGrid`.

- **Ceiling and floor.** For a horizontal patch the grid covers the patch area (bounding box of the targets, plus
  `lateral_margin`), the base stands under a ceiling or over a floor patch, `standoff` is 0 and `min_clearance` /
  `max_standoff` do not apply (no plane clearance). `normal` points into the free side (down for a ceiling, up for a
  floor). A zero or non-finite normal is still `invalid_request`.
- **Free space.** `StationRequest::free_space` (`core/free_space.hpp`, default unset = no change) holds walls other
  than the work plane (half-planes in the floor plane) and obstacle polygons (`FreeSpace::box` for axis-aligned
  ones; a finite wall is a thin polygon). `footprintFree(pose, base, margin)` is a hard filter on every candidate
  in `chooseStation`, `makeCandidateStands` and the set-cover candidates: all 24 yaw samples are tested with the full
  footprint inflated by `free_space_margin` (default 0.05 m), a candidate is kept when at least one yaw fits and
  takes the feasible yaw with the most work-plane clearance. At an inside corner the footprint is therefore kept off
  the perpendicular wall; without a base model only the arm-base point is tested.
- **Wall end and outside corner.** The work plane is infinite by default. With `FreeSpace::has_work_span` (a
  `[work_span_lo, work_span_hi]` interval along `u` from `wall_point`) the work-plane clearance applies only to the
  footprint points inside the span, so a stand beyond the end is allowed; put the wall's end in `obstacles` to keep the
  footprint off it.
- **Reason codes.** `Station::code` (`StationCode`: `ok`, `invalid_request`, `no_free_stand`, `no_reachable_stand`) and
  `StandSelectionCode::kNoFreeStand` (`no_free_stand`) say that every candidate was removed by the footprint
  (work-plane clearance or free space), as against "free stands exist but reach nothing". A too-narrow alcove ends in
  `no_free_stand` with all targets in `unreachable`; no stand is invented.
- **Non-planar surfaces: not done, the frame abstraction is the hook.** `SurfaceFrame` (origin, outward normal,
  tangents u and v) is the local frame of the work surface at a point; a plane has one everywhere, a curved patch
  could supply one per candidate point. The free-space filter only needs the other obstacles and the footprint, not
  the surface type. Nothing here handles curvature yet (the grid, the work-plane clearance and the span are planar).
- **Limits.** Obstacles are 2D (heights are not modelled); the stand stays on the free side of the work plane except
  through the work span (the search does not go behind a wall end); the arm and tool are not checked against an
  adjacent wall; the stand search itself has no navigation-feasibility (connectivity) check, so a free stand may not be
  reachable by the base (the [Base placement optimiser](#base-placement-optimiser) section adds one); yaw samples are 24 fixed steps. Tested with hand-made maps (`test/core/plane_stands_test.cpp`: 13 tests, ceiling,
  floor, inside corner, wall end, narrow alcove, determinism); not run in any simulation or on hardware.

## Base placement optimiser

`planBasePlacement` (`core/base_placement.hpp`, ROS-free) decides where the base stands from the reachability of the
whole column sequence, not from single targets. It composes the existing pieces; nothing is reimplemented.

**Decides from.** The surface frame (`SurfaceFrame`: a plane, or any frame with an origin and an outward normal), the
columns as stroke lists (each turned into approach, contact, strokes and retract by `buildColumnSequence` with the
manager's `dock_standoff_m`, `dock_approach_extra_m`, press depth and retract conventions, checked against the RM4D map per
stand and lift height), the base footprint, a `FreeSpace` (obstacles and other walls), the navigation start pose and the
options (`allow_splits`, `MarginConfig`, optional verifier, `max_stands`, yaw window). One map is taken (several maps are not
combined).

**Steps.**
1. Candidate stands: `makeCandidateStands` (grid on the free side, footprint and work-plane clearance, `FreeSpace` over all yaw samples).
2. Navigation filter (hard, before selection): a `NavFeasibility` port; the shipped `GridNavFeasibility` floods a grid
   over the free space plus the work wall, inflated by the footprint's circumscribed radius plus a margin, from the start
   pose, and every candidate must be connected (the stand and the start may sit up to circumscribed minus inscribed radius
   plus one cell inside the inflation, since the real footprint was already tested). A Nav2 costmap can replace the grid
   by implementing the port (`BasePlacementOptions::nav`). Without a start pose nothing is filtered by navigation.
3. Yaw window: with a base that cannot strafe, the yaw must stay within `diff_yaw_window_rad` (default 0.7) of facing the
   surface and keep the footprint clear; a base that can strafe is unrestricted (`yaw_window_rad` overrides). The stand takes
   the feasible yaw closest to facing.
4. `planColumnStands` over the surviving candidates (rejected ones are avoided).
5. Verifier loop (the `planStandsVerified` semantics): a (stand, column) the verifier cut puts that stand on the avoid list and
   the columns are planned again, at most `verifier_rounds`; the best round (most strokes, then whole columns, then fewer stands)
   is kept. A stand avoided for one column is avoided for all (coarse).

**Escalation when a column is not whole at its best stand**, in this order: another lift height of the same column (every
candidate is evaluated at all heights); another candidate stand that does it whole (whole-column set cover); column re-order
is the caller's (the optimiser reports and does not re-order); a split only as the last resort, reported. `allow_splits =
false` leaves such a column out (`skipped_to_avoid_split`). `BpoColumn::escalation` names the cheapest step at which a whole
stand exists: `none` (whole at the stand nearest the column at the nominal lift, the one that puts the arm base level with the
column's middle), `lift_height`, `other_stand`, or `split` / `skipped` / `unreachable`.

**Reason codes** (`BpoReason`, stable strings): `ok`, `split_no_whole_stand`, `skipped_to_avoid_split`, `out_of_reach`,
`low_margin`, `refused_by_verifier`, `near_singular`, `near_joint_limit`, `unreachable_by_nav` and `yaw_window` (a candidate
the filters removed would have done better), `no_free_stand`, `max_stands`. Each non-whole column also carries the limiting
pose (role, stroke, failure, score). No column is dropped: whole + split + unreachable equals the columns given.

**Outputs.** Stands in visiting order with the columns each covers, yaw, standoff, travel (time with a base model, and
metres) and the footprint clearance; `covered_fraction`; `reachable_fraction` (strokes usable at some navigable stand with
usable end poses: what the arm and the stands allow at best); `stand_count`; `exact_min_stands` (iterative-deepening
exact cover of the whole-coverable columns over the navigable candidates, up to `exact_max_columns` columns and a node
budget, a lower bound on the stands for whole columns, taken before verification); `split_count`; travel; rejected candidates
with their reasons.

**Tested** (`test/core/base_placement_test.cpp`, 19 tests, hand-made maps, no arm, no ROS): the plain wall against
`planColumnStands`; inside corner (coverage non-decreasing with the distance from the second wall, no stand inside the corner);
outside corner / wall end; window border with a bench; a pillar; a narrow alcove (`no_free_stand`, nothing invented); a corridor
(a diff base cannot turn in it: `yaw_window`; omni can); partition (`unreachable_by_nav`); a costmap port; a refusing
verifier; stand cap; allowed and forbidden splits; 40 random inside corners (angle 60 to 120 degrees, finite or infinite
walls, wall length, patch offset, ring reach and footprint size from a synthetic ring map, fixed seeds) and 30 random walls
with obstacles. Asserted on all: covered >= 0.99 of reachable, stands within 1 of the exact minimum, no split when every
column has a whole stand, stands footprint-clear with the margin, every unreachable column coded, determinism. The run prints
a metrics table (coverage, stands, exact minimum, splits, travel, clearance).

**Honest limits.** The arm links and the tool against an adjacent wall are not checked; obstacles are 2D; there is no Nav2
costmap yet (the grid is a stand-in: circle inflation is conservative, so a gap just wider than the footprint can be called
blocked); the yaw is not an input to reachability (the map is taken as yaw-free, like the rest of the core); the verifier
loop avoids a refusing stand outright; column re-order is not done here; RM4D is optimistic (no IK, no collision), so
whole means the map says so. The harness uses hand-made maps, not a built arm, and nothing was run in a simulator or on hardware.

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

