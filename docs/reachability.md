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

## Limits and next steps

- **Sampling density.** Uniform joint sampling leaves cells empty at fine resolution: 3 M samples
  set 11% of a 5 cm map. Use at least ten samples per reachable cell, or a coarser map; the
  saturated-cell count in the build stats shows where more samples add nothing.
- **Self-collision** is not checked by the built-in chain; plug a collision-aware `ArmModel` in
  before trusting a map for cluttered arms.
- **Joint range assumption.** The reduction needs the first joint to rotate a full turn and the
  last joint to rotate fully (UR family: yes). Check a new arm against brute force.
- **Done since the first version:** the ROS shell (`reachability_node`, see below: `~/is_reachable`,
  `~/get_base_candidates`, `~/plan_lift_bands`, `~/plan_stands`) and selection over several stands
  (`planStands`, `orderStands`).
- **Not done:** RViz slices of the map, a validation harness against brute-force 6D sampling,
  orientation-constrained wall queries, a yaw preference for contact work, and any caller:
  the manipulation manager docks at a fixed offset and never asks this module for a base pose.

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

Parameters: `map_file` (required), `robot_fingerprint` (rejects a map built for another robot), `base_profile` (optional robot profile YAML with a `base:` section; `~/plan_stands` then keeps the footprint clear of the wall and reports driving time for the drive type, otherwise the footprint is ignored; a set `base_profile` that cannot be loaded leaves the node not ready, like a missing `map_file`).

`~/plan_stands` rejects `grid_step` below 0.02 m, `lift_step` below 0.01 m, `lift_max` (when set) below `lift_min`, and searches whose work (stations x lift heights x targets) exceeds 5e7 per stand, so one request cannot keep the node's callbacks busy for minutes. All positions (targets, wall point, returned stands) are in the same world frame the caller chose; the service does not transform.
The map does not know which side of a wall is free, so base candidates come back on both sides of
the target plane; the caller (stationing logic) picks the free side.

```
ros2 launch reachability reachability.launch.py map_file:=/path/ur10e.rm4d
```

## Checking the base stance from the manager

`manipulation_manager` can ask `~/plan_stands` whether the base, where it stands, reaches the whole coverage plan
(`base_check:=off|warn|enforce`, default `off`). It sends the plan's poses in the base frame with the wall on the
+y side (the assumption the dock pose already makes) and accepts the stance only when exactly one stand covers
every target and that stand is within `base_check_tol_xy_m` (0.15) and `base_check_tol_yaw_rad` (0.1) of the
origin. Otherwise it reports where the base would have to be, or that several stands are needed. `warn` logs and
runs the plan; `enforce` does not run it, and also refuses when the service is unreachable. Set
`base_check_lift_min_m`, `base_check_lift_max_m` and `base_check_arm_mount_z_m` to match the robot (0 selects the
service defaults). This only verifies; it never moves the base (that waits for base motion in the simulators).

