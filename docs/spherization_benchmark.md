# Sphere sets vs planning: success, time, path length (UR10e)

Follow-up to [spherization_comparison.md](spherization_comparison.md) (which measured geometry only). Here
the same sphere sets drive real planners: cuRobo v2 and VAMP. Partial result: the cuRobo side was only
run in full for set B, see "Not run" below.

## Setup

- Robot: UR10e arm, 6 joints, collision meshes of `ur_description`, no tool, no base lift. Planning frame
  is the base frame.
- Sets: **A** FOAM; **B** cuRobo v2 fit (370 spheres, density 1.0, seed 0); **Bb2** = B with
  `collision_sphere_buffer` 0.02 m; **C** reference set (previous production model, FOAM-like, passed by
  path, not stored in the repo). Reference set D covers 4 links only and was not used for planning.
- Scenes (identical for every set): `empty`; `boxes` = 5 scenes with 4 random yawed boxes each (0.10 to 0.35
  m, 0.35 to 0.9 m from the base); `slab` = 5 scenes with one 0.06 x 1.2 x 0.9 m wall 0.45 to 0.6 m from
  the base at a random azimuth. Seed 0.
- Problems: 100 per family (empty 100x1 scene; boxes and slab 20 per scene x 5). Start and goal are uniform
  random joints in [-pi, pi], accepted only if free under the **reference**: sets A and Bb2 (each with its
  own self-collision policy) and the true link meshes against the boxes. So every pair is valid for every
  planner, and endpoint rejections are not mixed into the success rate (they are counted separately).
- Backends. cuRobo: `CuroboPlanner.plan_joint` (`plan_cspace`), RTX 3050 4 GB, one planner per process, all
  other settings default. VAMP: RRT-Connect then shortcut/B-spline simplify, same call as
  `vamp_planner` (max 100000 iterations, range 2.0), upstream `27cb9b6` in `motion-planner:full`, headers
  from Cricket (`docker/tools/vamp_bench.cpp`).
- Self-collision policy differs per set and is part of the model, not a fixed factor: A and C ignore
  links up to 2 joints apart (`rigid_pairs_srdf.py --hops 2`; FOAM's big adjacent spheres overlap
  otherwise), cuRobo's own B matrix ignores only neighbours (1 hop), so it also rejects some
  2-hop-apart overlaps. cuRobo FOAM config: `generate_curobo_robot.py --spheres-urdf ... --ignore-hops 2`.
  VAMP B header: B spheres converted to a spherized URDF (all 370, no thinning) + 1-hop SRDF.
- Timing: wall clock (`perf_counter` / `steady_clock`) around the planner call only; the first call after
  warmup is reported separately and not in the medians. The GPU and CPU were shared with other jobs at
  times (see caveats).
- Oracle (independent of the planners): every returned path is sampled so no joint moves more than 0.02 rad
  between states, then checked against (a) the true meshes vs the boxes (about one surface sample per
  1.5 cm^2 plus vertices), (b) each sphere set's own model, world only. Mesh-mesh self-collision was NOT
  checked (no fcl).
- "mesh-hit" = returned paths that touch a box at mesh level (false "free"). "hit X" = paths that
  intersect a box in X's spheres (world only; self-collision excluded because policies differ).
  "fail, feasible" = failures for which another config's path or the straight line is mesh-clean
  (a feasible path exists); "of which blocked by set" = every such witness collides under the set's own
  model including its self-collision policy.

Reproduce (needs `motion-planner:gpu`, `motion-planner:full`, the FOAM and Cricket tool images; host python
with numpy scipy trimesh pyyaml rtree embreex):

```
# prepare: bench_spheres.py prepare --work W --urdf robot.urdf --set A=spherized.urdf#2 \
#          --set Bb2=curobo_B.yml@0.02#1 --reference A Bb2      (NAME=PATH[@buffer][#hops])
# cuRobo configs: generate_curobo_robot.py (B, and A with --spheres-urdf --ignore-hops 2);
#          bench_spheres.py buffer-cfg --cfg curobo_B.yml --buffer 0.02 --ignore-hops 2 --urdf robot.urdf
# run:     bench_spheres.py run-curobo --problems problems.json --cfg X.yml --name X --out curobo_X.json   (gpu image)
#          bench_spheres.py run-vamp --work W --header X.hh --struct X --name X --out vamp_X.jsonl
# report:  bench_spheres.py report --work W --urdf robot.urdf --set A=... --set B=...#1 --results ...
```

## Results

### Full: 100 problems per scene family (300 per row group)

| backend | set | scene | success | median ms | p95 ms | joint path rad (median) | mesh-hit paths |
|---|---|---|---|---|---|---|---|
| VAMP | A (FOAM) | empty | 100/100 | 0.1 | 0.3 | 5.85 | 0 |
| | | boxes | 100/100 | 0.3 | 1.3 | 6.34 | 0 |
| | | slab | 100/100 | 0.2 | 0.5 | 6.10 | 0 |
| VAMP | B (370) | empty | 100/100 | 0.2 | 0.9 | 5.84 | 0 |
| | | boxes | 100/100 | 1.3 | 8.6 | 6.13 | 7 |
| | | slab | 100/100 | 0.8 | 6.3 | 6.10 | 4 |
| VAMP | C (reference) | empty | 99/100 | 0.1 | 0.3 | 5.83 | 0 |
| | | boxes | 100/100 | 0.2 | 0.8 | 6.30 | 0 |
| | | slab | 100/100 | 0.1 | 0.5 | 6.02 | 0 |
| cuRobo | B (own fit) | empty | 66/100 | 1348 | 2878 | 5.34 | 0 |
| | | boxes | 55/100 | 3777 | 5280 | 5.16 | 0 |
| | | slab | 61/100 | 4135 | 9263 | 5.64 | 0 |

VAMP C's single empty failure: its endpoint was invalid for that header (1 case).

### Subset: first 10 problems per scene (110 per set, all sets compared on identical problems)

| backend | set | scene | success | median ms | p95 ms | path rad | mesh-hit | hit A | hit B | hit Bb2 | endpoints invalid |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cuRobo | A (FOAM) | empty (10) | 10/10 | 227 | 357 | 6.45 | 0 | 0 | 0 | 0 | 0 |
| | | boxes (50) | 49/50 | 273 | 390 | 6.10 | 0 | 0 | 0 | 7 | 0 |
| | | slab (50) | 48/50 | 256 | 470 | 5.94 | 0 | 0 | 0 | 9 | 0 |
| cuRobo | B | empty | 6/10 | 928 | 1136 | 6.12 | 0 | 0 | 0 | 0 | 0 |
| | | boxes | 26/50 | 3797 | 5126 | 5.11 | 0 | 7 | 0 | 7 | 0 |
| | | slab | 31/50 | 4135 | 7797 | 4.88 | 0 | 6 | 0 | 8 | 0 |
| cuRobo | Bb2 | empty | 7/10 | 385 | 759 | 5.90 | 0 | 0 | 0 | 0 | 3 |
| | | boxes | 28/50 | 1707 | 1801 | 6.21 | 0 | 2 | 0 | 0 | 21 |
| | | slab | 33/50 | 1394 | 2455 | 6.07 | 0 | 1 | 0 | 0 | 14 |
| VAMP | A | all | 110/110 | 0.2 to 0.3 | 0.8 to 1.7 | 5.9 to 6.5 | 0 | 0 to 1 | 0 | 0 to 6 | 0 |
| VAMP | B | boxes/slab | 100/100 | 0.8 to 1.4 | 7 to 8 | 5.9 to 6.1 | 3 / 1 | 18 / 7 | 0 / 1 | 16 / 9 | 0 |

(Full VAMP per-scene rows for the subset are in the reproducible report output; the full table above is the
one to read for VAMP.) First call after warmup: cuRobo 0.4 s (A), 0.8 s (B), 0.2 s (Bb2), construction
plus warmup 78 to 303 s; VAMP 3 to 5 ms.

### Collision-check micro timings (median over scenes)

| backend | set | spheres | per-state check | per-motion check |
|---|---|---|---|---|
| VAMP | A | 21 | 0.98 us | 6.7 us (32 steps) |
| VAMP | C | 21 | 0.64 us | 5.3 us |
| VAMP | B | 370 | 20.2 us | 242 us |
| cuRobo | A | 21 | 3.1 us/state in a 1000-state batch (488 us single state) | n/a |
| cuRobo | B | 370 | 12.6 us/state batched (6.2 ms single) | n/a |
| cuRobo | Bb2 | 370 | 3.2 us/state batched (the cuRobo numbers move 3x between runs; GPU was shared) | n/a |

cuRobo per-state numbers include the kernel launch and the host sync of `check_samples_feasibility`;
they say little about the in-solver cost. VAMP numbers are its SIMD kernel on this CPU.

## What the data says

- **VAMP is insensitive to the set in success (899 of 900 runs, the one miss being an invalid endpoint)
  but very sensitive in time.** 370 spheres cost about 20x per state and about 5x in median plan time
  against 21 spheres (boxes: 1.3 ms vs 0.3 ms median, p95 8.6 vs 1.3 ms). Still milliseconds.
- **B in VAMP gives false "free" paths.** 7/100 boxes and 4/100 slab paths touch a box at mesh level
  (sets A and C: 0 of 600 paths). That is the 1 to 3 cm gap of the fit, found by the oracle, not assumed. A
  buffered B was not run in VAMP.
- **Conservative sets are safe but eat space**: paths planned with A or C intersect B's spheres only
  rarely, the reverse is much more common (VAMP-B paths intersect A's spheres in 36/100 boxes and 19/100
  slab paths, and C's in 31 and 18), so a B path is not valid for a robot modelled by A or C.
  Paths from A/C also hit Bb2 (6 to 14 of 100): the buffered B is not a subset of FOAM's envelope.
- **cuRobo with FOAM (A) was the most reliable and fastest of the cuRobo cases** on the 110-problem subset:
  96 to 100% success, about 250 ms median, versus 52 to 62% and 0.9 to 4 s for B. Do not read this as
  "FOAM is better geometry": B's number is dominated by its 1-hop self-collision matrix, which makes the
  feasible set smaller (on the empty scene, where there are no obstacles, B still fails 34% of 100 random
  pairs, A's paths collide with B's self model in 4/10 empty cases), plus cuRobo's optimizer failing on
  large random joint moves. Why exactly B fails was not isolated (no run with B's matrix relaxed to
  2 hops was made). The conclusion that holds: **set plus self-collision matrix decide cuRobo's success
  here, not just sphere size**; FOAM with `--ignore-hops 2` plans through a model that is a cruder envelope but
  has more freedom.
- cuRobo's `collision_sphere_buffer` is applied to self-collision as well: Bb2 with cuRobo's default matrix
  rejected 100% of the starts (first attempt, not shown); with the hops-2 ignore added, 38 of 110
  endpoints are still invalid for cuRobo although they were free under the reference, so Bb2's 56 to
  70% success is not comparable to B's (different, easier effective problem set: of the problems with
  valid endpoints 68 of 72 succeeded). Treat Bb2 as "needs a re-run with a matched reference".
- Path lengths are similar (about 5 to 6.5 rad) because the pairs are random far-apart joint targets;
  cuRobo's are slightly shorter where it succeeds (5.1 vs 6.3 in boxes) but they are different problem
  subsets (successes only), so this is not a paired comparison. VAMP paths are 2 to 4 waypoint polylines,
  cuRobo's are time-parametrised trajectories.
- First call: negligible for VAMP (ms), 0.2 to 0.8 s for cuRobo after a 78 to 303 s construct and warmup.

## Caveats and what was not covered

- **Not run:** cuRobo with A or Bb2 on all 100 problems per family (only the first 10 per scene, 110
  total, because one full cuRobo config takes about 50 min with 40% failures); cuRobo with C (A and C are
  near-identical); VAMP with a buffered B; VAMP with B thinned (`--max-per-link`); reference set D;
  any pose-goal planning (only joint-to-joint); a tool or payload; point-cloud / ESDF worlds; more than
  one RNG seed (VAMP uses a fixed Halton sequence; cuRobo seeding is not fixed, so reruns differ a bit);
  mesh-level self-collision of returned paths.
- Subset medians over 10 to 50 problems have wide error bars; success differences below about 15 points
  are not significant.
- The VAMP B model disagrees with the numpy model of the same sphere set on 14% of random probe states
  (2831/3300); A and C agree on 3300/3300. Cause not found (Cricket handling of the 370-sphere model
  or its SRDF); B's VAMP rows are therefore less certain than A's and C's.
- Another GPU/CPU job ran during parts of the runs (a simulation and build container), so absolute
  times, cuRobo's most, may be pessimistic. VAMP was timed while a cuRobo process ran on another core.
- "Feasible" witnesses come from the other configs' paths and the straight line only; a failure with no
  witness is not proof the problem is infeasible.
- Boxes only rotate about z; the mesh oracle uses surface samples and can miss a box smaller than the
  sample spacing (about 1.2 cm), none are.
