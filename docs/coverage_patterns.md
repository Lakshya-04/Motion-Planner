# Coverage planners and wall patterns

## Two layers

| Layer | What it decides | Implementations |
|---|---|---|
| **Grid planner** (`CoveragePlannerBase`) | the passes inside one free region of a coverage grid | `boustrophedon` (strip sweeps with connectors, components), `sweepline` |
| **Wall pattern** (`planWallPattern`) | how a whole wall is covered: which region each plan covers, in what order, with what tool orientation | `vertical`, `horizontal`, `zigzag` |

Any pattern runs on any grid planner (`planner: sweepline | boustrophedon`). The result of both
layers is the same contract: waypoints plus `MotionPlanSegment`s (strip = cartesian + contact,
connector = free-space + retraction), then lifted onto the wall with standoff and tool
orientation.

## Grid planners

**boustrophedon** (existing): strip sweeps over the free cells of each connected component, with
optional connectors between strips and a boundary pass.

**sweepline** (new): slices the free space at the stepover spacing, finds the free spans on each
slice (obstacles, concave ones included, split a slice into several spans) and chains spans of
consecutive slices into serpentine passes. A span joins a chain when its nearer endpoint can be
reached from the chain's last point by a connector that crosses no obstacle and is at most two
stepovers long; otherwise it starts a new chain. An obstacle therefore yields separate serpentine
blocks around it, each internally continuous. Details: first slice inset by half a swath;
`ensure_coverage_within_boundary` adds a final pass flush with the far edge; the plan is built in
a frame mirrored toward `start_corner`, so it begins at that corner; chains are ordered nearest
first and reversed when their far end is closer. Design after the sweepline planner of an earlier
production stack (polygon based there, on a grid here); written from the algorithm, not copied.

Not done yet: a true cellular decomposition (events, cells, exact cell ordering by a routing
solver) for large cluttered walls, and spiral. Both fit behind the same interface.

## Wall patterns

Patterns come from an earlier coverage workflow (vertical strokes, panelled horizontal rows,
panelled raster with swing-back); re-implemented here on top of the grid planners.

| Pattern | Behavior |
|---|---|
| `vertical` | vertical strokes over the whole wall. Optional **bottom pass** (`enable_bottom_pass`, `bottom_band_height_m`): horizontal rows over the strip the vertical strokes cannot reach, done first, with the tool turned like the panels |
| `horizontal` | the wall is split into **panels**, each reachable from one base stand (`panel_width_m`, `panel_overlap_m`). Each panel is covered with horizontal rows, **serpentine**: a row ends where the next begins. With `panel_alternate`, each panel starts at whichever end is nearer the previous panel's last point, so the arm is already at the far end when the base steps along the wall |
| `zigzag` | the same panels, but **every row runs the same way** (left to right when entering from the left) and the tool swings back diagonally to the start of the next row. The swing back is a free-space connector with retraction |

Panel options: `panel_swath_m` (row swath, default the tool's), `panel_end_margin_m` (the base
cannot stand nearer the wall ends), `floor_clearance_m` (lowest row's edge above the wall bottom),
`panel_tool_turn_deg` (added to the tool yaw for horizontal rows, e.g. a mop turned 90 degrees),
`start_left`. Every panel is also a **base stand**: `PatternPlan::panels` carries each panel's
`stand_u`, which is the input base stationing and reachability checks
([reachability.md](reachability.md)) need. Segments are tagged `panel_<k>` and `part:<name>`.

## From a raw scan

`planCoverageFromScan(scan, obstacles, tool, options)`: fit the wall plane (RANSAC + PCA, wall
frame u along the wall, v up), rasterize observed cells and obstacle footprints, plan the chosen
pattern, lift every part with its own tool profile and concatenate. See
[scene_model.md](scene_model.md).

## Floors, ceilings and columns: planar patches

`SurfacePatch` (`core/surface_patch.hpp`) is the planar `SurfaceModel`: a plane origin, a right-handed
(u, v, normal) frame, a rectangular (u, v) extent and a kind (`wall`, `floor`, `ceiling`, classified from
the normal). The normal points to the side the tool works from: floor +z, ceiling -z, a column face
horizontally outward. `makeFloorPatch`, `makeCeilingPatch` and `makeSurfacePatch` build one; for a
wall `u = up x normal` (v up), for a horizontal plane u = world x. The API:

- `project(world) -> (u, v, n)`, `point(u, v, n)`, `contains(u, v)`;
- `liftPatchPath(patch, uv_waypoints, tool_task)`: 2D waypoints to tool poses, tool z along -normal, standoff
  along +normal, edge tilt and yaw from the tool profile (it is `liftCoveragePathPose` on the patch plane);
  `liftPatchPose` for one point without tilt;
- `patchObstacles(patch, scene)`: obstacle footprints (any orientation, same filter as a wall);
- `planPatchCoverage(patch, obstacles, tool, options)`: rasterize, plan (boustrophedon or sweepline, any wall
  pattern), lift, expand passes. This is the tail of `planCoverageFromScan`, now `planCoverageOnWall`, so a wall
  gives the same poses as before and a floor or ceiling uses the same code. A scanned floor also works through
  `planCoverageFromScan` with `plane.normal_hint = +z`.

A room is a list of patches, each planned on its own. Not done: reachability and stand selection for a horizontal
surface (`chooseStation` still rejects a horizontal plane), reach limits for ceilings, ordering patches, and
`loadWallScene` flatness is still a threshold on the scene file (it does not look at orientation).
Tests: `test/core/surface_patch_test.cpp` (floor and ceiling with obstacles, orientation, verification raster).

## Non-planar surfaces (long term)

The grid planners and patterns work in a surface's (u, v) parameter space, and only the final
lift depends on the surface geometry. Non-planar coverage therefore plugs in through the
surface abstraction in [architecture.md](architecture.md) (`SurfaceModel`: `project`, `lift`,
`contains`, obstacles) without changing the planners:

- **Mildly curved or piecewise-planar surfaces:** one planar patch per face or per fitted
  region; panels follow patch boundaries.
- **Parametric or mesh surfaces:** a (u, v) parameterization (unfolded mesh, parametric patch)
  with spacing measured on the surface, so strip spacing stays uniform where the parameterization
  stretches; `lift` returns position, outward normal and a tangent frame per point.
- **What changes:** the coverage grid needs a metric (cell size in surface distance), obstacle
  footprints project through the surface, and tool tilt follows the local normal.
- **What does not:** the `CoveragePlannerBase` contract, pattern logic and segment semantics.

Learned or non-grid planners (including RL) are further implementations of the same port.

## Verifying what a run covered

`CoverageVerifier` (`coverage_planner/core/coverage_verification.hpp`, ROS-free) rasterises a wall's (u, v)
rectangle and marks the cells each executed stroke reached: a band of `swath_m` around the stroke's path, counted
once per stroke per cell, only where the tool was working (`engaged`: contact held or valve open). `report()` gives
the covered fraction of the target cells (a mask excludes obstacles), the mean number of strokes per covered cell
(overlap), the number of worked cells outside the target (overspray) and the indices of the missed cells.

It is a geometric model: no deposition thickness, spray falloff or timing. It is not wired to execution yet: nothing
converts the executed tool poses from a run into wall (u, v) strokes, which is the next step toward a scorecard.

### Run scorecard

`scoreRun` (`coverage_planner/core/run_scorecard.hpp`, ROS-free) turns a recorded run into a pass/fail card on one
wall: coverage (through `CoverageVerifier`), contact force RMS error against the target, force peak, the fraction
of working samples inside the force band, and the duration. A contact tool counts as working while it presses at
least `engaged_force_n`; a non-contact tool while `tool_active` is set. A new stroke starts after every sample where the
tool was not working. Obstacles are excluded from the target. The limits are the caller's numbers
(`ScorecardLimits`); each one that fails adds a line to `failures`. The coverage is geometric. It counts obstacle contacts (below), not
other collisions (the recorder must). It scores recorded samples; recording them from a simulator run is not wired yet.

`ros2 run coverage_planner score_run --samples run.csv --wall wall.json [--swath 0.2] [--target-force 10] [--band 1]
[--no-contact] [--min-coverage 0.95] [--max-rms N] [--max-peak N] [--min-in-band F]` scores a recording and prints the
card as JSON (exit 0 passed, 1 failed a limit, 2 bad input). The CSV has the header `t,x,y,z,force,active` (seconds,
metres, newtons pressing on the wall, 0/1 valve); the wall JSON is `{"origin": [x,y,z], "u": [..], "v": [..],
"u_range": [min,max], "v_range": [min,max]}` in the same frame as the positions, with `u` and `v` orthonormal and the
normal `u x v`. It takes any recording, a simulator run or a hardware log.

Obstacles on the wall: the wall JSON may carry `"obstacles": [{"id": 0, "u": [min,max], "v": [min,max], "height_m": 0.04}]`
(u, v in the wall's coordinates, `height_m` how far the obstacle stands off the wall, against `u x v`; 0 for a window
hole). `score_run` then (1) leaves the obstacle out of the coverage target, grown by `--keepout-margin M` (use the
planner's clearance, so the target is the area the planner may reach) and (2) counts obstacle contacts: an episode is a
run of consecutive samples, any force, whose tool point is inside an obstacle's footprint (grown by `--obstacle-margin`)
and not above its top (`height_m` + `--obstacle-tol`, default 3 mm). The card has `obstacle.contacts` and
`obstacle.contact_samples`; `--max-obstacle-contacts N` fails the run above N. Only the tool point is checked, not the
arm's links, so an elbow hitting the obstacle is not seen. `record_run.py --scene wall_scene.json` writes the list
from a wall scene (`docs/sim_mujoco.md`, scenario set).

