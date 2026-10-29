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
