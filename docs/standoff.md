---
type: Reference
title: Standoff advisor
description: How far from the wall should the base dock: reachable fraction and pose quality versus distance on an RM4D map, the robust recommended distance, and how to feed it into the manager and the sims.
tags: [reachability, standoff, base-placement, wall]
status: experimental
---

# Standoff advisor: how far from the wall should the base dock?

Package: [`src/common/standoff_advisor`](../src/common/standoff_advisor) (ROS-free C++ core, the `standoff_sweep`
command line tool, a plot script). It sits on top of the [reachability](reachability.md) core and changes nothing in it.

`reachability::chooseStation` already searches stand positions on an RM4D map, but it answers "which stand", not
"how does reach change with the distance, and how far may the base be off before the work is lost". `sweepStandoff`
answers that for an assembled robot (base + lift + arm).

## What is computed

For every distance `d` from `standoff_min` to `standoff_max` (default 0.30 .. 1.60 m, step 0.02):

1. The arm base is put `d` metres from the wall plane, at the best position along the wall (a search over the targets'
   extent, `lateral_step` 0.1 m; the position that reaches most targets, ties to the better quality, then to the
   middle). Base yaw is free (the arm's first joint absorbs it, as in the map).
2. **Reachable fraction**: share of the target tool poses (a wall patch on a grid, tool z axis along minus the wall
   normal) that the map reaches at some lift height in `lift_min .. lift_max` (arm base height = lift +
   `arm_mount_offset_z`).
3. **Quality**: `scorePose` of `pose_quality.hpp` (map sample count, and the share of reachable neighbour cells: low at the
   workspace edge) with the best lift height for each target; mean and minimum over the reachable targets.
4. **Lift range**: `planLiftBands` for the reachable targets: number of bands and the lowest and highest band height.
5. **Footprint**: with a `BaseModel` the best yaw (`bestYaw`) gives the footprint-to-wall clearance and the
   `base_footprint` origin's distance to the wall (the arm mount may be off the origin); the distance is excluded from
   the plateau when the clearance is under `min_clearance` (0.30 m). Without a base model the arm base point is tested.
6. **Robust fraction**: the smallest reachable fraction within `+- nav_tolerance_m` of this distance.

**Recommended distance.** Not the single best distance. A *plateau* is a contiguous run of distances that fit the
footprint, have a reachable fraction within `fraction_tolerance` (0.05) of the best one that fits, and a mean quality
of at least `quality_min` (0.50). The widest plateau wins (ties: better mean quality). It is shrunk by
`nav_tolerance_m` (default 0.05 m; the manager's stand check accepts 0.02 .. 0.15 m) on both sides, so a base that
stops that far early or late is still on the plateau, and the recommendation is the centre of the shrunk band.
`min_distance` / `max_distance` are the plateau, `robust_min` / `robust_max` the shrunk band.

Edge cases are reported, not hidden (`status`, `recommended.reason`):

| Case | Result |
|---|---|
| Non-finite number, zero step, ceiling or floor, no targets, absurd size | refused (`invalid_request`, `problem`) |
| No target reachable at any distance | `nothing_reachable`, no recommendation |
| Reachable, but the footprint never fits the clearance in the range | `no_footprint_fit` |
| Plateau narrower than twice the tolerance, or one distance only | recommended with `robust: false` and the reason says so |
| No distance reaches `quality_min` | quality gate dropped, `quality_ok: false`, "WARNING" in the reason |
| Plateau reaches the end of the swept range | `touches_range: true`, "extend it" |

## How to run

The map is the **arm's** map (base frame `arm_base_link`, tool `tcp_link`); the lift and the base enter through the
profile (`--profile`: the `base:` footprint and arm mount, `lift.stroke_m`, and the arm base height at lift zero =
`base.lift_mount_height_m + lift.mount_height_m`, 0.35 + 0.82 = 1.17 m for the shipped telescopic profiles).

```bash
# 1. profile -> chain -> map (needs the ROS workspace: xacro, urdf_to_chain.py, rm4d_tool). <arm> is the profile's
#    xacro `arm:` value (ur10e, xarm7, ...). 3 M samples take seconds on 6 joints.
src/common/standoff_advisor/scripts/profile_to_map.sh ur10e /tmp/ur10e_map
#    equivalent to:
#    xacro mobile_manipulator.urdf.xacro arm:=ur10e -o robot.urdf
#    urdf_to_chain.py robot.urdf --base arm_base_link --tip tcp_link -o ur10e_chain.yaml
#    rm4d_tool build ur10e_chain.yaml ur10e.rm4d --samples 3000000 --resolution 0.05 --extent 2.0 --zmin -1.8 --zmax 2.0

# 2. sweep the smoke wall (whole wall, or a patch), JSON + CSV + table and bar chart on stdout
standoff_sweep --map /tmp/ur10e_map/ur10e.rm4d \
  --wall-scene src/simulation/sim_gazebo/test/data/smoke_wall_scene.json \
  --profile src/description/mobile_manipulator_description/config/profiles/ur10e_telescopic.yaml \
  --wall-z 1.0 1.3 --wall-patch -0.5 0.5 0.7 1.0 --at 0.98 \
  --out-json sweep.json --out-csv sweep.csv

# 3. picture (matplotlib; ASCII chart when it is missing)
src/common/standoff_advisor/scripts/standoff_plot.py sweep.json -o sweep.png --today 0.98
```

`--chain chain.yaml` builds the map in place instead of `--map` (`--samples --resolution --extent --zmin --zmax
--threads --save-map`). Wall: `--wall-scene scene.json [--wall-id 0]` (origin, basis u/v, `normal_outward`, u/v
ranges; the whole wall unless narrowed) or `--wall-point x y z --wall-normal nx ny nz --wall-patch u0 u1 v0 v1`
(u = Z x n along the wall, v up, relative to the point). `--wall-patch u0 u1 v0 v1` and `--wall-z z0 z1` (world
heights, for walls whose v axis is vertical) narrow a scene's wall; `--patch-res` is the target spacing (0.1 m).
`--tool-standoff m` moves the targets off the wall by that much (use it to check the approach pose of a column, see below).
Other options: `--standoff-min/-max/-step`, `--lift-min/-max/-step`, `--mount-z`, `--min-clearance`,
`--nav-tol`, `--quality-min`, `--count-ref`, `--fraction-tol`, `--lateral-step`, `--at d1,d2` (evaluate given distances,
for example the one in use today). Exit code 0 when a distance is recommended, 1 when not, 2 for bad arguments.

JSON: `recommended.recommended_standoff_m` (= `arm_base_to_wall_m`), `base_origin_to_wall_m`, `arm_mount_x_in_base_m`,
`base_yaw_rad`, the plateau and robust band, `reachable_fraction`, `robust_fraction`, `mean_quality`, lift range, `reason`;
`evaluated_at` (the `--at` distances); `sweep` (one row per distance, same columns as the CSV).

C++: `standoff_advisor::sweepStandoff(map, StandoffSweepRequest)` and `evaluateStandoff(map, request, d)`
(`standoff_advisor/standoff_sweep.hpp`); `patchTargets`, `patchFromPlane`, `sweepProblem`.

## Which distance is which

- **Standoff of the base** (this tool): arm base point to the wall plane, `recommended_standoff_m`. The reachability
  map and the `PlanStands` request measure the stand the same way (`Station::standoff`). The base's `base_footprint`
  origin is `base_origin_to_wall_m` away (they differ when `arm_mount_xy` is not the origin; the footprint's nearest
  point is `footprint_clearance`).
- **Wall plane**: the plane the tool touches. In the sims it is x = 0.98 m in `base_footprint` (the Gazebo world wall is at
  0.915 m, the TCP ends 6.5 cm before it; see `docs/sim_gazebo.md`).
- **`dock_standoff_m`** of `manipulation_manager` (default 0.12 m, plus `dock_approach_extra_m`): tool-to-wall, the
  distance of the dock pose from the plane before the press. It is an arm motion, not a base position. The base-to-wall
  standoff must leave that dock pose and the contact pose reachable: sweep the contact patch (default) and again with
  `--tool-standoff 0.12` (+ the approach extra) to see the approach poses.

## Using the result

- **Stand search** (`plan_stands`, `PlanStands.srv`): `min_clearance` and `max_standoff` bound the footprint clearance
  and the arm base distance (0 = defaults 0.3 and 1.6 m). To keep the stands near the recommendation set
  `min_clearance` and `max_standoff` so that the arm base distance stays inside `robust_min .. robust_max` (the search ranks by unreachable
  targets, lift bands and nearness to the work area, with no preference for a distance of its own). `arm_mount_offset_z` and `lift_max` must be the values
  the sweep used. `column_reach_tool` takes the same fields in its plan JSON (`min_clearance`, `max_standoff`).
- **Docking** (`stand_navigator`, `WallDock` goal `standoff_m`): distance from a reference point (default the profile's
  `arm_mount_xy`, parameter `ref_xy`) to the fitted wall plane, i.e. the arm base distance: pass `recommended_standoff_m`.
  Tolerance `tolerance_standoff_m` is 5 mm by default, well inside `robust` (+- `nav_tolerance_m`).
- **Sim start pose / wall placement**: the Gazebo scripts keep the base at its start pose and put the wall at
  x = 0.98 m (`WALL_X` in `run_multi_stand_pass.sh`, the world wall at `standoff:=0.915`); to test another distance move
  both (world wall = contact plane minus 0.065 m). MuJoCo `stand_drive.py --wall-x` is the wall's x in the world
  frame; the arm base is at the stand `plan_stands` returns.

## Result on the real chain and the smoke wall

UR10e on the generic omni base, telescopic lift 0..1.5 m, arm base 1.17 m above the ground at lift zero, wall plane x = 2.0
of `smoke_wall_scene.json` (3 x 2 m), tool pointing into the wall, targets 0.1 m apart, footprint 0.9 x 0.6 m with
`min_clearance` 0.30 m (best yaw is sideways, so the clearance limit is 0.60 m arm base distance), `nav_tolerance_m` 0.05.
Map: 3 M samples from the chain made out of the `ur10e` URDF, 0.05 m cells. Today's dock: wall plane 0.98 m from the arm
base (`arm_mount_xy` = origin).

| Targets | n | recommended | plateau | robust band | reach at recommended | reach at 0.98 m | quality at recommended / 0.98 |
|---|---|---|---|---|---|---|---|
| patch z 1.0..1.3, one stand wide (u -0.5..0.5) | 44 | 0.97 m | 0.60 .. 1.34 | 0.64 .. 1.30 | 100% | 100% | 0.76 / 0.76 |
| patch z 1.0..1.3, full width 3 m | 124 | 0.70 m | 0.60 .. 0.80 | 0.64 .. 0.76 | 76.6% | 65.3% | 0.92 / 0.71 |
| whole wall 3 x 2 m | 651 | 0.70 m | 0.60 .. 0.80 | 0.64 .. 0.76 | 63.1% | 50.7% | 0.89 / 0.70 |

Reading it. One stand's work (a patch the arm spans) is reachable at every distance from 0.6 to 1.34 m, so today's 0.98 m
is inside the plateau (only 0.01 m from its centre), but the pose quality falls steadily with distance (0.94 at 0.70 m, 0.76 at
0.98 m, 0.5 at 1.3 m): with `--quality-min 0.8` the plateau is 0.60 .. 0.94 m and the recommendation 0.77 m. For wall areas wider than one stand's
reach the share one stand covers peaks close to the wall (85% at 0.42 m, which does not fit the footprint) and falls
as the base backs off, so the 3 m patch loses 11 points (76.6% to 65.3%) at 0.98 m and the whole wall 12 points. The
base is therefore not too close at the Gazebo test's 0.98 m; the dock near 0.6 m (the footprint limit) is. What
`standoff_sweep` does not say is how many stands the wall needs: use `plan_stands` for that.

A second arm: the generic 6-DOF arm (chain from `generic_6dof` mock URDF, profile `generic_6dof_telescopic`): narrow patch recommended
0.95 m (plateau 0.60 .. 1.30), 100% at 0.98 m; full 3 m patch 0.67 m (plateau 0.60 .. 0.74), 73.4% there and 60.5% at 0.98 m.

## Limits

- The map has 0.05 m cells and one lift-independent arm map: the fraction moves in steps and a distance difference under
  a cell is not resolved. Reachable means "some arm configuration lands in the cell" (joint limits only), not IK, not
  collision-free.
- The quality is the map's sample-count and neighbour-cell score, not manipulability; it is high when the arm is folded close
  to the wall. `count_ref` and `quality_min` are starting values to set against the map in use.
- No obstacles, no self-collision, no wall protrusions, no wall edge; only straight vertical walls (a horizontal normal);
  the arm base is positioned anywhere along the wall, and one stand per distance (a wall wider than the arm needs several stands).
- No dynamics: nothing about the base's settling, the lift's speed or the contact force; the navigation tolerance is a number you
  give, not one measured. The approach pose and the strokes are checked only if you sweep them (`--tool-standoff`).
- Not run on hardware or in a simulation with the recommended distance; the Gazebo and MuJoCo paths above were read, not driven.
