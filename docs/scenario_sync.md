# One scenario for Gazebo, MuJoCo and RViz (`scenario_sync`)

Status: tools written and run on the host with python only; **no simulator, planner or RViz was run**, so nothing here shows
that the three views agree on a live run, only that the files they are built from do or do not.

Gazebo, MuJoCo, RViz and the coverage plan each get their wall, base start and frame from different places, and they had
drifted apart. `src/common/scenario_sync` (python only, no ROS import) puts those numbers in one `scenario.yaml`,
generates each consumer's inputs from it, and checks that the files agree.

## The tools

| Tool | What it does |
|---|---|
| `scenario.yaml` | The schema (documented in `scenario_sync/schema.py` and the README), validated in python: every problem is listed at once, unknown keys are errors. |
| `python3 -m scenario_sync.scenario_gen scenario.yaml OUT` | Writes `plan/wall_scene.json` (the work patch), `plan/wall_scene_full.json`, `gazebo/wall_scene.json` + `gazebo/wall.sdf`, `mujoco/wall_scene.json` + `mujoco/wall_world.xml`, `rviz_args.yaml`, `env.sh`, `manifest.json`. The SDF and the MJCF geoms are written by the simulators' own `wall_world.py` modules, loaded from the source tree by path; nothing under `src/simulation` is edited. |
| `python3 -m scenario_sync.check_sync scenario.yaml [files]` | Pass/fail table, exit 1 on a mismatch (0 with only WARN or SKIP, `--strict` fails on WARN, 2 if an input cannot be read). `--gen-dir OUT` takes everything `scenario_gen` wrote. |
| `python3 -m scenario_sync.audit_repo` | Runs the repository's own generators (`shift_wall_scene.py`, `make_scenario_scene.py`, the two `wall_world.py`) and `check_sync`s their output against the example scenarios. Produced the tables below. |

Conventions: the world frame defaults to `odom`, where the base starts at `base.x/y/yaw`. The wall is a vertical plane given by
a point on it and its outward normal (pointing at the robot); `u` runs along the wall (`u = n x up`, +y for a wall at +x) and `v`
is metres down from the point, the `wall_scene.json` convention. `base.standoff_m` is the base-to-plane distance along the
normal. It can come from a standoff advisor's JSON (`base.standoff_advisor_json`, only the key `recommended_standoff_m` is
read; the base is moved along the normal to it). The work patch is `mode: full` (the whole wall) or `mode: crop` with a u/v
window. Consumers: `consumers.gazebo.sim_wall_offset_m` (0.065: the Gazebo world wall stands 6.5 cm nearer the base than
the planned plane, documented in `docs/sim_gazebo.md`), `consumers.rviz.fixed_frame`.

Checks, per source: frame name vs the world frame (`base_link` and `base_footprint` are a WARN while the base starts at the
origin, anything else such as `map` or `link_base` is a FAIL); wall normal; plane distance from the base (standoff, minus the
Gazebo offset); u/v extents against the full wall (sim files) or the work patch (plan scene); plan scene inside the wall; plan
poses (`coverage_output.json`) inside the patch, their distance from the plane, planner area vs patch; "covers X % of the wall"
(WARN below 99.5 %, so a cropped test is flagged every time); RViz fixed/plan frame and base x, y, yaw; obstacle box count.

## Three example scenarios

`src/common/scenario_sync/examples/`:

| File | Wall | Work patch | Source of the numbers |
|---|---|---|---|
| `gazebo_smoke_full.yaml` | 3.0 x 2.0 m, z 0..2, plane 0.98 m ahead of the base, one 5 cm box | the whole wall | `sim_gazebo/test/data/smoke_wall_scene.json`, moved onto the plane the tool touches |
| `gazebo_patch_crop.yaml` | the same | crop y 0.3..1.2, z 1.0..1.3 (0.9 x 0.3 m, 4.5 % of the wall) | `run_multi_stand_pass.sh` (DY 0.3, WIDTH 0.9, coverage z 1.0..1.3) |
| `mujoco_tall_wall.yaml` | 1.4 x 2.0 m, y 0.05..1.45, z 0.3..2.3 | crop y 0.3..1.2, z 0.6..2.0 (0.9 x 1.4 m, 45 % of the wall) | `two_panels_tall` of `make_scenario_scene.py` (PR #190), generator output |

All three generate and check clean (`check_sync` exit 0; the only WARN rows are the declared crops).

## Findings: what disagrees today

`python3 -m scenario_sync.audit_repo` on this checkout (generators run on the host; the coverage planner itself was not run, so
"the plan" below is the `wall_scene.json` it is given, which fixes its box; a real `coverage_output.json` can be checked with
`--plan-output`).

| # | Disagreement | Numbers |
|---|---|---|
| 1 | The Gazebo smoke scene is not where the planner plans | `smoke_wall_scene.json`: plane x = 2.000 in frame `map`. The plan scene: plane x = 0.980, frame `base_link`. Difference 1.020 m and two frame names. The Gazebo launch moves the wall to its `standoff` argument, so the 2.0 never reaches the sim, but any consumer reading the file as it is (an RViz view of the shipped scene) sees a wall 1.02 m behind the planned one. |
| 2 | Plan covers a small crop of the wall | The multi-stand plan is 0.9 x 0.3 m = 0.27 m^2: 4.5 % of the nominal 3 x 2 m wall. |
| 3 | The Gazebo multi-stand wall is not the 3 x 2 m wall | `shift_wall_scene.py` widens the sim wall together with the patch: 3.6 x 2.0 m (7.2 m^2), and `wall_world.generate_world` re-centres it on the robot (y -1.8..1.8), while the patch is at y 0.3..1.2. Against `gazebo_patch_crop` the extents fail by 0.30 m per side; the patch is 3.8 % of the wall as built. The wall also cannot be placed off-centre through `wall_scene:=`, only through a generated world (`world:=`). |
| 4 | Gazebo wall plane is not the planned plane | 0.915 m vs 0.980 m, 65 mm. Intended and documented (tool tip 6.5 cm short); `sim_wall_offset_m` makes it explicit and the check passes with it. |
| 5 | The two simulators' stock walls differ | MuJoCo `smoke_wall_scene.json`: 0.8 x 0.7 m (y -0.25..0.55, z 0.8..1.5), no obstacle. Gazebo: 3.0 x 2.0 m (y -1.5..1.5, z 0..2), one obstacle. Same plane (0.980). Max extent difference 1.25 m. |
| 6 | MuJoCo tall wall vs Gazebo patch test | `two_panels_tall` is internally consistent (all extents, plane and MJCF PASS, `base_link` frame WARN). Against the Gazebo wall it differs: y 0.05..1.45 vs -1.5..1.5, z 0.3..2.3 vs 0..2. A scenario that is "the same in both simulators" needs the generated worlds, not the runners' own. |
| 7 | Frame names | Plan scenes: `base_link`; shipped Gazebo scene: `map`; `wall_coverage_demo.launch.py` default Fixed Frame: `link_base`; recording frame: `odom` (multi-stand) or `base_footprint` (other passes); mp_rviz profiles (branch `feat/rviz-profiles`): `base_footprint` 16 profiles, `base_link` 2, `map` 3 (`coverage_planner`, `stand_navigator`, `layers`), `world` 1. With the base at the origin `base_link`, `base_footprint` and `odom` coincide; the moment the base starts elsewhere or drives, only `odom` matches the plan. `map`, `link_base` and `world` match only through a transform to `odom` that this check cannot see (not looked for). |
| 8 | Base starts at different distances | `sim.launch.py` / `wall_world_gen.py` default standoff 0.900 m; every Gazebo pass script 0.915 m (to the sim wall); MuJoCo passes and all planning 0.980 m; smoke file 2.0 m. |

Where the checks pass today: MuJoCo `two_panels_tall` internally (plan scene, sim scene and MJCF agree with each other to 0 mm,
row-by-row below); Gazebo multi-stand plan scene vs the crop (plane, extents, normal), and its plane distance with the 65 mm
offset.

Not found, not looked for: whether the plan, the recorded run and RViz agree on a live run (not run), the `walls.pcd` clouds
(not read), the real `coverage_output.json` of a run (none in the repository).

## How to run the three views from one scenario file

```bash
cd src/common/scenario_sync
python3 -m scenario_sync.scenario_gen examples/gazebo_patch_crop.yaml /tmp/scn/crop
source /tmp/scn/crop/env.sh
python3 -m scenario_sync.check_sync examples/gazebo_patch_crop.yaml --gen-dir /tmp/scn/crop
```

* Coverage plan: `ros2 run coverage_planner wall_coverage_demo_node --ros-args -p wall_scene_json:=$SCENARIO_PLAN_SCENE -p output_json:=run/coverage_output.json ...`
  (plan stays in `odom`). After the run: `check_sync ... --gen-dir /tmp/scn/crop --plan-output run/coverage_output.json`.
* Gazebo: `ros2 launch sim_gazebo sim.launch.py "${SCENARIO_GAZEBO_LAUNCH_ARGS[@]}"` (`world:=.../wall.sdf profile:=...`). The runner
  scripts always pass `wall_scene:=` and `standoff:=`, which wins over `world:=`, so they build their own wall; until they read
  `$SCENARIO_DIR` use the launch directly, or `SCENARIO_GAZEBO_ALIGNED_ARGS` for a wall centred on y = 0.
* MuJoCo: `ros2 launch sim_mujoco sim_mujoco.launch.py "${SCENARIO_MUJOCO_LAUNCH_ARGS[@]}"` (`scene:=` the world-frame scene,
  `scene_offset:=` the inverse of the base start). The runners use `make_scenario_scene.py`; the `MULTI_STAND_*` variables in
  `env.sh` are the legacy equivalents and `env.sh` warns when they cannot reproduce the scenario (margins, obstacles, patch z).
* RViz: set Fixed Frame to `$SCENARIO_RVIZ_FIXED_FRAME`, take `wall_scene_json` and `base_x/y/yaw` from `$SCENARIO_RVIZ_ARGS`
  (`mp_viz_scene` parameters `frame_id`, `wall_point`, `wall_normal` are in the same file, to be passed with `--params-file`).
  The base itself comes from the sim's TF; both sims spawn it at their own origin, so a base start other than 0, 0, 0 is
  expressed by moving the wall, and `check_sync` reports the wall in the world frame.

## Not tested, limits

* No simulator, no planner, no RViz run. The generated `wall.sdf` was never loaded by `gz sim`, the MJCF geoms never by MuJoCo,
  `rviz_args.yaml` never read by a launch (no launch reads it yet), `env.sh` never sourced by a runner (it is plain bash).
* The runners do not consume `scenario.yaml` yet; wiring them (`SCENARIO_DIR`) touches `src/simulation`, which is frozen for the
  pending submodule split.
* Vertical walls with a horizontal normal only; one wall (id 0); the SDF and MJCF readers assume the generators' own box
  layout (front face = local +z, `wall_*` / `obstacle_*` names).
* `check_sync` compares geometry, not obstacle positions (only their count), not `walls.pcd`, not the arm's reach or the lift
  range against the patch.
* Legacy `MULTI_STAND_*` output is only as good as the runner's own wall; the audit shows it is not the same wall.

## Audit output

Generated by `python3 -m scenario_sync.audit_repo --format md`.

### Gazebo smoke_wall_scene.json as shipped vs gazebo_smoke_full (the whole wall)

| Source | Check | Status | Detail |
|---|---|---|---|
| gazebo smoke_wall_scene.json | frame | FAIL | 'map' vs expected 'odom' |
| gazebo smoke_wall_scene.json | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| gazebo smoke_wall_scene.json | wall plane distance | FAIL | 2.000 m from the base start, expected 0.980 m (diff -1.020 m, tol 0.005) |
| gazebo smoke_wall_scene.json | extents vs full wall | PASS | u -1.500..1.500 (expected -1.500..1.500), v 0.000..2.000 (expected 0.000..2.000); 3.000 x 2.000 m, z 0.000..2.000 (expected 0.000..2.000), max diff 0.000 m (tol 0.010) |
| gazebo smoke_wall_scene.json | obstacle boxes | PASS | 1 in the file, 1 in the scenario |

The file is what `sim.launch.py wall_scene:=` takes; the launch moves the wall to `standoff` anyway, so its x = 2.0 only matters to a consumer that reads the file as it is.

### Gazebo multi-stand pass as its runner builds it vs gazebo_patch_crop

| Source | Check | Status | Detail |
|---|---|---|---|
| gazebo plan scene (shift 0 0.3 0.9) | frame | WARN | 'base_link' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| gazebo plan scene (shift 0 0.3 0.9) | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| gazebo plan scene (shift 0 0.3 0.9) | wall plane distance | PASS | 0.980 m from the base start, expected 0.980 m (diff +0.000 m, tol 0.005) |
| gazebo plan scene (shift 0 0.3 0.9) | extents vs work patch | PASS | u 0.300..1.200 (expected 0.300..1.200), v 0.700..1.000 (expected 0.700..1.000); 0.900 x 0.300 m, z 1.000..1.300 (expected 1.000..1.300), max diff 0.000 m (tol 0.010) |
| gazebo plan scene (shift 0 0.3 0.9) | plan scene inside the wall | PASS | u 0.300..1.200 v 0.700..1.000 vs wall u -1.500..1.500 v 0.000..2.000 |
| gazebo plan scene (shift 0 0.3 0.9) | share of the wall | WARN | the planned window covers 4.5 % of the wall (0.270 of 6.000 m^2; crop declared in the scenario) |
| gazebo wall.sdf (launch, standoff 0.915) | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| gazebo wall.sdf (launch, standoff 0.915) | wall plane distance | PASS | 0.915 m from the base start, expected 0.915 m (diff -0.000 m, tol 0.005) (scenario standoff 0.980 m minus the sim wall offset 0.065 m) |
| gazebo wall.sdf (launch, standoff 0.915) | extents vs full wall | FAIL | u -1.800..1.800 (expected -1.500..1.500), v -0.000..2.000 (expected 0.000..2.000); 3.600 x 2.000 m, z 0.000..2.000 (expected 0.000..2.000), max diff 0.300 m (tol 0.010) |
| gazebo wall.sdf (launch, standoff 0.915) | obstacle boxes | PASS | 0 in the file, 0 in the scenario |

As built, the Gazebo wall is 3.60 x 2.00 m (7.20 m^2); the planned patch is 0.90 x 0.30 m = 3.8 % of it.

### MuJoCo stock smoke_wall_scene.json vs gazebo_smoke_full (the same wall in the other simulator)

| Source | Check | Status | Detail |
|---|---|---|---|
| mujoco smoke_wall_scene.json | frame | WARN | 'base_link' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| mujoco smoke_wall_scene.json | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| mujoco smoke_wall_scene.json | wall plane distance | PASS | 0.980 m from the base start, expected 0.980 m (diff +0.000 m, tol 0.005) |
| mujoco smoke_wall_scene.json | extents vs full wall | FAIL | u -0.250..0.550 (expected -1.500..1.500), v 0.500..1.200 (expected 0.000..2.000); 0.800 x 0.700 m, z 0.800..1.500 (expected 0.000..2.000), max diff 1.250 m (tol 0.010) |
| mujoco smoke_wall_scene.json | obstacle boxes | FAIL | 0 in the file, 1 in the scenario |

### MuJoCo two_panels_tall (make_scenario_scene.py) vs mujoco_tall_wall

| Source | Check | Status | Detail |
|---|---|---|---|
| mujoco plan scene (two_panels_tall) | frame | WARN | 'base_link' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| mujoco plan scene (two_panels_tall) | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| mujoco plan scene (two_panels_tall) | wall plane distance | PASS | 0.980 m from the base start, expected 0.980 m (diff +0.000 m, tol 0.005) |
| mujoco plan scene (two_panels_tall) | extents vs work patch | PASS | u -0.450..0.450 (expected -0.450..0.450), v -0.700..0.700 (expected -0.700..0.700); 0.900 x 1.400 m, z 0.600..2.000 (expected 0.600..2.000), max diff 0.000 m (tol 0.010) |
| mujoco plan scene (two_panels_tall) | plan scene inside the wall | PASS | u -0.450..0.450 v -0.700..0.700 vs wall u -0.700..0.700 v -1.000..1.000 |
| mujoco plan scene (two_panels_tall) | share of the wall | WARN | the planned window covers 45.0 % of the wall (1.260 of 2.800 m^2; crop declared in the scenario) |
| mujoco sim scene (two_panels_tall) | frame | WARN | 'base_link' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| mujoco sim scene (two_panels_tall) | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| mujoco sim scene (two_panels_tall) | wall plane distance | PASS | 0.980 m from the base start, expected 0.980 m (diff +0.000 m, tol 0.005) |
| mujoco sim scene (two_panels_tall) | extents vs full wall | PASS | u -0.700..0.700 (expected -0.700..0.700), v -1.000..1.000 (expected -1.000..1.000); 1.400 x 2.000 m, z 0.300..2.300 (expected 0.300..2.300), max diff 0.000 m (tol 0.010) |
| mujoco sim scene (two_panels_tall) | obstacle boxes | PASS | 0 in the file, 0 in the scenario |
| mujoco MJCF (two_panels_tall) | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| mujoco MJCF (two_panels_tall) | wall plane distance | PASS | 0.980 m from the base start, expected 0.980 m (diff +0.000 m, tol 0.005) |
| mujoco MJCF (two_panels_tall) | extents vs full wall | PASS | u -0.700..0.700 (expected -0.700..0.700), v -1.000..1.000 (expected -1.000..1.000); 1.400 x 2.000 m, z 0.300..2.300 (expected 0.300..2.300), max diff 0.000 m (tol 0.010) |
| mujoco MJCF (two_panels_tall) | obstacle boxes | PASS | 0 in the file, 0 in the scenario |

make_scenario_scene.py reports {"MODE": "multi", "DX": 0.0, "DY": 0.3, "WIDTH": 0.9, "WALL_X": 0.98, "WALL_Y0": 0.3, "WALL_Y1": 1.2, "OBSTACLES": 0, "PATCH_Z_LO": 0.6, "PATCH_Z_HI": 2.0, "WALL_Z_LO": 0.3, "WALL_Z_HI": 2.3}

### MuJoCo two_panels_tall sim scene vs gazebo_patch_crop (does one scenario fit both simulators today?)

| Source | Check | Status | Detail |
|---|---|---|---|
| mujoco sim scene (two_panels_tall) | frame | WARN | 'base_link' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| mujoco sim scene (two_panels_tall) | wall normal | PASS | 0.00 deg from the scenario normal (tol 0.57) |
| mujoco sim scene (two_panels_tall) | wall plane distance | PASS | 0.980 m from the base start, expected 0.980 m (diff +0.000 m, tol 0.005) |
| mujoco sim scene (two_panels_tall) | extents vs full wall | FAIL | u 0.050..1.450 (expected -1.500..1.500), v -0.300..1.700 (expected 0.000..2.000); 1.400 x 2.000 m, z 0.300..2.300 (expected 0.000..2.000), max diff 1.550 m (tol 0.010) |
| mujoco sim scene (two_panels_tall) | obstacle boxes | PASS | 0 in the file, 0 in the scenario |

### Frame names of the views vs the world frame odom

| Source | Check | Status | Detail |
|---|---|---|---|
| frames | wall_coverage_demo.launch.py default Fixed Frame | FAIL | 'link_base' vs expected 'odom' |
| frames | wall_scene.json written by the MuJoCo/Gazebo scenario builders | WARN | 'base_link' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| frames | Gazebo smoke_wall_scene.json | FAIL | 'map' vs expected 'odom' |
| frames | recording frame, multi-stand pass (docs/how_to_read_a_run.md) | PASS | 'odom' |
| frames | recording frame, other passes (docs/how_to_read_a_run.md) | WARN | 'base_footprint' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| frames | mp_rviz profiles (16): ft_sensor, manipulation_manager, mission_overview, moveit_planning, planner_backends, reachability, reachability_moveit, robot_model, safety_teleop, scene_model, self_filter, sim_gazebo, sim_isaac, sim_mujoco, tool_trace, wall_detector | WARN | 'base_footprint' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| frames | mp_rviz profiles (2): cartesian_validation, jtac_compliance | WARN | 'base_link' vs world frame 'odom': the same place only while the base stands at the origin (a different name for tf and RViz) |
| frames | mp_rviz profiles (3): coverage_planner, layers, stand_navigator | FAIL | 'map' vs expected 'odom' |
| frames | mp_rviz profiles (1): fsp_benchmark | FAIL | 'world' vs expected 'odom' |

### Base-to-wall distance by entry point

| Source | Check | Status | Detail |
|---|---|---|---|
| sim_gazebo/sim.launch.py | default standoff | WARN | 0.900 m (also wall_world_gen.py --standoff) |
| sim_gazebo run_multi_stand_pass / run_manager_pass / run_spray_pass / run_depth_wall | standoff | WARN | 0.915 m to the sim wall; the planned plane is 0.980 m (tool tip 6.5 cm short, documented) |
| sim_mujoco run_*pass | plane | WARN | 0.980 m (+ MULTI_STAND_WALL_DX) |
| sim_gazebo smoke_wall_scene.json | plane | FAIL | x = 2.000 m in frame map (not used as is by any pass) |

