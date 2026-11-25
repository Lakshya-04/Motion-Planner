# scenario_sync

One `scenario.yaml` for Gazebo, MuJoCo, RViz and the coverage planner. Python only: no ROS import, runs on a plain host
(`python3`, `numpy`, `pyyaml`). The simulators are not edited; their own generators are loaded from the source tree by path.
Findings, conventions and the full audit: [docs/scenario_sync.md](../../../docs/scenario_sync.md).

```bash
cd src/common/scenario_sync
python3 -m scenario_sync.scenario_gen examples/gazebo_patch_crop.yaml /tmp/scn/crop   # inputs for every consumer
source /tmp/scn/crop/env.sh                                                           # launch args, file paths, MULTI_STAND_*
python3 -m scenario_sync.check_sync examples/gazebo_patch_crop.yaml --gen-dir /tmp/scn/crop \
    [--plan-output run/coverage_output.json] [--strict] [--format md|json]            # exit 1 on a mismatch
python3 -m scenario_sync.audit_repo                                                   # the repo's own generators vs the examples
python3 -m pytest src/common/scenario_sync                                            # from the repository root
```

## scenario.yaml (schema_version 1)

Lengths in metres, angles in radians; the full reference is the docstring of `scenario_sync/schema.py`.

```yaml
name: gazebo_patch_crop
world_frame: odom                 # where the base starts at base.x/y/yaw (default odom)
wall:                             # the FULL wall: a point on the plane, the outward normal (at the robot), u/v extents
  point: [0.98, 0.0, 2.0]         # u = n x up (+y for a wall at +x); v is metres DOWN from the point
  normal: [-1.0, 0.0, 0.0]
  u_range: [-1.5, 1.5]
  v_range: [0.0, 2.0]
patch:                            # what is planned and executed
  mode: crop                      # full | crop
  u_range: [0.3, 1.2]
  v_range: [0.7, 1.0]
obstacles:                        # optional: box (depth off the wall) or opening (a hole)
  - {kind: box, u: [0.6, 0.9], v: [0.5, 0.8], depth: 0.05}
base: {x: 0.0, y: 0.0, yaw: 0.0, standoff_m: 0.98}   # or standoff_advisor_json: path (key recommended_standoff_m)
robot_profile: ur10e_telescopic
tool_profile: spray_gun
lift_range_m: [0.0, 1.5]
consumers:
  gazebo: {sim_wall_offset_m: 0.065}   # the sim wall stands this much nearer the base than the planned plane
  rviz: {fixed_frame: odom}
```

`base.standoff_m` must equal the base-to-plane distance of `base.x/y`; leave one of them out to have it derived. With
`standoff_advisor_json` the standoff is read from the advisor's JSON and the base is moved along the normal to it.

## What `scenario_gen` writes

`plan/wall_scene.json` (the work patch, world frame), `plan/wall_scene_full.json`, `gazebo/wall_scene.json` and `gazebo/wall.sdf`
(`wall_world_gen`'s generator, scene frame kept, load with `world:=`), `mujoco/wall_scene.json` and `mujoco/wall_world.xml`
(`wall_world` geoms placed with `scene_offset`), `rviz_args.yaml`, `env.sh`, `manifest.json`.

## What `check_sync` compares

Frame names, wall normal, plane distance from the base (standoff, minus the Gazebo offset), u/v extents of each source against the
full wall or the work patch, plan scene inside the wall, plan poses (`coverage_output.json`) inside the patch and near the plane,
planner area against the patch and against the wall ("covers X % of the wall", a WARN for any crop), RViz frames and base pose,
obstacle count. Exit 0: PASS (WARN and SKIP allowed unless `--strict`), 1: a FAIL, 2: an input cannot be read.

Limits: vertical walls with a horizontal normal, one wall, geometry only. Not run in any simulator.
