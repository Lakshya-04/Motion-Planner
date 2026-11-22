# Edge columns: what a stand can reach on a vertical-strip wall

Tool: `docker/tools/wall_reach_report.py` (host Python, numpy only, no ROS, no docker). Tests:
`docker/tools/tests/test_wall_reach_report.py`.

It lives under `docker/tools` next to the other host-side helpers (`check_public_readiness.py`,
`bench_spheres.py`, their `tests/`), which already run with plain `python3` and `pytest`. There is no
`src/common/standoff_advisor` in this tree, so there was no sweep to reuse.

## Which model this is (read first)

This is an **analytic reach model, not an RM4D map.** What it does and does not do:

- The arm is the UR10e as a published Universal Robots DH chain (`d1 0.1807, a2 -0.6127, a3 -0.57155,
  d4 0.17415, d5 0.11985, d6 0.11655`) with closed-form inverse kinematics over all 8 branches.
  Joint limits are the `ur_description` ones: elbow +-pi, the others +-2 pi. The IK is checked against
  forward kinematics on random configurations in the tests.
- The tool is a contact tool: its z axis points straight into the wall (tilt 0, as `tool_profiles.yaml`
  requires for contact tools) and it may roll freely about that axis (sampled every 15 degrees).
- A wall point counts only if the contact point and the point `--approach-m` (default 0.12 m, the
  `sequence.standoff_m` default of `column_reach_tool`) in front of it are both reachable, and the joint
  origins of frames 2 to 6 stay in front of the wall.
- The lift is continuous over 0..1.5 m, so a wall height is reachable if some lift height puts it in the
  arm's band. The arm base is 1.17 m above the floor with the lift retracted: 0.35 m (generic base top,
  `urdf/base/generic_base.xacro`: 0.05 clearance + 0.30 height) plus 0.82 m (`lift.mount_height_m` in
  `profiles/ur10e_telescopic.yaml`). Change it with `--arm-base-z0` if your mount differs.
- NOT modelled: self-collision, collision of links or the base with the wall, floor, or lift column,
  IK branch continuity along a stroke, lift speed, the real controller's joint-limit margins, orientation
  tolerance of the planner. So "reachable" here is a necessary condition. Where this tool says a strip is
  unreachable it is unreachable; where it says reachable, a stroke can still fail for the reasons listed.

Why not RM4D: `rm4d_tool` and `column_reach_tool` build on the host with g++ (Eigen and yaml-cpp
headers are present), but they need a chain YAML written by `scripts/urdf_to_chain.py`, which needs
`urdfdom_py` and a rendered URDF (`xacro` plus the `ur_description` package); neither `urdfdom_py` nor the
package is installed on this host, and a map needs a multi-million sample build. The analytic model uses
the same published kinematics, so it answers the same pure-reach question; it is not a replacement for the
RM4D verification when you can build the map.

## Commands

```bash
# Tests (about 2 s)
python3 -m pytest -q docker/tools/tests/test_wall_reach_report.py

# The wall in question: 19 strips, 0.16 m apart, z 0.5..1.9, standoffs 0.60..0.80 m (about 3 min)
python3 docker/tools/wall_reach_report.py --json /tmp/edge_columns.json

# Per-strip report for a given stand layout (stands 0.5 m span each: 5 stands), standoff 0.60 and 0.80
python3 docker/tools/wall_reach_report.py --standoff 0.60 0.80 --max-lateral 0.6 --spans 0.5 1.0 \
    --stand-y 0.24 0.88 1.52 2.16 2.72 --stand-standoff 0.60

# Stricter: contact point only, joint origins 0.15 m in front of the wall
python3 docker/tools/wall_reach_report.py --standoff 0.60 0.80 --approach-m 0 --link-clearance-m 0.15 --spans 0.5
```

`--stand-y` positions are along the wall in the same coordinates as the strips (strip i at
`--strip-y0 + i * --pitch`, default 0). A stand's lateral position is where the arm base sits.

## Results (this wall, UR10e, lift 0..1.5 m, base z0 1.17 m, approach 0.12 m)

Largest lateral offset between a stand and a strip such that the strip is reachable over the whole
0.50..1.90 m (grid 0.02 m):

| standoff (m) | max lateral offset (m) | so `stand_max_span_m` <= | stands for 19 strips (span rule) |
|---|---|---|---|
| 0.60 | 0.88 | 1.76 | 2 |
| 0.65 | 0.86 | 1.72 | 2 |
| 0.70 | 0.84 | 1.68 | 2 |
| 0.75 | 0.82 | 1.64 | 2 |
| 0.80 | 0.78 | 1.56 | 2 |

Straight ahead of the stand the whole 0.50..1.90 m is reachable at every standoff from 0.60 to 0.80 m.
At 0.60 m the lateral reach at the bottom of the wall (z 0.50) is +-0.88 m and +-1.14 m at z 1.20 and
1.90; the bottom is the limiting height, not the top. With stands spaced for a 0.5 m span (offsets up to
0.24 m) every one of the 19 strips is FULL (0.50..1.90 m) at 0.60 m standoff; the per-strip table is
printed by the second command. A deliberately bad layout (two stands at 0.25 m and 0.75 m) shows the
edge of the model: strips 0..10 FULL, strip 11 (offset 1.01 m) only 0.66..1.90, strips 12 to 18 (offset
1.17 m and more) unreachable.

Mount sensitivity (arm base height z0 with the lift retracted): with a 0.5 m span at 0.60 m standoff the
full 0.50..1.90 m stays covered for any z0 from 0 up to 1.54 m; at 0.80 m standoff up to 1.48 m. The
assumed 1.17 m therefore has 0.3 m or more of margin on the high side. If the lift's real mount height is
above about 1.5 m the top of the wall starts to fail from the lift side only when the lift is not at 0.
Stricter run (contact only, joint origins 0.15 m in front of the wall): the same 0.88 m / 0.78 m limits at
0.60 / 0.80 m.

## What this means for the mock-hardware failures

Recommended parameters for a 0.60..0.80 m standoff: `stand_max_span_m` up to 1.5 (keeping the 0.5 m that
was used is also fully inside the model's reach, with a lateral margin of at least 0.5 m), z limits the
full 0.50..1.90 m. **Nothing of the 3 x 2 m wall is unreachable from a 0.60..0.80 m standoff as long as the
stand is within 0.78 m laterally of every strip it serves (0.88 m at 0.60 m).**

So pure kinematic reach does not explain strokes ending "partially completed" at stand edges or at the top
of the reach, at 0.5 m spans. That is a finding, not a fix. The causes to check next are the things this
model leaves out: IK branch continuity along the stroke (not measured here; a branch change near the lateral
limit would break a stroke), the planner's orientation tolerance and the reachability-tilt tier (a contact tool must keep tilt
0), joint-limit margins in the controller, self-collision between the forearm and the lift column when the
arm folds for low strokes, and lift travel during a stroke. If the real stand sits farther from the wall
than assumed or the mount height differs, rerun with `--standoff` and `--arm-base-z0`. An RM4D map
(`rm4d_tool build`, then `column_reach_tool`) checks continuity and branch problems that this tool cannot.
