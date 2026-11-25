#!/usr/bin/env python3
"""Writes the data files of one named wall scenario for the MuJoCo manager passes.

    make_scenario_scene.py [--list] SHARE_DATA_DIR OUT_DIR NAME

Output in OUT_DIR: smoke_wall_scene.json (the wall the sim builds), wall_pass_coverage_scene.json (the wall the
coverage planner plans) and walls.pcd (the wall as MoveIt's octomap and the planner's observed mask), the stock
smoke-wall files moved and widened like shift_wall_scene.py does and then given the scenario's obstacles. The same
obstacle entries go into both JSON files, so the sim wall and the planned wall agree. Prints `KEY=value` lines
(MODE, DX, DY, WIDTH, WALL_X, WALL_Y0, WALL_Y1, OBSTACLES) for the runner script to eval.

Obstacles (wall_scene.json schema, `obstacles[]`; the extra key `kind` is ignored by the planner):
  * `box`: a protrusion standing `depth` off the wall (a socket, a junction box, a rib): an oriented box in the
    sim, a blocked region for the planner, a surface in the point cloud;
  * `opening`: a hole through the wall (a window): the sim wall is cut around it (wall_world.py), the planner
    avoids it, the point cloud has no points there.
A scenario may set `patch_z=(lo, hi)` (the contact patch's height, default PATCH_Z) and `wall_z=(lo, hi)` (the sim
wall's height, default the stock 0.7 m around the patch): the planned wall, the sim wall, walls.pcd and the
planning-scene curtain (walls_sim.pcd) follow. `--patch-z lo,hi` overrides the patch (the wall then spans
lo - 0.3 .. hi + 0.3 unless the scenario sets it). A patch that tall makes the lift telescope (the arm base rides
the lift: 1.17 m retracted, 2.67 m at the 1.5 m stroke).
Positions are given relative to the contact patch's centre: `dy` along the wall (+y), `dz` up, size `w` along y,
`h` along z, `depth` off the wall.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shift_wall_scene as sws  # noqa: E402

PATCH_Z = (1.0, 1.3)       # the contact patch's height (the manager's coverage_z range), centre 1.15
PCD_STEP = 0.005

# name -> wall placement and obstacles; `mode` is the pass that runs it: single (run_manager_pass.sh, one stand) or
# multi (run_multi_stand_pass.sh, the stand planner splits the wall)
SCENARIOS = {
    'plain': dict(mode='single', dx=0.0, dy=0.0, width=0.3, obstacles=[],
                  about='the stock smoke wall, 0.3 m wide, nothing on it'),
    'plain_wide': dict(mode='single', dx=0.0, dy=-0.1, width=0.5, obstacles=[],
                       about='0.5 m wide, nothing on it (the control for the two scenarios below)'),
    'window': dict(mode='single', dx=0.0, dy=-0.1, width=0.5,
                   obstacles=[dict(kind='opening', dy=0.0, dz=0.0, w=0.10, h=0.08)],
                   about='0.5 m wide, a 0.10 x 0.08 m window hole in the middle'),
    'protrusion': dict(mode='single', dx=0.0, dy=-0.1, width=0.5,
                       obstacles=[dict(kind='box', dy=-0.07, dz=0.04, w=0.07, h=0.07, depth=0.035)],
                       about='0.5 m wide, a 0.07 x 0.07 m box standing 35 mm off the wall (a socket / junction box)'),
    'two_panels': dict(mode='multi', dx=0.0, dy=0.3, width=0.9,
                       obstacles=[dict(kind='box', dy=0.0, dz=0.0, w=0.04, h=0.5, depth=0.03)],
                       about='0.9 m wide (two stands), a full-height 40 mm rib 30 mm proud of the wall between the panels'),
    # a wall 2 m tall: the arm base (1.17 m retracted) must ride the lift to reach z 0.6 and 2.0
    'two_panels_tall': dict(mode='multi', dx=0.0, dy=0.3, width=0.9, obstacles=[],
                            patch_z=(0.6, 2.0), wall_z=(0.3, 2.3),
                            about='0.9 m wide, wall z 0.3..2.3 m, patch z 0.6..2.0 m: the lift has to telescope'),
    'two_panels_plain': dict(mode='multi', dx=0.0, dy=0.3, width=0.9, obstacles=[],
                             about='0.9 m wide (two stands), no rib (the control for two_panels)'),
}


# ------------------------------------------------------------------ scene
def obstacle_entry(ident, spec, wall):
    """The wall_scene.json entry of one obstacle on `wall` (a scene wall dict, plane x = const facing -x)."""
    ox, oy, oz = wall['origin_xyz']
    cu = spec['dy']                    # basis_u = +y
    cv = -spec['dz']                   # basis_v = -z
    w, h = spec['w'], spec['h']
    depth = spec.get('depth', 0.0) if spec['kind'] == 'box' else 0.0
    u = (0.0, 1.0, 0.0)
    v = (0.0, 0.0, -1.0)
    n = (-1.0, 0.0, 0.0)
    cols = [u, v, n]
    # unit quaternion (x, y, z, w) of the rotation with columns u, v, n (the same math as wall_world)
    from sim_mujoco.wall_world import quat_from_matrix
    qw, qx, qy, qz = quat_from_matrix(cols)
    entry = {
        'id': ident,
        'parent_wall': wall['id'],
        'kind': spec['kind'],
        'box_center_xyz': [round(ox - depth / 2, 6), round(oy + cu, 6), round(oz - cv, 6)],
        'box_extents_uvn': [w, h, depth],
        'box_orientation_xyzw': [round(qx, 9), round(qy, 9), round(qz, 9), round(qw, 9)],
        'protrusion_min_m': depth,
        'protrusion_max_m': depth,
        'uv_on_wall': {'u': [round(cu - w / 2, 6), round(cu + w / 2, 6)],
                       'v': [round(cv - h / 2, 6), round(cv + h / 2, 6)]},
        'inflation_m': 0.0,
        'above_wall_scatter': True,
    }
    return entry


def add_obstacles(scene_path, specs):
    with open(scene_path) as f:
        scene = json.load(f)
    wall = scene['walls'][0]
    scene['obstacles'] = [obstacle_entry(i, s, wall) for i, s in enumerate(specs)]
    with open(scene_path, 'w') as f:
        json.dump(scene, f, indent=2)
    return scene['obstacles']


# ------------------------------------------------------------------ point cloud
def read_pcd(path):
    header, points, in_data = [], [], False
    with open(path) as f:
        for line in f:
            if in_data and line.strip():
                points.append(line.split())
            else:
                header.append(line)
                if line.startswith('DATA'):
                    in_data = True
    return header, points


def write_pcd(path, header, points):
    header = [f'WIDTH {len(points)}\n' if x.startswith('WIDTH') else
              f'POINTS {len(points)}\n' if x.startswith('POINTS') else x for x in header]
    with open(path, 'w') as f:
        f.writelines(header)
        f.writelines(' '.join(p) + '\n' for p in points)


def frange(lo, hi, step=PCD_STEP):
    n = max(1, int(round((hi - lo) / step)))
    return [lo + (hi - lo) * i / n for i in range(n + 1)]


def edit_pcd(src, dst, scene_obstacles, wall):
    """Cuts the openings out of the wall cloud and adds the boxes' surfaces (label 1, the planner only reads the
    wall's own label 0, MoveIt's octomap takes every point)."""
    header, points = read_pcd(src)
    wx = float(points[0][0])
    oy, oz = wall['origin_xyz'][1], wall['origin_xyz'][2]
    for ob in scene_obstacles:
        u0, u1 = ob['uv_on_wall']['u']
        v0, v1 = ob['uv_on_wall']['v']
        y0, y1, z0, z1 = oy + u0, oy + u1, oz - v1, oz - v0
        depth = ob['protrusion_max_m']
        if ob['kind'] == 'opening':
            points = [p for p in points if not (y0 <= float(p[1]) <= y1 and z0 <= float(p[2]) <= z1)]
            continue
        face = wx - depth
        for y in frange(y0, y1):
            for z in frange(z0, z1):
                points.append([f'{face:.4f}', f'{y:.4f}', f'{z:.4f}', '1'])
        for x in frange(face, wx)[:-1]:
            for t in frange(y0, y1):
                points += [[f'{x:.4f}', f'{t:.4f}', f'{z0:.4f}', '1'], [f'{x:.4f}', f'{t:.4f}', f'{z1:.4f}', '1']]
            for t in frange(z0, z1):
                points += [[f'{x:.4f}', f'{y0:.4f}', f'{t:.4f}', '1'], [f'{x:.4f}', f'{y1:.4f}', f'{t:.4f}', '1']]
    write_pcd(dst, header, points)


def obstacles_from_scene(path):
    """The obstacles of a wall scene (wall_scene.json, planner frame) as score_run wall-JSON entries: the wall's
    (y, z) rectangle of each obstacle and how far it stands off the wall. The wall must face along x (u = y)."""
    with open(path) as f:
        scene = json.load(f)
    walls = {w['id']: w for w in scene['walls']}
    out = []
    for ob in scene.get('obstacles', []):
        w = walls[ob['parent_wall']]
        o, bu, bv = w['origin_xyz'], w['basis_u'], w['basis_v']
        ys, zs = [], []
        for u in ob['uv_on_wall']['u']:
            for v in ob['uv_on_wall']['v']:
                ys.append(o[1] + u * bu[1] + v * bv[1])
                zs.append(o[2] + u * bu[2] + v * bv[2])
        out.append({'id': ob['id'], 'u': [round(min(ys), 6), round(max(ys), 6)],
                    'v': [round(min(zs), 6), round(max(zs), 6)], 'height_m': ob.get('protrusion_max_m', 0.0)})
    return out


# ------------------------------------------------------------------ tall walls
WALL_MARGIN_Z = 0.3     # wall above and below the patch when only the patch is given
CURTAIN_ABOVE_M = 0.3   # the planning-scene curtain stands this far above the wall's top (at least 2.6 m)


def patch_and_wall_z(sc, patch_z=None):
    """(patch_z, wall_z) of a scenario; wall_z is None for the stock wall (nothing to retarget)."""
    pz = tuple(patch_z) if patch_z else tuple(sc.get('patch_z', PATCH_Z))
    wz = sc.get('wall_z') if not patch_z else None
    if wz is None and pz != PATCH_Z:
        wz = (round(pz[0] - WALL_MARGIN_Z, 6), round(pz[1] + WALL_MARGIN_Z, 6))
    if pz[0] >= pz[1]:
        raise ValueError(f'patch z range {pz} is empty')
    if wz is not None and not (wz[0] <= pz[0] and pz[1] <= wz[1]):
        raise ValueError(f'wall z {wz} does not contain the patch z {pz}')
    return pz, (tuple(wz) if wz else None)


def retarget_scene_z(path, patch_z, wall_z, sim):
    """Moves the wall's origin to the patch centre height and sets v_range (basis_v = -z, v = origin z - z): the sim
    file gets the wall's extent, the planner file the patch's."""
    with open(path) as f:
        scene = json.load(f)
    w = scene['walls'][0]
    c = round((patch_z[0] + patch_z[1]) / 2, 6)
    w['origin_xyz'][2] = c
    lo, hi = wall_z if sim else patch_z
    w['v_range_m'] = [round(c - hi, 6), round(c - lo, 6)]
    if not sim and 'observed_area_m2' in w:
        u0, u1 = w['u_range_m']
        w['observed_area_m2'] = round((u1 - u0) * (patch_z[1] - patch_z[0]), 4)
    with open(path, 'w') as f:
        json.dump(scene, f, indent=2)


def retarget_pcd_z(path, patch_z, margin=0.05):
    """walls.pcd (the planner's observed mask, the contact patch with a 0.05 m rim) regenerated over the patch's z."""
    header, points = read_pcd(path)
    x = points[0][0]
    ys = sorted({float(p[1]) for p in points})
    zs = sorted({float(p[2]) for p in points})
    step = zs[1] - zs[0]
    cloud = [[x, f'{y:.4f}', f'{z:.4f}', '0'] for y in ys for z in frange(patch_z[0] - margin, patch_z[1] + margin, step)]
    write_pcd(path, header, cloud)


# ------------------------------------------------------------------ build
def build(src, out, name, patch_z=None):
    sc = SCENARIOS[name]
    pz, wz = patch_and_wall_z(sc, patch_z)
    os.makedirs(out, exist_ok=True)
    dx, dy, width = sc['dx'], sc['dy'], sc['width']
    sws.shift_scene(os.path.join(src, 'smoke_wall_scene.json'), os.path.join(out, 'smoke_wall_scene.json'), dx, dy, width)
    sws.shift_scene(os.path.join(src, 'wall_pass_coverage_scene.json'),
                    os.path.join(out, 'wall_pass_coverage_scene.json'), dx, dy, width)
    sws.shift_pcd(os.path.join(src, 'walls.pcd'), os.path.join(out, 'walls.pcd'), dx, dy, width)
    if wz is not None:
        retarget_scene_z(os.path.join(out, 'smoke_wall_scene.json'), pz, wz, sim=True)
        retarget_scene_z(os.path.join(out, 'wall_pass_coverage_scene.json'), pz, wz, sim=False)
        retarget_pcd_z(os.path.join(out, 'walls.pcd'), pz)
    obstacles = []
    for fn in ('smoke_wall_scene.json', 'wall_pass_coverage_scene.json'):
        obstacles = add_obstacles(os.path.join(out, fn), sc['obstacles'])
    with open(os.path.join(out, 'wall_pass_coverage_scene.json')) as f:
        wall = json.load(f)['walls'][0]
    if obstacles:
        edit_pcd(os.path.join(out, 'walls.pcd'), os.path.join(out, 'walls.pcd'), obstacles, wall)
    # MoveIt's octomap: the sim wall's whole extent (with the obstacles cut / added like walls.pcd)
    sws.sim_wall_pcd(os.path.join(out, 'smoke_wall_scene.json'), os.path.join(out, 'walls.pcd'),
                     os.path.join(out, 'walls_sim.pcd'),
                     curtain_top=max(sws.CURTAIN_TOP_M, (wz[1] + CURTAIN_ABOVE_M) if wz else 0.0))
    if obstacles:
        edit_pcd(os.path.join(out, 'walls_sim.pcd'), os.path.join(out, 'walls_sim.pcd'), obstacles, wall)
    return {'MODE': sc['mode'], 'DX': dx, 'DY': dy, 'WIDTH': width, 'WALL_X': round(0.98 + dx, 6),
            'WALL_Y0': dy, 'WALL_Y1': round(dy + width, 6), 'OBSTACLES': len(obstacles),
            'PATCH_Z_LO': pz[0], 'PATCH_Z_HI': pz[1], 'WALL_Z_LO': wz[0] if wz else 'default', 'WALL_Z_HI': wz[1] if wz else 'default'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true', help='print the scenario names and what they are')
    ap.add_argument('--patch-z', metavar='LO,HI', help='contact patch height in m, replacing the scenario\'s')
    ap.add_argument('src', nargs='?')
    ap.add_argument('out', nargs='?')
    ap.add_argument('name', nargs='?')
    a = ap.parse_args()
    if a.list:
        for k, v in SCENARIOS.items():
            print(f'{k:18s} {v["mode"]:7s} {v["about"]}')
        return
    if not (a.src and a.out and a.name):
        ap.error('SHARE_DATA_DIR OUT_DIR NAME are required')
    if a.name not in SCENARIOS:
        sys.exit(f'unknown scenario {a.name!r}; one of {", ".join(SCENARIOS)}')
    pz = None
    if a.patch_z:
        try:
            pz = tuple(float(v) for v in a.patch_z.split(','))
            assert len(pz) == 2
        except (ValueError, AssertionError):
            sys.exit(f'--patch-z wants LO,HI in metres, got {a.patch_z!r}')
    try:
        info = build(a.src, a.out, a.name, pz)
    except ValueError as e:
        sys.exit(str(e))
    for k, v in info.items():
        print(f'{k}={v}')


if __name__ == '__main__':
    main()
