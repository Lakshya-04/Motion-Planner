#!/usr/bin/env python3
"""Writes copies of the smoke wall's three data files (sim scene, coverage scene, walls.pcd) with the wall moved by
(dx, dy) in the world frame, so a stand test can put the wall where the planned stand is far from the base's start.

    shift_wall_scene.py SHARE_DATA_DIR OUT_DIR DX DY [WIDTH [LO,HI]]

Planes are `a x + b y + c z + d = 0` (plane_abcd), so shifting by (dx, dy) changes d by -(a dx + b dy); origins and the
point cloud move by the offset.

With WIDTH (m, along y; default 0.3 = the stock patch) the contact patch is made that wide, starting at y = DY: the
coverage scene's u extent, the sim wall (the stock 0.25 m of margin each side of the patch) and the walls.pcd grid grow
with it. LO,HI (m) moves the patch to that height (coverage scene, walls.pcd) and makes the sim wall reach
HI + 0.3 m when that is above the stock 2 m (unverified in Gazebo). A wall wider than one stand's reach is what the multi-stand pass needs.

Also writes walls_scene.pcd (sim_wall_scene_pcd): the sim wall's whole face, for MoveIt's planning scene only
(run_multi_stand_pass.sh, MULTI_STAND_FULL_WALL_SCENE); walls.pcd stays the patch for coverage, reachability and scoring.
"""
import json
import os
import sys

PATCH_WIDTH = 0.3   # the stock patch's width (y 0..0.3)
SCENE_STEP_M = 0.01  # walls_scene.pcd grid step = MoveIt's octomap resolution


def shift_scene(path, out, dx, dy, width=PATCH_WIDTH):
    with open(path) as f:
        scene = json.load(f)
    grow = (width - PATCH_WIDTH) / 2.0   # per side; the wall's centre moves by the same amount
    for w in scene['walls']:
        a, b, c, d = w['plane_abcd']
        w['plane_abcd'] = [a, b, c, d - (a * dx + b * dy)]
        w['origin_xyz'] = [w['origin_xyz'][0] + dx, w['origin_xyz'][1] + dy + grow, w['origin_xyz'][2]]
        if grow:
            u0, u1 = w['u_range_m']
            w['u_range_m'] = [round(u0 - grow, 6), round(u1 + grow, 6)]
            if 'observed_area_m2' in w:
                w['observed_area_m2'] = round(w['observed_area_m2'] * width / PATCH_WIDTH, 4)
    with open(out, 'w') as f:
        json.dump(scene, f, indent=2)


def shift_pcd(path, out, dx, dy, width=PATCH_WIDTH):
    header, points, in_data = [], [], False
    with open(path) as f:
        for line in f:
            if in_data and line.strip():
                v = line.split()
                v[0], v[1] = f'{float(v[0]) + dx:.4f}', f'{float(v[1]) + dy:.4f}'
                points.append(v)
            else:
                header.append(line)
                if line.startswith('DATA'):
                    in_data = True
    if width != PATCH_WIDTH and points:
        # repeat the grid along y (the file's own y step), so it spans y = DY - 0.05 .. DY + width + 0.05
        ys = sorted({float(v[1]) for v in points})
        step = ys[1] - ys[0]
        n = round((width + 0.1) / step) + 1
        column = [v for v in points if abs(float(v[1]) - ys[0]) < 1e-6]
        points = [[v[0], f'{ys[0] + i * step:.4f}', v[2], v[3]] for i in range(n) for v in column]
    header = [f'WIDTH {len(points)}\n' if x.startswith('WIDTH') else
              f'POINTS {len(points)}\n' if x.startswith('POINTS') else x for x in header]
    with open(out, 'w') as f:
        f.writelines(header)
        f.writelines(' '.join(v) + '\n' for v in points)


WALL_MARGIN_Z = 0.3   # sim wall above the patch's top, as in the MuJoCo tall wall (it also grows 0.3 m below, here capped at the floor)
STOCK_WALL_TOP = 2.0  # the stock Gazebo wall spans z 0..2 (v_range 0..2, the world generator puts its bottom on the floor)


def parse_patch_z(text):
    lo, hi = (float(v) for v in text.split(','))
    if not 0.0 <= lo < hi:
        raise SystemExit(f'patch z {text!r}: need 0 <= lo < hi')
    return lo, hi


def retarget_z(out, lo, hi, margin=0.05):
    """Moves the contact patch to z lo..hi in the three files of `out` (shift_wall_scene already wrote them): the
    coverage scene and walls.pcd get the patch (the pcd with a 0.05 m rim), the sim wall is made tall enough to
    hold it with WALL_MARGIN_Z above (never shorter than the stock 2 m). basis_v = -z, so v = origin z - z."""
    c = round((lo + hi) / 2, 6)
    top = max(STOCK_WALL_TOP, round(hi + WALL_MARGIN_Z, 6))
    cov = os.path.join(out, 'wall_pass_coverage_scene.json')
    sim = os.path.join(out, 'smoke_wall_scene.json')
    with open(cov) as f:
        scene = json.load(f)
    w = scene['walls'][0]
    w['origin_xyz'][2] = c
    w['v_range_m'] = [round(c - hi, 6), round(c - lo, 6)]
    if 'observed_area_m2' in w:
        u0, u1 = w['u_range_m']
        w['observed_area_m2'] = round((u1 - u0) * (hi - lo), 4)
    with open(cov, 'w') as f:
        json.dump(scene, f, indent=2)
    with open(sim) as f:
        scene = json.load(f)
    w = scene['walls'][0]
    w['origin_xyz'][2] = top
    w['v_range_m'] = [0.0, top]
    with open(sim, 'w') as f:
        json.dump(scene, f, indent=2)
    pcd = os.path.join(out, 'walls.pcd')
    header, points = [], []
    in_data = False
    with open(pcd) as f:
        for line in f:
            if in_data and line.strip():
                points.append(line.split())
            else:
                header.append(line)
                in_data = in_data or line.startswith('DATA')
    ys = sorted({v[1] for v in points})
    zs = sorted({float(v[2]) for v in points})
    step = zs[1] - zs[0]
    n = round((hi - lo + 2 * margin) / step) + 1
    cloud = [[points[0][0], y, f'{lo - margin + k * step:.4f}', '0'] for y in ys for k in range(n)]
    header = [f'WIDTH {len(cloud)}\n' if x.startswith('WIDTH') else f'POINTS {len(cloud)}\n' if x.startswith('POINTS')
              else x for x in header]
    with open(pcd, 'w') as f:
        f.writelines(header)
        f.writelines(' '.join(v) + '\n' for v in cloud)


def sim_wall_scene_pcd(scene_path, pcd_in, pcd_out, step=SCENE_STEP_M):
    """walls_scene.pcd: the Gazebo world wall's WHOLE face (smoke_wall_scene.json u/v extent, the sim wall as generated,
    including a shift, a width and a tall patch) as one plane of points at walls.pcd's x (the tool's contact plane, 0.98),
    for MoveIt's planning scene only. walls.pcd is the contact patch, so OMPL free-space plans swung the tool through wall
    regions MoveIt did not know (4 to 19 kN spikes at the first approach of each stand). Coverage, reachability and
    scoring keep reading walls.pcd. `step` defaults to the octomap's 0.01 m resolution: a coarser grid leaves gaps."""
    with open(scene_path) as f:
        w = json.load(f)['walls'][0]
    x = None
    with open(pcd_in) as f:
        in_data = False
        for line in f:
            if in_data and line.strip():
                x = line.split()[0]
                break
            in_data = in_data or line.startswith('DATA')
    if x is None:
        raise SystemExit(f'{pcd_in}: no points')
    oy, oz = w['origin_xyz'][1], w['origin_xyz'][2]
    u0, u1 = w['u_range_m']
    v0, v1 = w['v_range_m']
    y_lo, y_hi, z_lo, z_hi = oy + u0, oy + u1, oz - v1, oz - v0   # basis_u = +y, basis_v = -z
    ny, nz = round((y_hi - y_lo) / step), round((z_hi - z_lo) / step)
    cloud = [(x, f'{y_lo + i * step:.4f}', f'{z_lo + j * step:.4f}', '0') for i in range(ny + 1) for j in range(nz + 1)]
    with open(pcd_out, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z label\nSIZE 4 4 4 4\nTYPE F F F U\n'
                f'COUNT 1 1 1 1\nWIDTH {len(cloud)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(cloud)}\nDATA ascii\n')
        f.writelines(' '.join(v) + '\n' for v in cloud)


def main():
    src, dst, dx, dy = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    width = float(sys.argv[5]) if len(sys.argv) > 5 else PATCH_WIDTH
    os.makedirs(dst, exist_ok=True)
    shift_scene(os.path.join(src, 'smoke_wall_scene.json'), os.path.join(dst, 'smoke_wall_scene.json'), dx, dy, width)
    shift_scene(os.path.join(src, 'wall_pass_coverage_scene.json'), os.path.join(dst, 'wall_pass_coverage_scene.json'),
                dx, dy, width)
    shift_pcd(os.path.join(src, 'walls.pcd'), os.path.join(dst, 'walls.pcd'), dx, dy, width)
    if len(sys.argv) > 6 and sys.argv[6]:
        lo, hi = parse_patch_z(sys.argv[6])
        retarget_z(dst, lo, hi)
    sim_wall_scene_pcd(os.path.join(dst, 'smoke_wall_scene.json'), os.path.join(dst, 'walls.pcd'),
                       os.path.join(dst, 'walls_scene.pcd'))


if __name__ == '__main__':
    main()
