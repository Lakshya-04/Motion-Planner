#!/usr/bin/env python3
"""Writes copies of the smoke wall's three data files (sim scene, coverage scene, walls.pcd) with the wall moved by
(dx, dy) in the world frame, so a stand test can put the wall where the planned stand is far from the base's start.

    shift_wall_scene.py SHARE_DATA_DIR OUT_DIR DX DY [WIDTH [DZ]]

Planes are `a x + b y + c z + d = 0` (plane_abcd), so shifting by (dx, dy) changes d by -(a dx + b dy); origins and the
point cloud move by the offset.

With WIDTH (m, along y; default 0.3 = the stock patch) the contact patch is made that wide, starting at y = DY: the
coverage scene's u extent, the sim wall (the stock 0.25 m of margin each side of the patch) and the walls.pcd grid grow
with it. A wall wider than one stand's reach is what the multi-stand pass needs.

With DZ (m, default 0) the wall is also moved in z (a robot profile whose arm works at another height than the UR10e's
1.0..1.3 m, profile_reach.py); the curtain of walls_sim.pcd keeps its z 0..2.6 m.
"""
import json
import os
import sys

PATCH_WIDTH = 0.3   # the stock patch's width (y 0..0.3)


def shift_scene(path, out, dx, dy, width=PATCH_WIDTH, dz=0.0):
    with open(path) as f:
        scene = json.load(f)
    grow = (width - PATCH_WIDTH) / 2.0   # per side; the wall's centre moves by the same amount
    for w in scene['walls']:
        a, b, c, d = w['plane_abcd']
        w['plane_abcd'] = [a, b, c, d - (a * dx + b * dy + c * dz)]
        w['origin_xyz'] = [w['origin_xyz'][0] + dx, w['origin_xyz'][1] + dy + grow, w['origin_xyz'][2] + dz]
        if grow:
            u0, u1 = w['u_range_m']
            w['u_range_m'] = [round(u0 - grow, 6), round(u1 + grow, 6)]
            if 'observed_area_m2' in w:
                w['observed_area_m2'] = round(w['observed_area_m2'] * width / PATCH_WIDTH, 4)
    with open(out, 'w') as f:
        json.dump(scene, f, indent=2)


def shift_pcd(path, out, dx, dy, width=PATCH_WIDTH, dz=0.0):
    header, points, in_data = [], [], False
    with open(path) as f:
        for line in f:
            if in_data and line.strip():
                v = line.split()
                v[0], v[1], v[2] = f'{float(v[0]) + dx:.4f}', f'{float(v[1]) + dy:.4f}', f'{float(v[2]) + dz:.4f}'
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


def sim_wall_pcd(scene_path, pcd_in, pcd_out, curtain=True):
    """walls.pcd stretched to the sim wall's whole extent (smoke_wall_scene.json, with its 0.25 m margin around the
    contact patch), and with `curtain` on to the whole plane behind which the arm has no business (y 0.5 m beyond
    the wall on each side, z 0 to 2.6 m): the cloud MoveIt's octomap is made of. walls.pcd covers the patch only, so
    MoveIt let the arm swing through the part of the physical wall it did not know, and over the top of the wall: the
    approach path from the stowed pose crosses the wall plane at z 1.55 and the tool came to rest on the wall's top
    edge (tcp z 1.53, x 1.0 to 1.14, 100 to 170 N), in about one run in four of the multi-stand pass. Same grid step
    and x as walls.pcd, label 0. No clearance or collision margin changes: the space behind the wall is only declared
    occupied."""
    with open(scene_path) as f:
        w = json.load(f)['walls'][0]
    header, points, in_data = [], [], False
    with open(pcd_in) as f:
        for line in f:
            if in_data and line.strip():
                points.append(line.split())
            else:
                header.append(line)
                if line.startswith('DATA'):
                    in_data = True
    ys = sorted({float(v[1]) for v in points})
    zs = sorted({float(v[2]) for v in points})
    step = max(min(ys[1] - ys[0], zs[1] - zs[0]), 0.01)   # the octomap's resolution is 0.01
    x = points[0][0]
    oy, oz = w['origin_xyz'][1], w['origin_xyz'][2]
    u0, u1 = w['u_range_m']
    v0, v1 = w['v_range_m']
    y_lo, y_hi, z_lo, z_hi = oy + u0, oy + u1, oz - v1, oz - v0   # basis_v is -z
    if curtain:
        y_lo, y_hi, z_lo, z_hi = y_lo - 0.5, y_hi + 0.5, 0.0, 2.6
    ny, nz = round((y_hi - y_lo) / step), round((z_hi - z_lo) / step)
    cloud = [[x, f'{y_lo + i * step:.4f}', f'{z_lo + j * step:.4f}', '0'] for i in range(ny + 1) for j in range(nz + 1)]
    header = [f'WIDTH {len(cloud)}\n' if h.startswith('WIDTH') else
              f'POINTS {len(cloud)}\n' if h.startswith('POINTS') else h for h in header]
    with open(pcd_out, 'w') as f:
        f.writelines(header)
        f.writelines(' '.join(v) + '\n' for v in cloud)


def main():
    src, dst, dx, dy = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    width = float(sys.argv[5]) if len(sys.argv) > 5 else PATCH_WIDTH
    dz = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    os.makedirs(dst, exist_ok=True)
    shift_scene(os.path.join(src, 'smoke_wall_scene.json'), os.path.join(dst, 'smoke_wall_scene.json'), dx, dy, width, dz)
    shift_scene(os.path.join(src, 'wall_pass_coverage_scene.json'), os.path.join(dst, 'wall_pass_coverage_scene.json'),
                dx, dy, width, dz)
    shift_pcd(os.path.join(src, 'walls.pcd'), os.path.join(dst, 'walls.pcd'), dx, dy, width, dz)
    sim_wall_pcd(os.path.join(dst, 'smoke_wall_scene.json'), os.path.join(dst, 'walls.pcd'),
                 os.path.join(dst, 'walls_sim.pcd'))


if __name__ == '__main__':
    main()
