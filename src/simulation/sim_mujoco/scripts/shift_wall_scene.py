#!/usr/bin/env python3
"""Writes copies of the smoke wall's three data files (sim scene, coverage scene, walls.pcd) with the wall moved by
(dx, dy) in the world frame, so a stand test can put the wall where the planned stand is far from the base's start.

    shift_wall_scene.py SHARE_DATA_DIR OUT_DIR DX DY

Planes are `a x + b y + c z + d = 0` (plane_abcd), so shifting by (dx, dy) changes d by -(a dx + b dy); origins and the
point cloud move by the offset.
"""
import json
import os
import sys


def shift_scene(path, out, dx, dy):
    with open(path) as f:
        scene = json.load(f)
    for w in scene['walls']:
        a, b, c, d = w['plane_abcd']
        w['plane_abcd'] = [a, b, c, d - (a * dx + b * dy)]
        w['origin_xyz'] = [w['origin_xyz'][0] + dx, w['origin_xyz'][1] + dy, w['origin_xyz'][2]]
    with open(out, 'w') as f:
        json.dump(scene, f, indent=2)


def shift_pcd(path, out, dx, dy):
    lines, in_data = [], False
    with open(path) as f:
        for line in f:
            if in_data and line.strip():
                v = line.split()
                v[0], v[1] = f'{float(v[0]) + dx:.4f}', f'{float(v[1]) + dy:.4f}'
                line = ' '.join(v) + '\n'
            elif line.startswith('DATA'):
                in_data = True
            lines.append(line)
    with open(out, 'w') as f:
        f.writelines(lines)


def main():
    src, dst, dx, dy = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    os.makedirs(dst, exist_ok=True)
    shift_scene(os.path.join(src, 'smoke_wall_scene.json'), os.path.join(dst, 'smoke_wall_scene.json'), dx, dy)
    shift_scene(os.path.join(src, 'wall_pass_coverage_scene.json'), os.path.join(dst, 'wall_pass_coverage_scene.json'),
                dx, dy)
    shift_pcd(os.path.join(src, 'walls.pcd'), os.path.join(dst, 'walls.pcd'), dx, dy)


if __name__ == '__main__':
    main()
