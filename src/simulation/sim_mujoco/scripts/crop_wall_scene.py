#!/usr/bin/env python3
"""Crop a detected wall_scene.json to the contact region the arm can reach, for the coverage planner.

wall_detector reports the whole sensed face of the wall; the manager pass only reaches y 0..0.3, z 1.0..1.3 in
base_footprint (the manager's coverage_z_min/max and the arm's reach). The crop is the box
[y_min, y_max] x [z_min, z_max] (frame of the scene) projected into the detected plane's (u, v) coordinates and
intersected with the detected extents, so a patch the sensor did not see is never planned. Everything else
(plane, flatness figures, obstacles) is copied from the detection, and walls.pcd is not touched.

  crop_wall_scene.py wall_scene.json out.json --y 0 0.3 --z 1.0 1.3
"""
import argparse
import json
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scene')
    ap.add_argument('out')
    ap.add_argument('--y', type=float, nargs=2, default=[0.0, 0.3])
    ap.add_argument('--z', type=float, nargs=2, default=[1.0, 1.3])
    ap.add_argument('--wall-id', type=int, default=0)
    args = ap.parse_args()
    with open(args.scene) as f:
        scene = json.load(f)
    wall = next(w for w in scene['walls'] if w['id'] == args.wall_id)
    origin = np.array(wall['origin_xyz'])
    u, v, n = (np.array(wall[k]) for k in ('basis_u', 'basis_v', 'normal_outward'))
    # the box's corners on the plane (move each along the normal onto the plane: x from the plane equation)
    a, b, c, d = wall['plane_abcd']
    us, vs = [], []
    for y in args.y:
        for z in args.z:
            if abs(a) < 1e-6:
                sys.exit('the wall plane is parallel to the crop box axis x; cannot crop')
            p = np.array([-(b * y + c * z + d) / a, y, z])
            us.append(float((p - origin) @ u))
            vs.append(float((p - origin) @ v))
    u_lo, u_hi = max(min(us), wall['u_range_m'][0]), min(max(us), wall['u_range_m'][1])
    v_lo, v_hi = max(min(vs), wall['v_range_m'][0]), min(max(vs), wall['v_range_m'][1])
    if u_hi <= u_lo or v_hi <= v_lo:
        sys.exit(f'the crop y {args.y} z {args.z} does not overlap the detected wall '
                 f'(u {wall["u_range_m"]}, v {wall["v_range_m"]})')
    wall['u_range_m'] = [u_lo, u_hi]
    wall['v_range_m'] = [v_lo, v_hi]
    scene['note'] = (scene.get('note', '') + f' Cropped to y {args.y} z {args.z} for the manager pass.').strip()
    with open(args.out, 'w') as f:
        json.dump(scene, f, indent=2)
    print(f'crop: u [{u_lo:.3f}, {u_hi:.3f}] v [{v_lo:.3f}, {v_hi:.3f}] ({(u_hi - u_lo) * (v_hi - v_lo):.3f} m^2)')


if __name__ == '__main__':
    main()
