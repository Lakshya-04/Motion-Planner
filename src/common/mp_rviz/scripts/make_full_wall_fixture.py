#!/usr/bin/env python3
"""Write a complete wall_scene.json plus a dense synthetic walls.pcd for a full-size wall, so that
``ros2 launch coverage_planner wall_coverage_demo.launch.py wall_scene_json:=<out>/wall_scene.json tool_name:=...``
plans the WHOLE wall offline (no simulator).

The coverage_planner loader reads ``walls.pcd`` from the directory of the scene JSON (no launch argument for the path),
as pcl PointXYZL: fields x y z label (label = wall id, uint32). This tool writes an ASCII PCD with those fields.

usage: make_full_wall_fixture.py OUT_DIR [--width 3.0] [--height 2.0] [--plane-x 2.0] [--step 0.02] [--frame map]
Default wall = the Gazebo multi-stand wall: plane x=2, u (y) -1.5..1.5, v (down from z=2) 0..2.
"""
import argparse
import json
import os


def build_scene(width=3.0, height=2.0, plane_x=2.0, frame='map', n_points=0):
    half = width / 2.0
    wall = {
        'id': 0,
        'plane_abcd': [-1.0, 0.0, 0.0, plane_x],
        'origin_xyz': [plane_x, 0.0, height],
        'basis_u': [0.0, 1.0, 0.0],
        'basis_v': [0.0, 0.0, -1.0],
        'normal_outward': [-1.0, 0.0, 0.0],
        'u_range_m': [-half, half],
        'v_range_m': [0.0, height],
        'observed_area_m2': width * height,
        'observed_fraction': 1.0,
        'plane_rms_mm': 1.0,
        'spread_p95_mm': 2.0,
        'spread_p99_mm': 3.0,
        'points': n_points,
        'merged_layers': 1,
        'layer_offset_spread_mm': 0.0,
    }
    return {'frame_id': frame, 'note': 'synthetic full wall (make_full_wall_fixture.py)', 'walls': [wall],
            'obstacles': [], 'worst_wall_spread_p99_m': 0.003}


def grid_points(width, height, plane_x, step):
    """(x, y, z) grid over the wall, cell centres; y runs -width/2..width/2, z runs 0..height."""
    nu = max(1, int(round(width / step)))
    nv = max(1, int(round(height / step)))
    pts = []
    for j in range(nv):
        z = height - (j + 0.5) * height / nv
        for i in range(nu):
            y = -width / 2.0 + (i + 0.5) * width / nu
            pts.append((plane_x, y, z))
    return pts


def write_pcd(path, pts, label=0):
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z label\nSIZE 4 4 4 4\n'
                'TYPE F F F U\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'
                % (len(pts), len(pts)))
        for x, y, z in pts:
            f.write('%.4f %.4f %.4f %d\n' % (x, y, z, label))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out_dir')
    ap.add_argument('--width', type=float, default=3.0)
    ap.add_argument('--height', type=float, default=2.0)
    ap.add_argument('--plane-x', type=float, default=2.0)
    ap.add_argument('--step', type=float, default=0.02)
    ap.add_argument('--frame', default='map')
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    pts = grid_points(a.width, a.height, a.plane_x, a.step)
    write_pcd(os.path.join(a.out_dir, 'walls.pcd'), pts)
    with open(os.path.join(a.out_dir, 'wall_scene.json'), 'w') as f:
        json.dump(build_scene(a.width, a.height, a.plane_x, a.frame, len(pts)), f, indent=1)
    print('wrote %s/{wall_scene.json,walls.pcd}: %.1f x %.1f m wall, %d points' % (a.out_dir, a.width, a.height, len(pts)))


if __name__ == '__main__':
    main()
