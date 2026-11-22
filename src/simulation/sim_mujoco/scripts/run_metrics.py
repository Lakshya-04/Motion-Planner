#!/usr/bin/env python3
"""Prints the lift range, chassis-to-wall clearance and tool reach fraction of a multi-stand run directory.

    run_metrics.py RUN_DIR [--profile PROFILE.yaml] [--stroke M] [--arm-mount-z M] [--reach M] [--json]

Reads RUN_DIR/manager.log and the wall plane from RUN_DIR/data/wall_pass_coverage_scene.json (clearance is skipped
without it). Numbers default to ur10e_telescopic (stroke 1.5, arm mount 1.17, reach 1.3, footprint +-0.45 x +-0.30);
--profile takes the stroke, arm mount height, footprint and arm mount xy from a robot profile (needs PyYAML).
Exit code 0 always (a report, not a gate); flags are printed as FLAG lines.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from sim_mujoco import run_metrics as rm  # noqa: E402


def profile_config(path):
    import yaml
    with open(path) as f:
        p = yaml.safe_load(f)
    base, lift = p.get('base', {}), p.get('lift', {})
    cfg = dict(stroke_m=lift.get('stroke_m'))
    if 'lift_mount_height_m' in base and 'mount_height_m' in lift:
        cfg['arm_mount_z_m'] = float(base['lift_mount_height_m']) + float(lift['mount_height_m'])
    if base.get('footprint'):
        cfg['footprint'] = tuple(tuple(pt) for pt in base['footprint'])
    if base.get('arm_mount_xy'):
        cfg['arm_mount_xy'] = tuple(base['arm_mount_xy'])
    return cfg


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--profile')
    ap.add_argument('--stroke', type=float)
    ap.add_argument('--arm-mount-z', type=float)
    ap.add_argument('--reach', type=float)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    log = os.path.join(a.run_dir, 'manager.log')
    if not os.path.isfile(log):
        print(f'run_metrics: no {log}', file=sys.stderr)
        return 2
    cfg = profile_config(a.profile) if a.profile else {}
    for k, v in (('stroke_m', a.stroke), ('arm_mount_z_m', a.arm_mount_z), ('reach_m', a.reach)):
        if v is not None:
            cfg[k] = v
    scene = os.path.join(a.run_dir, 'data', 'wall_pass_coverage_scene.json')
    plane = rm.wall_plane_from_scene(scene) if os.path.isfile(scene) else None
    with open(log, errors='replace') as f:
        res = rm.evaluate(f, plane, **cfg)
    if a.json:
        print(json.dumps(res, indent=2))
    else:
        print('\n'.join(rm.format_table(res)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
