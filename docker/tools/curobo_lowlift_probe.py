#!/usr/bin/env python3
"""Low-lift docked-to-stow probe for a 7-joint cuRobo config (docs/curobo_v2_notes.md, "Low-lift docked-to-stow no_path").

Runs inside motion-planner:gpu (needs a CUDA device and a lift_arm.yml from generate_curobo_7dof.sh):

    python curobo_lowlift_probe.py --robot DIR/lift_arm.yml [--poses curobo_lowlift_poses.json] [--plan-limit 4]
                                   [--max-attempts 6] [--out result.json]

For every logged docked arm pose, at the lift as logged and at lift 0.0 / 0.01 / 0.05 / 0.2 / 0.4:
  * the planner's own state check (check_states = joint limits + self-collision, empty world) says valid or not;
  * for an invalid state the offending sphere pairs are listed with a numpy re-computation (FK of the URDF named in the
    yml, spheres and ignore table of the yml; same overlap rule as cuRobo without its activation distance);
Then, for a few poses at lift 0.01: plan_joint docked -> stow with the endpoint precheck (answers start_in_collision),
and with precheck off (a real plan, `--max-attempts`), and hypothesis 3: lift-only retract to 0.4 (check_motion and a plan),
then stow from there.
"""
import argparse
import itertools
import json
import pathlib
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'src/manipulation/curobo_bridge'))
sys.path.insert(0, '/repo/src/manipulation/curobo_bridge')


def find_key(node, key):
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for v in node.values():
            r = find_key(v, key)
            if r is not None:
                return r
    return None


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Pairs:
    """Sphere-pair self-collision from the yml: FK of the URDF (revolute, prismatic, mimic), yml spheres + ignore table."""

    def __init__(self, yml_path):
        doc = yaml.safe_load(open(yml_path))
        kin = doc['robot_cfg']['kinematics'] if 'robot_cfg' in doc else find_key(doc, 'kinematics')
        base = pathlib.Path(yml_path).parent

        def load(v):
            if isinstance(v, str):
                p = pathlib.Path(v)
                p = p if p.is_absolute() else base / p
                return yaml.safe_load(open(p))
            return v
        spheres = load(find_key(kin, 'collision_spheres'))
        self.spheres = {l: [(np.array(s['center'], float), float(s['radius'])) for s in v] for l, v in spheres.items()}
        self.buffer = float(kin.get('collision_sphere_buffer', 0.0) or 0.0)
        ign = find_key(kin, 'self_collision_ignore') or {}
        self.ignore = {frozenset((a, b)) for a, bs in ign.items() for b in bs}
        self.names = find_key(kin, 'joint_names') or find_key(doc, 'joint_names')
        urdf = pathlib.Path(find_key(kin, 'urdf_path'))
        root = ET.parse(urdf if urdf.is_absolute() else base / urdf).getroot()
        self.joints = root.findall('joint')
        self.by_child = {j.find('child').get('link'): j for j in self.joints}

    def link_poses(self, q):
        val = dict(zip(self.names, q))
        for j in self.joints:       # mimic joints follow their master
            m = j.find('mimic')
            if m is not None and m.get('joint') in val:
                val[j.get('name')] = float(m.get('multiplier', 1)) * val[m.get('joint')] + float(m.get('offset', 0))
        cache = {}

        def pose(link):
            if link in cache:
                return cache[link]
            j = self.by_child.get(link)
            if j is None:
                cache[link] = np.eye(4)
                return cache[link]
            o = j.find('origin')
            t = np.eye(4)
            if o is not None:
                t[:3, :3] = rpy(*map(float, o.get('rpy', '0 0 0').split()))
                t[:3, 3] = list(map(float, o.get('xyz', '0 0 0').split()))
            p = pose(j.find('parent').get('link')) @ t
            v = val.get(j.get('name'), 0.0)
            typ = j.get('type')
            if typ in ('revolute', 'continuous', 'prismatic'):
                a = np.array(list(map(float, j.find('axis').get('xyz').split())))
                a /= np.linalg.norm(a)
                mv = np.eye(4)
                if typ == 'prismatic':
                    mv[:3, 3] = a * v
                else:
                    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                    mv[:3, :3] = np.eye(3) + np.sin(v) * k + (1 - np.cos(v)) * k @ k
                p = p @ mv
            cache[link] = p
            return p
        return pose

    def colliding(self, q):
        pose = self.link_poses(q)
        pts = {l: [((pose(l) @ np.append(c, 1))[:3], r + self.buffer) for c, r in s] for l, s in self.spheres.items()}
        out = []
        for a, b in itertools.combinations(sorted(pts), 2):
            if frozenset((a, b)) in self.ignore:
                continue
            worst = max(((ra + rb) - np.linalg.norm(p - p2) for (p, ra), (p2, rb) in itertools.product(pts[a], pts[b])),
                        default=-1)
            if worst > 0:
                out.append((a, b, round(float(worst), 4)))
        return out


def timed(fn, *a, **k):
    t = time.time()
    r = fn(*a, **k)
    return r, time.time() - t


def brief(r):
    return {'success': bool(r.success), 'code': r.failure_reason_code, 'stage': r.failure_stage,
            'route': getattr(r, 'route', None), 'message': r.message}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', required=True)
    ap.add_argument('--poses', default=str(HERE / 'curobo_lowlift_poses.json'))
    ap.add_argument('--plan-limit', type=int, default=4, help='how many docked poses get a plan_joint run')
    ap.add_argument('--max-attempts', type=int, default=6)
    ap.add_argument('--lifts', default='0.0,0.01,0.05,0.2,0.4')
    ap.add_argument('--out')
    ap.add_argument('--pairs-only', action='store_true', help='skip the GPU: only the numpy pair analysis')
    args = ap.parse_args()

    poses = json.load(open(args.poses))
    stow, docked = poses['stow'], poses['docked']
    lifts = [float(x) for x in args.lifts.split(',')]
    pairs = Pairs(args.robot)
    res = {'robot': args.robot, 'joint_names': pairs.names, 'ignore_pairs': len(pairs.ignore) // 1, 'states': [], 'plans': []}
    print('robot', args.robot, 'joints', pairs.names)

    planner = None
    if not args.pairs_only:
        from curobo_bridge.planner import CuroboPlanner
        planner = CuroboPlanner(robot=args.robot, max_boxes=16, prismatic_joints=['lift_joint'],
                                max_attempts=args.max_attempts, precheck_endpoints=True)
        print('stow valid:', planner.check_states([stow]))

    rows = []
    for name, q in docked.items():
        for lift in [None] + lifts:
            s = list(q) if lift is None else [lift] + list(q[1:])
            rows.append((name, 'as_logged' if lift is None else lift, s))
    valid = planner.check_states([r[2] for r in rows]) if planner else [None] * len(rows)
    for (name, lift, s), ok in zip(rows, valid):
        pr = pairs.colliding(s)
        rec = {'pose': name, 'lift': lift, 'q': s, 'cuRobo_valid': ok, 'numpy_pairs': pr}
        res['states'].append(rec)
        print(f'{name:28s} lift={lift!s:9s} curobo_valid={ok!s:5s} numpy_pairs={pr if pr else "-"}')

    if planner:
        # plans: first the poses that are invalid at lift 0.01 (the hypothesis), then valid ones
        by = {(r['pose'], r['lift']): r for r in res['states']}
        order = sorted(docked, key=lambda n: by[(n, 0.01)]['cuRobo_valid'] if (n, 0.01) in by else True)
        for name in order[:args.plan_limit]:
            q0 = [0.01] + list(docked[name][1:])
            rec = {'pose': name, 'start': q0}
            planner._precheck_endpoints = True
            r, dt = timed(planner.plan_joint, q0, stow)
            rec['with_precheck'] = {**brief(r), 'seconds': round(dt, 2)}
            if rec['with_precheck']['code'] != 'start_in_collision':
                rec['without_precheck'] = rec['with_precheck']
            else:
                planner._precheck_endpoints = False
                r, dt = timed(planner.plan_joint, q0, stow)
                rec['without_precheck'] = {**brief(r), 'seconds': round(dt, 2)}
                planner._precheck_endpoints = True
            # hypothesis 3: lift-only retract with the arm swung out, then stow from there
            mid = [0.4] + list(docked[name][1:])
            rec['h3_mid_valid'] = planner.check_states([mid])[0]
            rec['h3_lift_only_check_motion'] = bool(planner.check_motion([q0, mid]))
            r, dt = timed(planner.plan_joint, q0, mid)
            rec['h3_plan_to_mid'] = {**brief(r), 'seconds': round(dt, 2)}
            r, dt = timed(planner.plan_joint, mid, stow)
            rec['h3_plan_mid_to_stow'] = {**brief(r), 'seconds': round(dt, 2)}
            # hypothesis 3 variant: fold the arm (stow arm joints) at lift 0.01, then raise the lift
            fold = [0.01] + list(stow[1:])
            rec['h3_fold_low_valid'] = planner.check_states([fold])[0]
            r, dt = timed(planner.plan_joint, q0, fold)
            rec['h3_plan_to_fold_low'] = {**brief(r), 'seconds': round(dt, 2)}
            res['plans'].append(rec)
            print(json.dumps(rec, indent=1))
        planner.close()
    if args.out:
        json.dump(res, open(args.out, 'w'), indent=1, default=str)
        print('wrote', args.out)


if __name__ == '__main__':
    main()
