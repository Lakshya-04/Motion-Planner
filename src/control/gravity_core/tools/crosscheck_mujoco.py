#!/usr/bin/env python3
"""Compare gravity_core torques with MuJoCo's own at random poses.

    crosscheck_mujoco.py --urdf robot.urdf --joints lift_joint,arm_shoulder_pan_joint,... \
        --eval build/gravity_eval [--payload tool_mount_link 0.8 0 0 0.02] [--n 500] [--seed 0]

MuJoCo loads the URDF with geometry stripped (inertials only). At zero velocity, qfrc_bias is the
gravity term, and holding a pose takes exactly that. MuJoCo ignores <mimic>, so the mimic joints are
set by hand and their torque is folded into the joint they follow, the same rule gravity_core uses.
"""
import argparse
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def strip_geometry(urdf_text):
    text = re.sub(r'<visual>.*?</visual>', '', urdf_text, flags=re.S)
    return re.sub(r'<collision>.*?</collision>', '', text, flags=re.S)


def add_payload(root, link, mass, com):
    inertial = root.find(f"./link[@name='{link}']/inertial")
    if inertial is None:
        raise SystemExit(f'no inertial on {link}')
    m0 = float(inertial.find('mass').get('value'))
    c0 = np.array([float(v) for v in inertial.find('origin').get('xyz', '0 0 0').split()]) \
        if inertial.find('origin') is not None else np.zeros(3)
    c1 = (m0 * c0 + mass * np.asarray(com)) / (m0 + mass)
    inertial.find('mass').set('value', repr(m0 + mass))
    origin = inertial.find('origin')
    if origin is None:
        origin = ET.SubElement(inertial, 'origin')
    origin.set('xyz', ' '.join(repr(float(v)) for v in c1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--joints', required=True, help='actuated joints, comma separated')
    ap.add_argument('--eval', required=True, help='path to the gravity_eval binary')
    ap.add_argument('--payload', nargs=5, metavar=('LINK', 'MASS', 'CX', 'CY', 'CZ'))
    ap.add_argument('--n', type=int, default=500)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--json')
    a = ap.parse_args()

    joints = a.joints.split(',')
    root = ET.fromstring(open(a.urdf).read())
    if a.payload:
        add_payload(root, a.payload[0], float(a.payload[1]), [float(v) for v in a.payload[2:]])
    mimics = {}
    limits = {}
    for j in root.findall('joint'):
        m = j.find('mimic')
        if m is not None:
            mimics[j.get('name')] = (m.get('joint'), float(m.get('multiplier', 1)), float(m.get('offset', 0)))
        lim = j.find('limit')
        if lim is not None and j.get('type') in ('revolute', 'prismatic'):
            limits[j.get('name')] = (float(lim.get('lower', -3)), float(lim.get('upper', 3)))
    urdf_text = strip_geometry(ET.tostring(root, encoding='unicode'))
    # Fixed bodies are kept as bodies so no mass is merged away or dropped.
    urdf_text = urdf_text.replace('<robot ', '<robot ', 1)
    tmp = a.urdf + '.stripped.urdf'
    open(tmp, 'w').write(urdf_text)
    spec = mujoco.MjSpec.from_file(tmp)
    spec.compiler.fusestatic = 0
    spec.compiler.balanceinertia = 1
    spec.option.gravity = [0, 0, -9.81]
    model = spec.compile()
    data = mujoco.MjData(model)

    def jid(name):
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if i < 0:
            raise SystemExit(f'MuJoCo has no joint {name}')
        return i

    rng = np.random.default_rng(a.seed)
    qs = np.array([[rng.uniform(*limits.get(j, (-1.5, 1.5))) for j in joints] for _ in range(a.n)])

    def driver_of(name):
        scale, off = 1.0, 0.0
        while name not in joints:
            drv, mult, o = mimics[name]
            off += scale * o
            scale *= mult
            name = drv
        return name, scale, off

    all_joints = [model.joint(i).name for i in range(model.njnt)]
    folds = [(n, *driver_of(n)) for n in all_joints if n in joints or n in mimics]
    expect = np.zeros_like(qs)
    for r, q in enumerate(qs):
        data.qpos[:] = 0
        data.qvel[:] = 0
        for n, drv, scale, off in folds:
            data.qpos[model.jnt_qposadr[jid(n)]] = scale * q[joints.index(drv)] + off
        mujoco.mj_forward(model, data)
        for n, drv, scale, off in folds:
            expect[r, joints.index(drv)] += scale * data.qfrc_bias[model.jnt_dofadr[jid(n)]]

    cmd = [a.eval, a.urdf, a.joints] + (a.payload if a.payload else [])
    out = subprocess.run(cmd, input='\n'.join(' '.join(repr(float(v)) for v in q) for q in qs) + '\n',
                         capture_output=True, text=True, check=True).stdout
    got = np.array([[float(v) for v in line.split()] for line in out.strip().splitlines()])
    err = np.abs(got - expect)
    rows = []
    for k, j in enumerate(joints):
        rows.append({'joint': j, 'max_abs_err': float(err[:, k].max()),
                     'max_torque': float(np.abs(expect[:, k]).max())})
        print(f'{j:32s} max |err| {err[:, k].max():.2e}   max |tau| {np.abs(expect[:, k]).max():8.3f}')
    worst = float(err.max())
    print(f'worst absolute error over all joints and poses: {worst:.2e} (N m, N for the lift)')
    if a.json:
        json.dump({'n': a.n, 'worst_abs': worst, 'joints': rows}, open(a.json, 'w'), indent=1)
    return 0 if worst < 1e-6 else 1  # joints with a vertical axis carry no torque, so no relative test


if __name__ == '__main__':
    sys.exit(main())
