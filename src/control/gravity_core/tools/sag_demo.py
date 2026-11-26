#!/usr/bin/env python3
"""How far does a position PD sag with real gravity, and what does the gravity feedforward fix?

    sag_demo.py --urdf robot.urdf --joints lift_joint,... --eval build/gravity_eval --tool-link tool_mount_link

Same actuator law as sim_mujoco: kp = 10 * effort (N m/rad), 100 * effort (N/m) for the lift,
kv = 0.1 kp, force clamped to the effort limit. Bodies are NOT gravity compensated here.
Variant B adds the feedforward the way the plugin adds velocity feedforward: ctrl = q* + tau_g / kp.
"""
import argparse
import re
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--joints', required=True)
    ap.add_argument('--eval', required=True)
    ap.add_argument('--tool-link', default='tool_mount_link')
    ap.add_argument('--payload', nargs=4, type=float, default=None, metavar=('MASS', 'CX', 'CY', 'CZ'),
                    help='extra payload on the tool link, also given to gravity_core')
    ap.add_argument('--seconds', type=float, default=4.0)
    a = ap.parse_args()
    joints = a.joints.split(',')

    root = ET.fromstring(open(a.urdf).read())
    effort, mimics, kinds = {}, {}, {}
    for j in root.findall('joint'):
        kinds[j.get('name')] = j.get('type')
        lim = j.find('limit')
        if lim is not None:
            effort[j.get('name')] = float(lim.get('effort'))
        m = j.find('mimic')
        if m is not None:
            mimics[j.get('name')] = (m.get('joint'), float(m.get('multiplier', 1)), float(m.get('offset', 0)))
    text = ET.tostring(root, encoding='unicode')
    text = re.sub(r'<visual>.*?</visual>|<collision>.*?</collision>', '', text, flags=re.S)
    open(a.urdf + '.demo.urdf', 'w').write(text)
    spec = mujoco.MjSpec.from_file(a.urdf + '.demo.urdf')
    spec.compiler.fusestatic = 0
    spec.compiler.balanceinertia = 1
    spec.option.gravity = [0, 0, -9.81]
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if a.payload:
        body = spec.body(a.tool_link)
        body.mass += a.payload[0]
    for jn, (drv, mult, off) in mimics.items():
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_JOINT
        eq.name1, eq.name2 = jn, drv
        eq.data[:5] = [off, mult, 0, 0, 0]
    for jn in joints:
        act = spec.add_actuator()
        act.name = 'act_' + jn
        act.target = jn
        kp = effort[jn] * (100.0 if kinds[jn] == 'prismatic' else 10.0)
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.gainprm[0] = kp
        act.biasprm[:3] = [0, -kp, -0.1 * kp]
        act.forcerange = [-effort[jn], effort[jn]]
        act.forcelimited = 1
        act.ctrllimited = 0
    for d in spec.joints:
        d.damping = np.array([2.0 if d.type == mujoco.mjtJoint.mjJNT_HINGE else 100.0, 0, 0])
        d.armature = 0.01 if d.type == mujoco.mjtJoint.mjJNT_HINGE else 2.0
    model = spec.compile()
    data = mujoco.MjData(model)
    kp_of = np.array([effort[j] * (100.0 if kinds[j] == 'prismatic' else 10.0) for j in joints])
    jid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joints]
    tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, a.tool_link)

    def settle(q_goal, ff):
        data.qpos[:] = 0
        data.qvel[:] = 0
        for jn, (drv, mult, off) in mimics.items():
            data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = \
                mult * q_goal[joints.index(drv)] + off
        for k, j in enumerate(jid):
            data.qpos[model.jnt_qposadr[j]] = q_goal[k]
        mujoco.mj_forward(model, data)
        tool_ref = data.xpos[tool].copy()
        tau = np.zeros(len(joints))
        if ff:
            cmd = [a.eval, a.urdf, a.joints] + ([a.tool_link, str(a.payload[0]), *map(str, a.payload[1:])]
                                                 if a.payload else [])
            out = subprocess.run(cmd, input=' '.join(repr(float(v)) for v in q_goal) + '\n', capture_output=True, text=True,
                                 check=True).stdout
            tau = np.array([float(v) for v in out.split()])
        data.ctrl[:] = q_goal + tau / kp_of
        for _ in range(int(a.seconds / model.opt.timestep)):
            mujoco.mj_step(model, data)
        sag_q = np.array([data.qpos[model.jnt_qposadr[j]] for j in jid]) - q_goal
        return np.linalg.norm(data.xpos[tool] - tool_ref), sag_q

    rng = np.random.default_rng(3)
    poses = [np.array([0.5, 0.0, -1.2, 1.4, -0.3, 0.0, 0.0]),
             np.array([0.9, 0.5, -0.4, 0.3, -1.0, 0.4, 0.0]),
             np.array([0.2, -0.3, -1.5, 1.8, -1.6, 0.0, 0.0])]
    print(f'{"pose":>4}  {"no feedforward":>26}  {"with gravity feedforward":>26}')
    for i, q in enumerate(poses):
        q = q[:len(joints)]
        e0, s0 = settle(q, False)
        e1, s1 = settle(q, True)
        print(f'{i:>4}  tool off {e0 * 1000:7.2f} mm  q {np.degrees(np.abs(s0[1:]).max()):5.2f} deg'
              f'  tool off {e1 * 1000:7.3f} mm  q {np.degrees(np.abs(s1[1:]).max()):6.3f} deg')


if __name__ == '__main__':
    main()
