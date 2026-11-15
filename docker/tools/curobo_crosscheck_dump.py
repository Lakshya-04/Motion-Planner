#!/usr/bin/env python3
"""cuRobo side of the MuJoCo cross-check: tool-frame FK and native self-collision verdicts.

Needs the gpu/sim image and a GPU (a few seconds, one batch). Writes an .npz the cross-check reads,
so the comparison itself never needs CUDA.

  docker run --rm --gpus all -v $PWD/ur10e:/robots -v $PWD/docker/tools:/tools:ro motion-planner:sim \
    /opt/venv/bin/python /tools/curobo_crosscheck_dump.py --cfg /robots/curobo.yml \
      --q /robots/q.npy --tool-frame tcp_link --out /robots/curobo_dump.npz

`q.npy` is n x dof in the cuRobo joint order (`joint_names` is saved in the npz and checked by the
cross-check). Outputs: `q` (the input, so the cross-check can verify the rows match), `position` (n,3), `quaternion` (n,4, wxyz order as cuRobo returns), `self_collision` (n,) bool.
"""
import argparse

import numpy as np
import torch
import yaml

from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import JointState


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', required=True)
    ap.add_argument('--q', required=True)
    ap.add_argument('--tool-frame', default='tcp_link')
    ap.add_argument('--out', required=True)
    ap.add_argument('--chunk', type=int, default=500)
    args = ap.parse_args()

    kin = Kinematics(KinematicsCfg.from_robot_yaml_file(args.cfg))
    q = np.load(args.q).astype(np.float32)
    checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
        robot_config=yaml.safe_load(open(args.cfg)), scene_model=None, self_collision_activation_distance=0.0))
    pos, quat, selfcol = [], [], []
    for i in range(0, len(q), args.chunk):
        js = JointState.from_position(torch.as_tensor(q[i:i + args.chunk], device='cuda'), joint_names=kin.joint_names)
        state = kin.compute_kinematics(js)
        poses = state.tool_poses
        idx = poses.tool_frames.index(args.tool_frame)
        qt = torch.as_tensor(q[i:i + args.chunk], device='cuda').unsqueeze(1)
        # self-collision cost is positive when spheres of a non-ignored pair overlap
        _, d_self = checker.get_scene_self_collision_distance_from_joints(qt)
        selfcol.append((d_self.reshape(len(qt), -1).sum(-1) > 0).cpu().numpy())
        pos.append(poses.position[..., idx, :].reshape(-1, 3).cpu().numpy())
        quat.append(poses.quaternion[..., idx, :].reshape(-1, 4).cpu().numpy())
    np.savez(args.out, joint_names=np.array(kin.joint_names), q=q, position=np.concatenate(pos),
             quaternion=np.concatenate(quat), self_collision=np.concatenate(selfcol))
    print('wrote', args.out, kin.joint_names)


if __name__ == '__main__':
    main()
