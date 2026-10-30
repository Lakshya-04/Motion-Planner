#!/usr/bin/env python3
"""URDF -> cuRobo v2 robot config (collision spheres + collision matrix), via cuRobo's RobotBuilder.

Run inside the gpu image (needs CUDA):

  docker run --rm --gpus all -v "$PWD/robots:/robots" motion-planner:gpu \
    /opt/venv/bin/python docker/tools/generate_curobo_robot.py \
      --urdf /robots/meshed/robot.urdf --tool-frame tcp_link --out /robots/mm_arm.yml

The URDF's collision geometry must be meshes (docker/tools/primitives_to_meshes.py converts
primitives, as for the VAMP pipeline). cuRobo stores absolute paths in the config, so the URDF and
its meshes must live where the planner will read them: pass the final location (by default the
directory the URDF is in) and mount the same path when planning.
"""
import argparse
import pathlib

from curobo.robot_builder import RobotBuilder


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--asset-root', help='directory mesh paths are relative to (default: the URDF directory)')
    ap.add_argument('--tool-frame', action='append', required=True, help='repeatable')
    ap.add_argument('--out', required=True)
    ap.add_argument('--sphere-density', type=float, default=1.0)
    ap.add_argument('--batch-size', type=int, default=1000,
                    help='collision-matrix sampling batch; cuRobo defaults to 10000, which needs >7 GB of GPU memory')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    import numpy as np
    import torch
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    urdf = pathlib.Path(args.urdf).resolve()
    root = pathlib.Path(args.asset_root).resolve() if args.asset_root else urdf.parent
    builder = RobotBuilder(urdf_path=str(urdf), asset_path=str(root), tool_frames=args.tool_frame)
    # cuRobo fits to the *visual* meshes by default; our pipelines carry collision meshes only (the
    # resolver drops visuals), so fit to those.
    builder.fit_collision_spheres(sphere_density=args.sphere_density, use_collision_mesh=True)
    if builder.num_spheres == 0:
        raise SystemExit('no collision spheres fitted: the URDF has no collision meshes cuRobo could load')
    builder.compute_collision_matrix(batch_size=args.batch_size)
    builder.save(builder.build(), args.out)
    print(f'wrote {args.out}: {builder.num_spheres} spheres on {len(builder.collision_link_names)} links')


if __name__ == '__main__':
    main()
