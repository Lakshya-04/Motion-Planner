#!/usr/bin/env python3
"""Write a Gazebo SDF world for a wall scene.

  ros2 run sim_gazebo wall_world_gen.py scene.json -o wall.sdf --wall-id 0 --standoff 0.9
  ros2 run sim_gazebo wall_world_gen.py scene.yaml -o objects.sdf      (SceneModel YAML)
"""
import argparse

from sim_gazebo.wall_world import generate_world


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('scene', help='wall_scene.json or SceneModel YAML')
    p.add_argument('-o', '--out', required=True)
    p.add_argument('--wall-id', type=int, default=0)
    p.add_argument('--standoff', type=float, default=0.9, help='wall plane distance in front of the base (m)')
    p.add_argument('--no-align', action='store_true', help='keep the scene frame (json only)')
    p.add_argument('--crop-u', type=float, nargs=2, metavar=('MIN', 'MAX'), help='wall u window (m)')
    p.add_argument('--crop-v', type=float, nargs=2, metavar=('MIN', 'MAX'), help='wall v window (m)')
    p.add_argument('--friction', type=float, default=0.8)
    a = p.parse_args()
    shapes = generate_world(a.scene, a.out, wall_id=a.wall_id, align=not a.no_align,
                            standoff=a.standoff, crop_u=a.crop_u, crop_v=a.crop_v, friction=a.friction)
    print(f'wrote {a.out}: {len(shapes)} shapes ({sum(s.role == "wall" for s in shapes)} wall)')


if __name__ == '__main__':
    main()
