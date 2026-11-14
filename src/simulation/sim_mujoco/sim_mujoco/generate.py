"""Generate the MuJoCo model (robot + optional wall world) from a robot profile.

    ros2 run sim_mujoco generate_mjcf --profile ur10e_telescopic --out /tmp/mj \
        --scene wall_scene.json --scene-offset 0.2 0 0 0

The profile (`mobile_manipulator_description/config/profiles/*.yaml`) gives the xacro arguments, the
joint names and order, the frame names and the home pose, so nothing here repeats a joint list.
`xacro` must be on PATH (a sourced ROS environment) unless `--urdf` is given.
"""
import argparse
import pathlib
import subprocess
import sys

import yaml

from sim_mujoco.mjcf_generator import urdf_to_mjcf
from sim_mujoco.wall_world import wall_world_geoms


def _share(package):
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory(package))


def load_profile(profile):
    path = pathlib.Path(profile)
    if not path.is_file():
        path = _share('mobile_manipulator_description') / 'config' / 'profiles' / f'{profile}.yaml'
    with open(path) as f:
        return yaml.safe_load(f)


def render_urdf(profile_data, xacro_file=None, extra_args=()):
    """Run xacro with the profile's arguments; returns the URDF text (hardware:=none)."""
    if xacro_file is None:
        xacro_file = _share('mobile_manipulator_description') / 'urdf' / 'mobile_manipulator.urdf.xacro'
    args = [f'{k}:={v}' for k, v in profile_data['robot'].get('xacro_args', {}).items()]
    cmd = ['xacro', str(xacro_file), *args, *extra_args]
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def command_joints(profile_data):
    j = profile_data['joints']
    return [j['lift'], *j['arm']]


def generate(profile_data, urdf_text, out_dir, scene=None, scene_offset=(0, 0, 0, 0), thickness=0.1,
             **options):
    """Write `out_dir/robot.xml`; returns (path, list of skipped scene objects)."""
    frames = profile_data.get('frames', {})
    geoms, skipped = [], []
    if scene:
        geoms, _, skipped = wall_world_geoms(scene, scene_offset, thickness)
    path = urdf_to_mjcf(
        urdf_text, out_dir, home=profile_data.get('home'), commanded=command_joints(profile_data),
        ft_link=frames.get('tool_mount', 'tool_mount_link'), tcp_link=frames.get('tcp', 'tcp_link'),
        world_geoms=geoms, **options)
    return path, skipped


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--profile', required=True, help='profile name or YAML path')
    ap.add_argument('--out', required=True, help='output directory (robot.xml and meshes/)')
    ap.add_argument('--urdf', help='use this URDF instead of rendering the xacro')
    ap.add_argument('--scene', help='wall_scene.json / scene_model scene / cuRobo world to add')
    ap.add_argument('--scene-offset', type=float, nargs=4, default=[0, 0, 0, 0],
                    metavar=('X', 'Y', 'Z', 'YAW'), help='scene pose in the robot base frame')
    ap.add_argument('--wall-thickness', type=float, default=0.1)
    ap.add_argument('--timestep', type=float, default=0.002)
    args = ap.parse_args(argv)

    profile = load_profile(args.profile)
    urdf = pathlib.Path(args.urdf) if args.urdf else None
    text = urdf.read_text() if urdf else render_urdf(profile)
    path, skipped = generate(profile, text, args.out, scene=args.scene,
                             scene_offset=tuple(args.scene_offset), thickness=args.wall_thickness,
                             urdf_dir=urdf.parent if urdf else '.', timestep=args.timestep)
    for s in skipped:
        print(f'skipped scene object {s}', file=sys.stderr)
    print(path)


if __name__ == '__main__':
    main()
