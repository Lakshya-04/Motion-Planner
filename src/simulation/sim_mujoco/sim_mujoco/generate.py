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

from sim_mujoco.mjcf_generator import UrdfRobot, _rpy_matrix, urdf_to_mjcf
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
    """Commanded joints in order: the lift when the profile has one (`lift: null` has none), then the arm."""
    j = profile_data['joints']
    return [*([j['lift']] if j.get('lift') else []), *j['arm']]


def tool_payload(urdf_text, ft_link='tool_mount_link'):
    """(mass in kg, centre of gravity in the `ft_link` frame) of everything beyond the F/T sensor.

    Sums the inertials of `ft_link` and the links below it (fixed joints compose their origins).
    """
    robot = UrdfRobot(urdf_text)
    mass, moment = 0.0, [0.0, 0.0, 0.0]
    stack = [(ft_link, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])]
    while stack:
        name, rot, pos = stack.pop()
        inertial = robot.links[name].inertial
        if inertial and inertial['mass'] > 0:
            c = [pos[i] + sum(rot[i][k] * inertial['pos'][k] for k in range(3)) for i in range(3)]
            mass += inertial['mass']
            moment = [moment[i] + inertial['mass'] * c[i] for i in range(3)]
        for j in robot.child_joints.get(name, []):
            jr = _rpy_matrix(j.rpy)
            stack.append((j.child, [[sum(rot[i][k] * jr[k][c] for k in range(3)) for c in range(3)]
                                    for i in range(3)],
                          [pos[i] + sum(rot[i][k] * j.pos[k] for k in range(3)) for i in range(3)]))
    cog = [m / mass for m in moment] if mass > 0 else [0.0, 0.0, 0.0]
    return mass, cog


def jtac_overrides(base, joints, robot_description, frames, ft_name, mass, cog, gravity=9.81):
    """Apply this robot to the repo's jtac_controller.yaml (dict, edited in place and returned).

    The plugin's F/T sensor carries the raw wrench at the sensor (`tool_mount`) in the sensor axes, with
    the tool weight. So the sensor frame is the mount, the control frame is the TCP, and the tool weight
    is compensated by JTAC's gravity compensation (CoG force = m g, position in the mount frame).
    JTAC only rotates the wrench to the control frame: the lever arm to the TCP and the bias are not
    removed (docs/ft_sensing.md).
    """
    p = base['combined_arm_lift_controller']['ros__parameters']
    p['joints'] = joints
    p['robot_description'] = robot_description
    p['kinematics']['base'] = frames.get('base', 'base_link')
    p['kinematics']['tip'] = frames.get('tcp', 'tcp_link')
    p['ft_sensor']['name'] = ft_name
    p['ft_sensor']['frame']['id'] = frames.get('tool_mount', 'tool_mount_link')
    p['control']['frame']['id'] = frames.get('tcp', 'tcp_link')
    p['gravity_compensation']['frame']['id'] = frames.get('tool_mount', 'tool_mount_link')
    p['gravity_compensation']['CoG']['pos'] = [float(v) for v in cog]
    p['gravity_compensation']['CoG']['force'] = float(mass * gravity)
    p['fixed_world_frame']['frame']['id'] = frames.get('base', 'base_link')
    return base


def base_motion_options(profile_data):
    """`base_motion` generator option from the profile's `base` block (skid steers like a diff drive)."""
    b = profile_data.get('base', {})
    drive = 'diff' if b.get('drive_type', 'omni') in ('diff', 'skid') else 'omni'
    return dict(drive=drive, max_linear=float(b.get('max_linear_velocity_mps') or 0.5),
                max_angular=float(b.get('max_angular_velocity_radps') or 1.0))


def generate(profile_data, urdf_text, out_dir, scene=None, scene_offset=(0, 0, 0, 0), thickness=0.1,
             base_motion=False, **options):
    """Write `out_dir/robot.xml`; returns (path, list of skipped scene objects).

    base_motion  True adds the planar base joints (x, y, yaw) driven from /cmd_vel; off by default.
    """
    if base_motion:
        options['base_motion'] = base_motion_options(profile_data)
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
    ap.add_argument('--base-motion', action='store_true', help='add the planar base joints (/cmd_vel)')
    ap.add_argument('--timestep', type=float, default=0.002)
    args = ap.parse_args(argv)

    profile = load_profile(args.profile)
    urdf = pathlib.Path(args.urdf) if args.urdf else None
    text = urdf.read_text() if urdf else render_urdf(profile)
    path, skipped = generate(profile, text, args.out, scene=args.scene,
                             scene_offset=tuple(args.scene_offset), thickness=args.wall_thickness,
                             urdf_dir=urdf.parent if urdf else '.', timestep=args.timestep,
                             base_motion=args.base_motion)
    for s in skipped:
        print(f'skipped scene object {s}', file=sys.stderr)
    print(path)


if __name__ == '__main__':
    main()
