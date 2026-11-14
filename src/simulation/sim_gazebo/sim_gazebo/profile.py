"""Read a description profile (config/profiles/*.yaml) into xacro arguments and joint names.

The profile YAML is the single source of joint names and xacro arguments; launch files and
tests read it through here instead of repeating lists.
"""
import os

import yaml


def profile_dir():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('mobile_manipulator_description'),
                        'config', 'profiles')


def resolve_profile(name_or_path):
    """`generic_6dof_telescopic`, `generic_6dof_telescopic.yaml` or a path."""
    if os.path.isfile(name_or_path):
        return name_or_path
    base = name_or_path if name_or_path.endswith('.yaml') else name_or_path + '.yaml'
    path = os.path.join(profile_dir(), base)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'profile {name_or_path!r} not found (looked in {profile_dir()})')
    return path


def load_profile(name_or_path):
    with open(resolve_profile(name_or_path)) as f:
        return yaml.safe_load(f)


def xacro_mappings(profile, hardware='gazebo'):
    """robot.xacro_args of the profile as xacro mappings (all values as strings)."""
    args = {str(k): str(v) for k, v in (profile.get('robot', {}).get('xacro_args') or {}).items()}
    args['hardware'] = hardware
    return args


def controlled_joints(profile):
    """Lift joint first, then the six arm joints: the order every controller here uses."""
    j = profile['joints']
    return [j['lift']] + list(j['arm'])


def frames(profile):
    f = profile['frames']
    return {'base': f['base'], 'tcp': f['tcp'], 'tool_mount': f['tool_mount'],
            'world_base': f.get('world_base', 'base_footprint')}


def home_positions(profile):
    """Home pose in controlled_joints order (missing entries default to 0)."""
    home = profile.get('home', {})
    return [float(home.get(name, 0.0)) for name in controlled_joints(profile)]
