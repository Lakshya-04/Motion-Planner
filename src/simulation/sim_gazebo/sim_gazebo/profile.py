"""Profile access for the Gazebo backend, on top of the shared reader.

The profile YAML is read and validated by `mobile_manipulator_description.profile` (the same
reader every other node uses), so a profile means the same thing here: `lift: null` is a chain
without a lift joint and a missing home entry is an error, not 0. These helpers only add what is
specific to this package.
"""
from mobile_manipulator_description.profile import (  # noqa: F401  (re-exported)
    Profile, ProfileError, load_profile, parse_profile)


def xacro_mappings(profile, hardware='gazebo'):
    """The profile's xacro arguments as xacro mappings, with the hardware backend set."""
    args = profile.xacro_mappings()
    args['hardware'] = hardware
    return args


def controlled_joints(profile):
    """Actuated joints in the order every controller here uses (lift first when there is one)."""
    return list(profile.joints)


def frames(profile):
    f = profile.frames
    return {'base': f['base'], 'tcp': f['tcp'], 'tool_mount': f['tool_mount'],
            'world_base': f['world_base']}


def home_positions(profile):
    """Home pose in controlled_joints order."""
    return list(profile.home_positions)
