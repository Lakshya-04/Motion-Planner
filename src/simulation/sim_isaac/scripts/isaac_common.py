"""Pure-Python parts of the Isaac add-on: profile to joint list, URDF mimic, graph spec, arguments.

No Isaac, ROS or numpy imports here, so host pytest and the Isaac container's python can both load it.
"""
import argparse
import pathlib
import xml.etree.ElementTree as ET

DEFAULT_PROFILE = 'ur10e_telescopic'
DEFAULT_COMMAND_TOPIC = '/isaac/joint_command'
DEFAULT_STATE_TOPIC = '/joint_states'
DEFAULT_PHYSICS_HZ = 120.0
STIFFNESS = 1.0e5     # joint drive gains: stiff enough to hold the arm under gravity
DAMPING = 1.0e4
LIFT_STIFFNESS = 1.0e6
LIFT_DAMPING = 1.0e5
MIMIC_FREQUENCY_HZ = 400.0   # stiff enough that the mimic stages do not sag under gravity
MIMIC_DAMPING_RATIO = 1.0


def find_profile(name_or_path, search_dirs=()):
    """A profile file from a bare name (looked up in search_dirs) or a path."""
    p = pathlib.Path(name_or_path)
    if p.suffix and p.is_file():
        return p
    for d in search_dirs:
        cand = pathlib.Path(d) / f'{name_or_path}.yaml'
        if cand.is_file():
            return cand
    raise FileNotFoundError(f'profile {name_or_path!r} not found in {[str(d) for d in search_dirs]}')


def load_profile(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def joint_order(profile):
    """Lift joint first, then the arm joints in kinematic order (the profile's `lift_arm` order)."""
    j = profile['joints']
    names = ([j['lift']] if j.get('lift') else []) + list(j['arm'])
    if len(set(names)) != len(names):
        raise ValueError(f'duplicate joint names in profile: {names}')
    return names


def mimic_joint_names(profile):
    return list((profile['joints'].get('mimic')) or [])


def home_positions(profile, joints):
    """Start pose for `joints` from the profile's home: section (0 for a joint it does not name)."""
    home = profile.get('home') or {}
    return [float(home.get(j, 0.0)) for j in joints]


def urdf_mimics(urdf_path):
    """{joint: (master, multiplier, offset)} for every <mimic> in the URDF."""
    out = {}
    for joint in ET.parse(urdf_path).getroot().iter('joint'):
        m = joint.find('mimic')
        if m is not None:
            out[joint.get('name')] = (m.get('joint'), float(m.get('multiplier', 1.0)), float(m.get('offset', 0.0)))
    return out


def urdf_prismatic_limits(urdf_path):
    """{joint: (lower, upper)} for every prismatic joint with a <limit>."""
    out = {}
    for joint in ET.parse(urdf_path).getroot().iter('joint'):
        lim = joint.find('limit')
        if joint.get('type') == 'prismatic' and lim is not None:
            out[joint.get('name')] = (float(lim.get('lower', 0.0)), float(lim.get('upper', 0.0)))
    return out


def check_mimics(profile, mimics):
    """Profile mimic list and URDF <mimic> tags must agree, each following a commanded joint."""
    listed = set(mimic_joint_names(profile))
    if listed != set(mimics):
        raise ValueError(f'profile mimic joints {sorted(listed)} differ from URDF mimic joints {sorted(mimics)}')
    commanded = set(joint_order(profile))
    for joint, (master, _, _) in mimics.items():
        if master not in commanded:
            raise ValueError(f'{joint} mimics {master}, which the profile does not command')


def drive_gains(joint, profile):
    """(stiffness, damping) for a joint's position drive: the lift is prismatic and carries the arm."""
    if joint == (profile['joints'].get('lift') or ''):
        return LIFT_STIFFNESS, LIFT_DAMPING
    return STIFFNESS, DAMPING


def graph_spec(command_topic=DEFAULT_COMMAND_TOPIC, state_topic=DEFAULT_STATE_TOPIC, robot_prim='/World/robot',
               base_link_prim=None, tf=True):
    """OmniGraph description for og.Controller.edit: ROS 2 bridge nodes, their wiring and constants.

    Returns {'nodes': [(name, type)], 'connect': [(src, dst)], 'values': [(attr, value)]}. The articulation
    root prim is set by the caller on the nodes listed in 'targets'.
    """
    ns = 'isaacsim.ros2.bridge'
    nodes = [
        ('OnTick', 'omni.graph.action.OnPlaybackTick'),
        ('Context', f'{ns}.ROS2Context'),
        ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
        ('PubClock', f'{ns}.ROS2PublishClock'),
        ('PubJointState', f'{ns}.ROS2PublishJointState'),
        ('SubJointState', f'{ns}.ROS2SubscribeJointState'),
        ('ArtController', 'isaacsim.core.nodes.IsaacArticulationController'),
    ]
    connect = [
        ('OnTick.outputs:tick', 'PubClock.inputs:execIn'),
        ('Context.outputs:context', 'PubClock.inputs:context'),
        ('SimTime.outputs:simulationTime', 'PubClock.inputs:timeStamp'),
        ('OnTick.outputs:tick', 'PubJointState.inputs:execIn'),
        ('Context.outputs:context', 'PubJointState.inputs:context'),
        ('SimTime.outputs:simulationTime', 'PubJointState.inputs:timeStamp'),
        ('OnTick.outputs:tick', 'SubJointState.inputs:execIn'),
        ('Context.outputs:context', 'SubJointState.inputs:context'),
        ('OnTick.outputs:tick', 'ArtController.inputs:execIn'),
        ('SubJointState.outputs:jointNames', 'ArtController.inputs:jointNames'),
        ('SubJointState.outputs:positionCommand', 'ArtController.inputs:positionCommand'),
    ]
    values = [
        ('Context.inputs:useDomainIDEnvVar', True),
        ('PubClock.inputs:topicName', '/clock'),
        ('PubJointState.inputs:topicName', state_topic),
        ('SubJointState.inputs:topicName', command_topic),
    ]
    targets = [('PubJointState.inputs:targetPrim', robot_prim), ('ArtController.inputs:targetPrim', robot_prim)]
    if tf:
        nodes += [('PubTF', f'{ns}.ROS2PublishTransformTree')]
        connect += [('OnTick.outputs:tick', 'PubTF.inputs:execIn'),
                    ('Context.outputs:context', 'PubTF.inputs:context'),
                    ('SimTime.outputs:simulationTime', 'PubTF.inputs:timeStamp')]
        targets += [('PubTF.inputs:targetPrims', base_link_prim or robot_prim)]
    return {'nodes': nodes, 'connect': connect, 'values': values, 'targets': targets}


def mimic_targets(mimics, lift_position):
    """Position for each mimic joint, given the master joint's position (one level, masters are commanded joints)."""
    return {j: m * lift_position + o for j, (_, m, o) in mimics.items()}


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description='Headless Isaac Sim runner with a ROS 2 bridge graph.')
    ap.add_argument('--usd', required=True, help='robot USD (from urdf_to_usd.py)')
    ap.add_argument('--profile', default=DEFAULT_PROFILE, help='profile name or YAML path (joint list and order)')
    ap.add_argument('--profile-dir', action='append', default=[], help='where to look up a bare profile name')
    ap.add_argument('--command-topic', default=DEFAULT_COMMAND_TOPIC)
    ap.add_argument('--state-topic', default=DEFAULT_STATE_TOPIC)
    ap.add_argument('--physics-hz', type=float, default=DEFAULT_PHYSICS_HZ)
    ap.add_argument('--no-tf', action='store_true', help='skip the transform tree publisher')
    ap.add_argument('--no-realtime', action='store_true', help='step as fast as possible instead of at wall-clock rate')
    lim = ap.add_mutually_exclusive_group()
    lim.add_argument('--steps', type=int, help='stop after this many physics steps')
    lim.add_argument('--seconds', type=float, help='stop after this much wall time')
    args = ap.parse_args(argv)
    if args.physics_hz <= 0:
        ap.error('--physics-hz must be positive')
    return args
