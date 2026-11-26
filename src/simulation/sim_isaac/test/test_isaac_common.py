"""Pure-Python parts of the Isaac add-on. These run without Isaac, ROS or a GPU."""
import pathlib
import sys

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'scripts'))

import isaac_common as ic  # noqa: E402

PROFILE_DIRS = [p / 'src' / 'description' / 'mobile_manipulator_description' / 'config' / 'profiles'
                for p in PKG.parents if (p / 'src' / 'description').is_dir()][:1]
UR_JOINTS = ['lift_joint', 'arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
             'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']

URDF = """<robot name="r">
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="lift_joint" type="prismatic"><parent link="a"/><child link="b"/></joint>
  <joint name="lift_stage_1_joint" type="prismatic"><parent link="a"/><child link="c"/>
    <mimic joint="lift_joint" multiplier="0.5" offset="0.1"/></joint>
</robot>"""


def _profile():
    return ic.load_profile(ic.find_profile('ur10e_telescopic', PROFILE_DIRS))


def test_joint_order_from_the_real_profile():
    assert ic.joint_order(_profile()) == UR_JOINTS


def test_joint_order_without_lift():
    assert ic.joint_order({'joints': {'lift': None, 'arm': ['a', 'b']}}) == ['a', 'b']


def test_duplicate_joint_names_rejected():
    with pytest.raises(ValueError):
        ic.joint_order({'joints': {'lift': 'a', 'arm': ['a']}})


def test_home_positions_follow_the_joint_order():
    p = _profile()
    home = ic.home_positions(p, ic.joint_order(p))
    assert home[0] == pytest.approx(0.4)
    assert home[2] == pytest.approx(-1.5708)
    assert len(home) == 7


def test_find_profile_missing():
    with pytest.raises(FileNotFoundError):
        ic.find_profile('no_such_profile', PROFILE_DIRS)


def test_urdf_mimics_and_targets(tmp_path):
    f = tmp_path / 'r.urdf'
    f.write_text(URDF)
    mim = ic.urdf_mimics(f)
    assert mim == {'lift_stage_1_joint': ('lift_joint', 0.5, 0.1)}
    assert ic.mimic_targets(mim, 0.4) == {'lift_stage_1_joint': pytest.approx(0.3)}


def test_urdf_prismatic_limits(tmp_path):
    f = tmp_path / 'r.urdf'
    f.write_text('<robot><joint name="j" type="prismatic"><limit lower="0" upper="0.75"/></joint>'
                 '<joint name="r" type="revolute"><limit lower="-1" upper="1"/></joint></robot>')
    assert ic.urdf_prismatic_limits(f) == {'j': (0.0, 0.75)}


def test_real_urdf_mimics_match_profile():
    p = _profile()
    mim = {'lift_stage_1_joint': ('lift_joint', 0.5, 0.0), 'lift_stage_2_joint': ('lift_joint', 0.5, 0.0)}
    ic.check_mimics(p, mim)
    with pytest.raises(ValueError):
        ic.check_mimics(p, {'lift_stage_1_joint': ('lift_joint', 0.5, 0.0)})
    with pytest.raises(ValueError):
        ic.check_mimics(p, {k: ('nope', 1.0, 0.0) for k in mim})


def test_drive_gains_lift_is_stiffer():
    p = _profile()
    assert ic.drive_gains('lift_joint', p)[0] > ic.drive_gains('arm_elbow_joint', p)[0]


def test_graph_spec_wiring_is_consistent():
    spec = ic.graph_spec(tf=True)
    names = {n for n, _ in spec['nodes']}
    for src, dst in spec['connect']:
        assert src.split('.')[0] in names and dst.split('.')[0] in names
    topics = dict(spec['values'])
    assert topics['PubClock.inputs:topicName'] == '/clock'
    assert topics['PubJointState.inputs:topicName'] == '/joint_states'
    assert topics['SubJointState.inputs:topicName'] == '/isaac/joint_command'
    assert ('SubJointState.outputs:positionCommand', 'ArtController.inputs:positionCommand') in spec['connect']


def test_graph_spec_topics_and_no_tf():
    spec = ic.graph_spec(command_topic='/cmd', state_topic='/js', tf=False)
    assert dict(spec['values'])['SubJointState.inputs:topicName'] == '/cmd'
    assert dict(spec['values'])['PubJointState.inputs:topicName'] == '/js'
    assert not any(n == 'PubTF' for n, _ in spec['nodes'])


def test_parse_args_defaults_and_limits():
    a = ic.parse_args(['--usd', 'x.usd'])
    assert a.profile == 'ur10e_telescopic' and a.physics_hz == 120.0 and a.steps is None
    assert ic.parse_args(['--usd', 'x.usd', '--steps', '5']).steps == 5
    with pytest.raises(SystemExit):
        ic.parse_args(['--usd', 'x.usd', '--steps', '5', '--seconds', '2'])
    with pytest.raises(SystemExit):
        ic.parse_args(['--usd', 'x.usd', '--physics-hz', '0'])
    with pytest.raises(SystemExit):
        ic.parse_args([])
