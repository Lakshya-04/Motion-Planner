"""Catalog-wide URDF check: every profile's expanded xacro agrees with the profile.

Host pytest, no ROS runtime: xacro plus a temporary ament index (like test_lifts.py). Skips with a clear
reason when `xacro` is not importable; profiles whose arm needs ur_description skip when it is absent.
For each profile: joint names and lift limits match, the chain is connected, no duplicate links or
joints, and the stage travels add up to the profile's stroke.
"""
import importlib.util
import os
import pathlib
import tempfile

import pytest
import yaml

xacro = pytest.importorskip('xacro', reason='xacro is not installed: URDF catalog test skipped')
urdf_parser = pytest.importorskip('urdf_parser_py.urdf', reason='urdf_parser_py is not installed')

PKG = pathlib.Path(__file__).resolve().parents[1]
NAME = 'mobile_manipulator_description'
PROFILES = sorted((PKG / 'config' / 'profiles').glob('*.yaml'))
EXPECTED_LIFT_CATALOG = {'fr3_tiago_style_torso', 'generic_6dof_fetch_style_torso', 'generic_6dof_hsr_torso_lift',
                         'generic_6dof_pr2_spine_lift', 'generic_6dof_stretch_mast_lift'}


@pytest.fixture(scope='module', autouse=True)
def ament_prefix():
    tmp = tempfile.TemporaryDirectory()
    root = pathlib.Path(tmp.name)
    (root / 'share').mkdir()
    (root / 'share' / NAME).symlink_to(PKG)
    idx = root / 'share' / 'ament_index' / 'resource_index' / 'packages'
    idx.mkdir(parents=True)
    (idx / NAME).touch()
    old = os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['AMENT_PREFIX_PATH'] = f'{root}:{old}' if old else str(root)
    yield
    os.environ['AMENT_PREFIX_PATH'] = old
    tmp.cleanup()


def _has_ur_description():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('ur_description')
        return True
    except Exception:  # noqa: BLE001
        return False


def _xacro_args(profile):
    spec = importlib.util.spec_from_file_location('profile_xacro_args', PKG / 'scripts' / 'profile_xacro_args.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.xacro_args(profile)


@pytest.fixture(params=PROFILES, ids=lambda p: p.stem)
def rendered(request):
    profile = yaml.safe_load(request.param.read_text())
    arm = (profile['robot']['xacro_args'] or {}).get('arm', 'generic_6dof')
    if arm.startswith('ur') and not _has_ur_description():
        pytest.skip('ur_description not installed')
    args = {k: str(v) for k, v in _xacro_args(profile).items()}
    xml = xacro.process_file(str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'), mappings=args).toxml()
    return profile, urdf_parser.URDF.from_xml_string(xml), xml


def test_catalog_lift_profiles_exist():
    assert EXPECTED_LIFT_CATALOG <= {p.stem for p in PROFILES}


def test_joint_names_and_limits_match_profile(rendered):
    profile, robot, _ = rendered
    joints = robot.joint_map
    names = profile['joints']
    for name in names['arm'] + names['mimic'] + ([names['lift']] if names['lift'] else []):
        assert name in joints, f'profile joint {name} missing from the URDF'
    for name in names['arm']:
        joint = joints[name]
        assert joint.type in ('revolute', 'continuous', 'prismatic')
        if joint.type == 'revolute':
            assert joint.limit.lower < joint.limit.upper, name
            if name in profile['home']:
                assert joint.limit.lower <= profile['home'][name] <= joint.limit.upper, f'home {name} outside limits'
    lift = profile['lift']
    if names['lift']:
        joint = joints[names['lift']]
        if lift['type'] in ('telescopic', 'scissor'):
            assert joint.type == 'prismatic'
            assert joint.limit.lower == pytest.approx(0.0)
            assert joint.limit.upper == pytest.approx(lift['stroke_m'])
            assert joint.limit.velocity == pytest.approx(lift['max_velocity_mps'])
            assert joint.limit.lower <= profile['home'][names['lift']] <= joint.limit.upper
        else:
            assert joint.type == 'fixed'
    # every moving, non-mimic joint of the model is one the profile lists
    moving = {j.name for j in robot.joints if j.type != 'fixed' and j.mimic is None}
    assert moving == set(names['arm']) | ({names['lift']} if names['lift'] and lift['type'] in
                                          ('telescopic', 'scissor') else set())


def test_chain_connected_and_no_duplicates(rendered):
    profile, robot, xml = rendered
    link_names = [l.name for l in robot.links]
    joint_names = [j.name for j in robot.joints]
    assert len(set(link_names)) == len(link_names), 'duplicate links'
    assert len(set(joint_names)) == len(joint_names), 'duplicate joints'
    # one tree: exactly one root, every joint's parent and child exist, every link reachable from the root
    roots = [n for n in link_names if n not in {j.child for j in robot.joints}]
    assert len(roots) == 1, f'expected one root link, got {roots}'
    for j in robot.joints:
        assert j.parent in link_names and j.child in link_names, j.name
    seen, todo = set(), [roots[0]]
    while todo:
        n = todo.pop()
        seen.add(n)
        todo += [c for _, c in robot.child_map.get(n, [])]
    assert seen == set(link_names), f'unreachable links: {set(link_names) - seen}'
    frames = profile['frames']
    for key in ('base', 'lift_base', 'lift_output', 'arm_base', 'flange', 'tool_mount', 'tcp'):
        assert frames[key] in link_names, f'frame {key}={frames[key]} missing'
    chain = robot.get_chain(frames['lift_base'], frames['tcp'], links=False)
    assert len(chain) > 0


def test_total_lift_stroke_equals_profile(rendered):
    profile, robot, _ = rendered
    lift = profile['lift']
    if lift['type'] != 'telescopic' or not lift['stroke_m']:
        pytest.skip('not a telescopic lift')
    main = robot.joint_map[profile['joints']['lift']]
    assert main.limit.upper == pytest.approx(lift['stroke_m'])
    stages = [robot.joint_map[n] for n in profile['joints']['mimic'] if n.startswith('lift_stage_')]
    assert len(stages) == lift['stages']
    if stages:
        total = sum(j.limit.upper for j in stages)
        assert total == pytest.approx(lift['stroke_m']), 'stage travels do not add up to the stroke'
        assert sum(j.mimic.multiplier for j in stages) == pytest.approx(1.0)
        assert all(j.mimic.joint == main.name for j in stages)
