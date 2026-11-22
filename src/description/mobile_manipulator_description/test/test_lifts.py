"""Lift variants behind the lift contract: telescopic, scissor, fixed, none.

Every variant must render, keep the contract links and `lift_joint`, and take its stroke, limits and
mount height from the profile's `lift:` section. Runs from the source tree (temporary ament prefix).
"""
import importlib.util
import os
import pathlib
import re
import tempfile

import pytest
import xacro
import yaml
from urdf_parser_py.urdf import URDF

PKG = pathlib.Path(__file__).resolve().parents[1]
NAME = 'mobile_manipulator_description'
PROFILES = sorted((PKG / 'config' / 'profiles').glob('*.yaml'))


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


def _args_module():
    spec = importlib.util.spec_from_file_location('profile_xacro_args', PKG / 'scripts' / 'profile_xacro_args.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _render(**mappings):
    return xacro.process_file(str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'),
                              mappings={k: str(v) for k, v in mappings.items()}).toxml()


def _robot(**mappings):
    return URDF.from_xml_string(_render(**mappings))


def _profile(path):
    return yaml.safe_load(path.read_text())


def _usable(profile):
    arm = (profile['robot']['xacro_args'] or {}).get('arm', 'generic_6dof')
    return not arm.startswith('ur') or _has_ur_description()


CONTRACT = ('lift_base_link', 'lift_carriage_link', 'lift_output_link', 'lift_joint')


def _check_contract(robot, ltype, stroke=None):
    links = {l.name for l in robot.links}
    assert {'lift_base_link', 'lift_output_link'} <= links
    lift = robot.joint_map['lift_joint']
    assert lift.parent == 'lift_base_link'
    assert list(lift.axis or [0, 0, 1]) == [0, 0, 1]
    if ltype in ('none', 'fixed'):
        assert lift.type == 'fixed'
    else:
        assert lift.type == 'prismatic'
        assert lift.limit.lower == 0.0
        assert lift.limit.upper == pytest.approx(stroke)
    # the arm hangs off lift_output_link in every variant
    assert robot.joint_map['lift_output_joint'].child == 'lift_output_link'
    chain = robot.get_chain('lift_base_link', 'tcp_link', links=False)
    assert 'lift_output_joint' in chain


@pytest.mark.parametrize('path', PROFILES, ids=lambda p: p.stem)
def test_profile_lift_renders_with_contract_and_profile_limits(path):
    profile = _profile(path)
    if not _usable(profile):
        pytest.skip('ur_description not installed')
    lift = profile['lift']
    robot = _robot(**_args_module().xacro_args(profile))
    _check_contract(robot, lift['type'], lift.get('stroke_m'))
    joint = robot.joint_map['lift_joint']
    if lift['type'] in ('telescopic', 'scissor'):
        assert joint.limit.velocity == pytest.approx(lift['max_velocity_mps'])
        assert joint.limit.upper == pytest.approx(lift['stroke_m'])
    # the retracted output height equals the profile's mount height
    z = sum(robot.joint_map[j].origin.xyz[2] for j in ('lift_joint', 'lift_output_joint'))
    if 'mount_height_m' in lift:
        assert z == pytest.approx(lift['mount_height_m'])
    # the mimic joints the profile lists are exactly the mimic joints of the model
    mimics = sorted(j.name for j in robot.joints if j.mimic is not None)
    assert mimics == sorted(profile['joints']['mimic'])
    # actuated joints: lift (when moving) first, then the arm
    moving = [j for j in robot.get_chain('base_link', 'tcp_link', links=False, fixed=False)
              if robot.joint_map[j].mimic is None]
    expected = ([profile['joints']['lift']] if profile['joints']['lift'] else []) + profile['joints']['arm']
    assert moving == expected


@pytest.mark.parametrize('stroke,height,stages', [(1.0, 0.27, 3), (0.8, 0.35, 2), (1.2, 0.3, 4)])
def test_scissor_parameters(stroke, height, stages):
    robot = _robot(arm='generic_6dof', lift='scissor', lift_stroke=stroke, lift_mount_height=height,
                   lift_stages=stages, lift_max_velocity=0.07)
    _check_contract(robot, 'scissor', stroke)
    lift = robot.joint_map['lift_joint']
    assert lift.limit.velocity == pytest.approx(0.07)
    assert lift.origin.xyz[2] == pytest.approx(height - 0.02)
    mimics = [j for j in robot.joints if j.mimic is not None]
    assert len(mimics) == 3 * stages
    assert all(j.mimic.joint == 'lift_joint' for j in mimics)
    centres = sorted(j.mimic.multiplier for j in mimics if j.type == 'prismatic')
    assert centres == pytest.approx([(k + 0.5) / stages for k in range(stages)])
    # the arm angles span a sensible range and stay inside the joint limit
    for j in mimics:
        if j.type == 'revolute':
            top = j.mimic.offset + j.mimic.multiplier * stroke
            assert j.mimic.offset < top <= j.limit.upper + 1e-6


def test_none_is_a_fixed_mount_at_the_requested_height():
    robot = _robot(arm='generic_6dof', lift='none', lift_mount_height=0.4)
    _check_contract(robot, 'none')
    assert not [j for j in robot.joints if j.mimic is not None]
    moving = robot.get_chain('base_link', 'tcp_link', links=False, fixed=False)
    assert len(moving) == 6
    z = (robot.joint_map['lift_joint'].origin.xyz[2] + robot.joint_map['lift_output_joint'].origin.xyz[2])
    assert z == pytest.approx(0.4)


def test_fixed_pins_the_column_at_lift_fixed_height_and_matches_the_legacy_arg():
    robot = _robot(arm='generic_6dof', lift='fixed', lift_fixed_height=0.5)
    _check_contract(robot, 'fixed')
    legacy = _robot(arm='generic_6dof', lift_fixed_height=0.5)
    assert robot.joint_map['lift_joint'].origin.xyz == pytest.approx(legacy.joint_map['lift_joint'].origin.xyz)
    assert robot.joint_map['lift_joint'].origin.xyz[2] == pytest.approx(0.8 + 0.5)
    assert not [j for j in robot.joints if j.mimic is not None]


def test_pinned_scissor_renders_as_a_column_with_the_same_output_height():
    moving = _robot(arm='generic_6dof', lift='scissor', lift_mount_height=0.27)
    pinned = _robot(arm='generic_6dof', lift='scissor', lift_mount_height=0.27, lift_fixed_height=0.3)
    z = lambda r: sum(r.joint_map[j].origin.xyz[2] for j in ('lift_joint', 'lift_output_joint'))
    assert z(pinned) == pytest.approx(z(moving) + 0.3)
    assert pinned.joint_map['lift_joint'].type == 'fixed'


def test_telescopic_default_is_unchanged():
    robot = _robot(arm='generic_6dof')
    assert robot.joint_map['lift_joint'].origin.xyz[2] == pytest.approx(0.8)
    assert robot.joint_map['lift_joint'].limit.upper == pytest.approx(1.5)
    assert {'lift_stage_1_link', 'lift_stage_2_link'} <= {l.name for l in robot.links}


@pytest.mark.parametrize('mappings, message', [
    ({'lift_stages': 0}, 'at least 1 stage'),
    ({'lift_mount_height': 0.05}, 'mount height'),
])
def test_scissor_rejects_inputs_it_cannot_render(mappings, message):
    with pytest.raises(Exception) as err:
        _render(arm='generic_6dof', lift='scissor', **mappings)
    assert message in str(err.value)


def test_unknown_lift_fails_loudly():
    with pytest.raises(Exception) as err:
        _render(arm='generic_6dof', lift='not_a_lift')
    assert 'unknown lift' in str(err.value)


@pytest.mark.parametrize('lift', ['telescopic', 'scissor', 'fixed', 'none'])
def test_srdf_matches_the_lift(lift):
    extra = {'lift_fixed_height': '0.4'} if lift == 'fixed' else {}
    srdf = xacro.process_file(str(PKG / 'srdf' / 'mobile_manipulator.srdf.xacro'),
                              mappings={'arm': 'generic_6dof', 'lift': lift, **extra}).toxml()
    in_home = 'name="lift_joint" value' in srdf
    assert in_home == (lift in ('telescopic', 'scissor'))
    assert 'group name="lift_arm"' in srdf and 'group name="arm"' in srdf
    assert ('lift_scissor_0_a_link' in srdf) == (lift == 'scissor')


def _scissor_stage_counts():
    counts = {1, 2, 3}
    for path in PROFILES:
        lift = (yaml.safe_load(path.read_text()).get('lift') or {})
        if lift.get('type') == 'scissor':
            counts.add(int(lift.get('stages', 3)))
    return sorted(counts)


def _acm_module():
    spec = importlib.util.spec_from_file_location('generate_acm', PKG / 'scripts' / 'generate_acm.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('stages', _scissor_stage_counts())
def test_scissor_srdf_disables_every_lift_pair_for_any_stage_count(stages):
    """Lift links overlap by design; one missing pair puts every state in self-collision."""
    module = _acm_module()
    robot = _robot(arm='generic_6dof', lift='scissor', lift_stages=stages)
    srdf = xacro.process_file(str(PKG / 'srdf' / 'mobile_manipulator.srdf.xacro'),
                              mappings={'arm': 'generic_6dof', 'lift': 'scissor', 'lift_stages': str(stages)}).toxml()
    pairs = re.findall(r'link1="([^"]+)" link2="([^"]+)"', srdf)
    disabled = {tuple(sorted(m)) for m in pairs}
    assert module.lift_pairs(robot, mounted=False) <= disabled  # the SRDF writes the scissor-internal pairs
    assert len(pairs) == len(disabled), 'duplicate pairs'


@pytest.mark.parametrize('lift', ['none', 'scissor'])
def test_lift_specific_acm_is_up_to_date(lift):
    module = _acm_module()
    robot = _robot(arm='generic_6dof', lift=lift)
    adjacent = module.adjacent_pairs(robot)
    generated = adjacent | (module.lift_pairs(robot, mounted=lift != "scissor") - adjacent)
    if lift == 'scissor':  # the SRDF writes the scissor's lift-internal pairs from lift_stages
        generated = {p for p in generated if not all(n.startswith('lift_') for n in p)}
    text = (PKG / 'srdf' / f'acm_generic_6dof_{lift}.xacro').read_text()
    committed = {tuple(sorted(m)) for m in re.findall(r'link1="([^"]+)" link2="([^"]+)"', text)}
    assert generated == committed, (
        f'stale: run scripts/generate_acm.py --arm generic_6dof --lift {lift} --out srdf/acm_generic_6dof_{lift}.xacro')


# ---- column geometry from the profile: stage count, sizes, continuity ------------------------------

def _geometry_profiles():
    return [p for p in PROFILES if ((_profile(p).get('lift') or {}).get('geometry') or {}).get('stages')]


def _lift_args(path, **extra):
    """Xacro arguments of a profile's lift, on the dependency-free arm so no ur_description is needed."""
    args = _args_module().xacro_args(_profile(path))
    args['arm'] = 'generic_6dof'
    args.update({k: str(v) for k, v in extra.items()})
    return args


def _stage_spans(robot, q):
    """(name, bottom, top, width) of the fixed column and every stage, in lift_base_link, at lift q."""
    def size(link):
        return robot.link_map[link].visuals[0].geometry.size

    column = size('lift_base_link')
    spans = [('lift_base_link', 0.0, column[2], min(column[0], column[1]))]
    parent_bottom, link, index = 0.0, 'lift_base_link', 1
    while f'lift_stage_{index}_link' in robot.link_map:
        joint = robot.joint_map[f'lift_stage_{index}_joint']
        assert joint.parent == link
        travel = joint.mimic.multiplier * q + joint.mimic.offset if joint.mimic else 0.0
        bottom = parent_bottom + joint.origin.xyz[2] + travel
        dims = size(joint.child)
        spans.append((joint.child, bottom, bottom + dims[2], min(dims[0], dims[1])))
        parent_bottom, link, index = bottom, joint.child, index + 1
    return spans


def _stack_violations(robot, stroke, min_overlap=0.05, samples=9):
    """Everything wrong with a telescoping stack over the whole stroke: stages that no longer overlap
    the tube below (a gap), a stage not narrower than the one below, a carriage off the last stage."""
    problems = []
    carriage_z = robot.joint_map['lift_joint'].origin.xyz[2]
    for k in range(samples):
        q = stroke * k / (samples - 1)
        spans = _stage_spans(robot, q)
        for (below, b0, t0, w0), (above, b1, t1, w1) in zip(spans, spans[1:]):
            overlap = min(t0, t1) - max(b0, b1)
            if overlap < min_overlap - 1e-9:
                problems.append(f'{above} overlaps {below} by {overlap:.3f} m at lift {q:.2f}')
            if w1 >= w0:
                problems.append(f'{above} is not narrower than {below}')
        top = spans[-1][2]
        if abs(top - (carriage_z + q)) > 1e-9:
            problems.append(f'the carriage is {carriage_z - top:+.4f} m off the last stage at lift {q:.2f}')
    return problems


@pytest.mark.parametrize('path', _geometry_profiles(), ids=lambda p: p.stem)
def test_profile_stage_geometry_is_continuous_over_the_whole_stroke(path):
    lift = _profile(path)['lift']
    robot = _robot(**_lift_args(path))
    assert _stack_violations(robot, lift['stroke_m']) == []
    stages = lift['geometry']['stages']
    assert len(stages) == lift['stages'] == len(_stage_spans(robot, 0.0)) - 1
    # every stage is the size and rest height the profile states
    spans = _stage_spans(robot, 0.0)
    for st, (_, bottom, top, width) in zip(stages, spans[1:]):
        assert bottom == pytest.approx(st['rest_z_m'])
        assert top - bottom == pytest.approx(st['size_m'][2])
        assert width == pytest.approx(min(st['size_m'][:2]))
    # the last stage follows the lift joint one to one, each lower one is a share of it
    for i, stage in enumerate(stages, start=1):
        assert robot.joint_map[f'lift_stage_{i}_joint'].mimic.multiplier == pytest.approx(1.0 / len(stages))
    # the output moves with the joint from the profile's mount height
    z0 = sum(robot.joint_map[j].origin.xyz[2] for j in ('lift_joint', 'lift_output_joint'))
    assert z0 == pytest.approx(lift['mount_height_m'])
    assert robot.joint_map['lift_joint'].limit.upper == pytest.approx(lift['stroke_m'])


def test_reference_profile_has_the_measured_stroke_and_nested_tube_widths():
    path = PKG / 'config' / 'profiles' / 'ur10e_lift_reference.yaml'
    lift = _profile(path)['lift']
    assert (lift['stroke_m'], lift['stages']) == (0.8, 2)
    assert lift['max_velocity_mps'] > 0 and lift['max_acceleration_mps2'] > 0
    assert lift['geometry']['shape'] == 'box'
    widths = [lift['geometry']['column']['size_m'][0]] + [s['size_m'][0] for s in lift['geometry']['stages']]
    assert widths == sorted(widths, reverse=True) and len(set(widths)) == 3


def test_a_stroke_the_stages_cannot_cover_is_reported_as_a_gap():
    """The same tubes asked to travel 1.2 m: the second stage leaves the first, which the check sees."""
    path = PKG / 'config' / 'profiles' / 'ur10e_lift_reference.yaml'
    robot = _robot(**_lift_args(path, lift_stroke=1.2))
    problems = _stack_violations(robot, 1.2)
    assert any('lift_stage_2_link overlaps lift_stage_1_link' in p for p in problems), problems


def test_the_stack_must_end_where_the_carriage_starts():
    path = PKG / 'config' / 'profiles' / 'ur10e_lift_reference.yaml'
    with pytest.raises(Exception) as err:
        _render(**_lift_args(path, lift_stage_rest_z='-0.014 0.2'))
    assert 'not continuous' in str(err.value)


def test_a_stage_that_is_not_narrower_than_the_one_below_is_rejected():
    path = PKG / 'config' / 'profiles' / 'ur10e_lift_reference.yaml'
    with pytest.raises(Exception) as err:
        _render(**_lift_args(path, lift_stage_sizes='0.164 0.164 0.69 0.17 0.146 0.49'))
    assert 'narrower' in str(err.value)


def test_a_stage_list_of_the_wrong_length_is_rejected():
    path = PKG / 'config' / 'profiles' / 'ur10e_lift_reference.yaml'
    with pytest.raises(Exception) as err:
        _render(**_lift_args(path, lift_stages=3))
    assert 'stage_sizes needs 3 numbers' in str(err.value)


@pytest.mark.parametrize('stages', [1, 2, 3])
def test_explicit_geometry_for_any_stage_count(stages):
    """Tubes as tall as one stage's travel plus 0.15 m, so 0.1 m of overlap is left at full extension."""
    stroke = 0.9
    widths = [0.2 - 0.03 * i for i in range(stages + 1)]
    rests = [0.05 * (i + 1) for i in range(stages)]
    height = stroke / stages + 0.15
    stack = rests[-1] + height
    sizes = ' '.join(f'{w} {w} {height}' for w in widths[1:])
    robot = _robot(arm='generic_6dof', lift_stroke=stroke, lift_stages=stages, lift_shape='box',
                   lift_column_size=f'{widths[0]} {widths[0]} {height}', lift_stage_sizes=sizes,
                   lift_stage_rest_z=' '.join(str(r) for r in rests), lift_mount_height=stack + 0.02)
    assert len([j for j in robot.joints if j.mimic is not None]) == stages
    assert _stack_violations(robot, stroke, min_overlap=0.1) == []
    assert robot.joint_map['lift_joint'].origin.xyz[2] == pytest.approx(stack)


def test_geometry_arguments_come_from_the_profile_section():
    args = _args_module().geometry_args({
        'shape': 'box', 'column': {'size_m': [0.1, 0.2, 0.3]},
        'stages': [{'size_m': [0.09, 0.19, 0.29], 'rest_z_m': 0.01}, {'size_m': [0.08, 0.18, 0.28], 'rest_z_m': 0.02}],
        'carriage': {'size_m': [0.3, 0.3, 0.03]}})
    assert args == {'lift_shape': 'box', 'lift_column_size': '0.1 0.2 0.3',
                    'lift_stage_sizes': '0.09 0.19 0.29 0.08 0.18 0.28', 'lift_stage_rest_z': '0.01 0.02',
                    'lift_carriage_size': '0.3 0.3 0.03'}
    assert _args_module().geometry_args(None) == {}
    assert _args_module().geometry_args({}) == {}
