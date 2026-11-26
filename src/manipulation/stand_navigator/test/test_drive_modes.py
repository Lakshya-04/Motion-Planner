import glob
import hashlib
import os

import pytest
import yaml

from stand_navigator.core import BaseProfile, load_profile
from stand_navigator.modes import MODE_NAMES, ModeError, derive_mode, parse_modes
from test_core import simulate

PROFILES = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'description',
                        'mobile_manipulator_description', 'config', 'profiles')
FILES = sorted(glob.glob(os.path.join(PROFILES, '*.yaml')))
LIM = (1.0, 1.5, 0.8, 1.2)


def _base(**kw):
    return {'name': 'bot', 'modes': [dict(name='diff_turn_in_place', **kw)]}


def test_shipped_profiles_found():
    assert len(FILES) >= 30


@pytest.mark.parametrize('path', FILES, ids=[os.path.basename(f) for f in FILES])
def test_every_shipped_profile_loads(path):
    raw = yaml.safe_load(open(path))
    if 'base' not in raw:
        pytest.skip('no base section')
    b = raw['base']
    p = load_profile(path)
    # the old loader's fields are unchanged
    assert p.drive == str(b.get('drive_type', 'omni'))
    assert p.max_lin == float(b.get('max_linear_velocity_mps', 0.5))
    assert p.max_ang == float(b.get('max_angular_velocity_radps', 1.0))
    modes = p.drive_modes()
    assert sum(m.default for m in modes) == 1
    if 'modes' not in b:
        assert p.modes == ()
        (m,) = modes
        assert m.name == {'omni': 'omni', '4ws': 'omni', 'skid': 'skid'}.get(p.drive, 'diff_turn_in_place')
        assert m.can_strafe == p.holonomic
        assert (m.max_lin, m.max_ang, m.max_lin_acc, m.max_ang_acc) == (
            p.max_lin, p.max_ang, p.max_lin_acc, p.max_ang_acc)
    for m in modes:
        assert m.name in MODE_NAMES and m.max_lin > 0 and m.max_ang > 0
        assert (m.min_turn_radius is not None) == (m.name == 'ackermann')


def test_ranger_mini_modes():
    p = load_profile(os.path.join(PROFILES, 'ur10e_ranger_mini_telescopic.yaml'))
    by = {m.name: m for m in p.modes}
    assert set(by) == {'ackermann', 'crab', 'spin'}
    assert by['ackermann'].default and by['ackermann'].min_turn_radius == 0.81
    assert by['crab'].can_strafe and not by['crab'].can_spin_in_place
    assert by['spin'].can_spin_in_place and by['spin'].max_ang == 3.259
    # profile numbers still drive the legacy path
    assert p.drive == '4ws' and p.holonomic


@pytest.mark.parametrize('base, text', [
    ({'name': 'bot', 'modes': [{'name': 'hover'}]}, "profile bot: mode 'hover': unknown name"),
    (_base(max_linear_mps=0.0), 'profile bot: mode diff_turn_in_place: max_linear_mps must be positive'),
    (_base(max_angular_radps=-1), 'max_angular_radps must be positive'),
    ({'name': 'bot', 'modes': [{'name': 'omni', 'default': True}, {'name': 'crab', 'default': True}]},
     'profile bot: more than one mode has default'),
    ({'name': 'bot', 'modes': [{'name': 'ackermann'}]}, 'profile bot: mode ackermann: min_turn_radius_m is required'),
    ({'name': 'bot', 'modes': [{'name': 'ackermann', 'min_turn_radius_m': -1}]}, 'min_turn_radius_m must be positive'),
    (_base(can_strafe=True), 'profile bot: mode diff_turn_in_place: can_strafe must be false'),
    ({'name': 'bot', 'modes': [{'name': 'omni', 'min_turn_radius_m': 0.5}]}, 'min_turn_radius_m must be null'),
    ({'name': 'bot', 'modes': [{'name': 'omni'}, {'name': 'omni'}]}, 'mode omni: listed twice'),
    ({'name': 'bot', 'modes': []}, 'non-empty'),
])
def test_validation_errors_name_profile_and_mode(base, text):
    with pytest.raises(ModeError) as e:
        parse_modes(base, base['name'], LIM)
    assert text in str(e.value)


def test_missing_limits_inherit_and_first_mode_is_default():
    ms = parse_modes({'modes': [{'name': 'ackermann', 'min_turn_radius_m': 1.0, 'max_linear_mps': 2.0},
                                {'name': 'spin'}]}, 'bot', LIM)
    assert ms[0].default and not ms[1].default
    assert (ms[0].max_lin, ms[0].max_ang) == (2.0, 1.5) and ms[1].max_lin == 1.0


def test_load_profile_error_names_the_profile(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text("base:\n  name: broken_bot\n  modes:\n    - {name: ackermann}\n")
    with pytest.raises(ModeError, match='broken_bot'):
        load_profile(str(f))


# Without `modes` the controller output must be what it was before this change (digests taken on main).
GOLDEN = {
    'omni': (307, 'ea20678a68e20aa6', (1.982791987, -1.490979641, -0.999741881)),
    'diff': (511, '23b91660998b42f5', (1.984665306, -1.487391504, -0.987328998)),
    'skid': (511, '23b91660998b42f5', (1.984665306, -1.487391504, -0.987328998)),
    '4ws': (307, 'ea20678a68e20aa6', (1.982791987, -1.490979641, -0.999741881)),
}


@pytest.mark.parametrize('drive', sorted(GOLDEN))
def test_profiles_without_modes_behave_as_before(drive):
    p = BaseProfile(drive=drive, max_lin=1.0, max_ang=1.5, max_lin_acc=0.8, max_ang_acc=1.2)
    assert p.modes == ()
    pose, _, trace, _ = simulate(p, (0.0, 0.0, 0.3), (2.0, -1.5, -1.0))
    h = hashlib.sha256(repr([(round(a, 12), round(b, 12), round(c, 12)) for _, a, b, c, _ in trace]).encode())
    n, digest, end = GOLDEN[drive]
    assert (len(trace), h.hexdigest()[:16]) == (n, digest)
    assert tuple(round(v, 9) for v in pose) == pytest.approx(end, abs=1e-9)


def test_derived_mode_matches_planner_split():
    for drive in ('omni', '4ws', 'diff', 'skid', 'whatever'):
        m = derive_mode(drive, *LIM)
        assert m.can_strafe == BaseProfile(drive=drive).holonomic
