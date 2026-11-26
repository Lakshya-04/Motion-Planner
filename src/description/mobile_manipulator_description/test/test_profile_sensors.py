"""The `sensors:` block of the profiles: helper output and a schema check over every profile."""
import copy
import importlib.util
import json
import pathlib

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
PROFILES = sorted((PKG / 'config' / 'profiles').glob('*.yaml'))

_spec = importlib.util.spec_from_file_location('profile_sensors', PKG / 'scripts' / 'profile_sensors.py')
ps = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ps)

LIDAR = {
    'type': 'lidar2d', 'name': 'lidar', 'frame': 'lidar_link', 'factory': False, 'source': 'test',
    'mount': {'xyz': [0.3, 0.0, 0.2], 'rpy': [0, 0, 0]},
    'spec': {'angle_min_rad': -2.0, 'angle_max_rad': 2.0, 'angle_increment_rad': 0.01,
             'range_min_m': 0.1, 'range_max_m': 10.0, 'rate_hz': 40.0},
}
CAMERA = {
    'type': 'rgbd', 'name': 'cam', 'frame': 'cam_link', 'factory': True, 'source': 'test',
    'mount': {'xyz': [0.3, 0.0, 0.3], 'rpy': [0, 0, 0]},
    'spec': {'width': 640, 'height': 480, 'hfov_rad': 1.5, 'range_min_m': 0.3, 'range_max_m': 3.0,
             'rate_hz': 30.0, 'depth_noise_note': 'none'},
}


def _lidar(**changes):
    e = copy.deepcopy(LIDAR)
    for k, v in changes.items():
        if k in e['spec'] or k == 'range_noise_std_m':
            e['spec'][k] = v
        else:
            e[k] = v
    return e


def test_absent_block_is_empty():
    assert ps.sensors_from_profile({}) == []
    assert ps.load_sensors('ur10e_telescopic') == []


def test_valid_block_gives_neutral_dicts():
    out = ps.sensors_from_profile({'sensors': [LIDAR, CAMERA]}, 'p')
    assert [s['type'] for s in out] == ['lidar2d', 'rgbd']
    assert out[0]['xyz'] == [0.3, 0.0, 0.2]
    assert out[0]['spec']['range_max_m'] == 10.0
    assert 'range_noise_std_m' not in out[0]['spec']
    assert out[1]['depth_noise_note'] == 'none'
    json.dumps(out)


@pytest.mark.parametrize('entry, text', [
    (_lidar(type='sonar'), 'unknown type'),
    (_lidar(angle_min_rad=2.5), 'angle_min_rad must be below'),
    (_lidar(rate_hz=0.0), 'rate_hz must be positive'),
    (_lidar(range_max_m=float('inf')), 'not a finite number'),
    (_lidar(range_max_m=0.05), 'must exceed range_min_m'),
    (_lidar(range_min_m=-0.1), 'must be positive'),
    (_lidar(range_noise_std_m=-1.0), 'negative'),
    (_lidar(mount={'xyz': [0, 0], 'rpy': [0, 0, 0]}), 'mount.xyz needs three numbers'),
    (_lidar(mount={'xyz': [0, 0, 0], 'rpy': [0, 0, 'x']}), 'mount.rpy is not a finite number'),
    (_lidar(source=''), 'missing source'),
    (_lidar(factory='yes'), 'factory must be true or false'),
    (_lidar(frame='base_link'), 'frame must be unique'),
])
def test_validation_failures_name_profile_and_entry(entry, text):
    with pytest.raises(ps.SensorError) as e:
        ps.sensors_from_profile({'sensors': [entry]}, 'badprof')
    assert 'profile badprof' in str(e.value) and '(lidar)' in str(e.value)
    assert text in str(e.value)


def test_missing_spec_field_and_duplicates():
    e = copy.deepcopy(LIDAR)
    del e['spec']['rate_hz']
    with pytest.raises(ps.SensorError, match='spec.rate_hz missing'):
        ps.sensors_from_profile({'sensors': [e]})
    with pytest.raises(ps.SensorError, match='duplicate name'):
        ps.sensors_from_profile({'sensors': [LIDAR, LIDAR]})
    other = dict(copy.deepcopy(CAMERA), frame='lidar_link')
    with pytest.raises(ps.SensorError, match='frame must be unique'):
        ps.sensors_from_profile({'sensors': [LIDAR, other]})
    cam = copy.deepcopy(CAMERA)
    del cam['spec']['depth_noise_note']
    with pytest.raises(ps.SensorError, match='depth_noise_note'):
        ps.sensors_from_profile({'sensors': [cam]})


def test_cli_prints_json(capsys):
    assert ps.main(['ur10e_husky_telescopic']) == 0
    out = json.loads(capsys.readouterr().out)
    assert {s['type'] for s in out} == {'lidar2d', 'rgbd'}
    assert ps.main(['ur10e_husky_telescopic', '--factory-only']) == 0
    assert json.loads(capsys.readouterr().out) == []


@pytest.mark.parametrize('path', PROFILES, ids=lambda p: p.stem)
def test_every_profile_sensors_block_is_valid(path):
    profile = yaml.safe_load(path.read_text())
    sensors = ps.sensors_from_profile(profile, path.stem)
    for s in sensors:
        assert s['source'].strip()
