"""Turn a platform's `sensors:` suite into the lidar_* / depth_sensor_* launch values of the simulators.

Layers, last wins: built-in defaults, then the suite of `platform`, then arguments the user set.
No platform means the defaults plus the user's arguments, which is what the launches did before.
Pure python; both sim launch files call resolve_sensor_args().
"""
import math
import pathlib

import yaml

from .profile import profiles_dir
from .sensors import SensorError, sensors_from_profile

LIDAR_ARGS = ('lidar_pose', 'lidar_angle_min_deg', 'lidar_angle_max_deg', 'lidar_angle_increment_deg',
              'lidar_range_min_m', 'lidar_range_max_m', 'lidar_rate_hz', 'lidar_noise_m')
DEPTH_ARGS = ('depth_sensor_pose', 'depth_sensor_rate_hz', 'depth_sensor_noise_m')
DEFAULTS = {
    'lidar_pose': '0.45 0 0.2 0 0 0', 'lidar_angle_min_deg': '-135.0', 'lidar_angle_max_deg': '135.0',
    'lidar_angle_increment_deg': '0.5', 'lidar_range_min_m': '0.05', 'lidar_range_max_m': '10.0',
    'lidar_rate_hz': '10.0', 'lidar_noise_m': '0.0',
    'depth_sensor_pose': '0.1 0 1.15 0 0 0', 'depth_sensor_rate_hz': '5.0', 'depth_sensor_noise_m': '0.0',
    'lidar_frame': 'lidar_link', 'depth_sensor_frame': 'depth_sensor_link',
}


class SuiteError(ValueError):
    """The platform is unknown, or it has no sensor of the kind that was asked for."""


def _fmt(value):
    return format(value, '.6g')


def _pose(xyz, rpy):
    return ' '.join(_fmt(v) for v in list(xyz) + list(rpy))


def _profiles():
    return sorted(pathlib.Path(profiles_dir()).glob('*.yaml'))


def _load_platform(platform):
    """(name, profile dict) for a profile name, a path, or the name of a base that one profile with sensors uses."""
    path = pathlib.Path(platform)
    if path.suffix != '.yaml':
        path = pathlib.Path(profiles_dir()) / f'{platform}.yaml'
    if path.is_file():
        return path.stem, yaml.safe_load(path.read_text()) or {}
    by_base = []
    for p in _profiles():
        data = yaml.safe_load(p.read_text()) or {}
        if data.get('sensors') and (data.get('base') or {}).get('name') == platform:
            by_base.append((p.stem, data))
    if len(by_base) == 1:
        return by_base[0]
    if by_base:
        raise SuiteError(f'platform {platform} matches several profiles ({", ".join(n for n, _ in by_base)}); '
                         'name one of them')
    known = [p.stem for p in _profiles() if (yaml.safe_load(p.read_text()) or {}).get('sensors')]
    raise SuiteError(f'unknown platform {platform}; profiles with a sensor suite: {", ".join(known)}')


def _body_offset(profile, name):
    """base_link in base_footprint: the body box centre, half way between the ground clearance and the lift mount."""
    base = profile.get('base') or {}
    try:
        return [0.0, 0.0, (float(base['ground_clearance_m']) + float(base['lift_mount_height_m'])) / 2.0]
    except (KeyError, TypeError, ValueError):
        raise SuiteError(f'platform {name}: base needs ground_clearance_m and lift_mount_height_m to place '
                         'the sensors (their mounts are in base_link)') from None


def suite_values(platform):
    """Launch values from a platform's first lidar2d and first rgbd; keys of a missing sensor are left out.

    Poses are in base_footprint (the sims' root frame): the suite's base_link mounts shifted up by the body offset.
    """
    name, profile = _load_platform(platform)
    sensors = sensors_from_profile(profile, name)
    if (profile.get('frames') or {}).get('world_base', 'base_footprint') != 'base_footprint':
        raise SuiteError(f'platform {name}: the sensor mounts assume the root frame base_footprint')
    dz = _body_offset(profile, name) if sensors else [0.0, 0.0, 0.0]
    out = {}
    lidar = next((s for s in sensors if s['type'] == 'lidar2d'), None)
    if lidar:
        sp = lidar['spec']
        xyz = [a + b for a, b in zip(lidar['xyz'], dz)]
        out.update(lidar_pose=_pose(xyz, lidar['rpy']), lidar_frame=lidar['frame'],
                   lidar_angle_min_deg=_fmt(math.degrees(sp['angle_min_rad'])),
                   lidar_angle_max_deg=_fmt(math.degrees(sp['angle_max_rad'])),
                   lidar_angle_increment_deg=_fmt(math.degrees(sp['angle_increment_rad'])),
                   lidar_range_min_m=_fmt(sp['range_min_m']), lidar_range_max_m=_fmt(sp['range_max_m']),
                   lidar_rate_hz=_fmt(sp['rate_hz']))
        if 'range_noise_std_m' in sp:
            out['lidar_noise_m'] = _fmt(sp['range_noise_std_m'])
    cam = next((s for s in sensors if s['type'] == 'rgbd'), None)
    if cam:
        xyz = [a + b for a, b in zip(cam['xyz'], dz)]
        out.update(depth_sensor_pose=_pose(xyz, cam['rpy']), depth_sensor_frame=cam['frame'],
                   depth_sensor_rate_hz=_fmt(cam['spec']['rate_hz']))
    return out


def resolve_sensor_args(explicit, platform='', want_lidar=False, want_depth=False):
    """All lidar_* / depth_sensor_* values as strings, plus lidar_frame and depth_sensor_frame.

    `explicit` maps argument names to what the user passed; an empty string means not passed.
    """
    values = dict(DEFAULTS)
    if platform:
        suite = suite_values(platform)
        if want_lidar and 'lidar_pose' not in suite:
            raise SuiteError(f'platform {platform} has no lidar2d in its sensors')
        if want_depth and 'depth_sensor_pose' not in suite:
            raise SuiteError(f'platform {platform} has no rgbd sensor in its sensors')
        values.update(suite)
    values.update({k: v for k, v in explicit.items() if v != '' and k in values})
    return values


__all__ = ['DEFAULTS', 'DEPTH_ARGS', 'LIDAR_ARGS', 'SensorError', 'SuiteError', 'resolve_sensor_args',
           'suite_values']
