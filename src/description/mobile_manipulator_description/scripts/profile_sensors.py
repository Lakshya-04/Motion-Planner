#!/usr/bin/env python3
"""Read the `sensors:` block of a profile YAML and print one neutral dict per sensor as JSON.

    profile_sensors.py ur10e_husky_telescopic            # name in config/profiles, or a path
    profile_sensors.py profile.yaml --factory-only

Each entry: type, name, frame, xyz, rpy (mount in base_link), factory, source, spec (numbers of the
type) and, for rgbd, depth_noise_note. A profile without the block gives an empty list. The same
rules as the C++ reader (mobile_manipulator_profile); no simulator reads this yet.
"""
import argparse
import json
import math
import pathlib
import sys

import yaml

PROFILES = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'profiles'

SPEC_FIELDS = {
    'lidar2d': (('angle_min_rad', 'angle_max_rad', 'angle_increment_rad', 'range_min_m',
                 'range_max_m', 'rate_hz'), ('range_noise_std_m',)),
    'rgbd': (('width', 'height', 'hfov_rad', 'range_min_m', 'range_max_m', 'rate_hz'), ()),
    'imu': (('rate_hz',), ()),
    'lidar3d': (('channels', 'hfov_rad', 'vfov_rad', 'range_min_m', 'range_max_m', 'rate_hz'), ()),
}
SIGNED = ('angle_min_rad', 'angle_max_rad')


class SensorError(ValueError):
    """A sensors entry is missing a field or has a bad value."""


def _num(value, what, fail):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        fail(f'{what} is not a finite number')
    return float(value)


def sensors_from_profile(profile, name='profile'):
    """Validate profile['sensors'] and return the neutral list of dicts."""
    block = profile.get('sensors')
    if block is None:
        return []
    if not isinstance(block, list):
        raise SensorError(f'profile {name}: sensors must be a list')
    out, names, frames = [], set(), set()
    for i, entry in enumerate(block):
        where = f'sensors[{i}]' + (f' ({entry["name"]})' if isinstance(entry, dict) and 'name' in entry else '')

        def fail(what, where=where):
            raise SensorError(f'profile {name}: {where}: {what}')

        if not isinstance(entry, dict):
            fail('must be a map')
        for key in ('type', 'name', 'frame', 'source'):
            if not isinstance(entry.get(key), str) or not entry[key]:
                fail(f'missing {key}')
        if entry['type'] not in SPEC_FIELDS:
            fail(f'unknown type {entry["type"]}')
        if entry['name'] in names:
            fail('duplicate name')
        if entry['frame'] == 'base_link' or entry['frame'] in frames:
            fail('frame must be unique and not base_link')
        names.add(entry['name'])
        frames.add(entry['frame'])
        if not isinstance(entry.get('factory'), bool):
            fail('factory must be true or false')
        mount = entry.get('mount')
        if not isinstance(mount, dict):
            fail('missing mount')
        pose = {}
        for key in ('xyz', 'rpy'):
            v = mount.get(key)
            if not isinstance(v, list) or len(v) != 3:
                fail(f'mount.{key} needs three numbers')
            pose[key] = [_num(x, f'mount.{key}', fail) for x in v]
        spec = entry.get('spec')
        if not isinstance(spec, dict):
            fail('missing spec')
        required, optional = SPEC_FIELDS[entry['type']]
        values = {}
        for key in required:
            if key not in spec:
                fail(f'spec.{key} missing')
            values[key] = _num(spec[key], f'spec.{key}', fail)
        for key in optional:
            if key in spec:
                values[key] = _num(spec[key], f'spec.{key}', fail)
        for key, v in values.items():
            if key in SIGNED:
                continue
            if key == 'range_noise_std_m':
                if v < 0:
                    fail('spec.range_noise_std_m is negative')
            elif v <= 0:
                fail(f'spec.{key} must be positive')
        if entry['type'] == 'lidar2d' and values['angle_min_rad'] >= values['angle_max_rad']:
            fail('angle_min_rad must be below angle_max_rad')
        if 'range_max_m' in values and values['range_max_m'] <= values['range_min_m']:
            fail('range_max_m must exceed range_min_m')
        item = {'type': entry['type'], 'name': entry['name'], 'frame': entry['frame'],
                'xyz': pose['xyz'], 'rpy': pose['rpy'], 'factory': entry['factory'],
                'source': entry['source'], 'spec': values}
        if entry['type'] == 'rgbd':
            if values['hfov_rad'] >= math.pi:
                fail('hfov_rad must be below pi')
            note = spec.get('depth_noise_note')
            if not isinstance(note, str) or not note:
                fail('spec.depth_noise_note missing')
            item['depth_noise_note'] = note
        out.append(item)
    return out


def load_sensors(name_or_path):
    """Sensors of a profile given by name (config/profiles/<name>.yaml) or by path."""
    path = pathlib.Path(name_or_path)
    if path.suffix != '.yaml':
        path = PROFILES / f'{name_or_path}.yaml'
    with open(path) as f:
        return sensors_from_profile(yaml.safe_load(f) or {}, path.stem)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('profile', help='profile name or path to a .yaml')
    ap.add_argument('--factory-only', action='store_true', help='drop the add-on sensors')
    args = ap.parse_args(argv)
    try:
        sensors = load_sensors(args.profile)
    except (OSError, SensorError) as e:
        print(f'profile_sensors: {e}', file=sys.stderr)
        return 1
    if args.factory_only:
        sensors = [s for s in sensors if s['factory']]
    print(json.dumps(sensors, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
