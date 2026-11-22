#!/usr/bin/env python3
"""Turn a profile YAML into the xacro arguments that render its robot.

`robot.xacro_args` is passed as is; the profile's `lift:` section then fills the lift arguments
(lift, lift_stroke, lift_stages, lift_max_velocity, lift_max_acceleration, lift_mount_height) and wins
over a lift_* entry in xacro_args, so a profile states each lift value once. An optional
`lift.geometry` section describes the column with primitives (lift_shape, lift_column_size,
lift_stage_sizes, lift_stage_rest_z, lift_carriage_size; see urdf/lift/telescopic_lift.xacro):

    geometry:
      shape: box                                   # cylinder (default) | box
      column: {size_m: [x, y, height]}             # the fixed outer tube
      stages:                                      # outermost first, one entry per stage
        - {size_m: [x, y, height], rest_z_m: z}    # z: bottom above the column's bottom, retracted
      carriage: {size_m: [x, y, thickness]}

    profile_xacro_args.py profile.yaml          # prints  key:=value  pairs, one per line
"""
import sys

import yaml

LIFT_KEYS = {
    'type': 'lift',
    'stroke_m': 'lift_stroke',
    'stages': 'lift_stages',
    'max_velocity_mps': 'lift_max_velocity',
    'max_acceleration_mps2': 'lift_max_acceleration',
    'mount_height_m': 'lift_mount_height',
}


def _numbers(values):
    return ' '.join(repr(float(v)) for v in values)


def geometry_args(geometry):
    """The xacro arguments of a `lift.geometry` section (empty section: none)."""
    args = {}
    if not geometry:
        return args
    if 'shape' in geometry:
        args['lift_shape'] = str(geometry['shape'])
    if 'column' in geometry:
        args['lift_column_size'] = _numbers(geometry['column']['size_m'])
    stages = geometry.get('stages') or []
    if stages:
        args['lift_stage_sizes'] = _numbers(v for st in stages for v in st['size_m'])
        args['lift_stage_rest_z'] = _numbers(st['rest_z_m'] for st in stages)
    if 'carriage' in geometry:
        args['lift_carriage_size'] = _numbers(geometry['carriage']['size_m'])
    return args


def xacro_args(profile):
    args = {k: str(v) for k, v in (profile.get('robot', {}).get('xacro_args') or {}).items()}
    lift = profile.get('lift') or {}
    for key, arg in LIFT_KEYS.items():
        if key in lift:
            args[arg] = str(lift[key])
    args.update(geometry_args(lift.get('geometry')))
    return args


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for k, v in xacro_args(load(sys.argv[1])).items():
        print(f'{k}:={v}')
