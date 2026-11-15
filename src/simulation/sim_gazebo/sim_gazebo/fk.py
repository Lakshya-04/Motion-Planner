"""Minimal URDF forward kinematics (serial chain, revolute / prismatic / fixed), numpy only.

Used by the smoke test to put the test wall where a known joint configuration reaches it,
without a running simulation.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, a):
    x, y, z = axis / np.linalg.norm(axis)
    c, s = math.cos(a), math.sin(a)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _floats(text, default):
    return [float(v) for v in text.split()] if text else default


def chain_pose(urdf_xml, tip_link, joint_positions, root=None):
    """4x4 pose of tip_link in the URDF root (or `root`) for the given {joint: value}."""
    robot = ET.fromstring(urdf_xml)
    by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
    chain, link = [], tip_link
    while link in by_child and link != root:
        j = by_child[link]
        chain.append(j)
        link = j.find('parent').get('link')
    T = np.eye(4)
    for j in reversed(chain):
        origin = j.find('origin')
        xyz = _floats(origin.get('xyz') if origin is not None else None, [0, 0, 0])
        rpy = _floats(origin.get('rpy') if origin is not None else None, [0, 0, 0])
        Tj = np.eye(4)
        Tj[:3, :3] = _rpy_matrix(*rpy)
        Tj[:3, 3] = xyz
        T = T @ Tj
        kind, name = j.get('type'), j.get('name')
        if kind in ('revolute', 'continuous', 'prismatic'):
            axis_el = j.find('axis')
            axis = np.array(_floats(axis_el.get('xyz') if axis_el is not None else None, [1, 0, 0]))
            q = float(joint_positions.get(name, 0.0))
            Tm = np.eye(4)
            if kind == 'prismatic':
                Tm[:3, 3] = axis / np.linalg.norm(axis) * q
            else:
                Tm[:3, :3] = _axis_angle(axis, q)
            T = T @ Tm
    return T
