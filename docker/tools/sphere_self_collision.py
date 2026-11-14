#!/usr/bin/env python3
"""Self-collision check on a FOAM-spherized URDF, no VAMP needed.

    sphere_self_collision.py spherized.urdf [robot.srdf]

Lists, for a few 6-DOF poses, the link pairs whose spheres overlap and are not disabled in the SRDF,
then the fraction of 500 random poses that are free. Revolute joints are taken in URDF order. A pose
that should be free (home) listing pairs means the SRDF is missing them: see rigid_pairs_srdf.py
(--hops 2 for a serial arm).
"""
import itertools
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, :3] = rpy_matrix(*map(float, origin.get('rpy', '0 0 0').split()))
        t[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return t


class Model:
    def __init__(self, urdf, srdf=None):
        root = ET.parse(urdf).getroot()
        self.joints = list(root.iter('joint'))
        self.by_child = {j.find('child').get('link'): j for j in self.joints}
        self.dof = [j.get('name') for j in self.joints if j.get('type') == 'revolute']
        self.spheres = {}
        for link in root.iter('link'):
            for c in link.findall('collision'):
                sphere, origin = c.find('.//sphere'), c.find('origin')
                if sphere is None:
                    continue        # not a sphere model element
                radius = float(sphere.get('radius'))
                centre = np.array(list(map(float, (origin.get('xyz') if origin is not None else '0 0 0').split())))
                self.spheres.setdefault(link.get('name'), []).append((centre, radius))
        self.allowed = set()
        if srdf:
            text = open(srdf).read()
            for m in re.finditer(r'link1="([^"]+)"\s+link2="([^"]+)"', text):
                self.allowed.add(frozenset((m[1], m[2])))

    def poses(self, q):
        q = dict(zip(self.dof, q))
        cache = {}

        def pose(link):
            if link in cache:
                return cache[link]
            j = self.by_child.get(link)
            if j is None:
                cache[link] = np.eye(4)
                return cache[link]
            p = pose(j.find('parent').get('link')) @ transform(j.find('origin'))
            if j.get('type') == 'revolute':
                a = np.array(list(map(float, j.find('axis').get('xyz').split())))
                a /= np.linalg.norm(a)
                k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                rot = np.eye(4)
                th = q[j.get('name')]
                rot[:3, :3] = np.eye(3) + np.sin(th) * k + (1 - np.cos(th)) * k @ k
                p = p @ rot
            cache[link] = p
            return p
        return pose

    def colliding_pairs(self, q):
        pose = self.poses(q)
        pts = {l: [((pose(l) @ np.append(c, 1))[:3], r) for c, r in s] for l, s in self.spheres.items()}
        bad = []
        for a, b in itertools.combinations(sorted(pts), 2):
            if frozenset((a, b)) in self.allowed:
                continue
            if any(np.linalg.norm(p - p2) < ra + rb for (p, ra), (p2, rb) in itertools.product(pts[a], pts[b])):
                bad.append((a, b))
        return bad


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    m = Model(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    poses = {'home': [0, -1.57, 1.57, -1.57, -1.57, 0], 'ready': [0, -1.2, 1.4, -1.8, -1.57, 0],
             'folded (should collide)': [0, 0, 3.1, 0, 0, 0]}
    if len(m.dof) != 6:
        print(f'{len(m.dof)} revolute joints found, the named 6-joint poses are UR10e-only: skipped')
        poses = {}
    for name, q in poses.items():
        print(name, m.colliding_pairs(q))
    rng = np.random.default_rng(0)
    free = sum(not m.colliding_pairs(rng.uniform(-np.pi, np.pi, len(m.dof))) for _ in range(500))
    print('random free', free, 'of 500')
