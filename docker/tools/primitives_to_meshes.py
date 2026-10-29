#!/usr/bin/env python3
"""Replace box / cylinder / sphere collision geometry in a URDF with STL meshes.

FOAM spherizes mesh collisions only and silently skips URDF primitives, so a robot built from
primitives (the generic arm, the lift column) would come out with no spheres at all. This writes one
STL per primitive next to the output URDF and points the collision <geometry> at it.

    primitives_to_meshes.py in.urdf out_dir      # writes out_dir/robot.urdf and out_dir/meshes/*.stl
"""
import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET


def _box(sx, sy, sz):
    x, y, z = sx / 2, sy / 2, sz / 2
    v = [(-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z), (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (3, 0, 4, 7)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _cylinder(radius, length, n=24):
    h = length / 2
    ring = [(radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n)) for i in range(n)]
    tris = []
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        tris += [((a[0], a[1], -h), (b[0], b[1], -h), (b[0], b[1], h)),
                 ((a[0], a[1], -h), (b[0], b[1], h), (a[0], a[1], h)),
                 ((0, 0, h), (a[0], a[1], h), (b[0], b[1], h)),
                 ((0, 0, -h), (b[0], b[1], -h), (a[0], a[1], -h))]
    return tris


def _sphere(radius, n=16):
    def p(i, j):
        th, ph = math.pi * i / n, 2 * math.pi * j / (2 * n)
        return (radius * math.sin(th) * math.cos(ph), radius * math.sin(th) * math.sin(ph), radius * math.cos(th))
    tris = []
    for i in range(n):
        for j in range(2 * n):
            a, b, c, d = p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1)
            tris += [(a, b, c), (a, c, d)]
    return tris


def _write_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'primitives_to_meshes'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            f.write(struct.pack('<3f', 0, 0, 0))
            for v in t:
                f.write(struct.pack('<3f', *v))
            f.write(b'\0\0')


def convert(urdf_in, out_dir):
    out = pathlib.Path(out_dir)
    (out / 'meshes').mkdir(parents=True, exist_ok=True)
    tree = ET.parse(urdf_in)
    count = 0
    for link in tree.getroot().iter('link'):
        for k, coll in enumerate(link.findall('collision')):
            geom = coll.find('geometry')
            shape = next((c for c in geom if c.tag in ('box', 'cylinder', 'sphere')), None)
            if shape is None:
                continue
            if shape.tag == 'box':
                tris = _box(*[float(v) for v in shape.get('size').split()])
            elif shape.tag == 'cylinder':
                tris = _cylinder(float(shape.get('radius')), float(shape.get('length')))
            else:
                tris = _sphere(float(shape.get('radius')))
            name = f"{link.get('name')}_{k}.stl"
            _write_stl(out / 'meshes' / name, tris)
            geom.remove(shape)
            ET.SubElement(geom, 'mesh', filename=f'meshes/{name}')
            count += 1
    tree.write(out / 'robot.urdf', xml_declaration=True, encoding='utf-8')
    return count


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    print(f'converted {convert(sys.argv[1], sys.argv[2])} primitive collision shapes')
