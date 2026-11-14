#!/usr/bin/env python3
"""Emit an SRDF that disables collision between every pair of rigidly connected links.

Links joined only through fixed joints never move relative to each other, so any overlap between
their spheres is a modelling artifact (spherization bulges), never a real collision. Cricket derives
allowed pairs from an SRDF you give it and does not infer these, so without them a robot whose
mount chain is built from several fixed links (base, lift column, stages, arm mount) can end up with
every configuration "in collision".

    rigid_pairs_srdf.py [--hops N] robot.urdf [extra.srdf ...] > robot.srdf
Pairs from any extra SRDFs are carried over, and so are parent/child neighbours.

Rigid bodies are treated as one: a link is also allowed against every link that neighbours any link
rigidly attached to its partner (a sphere-less mount link between two sphere-bearing links would
otherwise leave that pair checked). With --hops N (default 1) links up to N joints apart, counting
each rigid group as one body, are allowed too. Spherization bulges, and in a chain such as
base -> shoulder -> upper arm the upper arm's spheres overlap the base's at every configuration
(the source model lists the same pairs as "never collides"). Use --hops 2 for a serial 6-DOF arm
(UR / Franka style) before handing the SRDF to Cricket.
"""
import itertools
import re
import sys
import xml.etree.ElementTree as ET


def pairs(urdf_path, extra_srdfs=(), hops=1):
    root = ET.parse(urdf_path).getroot()
    parent = {}
    fixed_edges = []
    allowed = set()
    for j in root.iter('joint'):
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        allowed.add(frozenset((p, c)))                      # neighbours
        if j.get('type') == 'fixed':
            fixed_edges.append((p, c))
    group = {l.get('name'): l.get('name') for l in root.iter('link')}

    def find(x):
        while group[x] != x:
            group[x] = group[group[x]]
            x = group[x]
        return x
    for p, c in fixed_edges:
        group[find(p)] = find(c)
    members = {}
    for name in group:
        members.setdefault(find(name), []).append(name)
    for names in members.values():
        for a, b in itertools.combinations(sorted(names), 2):
            allowed.add(frozenset((a, b)))
    # body graph: each rigid group is a node, joints between groups are edges
    edges = {}
    for j in root.iter('joint'):
        a, b = find(j.find('parent').get('link')), find(j.find('child').get('link'))
        if a != b:
            edges.setdefault(a, set()).add(b)
            edges.setdefault(b, set()).add(a)
    for g in members:
        seen, frontier = {g}, {g}
        for _ in range(hops):
            frontier = {n for f in frontier for n in edges.get(f, ())} - seen
            seen |= frontier
        for other in seen - {g}:
            for a in members[g]:
                for b in members[other]:
                    allowed.add(frozenset((a, b)))
    for path in extra_srdfs:
        for m in re.finditer(r'disable_collisions\s+link1="([^"]+)"\s+link2="([^"]+)"', open(path).read()):
            allowed.add(frozenset((m.group(1), m.group(2))))
    return sorted(tuple(sorted(p)) for p in allowed if len(p) == 2)


if __name__ == '__main__':
    args = sys.argv[1:]
    hops = 1
    if '--hops' in args:
        i = args.index('--hops')
        hops = int(args[i + 1])
        del args[i:i + 2]
    if not args:
        sys.exit(__doc__)
    sys.argv = [sys.argv[0]] + args
    name = ET.parse(sys.argv[1]).getroot().get('name', 'robot')
    print('<?xml version="1.0"?>')
    print(f'<robot name="{name}">')
    for a, b in pairs(sys.argv[1], sys.argv[2:], hops):
        print(f'  <disable_collisions link1="{a}" link2="{b}" reason="rigid_or_adjacent"/>')
    print('</robot>')
