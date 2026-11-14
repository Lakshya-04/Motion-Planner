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
otherwise leave that pair checked). With --hops N (default 1) links up to N joints apart are allowed
too: a rigid group counts as one body for its direct neighbours, further out every joint (fixed ones
included) counts, so a big rigid platform (a fixed lift) does not hide an arm link swinging into it. Spherization bulges, and in a chain such as
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
    for j in root.findall('joint'):   # direct children only: <transmission> also holds <joint> names
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
    for j in root.findall('joint'):
        a, b = find(j.find('parent').get('link')), find(j.find('child').get('link'))
        if a != b:
            edges.setdefault(a, set()).add(b)
            edges.setdefault(b, set()).add(a)
    # link graph: every joint (fixed ones too) is one step
    link_edges = {}
    for j in root.iter('joint'):
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        link_edges.setdefault(p, set()).add(c)
        link_edges.setdefault(c, set()).add(p)

    def link_distances(start):
        dist, frontier = {start: 0}, [start]
        while frontier:
            nxt = []
            for f in frontier:
                for n in link_edges.get(f, ()):
                    if n not in dist:
                        dist[n] = dist[f] + 1
                        nxt.append(n)
            frontier = nxt
        return dist

    for g in members:
        seen, frontier = {g}, {g}
        rings = {}
        for ring in range(1, hops + 1):
            frontier = {n for f in frontier for n in edges.get(f, ())} - seen
            seen |= frontier
            for n in frontier:
                rings[n] = ring
        for other, ring in rings.items():
            for a in members[g]:
                far = link_distances(a) if ring > 1 else None
                for b in members[other]:
                    # Second ring and beyond: a rigid group is one body only for its own neighbours.
                    # Further out the pair must really be close along the chain, otherwise a big rigid
                    # platform (fixed lift: chassis, column, carriage, arm base) would hide the upper
                    # arm swinging into the chassis.
                    if ring == 1 or far.get(b, hops + 1) <= hops:
                        allowed.add(frozenset((a, b)))
    for path in extra_srdfs:
        for m in re.finditer(r'disable_collisions\s+link1="([^"]+)"\s+link2="([^"]+)"', open(path).read()):
            allowed.add(frozenset((m.group(1), m.group(2))))
    return sorted(tuple(sorted(p)) for p in allowed if len(p) == 2)


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='Disable collision between rigidly connected / nearby links.')
    ap.add_argument('--hops', type=int, default=1, help='allow links up to N joints apart (default 1)')
    ap.add_argument('urdf')
    ap.add_argument('extra_srdf', nargs='*')
    args = ap.parse_args()
    name = ET.parse(args.urdf).getroot().get('name', 'robot')
    print('<?xml version="1.0"?>')
    print(f'<robot name="{name}">')
    for a, b in pairs(args.urdf, args.extra_srdf, args.hops):
        print(f'  <disable_collisions link1="{a}" link2="{b}" reason="rigid_or_adjacent"/>')
    print('</robot>')
