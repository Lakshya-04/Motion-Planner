#!/usr/bin/env python3
"""Generate the structural allowed-collision entries (SRDF <disable_collisions>) for one arm.

Rendered from the URDF, so it works for any arm that follows the description contracts:

  * Adjacent: two links that carry collision geometry and are neighbours in the kinematic tree
    are never checked against each other. Neighbours are found through links with no collision
    geometry (frames such as lift_output_link or arm_base_link), like MoveIt's setup assistant
    treats fixed-joint chains.
  * Lift: all lift links (column, telescoping stages, carriage) overlap by design, so every pair
    among them is excluded.

  * Always (opt in with --sample N, default 0): like the Setup Assistant's "always in collision" test, N seeded
    random configurations within the joint limits are checked against the primitive collision geometry, and a
    pair of non-lift links that overlaps in every sample (and is not already adjacent) is excluded with
    reason="Always". That is a modelling artefact of the primitives (two link cylinders that overlap whatever
    the joints do, such as the xArm7's link 2 and link 4, whose tubes are a few cm apart), never a contact that
    could be avoided. A pair that overlaps in only some samples stays checked.

Pairs that are never in collision are not enumerated (the check for them is cheap).

  generate_acm.py --arm ur10e --out srdf/acm_ur.xacro
  generate_acm.py --arm generic_6dof --out srdf/acm_generic_6dof.xacro
  generate_acm.py --arm ur10e --base husky --out /tmp/acm_husky.xacro   (wheel/body pairs too)
  generate_acm.py --arm ur10e --lift none --out srdf/acm_ur_none.xacro
  generate_acm.py --arm ur10e --lift scissor --out srdf/acm_ur_scissor.xacro

`--lift` selects the lift variant (telescopic and fixed share the default files). A moving scissor's
lift-internal pairs follow lift_stages, so the SRDF writes them itself and its file holds only the
other pairs. The SRDF picks the
file by arm family and lift (the lift-internal pairs of a moving scissor are written by the SRDF itself
from lift_stages, so a scissor file holds only the other pairs), see srdf/mobile_manipulator.srdf.xacro. `--set key=value` passes extra
xacro args (a profile's lift_* values), for matrices of a custom profile.
"""
import argparse
import itertools
import os
import sys

import numpy as np
import xacro
from urdf_parser_py.urdf import URDF

LIFT_PREFIXES = ('lift_',)


def collision_links(robot):
    return {link.name for link in robot.links if link.collision is not None}


def adjacent_pairs(robot):
    with_geometry = collision_links(robot)
    parent_of = {j.child: j.parent for j in robot.joints}
    pairs = set()
    for link in with_geometry:
        parent = parent_of.get(link)
        while parent is not None:
            if parent in with_geometry:
                pairs.add(tuple(sorted((link, parent))))
                break
            parent = parent_of.get(parent)
    return pairs


def lift_pairs(robot, mounted=True):
    lift = sorted(n for n in collision_links(robot) if n.startswith(LIFT_PREFIXES))
    pairs = {tuple(p) for p in itertools.combinations(lift, 2)}
    # Whatever is bolted to the lift (the arm's root link on the carriage, the chassis under the column)
    # sits inside the stages' envelope for part of the stroke: the arm base vs stage 2 truncated every
    # contact path. Those links never collide with the lift by construction.
    mounted_links = set()
    for a, b in adjacent_pairs(robot) if mounted else ():
        if (a in lift) != (b in lift):
            mounted_links.add(b if a in lift else a)
    # the next link (the shoulder) turns right beside the column and rides in the same envelope
    first = set(mounted_links)
    for a, b in adjacent_pairs(robot) if mounted else ():
        if a in first and a.startswith('arm_') and b not in lift:
            mounted_links.add(b)
        elif b in first and b.startswith('arm_') and a not in lift:
            mounted_links.add(a)
    for link in mounted_links:
        pairs |= {tuple(sorted((link, n))) for n in lift}
    return pairs


def arm_never_pairs(robot, arm):
    """Pairs of one arm family that cannot touch on the real arm, though their collision primitives overlap.

    UR family: the forearm against wrist 2 and wrist 3. The wrist 1 link separates them, and the primitive
    cylinders of the description overlap at wrist-1 angles the real arm reaches without touching. In the Gazebo
    two-stand run that pair made the dock joint goal 'invalid' (OMPL: 'Unable to sample any valid states for goal
    tree'), refused 79 of 140 plans from a start state with the pair overlapping, and truncated the Cartesian
    segments. Only pairs between links that exist and carry geometry are returned."""
    if not arm.startswith('ur'):
        return set()
    have = collision_links(robot)
    pairs = {('arm_forearm_link', 'arm_wrist_2_link'), ('arm_forearm_link', 'arm_wrist_3_link')}
    return {p for p in pairs if p[0] in have and p[1] in have}


def _rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = _rot(rpy or (0, 0, 0))
    m[:3, 3] = xyz or (0, 0, 0)
    return m


def _link_cylinders(robot):
    """{link: [(origin 4x4 in the link frame, radius, length)]} of the cylinder collision shapes."""
    out = {}
    for link in robot.links:
        shapes = []
        for c in link.collisions:
            g = c.geometry
            if type(g).__name__ != 'Cylinder':
                continue
            o = c.origin
            shapes.append((_tf(o.xyz if o else None, o.rpy if o else None), g.radius, g.length))
        if shapes:
            out[link.name] = shapes
    return out


def _forward(robot, q):
    """Link-frame transforms for the joint values `q` (missing joints at 0)."""
    joint_of = {j.child: j for j in robot.joints}
    cache = {}

    def get(name):
        if name in cache:
            return cache[name]
        j = joint_of.get(name)
        if j is None:
            cache[name] = np.eye(4)
            return cache[name]
        m = get(j.parent) @ _tf(j.origin.xyz if j.origin else None, j.origin.rpy if j.origin else None)
        v = q.get(j.name, 0.0)
        if j.type in ('revolute', 'continuous'):
            k = np.array(j.axis if j.axis else (0, 0, 1), dtype=float)
            k = k / np.linalg.norm(k)
            kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            rot = np.eye(4)
            rot[:3, :3] = np.eye(3) + np.sin(v) * kx + (1 - np.cos(v)) * kx @ kx
            m = m @ rot
        elif j.type == 'prismatic':
            m = m.copy()
            m[:3, 3] += m[:3, :3] @ (v * np.array(j.axis if j.axis else (1, 0, 0), dtype=float))
        cache[name] = m
        return m
    return get


def _fill_points(radius, length, n_axis=9, n_ring=8):
    """Points filling a cylinder (axis = z): the shell and the axis, ends included."""
    pts = []
    for z in np.linspace(-length / 2, length / 2, n_axis):
        pts.append((0.0, 0.0, z))
        for rr in (radius * 0.5, radius):
            for a in np.linspace(0, 2 * np.pi, n_ring, endpoint=False):
                pts.append((rr * np.cos(a), rr * np.sin(a), z))
    return np.array(pts)


def _inside(points, frame, radius, length):
    """Which world `points` lie inside the cylinder placed at `frame`."""
    local = (points - frame[:3, 3]) @ frame[:3, :3]
    return (np.hypot(local[:, 0], local[:, 1]) <= radius) & (np.abs(local[:, 2]) <= length / 2)


def overlap_fractions(robot, samples, seed=0, skip=lambda a, b: False):
    """Fraction of `samples` seeded random configurations (arm joints within their limits, the others at 0)
    in which each pair of links with cylinder collision shapes overlaps. Overlap = a filled-cylinder sample
    point of one lies inside a cylinder of the other (a crossing without a contained point counts as no overlap,
    which can only keep a pair checked)."""
    shapes = _link_cylinders(robot)
    names = sorted(shapes)
    pairs = [(a, b) for a, b in itertools.combinations(names, 2) if not skip(a, b)]
    free = [j for j in robot.joints if j.type in ('revolute', 'prismatic') and j.limit is not None
            and j.mimic is None and j.name.startswith('arm_')]
    rng = np.random.default_rng(seed)
    hits = {p: 0 for p in pairs}
    clouds = {n: [_fill_points(r, ln) for _, r, ln in shapes[n]] for n in names}
    for _ in range(samples):
        q = {}
        for j in free:
            lo, hi = j.limit.lower, j.limit.upper
            if j.type == 'revolute' and (lo is None or hi is None or hi - lo > 2 * np.pi):
                lo, hi = -np.pi, np.pi
            q[j.name] = rng.uniform(lo, hi)
        get = _forward(robot, q)
        world = {}
        for n in names:
            m = get(n)
            world[n] = [(m @ o, r, ln, (m @ o)[:3, :3] @ c.T + (m @ o)[:3, 3:4])
                        for (o, r, ln), c in zip(shapes[n], clouds[n])]
        for a, b in pairs:
            hit = False
            for fa, ra, la, pa in world[a]:
                for fb, rb, lb, pb in world[b]:
                    if _inside(pa.T, fb, rb, lb).any() or _inside(pb.T, fa, ra, la).any():
                        hit = True
                        break
                if hit:
                    break
            hits[(a, b)] += hit
    return {p: h / samples for p, h in hits.items()}


def always_pairs(robot, samples, adjacent, lift, seed=0):
    """Non-lift pairs, not already excluded, that overlap in every one of `samples` configurations."""
    def skip(a, b):
        return (a, b) in adjacent or (a, b) in lift or a.startswith(LIFT_PREFIXES) or b.startswith(LIFT_PREFIXES) \
            or a.startswith('base') or b.startswith('base')
    frac = overlap_fractions(robot, samples, seed, skip)
    return {p for p, f in frac.items() if f >= 1.0}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--arm', required=True)
    ap.add_argument('--base', default='generic_base', help='base:=<name> of the description')
    ap.add_argument('--lift', default='telescopic')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE', help='extra xacro args')
    ap.add_argument('--sample', type=int, default=0, metavar='N',
                    help='also exclude (reason Always) non-lift pairs that overlap in all of N random configurations')
    ap.add_argument('--out', required=True)
    ap.add_argument('--xacro', default=os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'mobile_manipulator.urdf.xacro'))
    args = ap.parse_args()
    if 0 < args.sample < 100:
        ap.error('--sample needs at least 100 configurations: with a handful of samples a pair that can really collide '
                 'overlaps in all of them by chance and would be disabled (the committed files use 200)')

    mappings = {'arm': args.arm, 'lift': args.lift, 'base': args.base}
    mappings.update(kv.split('=', 1) for kv in args.set)
    robot = URDF.from_xml_string(xacro.process_file(args.xacro, mappings=mappings).toxml())
    adjacent = adjacent_pairs(robot)
    lift = lift_pairs(robot) - adjacent
    if args.lift == 'scissor' and not mappings.get('lift_fixed_height'):
        # a moving scissor has a pair of links per level: the SRDF writes those pairs from lift_stages
        lift = set()
        adjacent = {p for p in adjacent if not all(n.startswith(LIFT_PREFIXES) for n in p)}
    if args.lift == 'scissor' and 'lift_fixed_height' not in dict(kv.split('=', 1) for kv in args.set):
        # a moving scissor has one pair of links per level: the SRDF writes those pairs from lift_stages
        lift = set()
        adjacent = {p for p in adjacent if not all(n.startswith(LIFT_PREFIXES) for n in p)}

    always = always_pairs(robot, args.sample, adjacent, lift) if args.sample else set()

    lines = [
        '<?xml version="1.0"?>',
        f'<!-- Generated by scripts/generate_acm.py for arm family of `{args.arm}`, base `{args.base}`, lift `{args.lift}`. '
        'Do not edit by hand; regenerate. -->',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
    ]
    for a, b in sorted(adjacent):
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>')
    for a, b in sorted(lift | arm_never_pairs(robot, args.arm)):
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Never"/>')
    if always:
        lines.append(f'  <!-- Overlap in all {args.sample} sampled configurations: a primitive-geometry artefact, not a contact. -->')
    for a, b in sorted(always):
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Always"/>')
    lines.append('</robot>')
    with open(args.out, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'{args.out}: {len(adjacent)} adjacent + {len(lift)} lift + {len(always)} always pairs')
    return 0


if __name__ == '__main__':
    sys.exit(main())
