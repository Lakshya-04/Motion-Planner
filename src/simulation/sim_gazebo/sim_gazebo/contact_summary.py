"""Sum of the contact wrench on the tool from a gz Contacts message, plus what went into the sum.

ROS-free (duck-typed on the message fields) so it is unit-testable without rclpy. Used by
scripts/contact_ground_truth.py. The sum is the same as before when no name filter is given; the
statistics are new and exist to tell a real force from a counting artefact (duplicate contact points,
contacts with something other than the wall) in a recorded run.
"""
import numpy as np


def _matches(name, patterns):
    return any(p in name for p in patterns)


def tool_wrench_world(contacts, tcp_pos, tool_body='tool_tip_link', include_other=(), exclude_other=()):
    """(force, torque about tcp_pos, stats), world axes, from a Contacts message.

    gz leaves the body names of the wrench empty: the collision entity names say which side is the tool.
    include_other / exclude_other: substrings of the OTHER collision's name; with include_other only
    matching pairs are summed, a pair matching exclude_other is dropped (both empty: every pair).
    stats: contacts (pairs with the tool that were summed), points, filtered (pairs dropped by the
    filter), others (sorted names of the other collisions summed), sum_point_norm (sum of |f| over points),
    max_point_norm, dup_points (points at a position already seen in the same message).
    """
    force, torque = np.zeros(3), np.zeros(3)
    stats = {'contacts': 0, 'points': 0, 'filtered': 0, 'sum_point_norm': 0.0, 'max_point_norm': 0.0,
             'dup_points': 0}
    others, seen = set(), set()
    for c in contacts.contacts:
        if tool_body in c.collision1.name:
            side, other = 1, c.collision2.name
        elif tool_body in c.collision2.name:
            side, other = 2, c.collision1.name
        else:
            continue
        if (include_other and not _matches(other, include_other)) or _matches(other, exclude_other):
            stats['filtered'] += 1
            continue
        stats['contacts'] += 1
        others.add(other)
        for pos, w in zip(c.positions, c.wrenches):
            wr = w.body_1_wrench if side == 1 else w.body_2_wrench
            f = np.array([wr.force.x, wr.force.y, wr.force.z])
            t = np.array([wr.torque.x, wr.torque.y, wr.torque.z])
            p = np.array([pos.x, pos.y, pos.z])
            key = tuple(np.round(p, 6))
            if key in seen:
                stats['dup_points'] += 1
            seen.add(key)
            force += f
            torque += t + np.cross(p - tcp_pos, f)
            n = float(np.linalg.norm(f))
            stats['points'] += 1
            stats['sum_point_norm'] += n
            stats['max_point_norm'] = max(stats['max_point_norm'], n)
    stats['others'] = sorted(others)
    return force, torque, stats
