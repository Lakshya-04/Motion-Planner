"""Sim-neutral overlay model. Pure python: no ROS, no simulator imports.

Input: visualization_msgs-like marker dicts (see ``marker_to_dict`` in marker_io.py)::

    {'type': 'LINE_STRIP'|'LINE_LIST'|'ARROW'|'CUBE'|'SPHERE'|'TEXT', 'ns': str, 'id': int,
     'action': 'ADD'|'DELETE'|'DELETEALL', 'pose': (x, y, z, qx, qy, qz, qw), 'scale': (x, y, z),
     'color': (r, g, b, a), 'points': [(x, y, z), ...], 'text': str}

Output: primitives in the simulator world frame, plain JSON-able dicts::

    {'kind': 'polyline', 'key': 'ns/id', 'ns': str, 'points': [[x,y,z],...], 'color': [r,g,b,a], 'width': m}
    {'kind': 'arrow', ... 'start': [..], 'end': [..], 'color', 'width'}
    {'kind': 'box', ... 'center': [..], 'size': [..], 'quat': [x,y,z,w], 'color', 'width'}
    {'kind': 'label', ... 'position': [..], 'text': str, 'color', 'height': m}

The marker frame -> sim world transform is a fixed ``FrameTransform`` (x, y, z, yaw), default identity.
"""
import math


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    q = (x / n, y / n, z / n, w / n)
    r = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (r[0], r[1], r[2])


class FrameTransform:
    """world = Rz(yaw) * p + (x, y, z)."""

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x, self.y, self.z, self.yaw = float(x), float(y), float(z), float(yaw)
        self.q = (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))

    def point(self, p):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [c * p[0] - s * p[1] + self.x, s * p[0] + c * p[1] + self.y, p[2] + self.z]

    def quat(self, q):
        return quat_mul(self.q, q)


def box_corners(center, size, quat):
    """8 corners; index bits: x (1), y (2), z (4), sign + when set."""
    out = []
    for i in range(8):
        d = [(size[k] / 2.0) * (1 if i & (1 << k) else -1) for k in range(3)]
        r = quat_rotate(quat, d)
        out.append([center[0] + r[0], center[1] + r[1], center[2] + r[2]])
    return out


_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]


def box_edges(center, size, quat):
    """The 12 box edges as [(p, q), ...]."""
    c = box_corners(center, size, quat)
    return [(c[a], c[b]) for a, b in _EDGES]


def box_polyline(center, size, quat):
    """One continuous polyline tracing every edge (16 points), for backends that draw only strips."""
    c = box_corners(center, size, quat)
    order = [0, 1, 3, 2, 0, 4, 5, 7, 6, 4, 5, 1, 3, 7, 6, 2]
    return [c[i] for i in order]


def arrow_polyline(start, end, head_ratio=0.25, head_angle_deg=25.0):
    """Shaft plus two head strokes as one strip: start, end, h1, end, h2 (heads lie in the xy plane)."""
    d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]]
    ln = math.sqrt(sum(c * c for c in d))
    if ln < 1e-9:
        return [list(start), list(end)]
    back = [-c / ln * ln * head_ratio for c in d]
    a = math.radians(head_angle_deg)
    pts = [list(start), list(end)]
    for sgn in (1.0, -1.0):
        ca, sa = math.cos(sgn * a), math.sin(sgn * a)
        h = [ca * back[0] - sa * back[1], sa * back[0] + ca * back[1], back[2]]
        pts += [[end[0] + h[0], end[1] + h[1], end[2] + h[2]]]
        if sgn > 0:
            pts.append(list(end))
    return pts


def _key(m):
    return '%s/%s' % (m.get('ns', ''), m.get('id', 0))


def convert_marker(m, tf=None):
    """One marker dict -> primitive dict (or None for unsupported types / ADD with no geometry)."""
    tf = tf or FrameTransform()
    t = m.get('type', '')
    pose = m.get('pose') or (0, 0, 0, 0, 0, 0, 1)
    scale = m.get('scale') or (1.0, 1.0, 1.0)
    col = list(m.get('color') or (1.0, 1.0, 1.0, 1.0))
    base = {'key': _key(m), 'ns': m.get('ns', ''), 'id': int(m.get('id', 0)), 'color': col}
    pos = tf.point(pose[:3])
    q = tf.quat(tuple(pose[3:7]))
    if t in ('LINE_STRIP', 'LINE_LIST'):
        pts = [tf.point(p) for p in (quat_apply_pose(pose, p) for p in m.get('points', []))]
        if len(pts) < 2:
            return None
        base.update(kind='polyline', points=pts, width=float(scale[0]), list=(t == 'LINE_LIST'))
        return base
    if t == 'ARROW':
        pts = m.get('points') or []
        if len(pts) >= 2:
            s, e = tf.point(quat_apply_pose(pose, pts[0])), tf.point(quat_apply_pose(pose, pts[1]))
        else:
            d = quat_rotate(q, (float(scale[0]), 0.0, 0.0))
            s, e = pos, [pos[0] + d[0], pos[1] + d[1], pos[2] + d[2]]
        base.update(kind='arrow', start=list(s), end=list(e), width=float(scale[1]) if len(scale) > 1 else 0.05)
        return base
    if t == 'CUBE':
        base.update(kind='box', center=list(pos), size=[float(c) for c in scale], quat=list(q), width=0.01)
        return base
    if t == 'SPHERE':
        base.update(kind='sphere', center=list(pos), radius=float(scale[0]) / 2.0)
        return base
    if t in ('TEXT', 'TEXT_VIEW_FACING'):
        base.update(kind='label', position=list(pos), text=m.get('text', ''), height=float(scale[2]))
        return base
    return None


def quat_apply_pose(pose, p):
    """Marker points are in the marker's own pose frame."""
    r = quat_rotate(tuple(pose[3:7]), p)
    return (r[0] + pose[0], r[1] + pose[1], r[2] + pose[2])


def apply_array(state, markers):
    """Apply one MarkerArray to ``state`` (dict key -> marker dict) with RViz semantics."""
    for m in markers:
        a = m.get('action', 'ADD')
        if a == 'DELETEALL':
            state.clear()
        elif a == 'DELETE':
            state.pop(_key(m), None)
        else:
            state[_key(m)] = m
    return state


class OverlayState:
    """Latest marker state per topic; ``primitives()`` is the union converted to the sim world."""

    def __init__(self, tf=None):
        self.tf = tf or FrameTransform()
        self.topics = {}

    def update(self, topic, markers):
        apply_array(self.topics.setdefault(topic, {}), markers)

    def primitives(self):
        out = []
        for topic in sorted(self.topics):
            for k in sorted(self.topics[topic]):
                p = convert_marker(self.topics[topic][k], self.tf)
                if p is not None:
                    p['key'] = '%s:%s' % (topic.strip('/').replace('/', '_'), p['key'])
                    out.append(p)
        return out
