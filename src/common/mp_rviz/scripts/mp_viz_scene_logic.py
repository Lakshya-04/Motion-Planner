"""Pure logic of mp_viz_scene.py (no rclpy): turn plan/stand data into plain marker specs.

Poses are (x, y, z, qx, qy, qz, qw) tuples, points (x, y, z). The node maps the specs onto visualization_msgs.
"""
import math

PALETTE = [
    (0.12, 0.47, 0.71), (1.00, 0.50, 0.05), (0.17, 0.63, 0.17), (0.84, 0.15, 0.16),
    (0.58, 0.40, 0.74), (0.55, 0.34, 0.29), (0.89, 0.47, 0.76), (0.09, 0.75, 0.81),
]
CONNECTOR_RGB = (0.6, 0.6, 0.6)


def palette(i):
    return PALETTE[int(i) % len(PALETTE)]


def is_connector(label, mode=''):
    text = ('%s %s' % (label, mode)).lower()
    return 'connector' in text or 'transit' in text


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rotate(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def wall_normal_from_tool(q):
    """Unit normal into the free side, taken as the opposite of the tool approach (+z) axis."""
    n = rotate(q, (0.0, 0.0, -1.0))
    norm = math.sqrt(sum(c * c for c in n)) or 1.0
    return tuple(c / norm for c in n)


def segment_polylines(segments, points, include_connectors=True):
    """segments: dicts with label, mode, start_index, end_index (inclusive; render_* if present).
    points: list of (x, y, z). Returns [{'label','points','connector','rgb'}], skipping empty or out-of-range ones.
    Strips take a palette colour by strip number; connectors are grey."""
    out = []
    strip = 0
    for seg in segments:
        conn = is_connector(seg.get('label', ''), seg.get('mode', ''))
        a = int(seg.get('render_start_index', seg.get('start_index', 0)))
        b = int(seg.get('render_end_index', seg.get('end_index', 0)))
        if a < 0 or b < a or b >= len(points):
            continue
        pts = list(points[a:b + 1])
        if len(pts) < 2 or (conn and not include_connectors):
            continue
        if conn:
            rgb = CONNECTOR_RGB
        else:
            rgb = palette(strip)
            strip += 1
        out.append({'label': seg.get('label', ''), 'points': pts, 'connector': conn, 'rgb': rgb})
    return out


def path_fallback(points):
    """One polyline for a plan that carries no segments."""
    return [{'label': 'path', 'points': list(points), 'connector': False, 'rgb': palette(0)}] if len(points) >= 2 else []


def subsample(items, max_count):
    """At most max_count items, evenly spread, first and last kept."""
    n = len(items)
    if max_count <= 0 or n <= max_count:
        return list(items)
    if max_count == 1:
        return [items[0]]
    return [items[round(i * (n - 1) / (max_count - 1))] for i in range(max_count)]


def stand_specs(stands, z=0.0):
    """stands: dicts x, y, yaw, standoff, footprint_clearance, travel_s, target_indices.
    Returns dicts with index, position, quat, label, rgb."""
    out = []
    for i, s in enumerate(stands):
        label = '#%d  %d tgt' % (i, len(s.get('target_indices', [])))
        if s.get('travel_s', 0.0) > 0.0:
            label += '  %.1fs' % s['travel_s']
        out.append({'index': i, 'position': (s['x'], s['y'], z), 'quat': yaw_to_quat(s.get('yaw', 0.0)),
                    'label': label, 'rgb': palette(i)})
    return out


def footprint_corners(x, y, yaw, length, width):
    """Closed rectangle (5 points, z=0) of length x width centred on (x, y), long side along yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    pts = []
    for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw), (hl, hw)):
        pts.append((x + c * dx - s * dy, y + s * dx + c * dy, 0.0))
    return pts


def circle_points(cx, cy, radius, n=48, z=0.0):
    return [(cx + radius * math.cos(2 * math.pi * k / n), cy + radius * math.sin(2 * math.pi * k / n), z)
            for k in range(n + 1)]


def reach_rgb(value, vmax):
    """Blue (0) to red (vmax) through green; value clipped."""
    t = 0.0 if vmax <= 0 else min(max(value / vmax, 0.0), 1.0)
    if t < 0.5:
        return (0.0, 2 * t, 1.0 - 2 * t)
    return (2 * (t - 0.5), 1.0 - 2 * (t - 0.5), 0.0)


# ---- wall layer -------------------------------------------------------------------------------------------------
def wall_corners(wall):
    """Four corners (x, y, z) of a wall_scene.json wall: origin + u*basis_u + v*basis_v over u_range_m x v_range_m.
    Order is counter-clockwise in (u, v): (u0,v0) (u1,v0) (u1,v1) (u0,v1)."""
    o, bu, bv = wall['origin_xyz'], wall['basis_u'], wall['basis_v']
    (u0, u1), (v0, v1) = wall['u_range_m'], wall['v_range_m']
    return [tuple(o[k] + u * bu[k] + v * bv[k] for k in range(3)) for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def wall_quad_triangles(wall):
    """Six points (two triangles) filling the wall rectangle, for a TRIANGLE_LIST."""
    c = wall_corners(wall)
    return [c[0], c[1], c[2], c[0], c[2], c[3]]


def wall_outline(wall):
    c = wall_corners(wall)
    return c + [c[0]]


def wall_back_outline(wall):
    """Outline of the wall's back face (front face moved back by thickness_m along the outward normal), [] without
    thickness_m. The front face is the wall's surface; with it the drawn wall has the world wall's thickness."""
    t = float(wall.get('thickness_m') or 0.0)
    if t <= 0.0:
        return []
    n = wall['normal_outward']
    return [tuple(c[k] - n[k] * t for k in range(3)) for c in wall_outline(wall)]


def wall_plane_distance(wall):
    """Distance of the wall's front plane from the frame origin, positive when the wall's outward normal points back at
    the origin (a wall at x = 0.915 facing -x gives 0.915)."""
    n, o = wall['normal_outward'], wall['origin_xyz']
    return -sum(n[k] * o[k] for k in range(3))


def wall_from_box(pos, quat_wxyz, half, wall_id=0, toward=(0.0, 0.0, 0.0)):
    """A wall_scene dict wall for a box (MuJoCo geom: pos, quat w x y z, half sizes). The thinnest axis is the thickness,
    the front face is the one facing `toward` (the robot); u is the horizontal in-plane axis, v = n x u."""
    w, x, y, z = quat_wxyz
    n2 = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n2, x / n2, y / n2, z / n2
    cols = [(1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)),
            (2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)),
            (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))]
    t_ax = min(range(3), key=lambda i: half[i])
    n = cols[t_ax]
    if sum(n[k] * (toward[k] - pos[k]) for k in range(3)) < 0:
        n = tuple(-c for c in n)
    ins = [i for i in range(3) if i != t_ax]
    ui = min(ins, key=lambda i: abs(cols[i][2]))   # the in-plane axis closest to horizontal
    vi = [i for i in ins if i != ui][0]
    u = cols[ui]
    v = (n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0])
    if v[2] > 0:   # v points down like the scene files (basis_v = -z for an upright wall)
        v = tuple(-c for c in v)
        u = (v[1] * n[2] - v[2] * n[1], v[2] * n[0] - v[0] * n[2], v[0] * n[1] - v[1] * n[0])
    t = half[t_ax]
    o = tuple(pos[k] + n[k] * t for k in range(3))
    return {'id': wall_id, 'origin_xyz': list(o), 'basis_u': list(u), 'basis_v': list(v), 'normal_outward': list(n),
            'u_range_m': [-half[ui], half[ui]], 'v_range_m': [-half[vi], half[vi]], 'thickness_m': 2.0 * t}


def wall_from_mjcf(path, geom='wall_0', toward=(0.0, 0.0, 0.0)):
    """The wall geom `geom` of a MuJoCo MJCF (type box, pos/quat/size; a top-level worldbody geom) as a scene wall."""
    import xml.etree.ElementTree as ET
    for g in ET.parse(path).getroot().iter('geom'):
        if g.get('name') == geom:
            pos = [float(c) for c in g.get('pos', '0 0 0').split()]
            quat = [float(c) for c in g.get('quat', '1 0 0 0').split()]
            half = [float(c) for c in g.get('size').split()][:3]
            return wall_from_box(pos, quat, half, toward=toward)
    raise KeyError('geom %s not in %s' % (geom, path))


def wall_area_m2(wall):
    (u0, u1), (v0, v1) = wall['u_range_m'], wall['v_range_m']
    return abs(u1 - u0) * abs(v1 - v0)


def wall_from_scene(scene, wall_id=0):
    """The wall dict with this id (or index) from a wall_scene.json dict; raises KeyError if none."""
    walls = scene.get('walls') or []
    for i, w in enumerate(walls):
        if w.get('id', i) == wall_id:
            return w
    raise KeyError('wall %s not in scene' % wall_id)


def plan_strip_area_m2(lines, strip_width):
    """Rough area swept by the plan: summed length of the non-connector polylines (segment_polylines output) times the
    tool strip width. Strokes drawn identically twice (return passes) count once; overlap between strips is not removed, so it is an upper bound."""
    total = 0.0
    seen = set()
    for ln in lines:
        if ln.get('connector'):
            continue
        pts = ln['points']
        key = tuple(sorted(tuple(round(c, 4) for c in q) for q in pts))
        if key in seen:  # a return pass drawn over the same render indices
            continue
        seen.add(key)
        total += sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    return total * strip_width


def densify_poses(poses, lines_idx, step):
    """poses: (x, y, z, qx, qy, qz, qw) tuples; lines_idx: (a, b) inclusive index ranges of the strokes. Returns the
    poses plus linearly interpolated positions (orientation of the first pose of each pair) every <= step metres
    inside each stroke. Without ranges, the poses are returned unchanged."""
    out = []
    for a, b in lines_idx:
        for k in range(a, b):
            p, q = poses[k], poses[k + 1]
            d = math.dist(p[:3], q[:3])
            n = max(1, int(math.ceil(d / step))) if step > 0 else 1
            for t in range(n):
                f = t / n
                out.append(tuple(p[m] + f * (q[m] - p[m]) if m < 3 else p[m] for m in range(7)))
        out.append(tuple(poses[b]))
    return out


def plan_bbox_area_m2(points, wall):
    """Area of the (u, v) bounding box of the plan points on the wall, clipped to nothing (informative only)."""
    if not points:
        return 0.0
    o, bu, bv = wall['origin_xyz'], wall['basis_u'], wall['basis_v']
    us = [sum((p[k] - o[k]) * bu[k] for k in range(3)) for p in points]
    vs = [sum((p[k] - o[k]) * bv[k] for k in range(3)) for p in points]
    return (max(us) - min(us)) * (max(vs) - min(vs))


# ---- reachable layer --------------------------------------------------------------------------------------------
def world_to_arm_base(pose, base_x, base_y, base_yaw, arm_base_z, arm_dx=0.0, arm_dy=0.0):
    """Express a world pose (x, y, z, qx, qy, qz, qw) in the arm base frame.
    The arm base sits at (base_x, base_y, arm_base_z) of the world, turned by base_yaw, shifted arm_dx/arm_dy in the
    base frame. Returns the pose tuple in that frame."""
    x, y, z, qx, qy, qz, qw = pose
    c, s = math.cos(-base_yaw), math.sin(-base_yaw)
    dx, dy = x - base_x, y - base_y
    lx, ly = c * dx - s * dy - arm_dx, s * dx + c * dy - arm_dy
    qb = yaw_to_quat(-base_yaw)
    return (lx, ly, z - arm_base_z) + quat_mul(qb, (qx, qy, qz, qw))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def reach_colour(reachable):
    """Green for reachable, red for not."""
    return (0.1, 0.8, 0.2) if reachable else (0.9, 0.1, 0.1)


def reach_summary(flags):
    n = len(flags)
    k = sum(1 for f in flags if f)
    return '%d/%d reachable (%.0f%%)' % (k, n, 100.0 * k / n if n else 0.0)


def transitions(segments, points, min_gap=0.01):
    """Joins between consecutive non-connector segments in execution order (start_index..end_index poses).
    Returns (joins, numbers): joins = [{'a','b'}] straight lines from the end of one stroke to the start of the next
    when they are more than min_gap apart (a plan without connector segments has none of its own); numbers =
    [{'n','at'}] visiting number at the start of each stroke that follows such a join (and the first stroke).
    Segments out of range are skipped."""
    joins, numbers = [], []
    prev_end = None
    n = 0
    for seg in segments:
        if is_connector(seg.get('label', ''), seg.get('mode', '')):
            continue
        a, b = int(seg.get('start_index', 0)), int(seg.get('end_index', 0))
        if a < 0 or b < a or b >= len(points):
            continue
        start, end = points[a], points[b]
        if prev_end is None or math.dist(prev_end, start) > min_gap:
            n += 1
            numbers.append({'n': n, 'at': start})
            if prev_end is not None:
                joins.append({'a': prev_end, 'b': start})
        prev_end = end
    return joins, numbers
