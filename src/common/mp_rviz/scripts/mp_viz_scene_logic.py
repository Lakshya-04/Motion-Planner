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
