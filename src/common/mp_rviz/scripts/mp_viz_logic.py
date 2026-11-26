"""Pure logic of mp_viz_markers.py (no rclpy): force to colour, a capped trace buffer, the text of the readout."""
from collections import deque
import math


def force_magnitude(fx, fy, fz):
    return math.sqrt(fx * fx + fy * fy + fz * fz)


def force_to_rgb(force_n, limit_n):
    """Green at 0 N, yellow at half the limit, red at the limit and above. Returns (r, g, b) in 0..1.

    A non-positive limit or a non-finite force gives grey (nothing to compare against)."""
    if not (limit_n > 0.0) or not math.isfinite(force_n):
        return (0.5, 0.5, 0.5)
    t = min(max(force_n / limit_n, 0.0), 1.0)
    if t <= 0.5:
        return (2.0 * t, 1.0, 0.0)
    return (1.0, 2.0 * (1.0 - t), 0.0)


class TraceBuffer:
    """The last `max_points` (x, y, z, force_n) samples; a sample closer than `min_step_m` to the last one is dropped."""

    def __init__(self, max_points, min_step_m=0.0):
        if max_points < 1:
            raise ValueError('max_points must be >= 1')
        self._pts = deque(maxlen=int(max_points))
        self._min_step = float(min_step_m)

    def add(self, x, y, z, force_n):
        if self._pts and self._min_step > 0.0:
            lx, ly, lz, _ = self._pts[-1]
            if math.dist((x, y, z), (lx, ly, lz)) < self._min_step:
                return False
        self._pts.append((x, y, z, force_n))
        return True

    def clear(self):
        self._pts.clear()

    def __len__(self):
        return len(self._pts)

    def points(self):
        return list(self._pts)


def format_readout(state, stand_index, stand_count, force_n, limit_n, column=None):
    """The three-line text: state, stand (and column), force against the limit. None means not received yet."""
    s = state if state else 'no mission feedback'
    if stand_count:
        stand = 'stand %d/%d' % (stand_index + 1, stand_count)
    else:
        stand = 'stand -'
    if column is not None and column >= 0:
        stand += '  column %d' % column
    if force_n is None or not math.isfinite(force_n):
        f = 'force - N'
    else:
        f = 'force %.1f N' % force_n
        if limit_n > 0.0:
            f += ' / %.0f N' % limit_n
            if force_n > limit_n:
                f += '  OVER LIMIT'
    return '%s\n%s\n%s' % (s, stand, f)


def stroke_colour(force_n, target_n=10.0, band_n=4.0, limit_n=50.0, lost_n=2.0):
    """Colour of one piece of the stroke line, by the contact force at that point (r, g, b in 0..1).

    Blue: contact lost (below lost_n). Cyan: touching but below the band. Green: inside target +- band. Yellow to red: above the
    band, red at limit_n and above. Grey: force unknown (None or not finite)."""
    if force_n is None or not math.isfinite(force_n):
        return (0.5, 0.5, 0.5)
    if force_n < lost_n:
        return (0.1, 0.3, 1.0)
    if force_n < target_n - band_n:
        return (0.0, 0.8, 0.9)
    if force_n <= target_n + band_n:
        return (0.0, 1.0, 0.0)
    span = max(limit_n - (target_n + band_n), 1e-9)
    t = min(max((force_n - (target_n + band_n)) / span, 0.0), 1.0)
    return (1.0, 1.0 - t, 0.0)


def line_list_points(points):
    """Vertices for a LINE_LIST marker from a polyline in which ``None`` marks a break (the tool left the wall); the points are
    (x, y, z) or (x, y, z, force). Returns (vertices, forces): each run of consecutive points becomes segments between
    neighbours, nothing joins two runs, and both vertices of a segment carry the force of its later point (None without one)."""
    verts, forces = [], []
    prev = None
    for pt in points:
        if pt is None:
            prev = None
            continue
        if prev is not None:
            f = pt[3] if len(pt) > 3 else None
            verts.append(tuple(prev[:3]))
            verts.append(tuple(pt[:3]))
            forces.extend([f, f])
        prev = pt
    return verts, forces
