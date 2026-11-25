"""ROS-free base path planner for the stand navigator.

Plans a collision-free path for the base footprint (the rectangle of the profile `base.footprint`) over a 2D
occupancy grid built from a wall scene (the wall_detector JSON: `walls` and `obstacles`), inflated by the
footprint's circumscribed radius. Search is 8-connected A* on the grid, followed by line-of-sight shortcutting
(any-angle smoothing). The grid guarantee holds for every heading (circumscribed disc); the start and goal
neighbourhoods, which the inflation would otherwise close (a stand is close to the wall by design), are opened
only where the exact footprint at that endpoint's yaw is collision-free. Everything is in one frame (the odom
frame); no transform is done. It is a classic grid A* plus shortcutting (the textbook any-angle post-process),
not hybrid-A*: the diff base turns in place in `StandController`, so no kinematic lattice is needed.

Pieces: `scene_obstacles`, `footprint_corners`, `straight_line_free`, `plan_path`, `time_parametrize`
and `PathFollower` (waypoint following with the existing `StandController`).
"""
import heapq
import math
from dataclasses import dataclass, field

from stand_navigator.core import StandController, wrap

DEFAULT_FOOTPRINT = ((-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3))
_WALL_THICKNESS = 0.02


# ---------------------------------------------------------------- geometry
def footprint_corners(footprint, pose, margin=0.0):
    """Footprint polygon (list of (x, y)) at pose (x, y, yaw), grown by `margin` about its centre."""
    fp = footprint or DEFAULT_FOOTPRINT
    cx = sum(p[0] for p in fp) / len(fp)
    cy = sum(p[1] for p in fp) / len(fp)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    out = []
    for px, py in fp:
        dx, dy = px - cx, py - cy
        n = math.hypot(dx, dy)
        if n > 1e-9 and margin:
            dx, dy = dx * (n + margin) / n, dy * (n + margin) / n
        qx, qy = cx + dx, cy + dy
        out.append((pose[0] + c * qx - s * qy, pose[1] + s * qx + c * qy))
    return out


def circumscribed_radius(footprint, margin=0.0):
    fp = footprint or DEFAULT_FOOTPRINT
    return max(math.hypot(p[0], p[1]) for p in fp) + margin


def _axes(poly):
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        n = math.hypot(y2 - y1, x1 - x2)
        if n > 1e-12:
            yield (y1 - y2) / n, (x2 - x1) / n


def polygons_overlap(a, b):
    """Separating-axis test for two convex polygons."""
    for ax, ay in list(_axes(a)) + list(_axes(b)):
        pa = [ax * x + ay * y for x, y in a]
        pb = [ax * x + ay * y for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def _seg_point_dist2(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    return (px - x1 - t * dx) ** 2 + (py - y1 - t * dy) ** 2


def point_polygon_dist(px, py, poly):
    """Distance from a point to a convex polygon (0 inside)."""
    inside = True
    sign = 0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        cr = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
        if abs(cr) > 1e-12:
            sg = 1 if cr > 0 else -1
            if sign and sg != sign:
                inside = False
            sign = sg
    if inside:
        return 0.0
    return math.sqrt(min(_seg_point_dist2(px, py, *poly[i], *poly[(i + 1) % len(poly)])
                         for i in range(len(poly))))


def _thick_segment(p, q, t=_WALL_THICKNESS):
    dx, dy = q[0] - p[0], q[1] - p[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return None
    nx, ny = -dy / n * t / 2, dx / n * t / 2
    return [(p[0] + nx, p[1] + ny), (q[0] + nx, q[1] + ny), (q[0] - nx, q[1] - ny), (p[0] - nx, p[1] - ny)]


def scene_obstacles(scene):
    """Floor-plane convex polygons from a wall scene dict (`walls`, `obstacles`).

    A wall is a vertical plane (|normal z| < 0.3): its floor line runs from origin + u0*basis_u to
    origin + u1*basis_u (`u_range_m`), thickened to 2 cm. An obstacle protrudes `protrusion_max_m` from its
    parent wall along the outward normal over its `uv_on_wall.u` range. Heights are ignored: a wall or obstacle
    anywhere in z blocks the base (conservative).
    """
    polys, walls = [], {}
    for w in scene.get('walls') or []:
        abcd = w.get('plane_abcd') or [0, 0, 0, 0]
        if abs(abcd[2]) >= 0.3:
            continue
        o, u = w.get('origin_xyz'), w.get('basis_u')
        if not o or not u:
            continue
        u0, u1 = w.get('u_range_m', [-1.0, 1.0])
        n = w.get('normal_outward') or [-abcd[0], -abcd[1], 0.0]
        walls[w.get('id')] = (o, u, n)
        poly = _thick_segment((o[0] + u[0] * u0, o[1] + u[1] * u0), (o[0] + u[0] * u1, o[1] + u[1] * u1))
        if poly:
            polys.append(poly)
    for ob in scene.get('obstacles') or []:
        parent = walls.get(ob.get('parent_wall'))
        uv = ob.get('uv_on_wall') or {}
        if not parent or 'u' not in uv:
            continue
        o, u, n = parent
        a, b = uv['u']
        d = float(ob.get('protrusion_max_m', 0.0))
        nn = math.hypot(n[0], n[1]) or 1.0
        nx, ny = n[0] / nn * d, n[1] / nn * d
        p0 = (o[0] + u[0] * a, o[1] + u[1] * a)
        p1 = (o[0] + u[0] * b, o[1] + u[1] * b)
        if d > 1e-6 and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > 1e-6:
            polys.append([p0, p1, (p1[0] + nx, p1[1] + ny), (p0[0] + nx, p0[1] + ny)])
    return polys


def pose_free(footprint, pose, obstacles, margin=0.0):
    poly = footprint_corners(footprint, pose, margin)
    return not any(polygons_overlap(poly, ob) for ob in obstacles)


def _heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def straight_line_free(profile_drive_holonomic, footprint, start, goal, obstacles, margin=0.0, step=0.05):
    """Exact check of the straight drive start -> goal (footprint poses, yaw interpolated for omni, along the
    line for diff). True if no sample hits an obstacle."""
    if not obstacles:
        return True
    d = math.hypot(goal[0] - start[0], goal[1] - start[1])
    n = max(1, int(math.ceil(d / step)))
    dyaw = wrap(goal[2] - start[2])
    head = _heading(start, goal) if d > 1e-6 else start[2]
    if not pose_free(footprint, start, obstacles, margin) or not pose_free(footprint, goal, obstacles, margin):
        return False
    for i in range(1, n):
        f = i / n
        x, y = start[0] + f * (goal[0] - start[0]), start[1] + f * (goal[1] - start[1])
        yaw = wrap(start[2] + f * dyaw) if profile_drive_holonomic else head
        if not pose_free(footprint, (x, y, yaw), obstacles, margin):
            return False
    return True


# ---------------------------------------------------------------- grid + A*
@dataclass
class PlanResult:
    ok: bool
    path: list = field(default_factory=list)     # [(x, y), ...] smoothed, start .. goal
    length: float = 0.0
    straight: float = 0.0
    reason: str = ''


def path_length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


class _Grid:
    def __init__(self, footprint, start, goal, obstacles, res, margin):
        self.res = res
        self.fp = footprint
        self.obs = obstacles
        self.margin = margin
        self.r = circumscribed_radius(footprint, margin)
        pts = [start[:2], goal[:2]] + [p for ob in obstacles for p in ob]
        pad = self.r + 1.0
        self.x0 = min(p[0] for p in pts) - pad
        self.y0 = min(p[1] for p in pts) - pad
        self.nx = int(math.ceil((max(p[0] for p in pts) + pad - self.x0) / res))
        self.ny = int(math.ceil((max(p[1] for p in pts) + pad - self.y0) / res))
        self.blocked = bytearray(self.nx * self.ny)
        for ob in obstacles:
            xs, ys = [p[0] for p in ob], [p[1] for p in ob]
            i0, i1 = self._ix(min(xs) - self.r), self._ix(max(xs) + self.r)
            j0, j1 = self._iy(min(ys) - self.r), self._iy(max(ys) + self.r)
            for i in range(max(i0, 0), min(i1, self.nx - 1) + 1):
                for j in range(max(j0, 0), min(j1, self.ny - 1) + 1):
                    cx, cy = self.center(i, j)
                    if point_polygon_dist(cx, cy, ob) <= self.r:
                        self.blocked[i * self.ny + j] = 1
        self.ends = [(start, start[2]), (goal, goal[2])]
        self.cache = {}

    def _ix(self, x):
        return int(math.floor((x - self.x0) / self.res))

    def _iy(self, y):
        return int(math.floor((y - self.y0) / self.res))

    def center(self, i, j):
        return self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res

    def free(self, i, j):
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return False
        if not self.blocked[i * self.ny + j]:
            return True
        key = (i, j)
        if key not in self.cache:
            cx, cy = self.center(i, j)
            best, ok = None, False
            for pose, yaw in self.ends:        # relaxed only near an endpoint, exact footprint at its yaw
                d = math.hypot(cx - pose[0], cy - pose[1])
                if d <= self.r + self.res and (best is None or d < best):
                    best = d
                    ok = pose_free(self.fp, (cx, cy, yaw), self.obs, self.margin)
            self.cache[key] = ok
        return self.cache[key]

    def point_free(self, x, y):
        return self.free(self._ix(x), self._iy(y))

    def seg_free(self, p, q):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(1, int(math.ceil(d / (self.res * 0.5))))
        return all(self.point_free(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n)
                   for k in range(n + 1))


def _astar(g, s, t):
    sq2 = math.sqrt(2.0)
    nbrs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, sq2), (1, -1, sq2), (-1, 1, sq2), (-1, -1, sq2))

    def h(i, j):
        dx, dy = abs(i - t[0]), abs(j - t[1])
        return max(dx, dy) + (sq2 - 1.0) * min(dx, dy)

    best = {s: 0.0}
    prev = {}
    heap = [(h(*s), 0.0, s)]
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if cur == t:
            out = [cur]
            while cur in prev:
                cur = prev[cur]
                out.append(cur)
            return out[::-1]
        if gc > best.get(cur, 1e18):
            continue
        for di, dj, c in nbrs:
            nb = (cur[0] + di, cur[1] + dj)
            if not g.free(*nb):
                continue
            if di and dj and not (g.free(cur[0] + di, cur[1]) and g.free(cur[0], cur[1] + dj)):
                continue          # no corner cutting
            ng = gc + c
            if ng < best.get(nb, 1e18):
                best[nb] = ng
                prev[nb] = cur
                heapq.heappush(heap, (ng + h(*nb), ng, nb))
    return None


def plan_path(footprint, start, goal, obstacles, resolution=0.05, margin=0.02):
    """Collision-free xy path for the footprint from start to goal (footprint poses (x, y, yaw))."""
    straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
    g = _Grid(footprint, start, goal, obstacles, resolution, margin)
    s = (g._ix(start[0]), g._iy(start[1]))
    t = (g._ix(goal[0]), g._iy(goal[1]))
    if not g.free(*s):
        return PlanResult(False, straight=straight, reason='start pose is in collision')
    if not g.free(*t):
        return PlanResult(False, straight=straight, reason='goal pose is in collision')
    cells = _astar(g, s, t)
    if cells is None:
        return PlanResult(False, straight=straight, reason='no collision-free path')
    pts = [tuple(start[:2])] + [g.center(*c) for c in cells[1:-1]] + [tuple(goal[:2])]
    # shortcut: from each kept point jump to the farthest point it can see
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not g.seg_free(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return PlanResult(True, out, path_length(out), straight)


# ---------------------------------------------------------------- time parametrisation and following
def pose_sequence(path, start_yaw, goal_yaw, holonomic):
    """Yaw per waypoint: omni spreads the turn over the path length, diff follows the segment heading and
    takes the start/goal yaw at the ends (the controller turns in place there)."""
    n = len(path)
    if n == 1:
        return [(path[0][0], path[0][1], goal_yaw)]
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    out = []
    for k, p in enumerate(path):
        if holonomic:
            yaw = wrap(start_yaw + (cum[k] / cum[-1] if cum[-1] > 1e-9 else 1.0) * wrap(goal_yaw - start_yaw))
        elif k == 0:
            yaw = start_yaw
        elif k == n - 1:
            yaw = goal_yaw
        else:
            yaw = _heading(path[k], path[k + 1])
        out.append((p[0], p[1], yaw))
    return out


def time_parametrize(poses, profile, dt=0.05):
    """Speed- and acceleration-limited timed pose sequence [(t, x, y, yaw), ...] along the path (trapezoidal
    speed profile on arc length; slowed so the yaw rate stays below `max_ang`, and the whole move takes at least
    the turn time at `max_ang_acc`)."""
    cum = [0.0]
    for a, b in zip(poses, poses[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = cum[-1]
    yaw_total = sum(abs(wrap(b[2] - a[2])) for a, b in zip(poses, poses[1:]))
    vmax, acc = profile.max_lin, profile.max_lin_acc
    if total < 1e-9:
        return [(0.0, *poses[0])] if poses else []
    # slow the cruise so that yaw_total/T <= max_ang with T the trapezoid duration
    def duration(v):
        ramp = v * v / acc
        return (total / v + v / acc) if ramp <= total else 2.0 * math.sqrt(total / acc)
    v = vmax
    for _ in range(40):
        if yaw_total / duration(v) <= profile.max_ang * 0.999 or v < 0.02:
            break
        v *= 0.9
    T = duration(v)
    ta = v / acc
    if v * ta > total:           # triangular profile
        ta = math.sqrt(total / acc)
        v = acc * ta

    def arc(t):
        if t <= ta:
            return 0.5 * acc * t * t
        if t <= T - ta:
            return 0.5 * acc * ta * ta + v * (t - ta)
        r = T - t
        return total - 0.5 * acc * r * r

    out, k, t = [], 0, 0.0
    while True:
        s = total if t >= T else arc(t)
        while k < len(cum) - 2 and cum[k + 1] < s:
            k += 1
        seg = cum[k + 1] - cum[k]
        f = 0.0 if seg < 1e-12 else max(0.0, min(1.0, (s - cum[k]) / seg))
        a, b = poses[k], poses[k + 1]
        out.append((t, a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]),
                    wrap(a[2] + f * wrap(b[2] - a[2]))))
        if t >= T:
            break
        t = min(t + dt, T)
    return out


class PathFollower:
    """Follow a waypoint path with `StandController`: the controller is aimed at a carrot point `lookahead`
    metres further along the path (the path's final pose once it is within reach), so the base neither stops at
    every corner nor cuts them. Interface mirrors the controller (`step`, `arrived`, `errors`, `phase`)."""

    def __init__(self, profile, path, goal, tol=None, lookahead=None, **ctl_kwargs):
        self.path, self.final = list(path), goal
        self.ctl = StandController(profile, goal, tol, **ctl_kwargs)
        self.tol, self.p = self.ctl.tol, profile
        self.look = lookahead if lookahead is not None else max(
            0.3, profile.max_lin ** 2 / (2.0 * profile.max_lin_acc) + 0.1)
        self.idx = 0           # first path segment not yet passed
        self.cmd = (0.0, 0.0, 0.0)

    @property
    def phase(self):
        return self.ctl.phase

    def errors(self, pose):
        return self.ctl.errors(pose)

    def arrived(self, pose, lin_speed=0.0, ang_speed=0.0):
        return self.ctl.arrived(pose, lin_speed, ang_speed)

    def remaining(self, pose):
        """Path length left from the pose's projection on the path."""
        q, i = self._project(pose)
        return math.hypot(self.path[i + 1][0] - q[0], self.path[i + 1][1] - q[1]) + path_length(self.path[i + 1:])

    def _project(self, pose):
        best, bi, bq = 1e18, self.idx, self.path[self.idx]
        for i in range(self.idx, min(self.idx + 4, len(self.path) - 1)):
            a, b = self.path[i], self.path[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            l2 = dx * dx + dy * dy
            t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((pose[0] - a[0]) * dx + (pose[1] - a[1]) * dy) / l2))
            q = (a[0] + t * dx, a[1] + t * dy)
            d = math.hypot(pose[0] - q[0], pose[1] - q[1])
            if d < best - 1e-9:
                best, bi, bq = d, i, q
        self.idx = bi
        return bq, bi

    def carrot(self, pose):
        q, i = self._project(pose)
        left = self.look
        cur = q
        for k in range(i + 1, len(self.path)):
            seg = math.hypot(self.path[k][0] - cur[0], self.path[k][1] - cur[1])
            if seg >= left:
                f = left / seg if seg > 1e-12 else 0.0
                return (cur[0] + f * (self.path[k][0] - cur[0]), cur[1] + f * (self.path[k][1] - cur[1]))
            left -= seg
            cur = self.path[k]
        return None            # within reach of the end

    def step(self, pose, dt):
        c = self.carrot(pose)
        self.ctl.goal = self.final if c is None else (c[0], c[1], self.final[2])
        self.ctl.cmd = self.cmd
        self.cmd = self.ctl.step(pose, dt)
        return self.cmd
