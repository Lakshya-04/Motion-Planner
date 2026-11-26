"""Mode-aware base planning: a feasible path plus the drive mode of each piece, from a start pose to a goal pose.

ROS-free and deterministic. Poses are base_footprint (x, y, yaw) in the odom frame; obstacles are the floor
polygons of `path_planner.scene_obstacles`. The planner only emits motion the active mode allows:

- omni: grid path as today (straight when free), yaw spread over the path.
- crab: the same xy path at a constant heading; a yaw change needs a spinning mode (rotate first, else last).
- diff_turn_in_place, skid, spin: rotate in place to each leg's heading, drive straight, rotate to the final yaw.
- ackermann: Hybrid A* (arcs at or above the minimum turn radius, forward and reverse) with a Reeds-Shepp
  connection to the exact goal pose. If none fits the radius the plan says so; it never returns a path that
  breaks the radius. The turn radius is taken at base_footprint.

Several modes (Ranger Mini): every strategy the mode set allows is planned and the one with the shortest
estimated duration wins, ties by listing order (translate, crab, spin and drive, ackermann). With the sourced
Ranger Mini numbers that means Ackermann for long moves and crab for short sideways ones. If a `dock_dir` is
given (world angle of the final move, e.g. toward the wall) and the base has a crab mode, the last
`crab_final_m` metres (shrunk by halves while blocked) are a crab leg along it at the goal yaw, and the
travel before it is planned to that leg's start.
"""
import heapq
import math
from dataclasses import dataclass, field

from stand_navigator import reeds_shepp as rs
from stand_navigator.core import wrap
from stand_navigator.path_planner import (
    _Grid, circumscribed_radius, plan_path, pose_free, pose_sequence, straight_line_free)

RADIUS_HEADROOM = 1.15   # plan arcs this much wider than the limit so tracking can still steer
DS = 0.05             # spacing of the emitted poses, m (and rad for rotations)


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    gear: int = 1          # +1 forward, -1 reverse (the body x axis is the front)
    curv: float = 0.0      # signed path curvature, yaw rate = curv * signed speed

    @property
    def xy(self):
        return (self.x, self.y)


@dataclass
class ModeSegment:
    mode: str              # DriveMode name
    kind: str              # rotate | drive | arc | translate | crab
    poses: list

    def length(self):
        return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(self.poses, self.poses[1:]))

    def sweep(self):
        return sum(wrap(b.yaw - a.yaw) for a, b in zip(self.poses, self.poses[1:]))


@dataclass
class ModePlan:
    ok: bool
    segments: list = field(default_factory=list)
    reason: str = ''
    strategy: str = ''
    duration: float = 0.0
    notes: list = field(default_factory=list)

    @property
    def waypoints(self):
        return [p for s in self.segments for p in s.poses]

    @property
    def modes(self):
        return [s.mode for s in self.segments]

    def length(self):
        return sum(s.length() for s in self.segments)


# ---------------------------------------------------------------- collision
class _Checker:
    def __init__(self, footprint, obstacles, margin):
        self.fp, self.obs, self.margin = footprint, obstacles, margin
        self.r = circumscribed_radius(footprint, margin)
        self.boxes = [(min(p[0] for p in o), max(p[0] for p in o), min(p[1] for p in o), max(p[1] for p in o))
                      for o in obstacles]

    def free(self, pose):
        x, y = pose[0], pose[1]
        r = self.r
        near = [o for o, b in zip(self.obs, self.boxes)
                if b[0] - r <= x <= b[1] + r and b[2] - r <= y <= b[3] + r]
        return not near or pose_free(self.fp, pose, near, self.margin)

    def poses_free(self, poses):
        return all(self.free((p.x, p.y, p.yaw)) for p in poses)


def _line(a, b, yaw, ds=DS):
    n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / ds)))
    return [Waypoint(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, yaw) for i in range(n + 1)]


def _arc_points(pose, k, d, ds=DS):
    """Poses along an arc of signed length d and curvature k from `pose` (excludes the start)."""
    n = max(1, int(math.ceil(abs(d) / ds)))
    step = d / n
    gear = 1 if d >= 0 else -1
    x, y, th = pose
    out = []
    for _ in range(n):
        if abs(k) < 1e-12:
            x, y = x + step * math.cos(th), y + step * math.sin(th)
        else:
            nth = th + k * step
            x += (math.sin(nth) - math.sin(th)) / k
            y -= (math.cos(nth) - math.cos(th)) / k
            th = nth
        out.append(Waypoint(x, y, th, gear, k))
    return out


# ---------------------------------------------------------------- pieces: rotation and straight drives
def _rotation(chk, mode, xy, y0, y1):
    """Rotate-in-place segment from y0 to y1 about xy: the short way, else the long way. None if blocked."""
    d = wrap(y1 - y0)
    for sweep in ((d, d - math.copysign(2 * math.pi, d)) if abs(d) > 1e-9 else (0.0,)):
        n = max(1, int(math.ceil(abs(sweep) / DS)))
        poses = [Waypoint(xy[0], xy[1], wrap(y0 + sweep * i / n)) for i in range(n + 1)]
        if chk.poses_free(poses):
            return ModeSegment(mode.name, 'rotate', poses)
    return None


def _xy_path(footprint, start, goal, obstacles, holonomic, res, margin):
    """Straight line when the exact check passes, else the grid path (as the navigator does today)."""
    if straight_line_free(holonomic, footprint, start, goal, obstacles, margin):
        return [tuple(start[:2]), tuple(goal[:2])], ''
    r = plan_path(footprint, start, goal, obstacles, res, margin)
    return (r.path, '') if r.ok else (None, r.reason)


def _dedupe(path):
    out = [path[0]]
    for p in path[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-9:
            out.append(p)
    return out


def _plan_translate(modes, chk, footprint, start, goal, obstacles, res, margin):
    """Omni: one translate segment with yaw spread along it."""
    m = next(md for md in modes if md.name == 'omni')
    path, why = _xy_path(footprint, start, goal, obstacles, True, res, margin)
    if path is None:
        return None, why
    path = _dedupe(path)
    seq = pose_sequence(path, start[2], goal[2], True) if len(path) > 1 else [tuple(goal)]
    poses = []
    for a, b in zip(seq, seq[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / DS)))
        for i in range(n):
            f = i / n
            poses.append(Waypoint(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), wrap(a[2] + f * wrap(b[2] - a[2]))))
    poses.append(Waypoint(goal[0], goal[1], goal[2]))
    if not chk.poses_free(poses):
        return None, 'omni path hits an obstacle with the exact footprint'
    return [ModeSegment(m.name, 'translate', poses)], ''


def _spinner(modes, avoid=None):
    return next((m for m in modes if m.can_spin_in_place and m.name != avoid), None)


def _plan_crab(modes, chk, footprint, start, goal, obstacles, res, margin):
    """Crab: constant heading along the xy path; the yaw change is a rotation first (else last)."""
    crab = next(md for md in modes if md.name == 'crab')
    need_rot = abs(wrap(goal[2] - start[2])) > 1e-3
    spin = _spinner(modes)
    if need_rot and spin is None:
        return None, 'crab cannot change yaw and no mode can spin in place'
    for rot_first in ((True, False) if need_rot else (True,)):
        yaw = goal[2] if rot_first else start[2]
        a = (start[0], start[1], yaw)
        b = (goal[0], goal[1], yaw)
        path, why = _xy_path(footprint, a, b, obstacles, True, res, margin)
        if path is None:
            return None, why
        path = _dedupe(path)
        poses = []
        for p, q in zip(path, path[1:]):
            poses += _line(p, q, yaw)[:-1]
        poses.append(Waypoint(goal[0], goal[1], yaw))
        if not chk.poses_free(poses):
            continue
        rot = None
        if need_rot:
            rot = (_rotation(chk, spin, start[:2], start[2], goal[2]) if rot_first
                   else _rotation(chk, spin, goal[:2], start[2], goal[2]))
            if rot is None:
                continue
        seg = ModeSegment(crab.name, 'crab', poses)
        return ([rot, seg] if rot_first and rot else [seg, rot] if rot else [seg]), ''
    return None, 'crab path or the rotation it needs is blocked'


def _plan_spin_drive(modes, chk, footprint, start, goal, obstacles, res, margin, rot_mode, drive_mode):
    path, why = _xy_path(footprint, start, goal, obstacles, False, res, margin)
    if path is None:
        return None, why
    path = _dedupe(path)
    segs, yaw = [], start[2]
    for p, q in zip(path, path[1:]):
        head = math.atan2(q[1] - p[1], q[0] - p[0])
        if abs(wrap(head - yaw)) > 1e-3:
            r = _rotation(chk, rot_mode, p, yaw, head)
            if r is None:
                return None, f'cannot rotate in place at ({p[0]:.2f}, {p[1]:.2f}): footprint hits an obstacle'
            segs.append(r)
        yaw = head
        poses = _line(p, q, head)
        if not chk.poses_free(poses):
            return None, 'straight leg hits an obstacle with the exact footprint'
        if segs and segs[-1].kind == 'drive' and segs[-1].poses[-1].yaw == head:
            segs[-1].poses += poses[1:]
        else:
            segs.append(ModeSegment(drive_mode.name, 'drive', poses))
    if abs(wrap(goal[2] - yaw)) > 1e-3 or not segs:
        r = _rotation(chk, rot_mode, path[-1], yaw, goal[2])
        if r is None:
            return None, 'cannot rotate in place at the goal: footprint hits an obstacle'
        segs.append(r)
    return segs, ''


# ---------------------------------------------------------------- Ackermann: Hybrid A* + Reeds-Shepp
def _heuristic_map(footprint, start, goal, obstacles, res, margin):
    g = _Grid(footprint, start, goal, obstacles, res, margin)
    t = (g._ix(goal[0]), g._iy(goal[1]))
    dist = {t: 0.0}
    heap = [(0.0, t)]
    sq2 = math.sqrt(2.0)
    while heap:
        d, c = heapq.heappop(heap)
        if d > dist.get(c, 1e18):
            continue
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nb = (c[0] + di, c[1] + dj)
            nd = d + (sq2 if di and dj else 1.0) * res
            if nd < dist.get(nb, 1e18) and g.free(*nb):
                dist[nb] = nd
                heapq.heappush(heap, (nd, nb))
    return g, dist


def plan_ackermann(footprint, start, goal, obstacles, mode, resolution=0.1, margin=0.02, max_expansions=8000,
                   radius_headroom=RADIUS_HEADROOM):
    """Hybrid A* with a Reeds-Shepp goal connection. Returns (segment list or None, reason).

    Arcs use radius min_turn_radius * radius_headroom so the follower has curvature left to correct with."""
    R = mode.min_turn_radius * radius_headroom
    kmax = 1.0 / R
    chk = _Checker(footprint, obstacles, margin)
    if not chk.free(start):
        return None, 'start pose is in collision'
    if not chk.free(goal):
        return None, 'goal pose is in collision'

    def connect(pose):
        for _, pieces in rs.paths(pose, goal, R)[:4]:
            pts = rs.sample(pose, pieces, R, DS)
            if chk.poses_free([Waypoint(p[0], p[1], p[2]) for p in pts]):
                return pts
        return None

    grid, hmap = _heuristic_map(footprint, start, goal, obstacles, resolution, margin)

    def h(x, y):
        e = math.hypot(goal[0] - x, goal[1] - y)
        return max(e, hmap.get((grid._ix(x), grid._iy(y)), 0.0))

    step = max(0.3, R * 0.4)
    n_th = 24
    bin_th = 2 * math.pi / n_th

    def key(p):
        return (int(round(p[0] / resolution)), int(round(p[1] / resolution)), int(round(wrap(p[2]) / bin_th)) % n_th)

    prims = [(k * kmax, d) for d in (1, -1) for k in (0.0, 1.0, -1.0, 0.5, -0.5)]
    nodes = [(tuple(start), None, 0.0, 0, 0.0)]       # pose, parent, k, gear, g
    best = {key(start): 0.0}
    heap = [(h(start[0], start[1]), 0, 0)]
    counter, pops = 1, 0
    done_pts, tail = None, None
    while heap:
        _, _, idx = heapq.heappop(heap)
        pose, parent, k_in, gear_in, g = nodes[idx]
        if g > best.get(key(pose), 1e18) + 1e-9:
            continue
        pops += 1
        if pops > max_expansions:
            return None, (f'no path found at turn radius {R:.2f} m (limit {mode.min_turn_radius:.2f} m + headroom) within {max_expansions} expansions '
                          '(search budget reached, not proven infeasible)')
        e = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        if e < 1e-6 and abs(wrap(goal[2] - pose[2])) < 1e-6:
            tail, done_pts = idx, []
            break
        if pops == 1 or e < max(4 * R, 2.0) or pops % 8 == 0:
            tail_pts = connect(pose)
            if tail_pts is not None:
                tail, done_pts = idx, tail_pts
                break
        for k, d in prims:
            pts = _arc_points(pose, k, d * step)
            if not chk.poses_free(pts[::2] + [pts[-1]]):
                continue
            q = (pts[-1].x, pts[-1].y, pts[-1].yaw)
            cost = step * (1.0 + (0.4 if d < 0 else 0.0)) + 0.15 * step * abs(k - k_in) / kmax
            if gear_in and d != gear_in:
                cost += 1.0
            ng = g + cost
            kq = key(q)
            if ng < best.get(kq, 1e18) - 1e-9:
                best[kq] = ng
                nodes.append((q, idx, k, d, ng))
                heapq.heappush(heap, (ng + h(q[0], q[1]), counter, len(nodes) - 1))
                counter += 1
    if done_pts is None:
        return None, (f'no path respects minimum turn radius {mode.min_turn_radius:.2f} m in this space '
                      f'(search exhausted after {pops} expansions)')
    chain = []
    i = tail
    while nodes[i][1] is not None:
        chain.append(i)
        i = nodes[i][1]
    chain.reverse()
    poses = [Waypoint(start[0], start[1], start[2], 1, 0.0)]
    for i in chain:
        parent = nodes[nodes[i][1]][0]
        k, d = nodes[i][2], nodes[i][3]
        poses += _arc_points(parent, k, d * step)
    for p in done_pts[1:]:
        poses.append(Waypoint(p[0], p[1], p[2], p[3], p[4]))
    if len(poses) == 1:
        poses.append(Waypoint(*goal))
    poses[0].gear, poses[0].curv = poses[1].gear, poses[1].curv
    poses[-1] = Waypoint(goal[0], goal[1], goal[2], poses[-1].gear, poses[-1].curv)
    return [ModeSegment(mode.name, 'arc', poses)], ''


# ---------------------------------------------------------------- strategy choice
def estimate_duration(segments, modes):
    by = {m.name: m for m in modes}
    t = 0.0
    for s in segments:
        m = by[s.mode]
        if s.kind == 'arc':
            t += sum(math.hypot(b.x - a.x, b.y - a.y) / (m.turning_speed if abs(b.curv) > 1e-6 else m.max_lin)
                     for a, b in zip(s.poses, s.poses[1:]))
        else:
            t += s.length() / m.max_lin
        t += abs(s.sweep()) / m.max_ang + m.max_lin / m.max_lin_acc
    return t


def _merge_crab(segs):
    out = []
    for s in segs:
        if out and s.kind == 'crab' and out[-1].kind == 'crab' and out[-1].mode == s.mode:
            out[-1].poses += s.poses[1:]
        else:
            out.append(s)
    return out


def _travel(modes, chk, footprint, start, goal, obstacles, res, margin, max_expansions):
    """(strategy, segments, duration) candidates and a list of failure reasons for start -> goal."""
    names = {m.name: m for m in modes}
    cands, why = [], []

    def add(name, r):
        segs, reason = r
        if segs is None:
            why.append(f'{name}: {reason}')
        else:
            cands.append((name, segs, estimate_duration(segs, modes)))

    if 'omni' in names:
        add('omni', _plan_translate(modes, chk, footprint, start, goal, obstacles, res, margin))
    if 'crab' in names:
        add('crab', _plan_crab(modes, chk, footprint, start, goal, obstacles, res, margin))
    for rot in [m for m in modes if m.can_spin_in_place and m.name != 'omni']:
        drv = rot if rot.name in ('diff_turn_in_place', 'skid') else names.get('ackermann')
        if drv is not None:
            add(f'{rot.name}+drive', _plan_spin_drive(modes, chk, footprint, start, goal, obstacles, res, margin, rot, drv))
    if 'ackermann' in names:
        add('ackermann', plan_ackermann(footprint, start, goal, obstacles, names['ackermann'], max(res, 0.1),
                                        margin, max_expansions))
    return cands, why


def plan_with_modes(profile, start, goal, obstacles=(), resolution=0.05, margin=0.02, dock_dir=None,
                    crab_final_m=1.0, max_expansions=8000):
    """Feasible base path and mode schedule from `start` to `goal` (footprint poses) for the profile's modes."""
    modes = profile.drive_modes()
    obstacles = list(obstacles)
    fp = profile.footprint
    chk = _Checker(fp, obstacles, margin)
    start, goal = tuple(start), tuple(goal)
    if not chk.free(start):
        return ModePlan(False, reason='start pose is in collision')
    if not chk.free(goal):
        return ModePlan(False, reason='goal pose is in collision')
    leg_len = 0.0
    names = {m.name for m in modes}
    notes = []
    target, crab_seg = goal, None
    if dock_dir is not None and 'crab' in names and 'omni' not in names:
        u = (math.cos(dock_dir), math.sin(dock_dir))
        L = crab_final_m
        while L >= 0.1:
            a = (goal[0] - u[0] * L, goal[1] - u[1] * L, goal[2])
            poses = _line(a, goal, goal[2])
            if chk.poses_free(poses):
                target, crab_seg, leg_len = a, ModeSegment('crab', 'crab', poses), L
                break
            L /= 2.0
        if crab_seg is None:
            notes.append('no free crab leg toward the dock direction; planned straight to the goal')
    if crab_seg is not None and math.hypot(start[0] - target[0], start[1] - target[1]) < 1e-6 \
            and abs(wrap(start[2] - target[2])) < 1e-6:
        cands, why = [('crab', [], 0.0)], []
    else:
        cands, why = _travel(modes, chk, fp, start, target, obstacles, resolution, margin, max_expansions)
    if not cands and crab_seg is not None:
        notes.append('travel to the crab leg failed; planned straight to the goal')
        target, crab_seg = goal, None
        cands, why = _travel(modes, chk, fp, start, goal, obstacles, resolution, margin, max_expansions)
    if not cands:
        return ModePlan(False, reason='; '.join(why) or 'no strategy for these modes', notes=notes)
    name, segs, dur = min(cands, key=lambda c: c[2])      # min keeps the first of equal durations
    if crab_seg is not None:
        segs = _merge_crab(segs + [crab_seg])
        name += f'+crab_leg({leg_len:.2f} m)'
        dur = estimate_duration(segs, modes)
    return ModePlan(True, segs, '', name, dur, notes)
