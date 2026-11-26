"""ROS-free base safety layer for navigate_to_stand.

Pieces (all take the time as an argument, so they are deterministic in tests):

 * `gate_command`: velocity-command sanity gate. NaN/inf is refused, a value clearly over the profile limit is
   refused, a value a hair over is clamped, and the change from the previous command is clamped to the profile
   acceleration (a re-check of what `StandController` already does).
 * `SafetyMonitor.check`: stale-odometry watchdog, stuck detection (no progress toward the goal for a window)
   and an optional obstacle stop driven by a front-clearance number (metres, None = no data).
 * `corridor_clearance`, `scan_to_points`, `cloud_to_points`: summarise a LaserScan or PointCloud2 (points in the
   base frame) into that number along the direction of travel.

A healthy run is unchanged: the controller output is already inside the limits, the stuck window is long and
reset by any progress, and the obstacle stop is off without a clearance source.
"""
import math
import struct
from dataclasses import dataclass

from stand_navigator.core import clamp

# reason codes (the node maps them to NavigateToStand error codes)
OK = 'OK'
NO_ODOM = 'NO_ODOM'
BAD_COMMAND = 'BAD_COMMAND'
NAVIGATION_STUCK = 'NAVIGATION_STUCK'
OBSTACLE = 'OBSTACLE'

# actions
CONTINUE = 'continue'
HOLD = 'hold'        # stop and wait, the monitor aborts after obstacle_hold_s
ABORT = 'abort'


@dataclass
class Verdict:
    action: str = CONTINUE
    code: str = OK
    reason: str = ''

    @property
    def ok(self):
        return self.action == CONTINUE


@dataclass
class SafetyConfig:
    odom_timeout_s: float = 1.0
    check_commands: bool = True
    over_limit_tol: float = 0.05        # fraction over a limit that is clamped; more is refused
    stuck_window_s: float = 20.0        # 0 turns stuck detection off
    stuck_min_progress: float = 0.02    # metric units (m) of progress that reset the window
    yaw_weight_m_per_rad: float = 0.25  # yaw error counts as this many metres of distance
    front_stop_m: float = 0.0           # 0 turns the obstacle stop off
    obstacle_hold_s: float = 5.0        # wait this long for the way to clear, then abort
    obstacle_data_timeout_s: float = 1.0
    corridor_half_width_m: float = 0.4
    max_range_m: float = 5.0


def progress_metric(dist, yaw_err, cfg):
    return dist + cfg.yaw_weight_m_per_rad * abs(yaw_err)


def gate_command(cmd, prev, dt, profile, cfg=None):
    """Return ((vx, vy, wz), Verdict). The command is only changed by clamping (small excess, acceleration)."""
    cfg = cfg or SafetyConfig()
    if not all(isinstance(c, (int, float)) and math.isfinite(c) for c in cmd):
        return (0.0, 0.0, 0.0), Verdict(ABORT, BAD_COMMAND, f'non-finite velocity command {tuple(cmd)}')
    vx, vy, wz = cmd
    lin = math.hypot(vx, vy)
    tol = 1.0 + cfg.over_limit_tol
    if lin > profile.max_lin * tol or abs(wz) > profile.max_ang * tol:
        return (0.0, 0.0, 0.0), Verdict(
            ABORT, BAD_COMMAND,
            f'velocity command {lin:.3f} m/s {wz:.3f} rad/s over the limits '
            f'{profile.max_lin:.3f} m/s {profile.max_ang:.3f} rad/s')
    if lin > profile.max_lin:
        vx, vy = vx * profile.max_lin / lin, vy * profile.max_lin / lin
    wz = clamp(wz, profile.max_ang)
    if dt > 0.0 and prev is not None:
        dv = profile.max_lin_acc * dt * tol
        ex, ey = vx - prev[0], vy - prev[1]
        en = math.hypot(ex, ey)
        if en > dv:
            vx, vy = prev[0] + ex * dv / en, prev[1] + ey * dv / en
        wz = prev[2] + clamp(wz - prev[2], profile.max_ang_acc * dt * tol)
    return (vx, vy, wz), Verdict()


class SafetyMonitor:
    def __init__(self, cfg, now):
        self.cfg = cfg
        self.best = None
        self.t_best = now
        self.hold_since = None

    def check(self, now, odom_stamp, metric=None, settled=False, clearance=None, clearance_stamp=None,
              speed=0.0, max_lin_acc=0.5):
        """One control cycle. `odom_stamp`/`clearance_stamp` are on the same clock as `now` (None = never).
        `metric` is `progress_metric(...)`; `settled` is true inside the goal tolerance (no stuck there).
        `clearance` is the free distance ahead in the direction of travel, None when there is no sensor/data."""
        c = self.cfg
        if odom_stamp is None or now - odom_stamp > c.odom_timeout_s:
            age = 'never received' if odom_stamp is None else f'{now - odom_stamp:.2f} s old'
            return Verdict(ABORT, NO_ODOM, f'odometry went stale ({age}, limit {c.odom_timeout_s:g} s)')
        if c.front_stop_m > 0.0:
            if clearance_stamp is None or now - clearance_stamp > c.obstacle_data_timeout_s:
                return Verdict(ABORT, OBSTACLE, 'obstacle sensor data missing or stale, cannot confirm the way is clear')
            stop_at = c.front_stop_m + speed * speed / (2.0 * max(max_lin_acc, 1e-6))
            if clearance is not None and clearance < stop_at:
                if self.hold_since is None:
                    self.hold_since = now
                if now - self.hold_since > c.obstacle_hold_s:
                    return Verdict(ABORT, OBSTACLE, f'obstacle {clearance:.2f} m ahead for more than '
                                   f'{c.obstacle_hold_s:g} s (stop distance {stop_at:.2f} m)')
                self.t_best = now          # waiting is not being stuck
                return Verdict(HOLD, OBSTACLE, f'obstacle {clearance:.2f} m ahead (stop distance {stop_at:.2f} m)')
            self.hold_since = None
        if c.stuck_window_s > 0.0 and metric is not None:
            if settled or self.best is None or metric < self.best - c.stuck_min_progress:
                self.best, self.t_best = metric, now
            elif now - self.t_best > c.stuck_window_s:
                return Verdict(ABORT, NAVIGATION_STUCK,
                               f'no progress toward the goal for {now - self.t_best:.0f} s '
                               f'(less than {c.stuck_min_progress:g} m)')
        return Verdict()


# ---- sensor summaries (points are (x, y) in the base frame, +x forward) ----

class HeldDirection:
    """Direction of travel to check for obstacles; remembers the last one while the base is held."""

    def __init__(self):
        self.last = None

    def hold(self, sent):
        if math.hypot(sent[0], sent[1]) >= 1e-3:
            self.last = sent

    def look(self, sent):
        if math.hypot(sent[0], sent[1]) >= 1e-3:
            return sent
        return self.last or sent


def corridor_clearance(points, direction, half_width, max_range=5.0):
    """Distance to the nearest point inside a corridor `half_width` each side of the ray at angle `direction`
    (rad, base frame). Points behind the base are ignored. Returns max_range when the corridor is empty."""
    c, s = math.cos(direction), math.sin(direction)
    best = max_range
    for x, y in points:
        a, l = c * x + s * y, -s * x + c * y
        if 0.0 <= a < best and abs(l) <= half_width:
            best = a
    return best


def scan_to_points(ranges, angle_min, angle_increment, range_min=0.0, range_max=float('inf')):
    pts = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and range_min <= r <= range_max:
            a = angle_min + i * angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def cloud_to_points(data, point_step, offsets, little_endian=True, z_range=(-0.2, 1.5)):
    """Points of a PointCloud2 (`data` bytes, float32 x/y/z at byte `offsets`=(ox, oy, oz)) inside `z_range`."""
    fmt = '<f' if little_endian else '>f'
    ox, oy, oz = offsets
    pts = []
    for i in range(0, len(data) - point_step + 1, point_step):
        x = struct.unpack_from(fmt, data, i + ox)[0]
        y = struct.unpack_from(fmt, data, i + oy)[0]
        z = struct.unpack_from(fmt, data, i + oz)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and z_range[0] <= z <= z_range[1]:
            pts.append((x, y))
    return pts
