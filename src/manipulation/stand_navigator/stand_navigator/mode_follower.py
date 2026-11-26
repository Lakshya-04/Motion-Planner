"""Turns a `ModePlan` into /cmd_vel, one leg at a time, in the plan's drive modes.

ROS-free. `step(pose, dt)` returns (vx, vy, wz) in the base frame. Diff, skid, spin and Ackermann legs never
get a lateral velocity; Ackermann also keeps |wz| <= |v| / min_turn_radius. Between legs of different modes the
base stops and `set_mode(old, new)` is called (the vendor driver request: a service or topic in reality); it
returns True once the new mode is active, and the base waits at zero until it does. The follower has the same
surface as the other controllers (`step`, `arrived`, `errors`, `phase`, `cmd`).
"""
import logging
import math

from stand_navigator.core import Tolerance, clamp, wrap
from stand_navigator.mode_planner import Waypoint
from stand_navigator.modes import default_mode

log = logging.getLogger('stand_navigator.modes')

V_MIN = 0.12          # creep speed so a leg still closes its last centimetres, m/s
END_EPS = 0.004       # along-track distance at which a leg counts as finished, m


def _ramp(dist, vmax, acc):
    return min(vmax, math.sqrt(2.0 * acc * max(dist, 0.0)))


class _Leg:
    def __init__(self, mode, kind, poses):
        self.mode, self.kind, self.poses = mode, kind, poses
        self.cum = [0.0]
        for a, b in zip(poses, poses[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b.x - a.x, b.y - a.y))
        self.idx = 0
        self.vlim = None
        self.sweep = sum(wrap(b.yaw - a.yaw) for a, b in zip(poses, poses[1:]))


def _split(plan):
    legs = []
    for s in plan.segments:
        if s.kind == 'arc':
            cur = [s.poses[0]]
            for p in s.poses[1:]:
                if p.gear != cur[-1].gear and len(cur) > 1:
                    legs.append(_Leg(s.mode, s.kind, cur))
                    cur = [cur[-1]]
                cur.append(p)
            legs.append(_Leg(s.mode, s.kind, cur))
        else:
            legs.append(_Leg(s.mode, s.kind, list(s.poses)))
    return legs


class ModeFollower:
    def __init__(self, profile, plan, goal, tol=None, set_mode=None, start_mode=None,
                 ky=40.0, kth=12.6, kp_ang=1.5, kp_lin=2.0):
        self.profile, self.goal = profile, goal
        self.tol = tol or Tolerance()
        self.modes = {m.name: m for m in profile.drive_modes()}
        self.set_mode = set_mode or (lambda old, new: True)
        self.active = start_mode or default_mode(profile.drive_modes()).name
        self.legs = _split(plan)
        self.i = 0
        self.cmd = (0.0, 0.0, 0.0)
        self.waiting = False
        self.ky, self.kth, self.kp_ang, self.kp_lin = ky, kth, kp_ang, kp_lin

    @property
    def phase(self):
        if self.i >= len(self.legs):
            return 'done'
        return f'{self.legs[self.i].mode}:{self.legs[self.i].kind}'

    @property
    def current_mode(self):
        return self.active

    def errors(self, pose):
        gx, gy, gyaw = self.goal
        return math.hypot(gx - pose[0], gy - pose[1]), wrap(gyaw - pose[2])

    def arrived(self, pose, lin_speed=0.0, ang_speed=0.0):
        d, e = self.errors(pose)
        t = self.tol
        return d <= t.xy and abs(e) <= t.yaw and lin_speed <= t.speed and ang_speed <= t.speed

    # ------------------------------------------------------------ legs
    def _finish_leg(self):
        self.i += 1
        self.cmd = (0.0, 0.0, 0.0)

    def _rotate(self, leg, pose):
        yt = leg.poses[-1].yaw
        err = wrap(yt - pose[2])
        if abs(err) > 0.2 and leg.sweep * err < 0:
            err += math.copysign(2 * math.pi, leg.sweep)      # the planned (long) way round
        m = self.modes[leg.mode]
        if abs(err) <= min(0.005, self.tol.yaw / 2):
            self._finish_leg()
            return 0.0, 0.0, 0.0
        w = max(V_MIN, min(m.max_ang, abs(self.kp_ang * err), _ramp(abs(err), m.max_ang, m.max_ang_acc)))
        return 0.0, 0.0, math.copysign(min(w, abs(err) / 0.02), err)

    def _project(self, leg, pose):
        p = leg.poses
        i = leg.idx
        while i < len(p) - 1 and (math.hypot(p[i + 1].x - pose[0], p[i + 1].y - pose[1])
                                  <= math.hypot(p[i].x - pose[0], p[i].y - pose[1])):
            i += 1
        leg.idx = i
        return i

    @staticmethod
    def _speed_limits(leg, m):
        """Speed cap per pose: the turning speed on curved pieces, braked toward them from earlier on."""
        p = leg.poses
        lim = [m.turning_speed if (leg.kind == 'arc' and abs(q.curv) > 1e-6) else m.max_lin for q in p]
        for n in range(len(p) - 2, -1, -1):
            ds = leg.cum[n + 1] - leg.cum[n]
            lim[n] = min(lim[n], math.sqrt(lim[n + 1] ** 2 + 2.0 * m.max_lin_acc * ds))
        return lim

    def _drive(self, leg, pose):
        """Kanayama-style tracking of an Ackermann or straight leg (v along the path, yaw rate from the errors)."""
        m = self.modes[leg.mode]
        i = self._project(leg, pose)
        j = min(i + 1, len(leg.poses) - 1)
        a, b = leg.poses[min(i, j)], leg.poses[j]
        seg = leg.cum[j] - leg.cum[min(i, j)]
        t = 0.0 if seg < 1e-9 else max(0.0, min(1.0, ((pose[0] - a.x) * (b.x - a.x) + (pose[1] - a.y) * (b.y - a.y)) / seg ** 2))
        # reference point on the segment under the base (continuous, so the curvature switch is not late)
        ref = Waypoint(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), wrap(a.yaw + t * wrap(b.yaw - a.yaw)),
                       b.gear, b.curv)
        gear = b.gear
        yaw = pose[2]
        dx, dy = ref.x - pose[0], ref.y - pose[1]
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        eth = wrap(ref.yaw - yaw)
        end = leg.poses[-1]
        # along-track distance left, measured on the end pose's travel direction
        ux, uy = math.cos(end.yaw) * gear, math.sin(end.yaw) * gear
        rem_chord = (end.x - pose[0]) * ux + (end.y - pose[1]) * uy
        rem = leg.cum[-1] - leg.cum[i]
        if i >= len(leg.poses) - 1 and rem_chord <= END_EPS:
            self._finish_leg()
            return 0.0, 0.0, 0.0
        k = ref.curv if leg.kind == 'arc' else 0.0
        if leg.vlim is None:
            leg.vlim = self._speed_limits(leg, m)
        v = max(V_MIN, _ramp(rem, leg.vlim[min(j, len(leg.vlim) - 1)], m.max_lin_acc))
        v *= max(0.2, math.cos(min(abs(eth), 1.2)))
        if abs(k) * v > m.max_ang:
            v = m.max_ang / abs(k)
        vs = gear * v
        w = k * vs + vs * self.ky * ey + abs(vs) * self.kth * math.sin(eth)
        if m.min_turn_radius:
            w = clamp(w, v * m.max_curvature)
        w = clamp(w, m.max_ang)
        return vs, 0.0, w

    def _translate(self, leg, pose):
        """Crab and omni legs: world-frame velocity along the path at a fixed (crab) or interpolated yaw."""
        m = self.modes[leg.mode]
        p = leg.poses
        i = self._project(leg, pose)
        end = p[-1]
        rem = leg.cum[-1] - leg.cum[i]
        d_end = math.hypot(end.x - pose[0], end.y - pose[1])
        if i >= len(p) - 1 and d_end <= END_EPS:
            yerr = wrap(end.yaw - pose[2])
            if abs(yerr) <= min(0.005, self.tol.yaw / 2):
                self._finish_leg()
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, clamp(self.kp_ang * yerr, m.max_ang)
        look = 0.15
        j = i
        while j < len(p) - 1 and leg.cum[j] - leg.cum[i] < look:
            j += 1
        tx, ty = p[j].x - pose[0], p[j].y - pose[1]
        n = math.hypot(tx, ty)
        v = max(V_MIN, _ramp(max(rem, d_end), m.max_lin, m.max_lin_acc))
        v = min(v, max(d_end, 1e-6) * 10.0 + V_MIN)
        wx, wy = (v * tx / n, v * ty / n) if n > 1e-9 else (0.0, 0.0)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wz = clamp(self.kp_ang * wrap(p[j].yaw - pose[2]), m.max_ang)
        return c * wx + s * wy, -s * wx + c * wy, wz

    # ------------------------------------------------------------ step
    def _limit(self, target, dt, m):
        tx, ty, tw = target
        dv = m.max_lin_acc * dt
        ex, ey = tx - self.cmd[0], ty - self.cmd[1]
        en = math.hypot(ex, ey)
        if en > dv:
            ex, ey = ex * dv / en, ey * dv / en
        if m.min_turn_radius:
            # steering is its own actuator (rate not in the profile): no yaw-accel limit, curvature limit instead
            wz = clamp(tw, abs(self.cmd[0] + ex) * m.max_curvature)
        else:
            wz = self.cmd[2] + clamp(tw - self.cmd[2], m.max_ang_acc * dt)
        self.cmd = (self.cmd[0] + ex, self.cmd[1] + ey, wz)
        return self.cmd

    def step(self, pose, dt):
        for _ in range(len(self.legs) + 1):
            if self.i >= len(self.legs):
                self.cmd = (0.0, 0.0, 0.0)
                return self.cmd
            leg = self.legs[self.i]
            if leg.mode != self.active:
                if not self.waiting:
                    log.info('mode change %s -> %s requested', self.active, leg.mode)
                    self.waiting = True
                if self.set_mode(self.active, leg.mode):
                    log.info('mode %s active', leg.mode)
                    self.active, self.waiting = leg.mode, False
                else:
                    self.cmd = (0.0, 0.0, 0.0)
                    return self.cmd
            m = self.modes[leg.mode]
            if leg.kind == 'rotate':
                t = self._rotate(leg, pose)
            elif leg.kind in ('drive', 'arc'):
                t = self._drive(leg, pose)
            else:
                t = self._translate(leg, pose)
            if self.i < len(self.legs) and self.legs[self.i] is leg:
                return self._limit(t, dt, m)
        return self.cmd
