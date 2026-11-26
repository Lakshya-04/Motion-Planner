"""Wheel-odometry noise on top of a true planar pose. ROS-free.

Per run: a scale error on linear speed and on yaw rate, and an initial yaw-rate bias. While running: the bias
random-walks, white noise goes on every increment, and slip spells give a short stretch of larger scale error.
The reported pose is the integral of the corrupted body-frame increments, so errors accumulate like real odometry.
"""
import math
from collections import deque

import numpy as np

from .presets import OdomParams


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _diag_cov(var_xy, var_yaw):
    cov = np.zeros(36)
    for i in (0, 7, 14, 21, 28, 35):
        cov[i] = 1e-9    # z, roll, pitch: a planar base, known
    cov[0] = cov[7] = max(var_xy, 1e-9)
    cov[35] = max(var_yaw, 1e-9)
    return cov


class OdomModel:
    def __init__(self, params: OdomParams, seed=0):
        self.p = params
        self.rng = np.random.default_rng(seed)
        p = params
        self.lin_err = self.rng.normal(0.0, p.scale_lin_sigma) if p.scale_lin_sigma else 0.0
        self.yaw_err = self.rng.normal(0.0, p.scale_yaw_sigma) if p.scale_yaw_sigma else 0.0
        self.bias = self.rng.normal(0.0, p.yaw_bias_init_sigma) if p.yaw_bias_init_sigma else 0.0
        self.slip_left = 0.0
        self.slip_lin = 0.0
        self.slip_yaw = 0.0
        self.slips = 0
        self.truth = None        # last truth (x, y, yaw, t)
        self.est = None          # reported (x, y, yaw)
        self.path = 0.0          # truth distance driven
        self.turned = 0.0        # truth |yaw| turned
        self.t_run = 0.0
        self.var_bias_int = 0.0  # variance of the bias integral, from the walk
        self.last_dt = 0.02

    def step(self, x, y, yaw, t, twist=(0.0, 0.0, 0.0)):
        """One truth sample. Returns (x, y, yaw, (vx, vy, wz), pose_cov36, twist_cov36) as reported."""
        p = self.p
        first = self.truth is None
        if first or p.is_identity():
            self.truth = (x, y, yaw, t)
            self.est = (x, y, yaw)
            return x, y, yaw, tuple(twist), self._pose_cov(), self._twist_cov(twist, self.last_dt)
        px, py, pyaw, pt = self.truth
        dt = t - pt
        if not (1e-6 < dt < 5.0):
            dt = self.last_dt
        self.last_dt = dt
        self.truth = (x, y, yaw, t)
        c, s = math.cos(pyaw), math.sin(pyaw)
        dxw, dyw = x - px, y - py
        dx, dy = c * dxw + s * dyw, -s * dxw + c * dyw      # increment in the previous body frame
        dth = wrap(yaw - pyaw)
        dist = math.hypot(dx, dy)
        self.path += dist
        self.turned += abs(dth)
        self.t_run += dt

        if self.slip_left > 0.0:
            self.slip_left -= dt
        elif p.slip_rate_per_m and self.rng.random() < p.slip_rate_per_m * dist:
            self.slip_left = p.slip_duration_s
            self.slip_lin = self.rng.normal(0.0, p.slip_scale_sigma)
            self.slip_yaw = self.rng.normal(0.0, p.slip_scale_sigma)
            self.slips += 1
        slipping = self.slip_left > 0.0
        s_lin = 1.0 + self.lin_err + (self.slip_lin if slipping else 0.0)
        s_yaw = 1.0 + self.yaw_err + (self.slip_yaw if slipping else 0.0)

        if p.yaw_bias_walk:
            self.bias += self.rng.normal(0.0, p.yaw_bias_walk * math.sqrt(dt))
            self.var_bias_int += (p.yaw_bias_walk ** 2) * self.t_run ** 2 * dt
        n_xy = p.trans_noise * math.sqrt(dist)
        n_th = p.yaw_noise * math.sqrt(dist) + p.yaw_noise_rot * math.sqrt(abs(dth))
        dxe = s_lin * dx + (self.rng.normal(0.0, n_xy) if n_xy else 0.0)
        dye = s_lin * dy + (self.rng.normal(0.0, n_xy) if n_xy else 0.0)
        dthe = s_yaw * dth + self.bias * dt + (self.rng.normal(0.0, n_th) if n_th else 0.0)

        ex, ey, eyaw = self.est
        mid = eyaw + 0.5 * dthe
        cm, sm = math.cos(mid), math.sin(mid)
        self.est = (ex + cm * dxe - sm * dye, ey + sm * dxe + cm * dye, wrap(eyaw + dthe))

        vx, vy, wz = twist
        tw = (s_lin * vx, s_lin * vy, s_yaw * wz + self.bias)
        return (*self.est, tw, self._pose_cov(), self._twist_cov(twist, dt))

    def drift(self):
        """(position error m, yaw error rad, truth path m) of the reported pose against the truth."""
        if self.truth is None or self.est is None:
            return 0.0, 0.0, 0.0
        tx, ty, tyaw, _ = self.truth
        return (math.hypot(self.est[0] - tx, self.est[1] - ty), wrap(self.est[2] - tyaw), self.path)

    def _pose_cov(self):
        p = self.p
        d, th, t = self.path, self.turned, self.t_run
        var_yaw = (p.yaw_noise ** 2 * d + p.yaw_noise_rot ** 2 * th + (p.scale_yaw_sigma * th) ** 2
                   + (p.yaw_bias_init_sigma * t) ** 2 + self.var_bias_int / 3.0)
        var_xy = p.trans_noise ** 2 * d + (p.scale_lin_sigma * d) ** 2 + d * d * var_yaw / 3.0
        return _diag_cov(var_xy, var_yaw)

    def _twist_cov(self, twist, dt):
        p = self.p
        vx, vy, wz = twist
        v = math.hypot(vx, vy)
        dt = max(dt, 1e-3)
        var_v = (p.scale_lin_sigma * v) ** 2 + p.trans_noise ** 2 * v / dt
        var_w = ((p.scale_yaw_sigma * wz) ** 2 + p.yaw_bias_init_sigma ** 2 + p.yaw_noise ** 2 * v / dt
                 + p.yaw_noise_rot ** 2 * abs(wz) / dt)
        return _diag_cov(var_v, var_w)


class MessageChannel:
    """Fixed latency in whole periods, then random loss. push() returns what comes out now, or None."""

    def __init__(self, latency_periods=0, dropout=0.0, seed=0):
        self.queue = deque()
        self.n = int(latency_periods)
        self.dropout = dropout
        self.rng = np.random.default_rng(seed)

    def push(self, item):
        self.queue.append(item)
        if len(self.queue) <= self.n:
            return None
        out = self.queue.popleft()
        if self.dropout and self.rng.random() < self.dropout:
            return None
        return out
