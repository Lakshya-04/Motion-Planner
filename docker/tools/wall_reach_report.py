#!/usr/bin/env python3
"""Which part of a vertical-strip wall can a UR10e on the telescopic lift reach from a stand? (host only)

Model: ANALYTIC, not an RM4D map. The UR10e is the published Universal Robots DH chain (closed-form
inverse kinematics, all 8 branches, joint limits elbow +-pi and the rest +-2*pi (ur_description), no self-collision, no link volumes). The tool is
a contact tool: its z axis points straight into the wall (tilt 0, as in coverage_planner's
tool_profiles.yaml) and it may roll freely about that axis (roll sampled every --roll-step-deg).
A wall point is reachable at one lift height if some roll has an IK solution whose joint-origin points
stay in front of the wall (x <= wall - --link-clearance-m). The lift is continuous over 0..--lift-max, so
a wall height z is reachable from a stand if some lift height puts z inside the arm's reachable band.

Frames (arm base frame, which only moves up and down with the lift): x toward the wall, y along the wall,
z up. A stand is where the arm base is: standoff = arm base to wall, lateral = position along the wall.
The arm base is --arm-base-z0 above the floor with the lift retracted (default 1.17 = generic_base top
0.35 + lift mount_height 0.82 from urdf/base/generic_base.xacro and profiles/ur10e_telescopic.yaml).

Because the wall strip must also be approached, a point counts only if the contact point and the point
--approach-m in front of it (column_reach_tool's sequence.standoff_m, default 0.12) are both reachable.

Usage examples are in docs/edge_columns.md.
"""
import argparse
import json
import math
import sys

import numpy as np

# Published UR10e DH parameters (Universal Robots, "Parameters for calculations of kinematics and
# dynamics"): d1, a2, a3, d4, d5, d6 in metres.
D1, A2, A3, D4, D5, D6 = 0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655
ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
A = (0.0, A2, A3, 0.0, 0.0, 0.0)
D = (D1, 0.0, 0.0, D4, D5, D6)
# ur_description config/ur10e/joint_limits.yaml (and docs/planner_benchmark.md): elbow +-pi, the others +-2*pi.
JOINT_LIMIT = np.array([2 * math.pi, 2 * math.pi, math.pi, 2 * math.pi, 2 * math.pi, 2 * math.pi])
TCP_Z = 0.165   # tcp_xyz "0 0 0.165" from the ur10e_telescopic profile, flange -> TCP along tool z


def dh(theta, i):
    """Transform of DH link i (0..5) for an array of angles; returns (...,4,4)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = math.cos(ALPHA[i]), math.sin(ALPHA[i])
    T = np.zeros(theta.shape + (4, 4))
    T[..., 0, 0] = ct
    T[..., 0, 1] = -st * ca
    T[..., 0, 2] = st * sa
    T[..., 0, 3] = A[i] * ct
    T[..., 1, 0] = st
    T[..., 1, 1] = ct * ca
    T[..., 1, 2] = -ct * sa
    T[..., 1, 3] = A[i] * st
    T[..., 2, 1] = sa
    T[..., 2, 2] = ca
    T[..., 2, 3] = D[i]
    T[..., 3, 3] = 1.0
    return T


def fk_frames(q):
    """q (...,6) -> list of 6 cumulative transforms T0i (...,4,4); the last is the flange."""
    q = np.asarray(q, float)
    out = []
    T = None
    for i in range(6):
        Ti = dh(q[..., i], i)
        T = Ti if T is None else T @ Ti
        out.append(T)
    return out


def fk(q):
    return fk_frames(q)[-1]


def _inv(T):
    R = T[..., :3, :3]
    p = T[..., :3, 3]
    Ti = np.zeros_like(T)
    Ti[..., :3, :3] = np.swapaxes(R, -1, -2)
    Ti[..., :3, 3] = -np.einsum('...ij,...j->...i', np.swapaxes(R, -1, -2), p)
    Ti[..., 3, 3] = 1.0
    return Ti


def ik(T06):
    """Closed-form IK (all branches) for flange poses T06 (N,4,4). Returns (N,8,6) angles and (N,8) valid."""
    N = T06.shape[0]
    sols = np.zeros((N, 8, 6))
    ok = np.zeros((N, 8), bool)
    px, py = T06[:, 0, 3], T06[:, 1, 3]
    zc = T06[:, :3, 2]
    p05 = T06[:, :3, 3] - D6 * zc
    R = np.hypot(p05[:, 0], p05[:, 1])
    with np.errstate(invalid='ignore', divide='ignore'):
        base = np.arctan2(p05[:, 1], p05[:, 0])
        ph = np.arccos(np.clip(D4 / R, -1, 1))
        valid1 = R >= abs(D4)
        k = 0
        for s1 in (1, -1):
            th1 = base + s1 * ph + math.pi / 2
            c1, sn1 = np.cos(th1), np.sin(th1)
            c5 = (px * sn1 - py * c1 - D4) / D6
            for s5 in (1, -1):
                th5 = s5 * np.arccos(np.clip(c5, -1, 1))
                v5 = valid1 & (np.abs(c5) <= 1)
                sn5 = np.sin(th5)
                nx, ny = T06[:, 0, 0], T06[:, 1, 0]
                ox, oy = T06[:, 0, 1], T06[:, 1, 1]
                th6 = np.arctan2((-ox * sn1 + oy * c1) / sn5, (nx * sn1 - ny * c1) / sn5)
                th6 = np.where(np.abs(sn5) < 1e-9, 0.0, th6)
                T01 = dh(th1, 0)
                T45 = dh(th5, 4)
                T56 = dh(th6, 5)
                T14 = _inv(T01) @ T06 @ _inv(T56) @ _inv(T45)
                p13 = np.einsum('nij,j->ni', T14, np.array([0, -D4, 0, 1.0]))[:, :3]
                d13 = np.hypot(p13[:, 0], p13[:, 1])
                c3 = (d13 ** 2 - A2 ** 2 - A3 ** 2) / (2 * A2 * A3)
                v3 = v5 & (np.abs(c3) <= 1)
                for s3 in (1, -1):
                    th3 = s3 * np.arccos(np.clip(c3, -1, 1))
                    th2 = -np.arctan2(p13[:, 1], -p13[:, 0]) + np.arcsin(
                        np.clip(A3 * np.sin(th3) / d13, -1, 1))
                    T12 = dh(th2, 1)
                    T23 = dh(th3, 2)
                    T34 = _inv(T23) @ _inv(T12) @ T14
                    th4 = np.arctan2(T34[:, 1, 0], T34[:, 0, 0])
                    q = np.stack([th1, th2, th3, th4, th5, th6], axis=1)
                    sols[:, k] = q
                    ok[:, k] = v3 & np.all(np.isfinite(q), axis=1)
                    k += 1
    # wrap into [-pi, pi]; the +-2*pi joints then never bind, only the elbow's +-pi can (and the wrap
    # keeps the elbow inside it by construction, so the check below is a guard, not a filter)
    sols = (sols + math.pi) % (2 * math.pi) - math.pi
    ok &= np.all(np.abs(sols) <= JOINT_LIMIT + 1e-9, axis=2)
    return sols, ok


def _tool_flange_poses(x, y, z, rolls):
    """Flange poses, TCP at (x, y, z) (y, z arrays of equal length), tool z = +x (into the wall), per roll.
    Returns (len(y) * len(rolls), 4, 4), point-major."""
    y = np.asarray(y, float)
    n, m = len(y), len(rolls)
    zt = np.array([1.0, 0, 0])
    xt0 = np.array([0, 1.0, 0])
    yt0 = np.cross(zt, xt0)
    xt = np.cos(rolls)[:, None] * xt0 + np.sin(rolls)[:, None] * yt0      # (m,3)
    yt = np.cross(zt, xt)
    Ts = np.zeros((n, m, 4, 4))
    Ts[:, :, :3, 0] = xt
    Ts[:, :, :3, 1] = yt
    Ts[:, :, :3, 2] = zt
    Ts[:, :, 0, 3] = x - TCP_Z
    Ts[:, :, 1, 3] = y[:, None]
    Ts[:, :, 2, 3] = np.asarray(z, float)[:, None]
    Ts[:, :, 3, 3] = 1.0
    return Ts.reshape(n * m, 4, 4)


def reachable_grid(reach_x, y_vals, z_vals, link_x_max, roll_step_deg=15.0, chunk=4000):
    """Boolean (len(y), len(z)): TCP at (reach_x, y, z_rel) reachable, tool z into the wall, some roll."""
    rolls = np.radians(np.arange(0.0, 360.0, roll_step_deg))
    Y, Z = np.meshgrid(y_vals, z_vals, indexing='ij')
    yy, zz = Y.ravel(), Z.ravel()
    out = np.zeros(len(yy), bool)
    for s in range(0, len(yy), chunk):
        poses = _tool_flange_poses(reach_x, yy[s:s + chunk], zz[s:s + chunk], rolls)
        sols, ok = ik(poses)
        n = poses.shape[0]
        frames = fk_frames(sols.reshape(-1, 6))
        # joint-origin x of frames 2..6 must stay in front of the wall
        xmax = np.max(np.stack([f[:, 0, 3] for f in frames[1:]], axis=1), axis=1).reshape(n, 8)
        good = ok & (xmax <= link_x_max)
        out[s:s + chunk] = good.reshape(-1, len(rolls) * 8).any(axis=1)
    return out.reshape(len(y_vals), len(z_vals))


def build_reach(standoff, approach_m, link_clearance_m, y_vals, z_vals, roll_step_deg):
    """Reach grid at the contact plane AND the approach plane (a point needs both)."""
    link_x_max = standoff - link_clearance_m
    g = reachable_grid(standoff, y_vals, z_vals, link_x_max, roll_step_deg)
    if approach_m > 0:
        g &= reachable_grid(standoff - approach_m, y_vals, z_vals, link_x_max, roll_step_deg)
    return g


def runs(mask, vals):
    """Contiguous True runs of mask as [(lo, hi)] in vals units (grid cell centres)."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if not m and start is not None:
            out.append((float(vals[start]), float(vals[i - 1])))
            start = None
    if start is not None:
        out.append((float(vals[start]), float(vals[-1])))
    return out


def wall_z_mask(rel_mask, z_rel, z_wall, z0, lift_max):
    """Wall heights reachable with the lift anywhere in 0..lift_max: z in z_rel+z0+[0,lift_max]."""
    step = z_rel[1] - z_rel[0]
    csum = np.concatenate([[0], np.cumsum(rel_mask.astype(int))])
    lo = np.floor((z_wall - z0 - lift_max - z_rel[0]) / step + 1e-9).astype(int)
    hi = np.floor((z_wall - z0 - z_rel[0]) / step + 1e-9).astype(int)
    lo = np.clip(lo, 0, len(z_rel))
    hi = np.clip(hi + 1, 0, len(z_rel))
    return (csum[hi] - csum[lo]) > 0


class Model:
    def __init__(self, a):
        self.a = a
        self.z_rel = np.round(np.arange(-1.45, 1.451, a.z_step), 6)
        self.y_vals = np.round(np.arange(0.0, a.max_lateral + 1e-9, a.y_step), 6)
        self.z_wall = np.round(np.arange(a.z_min, a.z_max + 1e-9, a.z_step), 6)
        self._cache = {}

    def reach(self, standoff):
        key = round(standoff, 6)
        if key not in self._cache:
            a = self.a
            self._cache[key] = build_reach(standoff, a.approach_m, a.link_clearance_m,
                                           self.y_vals, self.z_rel, a.roll_step_deg)
        return self._cache[key]

    def wall_cover(self, standoff):
        """(len(y), len(z_wall)) bool: wall height reachable at lateral |y| from the stand."""
        g = self.reach(standoff)
        a = self.a
        return np.stack([wall_z_mask(g[i], self.z_rel, self.z_wall, a.arm_base_z0, a.lift_max)
                         for i in range(len(self.y_vals))])


def lateral_index(model, y):
    return int(round(abs(y) / model.a.y_step)) if abs(y) <= model.y_vals[-1] + 1e-9 else None


def strip_report(model, standoff, stand_y, strip_y):
    idx = lateral_index(model, strip_y - stand_y)
    if idx is None:
        return {'offset_m': strip_y - stand_y, 'z_intervals': [], 'z_missing': [(model.a.z_min, model.a.z_max)],
                'fixed_lift_max_span_m': 0.0}
    cover = model.wall_cover(standoff)[idx]
    iv = runs(cover, model.z_wall)
    miss = runs(~cover, model.z_wall)
    g = model.reach(standoff)[idx]
    rr = runs(g, model.z_rel)
    return {'offset_m': float(strip_y - stand_y), 'z_intervals': iv, 'z_missing': miss,
            'fixed_lift_max_span_m': max([hi - lo for lo, hi in rr], default=0.0)}


def half_width(model, standoff, margin=0.0):
    """Largest h with every |y| <= h fully covering [z_min, z_max]; None if even y = 0 fails."""
    cover = model.wall_cover(standoff).all(axis=1)
    if not cover[0]:
        return None
    first_bad = np.argmin(cover) if not cover.all() else len(cover)
    h = float(model.y_vals[first_bad - 1])
    return max(0.0, h - margin)


def common_interval(model, standoff, h):
    """Largest contiguous wall-height interval covered at every |y| <= h (y grid up to h)."""
    n = lateral_index(model, h) + 1
    cover = model.wall_cover(standoff)[:n].all(axis=0)
    r = runs(cover, model.z_wall)
    return max(r, key=lambda t: t[1] - t[0]) if r else None


def z0_window(model, standoff, h, step=0.02):
    """Range of arm-base heights z0 (lift retracted) for which every |y| <= h covers [z_min, z_max] fully.
    Returns (lo, hi) or None; scanned from 0 to 3 m in `step`. Shows how sensitive the result is to the mount."""
    a = model.a
    g = model.reach(standoff)[:lateral_index(model, h) + 1]
    good = []
    for z0 in np.arange(0.0, 3.0001, step):
        ok = all(wall_z_mask(row, model.z_rel, model.z_wall, z0, a.lift_max).all() for row in g)
        if ok:
            good.append(round(float(z0), 4))
    return (min(good), max(good)) if good else None


def group_strips(ys, span):
    """Greedy groups of strips whose lateral spread (last - first) <= span, in order."""
    groups, cur = [], []
    for y in sorted(ys):
        if cur and y - cur[0] > span + 1e-9:
            groups.append(cur)
            cur = []
        cur.append(y)
    if cur:
        groups.append(cur)
    return groups


def fmt_iv(iv):
    return ', '.join('%.2f..%.2f' % t for t in iv) if iv else 'none'


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--strips', type=int, default=19, help='number of vertical strips')
    p.add_argument('--pitch', type=float, default=0.16, help='strip spacing along the wall, m')
    p.add_argument('--strip-y0', type=float, default=0.0, help='lateral position of strip 0, m')
    p.add_argument('--z-min', type=float, default=0.5)
    p.add_argument('--z-max', type=float, default=1.9)
    p.add_argument('--standoff', type=float, nargs='+', default=[0.60, 0.65, 0.70, 0.75, 0.80],
                   help='arm base to wall distances to evaluate, m')
    p.add_argument('--stand-y', type=float, nargs='*', default=None,
                   help='stand lateral positions; adds a per-strip report for each. Default: none')
    p.add_argument('--stand-standoff', type=float, default=None,
                   help='standoff used for the per-strip report (default: first --standoff)')
    p.add_argument('--lift-max', type=float, default=1.5)
    p.add_argument('--arm-base-z0', type=float, default=1.17,
                   help='arm base height above the floor with the lift retracted, m')
    p.add_argument('--approach-m', type=float, default=0.12,
                   help='pre-contact distance that must also be reachable (0 = contact point only)')
    p.add_argument('--link-clearance-m', type=float, default=0.0,
                   help='joint origins must stay this far in front of the wall')
    p.add_argument('--margin-m', type=float, default=0.0, help='lateral margin subtracted from the recommendation')
    p.add_argument('--roll-step-deg', type=float, default=15.0)
    p.add_argument('--z-step', type=float, default=0.02)
    p.add_argument('--y-step', type=float, default=0.02)
    p.add_argument('--max-lateral', type=float, default=1.2)
    p.add_argument('--spans', type=float, nargs='+', default=[0.25, 0.5, 0.75, 1.0, 1.25],
                   help='stand spans for the z-limit trade-off table, m')
    p.add_argument('--json', default=None, help='also write the result here')
    a = p.parse_args(argv)

    model = Model(a)
    strips = [a.strip_y0 + i * a.pitch for i in range(a.strips)]
    res = {'model': 'analytic UR10e DH + closed-form IK, no RM4D map, no self-collision', 'params': vars(a),
           'standoffs': {}}
    print('Model: analytic UR10e (published DH), closed-form IK, lift 0..%.2f m, arm base z0 %.2f m,' %
          (a.lift_max, a.arm_base_z0))
    print('       tool z into the wall, free roll, approach %.2f m, link clearance %.2f m. No RM4D map,'
          % (a.approach_m, a.link_clearance_m))
    print('       no self-collision, no collision with the wall beyond joint origins.')
    print('Wall: %d strips, pitch %.2f m, z %.2f..%.2f m\n' % (a.strips, a.pitch, a.z_min, a.z_max))

    for d in a.standoff:
        h = half_width(model, d, a.margin_m)
        cover = model.wall_cover(d)
        entry = {'full_height_half_width_m': h}
        print('== standoff %.2f m ==' % d)
        zr = runs(cover[0], model.z_wall)
        print('  directly in front of the stand (lateral 0): reachable z %s' % fmt_iv(zr))
        if h is None:
            print('  NO strip is fully reachable at this standoff, even straight ahead.')
        else:
            print('  every strip with |lateral offset| <= %.2f m is fully reachable -> stand_max_span_m <= %.2f'
                  % (h, 2 * h))
            groups = group_strips(strips, 2 * h) if h > 0 else [[s] for s in strips]
            entry.update(stand_max_span_m=2 * h, stands_needed=len(groups),
                         stand_centres_m=[(g[0] + g[-1]) / 2 for g in groups])
            print('  %d stands cover the %d strips; centres (m along wall): %s' %
                  (len(groups), len(strips), ', '.join('%.3f' % ((g[0] + g[-1]) / 2) for g in groups)))
        trade = []
        print('  span (m) | stand-lateral-offset limit | z interval covered at every offset')
        for sp in a.spans:
            hh = sp / 2
            if lateral_index(model, hh) is None:
                continue
            iv = common_interval(model, d, hh)
            trade.append({'span_m': sp, 'z_interval': iv})
            print('  %8.2f | +-%.3f                    | %s' % (sp, hh, ('%.2f..%.2f' % iv) if iv else 'none'))
        entry['span_tradeoff'] = trade
        win = {}
        for sp in a.spans:
            if lateral_index(model, sp / 2) is not None:
                win['%.2f' % sp] = z0_window(model, d, sp / 2)
        entry['arm_base_z0_window_by_span'] = win
        print('  arm-base height z0 (lift retracted) that still covers the full z range, by span: ' +
              ', '.join('%s: %s' % (k, ('%.2f..%.2f' % v) if v else 'none') for k, v in win.items()))
        # lateral reach at each wall height from the stand
        rows = []
        for zv in (a.z_min, (a.z_min + a.z_max) / 2, a.z_max):
            zi = int(round((zv - a.z_min) / a.z_step))
            col = cover[:, zi]
            hw = float(model.y_vals[np.argmin(col) - 1]) if (col[0] and not col.all()) else (
                float(model.y_vals[-1]) if col[0] else None)
            rows.append({'z_m': zv, 'lateral_half_width_m': hw})
        entry['lateral_reach_by_height'] = rows
        print('  lateral half-width reachable per height: ' +
              ', '.join('z=%.2f: %s' % (r['z_m'], ('+-%.2f' % r['lateral_half_width_m'])
                                         if r['lateral_half_width_m'] is not None else 'none') for r in rows))
        res['standoffs']['%.2f' % d] = entry
        print()

    if a.stand_y:
        d = a.stand_standoff if a.stand_standoff is not None else a.standoff[0]
        print('== per-strip report, standoff %.2f m ==' % d)
        print('  strip  y(m)   serving stand  offset  reachable z (lift 0..%.2f)       fixed-lift span' % a.lift_max)
        rep = []
        for i, sy in enumerate(strips):
            st = min(a.stand_y, key=lambda s: abs(s - sy))
            r = strip_report(model, d, st, sy)
            full = not r['z_missing']
            print('  %5d  %5.2f  %6.2f        %+5.2f   %-30s %s  span %.2f' %
                  (i, sy, st, r['offset_m'], fmt_iv(r['z_intervals']), 'FULL' if full else 'PARTIAL',
                   r['fixed_lift_max_span_m']))
            rep.append(dict(strip=i, y_m=sy, stand_y_m=st, **r))
        res['per_strip'] = {'standoff_m': d, 'strips': rep}

    if a.json:
        with open(a.json, 'w') as f:
            json.dump(res, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
