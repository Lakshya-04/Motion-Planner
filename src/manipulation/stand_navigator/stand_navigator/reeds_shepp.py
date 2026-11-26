"""Reeds-Shepp shortest paths for a car that drives forward and in reverse at a minimum turn radius.

ROS-free, deterministic. The word formulas follow the classic Reeds-Shepp / Dubins-family closed forms (the
set PythonRobotics also uses); every candidate is integrated and kept only if it really ends at the goal, so a
wrong formula can cost optimality but never returns a path that misses the goal.

Poses are (x, y, yaw). A path is a list of `(kind, d)` pieces, kind in 'L', 'R', 'S' and d the signed length in
metres (negative = reverse); L turns left (positive curvature) when driven forward.
"""
import math

_TOL = 1e-6


def _mod2pi(x):
    v = math.fmod(x, 2.0 * math.pi)
    if v < -math.pi:
        v += 2.0 * math.pi
    elif v > math.pi:
        v -= 2.0 * math.pi
    return v


def _polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _lsl(x, y, p):
    u, t = _polar(x - math.sin(p), y - 1.0 + math.cos(p))
    if t >= 0.0:
        v = _mod2pi(p - t)
        if v >= 0.0:
            return ('LSL', (t, u, v))
    return None


def _lsr(x, y, p):
    u1, t1 = _polar(x + math.sin(p), y - 1.0 - math.cos(p))
    u1 *= u1
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        th = math.atan2(2.0, u)
        t = _mod2pi(t1 + th)
        v = _mod2pi(t - p)
        if t >= 0.0 and v >= 0.0:
            return ('LSR', (t, u, v))
    return None


def _lrl_a(x, y, p):
    u1, th = _polar(x - math.sin(p), y - 1.0 + math.cos(p))
    if u1 <= 4.0:
        a = math.acos(0.25 * u1)
        t = _mod2pi(a + th + math.pi / 2)
        u = _mod2pi(math.pi - 2 * a)
        v = _mod2pi(p - t - u)
        return ('LRL', (t, -u, v))
    return None


def _lrl_b(x, y, p):
    u1, th = _polar(x - math.sin(p), y - 1.0 + math.cos(p))
    if u1 <= 4.0:
        a = math.acos(0.25 * u1)
        t = _mod2pi(a + th + math.pi / 2)
        u = _mod2pi(math.pi - 2 * a)
        v = _mod2pi(-p + t + u)
        return ('LRL', (t, -u, -v))
    return None


def _lrl_c(x, y, p):
    u1, th = _polar(x - math.sin(p), y - 1.0 + math.cos(p))
    if u1 <= 4.0:
        u = math.acos(1.0 - u1 * u1 * 0.125)
        a = math.asin(2.0 * math.sin(u) / u1) if u1 > 1e-12 else 0.0
        t = _mod2pi(-a + th + math.pi / 2)
        v = _mod2pi(t - u - p)
        return ('LRL', (t, u, -v))
    return None


def _lrlr_a(x, y, p):
    u1, th = _polar(x + math.sin(p), y - 1.0 - math.cos(p))
    if u1 <= 2.0:
        a = math.acos((u1 + 2.0) * 0.25)
        t = _mod2pi(th + math.pi / 2 + a)
        u = _mod2pi(a)
        v = _mod2pi(p - t + 2.0 * u)
        if t >= 0.0 and u >= 0.0 and v >= 0.0:
            return ('LRLR', (t, u, -u, -v))
    return None


def _lrlr_b(x, y, p):
    u1, th = _polar(x + math.sin(p), y - 1.0 - math.cos(p))
    u2 = (20.0 - u1 * u1) / 16.0
    if 0.0 <= u2 <= 1.0 and u1 > 1e-12:
        u = math.acos(u2)
        a = math.asin(2.0 * math.sin(u) / u1)
        t = _mod2pi(th + math.pi / 2 - a)
        v = _mod2pi(t - p)
        if t >= 0.0 and v >= 0.0:
            return ('LRLR', (t, -u, -u, v))
    return None


def _lrsl(x, y, p):
    u1, th = _polar(x - math.sin(p), y - 1.0 + math.cos(p))
    if u1 >= 2.0:
        u = math.sqrt(u1 * u1 - 4.0) - 2.0
        a = math.atan2(2.0, u + 2.0)
        t = _mod2pi(th + math.pi / 2 + a)
        v = _mod2pi(t - p + math.pi / 2)
        if t >= 0.0 and v >= 0.0:
            return ('LRSL', (t, -math.pi / 2, -u, -v))
    return None


def _lslr(x, y, p):
    u1, th = _polar(x - math.sin(p), y - 1.0 + math.cos(p))
    if u1 >= 2.0:
        u = math.sqrt(u1 * u1 - 4.0) - 2.0
        a = math.atan2(u + 2.0, 2.0)
        t = _mod2pi(th - math.pi / 2 + a)
        v = _mod2pi(t - p - math.pi / 2)
        if t >= 0.0 and v >= 0.0:
            return ('LSLR', (t, u, math.pi / 2, -v))
    return None


def _lrsr(x, y, p):
    u1, th = _polar(x + math.sin(p), y - 1.0 - math.cos(p))
    if u1 >= 2.0:
        t = _mod2pi(th + math.pi / 2)
        u = u1 - 2.0
        v = _mod2pi(p - t - math.pi / 2)
        if t >= 0.0 and v >= 0.0:
            return ('LRSR', (t, -math.pi / 2, -u, -v))
    return None


def _lsrl(x, y, p):
    u1, th = _polar(x + math.sin(p), y - 1.0 - math.cos(p))
    if u1 >= 2.0:
        t = _mod2pi(th)
        u = u1 - 2.0
        v = _mod2pi(-p - t - math.pi / 2)
        if t >= 0.0 and v >= 0.0:
            return ('LSRL', (t, u, math.pi / 2, -v))
    return None


def _lrslr(x, y, p):
    u1, th = _polar(x + math.sin(p), y - 1.0 - math.cos(p))
    if u1 >= 4.0:
        u = math.sqrt(u1 * u1 - 4.0) - 4.0
        a = math.atan2(2.0, u + 4.0)
        t = _mod2pi(th + math.pi / 2 + a)
        v = _mod2pi(t - p)
        if t >= 0.0 and v >= 0.0:
            return ('LRSLR', (t, -math.pi / 2, -u, -math.pi / 2, v))
    return None


_FUNCS = (_lsl, _lsr, _lrl_a, _lrl_b, _lrl_c, _lrlr_a, _lrlr_b, _lrsl, _lslr, _lrsr, _lsrl, _lrslr)
_SWAP = {'L': 'R', 'R': 'L', 'S': 'S'}


def _end(pieces, radius=1.0):
    """End pose of a path from the origin."""
    x = y = th = 0.0
    for kind, d in pieces:
        if kind == 'S':
            x, y = x + d * math.cos(th), y + d * math.sin(th)
        else:
            k = (1.0 if kind == 'L' else -1.0) / radius
            nth = th + k * d
            x += (math.sin(nth) - math.sin(th)) / k
            y -= (math.cos(nth) - math.cos(th)) / k
            th = nth
    return x, y, th


def paths(start, goal, radius):
    """All verified Reeds-Shepp paths start -> goal, shortest first (ties by word and variant, deterministic)."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    x, y = (c * dx + s * dy) / radius, (-s * dx + c * dy) / radius
    phi = _mod2pi(goal[2] - start[2])
    out = []
    for flip in (False, True):
        for refl in (False, True):
            # timeflip: (x, y, phi) -> (-x, y, -phi); reflect: (x, y, phi) -> (x, -y, -phi)
            xx = -x if flip else x
            yy = -y if refl else y
            pp = phi * (-1.0 if flip != refl else 1.0)
            for f in _FUNCS:
                r = f(xx, yy, pp)
                if r is None:
                    continue
                word, ls = r
                kinds = [_SWAP[k] if refl else k for k in word]
                ls = [-v if flip else v for v in ls]
                pieces = [(k, v * radius) for k, v in zip(kinds, ls) if abs(v) > 1e-9]
                ex, ey, eth = _end(pieces, radius)
                if (math.hypot(ex - x * radius, ey - y * radius) < 1e-4 * max(1.0, radius)
                        and abs(_mod2pi(eth - phi)) < 1e-4):
                    out.append((sum(abs(d) for _, d in pieces), ''.join(k for k, _ in pieces), len(out), pieces))
    out.sort(key=lambda r: (round(r[0], 9), r[1], r[2]))
    return [(r[0], r[3]) for r in out]


def sample(start, pieces, radius, ds=0.05):
    """Poses along a path: [(x, y, yaw, gear, k)], gear +1 forward / -1 reverse, k the signed geometric
    curvature of the piece (+ left), so the yaw rate is k * v with v the signed speed."""
    out = [(start[0], start[1], start[2], 1, 0.0)]
    x, y, th = start
    for kind, d in pieces:
        gear = 1 if d >= 0 else -1
        n = max(1, int(math.ceil(abs(d) / ds)))
        k = 0.0 if kind == 'S' else (1.0 if kind == 'L' else -1.0) / radius
        step = d / n
        for _ in range(n):
            if kind == 'S':
                x, y = x + step * math.cos(th), y + step * math.sin(th)
            else:
                nth = th + k * step
                x += (math.sin(nth) - math.sin(th)) / k
                y -= (math.cos(nth) - math.cos(th)) / k
                th = nth
            out.append((x, y, th, gear, k))
    if len(out) > 1:
        x0, y0, t0, _, k0 = out[0]
        out[0] = (x0, y0, t0, out[1][3], out[1][4])
    return out
