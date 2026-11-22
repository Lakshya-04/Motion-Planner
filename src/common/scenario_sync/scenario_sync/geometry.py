"""Small geometry helpers: vertical walls, planar base poses, box primitives. numpy only."""
import math

import numpy as np

UP = np.array([0.0, 0.0, 1.0])


def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError('zero vector')
    return v / n


def wall_basis(normal):
    """(u, v, n) of a vertical wall with horizontal outward normal n (pointing at the robot).

    u = n x up (horizontal, along the wall), v = -up (v is metres DOWN from the wall point, the wall_scene.json
    convention). For n = (-1, 0, 0) this is u = +y, v = -z, exactly the smoke scenes' basis.
    """
    n = unit(normal)
    u = np.cross(n, UP)
    return unit(u), -UP.copy(), n


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def base_to_world(p, base):
    """Point/vector p in the base-start frame -> world. base = (x, y, yaw)."""
    return rot_z(base[2]) @ np.asarray(p, float) + np.array([base[0], base[1], 0.0])


def world_to_base(p, base):
    return rot_z(-base[2]) @ (np.asarray(p, float) - np.array([base[0], base[1], 0.0]))


def vec_world_to_base(v, base):
    return rot_z(-base[2]) @ np.asarray(v, float)


def vec_base_to_world(v, base):
    return rot_z(base[2]) @ np.asarray(v, float)


def quat_from_matrix(cols):
    """Unit quaternion (x, y, z, w) of the rotation whose columns are `cols`."""
    R = np.column_stack([np.asarray(c, float) for c in cols])
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def matrix_from_quat_xyzw(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class WallFrame:
    """Coordinates of a wall: plane offset s = n.p, and (u, v) of a world point relative to the wall point."""

    def __init__(self, point, normal):
        self.point = np.asarray(point, float)
        self.u, self.v, self.n = wall_basis(normal)
        self.offset = float(self.n @ self.point)

    def uv(self, p):
        d = np.asarray(p, float) - self.point
        return float(d @ self.u), float(d @ self.v)

    def dist(self, p):
        """Signed distance of p from the plane along the outward normal (positive on the robot's side)."""
        return float(self.n @ np.asarray(p, float)) - self.offset

    def world(self, u, v):
        return self.point + self.u * u + self.v * v


def angle_between(a, b):
    c = float(np.clip(unit(a) @ unit(b), -1.0, 1.0))
    return math.acos(c)
