"""Scan-to-scan matching for a planar base (numpy and scipy only, no ROS)."""
import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Valid returns of a LaserScan as an (n, 2) array in the sensor frame."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))


def transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def _normals(points, k=5):
    """Unit normals from a local line fit through each point's k nearest neighbours."""
    _, idx = cKDTree(points).query(points, k=min(k, len(points)))
    nbr = points[idx] - points[idx].mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', nbr, nbr)
    _, vec = np.linalg.eigh(cov)
    return vec[:, :, 0]      # eigenvector of the smallest eigenvalue


@dataclass
class MatchResult:
    x: float
    y: float
    yaw: float          # pose of the new scan's frame in the reference scan's frame
    rms: float          # point-to-line residual of the inliers, m
    inlier_frac: float
    converged: bool


def match(ref, new, iterations=30, max_dist=0.3, tol=1e-4):
    """Align `new` to `ref` (both (n, 2), the same frame) with point-to-line ICP.

    Returns the transform that maps `new` onto `ref`, which is the pose of the new frame in the
    reference frame, so for a robot it is the motion between the two scans.
    """
    if len(ref) < 10 or len(new) < 10:
        return MatchResult(0.0, 0.0, 0.0, float('inf'), 0.0, False)
    tree, nrm = cKDTree(ref), _normals(ref)
    x = y = th = 0.0
    rms, frac, ok = float('inf'), 0.0, False
    for _ in range(iterations):
        moved = transform(new, x, y, th)
        dist, idx = tree.query(moved)
        keep = dist < max_dist
        if keep.sum() < 10:
            break
        # trim the worst fifth so a few bad pairs do not pull the fit
        keep &= dist <= np.quantile(dist[keep], 0.8)
        p, q, n = moved[keep], ref[idx[keep]], nrm[idx[keep]]
        # small-angle step: n . (R p + t - q) = 0 for t = (dx, dy), rotation dth about the origin
        A = np.column_stack((n[:, 0], n[:, 1], p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]))
        b = -np.einsum('ij,ij->i', n, p - q)
        step, *_ = np.linalg.lstsq(A, b, rcond=None)
        dx, dy, dth = step
        c, s = math.cos(dth), math.sin(dth)
        # compose the step (applied after the current transform)
        x, y, th = c * x - s * y + dx, s * x + c * y + dy, th + dth
        rms = float(np.sqrt(np.mean((b - A @ step) ** 2)))
        frac = float(keep.sum()) / len(new)
        if abs(dx) + abs(dy) + abs(dth) < tol:
            ok = True
            break
    return MatchResult(x, y, th, rms, frac, ok)


def body_twist(res, dt):
    """(vx, vy, wz) in the body frame from a match over dt seconds (midpoint heading)."""
    c, s = math.cos(-res.yaw / 2.0), math.sin(-res.yaw / 2.0)
    return ((c * res.x - s * res.y) / dt, (s * res.x + c * res.y) / dt, res.yaw / dt)
