"""Depth noise on an organised point grid (H, W, 3), NaN for no return. ROS-free.

Works on the range along each ray, so it does not care which way the sensor frame points. Order: range limits,
dropouts, flying pixels, range noise, outliers, quantisation.
"""
import numpy as np

from .presets import DepthParams


def _neighbours(a):
    """Left, right, down and up neighbours of every pixel of an (H, W) array, NaN past the border."""
    q = np.pad(a, 1, constant_values=np.nan)
    return [q[1:-1, 2:], q[1:-1, :-2], q[2:, 1:-1], q[:-2, 1:-1]]


def edge_info(rng_img, edge_abs, edge_rel):
    """(is_edge, nearest-neighbour range across the edge) for each pixel of a range image."""
    nb = _neighbours(rng_img)
    thresh = edge_abs + edge_rel * rng_img
    is_edge = np.zeros(rng_img.shape, bool)
    other = np.full(rng_img.shape, np.nan)
    with np.errstate(invalid='ignore'):
        for n in nb:
            d = np.abs(n - rng_img)
            hit = d > thresh
            take = hit & (~is_edge | (np.nan_to_num(d) > np.nan_to_num(np.abs(other - rng_img))))
            other = np.where(take, n, other)
            is_edge |= hit
    return is_edge, other


def incidence_cos(xyz, rng_img):
    """|cos| of the angle between the surface normal and the viewing ray, from the grid (1.0 where unknown)."""
    h, w = rng_img.shape
    cosv = np.ones((h, w))
    if h < 3 or w < 3:
        return cosv
    du = xyz[1:-1, 2:] - xyz[1:-1, :-2]
    dv = xyz[2:, 1:-1] - xyz[:-2, 1:-1]
    n = np.cross(du, dv)
    nn = np.linalg.norm(n, axis=-1)
    ray = xyz[1:-1, 1:-1] / rng_img[1:-1, 1:-1, None]
    with np.errstate(invalid='ignore', divide='ignore'):
        c = np.abs(np.sum(n * ray, axis=-1)) / nn
    c = np.where(np.isfinite(c), c, 1.0)
    cosv[1:-1, 1:-1] = c
    return cosv


def apply_depth_model(xyz, params: DepthParams, rng):
    """xyz: float array (H, W, 3) (or (N, 3), treated as one row). Returns (new xyz, stats dict)."""
    p = params
    shaped = xyz.ndim == 3
    pts = np.array(xyz if shaped else xyz[None, :, :], dtype=np.float64)
    if p.is_identity():
        return (pts if shaped else pts[0]).astype(xyz.dtype), {}
    r = np.linalg.norm(pts, axis=-1)
    valid = np.isfinite(r) & (r > 0)
    n_valid = int(valid.sum())
    stats = {'valid_in': n_valid}
    r = np.where(valid, r, np.nan)

    if p.min_range_m > 0:
        with np.errstate(invalid='ignore'):
            valid &= r >= p.min_range_m
    if p.max_range_m > 0:
        with np.errstate(invalid='ignore'):
            valid &= r <= p.max_range_m
    r = np.where(valid, r, np.nan)

    organised = pts.shape[0] > 2 and pts.shape[1] > 2
    edge, other = (edge_info(r, p.edge_abs_m, p.edge_rel) if organised else
                   (np.zeros(r.shape, bool), np.full(r.shape, np.nan)))
    drop_p = np.full(r.shape, p.dropout)
    if p.edge_dropout:
        drop_p = np.where(edge, 1 - (1 - drop_p) * (1 - p.edge_dropout), drop_p)
    if p.grazing_dropout and organised:
        angle = np.degrees(np.arccos(np.clip(incidence_cos(pts, r), 0.0, 1.0)))
        ramp = np.clip((angle - 60.0) / 25.0, 0.0, 1.0)
        drop_p = 1 - (1 - drop_p) * (1 - p.grazing_dropout * ramp)
    dropped = valid & (rng.random(r.shape) < drop_p)
    stats['dropped'] = int(dropped.sum())
    valid &= ~dropped

    rr = np.where(valid, r, np.nan)
    if p.flying_pixel_prob:
        fly = valid & edge & np.isfinite(other) & (rng.random(r.shape) < p.flying_pixel_prob)
        frac = rng.random(r.shape)
        rr = np.where(fly, rr + frac * (other - rr), rr)
        stats['flying'] = int(fly.sum())
    if p.sigma_a or p.sigma_b:
        sigma = p.sigma_a + p.sigma_b * rr ** 2
        rr = rr + rng.normal(size=r.shape) * sigma
    if p.outlier_prob:
        out = valid & (rng.random(r.shape) < p.outlier_prob)
        lo = max(p.min_range_m, 0.1)
        hi = p.max_range_m if p.max_range_m > 0 else max(float(np.nanmax(r)) if n_valid else 1.0, lo + 1.0)
        rr = np.where(out, rng.uniform(lo, hi, r.shape), rr)
        stats['outliers'] = int(out.sum())
    if p.quantum_m > 0:
        rr = np.round(rr / p.quantum_m) * p.quantum_m
    rr = np.where(rr > 0, rr, np.nan)    # noise can push a very near point behind the sensor: no return

    with np.errstate(invalid='ignore', divide='ignore'):
        scale = rr / r
    new = pts * scale[..., None]
    new[~np.isfinite(rr)] = np.nan
    stats['valid_out'] = int(np.isfinite(new[..., 0]).sum())
    return (new if shaped else new[0]).astype(xyz.dtype), stats
