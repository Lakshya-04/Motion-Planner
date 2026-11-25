"""Python mirror of the wall_detector core (host tests and offline checks; the node runs the C++ core).

`detect_wall` repeats `wall_detector::detectWall` (scene_model::fitPlane: RANSAC on random point triples, PCA
refinement on the inliers, one re-collect, normal toward the sensor, u = up x normal, v = normal x u) and
`wall_scene_json` repeats `wallSceneJson`, with the node's defaults (inlier_threshold 0.01, 300 iterations,
min_inliers 200, min_inlier_fraction 0.3). The random draws differ from the C++ generator, so inlier sets can differ
by a few points; the plane agrees to the noise. Use it to check what the detector can do with a cloud, not to
replace it.

`detect_obstacles` has NO counterpart in the C++ core (the node reports no obstacles): it groups the points that
stand proud of the fitted plane into boxes, as a reference for what an obstacle stage has to deliver.
"""
import numpy as np

DEFAULTS = dict(inlier_threshold=0.01, ransac_iterations=300, min_inliers=200, min_inlier_fraction=0.3,
                max_range_m=0.0, seed=1)


def _pca_plane(pts):
    c = pts.mean(axis=0)
    w, v = np.linalg.eigh((pts - c).T @ (pts - c))
    return c, v[:, 0]


def detect_wall(points, viewpoint=(0.0, 0.0, 0.0), **kw):
    """Dominant plane of `points` (N x 3, target frame) or None. Returns a dict (origin, u, v, normal, ranges, rms,
    spreads, inlier mask over the kept points, `kept` the points that were fitted)."""
    p = {**DEFAULTS, **kw}
    pts = np.asarray(points, dtype=float)
    pts = pts[np.isfinite(pts).all(axis=1)]
    vp = np.asarray(viewpoint, dtype=float)
    if p['max_range_m'] > 0:
        pts = pts[np.linalg.norm(pts - vp, axis=1) <= p['max_range_m']]
    n = len(pts)
    if n < max(p['min_inliers'], 3):
        return None
    rng = np.random.default_rng(p['seed'])
    best = None
    for _ in range(p['ransac_iterations']):
        a, b, c = rng.integers(0, n, 3)
        if a == b or b == c or a == c:
            continue
        nrm = np.cross(pts[b] - pts[a], pts[c] - pts[a])
        ln = np.linalg.norm(nrm)
        if ln < 1e-12:
            continue
        mask = np.abs((pts - pts[a]) @ (nrm / ln)) <= p['inlier_threshold']
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < p['min_inliers']:
        return None
    centroid, normal = _pca_plane(pts[best])
    refined = np.abs((pts - centroid) @ normal) <= p['inlier_threshold']
    if refined.sum() >= p['min_inliers']:
        best = refined
        centroid, normal = _pca_plane(pts[best])
    if normal @ (vp - pts.mean(axis=0)) < 0:     # normal toward the sensor (detectWall's normal_hint)
        normal = -normal
    u = np.cross([0.0, 0.0, 1.0], normal)
    if np.linalg.norm(u) < 1e-6:
        u = np.array([1.0, 0.0, 0.0]) - normal * normal[0]
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    d = pts[best] - centroid
    du, dv, dn = d @ u, d @ v, np.abs(d @ normal)
    frac = best.sum() / n
    if frac < p['min_inlier_fraction']:
        return None
    srt = np.sort(dn)
    pct = lambda q: float(srt[min(len(srt) - 1, int(q * len(srt)))])  # noqa: E731
    return dict(origin=centroid, u=u, v=v, normal=normal, u_range=(float(du.min()), float(du.max())),
                v_range=(float(dv.min()), float(dv.max())), rms=float(np.sqrt((dn ** 2).mean())),
                spread_p95=pct(0.95), spread_p99=pct(0.99), num_inliers=int(best.sum()), inlier_fraction=float(frac),
                mask=best, kept=pts)


def wall_scene_json(det, wall_id=0, frame_id='odom', source='wall_fit_ref'):
    """The scene dict of wall_detector::wallSceneJson (one wall, no obstacles)."""
    o, n = det['origin'], det['normal']
    wall = dict(id=wall_id, plane_abcd=[*map(float, n), float(-n @ o)], origin_xyz=[*map(float, o)],
                basis_u=[*map(float, det['u'])], basis_v=[*map(float, det['v'])],
                normal_outward=[*map(float, n)], u_range_m=list(det['u_range']), v_range_m=list(det['v_range']),
                plane_rms_mm=det['rms'] * 1000.0, spread_p95_mm=det['spread_p95'] * 1000.0,
                spread_p99_mm=det['spread_p99'] * 1000.0, num_inliers=det['num_inliers'])
    return dict(frame_id=frame_id, source=source, units='meters', walls=[wall], obstacles=[])


def detect_obstacles(det, min_height_m=0.015, max_height_m=0.5, cell_m=0.02, min_cells=4):
    """Boxes from the points that stand proud of the plane (toward the sensor) by min_height_m..max_height_m:
    cells of cell_m in (u, v), 8-connected, groups of at least min_cells cells. Entries follow the wall_scene.json
    obstacle schema (id, parent_wall, kind 'box', box_center_xyz, box_extents_uvn, uv_on_wall, protrusion_*)."""
    pts, wall_id = det['kept'], 0
    d = pts - det['origin']
    h, u, v = d @ det['normal'], d @ det['u'], d @ det['v']
    sel = (h >= min_height_m) & (h <= max_height_m)
    cells = {}
    for i in np.flatnonzero(sel):
        cells.setdefault((int(np.floor(u[i] / cell_m)), int(np.floor(v[i] / cell_m))), []).append(i)
    seen, out = set(), []
    for start in cells:
        if start in seen:
            continue
        stack, group = [start], []
        seen.add(start)
        while stack:
            c = stack.pop()
            group.append(c)
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    nb = (c[0] + du, c[1] + dv)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        if len(group) < min_cells:
            continue
        idx = np.concatenate([cells[c] for c in group])
        u0, u1, v0, v1, hmax = u[idx].min(), u[idx].max(), v[idx].min(), v[idx].max(), h[idx].max()
        cu, cv = (u0 + u1) / 2, (v0 + v1) / 2
        centre = det['origin'] + det['u'] * cu + det['v'] * cv + det['normal'] * hmax / 2
        out.append(dict(id=len(out), parent_wall=wall_id, kind='box', box_center_xyz=[*map(float, centre)],
                        box_extents_uvn=[float(u1 - u0), float(v1 - v0), float(hmax)],
                        protrusion_min_m=float(h[idx].min()), protrusion_max_m=float(hmax),
                        uv_on_wall=dict(u=[float(u0), float(u1)], v=[float(v0), float(v1)])))
    return out
