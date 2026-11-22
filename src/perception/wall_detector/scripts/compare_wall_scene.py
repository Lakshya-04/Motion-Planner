#!/usr/bin/env python3
"""Compare a detected wall scene (wall_scene.json from wall_detector) with the ground-truth scene of the scenario.

    compare_wall_scene.py DETECTED.json TRUTH.json [--wall-id 0] [--max-angle-deg 2] [--max-offset-mm 20]
                          [--max-extent-mm 50] [--json OUT.json]

ROS-free (numpy only). Reports
  * angle_deg    angle between the two plane normals (sign-insensitive);
  * offset_mm    distance between the planes: the detected plane's centre point to the true plane (mm);
  * extent       the detected rectangle's corners expressed in the TRUTH wall's (u, v) axes against the true u / v
                 range: error of each edge in mm and the largest of them (the two scenes may use different in-plane
                 axes, so the ranges are compared in the truth basis, not component by component); `iou` of the rectangles;
  * obstacle recall  true obstacles that a detected obstacle matches (box centres within max(30 mm, half the true
                 box's larger face side)), plus the number of detected obstacles with no true one. Recall is None when
                 the truth has no obstacles.
Exit code 0 when every figure is within its limit, 1 otherwise (obstacle recall is checked only with --min-recall).
The real wall_detector reports no obstacles, so its recall on a scene with obstacles is 0.
"""
import argparse
import json
import sys

import numpy as np


def _wall(scene, wall_id):
    return next(w for w in scene['walls'] if w['id'] == wall_id)


def _plane(w):
    n = np.asarray(w['normal_outward'], dtype=float)
    return np.asarray(w['origin_xyz'], dtype=float), n / np.linalg.norm(n)


def compare(detected, truth, wall_id=0):
    dw, tw = _wall(detected, wall_id), _wall(truth, wall_id)
    do, dn = _plane(dw)
    to, tn = _plane(tw)
    angle = float(np.degrees(np.arccos(np.clip(abs(dn @ tn), -1.0, 1.0))))
    offset = float(abs(tn @ (do - to)) * 1000.0)
    # detected rectangle -> truth (u, v)
    tu, tv = np.asarray(tw['basis_u'], float), np.asarray(tw['basis_v'], float)
    du, dv = np.asarray(dw['basis_u'], float), np.asarray(dw['basis_v'], float)
    us, vs = [], []
    for a in dw['u_range_m']:
        for b in dw['v_range_m']:
            p = do + du * a + dv * b - to
            us.append(p @ tu)
            vs.append(p @ tv)
    det_u, det_v = (min(us), max(us)), (min(vs), max(vs))
    tru_u, tru_v = tuple(tw['u_range_m']), tuple(tw['v_range_m'])
    edges = [det_u[0] - tru_u[0], det_u[1] - tru_u[1], det_v[0] - tru_v[0], det_v[1] - tru_v[1]]
    inter = max(0.0, min(det_u[1], tru_u[1]) - max(det_u[0], tru_u[0])) * \
        max(0.0, min(det_v[1], tru_v[1]) - max(det_v[0], tru_v[0]))
    area = lambda r, s: (r[1] - r[0]) * (s[1] - s[0])  # noqa: E731
    union = area(det_u, det_v) + area(tru_u, tru_v) - inter
    extent = dict(detected_u=list(det_u), detected_v=list(det_v), truth_u=list(tru_u), truth_v=list(tru_v),
                  edge_error_mm=[abs(e) * 1000.0 for e in edges], max_edge_error_mm=max(abs(e) for e in edges) * 1000.0,
                  iou=float(inter / union) if union > 0 else 0.0)
    t_obs, d_obs = truth.get('obstacles', []), detected.get('obstacles', [])
    matched, used = 0, set()
    for t in t_obs:
        side = max(t['box_extents_uvn'][:2])
        tol = max(0.03, 0.5 * side)
        for j, d in enumerate(d_obs):
            if j in used:
                continue
            if np.linalg.norm(np.asarray(d['box_center_xyz']) - np.asarray(t['box_center_xyz'])) <= tol:
                used.add(j)
                matched += 1
                break
    return dict(angle_deg=angle, offset_mm=offset, extent=extent,
                obstacles=dict(truth=len(t_obs), detected=len(d_obs), matched=matched,
                               recall=(matched / len(t_obs)) if t_obs else None,
                               false_positives=len(d_obs) - len(used)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('detected')
    ap.add_argument('truth')
    ap.add_argument('--wall-id', type=int, default=0)
    ap.add_argument('--max-angle-deg', type=float, default=2.0)
    ap.add_argument('--max-offset-mm', type=float, default=20.0)
    ap.add_argument('--max-extent-mm', type=float, default=50.0)
    ap.add_argument('--min-recall', type=float, default=None)
    ap.add_argument('--json', help='also write the figures here')
    a = ap.parse_args(argv)
    with open(a.detected) as f:
        det = json.load(f)
    with open(a.truth) as f:
        tru = json.load(f)
    r = compare(det, tru, a.wall_id)
    ok = (r['angle_deg'] <= a.max_angle_deg and r['offset_mm'] <= a.max_offset_mm
          and r['extent']['max_edge_error_mm'] <= a.max_extent_mm)
    ob = r['obstacles']
    if a.min_recall is not None and ob['recall'] is not None:
        ok = ok and ob['recall'] >= a.min_recall
    r['ok'] = bool(ok)
    e = r['extent']
    print(f"normal angle error   {r['angle_deg']:.3f} deg   (limit {a.max_angle_deg})")
    print(f"plane offset error   {r['offset_mm']:.2f} mm    (limit {a.max_offset_mm})")
    print(f"extent edge errors   u0 {e['edge_error_mm'][0]:.1f}  u1 {e['edge_error_mm'][1]:.1f}  "
          f"v0 {e['edge_error_mm'][2]:.1f}  v1 {e['edge_error_mm'][3]:.1f} mm, worst {e['max_edge_error_mm']:.1f} "
          f"(limit {a.max_extent_mm}), IoU {e['iou']:.3f}")
    rec = 'n/a (no true obstacles)' if ob['recall'] is None else f"{ob['recall']:.2f} ({ob['matched']} of {ob['truth']})"
    print(f"obstacle recall      {rec}, {ob['false_positives']} detected without a true match")
    print('perception check: ' + ('OK' if ok else 'OUT OF TOLERANCE'))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(r, f, indent=2)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
