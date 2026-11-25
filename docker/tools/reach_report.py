#!/usr/bin/env python3
"""Reach report: what reach do we really get from the robot, top to bottom?

ROS-free. Reads an RM4D map (see src/manipulation/reachability/include/reachability/core/io.hpp for
the file format), places a vertical wall patch in front of the arm and, for every lift height,
reports which wall cells the arm reaches with the tool axis normal to the wall.

Geometry (all in metres; x points from the arm base to the wall):
  arm-base-to-wall distance  D = chassis clearance + arm mount offset (arm base behind the chassis edge)
  arm base height at lift q  = arm_base_z0 + q   (arm_base_z0 = base lift_mount_height_m + lift
                               mount_height_m, i.e. BaseModel armBaseHeightAtLiftZero)
  wall cell (y, z)           -> RM4D key (pz = z - arm base z, theta = pi/2, x* = D, y* = y - arm_y)
Tool normal to the wall means the approach axis is +x (horizontal), so theta = pi/2 and the azimuth
is the +x axis itself: the map is queried with no rotation.

A cell counts as reachable "at fraction f of the arm reach" when the map holds it AND the tool lies
within f * reach of the arm base (distance from the arm base). 0.7 / 0.85 / 1.0 by default: the
margin you keep from the singular full stretch.

Usage:
  reach_report.py --map ur10e.rm4d --profile ur10e_telescopic.yaml --patch-z 0.6 2.0 --patch-y -0.5 0.5 \
      --clearance 0.42 --arm-offset 0.3 [--png out.png] [--json out.json]
"""
import argparse
import itertools
import json
import math
import struct
import sys

import numpy as np

MAGIC = b"RM4DMAP\0"


class RM4D:
    """Dense RM4D map: cells[iz, itheta, ix, iy] (uint8 counts)."""

    def __init__(self, params, cells, arm_dof=0, redundancy="", fingerprint=""):
        self.xy_extent, self.z_min, self.z_max, self.resolution, self.theta_bin = params
        self.cells = cells
        self.arm_dof = arm_dof
        self.redundancy = redundancy
        self.fingerprint = fingerprint

    @property
    def shape(self):
        return self.cells.shape

    def describe(self):
        if self.arm_dof == 0:
            return "arm and redundancy not recorded (map format version 1)"
        return "%d joints, redundancy: %s" % (self.arm_dof, self.redundancy or "not recorded")

    def estimate_reach(self):
        """Largest tool distance from the arm base over occupied cells (cell centres)."""
        nz, _, nxy, _ = self.cells.shape
        occ = self.cells.any(axis=1)  # [iz, ix, iy]
        iz, ix, iy = np.nonzero(occ)
        if iz.size == 0:
            return 0.0
        r = self.resolution
        x = (ix + 0.5) * r - self.xy_extent
        y = (iy + 0.5) * r - self.xy_extent
        z = self.z_min + (iz + 0.5) * r
        return float(np.sqrt(x * x + y * y + z * z).max())


def dims(xy_extent, z_min, z_max, resolution, theta_bin):
    nxy = max(1, int(math.ceil(2.0 * xy_extent / resolution)))
    nz = max(1, int(math.ceil((z_max - z_min) / resolution)))
    nth = max(1, int(math.ceil(math.pi / theta_bin)))
    return nz, nth, nxy


def load_map(path):
    with open(path, "rb") as f:
        buf = f.read()
    if buf[:8] != MAGIC:
        raise ValueError("%s: not a reachability map" % path)
    (version,) = struct.unpack_from("<I", buf, 8)
    if version not in (1, 2):
        raise ValueError("%s: unsupported map format version %d" % (path, version))
    params = struct.unpack_from("<5d", buf, 12)
    off = 12 + 40
    (fp_len,) = struct.unpack_from("<I", buf, off)
    off += 4
    fp = buf[off:off + fp_len].decode("utf-8", "replace")
    off += fp_len
    arm_dof, red = 0, ""
    if version >= 2:
        (arm_dof, red_len) = struct.unpack_from("<II", buf, off)
        off += 8
        red = buf[off:off + red_len].decode("utf-8", "replace")
        off += red_len
    (n,) = struct.unpack_from("<Q", buf, off)
    off += 8
    nz, nth, nxy = dims(*params)
    if n != nz * nth * nxy * nxy or len(buf) - off < n:
        raise ValueError("%s: cell count does not match the map parameters" % path)
    cells = np.frombuffer(buf, dtype=np.uint8, count=n, offset=off).reshape(nz, nth, nxy, nxy)
    return RM4D(params, cells, arm_dof, red, fp)


def save_map(path, m, fingerprint="", arm_dof=0, redundancy=""):
    """Writes the same binary format as reachability::saveMap (used by tests)."""
    with open(path, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<I", 2))
        f.write(struct.pack("<5d", m.xy_extent, m.z_min, m.z_max, m.resolution, m.theta_bin))
        fp = fingerprint.encode()
        f.write(struct.pack("<I", len(fp)) + fp)
        red = redundancy.encode()
        f.write(struct.pack("<II", arm_dof, len(red)) + red)
        f.write(struct.pack("<Q", m.cells.size))
        f.write(np.ascontiguousarray(m.cells).tobytes())


def synthetic_shell_map(reach=1.3, r_min=0.0, xy_extent=1.8, z_min=-1.0, z_max=1.8,
                        resolution=0.05, theta_bin=math.pi / 12):
    """Analytic sphere-shell model: every tool pose with r_min <= |p| <= reach is reachable, any
    polar angle. Cell centres decide, so edges are accurate to about one resolution."""
    nz, nth, nxy = dims(xy_extent, z_min, z_max, resolution, theta_bin)
    x = (np.arange(nxy) + 0.5) * resolution - xy_extent
    z = z_min + (np.arange(nz) + 0.5) * resolution
    r = np.sqrt(z[:, None, None] ** 2 + x[None, :, None] ** 2 + x[None, None, :] ** 2)
    ok = ((r <= reach) & (r >= r_min)).astype(np.uint8)
    cells = np.broadcast_to(ok[:, None, :, :], (nz, nth, nxy, nxy)).copy()
    return RM4D((xy_extent, z_min, z_max, resolution, theta_bin), cells)


def load_profile(path):
    import yaml
    with open(path) as f:
        p = yaml.safe_load(f)
    lift = p.get("lift") or {}
    base = p.get("base") or {}
    out = {}
    if "stroke_m" in lift:
        out["stroke"] = float(lift["stroke_m"])
    if "mount_height_m" in lift and "lift_mount_height_m" in base:
        out["arm_base_z0"] = float(base["lift_mount_height_m"]) + float(lift["mount_height_m"])
    return out


def reach_grid(m, D, ys_rel, zs_rel, reach, fraction):
    """Boolean [len(zs_rel), len(ys_rel)]: reachable with the tool axis +x at distance D.
    zs_rel are wall heights relative to the arm base; ys_rel are lateral offsets from the arm."""
    r = m.resolution
    ix = int(math.floor((D + m.xy_extent) / r))
    # same expression as Map4D::keyOf for axis (1, 0, 0): theta = atan2(1, 0) = pi/2
    ith = min(m.shape[1] - 1, int(math.floor(math.atan2(1.0, 0.0) / m.theta_bin)))
    iy = np.floor((ys_rel + m.xy_extent) / r).astype(int)
    iz = np.floor((zs_rel - m.z_min) / r).astype(int)
    ok = np.zeros((len(zs_rel), len(ys_rel)), dtype=bool)
    if not (0 <= ix < m.shape[2]):
        return ok
    vy = (iy >= 0) & (iy < m.shape[3])
    vz = (iz >= 0) & (iz < m.shape[0])
    sub = np.zeros_like(ok)
    zi, yi = np.nonzero(vz[:, None] & vy[None, :])
    sub[zi, yi] = m.cells[iz[zi], ith, ix, iy[yi]] > 0
    dist = np.sqrt(D * D + ys_rel[None, :] ** 2 + zs_rel[:, None] ** 2)
    return sub & (dist <= fraction * reach + 1e-9)


def min_cover(masks, universe, max_k=6):
    """Fewest lift positions whose union covers `universe` (int bitmask), exact up to max_k, then
    greedy; of several minimal sets the one with the least lift travel. masks: list of (lift, bitmask). Returns list of lifts (sorted) or None if impossible."""
    cand = [(q, b & universe) for q, b in masks if b & universe]
    covered = 0
    for _, b in cand:
        covered |= b
    if covered != universe:
        universe = covered  # caller reports the never-reachable part separately
    if universe == 0:
        return []
    for k in range(1, max_k + 1):
        if len(cand) > 80 and k > 3:
            break
        best = None
        for combo in itertools.combinations(cand, k):
            u = 0
            for _, b in combo:
                u |= b
            if u == universe:
                lifts = sorted(q for q, _ in combo)
                key = (lifts[-1] - lifts[0], lifts)   # smallest lift travel among the minimal sets
                if best is None or key < best:
                    best = key
        if best is not None:
            return best[1]
    chosen, u = [], 0  # greedy fallback
    while u != universe:
        q, b = max(cand, key=lambda c: bin(c[1] & ~u).count("1"))
        chosen.append(q)
        u |= b
    return sorted(chosen)


def runs(vals, step):
    """Compress a sorted list of values on a grid into [(lo, hi)] runs."""
    out = []
    for v in vals:
        if out and v - out[-1][1] <= step * 1.5:
            out[-1][1] = v
        else:
            out.append([v, v])
    return [(a, b) for a, b in out]


def build_report(m, *, patch_z, patch_y, clearance, arm_offset, stroke, arm_base_z0, reach=None,
                 arm_y=None, lift_step=0.05, fractions=(0.7, 0.85, 1.0), cell=None,
                 scan_z=(0.0, 3.0), cover_fraction=0.85):
    reach = reach if reach else m.estimate_reach()
    cell = cell or m.resolution
    D = clearance + arm_offset
    arm_y = 0.5 * (patch_y[0] + patch_y[1]) if arm_y is None else arm_y
    wz = np.arange(patch_z[0], patch_z[1] + 1e-9, cell)
    wy = np.arange(patch_y[0], patch_y[1] + 1e-9, cell)
    nl = int(round(stroke / lift_step))
    lifts = np.array([min(stroke, i * lift_step) for i in range(nl + 1)])
    sz = np.arange(scan_z[0], scan_z[1] + 1e-9, cell)
    rep = {
        "inputs": dict(map=m.describe(), arm_to_wall_m=D, reach_m=reach, arm_base_z0=arm_base_z0,
                       stroke_m=stroke, arm_y=arm_y, patch_z=list(patch_z), patch_y=list(patch_y),
                       cell_m=cell, lift_step_m=lift_step, clearance_m=clearance,
                       arm_offset_m=arm_offset),
        "warnings": [],
        "fractions": {},
    }
    if m.z_min > -reach or m.z_max < reach:
        rep["warnings"].append(
            "map z window %.2f..%.2f m is narrower than +-reach %.2f m: cells outside it count as "
            "unreachable (the lowest/highest z are clipped)" % (m.z_min, m.z_max, reach))
    if m.xy_extent < reach:
        rep["warnings"].append("map xy half-width %.2f m is below the reach %.2f m" % (m.xy_extent, reach))
    for f in fractions:
        grids = []   # per lift: [nz, ny] bool over the patch
        for q in lifts:
            grids.append(reach_grid(m, D, wy - arm_y, wz - (arm_base_z0 + q), reach, f))
        grids = np.array(grids)                   # [lift, z, y]
        per_lift = grids.mean(axis=(1, 2))
        union = grids.any(axis=0)                 # [z, y]
        # lowest / highest reachable wall z (unclipped scan, at the patch's centre line)
        yc = np.array([0.0 if arm_y is None else (0.5 * (patch_y[0] + patch_y[1]) - arm_y)])
        ext = {}
        for name, q in (("lift_min", lifts[0]), ("lift_max", lifts[-1])):
            g = reach_grid(m, D, yc, sz - (arm_base_z0 + q), reach, f)[:, 0]
            zz = sz[g]
            ext[name] = {"lift": float(q), "z_low": float(zz.min()) if zz.size else None,
                         "z_high": float(zz.max()) if zz.size else None}
        # minimum lift positions to cover the (coverable) patch
        masks = []
        for q, g in zip(lifts, grids):
            masks.append((float(q), int("".join("1" if v else "0" for v in g.ravel()) or "0", 2)))
        universe = int("".join("1" if v else "0" for v in union.ravel()) or "0", 2)
        cover = min_cover(masks, universe)
        never = ~union
        nz_idx, ny_idx = np.nonzero(never)
        never_rows = sorted(set(wz[nz_idx].tolist()))
        rep["fractions"][str(f)] = {
            "per_lift_fraction": {"%.3f" % q: float(v) for q, v in zip(lifts, per_lift)},
            "union_fraction": float(union.mean()),
            "per_height_fraction": [float(v) for v in union.mean(axis=1)],
            "z": [float(v) for v in wz],
            "extent": ext,
            "min_lifts": len(cover),
            "lift_heights": cover,
            "never_reachable_cells": int(never.sum()),
            "never_reachable_z_runs": [(float(a), float(b)) for a, b in runs(never_rows, cell)],
            "never_reachable_y_range": ([float(wy[ny_idx].min()), float(wy[ny_idx].max())]
                                        if never.any() else None),
        }
        rep["fractions"][str(f)]["_grids"] = grids
    rep["_wz"], rep["_wy"], rep["_lifts"] = wz, wy, lifts
    return rep


def format_report(rep, cover_fraction=None):
    i = rep["inputs"]
    L = []
    L.append("REACH REPORT")
    L.append("  map: %s" % i["map"])
    L.append("  arm base to wall %.3f m (clearance %.3f + arm offset %.3f), reach %.3f m" %
             (i["arm_to_wall_m"], i["clearance_m"], i["arm_offset_m"], i["reach_m"]))
    L.append("  arm base height at lift 0: %.3f m, stroke %.3f m, lift step %.3f m" %
             (i["arm_base_z0"], i["stroke_m"], i["lift_step_m"]))
    L.append("  patch z %.2f..%.2f m, y %.2f..%.2f m (arm at y=%.2f), cell %.3f m" %
             (i["patch_z"][0], i["patch_z"][1], i["patch_y"][0], i["patch_y"][1], i["arm_y"], i["cell_m"]))
    for w in rep["warnings"]:
        L.append("  WARNING: " + w)
    fr = sorted(rep["fractions"], key=float)
    L.append("")
    L.append("Reachable fraction of the patch per lift height (tool normal to the wall):")
    L.append("  lift[m] " + "".join("  %4.0f%% reach" % (float(f) * 100) for f in fr))
    lifts = list(rep["fractions"][fr[0]]["per_lift_fraction"])
    for q in lifts:
        L.append("  %6s  " % q + "".join("  %10.1f%%" % (100 * rep["fractions"][f]["per_lift_fraction"][q])
                                         for f in fr))
    L.append("")
    for f in fr:
        d = rep["fractions"][f]
        e = d["extent"]
        fmt = lambda v: "none" if v is None else "%.2f" % v
        L.append("At %.0f%% of reach:" % (float(f) * 100))
        L.append("  reachable wall z at lift min (%.2f): %s .. %s m; at lift max (%.2f): %s .. %s m" % (
            e["lift_min"]["lift"], fmt(e["lift_min"]["z_low"]), fmt(e["lift_min"]["z_high"]),
            e["lift_max"]["lift"], fmt(e["lift_max"]["z_low"]), fmt(e["lift_max"]["z_high"])))
        L.append("  patch coverable by any lift: %.1f%%; minimum lift positions: %d at %s m" % (
            100 * d["union_fraction"], d["min_lifts"],
            ", ".join("%.2f" % q for q in d["lift_heights"]) or "-"))
        if d["never_reachable_cells"]:
            L.append("  FLAG: %d patch cells never reachable (z runs %s, y %s)" % (
                d["never_reachable_cells"],
                ", ".join("%.2f..%.2f" % r for r in d["never_reachable_z_runs"]),
                "%.2f..%.2f" % tuple(d["never_reachable_y_range"])))
        else:
            L.append("  no patch cell is out of reach")
    L.append("")
    L.append(heat_strip(rep))
    return "\n".join(L)


def heat_strip(rep, fraction=None, width=20):
    """ASCII strip at one reach fraction (default 0.85 or the nearest): wall z (top first) vs the share
    of the patch width reachable at lift min, lift mid, lift max and by the best lift."""
    fr = sorted(rep["fractions"], key=float)
    f = min(fr, key=lambda v: abs(float(v) - (fraction if fraction else 0.85)))
    g = rep["fractions"][f]["_grids"]          # [lift, z, y]
    z = rep["fractions"][f]["z"]
    cols = [("lift %.2f" % rep["_lifts"][0], g[0].mean(axis=1)),
            ("lift %.2f" % rep["_lifts"][len(g) // 2], g[len(g) // 2].mean(axis=1)),
            ("lift %.2f" % rep["_lifts"][-1], g[-1].mean(axis=1)),
            ("best lift", np.array(rep["fractions"][f]["per_height_fraction"]))]
    L = ["Heat strip at %.0f%% of reach: wall z vs patch width reachable (full bar = all, empty = none)" %
         (float(f) * 100),
         "  z[m]  " + "  ".join("%-*s" % (width + 2, n) for n, _ in cols)]
    step = max(1, len(z) // 40)
    for k in range(len(z) - 1, -1, -step):
        row = ["%5.2f  " % z[k]]
        for _, v in cols:
            n = int(round(v[k] * width))
            row.append("|" + ("#" * n).ljust(width) + "|")
        L.append("  ".join(row))
    return "\n".join(L)


def write_png(rep, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fr = sorted(rep["fractions"], key=float)
    wz, lifts = rep["_wz"], rep["_lifts"]
    fig, ax = plt.subplots(1, len(fr) + 1, figsize=(4 * (len(fr) + 1), 5),
                           gridspec_kw={"width_ratios": [1] * len(fr) + [0.6]})
    for a, f in zip(ax, fr):
        g = rep["fractions"][f]["_grids"].mean(axis=2).T   # [z, lift]
        im = a.imshow(g, origin="lower", aspect="auto", vmin=0, vmax=1, cmap="viridis",
                      extent=[lifts[0], lifts[-1], wz[0], wz[-1]])
        a.set_title("%.0f%% of reach" % (float(f) * 100))
        a.set_xlabel("lift [m]")
        a.set_ylabel("wall z [m]")
    fig.colorbar(im, ax=ax[:-1], label="reachable fraction of patch width")
    for f in fr:
        ax[-1].plot(rep["fractions"][f]["per_height_fraction"], wz, label="%.0f%%" % (float(f) * 100))
    ax[-1].set_xlim(0, 1.02)
    ax[-1].set_xlabel("reachable (any lift)")
    ax[-1].set_title("heat strip")
    ax[-1].legend()
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--profile", help="arm profile yaml: supplies lift stroke and arm base height at lift 0")
    ap.add_argument("--patch-z", nargs=2, type=float, required=True, metavar=("LO", "HI"))
    ap.add_argument("--patch-y", nargs=2, type=float, default=[0.0, 0.0], metavar=("LO", "HI"))
    ap.add_argument("--clearance", type=float, required=True, help="chassis edge to wall (m)")
    ap.add_argument("--arm-offset", type=float, default=0.0, help="arm base behind the chassis edge (m)")
    ap.add_argument("--stroke", type=float, help="lift stroke (m); default from the profile")
    ap.add_argument("--arm-base-z0", type=float, help="arm base height at lift 0 (m); default from the profile")
    ap.add_argument("--reach", type=float, help="arm reach (m) the fractions refer to (e.g. 1.3 for the UR10e "
                    "datasheet reach); default: largest tool distance in the map, which includes the tool offset")
    ap.add_argument("--arm-y", type=float, help="arm base lateral position; default patch centre")
    ap.add_argument("--lift-step", type=float, default=0.05)
    ap.add_argument("--fractions", nargs="+", type=float, default=[0.7, 0.85, 1.0])
    ap.add_argument("--cell", type=float, help="wall cell size (m); default the map resolution")
    ap.add_argument("--png")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    prof = load_profile(a.profile) if a.profile else {}
    stroke = a.stroke if a.stroke is not None else prof.get("stroke")
    z0 = a.arm_base_z0 if a.arm_base_z0 is not None else prof.get("arm_base_z0")
    if stroke is None or z0 is None:
        ap.error("give --profile, or both --stroke and --arm-base-z0")
    m = load_map(a.map)
    rep = build_report(m, patch_z=a.patch_z, patch_y=a.patch_y, clearance=a.clearance,
                       arm_offset=a.arm_offset, stroke=stroke, arm_base_z0=z0, reach=a.reach,
                       arm_y=a.arm_y, lift_step=a.lift_step, fractions=a.fractions, cell=a.cell)
    print(format_report(rep))
    if a.png:
        write_png(rep, a.png)
        print("\nwrote " + a.png)
    if a.json:
        clean = json.loads(json.dumps(
            {k: ({kk: {k3: v3 for k3, v3 in vv.items() if k3 != "_grids"} for kk, vv in v.items()}
                 if k == "fractions" else v) for k, v in rep.items() if not k.startswith("_")}))
        with open(a.json, "w") as f:
            json.dump(clean, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
