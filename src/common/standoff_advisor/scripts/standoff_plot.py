#!/usr/bin/env python3
"""Plot a standoff_sweep result: reachable fraction and pose quality vs distance, recommended band shaded.

  standoff_plot.py sweep.json -o sweep.png [--title text] [--today 0.98]

Reads the JSON written by `standoff_sweep --out-json`. Uses matplotlib when it is installed and writes a PNG;
without it prints an ASCII chart (and writes nothing). ROS-free.
"""
import argparse
import json
import sys


def ascii_chart(rows, rec, today):
    out = ["distance  reachable fraction (= plateau, # other)   quality"]
    for r in rows:
        n = round(50 * r["reachable_fraction"])
        mark = "=" if r["in_plateau"] else "#"
        tag = ""
        if rec.get("found") and abs(r["distance_m"] - rec["recommended_standoff_m"]) < 0.5e-2 + 1e-9:
            tag += " <recommended"
        if today is not None and abs(r["distance_m"] - today) < 1e-2 + 1e-9:
            tag += " <today"
        out.append("%.2f  |%-50s| %5.1f%%  q=%.2f%s" % (
            r["distance_m"], mark * n, 100 * r["reachable_fraction"], r["mean_quality"], tag))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json")
    ap.add_argument("-o", "--out", default="standoff.png")
    ap.add_argument("--title", default="")
    ap.add_argument("--today", type=float, default=None, help="mark this arm-base distance (the dock used today)")
    a = ap.parse_args()
    data = json.load(open(a.json))
    rows, rec = data["sweep"], data["recommended"]
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(ascii_chart(rows, rec, a.today))
        print("matplotlib is not installed: ASCII chart only", file=sys.stderr)
        return 0

    d = [r["distance_m"] for r in rows]
    frac = [100 * r["reachable_fraction"] for r in rows]
    robust = [100 * r["robust_fraction"] for r in rows]
    qual = [r["mean_quality"] for r in rows]
    fits = [r["footprint_fits"] for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    for ax in (ax1, ax2):
        if rec.get("found"):
            ax.axvspan(rec["min_distance_m"], rec["max_distance_m"], color="#1f77b4", alpha=0.12,
                       label="plateau")
            if rec["robust"]:
                ax.axvspan(rec["robust_min_m"], rec["robust_max_m"], color="#1f77b4", alpha=0.25,
                           label="robust band (+-%.0f mm)" % (1000 * rec["nav_tolerance_m"]))
            ax.axvline(rec["recommended_standoff_m"], color="#1f77b4", lw=2,
                       label="recommended %.2f m" % rec["recommended_standoff_m"])
        if a.today is not None:
            ax.axvline(a.today, color="#d62728", ls="--", lw=1.5, label="today %.2f m" % a.today)
        # distances where the footprint does not fit the clearance
        run = None
        for x, f in zip(d, fits):
            if not f and run is None:
                run = x
            if f and run is not None:
                ax.axvspan(run, x, color="#7f7f7f", alpha=0.25, lw=0)
                run = None
        if run is not None:
            ax.axvspan(run, d[-1], color="#7f7f7f", alpha=0.25, lw=0)
        ax.grid(alpha=0.3)
    ax1.plot(d, frac, color="#2ca02c", lw=2, label="reachable fraction")
    ax1.plot(d, robust, color="#2ca02c", lw=1, ls=":", label="worst within nav tolerance")
    ax1.set_ylabel("targets reachable (%)")
    ax1.set_ylim(-2, 104)
    ax1.legend(loc="best", fontsize=8)
    ax2.plot(d, qual, color="#ff7f0e", lw=2)
    ax2.set_ylabel("mean pose quality")
    ax2.set_ylim(0, 1.02)
    ax2.set_xlabel("arm base to wall distance (m)   [grey: footprint does not fit]")
    ax1.set_title(a.title or "standoff sweep: %d targets" % data.get("targets", 0))
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
