#!/usr/bin/env python3
"""Force and coverage plot of a recorded run (the score_run CSV from record_run.py).

    plot_run.py --samples run.csv --out run.png [--score score.json] [--target-force 10] [--band 1.5]

Left: contact force over time with the target and its band. Right: the tool path on the wall, coloured by the
force, over the wall rectangle (from run.csv.wall.json). With --score (the JSON score_run prints) the title
carries the scored coverage, peak and RMS force, otherwise they are computed from the samples (peak, RMS error
over the samples that touch the wall).
Needs numpy and matplotlib only.
"""
import argparse
import json

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', required=True)
    ap.add_argument('--wall', help='default: SAMPLES.wall.json')
    ap.add_argument('--score', help='JSON printed by score_run')
    ap.add_argument('--out', required=True)
    ap.add_argument('--target-force', type=float, default=10.0)
    ap.add_argument('--band', type=float, default=1.5)
    ap.add_argument('--clip', type=float, default=40.0, help='upper limit of the force axis, N')
    args = ap.parse_args()
    d = np.genfromtxt(args.samples, delimiter=',', names=True)
    wall = json.load(open(args.wall or args.samples + '.wall.json'))
    t, y, z, f = d['t'], d['y'], d['z'], d['force']
    touching = f > 1.0

    title = ''
    if args.score:
        sc = json.load(open(args.score))
        title = (f"coverage {sc['coverage']['covered_fraction'] * 100:.1f} %   peak {sc['force']['peak_n']:.1f} N   "
                 f"RMS error {sc['force']['rms_error_n']:.1f} N   ({sc['time']['duration_s']:.0f} s)")
    else:
        title = f'peak {f.max():.1f} N   RMS error {np.sqrt(np.mean((f[touching] - args.target_force) ** 2)):.1f} N (touching)'

    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw={'width_ratios': [2.2, 1]})
    a.axhspan(args.target_force - args.band, args.target_force + args.band, color='#2a9d8f', alpha=0.25,
              label=f'target {args.target_force:g} N +- {args.band:g}')
    a.plot(t, np.minimum(f, args.clip), lw=0.7, color='#1d4e89')
    over = f > args.clip
    if over.any():
        a.plot(t[over], np.full(over.sum(), args.clip), 'v', color='#c1121f', ms=3,
               label=f'above {args.clip:g} N (peak {f.max():.0f} N)')
    a.set_xlabel('time (s)')
    a.set_ylabel('contact force |Fz| (N)')
    a.set_ylim(0, args.clip * 1.05)
    a.legend(loc='upper left', fontsize=8)
    a.grid(alpha=0.3)

    u0, u1 = wall['u_range']
    v0, v1 = wall['v_range']
    b.add_patch(plt.Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, ec='k', lw=1))
    sc_ = b.scatter(y[touching], z[touching], c=np.minimum(f[touching], 2 * args.target_force), s=3, cmap='viridis',
                    vmin=0, vmax=2 * args.target_force)
    b.set_aspect('equal')
    b.set_xlabel('wall y (m)')
    b.set_ylabel('wall z (m)')
    b.set_title('tool path on the wall', fontsize=10)
    fig.colorbar(sc_, ax=b, label='force (N)', shrink=0.8)
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
