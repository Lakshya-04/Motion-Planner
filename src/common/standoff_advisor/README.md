---
type: Reference
title: standoff_advisor
description: Package README for the standoff advisor: reachable fraction and pose quality versus base-to-wall distance on an RM4D map, and a robust recommended standoff.
tags: [reachability, standoff, package-readme]
status: experimental
---

# standoff_advisor

How far from the wall should the base dock? Sweeps the arm base's distance to a wall plane over an RM4D map
(`reachability`) and reports, per distance, the reachable fraction of the target poses, the mean and minimum pose quality,
the lift range, and whether the footprint fits; the recommendation is the centre of the widest good plateau, shrunk by
a navigation tolerance. Full description, results and limits: [docs/standoff.md](../../../docs/standoff.md).

- `standoff_advisor_core` (ROS-free, `standoff_advisor/standoff_sweep.hpp`): `sweepStandoff`, `evaluateStandoff`,
  `patchTargets`, `patchFromPlane`, `sweepProblem`.
- `standoff_sweep` (CLI): `--map` or `--chain`, `--wall-scene` or `--wall-point/--wall-normal/--wall-patch`, `--profile`; writes JSON and CSV, prints a table and an ASCII bar chart.
- `scripts/standoff_plot.py`: PNG of fraction and quality versus distance (matplotlib, ASCII fallback).
- `scripts/profile_to_map.sh <arm> <out_dir>`: profile to chain YAML to `.rm4d` (needs the ROS workspace).

```bash
colcon build --packages-up-to standoff_advisor && colcon test --packages-select standoff_advisor
ros2 run standoff_advisor standoff_sweep --map ur10e.rm4d --wall-scene wall_scene.json \
  --profile ur10e_telescopic.yaml --wall-z 1.0 1.3 --out-json sweep.json
```

Limits: map resolution (0.05 m), reach only (no IK, collision or obstacles), vertical straight walls, no dynamics; quality is the map's
sample-count score. Tests: 15 gtests on a hand-filled map (plateau choice, tolerance shrink, footprint, empty and refused cases).
