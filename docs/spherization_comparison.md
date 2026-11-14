# Collision-sphere approximations compared: FOAM vs cuRobo v2

The VAMP pipeline uses FOAM spheres ([vamp_notes.md](vamp_notes.md)); the cuRobo backend fits its own
with `curobo.sphere_fit` through `RobotBuilder` ([curobo_v2_notes.md](curobo_v2_notes.md)). This measures
how the two approximate the same link meshes, and how two earlier production models compare.

| Set | What | Source |
|---|---|---|
| A | FOAM, default options | `motion-planner-foam` image on the meshed URDF |
| B | cuRobo v2 fit, `--sphere-density 1.0`, seed 0, collision meshes | `docker/tools/generate_curobo_robot.py`, `motion-planner:gpu` |
| C | reference set (previous production model): hand-adjusted FOAM | external spherized URDF, passed by path |
| D | reference set (previous production model): cuRobo/XRDF side, newest version, sander tool config | external XRDF, passed by path |

C and D are read from paths given on the command line; nothing from them is stored in the repo.

## Method

`docker/tools/compare_spheres.py` loads each link's `<collision>` meshes (collision origin applied, link
frame) and each sphere set by link name, then per link, with seeded sampling:

- **count**: spheres on the link.
- **coverage**: share of points sampled uniformly in a box around mesh and spheres (200k) that are inside
  the mesh and also inside the sphere union (point-in-mesh by ray casting).
- **over**: volume of the union outside the mesh (cm^3 and ratio to mesh volume). Ratio 1.0 means the
  union is twice the mesh volume.
- **gap**: mesh surface (30k samples) to the union, mm; 0 where covered. Large max = geometry poking out
  of the spheres, which is the unsafe direction.
- **bulge**: sphere-surface samples on the union's outer boundary to the mesh surface, mm. Large = spheres
  sticking far out of the mesh (conservative, but eats free space).
- **query**: CPU time to test one point against the link's spheres (numpy, batched, microseconds). It
  only shows that cost scales with sphere count; it is not a measurement of VAMP's SIMD or cuRobo's CUDA
  kernels, which were not timed.

Robot geometry: the UR10e collision STLs (7 links, the same meshes as `ur_description`) and the generic
6-DOF arm with lift, converted to meshes by `primitives_to_meshes.py`. Reproduce:

```
# A: FOAM (pass --run-foam <image> to let the script do it)
docker run --rm --user $(id -u):$(id -g) -v $PWD/ur10e:/work -w /work motion-planner-foam:116928f71aaa \
  --filename robot.urdf --output spherized.urdf --database sphere_database.json
# B: cuRobo (GPU)
docker run --rm --gpus all --user $(id -u):$(id -g) -e HOME=/tmp -v $PWD/ur10e:/robots \
  -v $PWD/docker/tools:/tools:ro motion-planner:gpu /opt/venv/bin/python /tools/generate_curobo_robot.py \
  --urdf /robots/robot.urdf --tool-frame tool0 --out /robots/curobo.yml
# metrics (CPU; needs numpy scipy trimesh pyyaml rtree embreex)
python docker/tools/compare_spheres.py --urdf ur10e/robot.urdf --foam-urdf ur10e/spherized.urdf \
  --curobo-cfg ur10e/curobo.yml --ref-foam-urdf <C.urdf> --ref-curobo-xrdf <D.xrdf> --json out.json
```

## UR10e results

Columns: n spheres; coverage; over ratio; gap mean/max mm; bulge mean/max mm; query us per point.

| Link | Set | n | cover | over | gap mean/max | bulge mean/max | query us |
|---|---|---|---|---|---|---|---|
| base_link_inertia | A | 1 | 1.000 | 1.19 | 0.0 / 0.0 | 23.6 / 57.6 | 0.20 |
| | B | 63 | 0.970 | 0.18 | 0.7 / 11.2 | 2.1 / 17.0 | 8.85 |
| | C | 1 | 1.000 | 1.19 | 0.0 / 0.0 | 23.6 / 57.6 | 0.04 |
| | D | 5 | 0.628 | 0.00 | 12.6 / 43.3 | 0.0 / 1.2 | 0.26 |
| shoulder_link | A | 1 | 1.000 | 1.03 | 0.0 / 0.0 | 24.8 / 46.3 | 0.05 |
| | B | 76 | 0.975 | 0.16 | 0.7 / 12.3 | 1.1 / 16.2 | 7.97 |
| | C | 1 | 1.000 | 1.05 | 0.0 / 0.0 | 25.1 / 46.6 | 0.05 |
| | D | 4 | 0.696 | 0.00 | 11.8 / 34.4 | 0.0 / 1.4 | 0.19 |
| upper_arm_link | A | 8 | 1.000 | 1.19 | 0.0 / 0.0 | 34.3 / 59.4 | 0.66 |
| | B | 91 | 0.868 | 0.03 | 4.2 / 28.2 | 0.4 / 15.0 | 15.98 |
| | C | 8 | 1.000 | 0.68 | 0.0 / 3.1 | 21.6 / 43.7 | 0.47 |
| | D | 10 | 0.629 | 0.00 | 16.8 / 51.4 | 0.0 / 2.2 | 0.59 |
| forearm_link | A | 8 | 1.000 | 0.82 | 0.0 / 3.3 | 19.1 / 34.2 | 0.71 |
| | B | 83 | 0.866 | 0.04 | 3.2 / 17.9 | 0.3 / 15.4 | 9.77 |
| | C | 8 | 1.000 | 0.75 | 0.0 / 3.9 | 17.8 / 33.6 | 0.43 |
| | D | 11 | 0.652 | 0.00 | 11.7 / 42.0 | 0.0 / 2.0 | 1.52 |
| wrist_1_link | A | 1 | 1.000 | 1.01 | 0.0 / 0.0 | 16.2 / 33.6 | 0.05 |
| | B | 40 | 0.982 | 0.15 | 0.3 / 7.8 | 2.2 / 11.5 | 3.63 |
| | C | 1 | 1.000 | 1.01 | 0.0 / 0.0 | 16.2 / 33.6 | 0.03 |
| | D | - | no spheres on this link | | | | |
| wrist_2_link | A | 1 | 1.000 | 1.44 | 0.0 / 0.0 | 20.4 / 35.5 | 0.03 |
| | B | 12 | 0.996 | 0.47 | 0.1 / 4.4 | 5.6 / 18.5 | 1.76 |
| | C | 1 | 1.000 | 1.44 | 0.0 / 0.0 | 20.4 / 35.5 | 0.03 |
| | D | - | no spheres on this link | | | | |
| wrist_3_link | A | 1 | 1.000 | 1.07 | 0.0 / 0.0 | 11.0 / 28.2 | 0.03 |
| | B | 5 | 0.976 | 0.37 | 0.3 / 6.9 | 3.5 / 11.4 | 0.61 |
| | C | 1 | 1.000 | 1.07 | 0.0 / 0.0 | 11.0 / 28.2 | 0.03 |
| | D | - | no spheres on this link | | | | |
| **Total (7 links)** | A | 21 | 1.000 | 1.08 | | | 1.7 |
| | B | 370 | 0.896 | 0.08 | | | 48.6 |
| | C | 21 | 1.000 | 0.82 | | | 1.1 |
| | D (4 links) | 30 | 0.644 | 0.00 | | | 2.6 |

Total coverage is volume-weighted; total over is summed over volume; query is the sum over links.

Generic 6-DOF arm with lift (12 links, A and B only): A has 54 spheres, coverage 1.000, over 1.75
(the 3-sphere base is the worst, 131 mm mean bulge); B has 400 spheres, coverage 0.902, over 0.05,
gap max 4 to 68 mm per link. Same pattern as the UR10e.

### Reference sets C and D: how they line up with our UR10e description

- **C** uses the same link names and link frames (`base_link_inertia` ... `wrist_3_link`) and matches
  our meshes: the single-sphere links are practically identical to A, with 21 spheres in total. The
  hand adjustment shows on `upper_arm_link` (over 0.68 against 1.19) while keeping full coverage.
- **D** matches on `base_link_inertia`, `shoulder_link`, `upper_arm_link`, `forearm_link`: the spheres sit
  inside the meshes (bulge about 0), so the frames agree. It is an inscribed-style fit with 64%
  coverage. It has no spheres on the three wrist links; the wrist and tool are covered by a 30-sphere
  `tool_mount` that is not part of our description. Its `base_link` (60 spheres, radius up to 0.34 m),
  `liftkit_*`, `forearm_clip_*` and `upperarm_clip` spheres belong to the whole mobile robot, not the
  UR10e description, so they were skipped, not forced onto a mesh. D's numbers therefore describe only
  four links.

## Reading the numbers

- **FOAM is conservative and coarse.** Full coverage on every link, the union is about twice the true
  volume, and the boundary sticks out up to 3 to 6 cm. One sphere per small link. 21 spheres for the arm.
- **cuRobo's fit is tight and not conservative.** About 17x as many spheres (370), union only 8% larger
  than the mesh, boundary within 1 to 2 mm on average, but only 87% to 99% of each link covered, with
  uncovered surface up to 28 mm on the upper arm. That is a fit objective (MorphIt), not a guarantee. It
  can be made conservative by adding a per-link buffer (`collision_sphere_buffer`, or the XRDF
  `buffer_distance`) of about the max gap, 1 to 3 cm.
- **Cost scales with spheres.** 370 against 21 spheres is about 28x in the point-test proxy. cuRobo's GPU
  batches absorb that; VAMP's per-configuration SIMD kernel does not (its pair checks also grow
  quadratically with spheres).

## Recommendation

| Backend | Use | Why |
|---|---|---|
| VAMP | FOAM spheres (set A, or a hand-adjusted set like C) | conservative, few spheres, fast kernel |
| cuRobo | its own fit (B), plus a 1 to 3 cm buffer when safety margin matters | tight envelope, GPU handles 370 spheres, better free space in cluttered scenes |

**cuRobo spheres in VAMP: possible, not recommended as is.** Cricket reads a spherized URDF, so the
yaml converts to collision `<sphere>` elements. But 370 spheres make VAMP's kernel and Cricket's pair
generation far heavier, and without a buffer the model under-covers (gap up to 28 mm). It is the "feed cuRobo's fitted spheres into Cricket" next step listed in
[vamp_notes.md](vamp_notes.md) for the UR10e model that Cricket rejects.
Worth trying with `--sphere-density` below 1 and a buffer; not tested here.

**FOAM spheres in cuRobo: viable.** The cuRobo robot config only needs a `collision_spheres` table per
link, which converts directly from a FOAM URDF (the table format is the one `compare_spheres.py` reads).
It is conservative and 17x smaller, so queries get cheaper, but the 2x union volume shrinks the free
space: expect more failures in narrow passages and a tighter reachable set near obstacles. Self-collision
needs its own matrix for that model (adjacent-link sphere overlap is much bigger with FOAM). Not planned
with here.

What was not measured: planning success, end-to-end query time in VAMP or cuRobo, and the effect of the
sample count on tail values (maxima are sampled, so they can only be underestimates).
