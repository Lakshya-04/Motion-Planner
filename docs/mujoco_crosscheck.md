# MuJoCo / MJX cross-check of the robot models

An independent engine checking our robot description and our two collision-sphere models: MuJoCo
compiles the same URDF the planners use, and its forward kinematics and mesh contacts are compared
with the other implementations on seeded random configurations. It is a sanity check, not a second
planner backend.

Code: `docker/tools/mujoco_model.py` (loader and pure helpers), `docker/tools/mujoco_crosscheck.py`
(the comparison), `docker/tools/curobo_crosscheck_dump.py` (cuRobo side, needs a GPU for a few
seconds), tests in `docker/tools/tests/test_mujoco_crosscheck.py` (no GPU, no ROS).

## Method

**Model.** `mobile_manipulator.urdf.xacro arm:=ur10e` (and `arm:=generic_6dof`) with
`lift_fixed_height:=0.5`, so the robot is base + pinned lift + the 6 arm joints. The xacro is expanded
inside `motion-planner:build`, then the same prep as the VAMP/cuRobo pipelines: `resolve_package_uris.py`
(UR collision meshes copied next to the URDF) and `primitives_to_meshes.py`. `mujoco_model.patch_urdf`
injects a `<mujoco><compiler fusestatic="false" discardvisual="true" .../></mujoco>` block into a copy so
`tcp_link` stays a body and only collision geometry is imported. Joint limits come through unchanged
(max |MuJoCo - URDF| = 0). The lift mimic stages are fixed joints in this configuration, so no mimic
handling is needed.

**Samples.** N = 2000 configurations per robot, `numpy.random.default_rng(0)`, uniform inside the URDF
joint limits (UR10e: +-2 pi on most joints, so the set is dominated by unusual postures).

**(a) FK of `tcp_link`** (base_footprint frame): MuJoCo `mj_forward` vs
1. a from-the-spec URDF FK in numpy (`UrdfKinematics`, written independently of MuJoCo),
2. the analytic UR10e standard-DH chain (UR10e only; `arm_base_link` is the DH base turned by pi about z,
   `tool0` coincides with the DH flange, the TCP offset is the URDF's fixed transform),
3. cuRobo `Kinematics.compute_kinematics` (float32, `curobo_crosscheck_dump.py`).
MoveIt/KDL FK was not run: nothing in the images exposes it cheaply outside a full MoveIt launch.

**(b) Self-collision.** Ground truth is "any penetrating MuJoCo contact (dist < 0) between a pair of links
that is not on the ignore list". MuJoCo collides meshes as convex hulls. The compared models are the
FOAM sphere set (default options, `motion-planner-foam`), the cuRobo v2 sphere set (RobotBuilder,
density 1.0, collision meshes), the same cuRobo spheres through our numpy check, and cuRobo's own
native self-collision cost (its `self_collision_ignore` matrix, GPU). Sphere counts: UR10e FOAM 55, cuRobo
703 (13 links); generic arm FOAM 54, cuRobo 402 (12 links).

The ignore list is shared by MuJoCo, FOAM and cuRobo-numpy (`allowed_pairs`): links joined by fixed joints
form one rigid group, and two groups are ignored when at most `hops` revolute joints apart. `hops = 1`
is "adjacent only" (strict); `hops = 2` also ignores neighbour-of-neighbour pairs, which spherization
bulges into (`vamp_notes.md`). cuRobo's native check uses its own generated matrix, so it is a separate
list in both tables.

**(c) MJX.** `jax.vmap(mjx.forward)` and one `mjx.step` over all 2000 configurations under
`JAX_PLATFORMS=cpu`, FK compared with CPU MuJoCo.

## Results (N = 2000, seed 0, MuJoCo 4.0.1)

### FK, tool frame

| Robot | Comparison | pos mean / max (mm) | rot max (deg) |
|---|---|---|---|
| UR10e | MuJoCo vs URDF FK (numpy) | 3e-13 / 1e-12 | 4e-6 |
| UR10e | MuJoCo vs analytic DH | 1.5e-7 / 3.3e-7 | 3e-6 |
| UR10e | MuJoCo vs cuRobo (float32) | 1.8e-4 / 5.8e-4 | 4e-5 |
| generic 6-DOF | MuJoCo vs URDF FK (numpy) | 2e-13 / 1e-12 | 3e-6 |
| generic 6-DOF | MuJoCo vs cuRobo (float32) | 1.5e-4 / 5.3e-4 | 2e-5 |
| both | MJX (float32, CPU) vs CPU MuJoCo | max 6.9e-4 (UR10e), 6.3e-4 (generic) | 0.08 / 0.07 |

The URDF, the DH chain, MuJoCo and cuRobo agree to float precision: the xacro expansion, the URDF
joint axes and origins and the cuRobo kinematics are consistent. The MJX rotation figure is the
float32 `acos` of a trace near 3, not a pose error (position is 0.0007 mm).

### Self-collision verdicts vs MuJoCo (rates over all 2000 samples)

False-free = MuJoCo says collision, model says free (unsafe). False-blocked = MuJoCo free, model says
collision (conservative, costs free space).

| Robot | Ignore list | MuJoCo collision rate | Model | Agree | False-free | False-blocked |
|---|---|---|---|---|---|---|
| UR10e | hops 1 | 0.470 | FOAM spheres | 0.470 | 0.000 | 0.529 |
| | | | cuRobo spheres (numpy) | 0.972 | 0.021 | 0.007 |
| | | | cuRobo native | 0.972 | 0.021 | 0.007 |
| UR10e | hops 2 | 0.332 | FOAM spheres | 0.859 | 0.000 | 0.141 |
| | | | cuRobo spheres (numpy) | 0.988 | 0.003 | 0.009 |
| | | | cuRobo native | 0.871 | 0.002 | 0.128 |
| generic | hops 1 | 0.417 | FOAM spheres | 0.680 | 0.000 | 0.320 |
| | | | cuRobo spheres (numpy) | 0.956 | 0.007 | 0.037 |
| | | | cuRobo native | 0.956 | 0.007 | 0.037 |
| generic | hops 2 | 0.255 | FOAM spheres | 0.879 | 0.000 | 0.121 |
| | | | cuRobo spheres (numpy) | 0.954 | 0.001 | 0.045 |
| | | | cuRobo native | 0.807 | 0.000 | 0.193 |

Reading it:

- The two sphere models fail in opposite directions. FOAM's few large spheres (about 1 per link) never
  miss a hull collision (false-free 0 in all rows) but over-block a lot; cuRobo's dense fit tracks
  the meshes (99% agreement on the UR10e at hops 2) and its few misses are false-free, up to 2.1% of
  all samples with the strict list.
- With `hops = 1`, every UR10e FOAM configuration is reported colliding (neighbour spheres overlap),
  which is the same symptom as "0 of 2000 valid" in `vamp_notes.md`: the sphere model needs a looser
  ignore list (or a list sampled from the spheres themselves, as Cricket does) before it is usable.
- cuRobo's native rows equal the numpy rows at `hops = 1` sample for sample, so its generated
  ignore matrix behaves like "adjacent links only" on these robots, and the numpy re-implementation
  of the sphere check reproduces the GPU verdicts. At `hops = 2` the native row differs only because
  its list is stricter, not because of a different check.
- Most MuJoCo contacts at `hops = 2` are forearm vs lift stages, forearm vs wrist 3/tool (UR10e) and
  shoulder vs wrist links (generic): an arm folded into itself or into the lift column.

### MJX (vectorised) run

2000 configurations, jax 0.12 on CPU, contacts disabled: first call (compile) 12 s UR10e / 11 s generic,
then 0.011 s / 0.026 s per batch for `mjx.forward` of the tool frame; `mjx.step` compiles in about 21-24 s
and returns finite states. For scale, CPU MuJoCo took about 3 s for the same 2000 `mj_forward` calls
including contacts, so these timings are not a like-for-like speed claim.

MJX contact verdicts were not obtained: see the last section.

## Reproduce

```
# 1. expand + resolve + mesh (as for VAMP/cuRobo); arm:=ur10e or generic_6dof
xacro .../mobile_manipulator.urdf.xacro arm:=ur10e lift_fixed_height:=0.5 -o ur10e.urdf   # in motion-planner:build
python3 docker/tools/resolve_package_uris.py ur10e.urdf ur10e                                # drops visuals, copies meshes
python3 docker/tools/primitives_to_meshes.py ur10e/robot.urdf meshed/ur10e                   # + copy ur_description/ next to it
# 2. spheres: FOAM image (CPU) and generate_curobo_robot.py (GPU, seconds; --batch-size 500)
# 3. cuRobo FK + native self-collision on the same configurations (GPU, a few seconds)
python3 -c "import numpy as np, mujoco_model as m; k=m.UrdfKinematics('robot.urdf'); np.save('q.npy', m.sample_configs(k, 2000, 0))"
docker run --rm --gpus all ... motion-planner:sim /opt/venv/bin/python /tools/curobo_crosscheck_dump.py \
  --cfg /robots/curobo.yml --q /robots/q.npy --out /robots/dump.npz
# 4. the cross-check (CPU only)
JAX_PLATFORMS=cpu python docker/tools/mujoco_crosscheck.py --robot ur10e --urdf robot.urdf \
  --foam-urdf spherized.urdf --curobo-cfg curobo.yml --curobo-dump dump.npz --n 2000 --mjx-batch 2000
python -m pytest docker/tools/tests/test_mujoco_crosscheck.py      # works in motion-planner:sim, mujoco tests skip otherwise
```

Only `--urdf` is required; sphere and cuRobo inputs are optional. Meshes for the run live in
a scratch directory and are not committed.

## Honest limits

- **MuJoCo is not an exact ground truth.** Mesh geoms collide as convex hulls, so concave links (the UR
  shoulder and wrist housings) are over-approximated and a "MuJoCo collision" can be a hull-only
  contact. The generic arm's boxes and cylinders are exact (as STL hulls); the UR10e is where this
  matters. The false-free rates above are therefore against hulls, not against the exact meshes.
- Contacts are `dist < 0` only (no safety margin); the ignore lists are ours, not the SRDF's collision
  matrix, and the rates depend strongly on them (hops 1 vs hops 2 moves FOAM from 47% to 86% agreement).
- Samples are uniform in joint space with the wide UR limits, so the configurations are much more
  self-colliding (33-47%) than planned motions, and the false-free percentages are not the rates a
  planner would see near its working poses. There is no obstacle or world check here.
- Sphere models are one seeded fit each (FOAM defaults, cuRobo density 1.0); other settings change the
  rows. cuRobo quaternions were read as wxyz (the agreement of 4e-5 deg confirms it).
- FK is compared to float precision, so it detects frame, axis, limit or mesh-origin mistakes but not
  a wrong physical calibration (the DH values are the nominal UR10e ones).
- KDL/MoveIt FK, the VAMP/Cricket binary itself (the FOAM rows use the same sphere set VAMP is
  generated from, checked with our numpy code, not VAMP's SIMD checker) and MuJoCo with decomposed (non-hull) collision meshes were not run. The GPU path of MJX
  was not exercised: JAX stayed on CPU because the card is shared.

## MJX with contacts

Not obtained. With the full UR10e collision meshes, `mjx.forward` with contacts needs about 114 GB of
buffers for a batch of 64 (the run aborted with an allocation error). On a hull-thinned copy
(`--mjx-collision`, `mujoco_model.simplify_meshes`, 32 vertices per link) the jit compile on the CPU
backend was still running after 9.5 minutes and was stopped; a 16-vertex attempt was interrupted before
finishing. The code path is kept behind the flag but is unverified, so MJX is only demonstrated for
FK and `mjx.step`. Contact verdicts come from CPU MuJoCo.
