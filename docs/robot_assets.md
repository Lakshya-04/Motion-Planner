# Robot assets from a profile

One command turns a profile YAML (`mobile_manipulator_description/config/profiles/*.yaml`) into every
per-robot artefact the planners need, and records what it made:

```bash
docker/tools/build_robot_tools.sh                      # once: FOAM and Cricket images
docker/tools/generate_robot_assets.sh \
    src/description/mobile_manipulator_description/config/profiles/ur10e_telescopic.yaml \
    --out build/robot_assets/ur10e_telescopic
docker/tools/generate_robot_assets.sh <profile.yaml> --only srdf,reach     # or --skip curobo,vamp
```

`generate_robot_assets.sh` is a thin wrapper over `docker/tools/robot_assets.py`, which is glue only: it
calls the existing tools (`generate_vamp_robot.sh`, `rigid_pairs_srdf.py`, `primitives_to_meshes.py`,
`resolve_package_uris.py`, `generate_curobo_robot.py`, `sphere_self_collision.py`, `urdf_to_chain.py`,
`rm4d_tool`) and launches each stage in the image it needs. The output directory is mounted at the same
path inside the containers, because cuRobo stores absolute mesh paths.

## Stages

Order is fixed; stages read what earlier ones left in the output directory, so `--only` / `--skip` (comma
lists) can re-run any subset. `--list` prints the names.

| Stage | Image | Makes |
|---|---|---|
| `urdf` | `motion-planner:build` | `robot.urdf` (full model); `arm_only/robot.urdf` (lift pinned at the profile's home height, or `lift:=none`: 6 DOF) and `arm_only/resolved/` (`package://` meshes copied next to it, visuals dropped) |
| `srdf` | `motion-planner:build` | `robot.srdf` (groups + structural allowed-collision matrix, per arm family and lift), `lift_limits.yaml` (lift velocity / acceleration from the profile) |
| `spheres` | `motion-planner-foam` | `vamp/spherized.urdf`, `vamp/sphere_database.json` (FOAM on the arm-only model) |
| `vamp` | `motion-planner-cricket` | `vamp/rigid.srdf` (`rigid_pairs_srdf.py --hops 2`, `--hops N` to change), `vamp/<Name>.hh` |
| `curobo` | `motion-planner:gpu` (GPU) | `curobo/<Name>.yml` (fitted spheres + collision matrix) and `curobo/meshed/` |
| `reach` | `motion-planner:build` | `reach/chain.yaml` (`arm_base_link -> tcp_link`), `reach/build_map.sh`, `reach/<Name>.rm4d` (6-DOF map) |
| `check` | `motion-planner:build` | `check.json`: self-collision sanity on the spheres (below) |
| `manifest` | host | `manifest.json` |

`<Name>` is the profile file name in CamelCase (`ur10e_nolift.yaml` -> `Ur10eNolift`); it is the C++
struct name in the header. `docker/tools/generate_vamp_robot.sh` gained two env switches, `FOAM_ONLY=1`
and `SKIP_FOAM=1`, so FOAM and Cricket can be skipped independently.

Environment: `CRICKET_SRC` (a pinned Cricket checkout; only `resources/templates` is used, default
`~/.cache/motion-planner-tools/cricket`), `BUILD_IMG`, `GPU_IMG`, `FOAM_IMG`, `CRICKET_IMG`, `FOAM_ARGS`.
`--curobo-batch-size` (default 1000; cuRobo's own 10000 needs more than 7 GB of GPU memory).

## Which model is spherized

The profile's lift is pinned for the sphere models: VAMP and cuRobo plan the 6 arm DOF and the lift is
handled by bands (see [reachability.md](reachability.md)). `lift:=none` is used as is; telescopic and fixed
profiles render `lift:=fixed` at the profile's `home.lift_joint`; scissor renders as the equivalent pinned
column (same output height). The full `robot.urdf` keeps the real lift for the moving-lift consumers.
The profile is the only input: lift values come from its `lift:` section through
`scripts/profile_xacro_args.py`, which wins over lift entries in `robot.xacro_args`.

## Self-collision sanity

`check` loads `vamp/spherized.urdf` with `vamp/rigid.srdf` through `sphere_self_collision.py`'s model and
reports: the profile's `home` pose must be free, a folded pose (elbow back on itself,
`[0, 0, 3.1, 0, 0, 0]`) must collide, and the share of 500 random poses that are free. `ok` is true only
if both hold. A home pose that collides means the SRDF misses pairs (raise `--hops`); a folded pose that
does not means the spheres are too small or the pose does not fold that arm.

## Manifest

`manifest.json`: UTC time, profile path / hash / expanded xacro args, repo commit, FOAM and Cricket pins,
Cricket template hash, image ids, FOAM and cuRobo sphere counts per link, the header names, the
self-collision result, per-stage status and time, and sha256 + size for every artefact (meshes and the
sphere database excluded). Headers are large: keep the output under `build/`, never in git.

## Results

Run here on 2026-11, FOAM `116928f71aaa`, Cricket `a2e664f97b6a`, 12-core host with a shared 4 GB GPU:

| Profile | FOAM spheres | cuRobo spheres | VAMP header | Home / folded | Random poses free | Reach map |
|---|---|---|---|---|---|---|
| `ur10e_telescopic` (lift pinned at 0.4 m) | 55 on 13 links | 729 on 13 links | `Ur10eTelescopic.hh`, 997 KB, sha256 `0edac334dfc4` | free / collides | 250 of 500 | 5.8 MB, `2ebd873d7736` |
| `ur10e_nolift` | 33 on 11 links | not run | `Ur10eNolift.hh`, 482 KB, sha256 `9938e806184b` | free / collides | 198 of 500 | 5.8 MB, `2ebd873d7736` |

Wall time, telescopic: urdf 7 s, srdf 7 s, spheres 281 s (FOAM), vamp 3 s, curobo 765 s, reach 9 s, check 9 s.
The two reach maps hash the same because the arm and its limits are identical (the arm base is the root).

## Limits

- Cricket's templates must be available locally (`CRICKET_SRC`); the Cricket image only has the binary.
- The cuRobo stage needs a GPU and the same absolute output path inside the container.
- The profile's `home` must list every arm joint; the folded pose is a 6-joint UR-style fold.
- Meshes outside `package://` URIs are not copied; give the URDF relative or package paths.
