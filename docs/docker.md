# Docker environment

One reproducible environment for building, testing and running the stack, with every
dependency listed in the repo. Files live in [`docker/`](../docker/).

## Targets

| Target | Contents | Use |
|---|---|---|
| `base` | ROS 2 (default Jazzy) + build tools | internal |
| `deps` | all package dependencies from `rosdep` (reads every `package.xml`) + `docker/apt-extra.txt` | cached layer |
| `dev` | deps + gdb, clang-tidy, rviz2, a non-root user matching your host UID | interactive development |
| `build` | deps + workspace built and tested with colcon | CI-style check |
| `gpu` | dev + CUDA PyTorch + cuRobo v2 (pinned) | GPU planning, needs NVIDIA runtime |
| `vamp-build` (build stage) | upstream VAMP at a pinned commit, SIMD baseline x86-64-v3 + AVX2 | feeds `full` |
| `full` | `gpu` + VAMP | everything |
| `sim` | `gpu` + MuJoCo 4 (`mujoco-core`) + MJX with JAX (`docker/requirements-sim.txt`) | MuJoCo Python tools, cross-check, MJX; optional |
| `sim-gazebo` | `deps` + Gazebo Harmonic through `ros-<distro>-ros-gz-sim`, `ros-gz-bridge`, `gz-ros2-control` | headless Gazebo backend (`sim_gazebo`); optional, no GPU |

`sim` is built on its own (`docker build -f docker/Dockerfile --target sim -t motion-planner:sim .`) and
is not part of `full`. MuJoCo 4 splits the Python package: `mujoco-core` is the headless library and
bindings, which is all a container needs; `mujoco` and `mujoco-studio` add the viewer and renderer.
MJX needs Python 3.11 or newer. 4.0 removed the `composite` element, the `cable` plugin and `flexvert`
equality constraints, so older models that use them need updating.

## Simulation targets

Neither is part of `full`, and neither has a compose service: build and run them with plain `docker`.

**`sim`** (MuJoCo 4 + MJX + JAX). Built on `gpu`, so it also carries torch and cuRobo and needs the NVIDIA
runtime to use them. `docker/requirements-sim.txt` pins `mujoco-core==4.0.1`, `mujoco-mjx==4.0.1` and
`jax[cuda12]==0.12.0.1`; the file records that torch, cuRobo (planner tests) and an MJX step were run
together on it. It serves the Python side of MuJoCo: the MJCF generator tests, the MuJoCo / MJX cross-check
([mujoco_crosscheck.md](mujoco_crosscheck.md)) and sphere tools. It does **not** contain the MuJoCo C SDK,
which the `sim_mujoco` ROS plugin needs: that is mounted from a tarball and pointed at with `MUJOCO_ROOT`
(see [sim_mujoco.md](sim_mujoco.md), Build). Without it the plugin is skipped and only the Python tools build.

```bash
docker build -f docker/Dockerfile --target sim -t motion-planner:sim .
docker run --rm --gpus all -v $PWD:/ws -w /ws motion-planner:sim \
  /opt/venv/bin/python -c "import mujoco, mujoco.mjx, jax; print(mujoco.__version__, jax.devices())"
```

**`sim-gazebo`** (Gazebo Harmonic). A thin layer on `deps`: it adds the Gazebo ROS packages and nothing
else, so it needs no GPU or display (`gz sim -s`). The workspace is not built into the image; build
`mobile_manipulator_description` and `sim_gazebo` (and `compliant_trajectory_controller` plus
`ft_bridge_hardware` for the JTAC variants) inside it, then launch. Give each simulation its own
`ROS_DOMAIN_ID`: containers on one Docker bridge discover each other. Details and the test scripts:
[sim_gazebo.md](sim_gazebo.md).

```bash
docker build -f docker/Dockerfile --target sim-gazebo -t motion-planner:sim-gazebo .
docker run --rm -e ROS_DOMAIN_ID=51 -v $PWD/src:/ws/src:ro -v $PWD/docker:/ws/docker:ro \
  -v mp_build:/ws/build -v mp_install:/ws/install -v mp_log:/ws/log -w /ws \
  motion-planner:sim-gazebo bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  colcon build --packages-up-to sim_gazebo && source install/setup.bash &&
  ros2 launch sim_gazebo sim.launch.py'
```

## Usage

```bash
docker compose -f docker/compose.yaml build dev
docker compose -f docker/compose.yaml run --rm dev     # shell; repo mounted at /ws
docker compose -f docker/compose.yaml run --rm test    # colcon build + test, no GPU
docker compose -f docker/compose.yaml run --rm gpu     # cuRobo + CUDA
docker compose -f docker/compose.yaml run --rm full    # cuRobo + CUDA + VAMP
```

Inside a container: `colcon build --symlink-install && colcon test`. Build output lives in
named volumes, so it does not pollute the repo and survives container restarts.

Set a different distro with `ROS_DISTRO=humble` (the packages build as C++17 on both).

## Where the dependencies are listed

| Kind | File |
|---|---|
| ROS and system libraries required by packages | each `package.xml` (resolved by `rosdep`) |
| Extra system packages (MoveIt, PCL, ros2_control, Pinocchio, BehaviorTree.CPP, ...) | `docker/apt-extra.txt` |
| Python / GPU stack (torch, cuRobo, trimesh, ...) with pins | `docker/requirements-gpu.txt` |
| Pinned third-party commits | `Dockerfile` args (`VAMP_REF`), `requirements-gpu.txt` (cuRobo), `docker/tools/build_robot_tools.sh` (FOAM, Cricket) |

## Run all checks

CI ([`ci.yml`](../.github/workflows/ci.yml)) runs the build, the colcon tests and the readiness scan on free runners; simulators and GPU stay local ([CONTRIBUTING.md](../CONTRIBUTING.md)). One script runs lines 1 to 3 below:

```bash
docker/run_all_checks.sh                 # colcon build + test, pytest, public-readiness scan
docker/run_all_checks.sh --skip-sims     # without sim_gazebo, sim_mujoco, sim_isaac and their Python tests
docker/run_all_checks.sh --no-build --workers 2
```

It builds in the `motion-planner:build` image (override with `--image`), keeps build products in the Docker
volumes `mp_build`, `mp_install` and `mp_log`, prints a summary and exits non-zero on any failure. The lines
below are what it does, one by one, plus the simulator scripts it does not run. Each line is independent;
the sim lines need their image and skip what they lack. Each line is independent; the sim lines need their image and skip what they lack.

```bash
# 1. Everything colcon builds and tests (C++ and Python packages, no GPU)
docker compose -f docker/compose.yaml run --rm test           # = docker/build.sh test

# 2. Python tool tests (the MuJoCo cross-check tests skip without MuJoCo)
docker compose -f docker/compose.yaml run --rm dev bash -lc 'python3 -m pytest docker/tools/tests'

# 3. Public-readiness scan (links to documents, absolute home paths)
python3 docker/tools/check_public_readiness.py

# 4. Simulator scripts, only when you touched a simulator or the force path.
#    MuJoCo (needs the `sim` image and the MuJoCo SDK, see sim_mujoco.md):
#      src/simulation/sim_mujoco/scripts/run_smoke.sh      run_contact.sh
#      src/simulation/sim_mujoco/scripts/run_wall_pass.sh  run_manager_pass.sh
#    Gazebo (needs the `sim-gazebo` image, see the section above and sim_gazebo.md):
#      ros2 run sim_gazebo smoke_test.py    ros2 run sim_gazebo contact_control_test.py
#      src/simulation/sim_gazebo/scripts/run_manager_pass.sh
```

Give each simulator run its own `ROS_DOMAIN_ID`. The simulator scripts are not part of `colcon test`.
A pass of lines 1 to 3 says nothing about GPU planning, the simulators or hardware.

## VAMP and SIMD

VAMP defaults to `-march=native`, which bakes the build machine's CPU into the binary. The
image builds with `-march=x86-64-v3 -mavx2` so it runs on any AVX2 machine. For a
host-specific build: `--build-arg VAMP_ARCH="-march=native -mavx2"`.

## Robot generation tools

FOAM (URDF -> spheres) and Cricket (spheres -> VAMP header) ship their own heavy toolchains,
so they are separate images: `docker/tools/build_robot_tools.sh` builds both from upstream at
pinned commits.

## GPU

The `gpu` and `full` services request all GPUs through the NVIDIA container runtime
(`nvidia-container-toolkit` on the host). Check with
`docker compose -f docker/compose.yaml run --rm gpu nvidia-smi`.
Isaac Sim / Isaac Lab are large and ship as NVIDIA images; add them as a separate compose
service when needed instead of installing them here.

## Build and test in a container

`docker/build.sh [test]` is the one build entry point (the `build` target and the compose `test`
service both use it). For a fast edit-build loop, mount the source into the `deps` image and keep
the build output in volumes:

```bash
docker run --rm -v $PWD/src:/ws/src:ro -v $PWD/docker:/ws/docker:ro \
  -v mp_build:/ws/build -v mp_install:/ws/install -v mp_log:/ws/log \
  -w /ws motion-planner:deps docker/build.sh test
```

## Findings from the first Jazzy build

- **Octomap version clash (Ubuntu 24.04 / Jazzy apt snapshot).** MoveIt's binaries are built
  against Ubuntu's octomap 1.9 while `ros-jazzy-octomap` 1.10 is also installed. Code linked
  against 1.10 plus MoveIt's 1.9 loads two versions into one process; every test passes and then
  the process aborts at exit with `malloc_consolidate(): invalid chunk size`. `docker/build.sh`
  passes `-Doctomap_DIR=/usr/share/octomap` so everything links the same version. If you build
  outside the container on such a system, do the same.
- **`ur_description` macro differs by distro.** Humble's `ur_robot` takes `ur_type`; Jazzy's does
  not. The description's UR adapter switches on `ROS_DISTRO`.
- **Style linters are off in `colcon test`.** uncrustify, flake8 and pep257 flagged formatting
  only. `.clang-format` is the formatting standard.

## Status

The first Jazzy build (12 packages, 277 tests) is the origin of the findings above. The latest full run
recorded in [ROADMAP.md](ROADMAP.md) section 15 is 736 colcon tests in 26 packages with no failures.
Whether the `gpu`, `full` and `sim` targets and the FOAM / Cricket tool images were built after the
first build is recorded in the notes of the documents that use them ([curobo_v2_notes.md](curobo_v2_notes.md),
[vamp_notes.md](vamp_notes.md), [sim_mujoco.md](sim_mujoco.md)); this page makes no claim for them.

## Publishing prebuilt images (manual)

`docker/tools/publish_images.sh` tags and pushes `deps`, `build` and `sim` (and with `--gpu` the `full` image) to
`ghcr.io/<owner>/motion-planner:<target>`. It is a dry run unless `--push` is given. Publishing is a
deliberate manual step (CI does not push images), done after a full end-to-end workflow test passes. Notes:

- `sim` is built on `gpu`, so it contains the 12 GB torch and cuRobo layers; layers shared with `full` upload once.
- Push images built from the Dockerfile (`docker build --target <name>`), not a hand-committed container: the
  `build` image used day to day is a compile-only commit made while the Dockerfile `build` stage was failing two tests.
- Login needs the `write:packages` scope (`gh auth refresh -s write:packages`), and the package has to be made public
  once in the GitHub UI for anonymous pulls.
