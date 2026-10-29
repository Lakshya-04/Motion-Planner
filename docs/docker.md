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
| `vamp` (build stage) | upstream VAMP at a pinned commit, SIMD baseline x86-64-v3 + AVX2 | feeds `full` |
| `full` | `gpu` + VAMP | everything |

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

Verified on Jazzy in the container: all 12 packages build and install, and the 277 unit tests
in 10 packages pass, plus the description's pytest contract tests. The `gpu` / `full` targets
(CUDA PyTorch, cuRobo, VAMP) and the FOAM / Cricket tool images are written and lint-checked but
have not been built here yet.
