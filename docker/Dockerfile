# syntax=docker/dockerfile:1.7
#
# Reproducible build/dev environment for the whole workspace.
#
#   base  : ROS 2 + build tooling
#   deps  : every package dependency, resolved by rosdep from the package.xml files
#           (cached layer: only re-runs when a package.xml or apt-extra.txt changes)
#   dev   : deps + developer tools + non-root user matching the host UID
#   (every stage inherits ccache + mold + the CCACHE_*/launcher ENV from base; see docs/docker.md)
#   build : deps + the workspace built and tested with colcon (CI-style, no GPU needed)
#   gpu   : dev + CUDA-enabled PyTorch + cuRobo v2 pinned (needs the NVIDIA container runtime)
#   sim-gazebo : deps + Gazebo Harmonic (ros_gz, gz_ros2_control) for the headless simulation backend
#   nav2  : sim-gazebo + Nav2 (opt-in, docs/nav2.md)
#   vamp  : upstream VAMP (SIMD CPU planner) built from a pinned commit, installed to /opt/vamp
#   full  : gpu + vamp  (the "everything" image)
#
# Targets:  docker build -f docker/Dockerfile --target <name> -t motion-planner:<name> .

ARG ROS_DISTRO=jazzy

############################ base ############################
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    CMAKE_BUILD_PARALLEL_LEVEL=4

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git curl ca-certificates \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      python3-pip python3-venv ccache mold \
    && rm -rf /var/lib/apt/lists/*

# Compiler cache and fast linker, the default for every build in every stage below.
# /ccache is world-writable (1777) so any UID can use it; callers mount a host dir or volume there
# (docker/compose.yaml, docs/docker.md "Compiler cache (default)"). CMake >= 3.17 reads the
# CMAKE_<LANG>_COMPILER_LAUNCHER environment variables; the colcon defaults file repeats the launchers and
# adds mold, so a plain `colcon build` is cached. COMPILERCHECK=content makes identical toolchains share hits.
ENV CCACHE_DIR=/ccache \
    CCACHE_MAXSIZE=6G \
    CCACHE_COMPILERCHECK=content \
    CCACHE_BASEDIR=/ws \
    CMAKE_C_COMPILER_LAUNCHER=ccache \
    CMAKE_CXX_COMPILER_LAUNCHER=ccache \
    CMAKE_CUDA_COMPILER_LAUNCHER=ccache \
    COLCON_DEFAULTS_FILE=/etc/colcon/defaults.yaml
COPY docker/colcon-defaults.yaml /etc/colcon/defaults.yaml
RUN mkdir -p /ccache && chmod 1777 /ccache

WORKDIR /ws

############################ deps #############################
FROM base AS deps
ARG ROS_DISTRO

# 1. Dependencies declared by the packages themselves (single source of truth).
COPY src/ /tmp/src-manifest/
RUN find /tmp/src-manifest -type f ! -name package.xml -delete \
    && find /tmp/src-manifest -type d -empty -delete

# 2. Extra system packages that are not (or not reliably) expressed as rosdep keys.
COPY docker/apt-extra.txt /tmp/apt-extra.txt

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
    && (rosdep init 2>/dev/null || true) \
    && (for i in 1 2 3 4 5; do rosdep update --rosdistro ${ROS_DISTRO} && exit 0; echo "rosdep update failed (try $i), retrying"; sleep 10; done; exit 1) \
    && rosdep install --from-paths /tmp/src-manifest --ignore-src -y --rosdistro ${ROS_DISTRO} \
         --skip-keys "ament_cmake_clang_format" \
    && rm -rf /var/lib/apt/lists/* /tmp/src-manifest

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
    && grep -v '^\s*#' /tmp/apt-extra.txt | grep -v '^\s*$' | sed "s/@ROS@/${ROS_DISTRO}/g" | xargs apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

############################ dev ##############################
FROM deps AS dev
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      gdb valgrind clang-format clang-tidy sudo nano less \
      ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rqt-graph \
    && rm -rf /var/lib/apt/lists/* \
    && (getent passwd ${UID} && userdel -r $(getent passwd ${UID} | cut -d: -f1) || true) \
    && (getent group ${GID} || groupadd -g ${GID} ${USERNAME}) \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && chown -R ${UID}:${GID} /ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
USER ${USERNAME}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

############################ build ############################
FROM deps AS build
COPY docker/build.sh /usr/local/bin/mp-build
COPY src/ /ws/src/
RUN --mount=type=cache,target=/ccache,sharing=locked,mode=1777 \
    chmod +x /usr/local/bin/mp-build && mp-build test
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

############################ gpu ##############################
# cuRobo v2 + CUDA PyTorch on top of the dev image. Versions are pinned in
# docker/requirements-gpu.txt. Run with: docker compose run --rm gpu
FROM dev AS gpu
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
USER root
COPY docker/requirements-gpu.txt /tmp/requirements-gpu.txt
# PATH=/usr/lib/ccache puts ccache in front of gcc/g++ for extension builds (torch cpp_extension / ninja, pip
# sdists); CUDA_HOME-based nvcc steps are cached through CMAKE_CUDA_COMPILER_LAUNCHER where CMake drives them.
RUN --mount=type=cache,target=/root/.cache/pip \
    --mount=type=cache,target=/ccache,sharing=locked,mode=1777 \
    export PATH=/usr/lib/ccache:$PATH \
    && python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --upgrade pip \
    && /opt/venv/bin/pip install -r /tmp/requirements-gpu.txt \
    && chown -R ${UID}:${GID} /opt/venv
ENV PATH=/opt/venv/bin:$PATH \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility
USER ${USERNAME}

############################ sim ##############################
# MuJoCo 4 + MJX (JAX) on top of the gpu image. Optional: not needed for planning.
#   docker build -f docker/Dockerfile --target sim -t motion-planner:sim .
FROM gpu AS sim
ARG USERNAME=dev
USER root
COPY docker/requirements-sim.txt /tmp/requirements-sim.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    --mount=type=cache,target=/ccache,sharing=locked,mode=1777 \
    PATH=/usr/lib/ccache:$PATH /opt/venv/bin/pip install -r /tmp/requirements-sim.txt \
    && chown -R ${USERNAME}: /opt/venv
USER ${USERNAME}

############################ sim-gazebo #######################
# Thin layer: Gazebo Harmonic + the ROS bridges and gz_ros2_control (src/simulation/sim_gazebo).
# No workspace is built into it: mount src/ and colcon build inside (docs/sim_gazebo.md).
# Runs headless (gz sim -s), no GPU or display needed.
FROM deps AS sim-gazebo
ARG ROS_DISTRO
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz-sim ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

############################ nav2 #############################
# Opt-in: Nav2 on top of sim-gazebo, for the omni base (config/nav2/, docs/nav2.md). Not part of any other stage.
FROM sim-gazebo AS nav2
ARG ROS_DISTRO
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

############################ vamp #############################
# Upstream VAMP, unmodified, at a pinned commit. Built for a portable SIMD baseline
# (x86-64-v3 + AVX2) instead of -march=native, so the image runs on any AVX2 machine and
# not only the one that built it. Override with --build-arg VAMP_ARCH="-march=native -mavx2"
# for a host-specific build.
FROM deps AS vamp-build
ARG VAMP_REF=27cb9b66ebd8d101ce88d330f3b028398afc1cfa
ARG VAMP_ARCH="-march=x86-64-v3 -mavx2"
RUN --mount=type=cache,target=/ccache,sharing=locked,mode=1777 \
    git clone https://github.com/KavrakiLab/vamp.git /tmp/vamp \
    && git -C /tmp/vamp checkout ${VAMP_REF} \
    && cmake -S /tmp/vamp -B /tmp/vamp/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
         -DVAMP_ARCH="${VAMP_ARCH}" -DVAMP_BUILD_PYTHON_BINDINGS=OFF \
         -DVAMP_BUILD_CPP_DEMO=OFF -DVAMP_BUILD_TESTS=OFF \
         -DCMAKE_INSTALL_PREFIX=/opt/vamp \
         -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold \
    && cmake --build /tmp/vamp/build \
    && cmake --install /tmp/vamp/build \
    # Upstream's install step ships only its own headers, but they include the headers of the
    # dependencies CPM fetched (pdqsort, nigh, SIMDxorshift, cddlib). Install those next to them so
    # /opt/vamp is self-contained; nothing upstream is modified.
    && D=/tmp/vamp/build/_deps \
    && cp ${D}/pdqsort-src/pdqsort.h ${D}/simdxorshift-src/include/simdxorshift128plus.h /opt/vamp/include/ \
    && cp -r ${D}/nigh-src/src/nigh /opt/vamp/include/ \
    && cp ${D}/cddlib-src/lib-src/*.h /opt/vamp/include/ \
    && rm -rf /tmp/vamp

############################ full #############################
FROM gpu AS full
ARG USERNAME=dev
USER root
COPY --from=vamp-build /opt/vamp /opt/vamp
ENV CMAKE_PREFIX_PATH=/opt/vamp
USER ${USERNAME}
