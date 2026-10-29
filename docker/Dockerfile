# syntax=docker/dockerfile:1.7
#
# Reproducible build/dev environment for the whole workspace.
#
#   base  : ROS 2 + build tooling
#   deps  : every package dependency, resolved by rosdep from the package.xml files
#           (cached layer: only re-runs when a package.xml or apt-extra.txt changes)
#   dev   : deps + developer tools + non-root user matching the host UID
#   build : deps + the workspace built and tested with colcon (CI-style, no GPU needed)
#   gpu   : dev + CUDA-enabled PyTorch + cuRobo v2 pinned (needs the NVIDIA container runtime)
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
      python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

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
    && (rosdep init 2>/dev/null || true) && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src-manifest --ignore-src -y --rosdistro ${ROS_DISTRO} \
         --skip-keys "ament_cmake_clang_format" \
    && grep -v '^\s*#' /tmp/apt-extra.txt | grep -v '^\s*$' | sed "s/@ROS@/${ROS_DISTRO}/g" | xargs apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/src-manifest

############################ dev ##############################
FROM deps AS dev
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      gdb valgrind clang-format clang-tidy ccache sudo nano less \
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
COPY src/ /ws/src/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_cohesion+ \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
    && colcon test --event-handlers console_cohesion+ \
    && colcon test-result --verbose
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
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --upgrade pip \
    && /opt/venv/bin/pip install -r /tmp/requirements-gpu.txt \
    && chown -R ${UID}:${GID} /opt/venv
ENV PATH=/opt/venv/bin:$PATH \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility
USER ${USERNAME}

############################ vamp #############################
# Upstream VAMP, unmodified, at a pinned commit. Built for a portable SIMD baseline
# (x86-64-v3 + AVX2) instead of -march=native, so the image runs on any AVX2 machine and
# not only the one that built it. Override with --build-arg VAMP_ARCH="-march=native -mavx2"
# for a host-specific build.
FROM deps AS vamp-build
ARG VAMP_REF=27cb9b66ebd8d101ce88d330f3b028398afc1cfa
ARG VAMP_ARCH="-march=x86-64-v3 -mavx2"
RUN git clone https://github.com/KavrakiLab/vamp.git /tmp/vamp \
    && git -C /tmp/vamp checkout ${VAMP_REF} \
    && cmake -S /tmp/vamp -B /tmp/vamp/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
         -DVAMP_ARCH="${VAMP_ARCH}" -DVAMP_BUILD_PYTHON_BINDINGS=OFF \
         -DVAMP_BUILD_CPP_DEMO=OFF -DVAMP_BUILD_TESTS=OFF \
         -DCMAKE_INSTALL_PREFIX=/opt/vamp \
    && cmake --build /tmp/vamp/build \
    && cmake --install /tmp/vamp/build \
    && rm -rf /tmp/vamp

############################ full #############################
FROM gpu AS full
ARG USERNAME=dev
USER root
COPY --from=vamp-build /opt/vamp /opt/vamp
ENV CMAKE_PREFIX_PATH=/opt/vamp:${CMAKE_PREFIX_PATH:-}
USER ${USERNAME}
