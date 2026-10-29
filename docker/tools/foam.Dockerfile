# FOAM (CoMMALab/foam, MIT): URDF meshes -> spherized URDF, built from a pinned commit.
# Upstream's own Dockerfile is a developer shell that expects a file missing from the repo, so this
# is a thin, non-interactive tool image instead: mount a work dir, pass the URDF path.
#   docker build -f docker/tools/foam.Dockerfile -t motion-planner-foam:<ref> docker/tools
#   docker run --rm -v "$PWD:/work" -w /work motion-planner-foam:<ref> --filename robot.urdf --output spherized.urdf
FROM ubuntu:24.04
ARG FOAM_REF=116928f71aaa
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git ca-certificates python3 python3-pip python3-venv python3-dev \
      libprotobuf-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/CoMMALab/foam.git /foam \
    && git -C /foam checkout ${FOAM_REF} \
    && git -C /foam submodule update --init --recursive \
    && cmake -S /foam -B /foam/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /foam/build
# Same pins as upstream's requirements.txt minus pybullet (visualisation only).
RUN python3 -m venv /opt/foam-venv && /opt/foam-venv/bin/pip install --no-cache-dir \
      contourpy==1.3.1 cycler==0.12.1 fire==0.7.0 fonttools==4.55.3 kiwisolver==1.4.8 matplotlib==3.10.0 \
      numpy==2.2.2 packaging==24.2 pillow==11.1.0 pyglet==1.5.31 pyparsing==3.2.1 python-dateutil==2.9.0.post0 \
      scipy==1.15.1 six==1.17.0 termcolor==2.5.0 trimesh==4.5.3 xmltodict==0.14.2
ENV PYTHONPATH=/foam
ENTRYPOINT ["/opt/foam-venv/bin/python", "/foam/scripts/generate_sphere_urdf.py"]
