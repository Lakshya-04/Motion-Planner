# Optional add-on: NVIDIA Isaac Sim with this repo's Isaac scripts. Not part of any default build.
# Accepting NVIDIA's licence is up to whoever runs it (ACCEPT_EULA at run time); see docs/sim_isaac.md.
#   docker build -f docker/isaac/Dockerfile -t motion-planner:isaac .
ARG ISAAC_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0
FROM ${ISAAC_IMAGE}
COPY src/simulation/sim_isaac/scripts/ /sim_isaac/scripts/
COPY src/description/mobile_manipulator_description/config/profiles/ /sim_isaac/profiles/
# cache dirs the compose volumes mount over; world-writable so a fresh named volume works for uid 1234
USER root
RUN mkdir -p -m 777 /isaac-sim/kit/cache /isaac-sim/.cache /isaac-sim/.local/share/ov/data /isaac-sim/.nv/ComputeCache /isaac-sim/.nvidia-omniverse/logs
USER isaac-sim
