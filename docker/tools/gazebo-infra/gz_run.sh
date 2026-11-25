#!/usr/bin/env bash
# usage: [MULTI_STAND_*=... GZ_PASS_*=... STAND_MAP=...] gz_run.sh <name> <worktree>
# Build in the sim-gazebo image (shared ccache via cctool, per-lane build volumes) then a VISIBLE (gui:=true) Gazebo two-stand pass
# (src/simulation/sim_gazebo/scripts/run_multi_stand_pass.sh). Output: /home/hmm/mp-infra/cp/<name>_out, log cp/<name>.out.
# Start only through gz_queue.sh / simq (one sim at a time, simgui.md). Needs `xhost +SI:localuser:root` and an unlocked screen.
NAME=$1; W=$2; I=/home/hmm/mp-infra; H=/home/hmm/Motion-Planner/docker/tools/hostwatch; L=${SIM_LANE:-a}; IMG=${GZ_IMAGE:-motion-planner:sim-gazebo}
export DISPLAY=${DISPLAY:-:0}
docker volume create gzl_${L}_b >/dev/null; docker volume create gzl_${L}_i >/dev/null
while true; do o=$(bash $H/slots.sh); if echo "$o" | grep -q "^build  YES" && echo "$o" | grep -q "^sim    YES"; then break; fi; sleep 30; done
docker run --rm --user root --cpuset-cpus ${SIM_CPUSET:-0-5} --memory=2800m -v $I/cctool:/cct:ro -v $I/fast/ccache:/ccache -e CCACHE_DIR=/ccache -v $W/src:/ws/src:ro -v gzl_${L}_b:/ws/build -v gzl_${L}_i:/ws/install -v $I/cp:/cp -w /ws $IMG bash /cp/build_gz.sh 2>&1 | tail -4
echo BUILT
while true; do o=$(bash $H/slots.sh); if echo "$o" | grep -q "^sim    YES"; then break; fi; sleep 30; done
ENVS=$(env | grep -E '^(MULTI_STAND_|MANAGER_PASS_|GZ_PASS_|STAND_|SC=)' | sed "s/'/'\\\\''/g; s/^/-e '/; s/\$/'/" | tr '\n' ' ')
docker rm -f gz_sim_$L >/dev/null 2>&1
eval docker run --rm --name gz_sim_$L --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __NV_PRIME_RENDER_OFFLOAD=1 --user 1000:1000 -e HOME=/tmp --network host --ipc host --memory=3000m --memory-swap=3000m --cpuset-cpus=${SIM_CPUSET:-0-7} -e NAME=$NAME -e ROS_DOMAIN_ID=$((83+$( [ "$L" = b ] && echo 1 || echo 0 ))) -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix $ENVS -v $W/src:/ws/src:ro -v gzl_${L}_b:/ws/build -v gzl_${L}_i:/ws/install -v $I/cp:/cp -w /ws $IMG bash /cp/gz_in.sh > $I/cp/$NAME.out 2>&1 &
sleep 60
while docker ps --format '{{.Names}}' | grep -q "gz_sim_$L"; do sleep 30; v=$(bash $H/memwatch.sh); if echo "$v" | grep -q "verdict=PAUSE"; then docker pause gz_sim_$L >/dev/null; while ! bash $H/memwatch.sh | grep -q "verdict=GO"; do sleep 20; done; docker unpause gz_sim_$L >/dev/null; fi; done
echo FINISHED
tail -25 $I/cp/$NAME.out | cut -c1-200
