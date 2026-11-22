#!/usr/bin/env bash
# How many more jobs of each kind this host can take right now. Run it before starting a container; start a job only when
# its line says YES. Uses memwatch.sh for the memory verdict (so one hysteresis state) and counts the running containers.
#
#   class   cost   rule
#   sim     3000   one simulation container at a time (MuJoCo or Gazebo pass); cores 0-7
#   node    2500   one cuRobo node (GPU, 4 GB card: one GPU job at a time); only useful together with a sim
#   build   2800   colcon build/test container, one worker, -j2 (3 workers x -j3 was OOM-killed at 3.5 GB); at most 2,
#                  the second only when 7000 MB are available
#   light      0   git, gh, docs, downloads, ls: always
# Every start must leave RESERVE MB free (the host has 7.8 GB and ~2 GB baseline use). Memory verdict HOLD/PAUSE: nothing new.
RESERVE=${SLOTS_RESERVE:-700}
here=$(dirname "$(readlink -f "$0")")
line=$(bash "$here/memwatch.sh"); eval "$line"
names=$(docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null)
n_sim=$(echo "$names" | grep -c -E ' motion-planner:(sim|sim-gazebo)$')
n_node=$(echo "$names" | grep -c -E '^curobo-node ')
n_build=$(echo "$names" | grep -v -E '^(mp_rviz|plan_pb)' | grep -c -E ' motion-planner:(build|build-fast|build-live|dev)$')
gpu_used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1); gpu_used=${gpu_used:-0}
verdict=${verdict:-HOLD}; avail=${avail_mb:-0}
ok() { [ "$verdict" = GO ] && [ $((avail - $1)) -ge "$RESERVE" ]; }
say() { printf '%-6s %-3s %s\n' "$1" "$2" "$3"; }
echo "$line running: sim=$n_sim node=$n_node build=$n_build gpu_used_mb=$gpu_used"
if [ "$n_sim" -ge 2 ]; then say sim NO "two simulation containers are already running"
elif ok 3000; then say sim YES "cores 0-7"; else say sim NO "verdict=$verdict avail=${avail}MB"; fi
if [ "$n_node" -ge 1 ]; then say node NO "already running"
elif [ "$gpu_used" -gt 1500 ]; then say node NO "GPU busy (${gpu_used} MB used)"
elif ok 2500; then say node YES "GPU"; else say node NO "verdict=$verdict avail=${avail}MB"; fi
if [ "$n_build" -ge 2 ]; then say build NO "two build containers already running"
elif [ "$n_build" -ge 1 ] && [ "$avail" -lt 7000 ]; then say build NO "one is running; a second needs 7000 MB available"
elif ok 2400; then say build YES "one worker, -j1, --memory=2400m (-j2 and 3800m only above 5300 MB available)"; else say build NO "verdict=$verdict avail=${avail}MB"; fi
say light YES "git, gh, docs, downloads"
