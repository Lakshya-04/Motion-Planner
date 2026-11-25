#!/usr/bin/env bash
# usage: gz_queue.sh <prio> <name> <worktree> [VAR=value ...]   queue a VISIBLE Gazebo two-stand run through simq (one job: build + run + collect)
# e.g. gz_queue.sh 30 gazebo-plain <worktree>    ;    ... MULTI_STAND_PATCH_Z=0.6,2.0 MULTI_STAND_DOCK_IK_CONTINUITY=true
P=$1; N=$2; W=$3; shift 3
I=${MP_INFRA:-$HOME/mp-infra}; J=$I/jobs/$N.sh
{ echo "# Gazebo run $N ($W)"; for kv in "$@"; do echo "export '$kv'"; done; echo "bash $I/gz_run.sh $N $W > $I/cp/$N.driver.out 2>&1"; } > $J
SIMQ_BY=${SIMQ_BY:-master} bash ${SIMQ_ADD:-$HOME/simq_add.sh} $P $N $J
