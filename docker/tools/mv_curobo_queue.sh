#!/usr/bin/env bash
# usage: mv_curobo_queue.sh <prio> <name> <worktree> [VAR=value ...]   queue mv_curobo_run.sh as one simq job
# (VAR=value: MULTI_STAND_*, CUROBO_OPTIMIZE=1 CUROBO_TIME_BUDGET= CUROBO_CANDIDATES= CUROBO_OBJECTIVE=, SIM_LANE=)
P=$1; N=$2; W=$3; shift 3
I=${INFRA:-/home/hmm/mp-infra}; J=$I/jobs/$N.sh; R=${CUROBO_RUN_SH:-$I/mv_curobo_run.sh}
{ echo "# MuJoCo cuRobo-backend run $N ($W)"; for kv in "$@"; do echo "export $kv"; done; echo "bash $R $N $W > $I/cp/$N.driver.out 2>&1"; } > "$J"
SIMQ_BY=${SIMQ_BY:-worker} bash /home/hmm/.claude/coordination/motion-planner/simq_add.sh "$P" "$N" "$J"
