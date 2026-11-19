#!/usr/bin/env bash
# Memory-aware job runner. Jobs are files in queue/*.job (sourced bash):
#   NAME=checks MEM_MB=3000 CPUS=8-11 MAXRUN_MIN=90 WAIT_FILE=/path/log (optional: wait until it contains "exit=0")
#   CMD='docker run --rm --name jr_{NAME} --label jobrunner=1 --memory={MEM}m --memory-swap={MEM}m --cpuset-cpus={CPUS} ... '
# A job starts only when the verdict is GO and avail_mb - MEM_MB >= RESERVE and fewer than MAXJOBS run.
# Under PAUSE it `docker pause`s ONLY containers labelled jobrunner=1 (reversible); it never kills anything and never
# touches other containers or processes. Stop it with: touch $HOSTWATCH_DIR/STOP
O=${HOSTWATCH_DIR:-$HOME/.cache/hostwatch}; RESERVE=1500; MAXJOBS=3; INTERVAL=15
log() { echo "$(date +%T) $*" >> $O/logs/runner.log; }
log "runner started (reserve=${RESERVE}MB maxjobs=$MAXJOBS)"
while [ ! -e $O/STOP ]; do
  line=$(bash "$(dirname "$(readlink -f "$0")")/memwatch.sh"); eval "$line"; verdict=${verdict:-HOLD}
  # reap finished jobs
  for f in $O/running/*.job; do [ -e "$f" ] || continue
    pidf=${f%.job}.pid; if [ -e "$pidf" ] && ! kill -0 "$(cat $pidf)" 2>/dev/null; then
      rc=$(cat ${f%.job}.rc 2>/dev/null || echo '?'); mv "$f" $O/done/; rm -f "$pidf"; log "finished $(basename $f .job) rc=$rc"; fi; done
  # pause / resume the runner's own containers
  if [ "$verdict" = PAUSE ]; then
    for c in $(docker ps -q --filter label=jobrunner=1 --filter status=running); do docker pause $c >/dev/null 2>&1 && log "PAUSED $c ($line)"; done
  elif [ "$verdict" = GO ] || { [ "$verdict" = HOLD ] && [ "${avail_mb:-0}" -ge 3000 ]; }; then
    for c in $(docker ps -q --filter label=jobrunner=1 --filter status=paused); do docker unpause $c >/dev/null 2>&1 && log "RESUMED $c ($line)"; done
  fi
  # start at most one new job per cycle
  if [ "$verdict" = GO ] && [ "${jobs:-0}" -lt $MAXJOBS ]; then
    for jf in $(ls $O/queue/*.job 2>/dev/null | sort); do
      unset NAME MEM_MB CPUS MAXRUN_MIN WAIT_FILE CMD; . "$jf"
      if [ -n "${WAIT_FILE:-}" ] && ! grep -q '^exit=0' "$WAIT_FILE" 2>/dev/null; then continue; fi
      if [ $(( ${avail_mb:-0} - ${MEM_MB:-2000} )) -lt $RESERVE ]; then log "skip $NAME: needs ${MEM_MB}MB, avail ${avail_mb}MB"; continue; fi
      mv "$jf" $O/running/; run=$O/running/$(basename $jf); cmd=${CMD//\{MEM\}/$MEM_MB}; cmd=${cmd//\{CPUS\}/$CPUS}; cmd=${cmd//\{NAME\}/$NAME}
      ( timeout $(( ${MAXRUN_MIN:-90} * 60 )) bash -c "$cmd" > $O/logs/$NAME.log 2>&1; echo $? > ${run%.job}.rc ) &
      echo $! > ${run%.job}.pid; log "STARTED $NAME (mem ${MEM_MB}MB, cpus $CPUS) ($line)"; break
    done
  fi
  sleep $INTERVAL
done
log "runner stopped"
