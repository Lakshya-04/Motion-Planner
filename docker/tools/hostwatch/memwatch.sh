#!/usr/bin/env bash
# Prints one line: avail_mb swap_mb psi_some10 load running_jobs verdict. Verdict has hysteresis (state file):
#   GO     enough headroom to start another job   (avail >= 3600 MB, memory pressure < 8, swap growth ok)
#   HOLD   do not start anything new               (avail < 2200 MB or pressure >= 8)
#   PAUSE  pause the runner's own containers       (avail < 900 MB or pressure >= 25)
# Back to GO only when avail >= 4000 MB and pressure < 4 (so it does not flap).
S=${HOSTWATCH_DIR:-$HOME/.cache/hostwatch}/.verdict; mkdir -p "$(dirname "$S")"
avail=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
swap=$(awk '/SwapTotal/{t=$2}/SwapFree/{f=$2}END{print int((t-f)/1024)}' /proc/meminfo)
psi=$(awk '/^some/{split($2,a,"=");print a[2]}' /proc/pressure/memory 2>/dev/null); psi=${psi:-0}
load=$(awk '{print $1}' /proc/loadavg)
jobs=$(docker ps -q --filter label=jobrunner=1 2>/dev/null | wc -l)
prev=$(cat $S 2>/dev/null || echo HOLD)
v=$prev
lt() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a<b)}'; }
if lt "$avail" 900 || ! lt "$psi" 25; then v=PAUSE
elif lt "$avail" 2200 || ! lt "$psi" 8; then v=HOLD
elif [ "$prev" = GO ]; then v=GO
elif ! lt "$avail" 4000 && lt "$psi" 4; then v=GO
elif [ "$prev" = PAUSE ]; then v=HOLD
fi
[ -n "${MW_FORCE:-}" ] && v=$MW_FORCE   # test hook only
echo "$v" > $S
echo "avail_mb=$avail swap_mb=$swap psi=$psi load=$load jobs=$jobs verdict=$v"
