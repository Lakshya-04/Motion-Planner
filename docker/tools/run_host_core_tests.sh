#!/usr/bin/env bash
# Build and run the ROS-free C++ core tests with plain g++ (no ROS, no colcon, no docker).
# The list of cores, tests and exclusions is docker/tools/host_core_tests.manifest.
#
#   docker/tools/run_host_core_tests.sh [-j N] [-k REGEX] [-b BUILD_DIR] [-m MANIFEST]
#
# Needs: g++ (C++17), libgtest-dev, libeigen3-dev, libyaml-cpp-dev, libtinyxml2-dev, nlohmann-json3-dev.
# Prints a pass/fail table; exit status is non-zero if any test failed to build or failed to run.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
JOBS="$(nproc 2>/dev/null || echo 2)"
FILTER=""
BUILD="${TMPDIR:-/tmp}/host_core_tests_build"
MANIFEST="$ROOT/docker/tools/host_core_tests.manifest"
while getopts "j:k:b:m:h" o; do
  case "$o" in
    j) JOBS="$OPTARG" ;; k) FILTER="$OPTARG" ;; b) BUILD="$OPTARG" ;; m) MANIFEST="$OPTARG" ;;
    *) sed -n '2,9p' "$0"; exit 2 ;;
  esac
done
cd "$ROOT" || exit 2
CXX="${CXX:-g++}"
command -v "$CXX" >/dev/null || { echo "no $CXX found" >&2; exit 2; }
CXXFLAGS="-std=c++17 -O1 -w -I/usr/include/eigen3 ${CXXFLAGS:-}"
mkdir -p "$BUILD/obj" "$BUILD/bin" "$BUILD/log" "$BUILD/res"
rm -f "$BUILD"/res/* "$BUILD"/log/*
export ROOT BUILD CXX CXXFLAGS

trim() { local v="$*"; v="${v#"${v%%[![:space:]]*}"}"; echo "${v%"${v##*[![:space:]]}"}"; }
declare -A LIB_INC LIB_DEPS LIB_LINK
LIB_SRC_JOBS=(); TESTS=(); EXCL=()
while IFS= read -r line; do
  [[ -z "${line// }" || "$line" == \#* ]] && continue
  IFS='|' read -r kind a b c d e <<<"$line"
  kind="$(trim "$kind")"
  case "$kind" in
    lib)
      n="$(trim "$a")"; LIB_INC[$n]="$(trim "$b")"; LIB_DEPS[$n]="$(trim "$d")"; LIB_LINK[$n]="$(trim "$e")"
      i=0; for s in $c; do LIB_SRC_JOBS+=("$n $i $s"); i=$((i+1)); done ;;
    test) TESTS+=("$(trim "$a")|$(trim "$b")|$(trim "$c")|$(trim "$d")") ;;
    exclude) EXCL+=("$(trim "$a")|$(trim "$b")") ;;
  esac
done <"$MANIFEST"

# 1. Compile every library source once, in parallel.
compile_obj() {
  read -r lib idx src <<<"$1"
  incs=""; for l in $(closure_cached "$lib"); do incs+=" -I${INCS[$l]}"; done
  mkdir -p "$BUILD/obj/$lib"
  $CXX $CXXFLAGS $incs -c "$src" -o "$BUILD/obj/$lib/$idx.o" >"$BUILD/log/obj_${lib}_$idx.txt" 2>&1 \
    || { echo "LIB-FAIL $lib $src"; return 1; }
}
# export lib tables as flat strings for the xargs subshells
export LIBTAB="$(for l in "${!LIB_INC[@]}"; do echo "$l|${LIB_INC[$l]}|${LIB_DEPS[$l]}|${LIB_LINK[$l]}"; done)"
cat >"$BUILD/lib.sh" <<'LIBSH'
declare -A INCS DEPS LINK
while IFS='|' read -r l i d k; do INCS[$l]="$i"; DEPS[$l]="$d"; LINK[$l]="$k"; done <<<"$LIBTAB"
closure_cached() { local seen=" " out=""; _c() { local l="$1" d
    [[ "$seen" == *" $l "* ]] && return; seen+="$l "; for d in ${DEPS[$l]}; do _c "$d"; done; out+="$l "; }
  for l in "$@"; do _c "$l"; done; echo "$out"; }
LIBSH
export -f compile_obj
objs_fail=0
printf '%s\n' "${LIB_SRC_JOBS[@]}" | xargs -d '\n' -P "$JOBS" -I{} bash -c 'source "$BUILD/lib.sh"; compile_obj "$1"' _ {} \
  | tee "$BUILD/libfail.txt"
[[ -s "$BUILD/libfail.txt" ]] && objs_fail=1
for l in "${!LIB_INC[@]}"; do
  files=$(ls "$BUILD/obj/$l"/*.o 2>/dev/null || true)
  rm -f "$BUILD/lib$l.a"; [[ -n "$files" ]] && ar rcs "$BUILD/lib$l.a" $files
done

# 2. Build and run each test.
run_test() {
  IFS='|' read -r name libs src extra <<<"$1"
  source "$BUILD/lib.sh"
  cl="$(closure_cached $libs)"
  incs=""; ar_list=""; link=""
  for l in $cl; do incs+=" -I${INCS[$l]}"; link+=" ${LINK[$l]}"; done
  for l in $(echo "$cl" | tr ' ' '\n' | tac); do [[ -f "$BUILD/lib$l.a" ]] && ar_list+=" $BUILD/lib$l.a"; done
  log="$BUILD/log/$name.txt"
  if ! eval "$CXX $CXXFLAGS $incs $extra \"$src\" $ar_list -lgtest -lgtest_main $link -pthread -o \"$BUILD/bin/$name\"" >"$log" 2>&1; then
    echo "BUILD-FAIL" >"$BUILD/res/$name"; return 0
  fi
  start=$(date +%s%N)
  if timeout 300 "$BUILD/bin/$name" >>"$log" 2>&1; then r=PASS; else r=FAIL; fi
  ms=$(( ($(date +%s%N) - start) / 1000000 ))
  echo "$r $((ms / 1000)).$(printf %03d $((ms % 1000)))" >"$BUILD/res/$name"
}
sel=()
for t in "${TESTS[@]}"; do
  [[ -n "$FILTER" && ! "${t%%|*}" =~ $FILTER ]] && continue
  sel+=("$t")
done
printf '%s\n' "${sel[@]}" | xargs -d '\n' -P "$JOBS" -I{} bash -c 'source "$BUILD/lib.sh"; export -f closure_cached; '"$(declare -f run_test)"'; run_test "$1"' _ {}

# 3. Table.
pass=0; fail=0; bfail=0
printf '\n%-44s %-11s %s\n' TEST RESULT SECONDS
printf '%-44s %-11s %s\n' ---- ------ -------
for t in "${sel[@]}"; do
  n="${t%%|*}"; read -r r s <"$BUILD/res/$n" 2>/dev/null || { r=MISSING; s=-; }
  printf '%-44s %-11s %s\n' "$n" "$r" "${s:-}"
  case "$r" in PASS) pass=$((pass+1)) ;; BUILD-FAIL) bfail=$((bfail+1)) ;; *) fail=$((fail+1)) ;; esac
done
for e in "${EXCL[@]}"; do
  [[ -n "$FILTER" && ! "$(basename "${e%%|*}" .cpp)" =~ $FILTER ]] && continue
  printf '%-44s %-11s %s\n' "$(basename "${e%%|*}" .cpp)" EXCLUDED "${e#*|}"
done
printf '\npassed %d, failed %d, build failures %d, excluded %d (logs: %s/log)\n' "$pass" "$fail" "$bfail" "${#EXCL[@]}" "$BUILD"
for t in "${sel[@]}"; do n="${t%%|*}"; read -r r _ <"$BUILD/res/$n" 2>/dev/null || r=MISSING
  if [[ "$r" != PASS ]]; then echo "--- $n ($r): last log lines"; tail -n 12 "$BUILD/log/$n.txt" 2>/dev/null; fi; done
[[ $fail -eq 0 && $bfail -eq 0 && $objs_fail -eq 0 ]]
