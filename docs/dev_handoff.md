# Development handoff notes

Working notes so the current state, the open problems and the workflow survive a long session or a
change of hands. Kept honest: numbers come with the conditions they were measured under, failures
included. `docs/ROADMAP.md` is the plan; this file is the "where are we right now" page. Update it
when something here changes.

## Current state (2026-11-19)

- Pipeline (perception, scene, coverage, stands, planning, control) runs end to end in MuJoCo and
  Gazebo. Backends: MoveIt/OMPL, VAMP, cuRobo v0.9.0 (cuRoboV2-Fast, commit 42498cf4).
- Stand candidates from the RM4D reachability map are confirmed with IK and collision checks
  (`reachability_moveit`, `stand_verify_probe`, `docs/reachability.md`).
- IK default is TRAC-IK `Distance` (`ik:=trac_ik|kdl|pick_ik`).

## Open problems

1. **Multi-stand second stand fails about half the time** in older runs (MuJoCo 3 of 8 failed at the
   base measurement, Gazebo 1 of 3). A later batch passed 8 of 8; which change made the difference
   is not yet pinned down. Work: PR "Multi-stand: second stand fails about every other run".
2. **cuRobo cannot see the wall in a sim pass.** The manager never feeds the scene to the cuRobo
   client, and `planner_used` in the logs is always `ompl`. `curobo_world_loader` (curobo_planner)
   pushes the wall cloud to the cuRobo node in the base frame; wiring it into the pass scripts and
   logging the planner actually used are still to do. Until then no run is a cuRobo run.
3. **Verified stand selection is slow** (about 5 minutes on one wall, not converged at 64 rounds);
   the manager and `~/plan_stands` do not use verified stands yet.
4. **Dockerfile layer cache:** any `package.xml` change invalidates the dependency layers and the
   12 GB torch layer is rebuilt. Fix: make the GPU layers independent of `COPY src` manifests.
5. The Dockerfile `build` stage runs `mp-build test`; two tests failed there only because of the
   image environment (docs tree absent; `robot_self_filter` not on the test path). Fixed, see
   `git log -- src/manipulation/manipulation_bt_prototype/test`.
6. UR10e VAMP/Cricket model reports every configuration in collision (parked). A real UR10e
   forearm/wrist_2 contact is unconfirmed.

## Host constraints (the machine these numbers come from)

7.8 GB RAM (often about 2 GB free), RTX 3050 4 GB (one GPU job at a time), tight disk, one sim
container at a time. Pin sims to cores 0-7, benchmarks to 10-11. Wall-clock timeouts are
invalidated by laptop suspend, so discard runs that straddle one.

`docker/tools/hostwatch/` has a memory watchdog (`memwatch.sh`: GO / HOLD / PAUSE from free RAM,
swap and kernel memory pressure, with hysteresis) and a job runner (`jobrunner.sh`) that starts
queued container jobs only on GO, caps each job's memory and cores, and pauses (never kills) only
containers it started. State lives in `$HOSTWATCH_DIR` (default `~/.cache/hostwatch`); queue files
are `queue/*.job` (bash: NAME, MEM_MB, CPUS, MAXRUN_MIN, WAIT_FILE, CMD).

## Docker images

`deps` (dependencies from `package.xml` via rosdep), `build` (workspace built, tests run in the
Dockerfile), `dev`, `gpu`/`full` (cuRobo v0.9.0 + CUDA torch; the same image), `sim` (MuJoCo, MJX,
JAX), `sim-gazebo`, `vamp-build`, plus the `foam` and `cricket` experiment images. Most are stages of
one Dockerfile and share layers, so displayed sizes overstate the real disk use.

Rebuilding without the test step (when only a fast workspace is needed): run `docker/build.sh` in a
container from `deps` with the source copied in, then `docker commit`. Day to day, mount `src/` and
build only the changed packages with `--packages-select` into persistent named volumes.

**Registry plan:** publish `deps`, `build` and `sim` to `ghcr.io/lakshya-04/motion-planner` by hand
(no CI) after the first full end-to-end workflow test passes; the GPU image (about 12 GB) last and
optional. Needs `gh auth refresh -s write:packages`.

## Rules the project follows

- Commits are authored as the project owner; no AI attribution lines in commits, PRs or files.
- No CI/CD configuration in the repo.
- Be honest in docs: measured numbers with load, failures included.
