# Development handoff notes

Working notes so the current state, the open problems and the workflow survive a long session or a
change of hands. Kept honest: numbers come with the conditions they were measured under, failures
included. `docs/ROADMAP.md` is the plan; this file is the "where are we right now" page. Update it
when something here changes.

## Current state (2026-11-22, main at #173)

- Pipeline (perception, scene, coverage, stands, planning, control) runs end to end in MuJoCo; Gazebo runs it but the two-stand pass
  fails (below). Backends: MoveIt/OMPL, VAMP, cuRobo v0.9.0 (cuRoboV2-Fast, commit 42498cf). IK default is TRAC-IK `Distance`
  (`ik:=trac_ik|kdl|pick_ik`). Nothing has run on hardware.
- Latest measured MuJoCo two-stand pass (MoveIt/OMPL, main after #167 and #172, one run): 10 of 10 columns, coverage 100 %, peak 22.1 N,
  tool tilt at most 0.15 deg, wall gap at most 0.9 mm (README, "Two-stand pass with MoveIt"). cuRobo passes: four runs, 10 of 10,
  peaks 46.8 to 72.4 N, with MoveIt/OMPL doing the stow and stand 1 (`free_space_fallback`, #141).
- Merged since 2026-11-20 (#143 to #173), grouped (evidence per item in `CHANGELOG.md`, status in `docs/ROADMAP.md` section 19):
  - cuRobo: partial/complete fix (#143), fallback planner and live start state (#141, #147), joint limit guard (#152), endpoint precheck and
    7-joint hop ignore (#169; the low-lift cause is still a hypothesis, no GPU run since).
  - Manager: verify result with mission state (#151; column-count coverage, geometric coverage is open work), column feasibility port and
    reachability-backed adapter (#153, #159; advisory, off by default), stand-check gate and dock IK continuity (#169; opt-in, never run in a sim).
  - Reachability: column sequences with split avoidance (#153), cheaper verified stands (#154), N-DOF arms and redundancy (#155),
    `ColumnIkChain` and the path quality audit (#166; ROS-free, not called by the planner or manager yet), plane-generic stands and the base
    placement optimiser (#167).
  - Scoring and tools: tool-pose scoring (tilt, wall gap) and `MP_PROFILE` for any robot in the MuJoCo passes (#160 via #167), tool contact
    frame, tilt and plane-touch calibration core (#164 via #167).
  - New ROS-free cores, host-tested only: `diff_ik_core`, `safety_monitor` (not a certified safety function), `teleop_core` with the
    contact-free gate, stand_navigator lateral dock term and noise harness (#172).
  - Fixes: start-state clamps in OMPL and Cartesian planners (#156, #157), review fixes for the new cores (#172).
  - Repository: CI on free runners, issue and PR templates, package READMEs, `docs/modules.md`, `docs/development_plan.md` (#167).
- Not merged, in flight (see the board in the session notes, not in this repo): a submodule split of the package groups, a MoveIt
  null-TF fix in the manager node and a first 7-DOF (xArm 7) run, a mission behaviour tree, a Gazebo dock-continuity rerun.

## Open problems

1. **Multi-stand second stand (PR #110, merged 2026-11-20):** same-session 8-run batches: plain wall 6/8
   before, 8/8 after; `two_panels` 8/8 both (scored on reachable wall; the script's own wall-coverage score reads it as 0.88).
   The pass-rate gap is not significant (Fisher p about 0.48); truncated plans (5+ down to 0-6 per run) and peak force
   (304/141 N down to 88 N) are the clearer signs. Not isolated: which commit mattered; Gazebo not rerun.
2. **Gazebo two-stand pass fails** (2026-11-21, main after #157): 1 of 10 columns (6 of 10 in the run before; single runs, the spread is large). No
   "start state out of bounds" error is left (the Cartesian planner start state was the unclamped path, #157). The log shows the first dock
   approach on stand 0 ending in one IK branch, the next attempt jumping to another (shoulder pan -0.62 to +2.49 rad), every later column
   failing "dock failed" from that pose and the wrist winding to -4.6 rad, then 36 failed stows. Seeded dock IK is off in the Gazebo script
   (it gave an invalid goal there). Opt-in `dock_ik_continuity` (#169, `MULTI_STAND_DOCK_IK_CONTINUITY=true`) is the first attempt; not measured on
   main yet. Contact force spikes in the scored trace (1240 N, earlier 1820 N) are a separate, unexplained Gazebo contact-model problem. Details in `docs/sim_gazebo.md`.
3. **cuRobo as the free-space planner in a multi-stand pass: runs, with a fallback doing part of the work.** (History of the investigation:) Wired and run once in MuJoCo (2026-11-20):
   the node answers, the wall loads per stand (`source_frame:=odom`), the manager seeds the first segment's start state
   (`seed_start_from_live_state`), cuRobo is reached (`fsp_backend=curobo`) but refuses the stand approaches with `Start or End
   state in collision` (stow-to-stow with an empty world succeeds), and the manager's recovery loops. Likely the approach end
   state inside cuRobo's activation distance of the 0.05 m wall voxels; not confirmed. See `docs/sim_mujoco.md`. Later the same day: the refusals were the
   bridge's `--max-boxes 256` coarsening the wall to 0.2 m slabs (use 3000); with that, stand 0 runs under cuRobo, but cuRobo
   finds no docked-to-stow path even in an empty world (low lift + arm swung out, likely sphere-model collision), so the stow
   never finishes. `check_motion` confirms it: the docked arm pose is invalid at lift 0.0 and 0.01 and valid from 0.05. With
   `free_space_fallback: moveit_ompl` (#141) the two-stand pass PASSES end to end (10 of 10 columns, coverage 1.0), but
   MoveIt/OMPL did the work for the segments cuRobo planned (its trajectories stop `partially_completed` at execution; cause not
   isolated) and for the stow. Next: why cuRobo trajectories stop early at execution, the cuRobo robot spheres at low lift, and the
   per-stand world load timing out; then a Gazebo pass. Since: the four MuJoCo runs and a film (README), a start/goal precheck and
   `--ignore-hops 2` for the 7-joint config (#169), both unconfirmed on a GPU.
4. **Verified stand selection is slow** (about 5 minutes on one wall, not converged at 64 rounds, measured before the cost
   work). Since then: `stop_at_first_ok` and `max_verifier_calls` cut and bound the IK probes, `~/plan_stands` can verify
   (`verified` request flag, `verify_by_default`; `docs/reachability.md`). Call-count reductions are tested with fakes;
   not re-measured on MoveIt. The manager does not use verified stands yet, and the column feasibility port is advisory (nothing is skipped by it).
5. **Dockerfile layer cache:** any `package.xml` change invalidates the dependency layers and the
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

`slots.sh` says what may start right now (it calls `memwatch.sh` and counts the running containers). Run it before every
container and start only what it answers YES to. Limits it enforces: one simulation container (about 3 GB, cores 0-7); one
cuRobo node (GPU, about 2.5 GB, one GPU job at a time); one colcon build/test container (one worker; `-j1` and `--memory=2800m` from 4300 MB available, `-j2` and 3800 MB above
5300 MB), a second only with 7000 MB available; git, gh, docs and downloads always. Each start must leave 1500 MB
free. Do not keep scratch files in `/tmp`: a reboot clears it.

## Docker images

`deps` (dependencies from `package.xml` via rosdep), `build` (workspace built, tests run in the
Dockerfile), `dev`, `gpu`/`full` (cuRobo v0.9.0 + CUDA torch; the same image), `sim` (MuJoCo, MJX,
JAX), `sim-gazebo`, `vamp-build`, plus the `foam` and `cricket` experiment images. Most are stages of
one Dockerfile and share layers, so displayed sizes overstate the real disk use.

Rebuilding without the test step (when only a fast workspace is needed): run `docker/build.sh` in a
container from `deps` with the source copied in, then `docker commit`. Day to day, mount `src/` and
build only the changed packages with `--packages-select` into persistent named volumes.

**Registry plan:** publish `deps`, `build` and `sim` to `ghcr.io/lakshya-04/motion-planner` by hand
after the first full end-to-end workflow test passes; the GPU image (about 12 GB) last and
optional. Needs `gh auth refresh -s write:packages`.

## Rules the project follows

- Commits are authored as the project owner; no AI attribution lines in commits, PRs or files.
- CI runs only on free GitHub-hosted runners (public repo): readiness scan, tool tests, colcon build and tests. Simulators and GPU jobs stay local.
- Pull requests are squash-merged with Conventional Commit titles, one theme per PR (CONTRIBUTING.md).
- Be honest in docs: measured numbers with load, failures included.
