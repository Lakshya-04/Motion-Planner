# Development handoff notes

Working notes so the current state, the open problems and the workflow survive a long session or a
change of hands. Kept honest: numbers come with the conditions they were measured under, failures
included. `docs/ROADMAP.md` is the plan; this file is the "where are we right now" page. Update it
when something here changes.

## Current state (2026-11-20)

- Pipeline (perception, scene, coverage, stands, planning, control) runs end to end in MuJoCo and
  Gazebo. Backends: MoveIt/OMPL, VAMP, cuRobo v0.9.0 (cuRoboV2-Fast, commit 42498cf4).
- Stand candidates from the RM4D reachability map are confirmed with IK and collision checks
  (`reachability_moveit`, `stand_verify_probe`, `docs/reachability.md`).
- Merged 2026-11-20 (ROADMAP items, all software-only, none run in a sim): verify-result contract and thresholds (#128),
  adaptive edge spacing (#129), mission_command pause/resume/status service (#130), tool-actuation executor feed (#131),
  `planStandsVerified` (#132). The cuRobo-backed stand verifier was skipped (needs bridge changes and the GPU).
- IK default is TRAC-IK `Distance` (`ik:=trac_ik|kdl|pick_ik`).

## Open problems

1. **Multi-stand second stand (PR #110, merged 2026-11-20):** same-session 8-run batches: plain wall 6/8
   before, 8/8 after; `two_panels` 8/8 both (scored on reachable wall; the script's own wall-coverage score reads it as 0.88).
   The pass-rate gap is not significant (Fisher p about 0.48); truncated plans (5+ down to 0-6 per run) and peak force
   (304/141 N down to 88 N) are the clearer signs. Not isolated: which commit mattered; Gazebo not rerun.
2. **cuRobo as the free-space planner in a multi-stand pass does NOT work yet.** Wired and run once in MuJoCo (2026-11-20):
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
   per-stand world load timing out; then a film and a Gazebo pass.
3. **Verified stand selection is slow** (about 5 minutes on one wall, not converged at 64 rounds, measured before the cost
   work). Since then: `stop_at_first_ok` and `max_verifier_calls` cut and bound the IK probes, `~/plan_stands` can verify
   (`verified` request flag, `verify_by_default`; `docs/reachability.md`). Call-count reductions are tested with fakes;
   not re-measured on MoveIt. The manager does not use verified stands yet.
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

`slots.sh` says what may start right now (it calls `memwatch.sh` and counts the running containers). Run it before every
container and start only what it answers YES to. Limits it enforces: one simulation container (about 3 GB, cores 0-7); one
cuRobo node (GPU, about 2.5 GB, one GPU job at a time); one colcon build/test container (one worker, `-j2`,
`--memory=3800m`), a second only with 7000 MB available; git, gh, docs and downloads always. Each start must leave 1500 MB
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
(no CI) after the first full end-to-end workflow test passes; the GPU image (about 12 GB) last and
optional. Needs `gh auth refresh -s write:packages`.

## Rules the project follows

- Commits are authored as the project owner; no AI attribution lines in commits, PRs or files.
- No CI/CD configuration in the repo.
- Be honest in docs: measured numbers with load, failures included.
