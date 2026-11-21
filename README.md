# Motion Planner

ROS 2 packages for covering a surface with a tool using a mobile manipulator: sensor data to coverage
plan, base stands, arm motion planning and execution on a compliant (admittance) controller with force
feedback. The target is a stack that works for any arm, lift and base (chosen by a robot profile) and any
tool (contact like a mop, non-contact like a spray gun), in simulation first and on hardware by parameters.

**Status in one line:** the building blocks exist and are tested, and both simulators now run the pipeline
end to end in headless one-command passes (coverage planner, manager, MoveIt, compliant controller, recorded
and scored), including multi-stand passes where the base drives between stands. Nothing here has run on hardware.
The per-area checklist, with evidence for every item, is [docs/ROADMAP.md](docs/ROADMAP.md).

**Where it stands (2026-11-21):**
- **Single-stand wall passes** in MuJoCo are reliable (8 of 8 runs across four scenarios, with obstacles).
- **Two-stand passes in MuJoCo** now pass. Same-session batches of 8 runs each: plain wall 6 of 8 before the second-stand fix and 8 of 8
  after, the rib scenario `two_panels` 8 of 8 both times (scored on the reachable wall). That 6/8 vs 8/8 gap is not statistically
  significant on its own; the clear differences are fewer truncated plans (at least 5 per run before, 0 to 6 after) and lower
  peak forces. Peaks are still 47 to 88 N against a 10 N target on these passes (a 10 N hold works in the single-stand test).
- **cuRobo v0.9.0 (cuRoboV2-Fast) as the free-space planner** now completes a two-stand MuJoCo pass: four runs, all 10 of 10 columns,
  coverage 1.0, peak force 46.8 to 72.4 N. How much cuRobo did, honestly: it planned the stand-0 approach moves (12 to 16 s each on
  this 4 GB laptop GPU, executed in 0.4 to 0.7 s); MoveIt/OMPL planned the stow and the second stand, either because cuRobo
  could not (no path from the docked pose to the stow, even in an empty world: its robot model treats low lift with the arm swung out as
  in collision) or did not answer in time. A `free_space_fallback` makes that automatic and logged. See [the pass below](#curobo-in-a-two-stand-pass-mujoco).
- **Gazebo two-stand passes currently fail** (0 of 10 columns in two runs on 2026-11-21, with and without cuRobo: MoveIt reports an invalid
  dock goal and a start state out of bounds). Earlier Gazebo runs of the same script completed one stand; the regression is not isolated yet.
- Open work and exit tests: [docs/ROADMAP.md](docs/ROADMAP.md); the day-by-day handoff: [docs/dev_handoff.md](docs/dev_handoff.md).

```
sensors -> perception -> self filter -> scene -> surface -> coverage (tool-aware) -> base stands
        -> navigation / docking -> motion planning -> execution (JTAC + F/T) -> monitoring
```

## Packages

Under `src/`, by area (layout rules: [docs/package_layout.md](docs/package_layout.md); every package has a README, and [docs/modules.md](docs/modules.md) maps them all: tier, what each needs, dependency layers, recipes for reusing a part).

| Area | Packages | What they do |
|---|---|---|
| Common | `core_types`, `ros_adapters`, `force_core` | ROS-free types and ports, conversions to ROS messages, the force-control core (contact state machine, wrench conditioning) |
| Description | `mobile_manipulator_description`, `mobile_manipulator_moveit_config`, `mobile_manipulator_profile` | Generic robot model (base + lift + N-DOF arm + tool, all swappable) with 13 profiles, MoveIt 2 config generated from a profile, header-only profile loader |
| Perception | `scene_model`, `scene_adapters`, `robot_self_filter`, `wall_detector`, `coverage_planner`, `coverage_planner_msgs` | Scene model (clouds, meshes, voxels, ESDF, loaders), scene to planner geometry, robot self filter (library and node), wall detection from a live point cloud, wall coverage planning (boustrophedon, sweepline, wall patterns, scan to plan) |
| Manipulation | `motion_planner`, `motion_planner_moveit`, `vamp_planner`, `curobo_bridge`, `curobo_bridge_msgs`, `curobo_planner`, `reachability`, `reachability_moveit`, `reachability_msgs`, `manipulation_manager`, `coverage_task_msgs`, `stand_navigator`, `stand_navigator_msgs`, `manipulation_bt_prototype` | Planner, IK, collision and scene ports; MoveIt/OMPL, VAMP and cuRobo v2 backends; RM4D reachability map and multi-stand planning; base navigation and wall docking to a stand; the manager that runs coverage, planning and execution |
| Control | `compliant_trajectory_controller`, `ft_bridge_hardware`, `ft_conditioner` | JTAC (admittance joint trajectory controller for `ros2_control`), F/T sensor bridge, wrench conditioning node |
| Simulation | `sim_mujoco`, `sim_gazebo`, `sim_isaac` | MuJoCo 4 and Gazebo Harmonic backends behind `hardware:=mujoco` / `hardware:=gazebo`; `sim_isaac` is a stub |

## Build and test (Docker)

The supported environment is the Docker image (ROS 2 Jazzy). From the repo root:

```bash
docker compose -f docker/compose.yaml build dev
docker compose -f docker/compose.yaml run --rm test     # colcon build + colcon test, no GPU
docker compose -f docker/compose.yaml run --rm dev      # interactive shell, repo mounted at /ws
```

Inside a shell: `docker/build.sh test` builds the workspace and prints the test summary. GPU targets
(`gpu`, `full`: cuRobo v2, VAMP) and the simulator images are described in [docs/docker.md](docs/docker.md).
Building outside Docker needs ROS 2, MoveIt 2, `ros2_control` and the packages in `docker/apt-extra.txt`.
The last full run recorded in the roadmap: 736 colcon tests in 26 packages, no failures. There is
CI runs the build, the colcon tests and the repository checks on free GitHub-hosted runners
([.github/workflows/ci.yml](.github/workflows/ci.yml)). It does not run simulators, GPU planners or hardware; those
are run locally, and [CONTRIBUTING.md](CONTRIBUTING.md) lists every check.

## Showcase

A full MuJoCo manager pass on a plain wall (`run_manager_pass.sh`: coverage plan, dock, press, three stroke columns, retract),
rendered headless from the recorded joint states with `render_run.py`. The gif is the run at 9x, the
[mp4](docs/media/manager_pass.mp4) at 3x with a force trace.

![A full manager pass on a plain wall](docs/media/manager_pass.gif)

| Robot overview | Arm at the wall, mid-stroke | Tool on the wall |
|---|---|---|
| ![overview](docs/media/overview.jpg) | ![arm at the wall](docs/media/arm_at_wall.jpg) | ![tool close-up](docs/media/tool_closeup.jpg) |

![force and coverage of the run](docs/media/force_coverage.png)

A non-contact pass, filmed the same way (`run_spray_pass.sh`: a spray gun held 0.30 m from the wall, valve open only during the strokes;
the orange cone and the spots on the wall are drawn by `render_run.py --valve-log`, the sim itself has no spray). The gif is at 9x, the
[mp4](docs/media/spray_pass.mp4) at 3x. Scored (`score_run --no-contact`, spray cone half-angle 20 deg): 3 columns done, coverage 100 %
(930 of 930 cells), thickness CoV 0.52, valve open 15.6 s of 94.2 s, whole-run peak force 0.0 N.

![A spray pass on a plain wall](docs/media/spray_pass.gif)

Scored result of the contact run above (`score_run`, 10 N target, 1.5 N band): 3 columns done, 0 partial, 0 failed; coverage
93.1 % (866 of 930 cells); peak force 19.2 N; RMS force error 3.6 N; 49 % of the contact samples in the band; 144 s of
simulated time, 83 s engaged. While stroking, the force ripples between 0 and about 18 N around the 10 N target. Nothing is
tuned for the picture. The pass was run three times for filming: 93.1 % / 18.5 N / 3.6 N, 93.1 % / 19.2 N / 3.6 N (shown), and
a first run with 93.3 % / 153.4 N / 8.0 N, which was made while the host was heavily loaded by other simulations and builds
(load average 10 to 27); the cause was not isolated, but the two reruns on a quieter host agree with each other and with the standard run. How the media is made:
[docs/sim_mujoco.md](docs/sim_mujoco.md#rendering-photos-and-videos). The same non-contact pass in Gazebo (`sim_gazebo/scripts/run_spray_pass.sh`, colour camera `rgb_camera:=true`, UR meshes `arm_meshes:=true`,
stills from `capture_still.py`): the arm mid-stroke at the wall, and during the seeded first move. These stills come from a run with the arm meshes
loaded, which also gives the arm collision geometry; that run scored 0.58 coverage (the plain runs without meshes: 1.0). Details and the numbers:
[docs/sim_gazebo.md](docs/sim_gazebo.md#spray-pass-and-multi-stand-pass-parity-with-mujoco).

| Gazebo: first move to the stroke | Gazebo: mid-stroke |
|---|---|
| ![gazebo seed](docs/media/gazebo_spray_seed.png) | ![gazebo stroke](docs/media/gazebo_spray_stroke.png) |

### cuRobo in a two-stand pass (MuJoCo)

`run_multi_stand_pass.sh` on `two_panels_plain` (a free base, two stands, ten columns), with cuRobo v0.9.0 selected as the free-space planner
(`planning_curobo.yaml`, MoveIt/OMPL as the fallback) and the wall pushed to the cuRobo node at every stand. The video is the run at 12x
(1128 s of simulated time), with the contact force trace and a close-up of the tool; [mp4](docs/media/multi_stand_curobo.mp4).

![A two-stand MuJoCo pass with cuRobo as the free-space planner](docs/media/multi_stand_curobo.gif)

Scored result of that run: 10 columns done, 0 partial, 0 failed; coverage 100 % (2790 of 2790 cells); peak force 69.3 N (target 10 N);
RMS force error 5.1 N; 33 % of the contact samples in the 1.5 N band; one collision-monitor line (a lift link pair at depth 0 in the first
10 ms). Planner use in the run: cuRobo planned 7 segments (the priming move and 6 stand-0 approaches: 10 to 16 s of planning each, 0.4 to
0.7 s of motion), MoveIt/OMPL planned the other 6 free-space segments (including the stow and the second stand), 4 to 50 s of motion each, because
cuRobo's service did not answer for the stow (`unavailable`). Three earlier runs with the same setup also passed (peaks 46.8, 48.2, 72.4 N).
Before those, three runs failed or stalled and each led to a fix (a 256-box cache that turned the wall into 20 cm slabs, a missing waypoint
count that made every cuRobo plan look partial, the dock-to-stow move cuRobo cannot plan); the story and the commands:
[docs/sim_mujoco.md](docs/sim_mujoco.md#running-a-pass-with-curobo-as-the-planner). The cuRobo moves are about 14 times faster than the
MoveIt-retimed ones in the sim, far above what the MoveIt retiming allows, and have not been checked against the real arm's limits: do not run them on hardware as they are.

### Obstacle scenarios (MuJoCo)

`run_scenarios.sh` runs named walls through the manager pass (UR10e, each scenario twice). One filmed run per scenario is shown; the
failing one is included on purpose. Full results, load averages and the failure analysis:
[docs/sim_mujoco.md](docs/sim_mujoco.md#scenario-set-obstacles-on-the-wall).

| scenario | runs passed | filmed run | film | force and coverage |
|---|---|---|---|---|
| plain | 2 of 2 | pass, coverage 1.000, peak 21.4 N | ![plain](docs/media/scenario_plain.gif) | [plot](docs/media/scenario_plain.png) |
| plain_wide | 2 of 2 | pass, coverage 1.000, peak 19.9 N | ![plain_wide](docs/media/scenario_plain_wide.gif) | [plot](docs/media/scenario_plain_wide.png) |
| window (hole in the wall) | 2 of 2 | pass, reachable coverage 1.000, peak 22.3 N | ![window](docs/media/scenario_window.gif) | [plot](docs/media/scenario_window.png) |
| protrusion | 2 of 2 | pass, reachable coverage 0.983, peak 20.6 N | ![protrusion](docs/media/scenario_protrusion.gif) | [plot](docs/media/scenario_protrusion.png) |
| two_panels (two stands, a rib) | 0 of 2 | **fail**, reachable coverage 0.670, peak 128.3 N | ![two_panels](docs/media/scenario_two_panels.gif) | [plot](docs/media/scenario_two_panels.png) |
| two_panels_plain (two stands, no obstacle) | 1 of 2 | pass, coverage 1.000, one 451 N press transient | ![two_panels_plain](docs/media/scenario_two_panels_plain.gif) | [plot](docs/media/scenario_two_panels_plain.png) |

The single-stand scenarios pass, with peaks near 20 N against a 10 N target. The two-stand ones fail about every second run at the second stand,
including on a plain wall, so it is not the obstacle: the approach to the first columns is refused six times in a row by collision truncation
and recovery finds no safe configuration. The cause is not isolated (the arm's pose at the second stand is the suspect). Several host loads were
10 to 14 during these runs, recorded per run in the docs. No hardware was involved.

**Update, 2026-11-20 (same-session batches of 8, MuJoCo):** after the second-stand fix `two_panels_plain` passed 8 of 8 (6 of 8 on the code before it) and `two_panels` 8 of 8 on
both code states, scored on the reachable wall. The `two_panels` row above (0 of 2, coverage 0.670) was measured before the reachability and keep-out changes that
are now in; the 0.88 wall coverage the pass script prints for `two_panels` is the whole-wall figure, not the reachable-wall one the scenario judges. Details:
[docs/sim_mujoco.md](docs/sim_mujoco.md#scenario-set-obstacles-on-the-wall).

## Simulators

Both run headless and sit behind `ros2_control`, so the same controllers run against them as against
hardware ([docs/simulation.md](docs/simulation.md)).

| | MuJoCo 4 (`sim_mujoco`) | Gazebo Harmonic (`sim_gazebo`) |
|---|---|---|
| Docs | [docs/sim_mujoco.md](docs/sim_mujoco.md) | [docs/sim_gazebo.md](docs/sim_gazebo.md) |
| Environment | `build` image with the MuJoCo SDK mounted (`MUJOCO_ROOT`); Python tools also in the `sim` image | `sim-gazebo` image |
| Can do today | Position and JTAC control of lift + arm, wall worlds from `wall_scene.json`, raw wrist F/T plus ground-truth contact wrench, conditioned force path, a 10 N contact test that holds (10.13 N for a 10 N target) | Same joint control and wall worlds, raw F/T plus ground truth, conditioned path validated against ground truth, holds 10 N with a compliant tip, free base, depth and colour camera, spray and multi-stand passes |
| Cannot do today | Move the base, simulate sensors | Wheel physics, odom noise; the contact is compliant only through a 20 mm tool tip spring |

Isaac Sim is a stub.

## Quickstart: a simulated wall pass

Needs a built workspace (see above) with a sourced ROS environment; the MuJoCo scripts also need the MuJoCo SDK
(`MUJOCO_ROOT`, [docs/sim_mujoco.md](docs/sim_mujoco.md)), the Gazebo one needs the `sim-gazebo` image
([docs/docker.md](docs/docker.md)). Each script launches the sim headless, runs the pass, records it and scores it.

```bash
# MuJoCo: scripted wall strokes planned by MoveIt
bash $(ros2 pkg prefix sim_mujoco)/lib/sim_mujoco/run_wall_pass.sh
# MuJoCo: coverage planner -> manipulation_manager -> MoveIt -> JTAC
bash $(ros2 pkg prefix sim_mujoco)/lib/sim_mujoco/run_manager_pass.sh
# Gazebo Harmonic: the same manager pass (about 5 minutes)
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_manager_pass.sh
# Gazebo: non-contact spray pass, and the multi-stand pass (free base, two stands)
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_spray_pass.sh
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_multi_stand_pass.sh
```

Sources: `src/simulation/sim_mujoco/scripts/`, `src/simulation/sim_gazebo/scripts/`.

## What you can run today

- MoveIt demo on the description with mock hardware: `ros2 launch mobile_manipulator_moveit_config demo.launch.py arm:=ur10e`
- Wall coverage planning from a scene file: `ros2 launch coverage_planner wall_coverage_demo.launch.py`
- Reachability map building and stand planning (`rm4d_tool`, `reachability_node`): [docs/reachability.md](docs/reachability.md)
- The simulated wall passes above, and the smoke and contact tests of each simulator
- A two-stand pass with cuRobo as the free-space planner: [docs/sim_mujoco.md](docs/sim_mujoco.md#running-a-pass-with-curobo-as-the-planner)

## Documentation

[docs/README.md](docs/README.md) is the index: [architecture](docs/architecture.md), [robot description](docs/robot_description.md),
[design principles](docs/design_principles.md), [decision records](docs/adr/). Third-party software and licences:
[THIRD_PARTY.md](THIRD_PARTY.md). Changes: [CHANGELOG.md](CHANGELOG.md). Contributing: [CONTRIBUTING.md](CONTRIBUTING.md).

## License

Apache License 2.0. See [LICENSE](LICENSE).
