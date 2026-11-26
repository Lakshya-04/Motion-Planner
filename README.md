# Motion Planner

ROS 2 packages for covering a surface with a tool using a mobile manipulator: sensor data to coverage
plan, base stands, arm motion planning and execution on a compliant (admittance) controller with force
feedback. The target is a stack that works for any arm, lift and base (chosen by a robot profile) and any
tool (contact like a mop, non-contact like a spray gun), in simulation first and on hardware by parameters.

**Status in one line:** the building blocks exist and are tested, and both simulators now run the pipeline
end to end in headless one-command passes (coverage planner, manager, MoveIt, compliant controller, recorded
and scored), including multi-stand passes where the base drives between stands. Nothing here has run on hardware.
The per-area checklist, with evidence for every item, is [docs/ROADMAP.md](docs/ROADMAP.md).

**Where it stands (2026-11-27):**
- **Wall passes run end to end in both simulators** (coverage planner, manager, MoveIt/OMPL, compliant controller, recorded and scored).
  By default each seam is covered once (`serpentine_columns`) and neighbouring strips are joined by a connector that stays in contact
  (`contact_connectors`, `chain_columns`). Results, with failures, are under [Latest runs](#latest-runs): 10 of 10 columns in a tall-wall
  MuJoCo run and a one-wall Gazebo run, connectors used 8 of 8 times.
- **Not good yet:** contact force. The MuJoCo tall-wall run peaked at 307 N (the previous column mode: 21 N at the same speed) with
  repeated "Cartesian offset diverged" resets in the compliant controller, and the Gazebo run has single-sample peaks in the kN range
  while the manager's filtered feedback stays near 100 N. Both are under investigation, one run each so far.
- **Free-space planning:** OMPL by default; cuRobo and VAMP are wired as opt-in backends. cuRobo completed two-stand MuJoCo passes but
  needs 12 to 16 s per plan on the 4 GB laptop GPU and has no low-lift stow plan ([docs/curobo_v2_notes.md](docs/curobo_v2_notes.md));
  the cuRobo-vs-MoveIt table has not been produced.
- **Opt-in layers, mostly host-tested only (no simulator, GPU or hardware run unless stated):** simulated lidar, odometry noise, IMU and an
  EKF, Nav2 adapter, base drive modes, platform sensor suites, a safety-chain core (**not a certified safety function**), a Cartesian
  tracking controller, a ROS coverage feed, per-joint costs, a differential-IK core, coverage paint and RViz plan overlays. The list with
  evidence is [CHANGELOG.md](CHANGELOG.md) and [docs/ROADMAP.md](docs/ROADMAP.md).
- **Navigation and perception are the weakest links:** the base drives a straight line to each stand on simulator odometry; wall docking,
  Nav2 and perception-in-the-loop exist but are not part of the standard passes.
- **Nothing has run on hardware.** What is untested there, in commissioning order: [docs/hardware_readiness.md](docs/hardware_readiness.md).
- Open work and exit tests: [docs/ROADMAP.md](docs/ROADMAP.md); the handoff: [docs/dev_handoff.md](docs/dev_handoff.md); the order of
  the next themes: [docs/development_plan.md](docs/development_plan.md).

```
sensors -> perception -> self filter -> scene -> surface -> coverage (tool-aware) -> base stands
        -> navigation / docking -> motion planning -> execution (JTAC + F/T) -> monitoring
```

## Packages

Under `src/`, by area (layout rules: [docs/package_layout.md](docs/package_layout.md); every package has a README, and [docs/modules.md](docs/modules.md) maps them all: tier, what each needs, dependency layers, recipes for reusing a part).

| Area | Packages | What they do |
|---|---|---|
| Common | `core_types`, `ros_adapters`, `force_core`, `tool_calibration`, `standoff_advisor` | ROS-free types and ports, conversions to ROS messages, the force-control core (contact state machine, wrench conditioning), the tool calibration record, the base-to-wall standoff advisor |
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

## Latest runs

Two-stand wall passes (`run_multi_stand_pass.sh`) on current main with the defaults of #292: `serpentine_columns`, `chain_columns` and
`contact_connectors` on (one pass per strip, strips joined by connectors that stay in contact), `contact_wiper` one pass, tool speed
0.01 m/s. Recorded 2026-11-26/27, one run each, not repeated, so no spread is claimed. Peak and p99 are over every `run.csv` force sample
(the scorer's `score_run`); the speed is the median over contact windows (force above 3 N, chord over at least 0.5 s, target 0.01 m/s,
[docs/tcp_speed.md](docs/tcp_speed.md)). Only the contact strokes are held to a speed.

| Run | Simulator | Columns done | Connectors used / fell back | Peak force | p99 force | Duration | TCP speed median | Note |
|---|---|---|---|---|---|---|---|---|
| `mujoco-final-tall` | MuJoCo | 10 of 10 (0 partial, 0 failed) | 8 / 0 | 307.3 N | 19.8 N | 1752 s | 0.00806 m/s (-19 %) | `two_panels_tall`, UR10e telescopic profile, lift cost 2x; 227 samples above 60 N, 55 "Cartesian offset diverged" resets |
| `gazebo-final-onewall` | Gazebo | 10 of 10 | 8 / 0 | 8367 N | 173.7 N | 515 s | 0.01006 m/s (+0.6 %) | plain wall, joint position gain 2; single-sample spikes, 460 samples above 500 N |
| `mujoco-connectors2` (earlier) | MuJoCo | 10 of 10 | 8 / 0 | 293.2 N | 19.6 N | 1898 s | 0.00969 m/s (-3.1 %) | same idea, earlier build |
| `mujoco-main-liftcost` (earlier) | MuJoCo | 10 of 10 | none (old non-serpentine column mode) | 21.1 N | 14.3 N | 1774 s | 0.00942 m/s (-5.8 %) | same speed as the tall run, for comparison |

![MuJoCo tall wall run at 20x](docs/media/latest_mujoco_tall.gif)

The MuJoCo tall-wall run above, drawn from the recorded joint states with `render_run.py` at 20x (1752 s of simulated time in 88 s of video),
with the contact force trace and a close-up of the tool (the gif is a smaller copy, 560 px wide at 6 fps; [mp4](docs/media/latest_mujoco_tall.mp4)). How the media is made:
[docs/sim_mujoco.md](docs/sim_mujoco.md#rendering-photos-and-videos). There is no video of the Gazebo run (no film was recorded for it).

What is not good yet:

- **Force spikes in serpentine mode.** The tall-wall run peaked at 307 N (the old column mode peaked at 21.1 N at the same speed), mostly
  mid-stroke, with 55 "Cartesian offset diverged" resets in the simulator log. The cause is open.
- **Gazebo contact quality.** `run.csv` peaks at 8367 N in single samples (p99 174 N), while the manager's own filtered feedback peak is much
  lower; the two disagree and this is under investigation. Columns finish, but the force is not under control there.
- **Nothing on hardware.** All of this is simulation. The TCP-speed median is below the target in the MuJoCo runs, and the mean sits well below the median
  (windows near standstill at column ends, turns and force dips), so the speed is kept in the median, not at every sample
  ([docs/tcp_speed.md](docs/tcp_speed.md)).

More: [docs/sim_mujoco.md](docs/sim_mujoco.md), [docs/sim_gazebo.md](docs/sim_gazebo.md), [docs/how_to_read_a_run.md](docs/how_to_read_a_run.md),
[docs/manager_parameters.md](docs/manager_parameters.md#chained-columns-chain_columnstrue).

## Simulators

Both run headless and sit behind `ros2_control`, so the same controllers run against them as against
hardware ([docs/simulation.md](docs/simulation.md)).

| | MuJoCo 4 (`sim_mujoco`) | Gazebo Harmonic (`sim_gazebo`) |
|---|---|---|
| Docs | [docs/sim_mujoco.md](docs/sim_mujoco.md) | [docs/sim_gazebo.md](docs/sim_gazebo.md) |
| Environment | `build` image with the MuJoCo SDK mounted (`MUJOCO_ROOT`); Python tools also in the `sim` image | `sim-gazebo` image |
| Can do today | Position and JTAC control of lift + arm, wall worlds from `wall_scene.json`, raw wrist F/T plus ground-truth contact wrench, conditioned force path, a 10 N contact test that holds (10.13 N for a 10 N target) | Same joint control and wall worlds, raw F/T plus ground truth, conditioned path validated against ground truth, holds 10 N with a compliant tip, free base, depth and colour camera, spray and multi-stand passes |
| Cannot do today | Wheel, floor or slip physics; odometry is the simulated pose, no noise; the depth sensor is a ray cast, not a rendered image | Wheel physics, odom noise; the contact is compliant only through a 20 mm tool tip spring |

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
- The simulated wall passes above, and the smoke and contact tests of each simulator. Base navigation is a straight-line stand-in on ground-truth odometry, and the two-stand passes use a pre-baked wall rather than live perception: [docs/navigation_and_perception.md](docs/navigation_and_perception.md) says what is real and what is not
- A two-stand pass with cuRobo as the free-space planner: [docs/sim_mujoco.md](docs/sim_mujoco.md#running-a-pass-with-curobo-as-the-planner)

## Documentation

[docs/README.md](docs/README.md) is the index: [architecture](docs/architecture.md), [robot description](docs/robot_description.md),
[design principles](docs/design_principles.md), [decision records](docs/adr/), [TCP speed: what is held constant](docs/tcp_speed.md), [how to read a run](docs/how_to_read_a_run.md) (the files a pass writes and what each field means), [RViz profiles per module](docs/rviz/README.md). Third-party software and licences:
[THIRD_PARTY.md](THIRD_PARTY.md). Changes: [CHANGELOG.md](CHANGELOG.md). Contributing: [CONTRIBUTING.md](CONTRIBUTING.md).

## License

Apache License 2.0. See [LICENSE](LICENSE).
