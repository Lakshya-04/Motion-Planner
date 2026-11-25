# Contributing

Thanks for helping. This page is the working agreement; the design rules are in
[docs/design_principles.md](docs/design_principles.md) and the status of everything is in
[docs/ROADMAP.md](docs/ROADMAP.md) (pick an unchecked item, or open an issue first for anything large).
By contributing you agree your work is licensed under the Apache License 2.0 ([LICENSE](LICENSE)).

Please also read the [code of conduct](CODE_OF_CONDUCT.md); report security problems as described in the [security policy](SECURITY.md).

## Repository layout

```
src/common/        core_types, ros_adapters, force_core            ROS-free types, ports, force core
src/description/   robot description, MoveIt config, profile loader
src/perception/    scene_model, scene_adapters, robot_self_filter, coverage_planner(+_msgs)
src/manipulation/  motion_planner (+_moveit), vamp_planner, curobo_bridge(+_msgs), curobo_planner,
                   reachability(+_msgs), manipulation_manager
src/control/       compliant_trajectory_controller (JTAC), ft_bridge_hardware, ft_conditioner
src/simulation/    sim_mujoco, sim_gazebo, sim_isaac (stub)
docker/            Dockerfile, compose.yaml, build.sh, per-robot asset tools (docker/tools)
docs/              architecture, per-module notes, ROADMAP.md, decision records (docs/adr)
```

Every module splits into `core/` (plain C++17, no ROS) and `ros/` (a thin shell) with tests mirrored
under `test/core` and `test/ros`: [docs/package_layout.md](docs/package_layout.md). Keep new algorithms in
`core/` so they can be tested without ROS. Decisions that shaped the layout: [docs/adr/](docs/adr/).

## Building and testing

The supported environment is Docker ([docs/docker.md](docs/docker.md)); everything below runs from the repo root.

```bash
docker compose -f docker/compose.yaml build dev
docker compose -f docker/compose.yaml run --rm test          # docker/build.sh test: colcon build, colcon test, summary
docker compose -f docker/compose.yaml run --rm dev           # shell, repo mounted at /ws
```

Inside the shell, `docker/build.sh test` is the one build entry point. For a single package:
`colcon build --packages-select <pkg> && colcon test --packages-select <pkg> && colcon test-result --verbose`.
GPU tests (cuRobo) need the `gpu` image and skip without CUDA; simulator tests need the `sim` /
`sim-gazebo` images (and the MuJoCo SDK for the plugin) and skip what they lack.

## Checks: CI and local

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs the repository checks, the colcon build and all
colcon tests on every pull request, on standard GitHub-hosted runners only (free for a public repository). It cannot
run what needs a GPU, a display or a robot: the simulator passes, cuRobo and VAMP, hardware. Those stay local, and
anything that would cost money (larger runners, a private repository) is run locally too. Run the checks yourself
before opening a pull request. The commands are collected in
[docs/docker.md](docs/docker.md#run-all-checks); "run all checks" today is:

1. `docker compose -f docker/compose.yaml run --rm test` (all colcon tests, C++ and Python).
2. Python tool tests: `python -m pytest docker/tools/tests` (the MuJoCo cross-check tests skip without MuJoCo).
3. For changes to a simulator or the force path, the scripts the simulator docs describe:
   `ros2 run sim_gazebo smoke_test.py` and `contact_control_test.py` ([docs/sim_gazebo.md](docs/sim_gazebo.md)),
   `scripts/run_smoke.sh` and `run_contact.sh` of `sim_mujoco` ([docs/sim_mujoco.md](docs/sim_mujoco.md)).
   These are not part of `colcon test` yet (roadmap section 10).
4. `python3 docker/tools/check_public_readiness.py`, `python3 docker/tools/check_params.py` (every parameter the manager and reachability nodes declare is documented, every config key is declared) and a link check over the Markdown you touched
   (relative links must resolve). Before a release: [docs/release_checklist.md](docs/release_checklist.md). Reading the output of a simulated pass: [docs/how_to_read_a_run.md](docs/how_to_read_a_run.md).

## Code style

- C++17. Formatting follows `.clang-format` (Google-based, 100 columns, braces on their own line for
  classes, functions, namespaces and structs; run `clang-format` on what you change) and `.clang-tidy`;
  `.editorconfig` covers the rest (UTF-8, LF, final newline, 2 spaces for C++, YAML, XML, xacro; 4 spaces and
  99 columns for Python). The ament style linters (uncrustify, flake8, pep257) are switched off in
  `colcon test`, so run the formatter yourself.
- Ports are small abstract classes in `core/` (`Planner`, `IkPort`, `CollisionPort`, `ScenePort`, ...);
  backends implement them, nodes only wire parameters, topics and conversions.
- Names that vary per robot (joint, group, frame and controller names) come from the robot profile, not
  from constants in code.
- Tests: gtest for C++, pytest for Python (launch `ament_cmake_pytest`). A fix comes with a test that fails
  without it. Say in the test or the docs when a result is an estimate.
- Docs state what exists. Do not write "planned" next to something that is built or the reverse; when you
  are unsure, write "unverified".

## Adding things

| To add | Start here |
|---|---|
| An arm (6 or 7+ joints) | [docs/robot_description.md](docs/robot_description.md), "Adding an arm" and the arm table: xacro macro (primitives, published parameters, cite the source), profile YAML, ACM, MoveIt controller family; swap tests pick it up from their profile lists (`PROFILES` / `SEVEN_DOF` in `test_arm_swap.py`); a redundant arm needs a map policy ([docs/reachability.md](docs/reachability.md), "Redundant arms") |
| A base | same doc, "Swapping parts" and "Bases": a `<name>_base` macro, one branch in `urdf/base/base.xacro`, a `base:` section in the profile |
| A lift | same doc, "Lift variants": keep the three lift outputs, add one branch in `lift/lift.xacro`; recipe for a chain / column lift |
| Per-robot planner assets (spheres, cuRobo config, VAMP header, reach map) | [docs/robot_assets.md](docs/robot_assets.md): one command from the profile |
| A tool | There is no "how to add a tool" guide yet (roadmap section 4). Today a tool is a `ToolProfile` / `ToolCoverageProfile` (`core_types`, `coverage_planner/core/tool_coverage_profile.hpp`) filled from node parameters; mind the TCP offset (`tcp_xyz`, `tcp_rpy`) in the profile |
| A simulation backend | [docs/simulation.md](docs/simulation.md): the contract every backend meets (joints, clock, tool wrench, world, base), then follow [docs/sim_mujoco.md](docs/sim_mujoco.md) or [docs/sim_gazebo.md](docs/sim_gazebo.md) as a model |
| A planning, IK or collision backend | [docs/architecture.md](docs/architecture.md) for the ports; [docs/vamp_notes.md](docs/vamp_notes.md) and [docs/curobo_v2_notes.md](docs/curobo_v2_notes.md) for the two existing out-of-tree backends. Runtime backend selection (pluginlib) is not implemented yet |
| A coverage planner | `CoveragePlannerBase` and the factory in `coverage_planner`; [docs/coverage_patterns.md](docs/coverage_patterns.md) |
| Third-party software | Consume it upstream and unpatched ([docs/adr/0002-third-party-backends.md](docs/adr/0002-third-party-backends.md)); add a row with its licence to [THIRD_PARTY.md](THIRD_PARTY.md); do not vendor source |

## Commits and pull requests

The conventions follow what large open-source projects do (the Linux kernel's patch guide, Conventional Commits,
Keep a Changelog, the ROS 2 and MoveIt contribution guides).

- **Linear history.** `main` is never force-pushed and has no merge commits. Pull requests are merged with
  **squash**, so one pull request is one commit on `main` (the commit message is the pull request's title and
  description). Keep a branch current with `git rebase origin/main`, not by merging `main` into it.
- **One theme per pull request**, branched from `main`: a feature or fix together with its tests, docs and
  CHANGELOG entry. Do not open a pull request for a one-line follow-up or a documentation-only finding that
  belongs to work still in flight; add it to that branch. Review size: a pull request a reviewer can read in
  one sitting; split by theme, not by file.
- **Commit subjects use [Conventional Commits](https://www.conventionalcommits.org/):** `type(scope): subject`,
  with type one of `feat`, `fix`, `refactor`, `perf`, `test`, `docs`, `build`, `chore`; scope is the package or
  area (`reachability`, `sim_mujoco`, `curobo`, `docker`). Imperative mood, no trailing full stop, 72
  characters or fewer. A `!` after the type (or a `BREAKING CHANGE:` line in the body) marks an incompatible
  change.
- **The body says why**, not only what: the problem, the approach, what was measured (with the host load for
  timings), and what is not verified. Wrap at 72 columns. Reference issues and pull requests by number.
- Prefer commits that build and pass their own tests, so `git bisect` stays usable.
- Public history is not rewritten. The only exception so far is the one-time consolidation on 2026-11-20 (561
  commits into 15 milestones with identical content); the trees are unchanged.
- **No AI attribution** in commits or pull request text: no `Co-Authored-By` lines for tools, no
  "generated with" lines.
- **No company or earlier-project names** in code, docs, commit messages or test data. Describe a design
  by what it does; cite public papers and open-source projects by name.
- Do not commit generated artefacts (`build/`, `install/`, `log/`, reachability maps, robot assets under
  `robots/`) or secrets. `.claude/` stays out of the repository.
- Update the docs and [CHANGELOG.md](CHANGELOG.md) (under `Unreleased`, Keep a Changelog sections) with the change, and tick or edit
  the matching item in [docs/ROADMAP.md](docs/ROADMAP.md) with its evidence.
- In the pull request: what changed, how you tested it (which of the checks above), and what you did not
  verify.
