# MoveIt configuration (`mobile_manipulator_moveit_config`)

MoveIt 2 setup for [`mobile_manipulator_description`](robot_description.md): planning groups,
kinematics, OMPL, controllers and a mock-hardware demo, for the UR family and the generic arm
through one configuration.

## Groups and controller

| Group | Joints | Use |
|---|---|---|
| `lift_arm` | `lift_joint` + the six arm joints (7-DOF) | whole-body planning, lift and arm on one clock |
| `arm` | the six arm joints | in-band work with the lift locked |

One `FollowJointTrajectory` controller (`lift_arm_controller`) drives all seven joints. Groups
are chains, so they work for any arm; joint names come from `arm_joint_names.xacro` and the
per-family controller files (`config/ur/`, `config/generic_6dof/`).

## Run

```bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py arm:=ur10e          # with RViz
ros2 launch mobile_manipulator_moveit_config demo.launch.py arm:=generic_6dof rviz:=false
```

Starts robot_state_publisher, ros2_control with mock hardware (home pose), the joint state
broadcaster, the trajectory controller and `move_group`. Other launch files that need the same
parameters use `mobile_manipulator_moveit_config.build_moveit_config(arm)`.

## Settings worth knowing

- **Velocity and acceleration scaling are explicit** (`joint_limits.yaml`, 0.5): MoveIt's
  default of 0.1 silently runs every plan at 10% speed.
- **Kinematics:** KDL for both groups. It works, but it is a numerical solver: with a wide joint
  range (for example +-2*pi) it can return a full-turn-equivalent solution far from the seed. The
  reference arm therefore limits its joints to +-pi, and the UR family stays near the seed in
  tests. For real hardware with +-2*pi joints use a seed-preferring solver (pick_ik, TRAC-IK,
  or an analytic UR solver); neither is installed in the image yet.
- **Lift limit:** 0.2 m/s, matching the starter profile; set your lift's real value.

## Allowed-collision matrix

Generated from the URDF by `mobile_manipulator_description/scripts/generate_acm.py`:
adjacent collision links (also through empty frame links) and all lift parts among themselves.
Committed per arm family, and a test fails if they go stale. The first MoveIt run is why this
exists: with no matrix MoveIt reported the home pose itself as in collision (seven structural
pairs), so IK with collision checking failed even for a zero-distance target.

`scripts/sample_collisions.py` samples random states against a running `move_group` and lists the
link pairs that collide almost always (candidates for the matrix) and the share of valid states.
For the generic arm: 61% of random 7-DOF states are valid and no pair is almost always in
collision.

## Verification

`scripts/smoke_plan.py` (needs the demo running): for both groups it reads the real current
joint state, finds a nearby collision-free pose (small offsets in turn), solves IK with the state
as the seed, requires the solution to be close to the current state, and plans to it through
`move_group`. Passes for `ur10e` and `generic_6dof` on Jazzy in the container (IK moves at most
0.04 rad; plans of 6 to 10 waypoints). Run in the container:

```bash
docker run --rm -v $PWD/src:/ws/src:ro -v mp_build:/ws/build -v mp_install:/ws/install \
  -w /ws motion-planner:deps bash -lc '
  source /opt/ros/jazzy/setup.bash && colcon build --packages-select \
    mobile_manipulator_description mobile_manipulator_moveit_config &&
  source install/setup.bash &&
  ros2 launch mobile_manipulator_moveit_config demo.launch.py arm:=ur10e rviz:=false &
  sleep 25; python3 install/mobile_manipulator_moveit_config/lib/mobile_manipulator_moveit_config/smoke_plan.py'
```

## Not done

- A JTAC (compliant controller) variant of the controller config: `coverage_pass_demo_node` drives
  the compliant controller's force topics, so it needs that controller under `controller_name`.
  Its names and the base-to-footprint offset are parameters now.
- Pairs that are never or always in collision for one arm's geometry beyond the sampled check.

## Compliant controller (JTAC) variant

`demo.launch.py controller:=jtac` swaps the plain joint trajectory controller for the compliant
`JtacController` (admittance), under the same name `lift_arm_controller`, so MoveIt's controller
config and the wall-pass demo (`controller_name`, the `…/target_force_n` topic) work unchanged.

- `hardware:=jtac_mock` (description): the same seven joints with position **and** velocity command
  interfaces, plus a force/torque sensor named `ft_sensor` from `ft_bridge_hardware` in injection
  mode, so a wall contact can be scripted without a real sensor.
- `config/<family>/jtac_controllers.yaml`: controller parameters with our joint names and frames
  (`base_footprint` -> `tcp_link`, F/T and control frame `tcp_link`), all admittance axes deselected
  until a segment engages them. `robot_description` is injected through the spawner's `--param-file`
  because the controller reads it on its own node.
- Verified on Jazzy for both `ur10e` and `generic_6dof`: the controller goes active, and
  `scripts/smoke_plan.py --execute` plans with `lift_arm` through MoveIt, executes through the
  controller, and checks the joints arrived. The 6-joint `arm` group stays plan-only: the single
  controller takes all seven joints (no partial goals), same as the plain trajectory controller.

## IK solvers: `ik:=kdl | trac_ik | pick_ik`

`demo.launch.py ik:=...` / `build_moveit_config(arm, ik=...)` choose the kinematics plugin
(`config/kinematics*.yaml`). Measured with `scripts/smoke_plan.py --stress --spread=R` on Jazzy: 20
poses taken from random configurations within R rad of the current state, IK started from that state;
"excess" is how much further the solution lands from the seed than the configuration the pose came from,
"jump" is an excess over 0.5 rad.

| UR10e, R = 1.2 rad | group | solved | mean excess | jumps | worst move |
|---|---|---|---|---|---|
| KDL | lift_arm (7) | 20/20 | 0.60 rad | 4 | 6.4 rad (wrapped) |
| KDL | arm (6) | 20/20 | 0.12 rad | 2 | 3.1 rad |
| TRAC-IK `solve_type: Distance` | lift_arm | 20/20 | 0.13 rad | 2 | 2.3 rad |
| TRAC-IK `solve_type: Distance` | arm | 20/20 | 0.007 rad | 0 | 1.3 rad |
| pick_ik (default and a longer timeout) | lift_arm / arm | 2/20, 0/20 | | | |

The generic 6-DOF arm gives the same picture (KDL excess 0.65 / 0.29 with 4 / 2 jumps; TRAC-IK 0.15 /
0.00 with 1 / 0). For targets within 0.4 rad of the seed all three behave (KDL and TRAC-IK 20/20, no
excess), so the difference only shows on larger moves, where KDL can return a far or wrapped branch.

- **Default: `trac_ik` with `Distance`** (it prefers the seed's branch). `kdl` ships with MoveIt and
  stays available as `ik:=kdl`. The plugin is an `exec_depend`, so the Docker deps layer installs it.
- `pick_ik` is kept as an opt-in config but performed poorly here (tolerances / scales untuned for
  these poses); do not use it without tuning.
- **Seed preferring (`SeedPreferringIk`, parameter `ik_extra_seeds`, off by default):** N extra solves from
  seeds perturbed by up to 0.3 rad, wrapped by whole turns toward the real seed, closest kept. With
  TRAC-IK `Distance`, R = 1.2 rad, `--seeds=8` (same algorithm, applied in `smoke_plan.py`): `lift_arm`
  mean excess 0.132 -> 0.069 rad, worst move 2.26 -> 1.70 rad, jumps 2 -> 2 (of 20); `arm` 0.007 -> 0.000,
  0 jumps. It trims the redundant group's drift but does not remove the two branch jumps, and each
  extra seed costs one more solve.
- Switch it on from the launch line: `ros2 launch motion_planner_moveit coverage_pass_demo.launch.py ik_seeds:=8`
  (`ik_seed_perturbation:=0.3` sets the noise in rad). The parameters are written for a wildcard node name, so
  they reach the backend's own `motion_planner_backend` node.
- "Closest" is a weighted distance: prismatic joints (the lift) weigh `ik_prismatic_factor`^2 (default 6,
  so 1 m of lift counts as 6 rad, from the 0.3 rad / 0.05 m jump thresholds in `design_principles.md`
  section 4) and revolute joints weigh 1. The same knob exists on the cuRobo node (`prismatic_joints`,
  `ik_prismatic_factor`, `ik_weights`).
- The 7-DOF `lift_arm` group is redundant, so even TRAC-IK still jumps branch on a couple of poses;
  `SeedPreferringIk` (above) is the solver-agnostic wrapper that trims this.

### Prior art: a modified TRAC-IK from an earlier production stack

An earlier stack carried a TRAC-IK fork (ROS 2, installed into its image). What it changed, as ideas to
re-implement on top of the unmodified upstream solver, not to copy:

- a **continuous mode** on `CartToJnt` (stay on the seed's branch) vs the stock "any solution" mode;
- **several distinct solutions** per query (a configurable maximum, deduplicated), not just the first;
- **prioritised seeds**: noise-perturbed variants of the caller's seed (a configurable count and
  amplitude), tried best first, written for 6 joints;
- a **prefilter** that drops seeds whose forward kinematics are already far from the target, backed by
  an **FK cache** keyed by joint configuration;
- every solution **re-validated by forward kinematics** against position and orientation tolerances;
- joint angles **wrapped / normalised** against the seed.
The benchmark that came with it compared continuous against discontinuous mode over 1000 solves
(solve time, success rate, CSV per mode). Measuring both on our own poses (above) already shows why
seed continuity matters.
