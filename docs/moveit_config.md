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
- Seed-preferring IK solver.
