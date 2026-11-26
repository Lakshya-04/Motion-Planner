---
type: Reference
title: Joint gravity compensation
description: What joint-space gravity compensation is, why the repo needed it, what gravity_core does, and the measured evidence (cross-check against MuJoCo, sag with and without the feedforward). Says plainly what is not wired in.
tags: [gravity-compensation, mujoco, control, evidence]
status: experimental
---

# Joint gravity compensation

## What it is

Holding a pose against gravity takes a torque at each joint, `tau_g(q)`. If the controller has to produce it
through position error, the arm sags by `tau_g / kp`. A feedforward of `tau_g(q)` takes that load off the PD loop.

Two different things share the word, and the repo already had one of them:

| | Where | What it removes |
|---|---|---|
| Payload compensation | `force_core`, `ft_conditioner`, JTAC's `gravity_compensation` block | the tool's weight from the F/T wrench ([ft_sensing.md](ft_sensing.md)) |
| Joint compensation | [`gravity_core`](../src/control/gravity_core/README.md) (new) | the arm's and tool's weight from the joint torques |

## Why it matters here

MuJoCo marks arm links and the lift `gravcomp`, so the position actuators never carry the arm
([sim_mujoco.md](sim_mujoco.md)). That is free and exact, and a real arm is neither. The cases where it counts:

- torque-mode arms (Franka, iiwa, Kinova low level) if they are used through a torque interface;
- turning `gravcomp` off to see what a stiffness choice really costs;
- lower gains for softer wall contact, without paying for them in holding accuracy;
- hand guiding and teleop, where the arm should float.

UR and xArm run gravity compensation inside their own controller in position mode, so for those it changes nothing on hardware.

## Evidence

All on the UR10e profile (`ur10e_telescopic`: lift with two mimic stages, six arm joints), URDF rendered by xacro in
the `motion-planner:sim-gazebo` image. Host-side Python with MuJoCo 3.17; not run through the ROS plugin.

**1. Matches MuJoCo.** `tools/crosscheck_mujoco.py`, 1000 random poses inside the joint limits, MuJoCo's `qfrc_bias`
at zero velocity against `gravity_core`:

| Case | Worst absolute error |
|---|---|
| UR10e + lift, no payload | 5.0e-10 N m (N for the lift) |
| same with a 0.8 kg tool payload at the mount | 5.0e-10 |

The lift peaks at about 389 N, the shoulder lift at 130 N m, so this is rounding. A small test robot with rotated
joints, a skew axis, a mimic stage and a payload gives 3e-12. Eight gtests cover a hand-worked pendulum, lift,
mimic, payload and tilted gravity.

**2. What it fixes.** `tools/sag_demo.py`: MuJoCo with real gravity (no `gravcomp`), the repo's actuator law
(`kp = 10 x effort` per revolute joint, `100 x effort` for the lift, `kv = 0.1 kp`, force clamped to the limit),
implicit-fast integrator, 2 ms. Three poses, 4 s settle, tool mount displacement from the target:

| Pose | PD only | PD + `ctrl = q* + tau_g/kp` |
|---|---|---|
| 0 | 35.1 mm | 0.000 mm |
| 1 | 72.9 mm | 0.000 mm |
| 2 | 32.8 mm | 0.000 mm |
| 0 to 2, 0.8 kg tool | 36.6 to 79.7 mm | at most 0.020 mm |

Largest joint sag without it is 2.2 degrees at the shoulder. The feedforward is exact here because the model
matches the simulator's own masses; with a model error it would leave a residual of `error / kp`.

**3. In the plugin, through a manager pass.** `run_manager_pass.sh` (single stand, MoveIt/OMPL, JTAC, scored),
UR10e profile, one run per mode, `gravity_comp:=` as in [sim_mujoco.md](sim_mujoco.md):

| `gravity_comp` | Passed | Coverage | Peak force (target 10 N) | Tool tilt max | Wall gap max |
|---|---|---|---|---|---|
| `mujoco` (free `gravcomp`) | yes | 100 % | 20.5 N | 0.182 deg | 0.81 mm |
| `core` (`gravity_core`, `gravcomp` off) | yes | 100 % | 19.5 N | 0.196 deg | 0.73 mm |
| `none` (plain PD) | no | 0 % | none | none | none |

`core` matches the free-`gravcomp` pass: one run each, so the differences are noise, not a result. `none` fails at
the manager's first planning segment (OMPL aborts); the ready pose it reports sits at the sagged height. I did
not trace which check trips, so read it as "the pass does not survive the sag", not as a diagnosed cause.

**4. Against Gazebo.** `sim_gazebo/scripts/gravity_check.py` drives the UR10e profile to home plus four random
poses, waits until it settles, and compares the effort Gazebo reports for each joint with `gravity_core`:

| Joint | Gazebo effort, max | `gravity_core`, max | Worst difference |
|---|---|---|---|
| lift | 362.617 N | 362.617 N | 0.000 |
| shoulder lift | 75.330 N m | 75.330 N m | 0.000 |
| elbow | 48.418 N m | 48.418 N m | 0.000 |
| wrist 1 / 2 | 4.532 / 0.926 N m | 4.532 / 0.926 N m | 0.000 |
| shoulder pan, wrist 3 | 0 | 0 | 0 (vertical axes) |

So a second physics engine (gz-sim) carries the same gravity torques the model predicts. One accounting point: by
default `gravity_core` folds the telescopic stages' load into `lift_joint` (what a single actuator has to supply),
which gives 392.0 N here; Gazebo reports the lift joint alone, 362.6 N, and the 29.4 N gap is exactly the two
stages' share (`0.5 x 39.2 + 0.5 x 19.6`). The check lists the stages as outputs of their own to compare like with like.

## What is wired in

- MuJoCo plugin: `gravity_feedforward` hardware parameter, launch `gravity_comp:=mujoco|core|none`
  ([sim_mujoco.md](sim_mujoco.md)). Unit test `GravityFeedForwardRemovesTheSag`.
- Gazebo: validation only, no feed-forward. The arm holds its pose in Gazebo without any (the smoke test's home
  pose holds to 0.0000 rad, and the check above settles to 0.0000 at every pose), so there is nothing for a
  feed-forward to remove. I did not look into how `gz_ros2_control` achieves that.

## Not done

- Not in JTAC or any real hardware interface. Nothing here has run on hardware.
- One run per mode, single stand. No two-stand pass with `core` yet.
- A real arm's link masses and centres come from its URDF; they are as good as the vendor numbers.
- Friction is not modelled, so a stiction offset on hardware will not be compensated.
- No Gazebo effort-command mode.

## Next

1. A two-stand pass with `gravity_comp:=core`.
2. Push `identifyPayload` results into `setPayload` so a tool change updates the feedforward at runtime.
3. An effort-command interface for a torque-mode arm (the `joint_impedance_controller` item in [development_plan.md](development_plan.md) section 8).
