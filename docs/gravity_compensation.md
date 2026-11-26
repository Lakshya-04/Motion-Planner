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

## Not done

- Not in JTAC, the MuJoCo plugin or `sim_gazebo`. Nothing calls `gravity_core` yet.
- The sim run is standalone MuJoCo, not a manager pass, so no wall-contact score exists with it on or off.
- A real arm's link masses and centres come from its URDF; they are as good as the vendor numbers.
- Friction is not modelled, so a stiction offset on hardware will not be compensated.

## Next

1. In the MuJoCo plugin, call `gravity_core` and add `tau / kp` to `ctrl`, next to the velocity feedforward. Option `gravcomp:=false` to use it.
2. Run `run_multi_stand_pass.sh` three ways (free `gravcomp`, none, `gravity_core`) and compare tilt, wall gap and peak force.
3. Push `identifyPayload` results into `setPayload` so a tool change updates the feedforward.
