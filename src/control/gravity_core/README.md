---
type: Reference
title: gravity_core
description: Package README for the ROS-free joint-space gravity compensation core -- the torque each lift and arm joint needs to hold a pose, from a URDF, with a swappable tool payload.
tags: [gravity-compensation, dynamics, ros-free-core, package-readme]
status: experimental
---

# gravity_core

## Purpose

Gives `tau_g(q)`: the torque (N m, or N for the lift) each joint has to supply to hold a pose still. Feed it
forward next to a position or admittance loop and the loop only has to track, not carry the arm.

Not the same thing as the `gravity_compensation` block in the JTAC yaml. That one removes the tool's weight from
the F/T wrench (see [ft_sensing.md](../../../docs/ft_sensing.md)). This one is about the joints.

## Tier

ROS-free core ([package_layout.md](../../../docs/package_layout.md)). Eigen and tinyxml2 only. Builds and tests
with ROS not sourced.

## What it does

- Reads masses and centres of mass from a URDF. Inertia tensors are ignored, a still pose does not need them.
- Handles revolute, continuous, prismatic and fixed joints, and `<mimic>` (the lift stages): a mimic joint's
  torque is folded into the joint it follows.
- `setPayload(link, mass, com)` adds one point mass and replaces the last one, so a tool change or an
  `identifyPayload` result can be pushed in at runtime.
- `setGravity(g)` takes gravity in the root frame, for a tilted base.
- `compute(q, tau)` is O(joints), no allocation once `tau` is sized.

```cpp
auto model = gravity_core::loadUrdf(urdf_text, {"lift_joint", "arm_shoulder_pan_joint", /* ... */});
model.setPayload("tool_mount_link", 0.8, {0, 0, 0.0075});
Eigen::VectorXd tau;
model.compute(q, tau);   // q in the order of the names above
```

Sign: positive `tau` pushes the joint in its positive direction. It equals MuJoCo's `qfrc_bias` at zero velocity.

## Tests

```bash
docker/tools/run_host_core_tests.sh -k gravity            # 8 gtests, hand-worked pendulum, lift, mimic, payload
python3 src/control/gravity_core/tools/crosscheck_mujoco.py --urdf robot.urdf \
    --joints lift_joint,arm_shoulder_pan_joint,... --eval <build>/gravity_eval   # needs `pip install mujoco`
```

The cross-check loads the URDF into MuJoCo with geometry stripped and compares `qfrc_bias` against this core at
random poses inside the joint limits. Results are in [docs/gravity_compensation.md](../../../docs/gravity_compensation.md).

## Status

Nothing calls it yet. It is not in JTAC or in the MuJoCo plugin, and it has not run on hardware.
