---
type: Design
title: Hybrid simulation (Gazebo for the world, MuJoCo for the arm)
description: Design note, not built. Gazebo runs the base, sensors and navigation; MuJoCo runs the arm, tool contact and force control. Says when it is worth doing, how the two share state, and the smallest spike that would tell.
tags: [simulation, gazebo, mujoco, navigation, perception, design]
status: proposed
---

# Hybrid simulation

Nothing here is built. This is the plan, so the decision has a page to point at.

## Why look at it

- Gazebo has the sensors and base the navigation and perception work needs: `depth_camera`, an RGB camera,
  `VelocityControl`, `OdometryPublisher` ([sim_gazebo.md](sim_gazebo.md)). The MuJoCo depth sensor is a ray cast on
  scene geoms only ([navigation_and_perception.md](navigation_and_perception.md)).
- MuJoCo has the contact the force work needs. Gazebo's wall contact is rigid: about 106 N past the wall whatever
  the depth, and impacts of 1.2 to 1.8 kN in a manager pass ([sim_gazebo.md](sim_gazebo.md)).

Three launch modes would sit behind the existing `hardware:=` switch: `mujoco`, `gazebo`, `hybrid`.

## When it is worth building

Only if both hold:

1. The Gazebo-only passes are blocked by contact (peak force or settling), not by the dock IK branch jumps.
   Those jumps are a planning problem and the hybrid would not fix them.
2. A MuJoCo depth camera is not enough for the perception work. A rendered depth image in MuJoCo is a smaller job
   than bridging two simulators.

If either fails, stay on one simulator.

## Shape

| Owner | What |
|---|---|
| Gazebo | world, base motion (`/cmd_vel`), `/odom` and TF, depth and RGB sensors, later Nav2 |
| MuJoCo | arm and lift joints, tool and wall contact, F/T, JTAC |

State crossing the line:

- Base pose, Gazebo to MuJoCo. The MuJoCo base is already a stiff planar actuator (kp 2e4), so it can track the
  Gazebo `/odom` pose.
- Arm and lift joint positions, MuJoCo to Gazebo, set kinematically so the cameras see the arm move.
- The wall and obstacles come from the same scenario JSON on both sides (`make_scenario_scene.py`,
  `shift_wall_scene.py`), so there is one description to keep in step.

Clock: Gazebo steps the world, MuJoCo steps against Gazebo's `/clock`. If they drift, the arm lags the base by a
step or two, which a stand-to-stand pass tolerates because the arm is stowed while the base drives.

## Risks

- Two sources of truth for base and wall pose. Mismatch shows up as a wall gap that is not real.
- Base to wall contact only exists in Gazebo, and arm to wall contact only in MuJoCo.
- Latency in the joint mirror makes the Gazebo camera show the arm slightly behind.
- Debugging doubles: a bad run could be either simulator, or the bridge.

## Smallest spike

One stand, no navigation. Gazebo publishes `/odom`, MuJoCo's base follows it, the arm runs one wall column in
MuJoCo, joint states go back to Gazebo. Pass if the MuJoCo score matches a MuJoCo-only run of the same column and
the Gazebo camera sees the arm within 50 ms of the joint states. If that works, add a drive between two stands.
