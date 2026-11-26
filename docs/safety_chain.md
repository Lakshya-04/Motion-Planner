---
type: Reference
title: Safety chain
description: ROS-free core that ties the per-tool abort thresholds and the other independent safety inputs to one latched OK / HOLD / ESTOP state with defined stop outputs; what it is based on and what it does not provide.
tags: [safety, e-stop, protective-stop, safety_monitor]
status: experimental
---

# Safety chain (`safety_monitor::SafetyChain`)

> **NOT A CERTIFIED SAFETY FUNCTION.** Ordinary software on a general-purpose OS. A real e-stop is a
> hardware circuit that removes power without any software running; this chain only produces the
> stop requests that a deployment wires to the robot controller, the base and the tool, and a
> software-visible record of why. It does not replace the robot's own safety controller.

## Why

The abort thresholds existed per tool (`overload_abort_n`, `tangential_torque_abort_nm` in
[`tool_profiles.yaml`](../src/perception/coverage_planner/config/tool_profiles.yaml), the `force_core`
force limiter, the `stand_navigator` obstacle hold and abort, stale-data checks in the controller) but each ended in its
own local reaction and nothing combined them or tied them to a stop path (`hardware_readiness.md` items 9 and 11). The
chain is that combination: one state, defined outputs, one reset rule, one log.

## Inputs, causes and default severity

| Cause | Source | Default |
|---|---|---|
| `force_overload` | `abs(F)` above `force_abort_n` (the tool's `overload_abort_n`) | ESTOP |
| `torque_overload` | `abs(tau)` above `torque_abort_nm` (`tangential_torque_abort_nm`) | ESTOP |
| `stale_ft` | no valid F/T sample within `ft_stale_s` | HOLD |
| `stale_joint_state` | no valid joint state within `joint_stale_s` | ESTOP |
| `base_obstacle_hold` | base navigator holding for an obstacle | HOLD |
| `base_obstacle_abort` | base navigator gave up on an obstacle | ESTOP |
| `operator_estop` | operator topic or service (software e-stop) | ESTOP |
| `controller_fault` | arm controller or driver fault | ESTOP |

Severity per cause is a parameter (`kNone` ignores a cause, it is still logged).

## State machine and outputs

| State | arm_stop | base_stop | tool_off | power_removal_request | Stop category (IEC 60204-1 9.2.2 / ISO 13850) |
|---|---|---|---|---|---|
| OK | no | no | no | no | none |
| HOLD | yes | yes | no | no | category 2 style: stop, power kept |
| ESTOP | yes | yes | yes | yes | category 1 style: controlled stop, then power removal (by the hardware) |

The chain never produces a category 0 stop by itself: uncontrolled removal of power is the hardware e-stop circuit.

Rules, all covered by gtests (`safety_chain_test.cpp`):

- The state only escalates by itself (OK, HOLD, ESTOP). The first cause of the current stop is kept (`latchedCause()`).
- ESTOP is never cleared automatically. HOLD latches too, unless `hold_auto_clear` is set (default off); it then clears after
  `hold_clear_time_s` with nothing active.
- Leaving a stop needs an explicit `reset()`. It is refused while any non-ignored cause is active: the e-stop input still
  asserted, a stale stream (so fresh data is needed), an overload still measured, a one-shot event not yet seen by `step()`.
  A refused reset is logged.
- One-shot events (a force spike, an e-stop press, a fault pulse) that fall between two `step()` calls are remembered.
- Fail-safe: NaN or infinite force or torque is an overload; unstamped or future-dated samples are stale; a stream never fed is stale
  (`require_ft`, `require_joint_state` switch that off for tools without a wrench sensor).
- Event log: timestamped cause assertions, state changes, accepted and refused resets, bounded by `max_events` (oldest dropped, counted).

## Industry basis

- ISO 10218-1 and ISO 10218-2 (robots and robot systems): protective stop and emergency stop functions, stop on loss of safeguard.
- ISO 13850 and IEC 60204-1 9.2.2: emergency stop functions and stop categories (0 uncontrolled, 1 controlled then power off, 2 controlled with power kept); an e-stop must
  latch and need a deliberate reset that does not itself restart motion. The chain follows that reset rule and maps HOLD and ESTOP to categories 2 and 1.
- ros2_control practice: the e-stop belongs in the hardware interface or the drive, not in a controller; controllers only
  see command interfaces and a hardware fault shows up as an error from the hardware component. So the
  `stop` outputs here are requests to the controller and base, and a real deployment keeps the hardware e-stop independent of ROS.
- ISO/TS 15066 (power and force limiting): the contact force and torque thresholds are limits to be derived from the
  risk assessment for the body regions concerned; the shipped `60 N` and `8 Nm` are not such a derivation and must be lowered at commissioning.
- Speed-and-separation monitoring is the neighbouring `SafetyMonitor` in the same package (not an input here yet).

## What it does NOT provide

- Any ISO 13849 performance level or IEC 62061 SIL: no redundancy, no diagnostics, no certified runtime.
- A hardware e-stop, safe torque off, safe brakes, or power removal; `power_removal_request` is only a signal.
- Detection of its own death: nothing watches the chain. A deployment needs the controller's own watchdog (the `HeartbeatWatchdog` in this package
  is one half) and hardware that stops when the chain stops talking.
- Wiring: the ROS node (`/estop` topic and reset service, subscriptions to the existing F/T, joint state and base obstacle
  signals, publishing stop commands to JTAC, the base and the tool) is **not written yet**, and nothing in the stack
  consumes `ChainOutputs`. The default behaviour of every existing node is unchanged. Host tests only; no simulation, no hardware.
- Mapping of "stop the arm" to a concrete controller reaction (hold position, brake, protective stop of the vendor controller).
