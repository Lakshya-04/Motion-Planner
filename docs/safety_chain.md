---
type: Reference
title: Safety chain
description: ROS-free core, and its opt-in ROS node (not compiled or run), that ties the per-tool abort thresholds and the other independent safety inputs to one latched OK / HOLD / ESTOP state with defined stop outputs; what it is based on and what it does not provide.
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

## ROS node (`safety_chain_node`, opt-in)

> **Not compiled and not run.** The node was written and reviewed by hand on a host that cannot build this
> workspace (Humble headers, Jazzy target) with the simulators busy. Only its ROS-free helpers
> (`safety_chain_node_core.hpp`, 8 host gtests) were tested. **Nothing consumes the stop topic yet**: JTAC, the
> manager and the base do not subscribe to it. The node is not started by any existing launch file;
> run it with `ros2 launch safety_monitor safety_chain.launch.py` (see the launch file's docstring). Not a certified safety function.

What it does, every `1 / rate_hz` seconds: steps the `SafetyChain` and publishes.

| Direction | Topic (default) | Type | Notes |
|---|---|---|---|
| in | `wrench_topic` `/ft_conditioner/wrench_conditioned` | `geometry_msgs/WrenchStamped` | `abs(F)` and `abs(tau)` go to the chain; a non-finite component is a trip. Receive time is used, not the header stamp |
| in | `joint_states_topic` `/joint_states` | `sensor_msgs/JointState` | only its arrival (freshness) is used |
| in | `estop_topic` `/safety_chain/estop` | `std_msgs/Bool` | level; a rising edge between two steps is not lost |
| in | `controller_fault_topic` `/safety_chain/controller_fault` | `std_msgs/Bool` | level, same rule |
| out | `state_topic` `/safety_chain/state` | `std_msgs/String`, transient_local | `OK`, or `HOLD force_overload,stale_ft` / `ESTOP ...` (the causes asserted since the chain was last OK), on change |
| out | `arm_stop_topic` `/safety_chain/arm_stop` | `std_msgs/Bool`, transient_local | `true` in HOLD and ESTOP, `false` in OK; on change |
| out | `cmd_vel_topic` `/cmd_vel` | `geometry_msgs/Twist` | an all-zero Twist every step while the chain says `base_stop`; nothing in OK. Put it behind a `twist_mux` priority, a competing publisher on the same topic is not overridden |
| service | `~/reset` | `std_srvs/Trigger` | `SafetyChain::reset()`; on refusal `success` is false and `message` names the active cause, e.g. `reset refused: still active: operator_estop` |

Why a latched `Bool` for the arm stop: it is the least invasive interface. No existing node has to change or link to
this package, a later consumer (JTAC, the manager) adds one subscription, and a transient_local topic is seen by a
consumer that starts after the stop. A service would need a client in every consumer and loses the "stay stopped"
level. The base gets a zero `cmd_vel` because that is the interface it already has. `tool_off` and `power_removal_request` are computed
by the chain but not published (no tool or power interface exists); `base_obstacle_*` has no input yet.

Behaviour to know: with `require_ft` and `require_joint_state` true (default), the chain goes to HOLD or ESTOP at the first step if no wrench or joint
state has arrived yet; start the sensors first or set `require_ft:=false` for a non-contact tool. The node refuses to start
without known thresholds (a bad profile path, an unknown tool, a bad severity string all end the process with a fatal log).

| Parameter | Default | Meaning |
|---|---|---|
| `tool_profiles_file` | `""` | `tool_profiles.yaml` to read the tool's `overload_abort_n` and `tangential_torque_abort_nm` from (loaded with `ros_adapters::loadToolProfile`, as the coverage planner does) |
| `tool_name` | `""` | tool key in that file; with a file or a name set, both must be valid |
| `force_abort_n` | `-1.0` | override of the force threshold in N; negative = take it from the profile; `0` disables the check |
| `torque_abort_nm` | `-1.0` | the same for the torque in Nm. Without a profile both overrides must be `>= 0` |
| `ft_stale_s` | `0.1` | no wrench within this many seconds is `stale_ft` |
| `joint_stale_s` | `0.1` | the same for joint states, `stale_joint_state` |
| `require_ft` | `true` | a never-received wrench counts as stale |
| `require_joint_state` | `true` | the same for joint states |
| `hold_auto_clear` | `false` | HOLD may clear by itself; ESTOP never does |
| `hold_clear_time_s` | `1.0` | with `hold_auto_clear`, seconds with no cause before HOLD clears |
| `severity.<cause>` | per the cause table above | `none`, `hold` or `estop` for each of `force_overload`, `torque_overload`, `stale_ft`, `stale_joint_state`, `base_obstacle_hold`, `base_obstacle_abort`, `operator_estop`, `controller_fault` (launch argument `severity_<cause>`) |
| `rate_hz` | `50.0` | step and publish rate; must be above 0 |
| `wrench_topic`, `joint_states_topic`, `estop_topic`, `controller_fault_topic`, `cmd_vel_topic`, `arm_stop_topic`, `state_topic` | see the table above | topic names |

Industry reuse: the pattern follows what ROS 2 offers rather than a new mechanism: a latched `std_msgs/Bool` e-stop topic and a `std_srvs/Trigger`
reset as in the common e-stop relays and `twist_mux` lock topics, a zero `cmd_vel` as the base stop, and `transient_local` QoS for
state that late joiners must see. A node like this is still ordinary software and cannot be a safety function.

## What it does NOT provide

- Any ISO 13849 performance level or IEC 62061 SIL: no redundancy, no diagnostics, no certified runtime.
- A hardware e-stop, safe torque off, safe brakes, or power removal; `power_removal_request` is only a signal.
- Detection of its own death: nothing watches the chain. A deployment needs the controller's own watchdog (the `HeartbeatWatchdog` in this package
  is one half) and hardware that stops when the chain stops talking.
- Consumers: the opt-in `safety_chain_node` (section above) is written but not compiled or run, has no base obstacle input, and nothing in the stack
  consumes its stop topic (JTAC, the base driver and the tool do not subscribe). The default behaviour of every existing node is unchanged. Host tests of the helpers only; no simulation, no hardware.
- Mapping of "stop the arm" to a concrete controller reaction (hold position, brake, protective stop of the vendor controller).
