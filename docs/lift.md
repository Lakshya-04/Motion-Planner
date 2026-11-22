# Lift: `lift_hardware`

How the one commanded lift joint reaches a lift, in simulation, in tests and on a device. The model of the
column (stages, stroke, tube sizes, mount height) is in [robot_description.md](robot_description.md#lift-variants);
this page is about driving it.

## What the package does

`lift_hardware` ([src/control/lift_hardware](../src/control/lift_hardware/README.md)) is a ros2_control system
component, `lift_hardware/PrismaticLiftSystem`, for **one prismatic joint**: a `position` command, `position`
and `velocity` state. It is split like every module ([package_layout.md](package_layout.md)):

```
ros2_control  <->  PrismaticLiftSystem   (ros/: parameters, interfaces, lifecycle, logging)
                        |
                   LiftSupervisor        (core/: all behaviour, no ROS, no threads, no sleeping)
                        |
                   LiftTransport         (core/: the seam to a device)
                  /       |        \
        MockTransport  ScriptedTransport  DeviceTransport (skeleton, not implemented)
```

The supervisor does the work a control loop needs and a device driver should not repeat:

| Behaviour | Rule |
|---|---|
| Connect | Calls `connect()` and `read_stroke()`; on failure waits `reconnect_initial_s`, then grows the wait by `reconnect_factor` up to `reconnect_max_s`; a success starts the schedule over. |
| Home | After **every** connect the lift is driven to `home_height_m`; it counts as homed when the reading is within `tolerance_m` and the device says idle. A target that never arrives within `home_timeout_s` is a `timeout` fault. Commands are ignored until homed. |
| Clamp | A command is limited to `[0, stroke]`, where stroke is what the device reports (a smaller `stroke_m` may narrow it). Non-finite commands are ignored. |
| Rate limit | The command becomes a reference that speeds up at `max_acceleration_mps2` to at most `max_velocity_mps` and brakes onto the target without overshoot (`core/profile.hpp`). The device is sent that reference, at most every `send_period_s` and only when it moved by `resend_deadband_m`, so a step command never reaches the device as a step. |
| Arrival | After the reference reaches the target the device may take `move_timeout_s` to be within `tolerance_m`; otherwise `timeout` fault. |
| Fault | A device fault, an out-of-range reading, a rejected target (`out_of_range`) or a missed arrival makes the lift **Held**: `stop()` is sent at once and re-sent every cycle until the transport accepts it, the last good height is reported with zero velocity, commands are ignored. `reset_fault()` clears the hold and homes again. |
| Link loss | A `link_lost` result, or `max_io_failures` failed calls in a row (timeouts), ends in a best-effort `stop()`, `disconnect()` and the reconnect schedule above. Once reconnected it homes again, because the position can no longer be trusted. A stop that cannot be delivered `max_io_failures` times counts as link loss. |

`read()` and `write()` of the component always return OK. Returning ERROR makes ros2_control deactivate the
component, after which `read()` is never called again and a lost link could never be re-established. Safety
does not rest on that return value: a height reaches the device only while the supervisor is Ready.

### What it reports

State interfaces of the joint: `position` (last good measured height, 0 until the first reading) and
`velocity` (a low-pass estimate from the measured heights, time constant `velocity_filter_s`). An optional
`<gpio>` block adds `homed`, `phase` (0 disconnected, 1 homing, 2 ready, 3 held), `fault_code`
(0 none, 1 timeout, 2 out_of_range, 3 link_lost, 4 device, 5 unsupported) and `stop_confirmed`, plus the command
interface `reset_fault` (a rise above 0.5 asks for the reset). Interface and joint names come from the
ros2_control description, none are fixed in code. See [config/lift_system_example.yaml](../src/control/lift_hardware/config/lift_system_example.yaml).

## The transport interface

`core/lift_transport.hpp`. A transport speaks metres above the retracted position and nothing else:

| Call | Meaning |
|---|---|
| `connect()` / `disconnect()` | open and close the link; `disconnect()` never fails |
| `read_stroke(m)` | usable travel of the device, > 0 |
| `move_to(m)` | start moving to a height (replaces the previous target) |
| `stop()` | halt now; safe at any time, repeated, or when idle |
| `read(reading)` | `height_m`, `DeviceState` (idle, moving, faulted) and the device's `Fault` |

Every call returns a `Fault`: `none`, `timeout` (transient until repeated), `out_of_range`, `link_lost` (connect
again), `device`, `unsupported`. The calls must return quickly, because the supervisor runs in the control loop;
a slow device keeps its own worker thread and answers from cached data. Not thread safe: one owner.

## Transports in this repository

- **`MockTransport`** (`transport: mock`, the default): a simulated lift for simulation and CI. It moves
  toward the last accepted target with the same velocity and acceleration limits, advances with the clock at
  every call (no thread), rejects targets outside `[0, stroke]` with `out_of_range`, and starts at
  `mock_initial_height_m` so that homing has something to do. Parameters: `mock_stroke_m`,
  `mock_initial_height_m`, `max_velocity_mps`, `max_acceleration_mps2`. It has no faults of its own.
- **`ScriptedTransport`**: for tests. Perfect and instant unless told otherwise: `inject(op, fault, count)`
  makes the next calls of one operation (connect, read stroke, move, stop, read) fail with a chosen fault,
  `set_device_fault` makes the device report a fault, `set_follow_moves(false)` freezes the device, and it
  keeps a log of calls. The supervisor tests use it for timeouts, over-range, device faults, lost links and
  backoff timing.
- **`DeviceTransport`**: a skeleton, **deliberately not implemented**. Every call returns `unsupported`, so
  selecting it (`transport: device`) never moves a lift and the supervisor keeps reporting the fault.

## Plugging in a real device

The device protocol is not part of this public repository. A deployment writes its own subclass of
`LiftTransport` in a private or out-of-tree package. `core/device_transport.hpp` holds the checklist (open and
verify the link with bounded waits, report the device's own travel, convert units and refuse targets outside
travel, an idempotent stop, map the device's fault flags to `Fault`, keep any watchdog alive inside the calls).
To select it:

1. Build the subclass against `lift_hardware_core` and register it where the plugin can reach it. Today
   `transport_factory.cpp` knows two names; add the name of yours there in your fork, or extend the factory
   to take a pluginlib class (not done here: with one transport nothing needs it).
2. Set `transport` and the `LiftConfig` keys as `<param>` entries of the `lift_system` block, and run the
   description with `real_lift_plugin:=lift_hardware/PrismaticLiftSystem` ([hardware_bringup.md](hardware_bringup.md)).
3. Test it first with the supervisor's own scenarios: a transport that passes the checklist behaves under the
   same fault cases as `ScriptedTransport`. Nothing in this repository has been run against a device.

## Parameters

All are `<param>` entries of the hardware block. Defaults are in `core/lift_config.hpp`; a malformed or
inconsistent value fails `on_init` with the reason.

| Key | Default | Meaning |
|---|---|---|
| `transport` | `mock` | `mock` or `device` |
| `stroke_m` | 0 | 0 = device stroke; a smaller value narrows the travel |
| `max_velocity_mps`, `max_acceleration_mps2` | 0.1, 0.2 | limits of the reference (and of the mock) |
| `home_height_m`, `home_timeout_s` | 0, 60 | where to home and how long that may take |
| `tolerance_m`, `move_timeout_s` | 0.005, 20 | arrival band and allowance |
| `send_period_s`, `resend_deadband_m` | 0.05, 0.0005 | how often a changed reference is sent |
| `reconnect_initial_s`, `reconnect_max_s`, `reconnect_factor` | 0.5, 8, 2 | reconnect schedule |
| `max_io_failures` | 5 | consecutive failed calls before the link counts as lost |
| `velocity_filter_s` | 0.1 | velocity estimate filter |
| `mock_stroke_m`, `mock_initial_height_m` | 0.8, 0.25 | mock only |

## The reference lift profile

`mobile_manipulator_description/config/profiles/ur10e_lift_reference.yaml` carries the numbers of one real
two-stage column as a geometry and limits example: stroke 0.8 m, three nested square tubes (outer 0.186,
0.164, 0.146 m; heights 0.648, 0.69, 0.49 m), the middle tube moving half the stroke, retracted top of the
stack 0.703 m above the bottom of the fixed tube, a 0.035 m plate on top (mount height 0.738 m), a velocity
limit of 0.10 m/s and an acceleration of 0.08 m/s^2. The velocity and acceleration come from the drive's own
limits as read from its configuration, not from a measurement of the moving lift: check them on your unit.
The tube wall thickness and masses are not known; the model uses solid primitives and the generic masses.

## Tests and limits

- 48 gtests in `test/core` (profile, config and factory, mock, supervisor), ROS-free; built and run with host
  g++ and gtest. The supervisor tests cover homing on connect, command clamping, rate limits, stop semantics,
  fault hold and reset, timeouts, out-of-range, lost links and the reconnect backoff.
- Not built or run in a workspace: the plugin (`src/ros`), `CMakeLists.txt`, `package.xml` and the plugin
  XML. The plugin was only syntax-checked against ROS 2 Humble headers; Jazzy was not available. No launch
  test, no controller_manager run, nothing on hardware. A device transport does not exist here.
- Time is the clock of the `read()` call (`time.seconds()`); under simulated time it follows the simulation.
- `armBaseHeightAtLiftZero` in `reachability` assumes the default column, see
  [robot_description.md](robot_description.md#lift-variants).
