# Teleoperation: modes, jog shaping and limiting

JTAC (the compliant trajectory controller) commands position and velocity only, and there is no teleoperation code
in the stack. This page describes the ROS-free logic that sits around a Cartesian jog engine. Following the repository's
rule of adopting industry practice first, the engine itself is planned to be MoveIt Servo (twist in, joint commands
out, with its own singularity and joint-limit handling); it is not written here.

**Status: the logic below exists as the library `teleop_core` with a host-compiled gtest suite (66 teleop_core tests). There is no node, no
MoveIt Servo wiring, no joystick driver, and nothing has been run in a simulator or on hardware.**

## Modes and transitions

`TeleopModeManager` has three modes. Every request returns a reason code; refused requests are counted
(`refusedCount()`), and asking for the mode already active is a no-op, not an illegal transition.

| From | To | Condition | Reason code |
|---|---|---|---|
| AUTO | HOLD | stop or safety event, always | `kStopRequested` / `kSafetyStop` |
| AUTO | TELEOP | operator request and the running segment is halted (caller reports `segment_halted`) | `kOperatorRequest`; refused with `kSegmentStillRunning` |
| TELEOP | HOLD | stop or safety event, or no operator input for `input_timeout_s` (2 s). A clock that went backwards, an input stamped after the tick time, or a non-finite tick time also count as a timeout | `kStopRequested` / `kSafetyStop` / `kInputTimeout` |
| TELEOP | AUTO | explicit operator confirm and a replan-from-current-pose acknowledgement | `kOperatorConfirmed`; refused with `kNeedsConfirm` / `kNeedsReplanAck` |
| HOLD | TELEOP | operator request (refused with `kInvalidTime` for a non-finite time, as is AUTO to TELEOP) | `kOperatorRequest` |
| HOLD | AUTO | operator request; also the replan acknowledgement if teleoperation moved the arm since the last AUTO; also `operator_confirm` if the HOLD came from a safety stop (`onStop(safety=true)`; the reason is kept until HOLD is left) | `kOperatorRequest`; refused with `kNeedsConfirm` / `kNeedsReplanAck` |

The last row goes slightly beyond "by request only": after TELEOP the arm is somewhere the old plan did not
start from, so resuming through HOLD must not skip the replan.

## Jog shaping

`JogShaper` turns a normalised 6D input (spacemouse or joystick, -1 to 1) into a base-frame twist:
clamp, per-axis deadband (rescaled so full deflection still reaches the maximum), expo curve
`(1 - e) x + e x^3`, per-axis enable mask, scale to the maximum linear and angular speed, clamp the norm of the linear part to `max_linear_mps` and of the angular part to `max_angular_rps` (direction kept, so (1, 1, 1) is not faster than a single axis at full deflection), rotate from the selected
frame (tool, base or world, using a rotation the caller supplies), then a vector slew limit for linear and angular
parts (direction kept). A watchdog gives an immediate zero twist, flagged, when no valid input arrived for
`watchdog_s`; the watchdog also trips when time goes backwards (now before the last input). Non-finite input counts as zero and does not refresh the watchdog. A non-finite frame rotation gives a zero twist with `invalid_rotation` set and clears the slew state, so the next good sample ramps from rest.

Config validation: `input_timeout_s`, `watchdog_s` and `stale_timeout_s` must be finite and > 0, and the shaper accel limits must be finite (<= 0 still disables slew); otherwise the constructor or `setConfig` throws `std::invalid_argument` (the shaper keeps its old config).

The enable mask is how force-limited contact is meant to work: while the tool is pressing, the contact-normal axis is
masked off and the remaining axes slide along it. The mask is applied in the jog frame (tool Z for a wall contact)
before rotation. What is guaranteed: from the `update()` after an axis becomes masked, the commanded velocity along
that axis in the jog frame is zero (the masked axes of the slew state are cleared, there is no ramp-down), up to
floating-point rounding of the rotation. Not guaranteed: that the arm does not touch or press the wall. The shaper
does not sense contact, other axes can still move, and motion already issued downstream (Servo, controller
latency) is outside this class.

## Limiting

`JogLimiter` is the last stage: it scales the whole twist uniformly (direction preserved) by a safety `speed_scale`
in [0, 1], gives zero when a stop is requested, and gives zero when the scale is missing, older than
`stale_timeout_s`, not finite, or the clock goes backwards.

## How it will sit around MoveIt Servo and JTAC

```
spacemouse -> JogShaper -> JogLimiter -> MoveIt Servo (twist) -> joint command -> JTAC (position + velocity)
                  ^             ^                                                    |
   TeleopModeManager (gates)    safety speed_scale / stop                  force FSM state, ContactFreeGate
```

The planned `teleop_bridge` node would own the clock, call the three classes, publish the twist to Servo, and
arbitrate with the force state machine (no TELEOP while a force-staged segment group is active unless the
operator stops it first, which is the "segment halted" input). JTAC keeps tracking whatever Servo commands; in
TELEOP the force controller stays disengaged, or admittance runs on the contact axis only.

## Safety interaction

The planned `safety_monitor` (whole-robot collision and speed layer) would feed `onStop(safety=true)` and the
`speed_scale` of `JogLimiter`. The limiter is deliberately stateless apart from the last scale, so a lost safety
link fails to zero within `stale_timeout_s`. The mode manager and limiter are an operator-convenience and
fail-safe layer; they are not a certified safety function.

## Not done

- No node, topics or parameters; no MoveIt Servo configuration; no input device driver.
- Joint-space jog is not covered (only the Cartesian twist path).
- Default speeds, deadband, expo and timeouts are untuned guesses.
- Related, separate work: per-link `gravcomp` in the MuJoCo generator (`docs/sim_mujoco.md`) and the
  `contact_free` gate for the bias tracker (`docs/ft_sensing.md`). Effort-mode actuators in the generator were not
  added: they change the actuator block and `ctrlrange` handling, so they are not a trivial additive option.
