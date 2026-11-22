/**
 * @file profile.hpp
 * @brief One step of a velocity- and acceleration-limited approach to a target position.
 */

#pragma once

namespace lift_hardware
{

struct Kinematics
{
  double position{0.0};
  double velocity{0.0};
};

/**
 * @brief Advance `state` by `dt` seconds toward `target`.
 * @details Speeds up at `max_acceleration` to at most `max_velocity`, then slows so that it comes
 *          to rest on the target without overshooting (the velocity is capped by what the
 *          remaining distance allows at `max_acceleration`). Reversing first brakes to rest.
 *          Non-positive limits or a non-positive `dt` leave the state unchanged. The target is
 *          reached exactly: the last step snaps onto it with zero velocity, which can exceed the
 *          acceleration limit by up to one step's worth.
 */
Kinematics advance_toward(
  const Kinematics & state, double target, double max_velocity, double max_acceleration, double dt);

}  // namespace lift_hardware
