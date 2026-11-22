#include "lift_hardware/core/profile.hpp"

#include <algorithm>
#include <cmath>

namespace lift_hardware
{

Kinematics advance_toward(
  const Kinematics & state, double target, double max_velocity, double max_acceleration, double dt)
{
  if (!(max_velocity > 0.0) || !(max_acceleration > 0.0) || !(dt > 0.0)) {
    return state;
  }
  const double error = target - state.position;
  const double step = max_acceleration * dt;

  // Already (almost) at rest on the target: land exactly.
  if (std::abs(error) <= 0.5 * step * dt && std::abs(state.velocity) <= step) {
    return {target, 0.0};
  }

  // The fastest speed from which the rest of the distance can still be braked away at the limit.
  // The distance this step itself covers is taken off first, which keeps the discrete steps from
  // arriving a little too fast and being snapped onto the target.
  const double direction = error >= 0.0 ? 1.0 : -1.0;
  const double room = std::max(0.0, std::abs(error) - std::abs(state.velocity) * dt);
  const double wanted = direction * std::min(max_velocity, std::sqrt(2.0 * max_acceleration * room));
  const double velocity = state.velocity + std::clamp(wanted - state.velocity, -step, step);
  const double position = state.position + 0.5 * (state.velocity + velocity) * dt;

  if ((target - position) * direction < 0.0) {
    return {target, 0.0};  // would cross the target in this step
  }
  return {position, velocity};
}

}  // namespace lift_hardware
