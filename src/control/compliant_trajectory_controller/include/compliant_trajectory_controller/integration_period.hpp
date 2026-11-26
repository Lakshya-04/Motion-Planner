#pragma once

#include <algorithm>
#include <cmath>

namespace compliant_trajectory_controller
{

/// The step the admittance integrator and the force limiter use for one controller cycle.
///
/// The admittance rule integrates with explicit Euler, which is stable only while damping * dt / mass < 2. A
/// cycle that overruns (ros2_control reports a 14 to 22 ms loop at 100 Hz when the host is busy) hands the rule a
/// longer dt and can push it past that bound, after which the offset runs away and the controller resets it
/// (the "Cartesian offset diverged" warning), snapping the tool back into the wall. Capping the step slows the
/// compliance dynamics for that one cycle instead, which is harmless.
///
/// max_dt_s <= 0 (or not finite) turns the cap off. A negative or non-finite dt is returned as 0 so the
/// integrator never runs backwards.
inline double cappedIntegrationPeriod(double dt_s, double max_dt_s)
{
  if (!std::isfinite(dt_s) || dt_s < 0.0) {
    return 0.0;
  }
  if (!std::isfinite(max_dt_s) || max_dt_s <= 0.0) {
    return dt_s;
  }
  return std::min(dt_s, max_dt_s);
}

}  // namespace compliant_trajectory_controller
