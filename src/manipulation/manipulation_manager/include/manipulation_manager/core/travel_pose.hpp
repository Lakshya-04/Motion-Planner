#pragma once

#include <vector>

namespace manipulation_manager
{

/// The travel (stow) pose with its first joint, the lift, kept `margin` inside its travel [lo, hi].
/// A lift commanded to its end stop settles a millimetre or two beyond it; MoveIt's CheckStartStateBounds
/// then rejects the start state of every plan ("Joint 'lift_joint' from the starting state is outside
/// bounds by -0.0015"), the arm cannot be moved again and the rest of the run is lost. A pose a little
/// inside the range avoids that. `hi` <= `lo` means the upper end is unknown and only the lower side is
/// kept off. An empty pose is returned unchanged, as is a margin <= 0.
inline std::vector<double> keepLiftOffEndStops(
  std::vector<double> pose, double lo, double hi, double margin)
{
  if (pose.empty() || margin <= 0.0) {
    return pose;
  }
  double & lift = pose.front();
  const bool has_hi = hi > lo + 2.0 * margin;
  if (has_hi && lift > hi - margin) {
    lift = hi - margin;
  }
  if (lift < lo + margin) {
    lift = lo + margin;
  }
  return pose;
}

}  // namespace manipulation_manager
