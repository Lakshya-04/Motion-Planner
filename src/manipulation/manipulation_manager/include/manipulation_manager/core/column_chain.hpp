/**
 * @file column_chain.hpp
 * @brief When the approach to the next column can be one straight move from where the previous column ended.
 * @details Between two columns of one stand the manager retracts to the column's dock pose and then plans an approach
 *          to the next column with a randomized free-space planner and a joint goal; for a neighbour a few decimetres
 *          away that is a long, different-every-time swing. With `chain_columns` the approach is first tried as a
 *          single Cartesian move; this is the rule for when to try it. ROS-free.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <array>
#include <cmath>

namespace manipulation_manager
{

struct ChainDecision
{
  bool chain{false};
  double shift_m{0.0};   ///< distance between the previous dock pose and the next approach pose
};

/// `valid`: the previous column of this stand finished with the arm free (not stuck); `has_approach`: the next column
/// uses an approach pose (dock_approach_extra_m > 0). Non-finite poses never chain.
inline ChainDecision decideChain(
  bool enabled, bool valid, bool arm_stuck, bool has_approach,
  const std::array<double, 3> & last_dock, const std::array<double, 3> & next_approach, double max_shift_m)
{
  ChainDecision d;
  double s2 = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    const double v = next_approach[i] - last_dock[i];
    s2 += v * v;
  }
  d.shift_m = std::sqrt(s2);
  d.chain = enabled && valid && !arm_stuck && has_approach && std::isfinite(d.shift_m) && d.shift_m <= max_shift_m;
  return d;
}

}  // namespace manipulation_manager
