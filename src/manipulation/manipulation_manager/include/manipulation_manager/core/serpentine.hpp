/**
 * @file serpentine.hpp
 * @brief The pure decisions of serpentine columns: where the tool backs off after a stroke, and whether (and where
 *        to) the arm is brought back down before it is stowed. ROS-free.
 * @details With `serpentine_columns` every stroke keeps the direction the coverage plan gave it (up, then down, ...),
 *          so a stroke may end at the top of the reach. The retract then leaves the wall from the stroke's END, and a
 *          stand that ends there lowers the tool along the standoff plane before the stow, whose start state the
 *          planners refuse when the arm is stretched up.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <vector>

#include "core_types/pose.hpp"

namespace manipulation_manager
{

/// Where the tool backs off the wall: the column's standoff pose, at the height of the stroke's last pose for a
/// serpentine plan (the next column starts there), else unchanged (the stroke's start / dock height).
inline core_types::Pose chooseRetractPose(
  bool serpentine, const core_types::Pose & standoff, const std::vector<core_types::Pose> & stroke)
{
  core_types::Pose p = standoff;
  if (serpentine && !stroke.empty()) {
    p.z = stroke.back().z;
  }
  return p;
}

/// First hint of the retract: the straight move away from the wall for a serpentine plan, free-space otherwise.
/// A failed first try is repeated as "fsp" (see retractFallbackHint).
inline const char * retractModeHint(bool serpentine) {return serpentine ? "cartesian" : "fsp";}

/// Hint of every retry after the first try failed.
inline const char * retractFallbackHint() {return "fsp";}

struct StowLowerDecision
{
  bool lower{false};
  core_types::Pose target;   ///< the last standoff pose at `z_min` (same x, y, orientation)
};

/// Whether to come down the standoff plane before stowing: serpentine, the arm is still at `last_standoff` (the
/// previous column finished with it free: `chain_valid`), and it is higher than `z_min` by more than `min_drop_m`
/// (a zero-length move would only repeat the stow that just failed).
inline StowLowerDecision decideStowLower(
  bool serpentine, bool chain_valid, const core_types::Pose & last_standoff, double z_min, double min_drop_m = 0.01)
{
  StowLowerDecision d;
  d.target = last_standoff;
  d.target.z = z_min;
  d.lower = serpentine && chain_valid && last_standoff.z > z_min + min_drop_m;
  return d;
}

}  // namespace manipulation_manager
