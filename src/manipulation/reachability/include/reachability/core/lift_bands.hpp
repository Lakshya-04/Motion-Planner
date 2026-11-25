// Lift band design: which lift heights does a wall need, given where the base stands?
//
// With the arm's map and a base station (x, y), every target pose is reachable at some set of
// lift heights. Choosing the fewest heights that together reach all targets gives the "bands":
// the lift is indexed once per band and all in-band work runs with the lift locked
// (see docs/design_principles.md, section 4).
#pragma once

#include <cstddef>
#include <vector>

#include "reachability/core/map4d.hpp"

namespace reachability
{

struct LiftBand
{
  double lift_height{0.0};            ///< Lift position of this band (m).
  std::vector<size_t> pose_indices;   ///< Targets first covered by this band.
};

/// Reach margin options for stand planning. The reach is the arm's radius around its base (the tool origin's
/// distance from the arm base over it is the "reach fraction": 1 = fully stretched, where the dock IK picks
/// contorted branches and the tool is stiff-limited). All off (the default) leaves planning unchanged.
struct ReachLimit
{
  double reach_m{0.0};        ///< Arm reach (m); 0 or non-finite = unknown, the rest is ignored.
  double max_fraction{0.0};   ///< Hard cap: a pose farther than this fraction of the reach is not reachable at that
                              ///< height (0 = off; 0.85 recommended).
  bool balance{false};        ///< Among lift heights that cover equally many targets prefer the smaller worst fraction.

  bool active() const
  {
    return reach_m > 0.0 && reach_m < 1e9 && (max_fraction > 0.0 || balance);
  }
};

struct LiftBandPlan
{
  std::vector<LiftBand> bands;          ///< In the order the greedy cover chose them.
  std::vector<size_t> unreachable;      ///< Targets no candidate height reaches from this station.
  size_t reachable_count{0};
  /// Largest tool-distance / reach over the targets as the bands place them; 0 when the ReachLimit was not active.
  double max_reach_fraction{0.0};
};

/// Fraction-of-targets view of one station: for each target, the candidate heights that reach it.
/// `arm_mount_offset_z` is the arm base's height above the lift's zero position, so the arm base
/// sits at `lift_height + arm_mount_offset_z` in the world.
std::vector<std::vector<bool>> reachableByHeight(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & heights, double arm_mount_offset_z, const ReachLimit & limit = {});

/// Greedy set cover over the candidate heights.
LiftBandPlan planLiftBands(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & candidate_heights, double arm_mount_offset_z, const ReachLimit & limit = {});

/// Evenly spaced candidate heights, inclusive of both ends. Empty for a non-finite argument,
/// step <= 0, hi < lo or more than 100000 heights.
std::vector<double> heightRange(double lo, double hi, double step);

}  // namespace reachability
