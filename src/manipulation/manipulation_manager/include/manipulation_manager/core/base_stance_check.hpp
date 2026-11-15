/**
 * @file base_stance_check.hpp
 * @brief Verify that the base's current stance reaches a coverage plan (no base motion).
 * @details The manager runs a whole coverage plan from where the base stands. This checks that
 *          claim against the reachability planner: the targets are expressed in the base frame
 *          (the base at the origin, the wall on the +y side, as the manager already assumes),
 *          the planner returns the stands it would use, and the stance is accepted only when one
 *          stand covers every target and it is where the base already is.
 *          ROS-free: the node builds the service request from `stanceWall` and feeds the reply
 *          to `evaluateStance`.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <string>
#include <vector>

#include "manipulation_manager/core/coverage_source.hpp"

namespace manipulation_manager
{

/// The wall as the reachability planner wants it, in the base frame.
struct StanceWall
{
  std::vector<core_types::Pose> targets;  ///< Every pose of the plan.
  double wall_x{0.0};                     ///< Mean of the targets: a point on the wall plane.
  double wall_y{0.0};
  double wall_z{0.0};
};

/// A stand returned by the reachability planner (arm base x, y in the base frame, base yaw).
struct StanceStand
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct StanceTolerance
{
  double xy_m{0.15};    ///< How far the planned stand may be from where the base is.
  double yaw_rad{0.1};
};

struct StanceVerdict
{
  bool ok{false};
  std::string message;  ///< Why not, and where the base would have to be.
};

/// Targets and wall plane from a coverage plan. Empty targets when the plan has no poses.
StanceWall stanceWall(const std::vector<CoverageSegmentIntent> & intents);

/// Accept the stance only when no target is unreachable or unplanned, exactly one stand covers
/// the plan, and that stand is within `tol` of the origin (where the base is).
StanceVerdict evaluateStance(
  const std::vector<StanceStand> & stands, size_t unreachable, size_t not_planned,
  const StanceTolerance & tol = {});

}  // namespace manipulation_manager
