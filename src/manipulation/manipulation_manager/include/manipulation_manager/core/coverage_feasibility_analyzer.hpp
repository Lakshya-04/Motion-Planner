/**
 * @file coverage_feasibility_analyzer.hpp
 * @brief Proactively classifies a coverage plan's poses as plannable or not,
 *        before any real plan()/execute() attempt.
 * @details Walks each "cartesian"-hinted CoverageSegmentIntent's dense poses
 *          with Planner::ik()/collision(), carrying the previous point's own
 *          IK solution forward as the next point's seed (seed continuity --
 *          checking each pose in isolation from a default seed can report
 *          "feasible" for poses that aren't actually reachable in sequence
 *          without a large, physically-meaningless joint jump). Feasible runs
 *          become MotionSegmentSpecs; excluded points are reported with a
 *          reason. "fsp"-hinted intents (e.g. explicit connectors) are not
 *          point-checked -- FSP already handles arbitrary reachability via
 *          full OMPL planning -- they pass through as one segment each.
 *          Gaps between feasible runs are bridged automatically by
 *          ManipulationManager's existing forced_handoff_to_fsp logic; this
 *          analyzer only needs to produce index-discontinuous segments.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>
#include <vector>

#include "core_types/pose.hpp"
#include "manipulation_manager/core/coverage_source.hpp"
#include "manipulation_manager/core/motion_segment_spec.hpp"
#include "motion_planner/core/planners/planner.hpp"

namespace manipulation_manager
{

using motion_planner::Planner;

/**
 * @struct FeasibilityExcludedPoint
 * @brief One pose that didn't make it into any feasible segment, and why.
 */
struct FeasibilityExcludedPoint
{
  size_t global_index{0};      ///< Index into the analyzer's combined waypoint_poses output.
  std::string segment_label;   ///< Which CoverageSegmentIntent this point belongs to.
  std::string reason;          ///< "ik_failed" | "collision_detected" | "isolated_point".
};

/**
 * @struct FeasibilityReport
 * @brief Result of analyzing a full coverage plan.
 */
struct FeasibilityReport
{
  std::vector<MotionSegmentSpec> segments;             ///< Feasible + pass-through segments, in order.
  std::vector<core_types::Pose> waypoint_poses;    ///< Combined global pose array segments index into.
  std::vector<FeasibilityExcludedPoint> excluded_points; ///< Points that didn't make it into any segment.
};

/**
 * @class CoverageFeasibilityAnalyzer
 * @brief Pre-checks a coverage plan's feasibility using an injected Planner's
 *        ik()/collision() primitives -- no MoveGroupInterface of its own.
 */
class CoverageFeasibilityAnalyzer
{
public:
  /**
   * @param[in] planner Used for ik()/collision() queries. The caller owns it
   *                     (typically ManipulationManager's own cartesian_planner_)
   *                     -- this analyzer never constructs or owns a planner.
   */
  explicit CoverageFeasibilityAnalyzer(Planner & planner);

  /**
   * @brief Analyze a full coverage plan (in order) and produce a feasible
   *        segment list + combined global waypoint array + excluded points.
   * @param[in] intents The coverage plan, in mission order.
   * @param[in] frame_id Reference frame the intents' poses are expressed in.
   * @param[in] planning_group MoveIt planning group to check feasibility for.
   */
  FeasibilityReport analyze(
    const std::vector<CoverageSegmentIntent> & intents,
    const std::string & frame_id,
    const std::string & planning_group);

private:
  /// Minimum consecutive feasible points to form a segment -- a single point
  /// can't be a Cartesian segment on its own.
  static constexpr size_t kMinFeasibleRunLength = 2;

  Planner & planner_;
};

}  // namespace manipulation_manager
