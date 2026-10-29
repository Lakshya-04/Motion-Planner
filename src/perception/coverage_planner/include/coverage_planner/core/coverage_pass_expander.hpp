#ifndef COVERAGE_PLANNER__COVERAGE_PASS_EXPANDER_HPP_
#define COVERAGE_PLANNER__COVERAGE_PASS_EXPANDER_HPP_

#include "coverage_planner/core/types.hpp"
#include <vector>


#include "coverage_planner/core/tool_coverage_profile.hpp"

namespace coverage_planner
{


struct ExpandedCoverage
{
  Path path;
  std::vector<MotionPlanSegment> segments;
};

/**
 * @brief Lifts every segment of a flat 2D coverage path into final oriented/standoff-offset poses
 *        (via liftCoveragePathPose), and -- for cartesian (strip/boundary) segments only --
 *        expands each one into one sub-path per entry in `tool_profile.coverage.passes` (mop's
 *        double pass = two identical-standoff passes, reversed direction on the second).
 * @details fsp/connector segments are never duplicated -- passes are a contact-surface concept
 *          and connectors are traversed exactly once regardless of the adjacent strips' passes.
 * @param flat_path Positions only (world frame), identity orientation -- CoveragePlannerResult::path.
 * @param flat_segments The matching CoveragePlannerResult::segments (indices into flat_path).
 * @param wall The wall this path was planned on.
 * @param tool_profile The active tool's full profile.
 */
ExpandedCoverage expandCoveragePasses(
  const Path & flat_path, const std::vector<MotionPlanSegment> & flat_segments,
  const WallPlane & wall, const coverage_planner::ToolCoverageProfile & tool_profile);

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__COVERAGE_PASS_EXPANDER_HPP_
