#ifndef COVERAGE_PLANNER__COVERAGE_PASS_EXPANDER_HPP_
#define COVERAGE_PLANNER__COVERAGE_PASS_EXPANDER_HPP_

#include <vector>

#include <nav_msgs/msg/path.hpp>

#include "coverage_planner/tool_coverage_profile.hpp"
#include "coverage_planner_msgs/msg/motion_plan_segment.hpp"
#include "coverage_planner_msgs/msg/wall_plane.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace msg = coverage_planner_msgs::msg;

struct ExpandedCoverage
{
  nav_msgs::msg::Path path;
  std::vector<msg::MotionPlanSegment> segments;
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
  const nav_msgs::msg::Path & flat_path, const std::vector<msg::MotionPlanSegment> & flat_segments,
  const msg::WallPlane & wall, const coverage_planner::ToolCoverageProfile & tool_profile);

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__COVERAGE_PASS_EXPANDER_HPP_
