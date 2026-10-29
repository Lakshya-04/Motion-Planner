#include "coverage_planner/coverage_pass_expander.hpp"

#include <algorithm>
#include <string>

#include "coverage_planner/wall_pose_lift.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace msg = coverage_planner_msgs::msg;

namespace
{

nav_msgs::msg::Path extractSubPath(
  const nav_msgs::msg::Path & flat_path, uint32_t start_index, uint32_t end_index, bool reverse)
{
  nav_msgs::msg::Path sub_path;
  sub_path.header = flat_path.header;
  sub_path.poses.reserve(static_cast<size_t>(end_index - start_index) + 1);
  for (uint32_t i = start_index; i <= end_index; ++i) {
    sub_path.poses.push_back(flat_path.poses.at(i));
  }
  if (reverse) {
    std::reverse(sub_path.poses.begin(), sub_path.poses.end());
  }
  return sub_path;
}

}  // namespace

ExpandedCoverage expandCoveragePasses(
  const nav_msgs::msg::Path & flat_path, const std::vector<msg::MotionPlanSegment> & flat_segments,
  const msg::WallPlane & wall, const coverage_planner::ToolCoverageProfile & tool_profile)
{
  ExpandedCoverage expanded;
  expanded.path.header = flat_path.header;

  const auto & passes = tool_profile.coverage.passes;

  for (const auto & segment : flat_segments) {
    if (segment.mode != "cartesian") {
      // Connector/fsp segments: lift once at the base standoff, never duplicated per pass.
      const auto sub_path =
        extractSubPath(flat_path, segment.start_index, segment.end_index, false);
      const auto lifted = liftCoveragePathPose(sub_path, wall, tool_profile.task);

      auto new_segment = segment;
      new_segment.start_index = static_cast<uint32_t>(expanded.path.poses.size());
      expanded.path.poses.insert(
        expanded.path.poses.end(), lifted.poses.begin(), lifted.poses.end());
      new_segment.end_index = static_cast<uint32_t>(expanded.path.poses.size()) - 1;
      expanded.segments.push_back(std::move(new_segment));
      continue;
    }

    for (size_t pass_index = 0; pass_index < passes.size(); ++pass_index) {
      const auto & pass = passes[pass_index];
      const auto sub_path =
        extractSubPath(flat_path, segment.start_index, segment.end_index, pass.reverse_direction);

      coverage_planner::ToolTask pass_task = tool_profile.task;
      pass_task.standoff_m += pass.standoff_delta_m;

      const auto lifted = liftCoveragePathPose(sub_path, wall, pass_task);

      auto new_segment = segment;
      if (passes.size() > 1) {
        new_segment.label = segment.label + "_pass" + std::to_string(pass_index);
      }
      new_segment.start_index = static_cast<uint32_t>(expanded.path.poses.size());
      expanded.path.poses.insert(
        expanded.path.poses.end(), lifted.poses.begin(), lifted.poses.end());
      new_segment.end_index = static_cast<uint32_t>(expanded.path.poses.size()) - 1;
      if (pass.reverse_direction) {
        new_segment.tags.push_back("second_pass");
      }
      expanded.segments.push_back(std::move(new_segment));
    }
  }

  return expanded;
}

}  // namespace coverage_planner
