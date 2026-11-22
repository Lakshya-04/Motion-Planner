#include "coverage_planner/core/coverage_pass_expander.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "coverage_planner/core/wall_pose_lift.hpp"

namespace coverage_planner
{


namespace
{

Path extractSubPath(
  const Path & flat_path, uint32_t start_index, uint32_t end_index, bool reverse)
{
  Path sub_path;
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

double distanceBetween(const PoseStamped & a, const PoseStamped & b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Two-pose path from `from` to the end of the next strip after `current` that lies nearer to it. False when no
/// strip follows.
bool nextStripStart(
  const Path & flat_path, const std::vector<MotionPlanSegment> & segments,
  const MotionPlanSegment * current, const PoseStamped & from, Path & out)
{
  bool after = false;
  for (const auto & s : segments) {
    if (&s == current) {after = true; continue;}
    if (!after || s.mode != "cartesian") {continue;}
    const auto & first = flat_path.poses.at(s.start_index);
    const auto & last = flat_path.poses.at(s.end_index);
    out.header = flat_path.header;
    out.poses = {from, distanceBetween(last, from) < distanceBetween(first, from) ? last : first};
    return true;
  }
  return false;
}

}  // namespace

ExpandedCoverage expandCoveragePasses(
  const Path & flat_path, const std::vector<MotionPlanSegment> & flat_segments,
  const WallPlane & wall, const coverage_planner::ToolCoverageProfile & tool_profile)
{
  ExpandedCoverage expanded;
  expanded.path.header = flat_path.header;

  const auto & passes = tool_profile.coverage.passes;
  // A tool whose last pass is reversed ends each strip where it started. The flat plan alternates
  // strip directions (the next strip starts at this strip's END), so such a tool would jump the
  // full strip length at every joint. Start every strip at the end nearest to where the previous
  // strip finished instead: the joint is then only the pitch between strips.
  const bool returns_to_start = passes.size() > 1 && passes.back().reverse_direction;
  const PoseStamped * previous_finish = nullptr;

  for (const auto & segment : flat_segments) {
    if (segment.mode != "cartesian") {
      // With the strip start flipped, a straight two-pose connector left at the planner's end of the strip would
      // run the full strip length and back. Re-aim it from where the previous strip finished to where the next
      // one starts. A connector routed around something (more than two poses) is kept as planned.
      Path rerouted;
      const bool reroute = returns_to_start && previous_finish != nullptr &&
        segment.end_index == segment.start_index + 1 && nextStripStart(flat_path, flat_segments, &segment, *previous_finish, rerouted);
      // Connector/fsp segments: lift once at the base standoff, never duplicated per pass.
      const auto sub_path = reroute ? rerouted :
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

    bool flip = false;
    if (returns_to_start) {
      const auto & first = flat_path.poses.at(segment.start_index);
      const auto & last = flat_path.poses.at(segment.end_index);
      if (previous_finish != nullptr) {
        flip = distanceBetween(last, *previous_finish) < distanceBetween(first, *previous_finish);
      }
      previous_finish = flip ? &last : &first;
    }

    for (size_t pass_index = 0; pass_index < passes.size(); ++pass_index) {
      const auto & pass = passes[pass_index];
      const auto sub_path = extractSubPath(
        flat_path, segment.start_index, segment.end_index, pass.reverse_direction != flip);

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
