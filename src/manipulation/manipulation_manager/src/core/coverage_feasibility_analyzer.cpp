#include "manipulation_manager/core/coverage_feasibility_analyzer.hpp"

#include <cctype>
#include <utility>

namespace manipulation_manager
{

using motion_planner::CollisionRequest;
using motion_planner::CollisionResult;
using motion_planner::IkRequest;
using motion_planner::IkResult;

namespace
{

std::vector<double> toFlatPose(const core_types::Pose & pose)
{
  return {pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw};
}

std::string toLowerCopy(std::string value)
{
  for (char & c : value) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return value;
}

}  // namespace

CoverageFeasibilityAnalyzer::CoverageFeasibilityAnalyzer(Planner & planner)
: planner_(planner)
{
}

FeasibilityReport CoverageFeasibilityAnalyzer::analyze(
  const std::vector<CoverageSegmentIntent> & intents,
  const std::string & frame_id,
  const std::string & planning_group)
{
  FeasibilityReport report;

  // Persists across intent boundaries, not just within one -- the last
  // feasible point's IK solution seeds the next intent's first point too
  // (e.g. pass 2 seeded from pass 1's end), not just point-to-point inside a
  // single intent. An FSP pass-through intent (below) performs no per-point
  // check and so leaves this unchanged; it's carried forward as a best-effort
  // starting guess for whatever intent comes next, not a guarantee.
  std::vector<double> seed;

  for (const auto & intent : intents) {
    const size_t segment_start_global_index = report.waypoint_poses.size();
    for (const auto & pose : intent.poses) {
      report.waypoint_poses.push_back(pose);
    }

    if (toLowerCopy(intent.mode_hint) != "cartesian") {
      // fsp (or any other non-cartesian hint): pass through as one segment,
      // no per-point feasibility check -- FSP already handles arbitrary
      // reachability via full OMPL planning.
      MotionSegmentSpec spec;
      spec.label = intent.label;
      spec.mode_hint = intent.mode_hint;
      spec.allow_target_contact = intent.requires_contact;
      spec.target_eef_speed = intent.target_eef_speed;
      spec.waypoint_indices.reserve(intent.poses.size());
      for (size_t i = 0; i < intent.poses.size(); ++i) {
        spec.waypoint_indices.push_back(segment_start_global_index + i);
      }
      report.segments.push_back(std::move(spec));
      continue;
    }

    // Cartesian intent: walk each point with ik() + collision(), carrying
    // the last known-good IK solution forward as the next point's seed.
    std::vector<size_t> current_run;  // global indices of the in-progress feasible run.

    auto flushRun = [&]() {
        if (current_run.size() >= kMinFeasibleRunLength) {
          MotionSegmentSpec spec;
          spec.label = intent.label;
          spec.mode_hint = "cartesian";
          spec.allow_target_contact = intent.requires_contact;
          spec.target_eef_speed = intent.target_eef_speed;
          spec.waypoint_indices = current_run;
          report.segments.push_back(std::move(spec));
        } else {
          for (const size_t global_index : current_run) {
            report.excluded_points.push_back(
              {global_index, intent.label, "isolated_point"});
          }
        }
        current_run.clear();
      };

    for (size_t i = 0; i < intent.poses.size(); ++i) {
      const size_t global_index = segment_start_global_index + i;

      IkRequest ik_request;
      ik_request.frame_id = frame_id;
      ik_request.planning_group = planning_group;
      ik_request.target_pose = toFlatPose(intent.poses[i]);
      ik_request.joint_seed = seed;

      const IkResult ik_result = planner_.ik(ik_request);
      if (!ik_result.success) {
        flushRun();
        // Use the planner's own reason (e.g. "ik_failed", but also
        // "missing_current_state"/"invalid_joint_seed"/etc. on a precheck
        // failure) rather than a hardcoded string -- collapsing every
        // failure into "ik_failed" previously masked a real bug (an empty
        // seed fell through to a live getCurrentState() query that never
        // completes in this environment) as an indistinguishable wall of
        // "ik_failed" exclusions.
        report.excluded_points.push_back(
          {global_index, intent.label,
            ik_result.failure_reason_code});
        continue;
      }

      CollisionRequest collision_request;
      collision_request.planning_group = planning_group;
      collision_request.joint_values = ik_result.joint_solution;

      const CollisionResult collision_result = planner_.collision(collision_request);
      if (!collision_result.success || !collision_result.collision_free) {
        flushRun();
        // !success means the check itself failed (report why); success but
        // !collision_free means it ran fine and found a real collision.
        const std::string reason = !collision_result.success ?
          collision_result.failure_reason_code : "collision_detected";
        report.excluded_points.push_back({global_index, intent.label, reason});
        continue;
      }

      current_run.push_back(global_index);
      seed = ik_result.joint_solution;
    }
    flushRun();
  }

  return report;
}

}  // namespace manipulation_manager
