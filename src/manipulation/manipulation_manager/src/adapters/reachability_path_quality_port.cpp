#include "manipulation_manager/adapters/reachability_path_quality_port.hpp"

#include <algorithm>
#include <utility>

namespace manipulation_manager
{

ReachabilityPathQualityPort::ReachabilityPathQualityPort(
  std::shared_ptr<const reachability::ArmModel> arm, reachability::PathQualityConfig config,
  size_t skip_leading_joints)
: arm_(std::move(arm)), config_(config), skip_(skip_leading_joints)
{
}

PathQualitySummary ReachabilityPathQualityPort::audit(const std::vector<PathQualityStroke> & strokes)
{
  PathQualitySummary out;
  out.sigma_threshold = config_.min_sigma;
  out.limit_margin_threshold = config_.min_limit_margin;
  if (!arm_) {
    out.note = "no arm model";
    return out;
  }
  if (strokes.empty()) {
    out.note = "no joint samples were recorded (no stroke ran, or /joint_states was silent)";
    return out;
  }
  const size_t dof = static_cast<size_t>(arm_->dof());
  std::vector<reachability::PathStroke> in;
  in.reserve(strokes.size());
  for (const auto & s : strokes) {
    reachability::PathStroke ps;
    ps.id = std::to_string(s.column);
    for (const auto & q : s.waypoints) {
      if (q.size() < skip_ || q.size() - skip_ != dof) {
        continue;   // not this arm's joints: leave it out, the audit counts the stroke as not evaluated
      }
      ps.waypoints.emplace_back(q.begin() + static_cast<std::ptrdiff_t>(skip_), q.end());
    }
    in.push_back(std::move(ps));
  }
  const auto report = reachability::auditPathQuality(
    in, reachability::armJacobianFn(*arm_), reachability::armLimitMarginFn(*arm_), config_);
  out.available = report.strokes.size() == strokes.size();
  if (report.unevaluated == report.total) {
    out.available = false;
    out.note = "no joint sample matched the chain model (check the chain file and path_skip_joints)";
  }
  for (size_t i = 0; i < report.strokes.size(); ++i) {
    const auto & sq = report.strokes[i];
    ColumnPathQuality c;
    c.column = strokes[i].column;
    c.reason = reachability::toString(sq.reason);
    c.min_sigma = sq.min_sigma;
    c.min_limit_margin = sq.min_limit_margin;
    c.worst_index = sq.worst_index;
    out.columns.push_back(c);
  }
  out.near_singular = report.near_singular;
  out.near_limit = report.near_limit;
  out.min_sigma = report.min_sigma;
  out.min_limit_margin = report.min_limit_margin;
  return out;
}

}  // namespace manipulation_manager
