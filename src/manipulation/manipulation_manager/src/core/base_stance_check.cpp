#include "manipulation_manager/core/base_stance_check.hpp"

#include <cmath>
#include <sstream>

namespace manipulation_manager
{

StanceWall stanceWall(const std::vector<CoverageSegmentIntent> & intents)
{
  StanceWall wall;
  double sum_y = 0.0;
  for (const auto & intent : intents) {
    for (const auto & p : intent.poses) {
      wall.targets.push_back(p);
      sum_y += p.y;
    }
  }
  if (!wall.targets.empty()) {
    wall.wall_y = sum_y / static_cast<double>(wall.targets.size());
  }
  return wall;
}

StanceVerdict evaluateStance(
  const std::vector<StanceStand> & stands, size_t unreachable, size_t not_planned,
  const StanceTolerance & tol)
{
  StanceVerdict v;
  std::ostringstream msg;
  if (unreachable > 0 || not_planned > 0) {
    msg << unreachable << " target(s) no stand reaches, " << not_planned << " not planned";
  } else if (stands.empty()) {
    msg << "the reachability planner returned no stand";
  } else if (stands.size() > 1) {
    msg << stands.size() << " stands are needed to cover this plan; the manager runs it from one stance";
  } else {
    const auto & s = stands.front();
    const double dxy = std::hypot(s.x, s.y);
    const double dyaw = std::fabs(std::remainder(s.yaw, 2.0 * M_PI));
    if (dxy <= tol.xy_m && dyaw <= tol.yaw_rad) {
      v.ok = true;
      msg << "base stance reaches the plan (stand " << dxy << " m, " << dyaw << " rad away)";
    } else {
      msg << "base would have to move to x=" << s.x << " y=" << s.y << " yaw=" << s.yaw
          << " (" << dxy << " m, " << dyaw << " rad from here)";
    }
  }
  v.message = msg.str();
  return v;
}

}  // namespace manipulation_manager
