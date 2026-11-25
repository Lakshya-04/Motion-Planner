#include "reachability/core/lift_bands.hpp"

#include <algorithm>
#include <cmath>

namespace reachability
{

std::vector<double> heightRange(double lo, double hi, double step)
{
  std::vector<double> out;
  // Non-finite input or a huge count would loop (or overflow the int below): no heights.
  constexpr double kMaxHeights = 100000.0;
  if (!std::isfinite(lo) || !std::isfinite(hi) || !std::isfinite(step) || step <= 0.0 || hi < lo ||
    !((hi - lo) / step < kMaxHeights))
  {
    return out;
  }
  const int n = static_cast<int>(std::floor((hi - lo) / step + 1e-9));
  for (int i = 0; i <= n; ++i) {
    out.push_back(lo + i * step);
  }
  if (out.back() < hi - 1e-9) {
    out.push_back(hi);
  }
  return out;
}

std::vector<std::vector<bool>> reachableByHeight(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & heights, double arm_mount_offset_z, const ReachLimit & limit)
{
  const bool cap = limit.active() && limit.max_fraction > 0.0;
  std::vector<std::vector<bool>> out(targets_world.size(), std::vector<bool>(heights.size(), false));
  for (size_t i = 0; i < targets_world.size(); ++i) {
    for (size_t h = 0; h < heights.size(); ++h) {
      // Base yaw is free (the arm's first joint absorbs it), so a base at yaw 0 is enough.
      Pose in_base = targets_world[i];
      in_base.translation() -= Vec3(base_x, base_y, heights[h] + arm_mount_offset_z);
      out[i][h] = map.isReachable(in_base) &&
        (!cap || in_base.translation().norm() <= limit.max_fraction * limit.reach_m + 1e-9);
    }
  }
  return out;
}

LiftBandPlan planLiftBands(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & candidate_heights, double arm_mount_offset_z, const ReachLimit & limit)
{
  LiftBandPlan plan;
  const auto reach = reachableByHeight(map, targets_world, base_x, base_y, candidate_heights, arm_mount_offset_z, limit);
  const bool active = limit.active();
  // tool distance over reach for target i with the lift at height h
  auto fraction = [&](size_t i, size_t h) {
      const Vec3 d = targets_world[i].translation() - Vec3(base_x, base_y, candidate_heights[h] + arm_mount_offset_z);
      return d.norm() / limit.reach_m;
    };

  std::vector<bool> covered(targets_world.size(), false);
  for (size_t i = 0; i < targets_world.size(); ++i) {
    bool any = false;
    for (bool r : reach[i]) {any = any || r;}
    if (!any) {
      plan.unreachable.push_back(i);
      covered[i] = true;  // nothing can cover it: leave it out of the cover problem
    }
  }
  plan.reachable_count = targets_world.size() - plan.unreachable.size();

  while (true) {
    size_t best_h = 0, best_gain = 0;
    double best_worst = 0.0;
    for (size_t h = 0; h < candidate_heights.size(); ++h) {
      size_t gain = 0;
      double worst = 0.0;
      for (size_t i = 0; i < targets_world.size(); ++i) {
        if (!covered[i] && reach[i][h]) {
          ++gain;
          if (limit.balance && active) {worst = std::max(worst, fraction(i, h));}
        }
      }
      // Ties go to the lower height: the slow axis travels less. With the balance option a tie goes to the height
      // that keeps the tool farthest from full stretch (a clearly smaller worst fraction, 0.01 of the reach).
      if (gain > best_gain || (gain > 0 && gain == best_gain && limit.balance && active && worst < best_worst - 0.01)) {
        best_gain = gain;
        best_h = h;
        best_worst = worst;
      }
    }
    if (best_gain == 0) {
      break;
    }
    LiftBand band;
    band.lift_height = candidate_heights[best_h];
    for (size_t i = 0; i < targets_world.size(); ++i) {
      if (!covered[i] && reach[i][best_h]) {
        covered[i] = true;
        band.pose_indices.push_back(i);
      }
    }
    plan.bands.push_back(std::move(band));
  }
  std::sort(plan.bands.begin(), plan.bands.end(),
    [](const LiftBand & a, const LiftBand & b) {return a.lift_height < b.lift_height;});
  if (active) {
    for (const auto & b : plan.bands) {
      const Vec3 base(base_x, base_y, b.lift_height + arm_mount_offset_z);
      for (const size_t i : b.pose_indices) {
        plan.max_reach_fraction = std::max(plan.max_reach_fraction, (targets_world[i].translation() - base).norm() / limit.reach_m);
      }
    }
  }
  return plan;
}

}  // namespace reachability
