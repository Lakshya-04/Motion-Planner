// Choice of the dock joint goal among several IK solutions (ROS-free).
//
// A per-pose IK lands on whatever branch its seed leads to: the first dock ends in one shoulder branch, the next
// one jumps to the folded-back branch and the wrist winds past a turn. Here the IK is run once per seed (the previous
// dock, the live state, folded seeds), every solution is measured against a reference configuration (the previous
// dock, else the live state) and the cheapest acceptable one wins:
//
//   cost     max_i |dq_i| with the joints that are not `linear_joints` wrapped to [-pi, pi]
//   rejected a branch change (the sign of a `branch_joints` entry flips, a value inside the deadband counts as
//            either sign), a wrist joint outside [wrist_min_rad, wrist_max_rad] (winding), a cost over max_dq
//
// Same idea as reachability::ColumnIkChain (reseed, wrapped dq, branch signature) for one pose at a time; this
// header does not depend on that package so it can be tested alone. No acceptable candidate: ok is false and the
// caller keeps its previous behaviour. Not an IK, not a planner: it only chooses among what the IK returns.
//
// Reach preference (opt-in, reach_target_fraction > 0 with reach_m > 0 and a DockReachContext): with the lift free, the
// IK often docks with the lift near its start and the tool far out on the arm (87% of reach on a tall wall). The
// tool-to-arm-base distance of a candidate is sqrt(h^2 + (z_rel - lift)^2), h being the horizontal distance from the
// arm base to the tool and z_rel the tool height above the arm base at zero lift; its reach fraction is that over
// reach_m. Every seed is then also tried with its lift entry moved to the lift nearest the reference lift that keeps
// the fraction at or under the target (idealLift), and a candidate at or under the target beats any above it whatever
// its cost; the cost orders candidates on the same side of the target. No candidate under the target: the cheapest
// one as without the preference (reach_over_target is set). Off by default; nothing here changes then.
//
// Joint-limit margin (opt-in, min_limit_margin_rad > 0 with lower/upper of the solution's size): a solution with a
// limited non-linear joint (finite range under limit_range_cutoff_rad, so the +-2 pi joints never count) closer than the margin
// to a limit is rejected (rejected_limit). A seed that already sits inside the margin (the arm's ready pose can) is
// also tried with those joints moved toward the middle of their range (relief seeds, appended after the others), so
// the IK can leave the stop through the null space of a redundant arm. Reason: the xArm 7 docked with joint 2 on
// its lower stop (0.4 mrad inside it) and pressed 25-67 N during the contact stroke, while docks with 0.1 rad or more
// of margin stayed at 25 N or less. The linear_joints (the lift, which the dock may park on its stop) are not checked.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

namespace manipulation_manager
{

struct DockIkSelectorParams
{
  std::vector<size_t> linear_joints{0};     ///< prismatic joints (the lift): dq is not wrapped
  std::vector<size_t> branch_joints;        ///< joints whose sign is the branch (e.g. shoulder lift); empty = not tracked
  double branch_deadband_rad{0.05};
  int wrist_joint{-1};                      ///< joint index whose absolute value is limited; -1 = no check
  double wrist_min_rad{-3.2};
  double wrist_max_rad{3.2};
  double max_dq_rad{3.2};                   ///< candidates with a larger cost are rejected
  // Reach preference (see the header comment); reach_target_fraction <= 0 or reach_m <= 0 = off.
  double reach_target_fraction{0.0};        ///< wanted upper bound of tool distance / reach_m
  double reach_m{0.0};                      ///< arm reach from the arm base frame origin (m)
  size_t lift_joint{0};                     ///< index of the lift joint in a solution
  double lift_min_m{0.0};                   ///< lift stroke limits the proposed lift is clamped into
  double lift_max_m{1.5};
  // Joint-limit margin (see the header comment); min_limit_margin_rad <= 0 or limits of another size = off.
  double min_limit_margin_rad{0.0};         ///< rejected: a limited joint closer than this to a limit
  std::vector<double> lower;                ///< joint limits, same order as a solution
  std::vector<double> upper;
  double limit_range_cutoff_rad{6.0};       ///< joints with a range above this (continuous-like) are not checked
};

/// Where the tool goal lies relative to the arm base, for the reach preference.
struct DockReachContext
{
  double horizontal_m{0.0};   ///< horizontal distance arm base -> tool goal
  double z_rel_m{0.0};        ///< tool goal height minus the arm base height at zero lift
};

struct DockIkChoice
{
  bool ok{false};
  std::vector<double> q;
  size_t seed_index{0};          ///< which seed produced it
  double cost{0.0};
  size_t candidates{0};          ///< IK solutions obtained
  size_t rejected_branch{0};
  size_t rejected_wrist{0};
  size_t rejected_cost{0};
  size_t rejected_limit{0};
  double limit_margin{-1.0};     ///< smallest margin of the choice when the check is on, else -1
  double reach_fraction{-1.0};   ///< tool distance / reach_m of the choice; -1 when the preference is off
  bool reach_over_target{false}; ///< preference on, but no acceptable candidate was at or under the target
};

/// IK solution for the target starting at `seed`, nullopt when there is none.
using DockIkFn = std::function<std::optional<std::vector<double>>(const std::vector<double> & seed)>;

inline double wrapToPi(double a)
{
  const double two_pi = 2.0 * M_PI;
  a = std::fmod(a + M_PI, two_pi);
  if (a < 0.0) {
    a += two_pi;
  }
  return a - M_PI;
}

/// max_i |dq_i| between `q` and `ref`, wrapping every joint that is not linear. Infinity on a size mismatch.
inline double dockIkCost(
  const std::vector<double> & q, const std::vector<double> & ref, const DockIkSelectorParams & p)
{
  if (q.size() != ref.size()) {
    return INFINITY;
  }
  double worst = 0.0;
  for (size_t i = 0; i < q.size(); ++i) {
    double d = q[i] - ref[i];
    if (std::find(p.linear_joints.begin(), p.linear_joints.end(), i) == p.linear_joints.end()) {
      d = wrapToPi(d);
    }
    worst = std::max(worst, std::fabs(d));
  }
  return worst;
}

/// True when some branch joint is clearly on opposite sides of zero in `a` and `b`.
inline bool dockBranchChanged(
  const std::vector<double> & a, const std::vector<double> & b, const DockIkSelectorParams & p)
{
  for (const size_t i : p.branch_joints) {
    if (i >= a.size() || i >= b.size()) {
      continue;
    }
    if (std::fabs(a[i]) > p.branch_deadband_rad && std::fabs(b[i]) > p.branch_deadband_rad &&
      (a[i] > 0.0) != (b[i] > 0.0))
    {
      return true;
    }
  }
  return false;
}

/// A joint the margin does not apply to: linear, without a range, or with a continuous-like range.
inline bool dockLimitSkipped(size_t i, const DockIkSelectorParams & p)
{
  return std::find(p.linear_joints.begin(), p.linear_joints.end(), i) != p.linear_joints.end() ||
         !(p.upper[i] > p.lower[i]) || p.upper[i] - p.lower[i] > p.limit_range_cutoff_rad;
}

inline bool dockLimitCheckOn(const DockIkSelectorParams & p, size_t n)
{
  return p.min_limit_margin_rad > 0.0 && p.lower.size() == n && p.upper.size() == n;
}

/// Smallest distance of a limited joint of `q` to one of its limits (negative when outside). Infinity when no joint
/// is limited or the check is off for this size.
inline double dockLimitMargin(const std::vector<double> & q, const DockIkSelectorParams & p)
{
  double m = INFINITY;
  if (p.lower.size() != q.size() || p.upper.size() != q.size()) {
    return m;
  }
  for (size_t i = 0; i < q.size(); ++i) {
    if (dockLimitSkipped(i, p)) {
      continue;
    }
    m = std::min(m, std::min(q[i] - p.lower[i], p.upper[i] - q[i]));
  }
  return m;
}

/// `seed` with every limited joint inside the margin moved to the middle of its range; same vector when none is.
inline std::vector<double> dockReliefSeed(const std::vector<double> & seed, const DockIkSelectorParams & p)
{
  auto out = seed;
  if (p.lower.size() != seed.size() || p.upper.size() != seed.size()) {
    return out;
  }
  for (size_t i = 0; i < seed.size(); ++i) {
    if (dockLimitSkipped(i, p)) {
      continue;
    }
    if (std::min(seed[i] - p.lower[i], p.upper[i] - seed[i]) < p.min_limit_margin_rad) {
      out[i] = 0.5 * (p.lower[i] + p.upper[i]);
    }
  }
  return out;
}

inline bool dockReachPreferenceOn(const DockIkSelectorParams & p)
{
  return p.reach_target_fraction > 0.0 && p.reach_m > 0.0;
}

/// Tool distance from the arm base over reach_m for the lift value `lift`. Infinity without a reach.
inline double dockReachFraction(double lift, const DockReachContext & c, const DockIkSelectorParams & p)
{
  if (p.reach_m <= 0.0) {
    return INFINITY;
  }
  const double dz = c.z_rel_m - lift;
  return std::sqrt(c.horizontal_m * c.horizontal_m + dz * dz) / p.reach_m;
}

/// The lift value nearest `current` that keeps the reach fraction at or under the target, clamped into
/// [lift_min_m, lift_max_m]. When the horizontal distance alone is over the target no lift can help, and the lift
/// that puts the tool level with the arm base (the smallest fraction) is returned, clamped.
inline double idealLift(double current, const DockReachContext & c, const DockIkSelectorParams & p)
{
  const double lo = std::min(p.lift_min_m, p.lift_max_m);
  const double hi = std::max(p.lift_min_m, p.lift_max_m);
  const double r = p.reach_target_fraction * p.reach_m;
  const double dz2 = r * r - c.horizontal_m * c.horizontal_m;
  double v = c.z_rel_m;
  if (dz2 > 0.0) {
    const double dz = std::sqrt(dz2);
    v = std::min(std::max(current, c.z_rel_m - dz), c.z_rel_m + dz);
  }
  return std::min(std::max(v, lo), hi);
}

/// Runs `ik` from every seed (empty seeds are skipped) and returns the cheapest acceptable solution relative to
/// `reference`. Ties go to the earlier seed. With `reach` set and the preference on (see the header comment) the
/// seeds are followed by one lift-adjusted copy of each, and candidates at or under the target reach fraction win.
inline DockIkChoice selectDockIk(
  const DockIkFn & ik, const std::vector<std::vector<double>> & seeds_in, const std::vector<double> & reference,
  const DockIkSelectorParams & p, const DockReachContext * reach = nullptr)
{
  DockIkChoice best;
  if (!ik || reference.empty()) {
    return best;
  }
  const bool prefer = reach != nullptr && dockReachPreferenceOn(p) && p.lift_joint < reference.size();
  std::vector<std::vector<double>> seeds = seeds_in;
  if (prefer) {
    for (const auto & sd : seeds_in) {
      if (sd.size() == reference.size()) {
        auto adj = sd;
        adj[p.lift_joint] = idealLift(sd[p.lift_joint], *reach, p);
        seeds.push_back(std::move(adj));
      }
    }
    best.reach_fraction = INFINITY;
  }
  const bool limits = dockLimitCheckOn(p, reference.size());
  if (limits) {
    const size_t n_in = seeds.size();
    for (size_t s = 0; s < n_in; ++s) {
      if (seeds[s].size() == reference.size()) {
        auto rel = dockReliefSeed(seeds[s], p);
        if (rel != seeds[s]) {
          seeds.push_back(std::move(rel));
        }
      }
    }
  }
  bool best_under = false;
  for (size_t s = 0; s < seeds.size(); ++s) {
    if (seeds[s].empty()) {
      continue;
    }
    const auto sol = ik(seeds[s]);
    if (!sol || sol->size() != reference.size()) {
      continue;
    }
    ++best.candidates;
    if (p.wrist_joint >= 0 && static_cast<size_t>(p.wrist_joint) < sol->size()) {
      const double w = (*sol)[static_cast<size_t>(p.wrist_joint)];
      if (w < p.wrist_min_rad || w > p.wrist_max_rad) {
        ++best.rejected_wrist;
        continue;
      }
    }
    if (dockBranchChanged(*sol, reference, p)) {
      ++best.rejected_branch;
      continue;
    }
    double margin = -1.0;
    if (limits) {
      margin = dockLimitMargin(*sol, p);
      if (margin < p.min_limit_margin_rad) {
        ++best.rejected_limit;
        continue;
      }
    }
    const double cost = dockIkCost(*sol, reference, p);
    if (cost > p.max_dq_rad) {
      ++best.rejected_cost;
      continue;
    }
    bool under = true;
    double frac = -1.0;
    if (prefer) {
      frac = dockReachFraction((*sol)[p.lift_joint], *reach, p);
      under = frac <= p.reach_target_fraction + 1e-9;
    }
    // under the target beats over it; the cost orders the rest
    const bool better = !best.ok || (under && !best_under) || (under == best_under && cost < best.cost);
    if (better) {
      best.ok = true;
      best.q = *sol;
      best.seed_index = s;
      best.cost = cost;
      best.limit_margin = limits ? margin : -1.0;
      best.reach_fraction = prefer ? frac : -1.0;
      best_under = under;
    }
  }
  best.reach_over_target = prefer && best.ok && !best_under;
  if (prefer && !best.ok) {
    best.reach_fraction = -1.0;
  }
  return best;
}

}  // namespace manipulation_manager
