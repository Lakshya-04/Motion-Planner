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

/// Runs `ik` from every seed (empty seeds are skipped) and returns the cheapest acceptable solution relative to
/// `reference`. Ties go to the earlier seed.
inline DockIkChoice selectDockIk(
  const DockIkFn & ik, const std::vector<std::vector<double>> & seeds, const std::vector<double> & reference,
  const DockIkSelectorParams & p)
{
  DockIkChoice best;
  if (!ik || reference.empty()) {
    return best;
  }
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
    const double cost = dockIkCost(*sol, reference, p);
    if (cost > p.max_dq_rad) {
      ++best.rejected_cost;
      continue;
    }
    if (!best.ok || cost < best.cost) {
      best.ok = true;
      best.q = *sol;
      best.seed_index = s;
      best.cost = cost;
    }
  }
  return best;
}

}  // namespace manipulation_manager
