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
// Next-best distinct candidate (column retry): with a DockIkExclusion the solutions already used (the ones a failed
// attempt docked with) are dropped when they lie within tol_rad (max wrapped |dq|) of a candidate, so the winner is
// the cheapest candidate that differs from them; with allow_branch_change the branch test is skipped too, which lets
// the other IK branch through (the wrist and cost limits still apply). No exclusion: nothing changes.
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
};

/// Where the tool goal lies relative to the arm base, for the reach preference.
struct DockReachContext
{
  double horizontal_m{0.0};   ///< horizontal distance arm base -> tool goal
  double z_rel_m{0.0};        ///< tool goal height minus the arm base height at zero lift
};

/// Solutions to stay away from (column retry): see the header comment.
struct DockIkExclusion
{
  std::vector<std::vector<double>> used;   ///< earlier solutions; a candidate within tol_rad of one is dropped
  double tol_rad{0.05};
  bool allow_branch_change{false};
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
  size_t rejected_excluded{0};   ///< candidates equal (within tol) to an already used solution
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

inline bool allFiniteJoints(const std::vector<double> & q)
{
  return std::all_of(q.begin(), q.end(), [](double x) {return std::isfinite(x);});
}

/// max_i |dq_i| between `q` and `ref`, wrapping every joint that is not linear. Infinity on a size mismatch or a non-finite joint.
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
    if (std::isnan(d)) {
      return INFINITY;   // std::max(worst, NaN) keeps `worst`, so a NaN joint would look free
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

/// The configuration the branch check and the cost of a dock are measured against: the previous dock when there is
/// one; for the first dock of a stand the configured dock seed (the arm branch known to dock at the wall) when it
/// has the joint count of the live state, else the live state. Measuring the first dock against the live state (the
/// arm stowed, elbow folded one way) rejects the configured branch as a "branch change" and keeps the stow branch,
/// which for a low target (tall wall, tool well below the arm base) puts the upper arm into the lift carriage.
inline const std::vector<double> & dockBranchReference(
  const std::vector<double> & previous_dock, const std::vector<double> & dock_seed, const std::vector<double> & live)
{
  if (!previous_dock.empty()) {
    return previous_dock;
  }
  if (!dock_seed.empty() && dock_seed.size() == live.size()) {
    return dock_seed;
  }
  return live;
}

/// Runs `ik` from every seed (empty seeds are skipped) and returns the cheapest acceptable solution relative to
/// `reference`. Ties go to the earlier seed. With `reach` set and the preference on (see the header comment) the
/// seeds are followed by one lift-adjusted copy of each, and candidates at or under the target reach fraction win.
inline DockIkChoice selectDockIk(
  const DockIkFn & ik, const std::vector<std::vector<double>> & seeds_in, const std::vector<double> & reference,
  const DockIkSelectorParams & p, const DockReachContext * reach = nullptr,
  const DockIkExclusion * exclusion = nullptr)
{
  DockIkChoice best;
  if (!ik || reference.empty() || !allFiniteJoints(reference)) {
    return best;
  }
  const bool prefer = reach != nullptr && dockReachPreferenceOn(p) && p.lift_joint < reference.size();
  std::vector<std::vector<double>> seeds = seeds_in;
  if (prefer) {
    for (const auto & sd : seeds_in) {
      if (sd.size() == reference.size()) {
        auto adj = sd;
        adj[p.lift_joint] = idealLift(sd[p.lift_joint], *reach, p);
        if (std::isfinite(adj[p.lift_joint])) {
          seeds.push_back(std::move(adj));
        }
      }
    }
    best.reach_fraction = INFINITY;
  }
  bool best_under = false;
  for (size_t s = 0; s < seeds.size(); ++s) {
    if (seeds[s].empty() || !allFiniteJoints(seeds[s])) {
      continue;   // a NaN seed would send the IK nowhere
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
    if (exclusion != nullptr) {
      bool seen = false;
      for (const auto & u : exclusion->used) {
        if (dockIkCost(*sol, u, p) <= exclusion->tol_rad) {
          seen = true;
          break;
        }
      }
      if (seen) {
        ++best.rejected_excluded;
        continue;
      }
    }
    if (!(exclusion != nullptr && exclusion->allow_branch_change) && dockBranchChanged(*sol, reference, p)) {
      ++best.rejected_branch;
      continue;
    }
    const double cost = dockIkCost(*sol, reference, p);
    if (!(cost <= p.max_dq_rad)) {   // also rejects a NaN cost or limit
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
