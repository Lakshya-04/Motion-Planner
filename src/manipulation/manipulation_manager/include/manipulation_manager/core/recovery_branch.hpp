// Recovery that stays on the arm's current IK branch (ROS-free).
//
// A recovery after a short approach used to move the arm toward last_good_state_ (the configuration of an earlier
// dock, possibly another branch) and the next attempt then planned a pose goal the planner solves from its own
// samples. Either can land on another branch (shoulder pan and elbow flip sign, the wrist winds). Two decisions
// here keep the arm where it is:
//
//   selectRecoveryJogTarget  the configuration to jog toward: the last good one when it is on the live branch,
//                            else the last docked one (branch seed) when that is, else none (stay put)
//   selectRecoveryGoal       the joint goal of the next attempt: dock_ik_selector's selectDockIk with the live
//                            state as reference, so a solution on another branch is rejected
//
// "Same branch" is dockBranchChanged() with the same parameters (dock_ik_selector.hpp, unchanged). Nothing here
// plans or runs IK itself; with no acceptable candidate ok is false and the caller keeps its previous behaviour.
#pragma once

#include <cstddef>
#include <vector>

#include "manipulation_manager/core/dock_ik_selector.hpp"

namespace manipulation_manager
{

struct RecoveryTarget
{
  bool ok{false};
  std::vector<double> q;
  bool from_branch_seed{false};   ///< the branch seed was used because last_good was on another branch
  bool last_good_rejected{false};
};

/// True when `q` can serve as a recovery configuration for `live`: same size, same branch, wrist inside its
/// limits, every joint within max_dq of live.
inline bool recoveryConfigOnBranch(
  const std::vector<double> & q, const std::vector<double> & live, const DockIkSelectorParams & p)
{
  if (q.empty() || q.size() != live.size()) {
    return false;
  }
  if (dockBranchChanged(q, live, p)) {
    return false;
  }
  if (p.wrist_joint >= 0 && static_cast<size_t>(p.wrist_joint) < q.size()) {
    const double w = q[static_cast<size_t>(p.wrist_joint)];
    if (w < p.wrist_min_rad || w > p.wrist_max_rad) {
      return false;
    }
  }
  return dockIkCost(q, live, p, false) <= p.max_dq_rad;
}

/// The configuration to jog toward from `live`, on the live branch: last_good, else branch_seed; none -> ok false.
inline RecoveryTarget selectRecoveryJogTarget(
  const std::vector<double> & live, const std::vector<double> & last_good,
  const std::vector<double> & branch_seed, const DockIkSelectorParams & p)
{
  RecoveryTarget out;
  if (live.empty()) {
    return out;
  }
  if (recoveryConfigOnBranch(last_good, live, p)) {
    out.ok = true;
    out.q = last_good;
    return out;
  }
  out.last_good_rejected = !last_good.empty();
  if (recoveryConfigOnBranch(branch_seed, live, p)) {
    out.ok = true;
    out.q = branch_seed;
    out.from_branch_seed = true;
  }
  return out;
}

/// The joint goal for the attempt after a recovery: IK from the live state and the branch seed, best solution on
/// the live branch (reference = live). Not ok when the IK finds nothing acceptable.
inline DockIkChoice selectRecoveryGoal(
  const DockIkFn & ik, const std::vector<double> & live, const std::vector<double> & branch_seed,
  const DockIkSelectorParams & p)
{
  return selectDockIk(ik, {live, branch_seed}, live, p);
}

}  // namespace manipulation_manager
