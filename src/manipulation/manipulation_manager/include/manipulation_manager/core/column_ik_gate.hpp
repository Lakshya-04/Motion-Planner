/**
 * @file column_ik_gate.hpp
 * @brief Whole-column IK continuity check before a column is run, with a first escalation (another lift height).
 * @details ROS-free, header-only, free of the reachability package. A column can be reachable pose by pose and still have no
 *          continuous joint path: IK solved pose by pose may land on another arm branch halfway (elbow, wrist, lift jump), and
 *          the stroke then truncates or the dock fails (the Gazebo two-stand failure of 2026-11-21). `ColumnIkCheckPort`
 *          answers "is there a continuous IK chain along this column's approach, strokes and retract from this seed";
 *          `ColumnIkGate` calls it and, on a break, tries the same column from other lift heights (the lift is locked while a
 *          column runs, and a different lift height moves the arm to another branch family). The reachability-backed port is
 *          `adapters/reachability_column_ik_port.hpp` (`ColumnIkChain` over the planner's IK).
 *
 *          Escalation implemented: the given seed, then each `lift_candidates` height (the lift entry of the seed replaced).
 *          A continuous chain from a candidate is returned as `kGoWithSeed` with that seed (the node uses it as the dock IK
 *          seed). Not implemented (planned, docs/development_plan.md section 4): re-standing, re-ordering columns, splitting
 *          the column at the break and running the pieces from another stand. A column whose every attempt breaks is
 *          `kReject`; one whose IK is refused at some pose in every attempt is let through (`kGo`, warning) unless
 *          `reject_on_refused`, because the planner's own pose-goal search may still find a way the single IK call did not.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <string>
#include <vector>

#include "core_types/pose.hpp"

namespace manipulation_manager
{

constexpr size_t kNoIkPose = std::numeric_limits<size_t>::max();

struct ColumnIkVerdict
{
  bool solved{false};               ///< every pose has an IK solution
  bool continuous{false};           ///< solved and no branch break
  size_t split_count{0};            ///< steps flagged as branch breaks
  size_t branch_changes{0};
  double max_continuity_ratio{0.0};
  double lift_travel_m{0.0};
  std::string failure;              ///< "" | "branch_break" | "refused"
  size_t failure_pose{kNoIkPose};   ///< index into the sequence (approach, strokes, retract)
  std::string failure_role;         ///< "approach" | "contact" | "stroke" | "retract"
};

class ColumnIkCheckPort
{
public:
  virtual ~ColumnIkCheckPort() = default;
  /// Checks the column's stroke poses (the frame the manager plans in) with approach and retract poses added along
  /// `free_side_normal` (pointing from the wall back towards the robot, same frame), starting the IK at `seed`
  /// (the planning group's joints, lift first).
  virtual ColumnIkVerdict check(
    const std::vector<core_types::Pose> & strokes, const std::array<double, 3> & free_side_normal,
    const std::vector<double> & seed) = 0;
};

struct ColumnIkGateConfig
{
  size_t lift_joint{0};                     ///< index of the lift in the seed
  std::vector<double> lift_candidates;      ///< other lift heights to try on a break; empty = none
  bool reject_on_refused{false};
  bool reject_on_break{true};
};

enum class ColumnIkAction
{
  kNotChecked,   ///< no port, no strokes or no seed
  kGo,           ///< continuous from the given seed (or only refused poses and not rejecting them)
  kGoWithSeed,   ///< continuous from another lift height: use `seed`
  kReject,       ///< every attempt breaks: do not run this column from this stand as is
};

struct ColumnIkDecision
{
  ColumnIkAction action{ColumnIkAction::kNotChecked};
  std::vector<double> seed;          ///< the seed that gave the continuous chain (kGoWithSeed), else the given one
  ColumnIkVerdict verdict;           ///< the verdict of the deciding attempt (the best one for kReject)
  size_t attempts{0};
  std::string reason;                ///< a one-line explanation for the log
};

inline const char * toString(ColumnIkAction a)
{
  switch (a) {
    case ColumnIkAction::kNotChecked: return "not_checked";
    case ColumnIkAction::kGo: return "go";
    case ColumnIkAction::kGoWithSeed: return "go_with_seed";
    case ColumnIkAction::kReject: return "reject";
  }
  return "not_checked";
}

class ColumnIkGate
{
public:
  explicit ColumnIkGate(ColumnIkGateConfig config = {}) : config_(std::move(config)) {}

  ColumnIkDecision decide(
    ColumnIkCheckPort * port, const std::vector<core_types::Pose> & strokes,
    const std::array<double, 3> & free_side_normal, const std::vector<double> & seed) const
  {
    ColumnIkDecision d;
    d.seed = seed;
    if (port == nullptr || strokes.empty() || seed.empty()) {
      d.reason = port == nullptr ? "no IK check port" : (strokes.empty() ? "no strokes" : "no seed");
      return d;
    }
    ColumnIkVerdict best = port->check(strokes, free_side_normal, seed);
    d.attempts = 1;
    d.verdict = best;
    if (best.continuous) {
      d.action = ColumnIkAction::kGo;
      d.reason = "continuous IK chain from the current seed";
      return d;
    }
    bool any_break = best.failure == "branch_break";
    for (const double lift : config_.lift_candidates) {
      if (config_.lift_joint >= seed.size()) {
        break;
      }
      if (!std::isfinite(lift)) {
        continue;   // never hand a NaN / inf lift height to the IK or back as the dock seed
      }
      std::vector<double> s = seed;
      s[config_.lift_joint] = lift;
      const ColumnIkVerdict v = port->check(strokes, free_side_normal, s);
      ++d.attempts;
      if (v.continuous) {
        d.action = ColumnIkAction::kGoWithSeed;
        d.seed = s;
        d.verdict = v;
        d.reason = "branch break from the current seed, continuous from lift " + std::to_string(lift) + " m";
        return d;
      }
      any_break = any_break || v.failure == "branch_break";
      if (betterThan(v, best)) {
        best = v;
        d.verdict = v;
      }
    }
    if (any_break) {
      d.action = config_.reject_on_break ? ColumnIkAction::kReject : ColumnIkAction::kGo;
      d.reason = "IK chain breaks at pose " + std::to_string(d.verdict.failure_pose) + " (" + d.verdict.failure_role +
        ") from every seed tried (" + std::to_string(d.attempts) + ")";
    } else {
      d.action = config_.reject_on_refused ? ColumnIkAction::kReject : ColumnIkAction::kGo;
      d.reason = "IK refused pose " + std::to_string(d.verdict.failure_pose) + " (" + d.verdict.failure_role +
        ") from every seed tried (" + std::to_string(d.attempts) + ")" +
        (config_.reject_on_refused ? "" : "; letting the planner try");
    }
    return d;
  }

private:
  /// Fewer breaks, then a later failure pose, then a lower worst ratio.
  static bool betterThan(const ColumnIkVerdict & a, const ColumnIkVerdict & b)
  {
    if (a.split_count != b.split_count) {return a.split_count < b.split_count;}
    if (a.solved != b.solved) {return a.solved;}
    return a.max_continuity_ratio < b.max_continuity_ratio;
  }

  ColumnIkGateConfig config_;
};

}  // namespace manipulation_manager
