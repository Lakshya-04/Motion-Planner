/**
 * @file reachability_column_ik_port.hpp
 * @brief ColumnIkCheckPort backed by reachability::ColumnIkChain over an injected IK.
 * @details ROS-free; lives in `manipulation_manager_column_reach` with the other reachability-backed adapters. Per column it builds
 *          the pose sequence (approach, [contact], strokes, retract) from the dock parameters with
 *          `reachability::buildColumnSequence`, runs `ColumnIkChain` (each pose seeded from the previous solution, plus the
 *          folded seeds in the chain parameters) and reports whether the chain is continuous. The IK is whatever the node
 *          injects (the planner's `IkPort` in the base frame), so the answer is a property of that IK, not of an arm model.
 *
 *          The joints are those of the planning group, lift first: `prismatic_joints` (default {0}) are the lift, the joints in
 *          `continuous_joints` are unbounded revolutes (their step is wrapped), the rest limited revolutes. The chain size is the
 *          seed's size on every call. `reachability`'s thresholds (rad per metre of tool jump) are starting values, not calibrated.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "manipulation_manager/core/column_ik_gate.hpp"
#include "reachability/core/column_ik_chain.hpp"
#include "reachability/core/column_sequence.hpp"

namespace manipulation_manager
{

struct ReachabilityColumnIkConfig
{
  /// Dock geometry (as `ReachabilityColumnConfig`); `wall_normal` is overwritten per call from `free_side_normal`.
  reachability::SequenceParams sequence;
  /// Thresholds, branch joints, folded seeds. `joint_kinds` and `initial_seed` are set per call.
  reachability::ColumnIkChainParams chain;
  std::vector<size_t> prismatic_joints{0};
  std::vector<size_t> continuous_joints;
};

class ReachabilityColumnIkPort : public ColumnIkCheckPort
{
public:
  /// Joint solution for the pose (the manager's frame) from `seed`, or nullopt.
  using IkFn = std::function<std::optional<std::vector<double>>(const core_types::Pose &, const std::vector<double> & seed)>;

  ReachabilityColumnIkPort(ReachabilityColumnIkConfig config, IkFn ik);

  ColumnIkVerdict check(
    const std::vector<core_types::Pose> & strokes, const std::array<double, 3> & free_side_normal,
    const std::vector<double> & seed) override;

private:
  ReachabilityColumnIkConfig config_;
  IkFn ik_;
};

}  // namespace manipulation_manager
