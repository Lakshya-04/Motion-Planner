/**
 * @file reachability_path_quality_port.hpp
 * @brief PathQualityPort backed by reachability::auditPathQuality over a chain model (report only).
 * @details ROS-free; lives in `manipulation_manager_column_reach` next to the column port, so the manager core stays free of the
 *          reachability package (the dependency is one-way, manipulation_manager -> reachability).
 *
 *          The chain model comes from a chain YAML (`reachability::loadChainYaml`, written by
 *          `reachability/scripts/urdf_to_chain.py`): revolute arm joints only. The joint samples the manager records are in
 *          the planning group's order, lift first, so `skip_leading_joints` (1 for a lift + arm group, 0 for a bare arm)
 *          drops the lift before the audit; a waypoint whose remaining length is not the chain's dof is not evaluated.
 *          The thresholds are `reachability::PathQualityConfig`'s starting values, not calibrated on hardware.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>

#include "manipulation_manager/core/path_quality_port.hpp"
#include "reachability/core/arm_model.hpp"
#include "reachability/core/path_quality_audit.hpp"

namespace manipulation_manager
{

class ReachabilityPathQualityPort : public PathQualityPort
{
public:
  ReachabilityPathQualityPort(
    std::shared_ptr<const reachability::ArmModel> arm, reachability::PathQualityConfig config,
    size_t skip_leading_joints);

  PathQualitySummary audit(const std::vector<PathQualityStroke> & strokes) override;

private:
  std::shared_ptr<const reachability::ArmModel> arm_;
  reachability::PathQualityConfig config_;
  size_t skip_;
};

}  // namespace manipulation_manager
