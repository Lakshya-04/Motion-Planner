/**
 * @file reachability_column_port.hpp
 * @brief ColumnReachabilityPort backed by the reachability package's RM4D map.
 * @details ROS-free. Lives in its own library (`manipulation_manager_column_reach`) so that
 *          `manipulation_manager_core` stays free of the reachability package: only the node, and the tests of this
 *          adapter, link it. reachability does not depend on manipulation_manager, so the dependency is one-way
 *          (manipulation_manager -> reachability) and there is no cycle.
 *
 *          Per column it builds the pose sequence (approach, contact, strokes, retract) from the dock parameters and
 *          checks it with reachability::evaluateColumn for the arm base position the StandProvider reports and every
 *          offered lift height. Poses are handed over as they are (the same convention as the base stance check,
 *          which sends the plan's poses to the reachability planner unchanged); their z is measured from the floor,
 *          like `arm_mount_offset_z` and the lift height.
 *
 *          The result is an RM4D-level estimate (no IK, no collision): optimistic, thresholds not calibrated.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "manipulation_manager/core/column_reach_port.hpp"
#include "reachability/core/column_sequence.hpp"
#include "reachability/core/map4d.hpp"
#include "reachability/core/pose_quality.hpp"

namespace manipulation_manager
{

struct ReachabilityColumnConfig
{
  /// Dock geometry. `wall_normal` points into the free side (back towards the robot: the negated wall direction).
  reachability::SequenceParams sequence;
  reachability::MarginConfig margin;
  /// Lift heights to try (the lift is locked while a column runs). Empty = {0}.
  std::vector<double> lift_heights;
  /// Height of the arm base above the floor with the lift at zero stroke.
  double arm_mount_offset_z{0.0};
};

/// Lift heights from `lift_min` to `lift_max` (both included), `step` apart or closer.
/// `lift_max <= lift_min` or `step <= 0` gives just {lift_min}.
std::vector<double> liftHeights(double lift_min, double lift_max, double step);

class ReachabilityColumnPort : public ColumnReachabilityPort
{
public:
  /// Where the arm base is, in the frame of the columns' poses (xy). nullopt = unknown: the port then answers
  /// with an empty vector and the analyzer treats the columns as not checked.
  using StandProvider = std::function<std::optional<Eigen::Vector2d>()>;

  ReachabilityColumnPort(
    std::shared_ptr<const reachability::Map4D> map, ReachabilityColumnConfig config, StandProvider stand);

  std::vector<ColumnReachVerdict> check(const std::vector<ColumnReachInput> & columns) override;

  /// The verdict of one column from the arm base at `base_xy` (what check() does per column).
  ColumnReachVerdict checkColumn(const ColumnReachInput & column, const Eigen::Vector2d & base_xy) const;

private:
  std::shared_ptr<const reachability::Map4D> map_;
  ReachabilityColumnConfig config_;
  StandProvider stand_;
};

}  // namespace manipulation_manager
