/**
 * @file path_quality_port.hpp
 * @brief Port: how close the executed joint paths came to a singularity or a joint limit (report only).
 * @details ROS-free and free of the reachability package, like `column_reach_port.hpp`. The manager records the joint state
 *          while a column strokes (when `verify_path_quality` is on) and hands one stroke per column to an implementation
 *          after the run; the reachability-backed one (`adapters/reachability_path_quality_port.hpp`) calls
 *          `reachability::auditPathQuality`. The result goes into the verify result as a warning; it never fails a run and
 *          never changes a path. An implementation that cannot answer returns `available = false`.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace manipulation_manager
{

/// The joint states sampled along one column's stroke, in time order (group joint order, lift first).
struct PathQualityStroke
{
  size_t column{0};
  std::vector<std::vector<double>> waypoints;
};

struct ColumnPathQuality
{
  size_t column{0};
  std::string reason;            ///< "ok" | "near_singular" | "near_limit"
  double min_sigma{-1.0};        ///< smallest Jacobian singular value over the stroke; -1 = not evaluated
  double min_limit_margin{-1.0}; ///< smallest joint-limit margin (share of range, 0..0.5); -1 = not evaluated
  int worst_index{-1};           ///< waypoint index that caused the reason
};

struct PathQualitySummary
{
  bool available{false};         ///< false when the port could not audit (no model, no waypoints)
  std::string note;              ///< why it is unavailable
  std::vector<ColumnPathQuality> columns;
  int near_singular{0};
  int near_limit{0};
  double min_sigma{-1.0};
  double min_limit_margin{-1.0};
  double sigma_threshold{0.0};   ///< thresholds used, for the report
  double limit_margin_threshold{0.0};
};

class PathQualityPort
{
public:
  virtual ~PathQualityPort() = default;
  virtual PathQualitySummary audit(const std::vector<PathQualityStroke> & strokes) = 0;
};

}  // namespace manipulation_manager
