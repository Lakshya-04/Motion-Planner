/**
 * @file column_reach_port.hpp
 * @brief Port: "can the arm do this column from where the base stands", including approach and retract.
 * @details ROS-free and free of the reachability package: the manager's side of the column pose sequence check
 *          (reachability/core/column_sequence.hpp). An implementation holds the RM4D map, the stand (arm base
 *          position, lift range) and the dock parameters (`dock_standoff_m`, `dock_approach_extra_m`, press depth) and
 *          answers per column: whole, split (some strokes unusable) or unreachable, which strokes are usable, and the
 *          pose (approach, contact, stroke, retract) that limits it with the reason (out_of_reach, low_margin, refused,
 *          singular, joint_limit). CoverageFeasibilityAnalyzer::setColumnReachability takes one; the node can build the
 *          reachability-backed implementation later (docs/reachability.md, "Column pose sequences"). Nothing in the node
 *          uses this port yet.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "core_types/pose.hpp"

namespace manipulation_manager
{

/// One column of the plan: its stroke poses (the contact intent's poses), in order.
struct ColumnReachInput
{
  std::string label;
  std::vector<core_types::Pose> strokes;
};

enum class ColumnReachStatus
{
  kWhole,         ///< Every pose of the sequence is usable at one lift height.
  kSplit,         ///< End poses usable, only some strokes are: the column would be broken into pieces.
  kUnreachable,   ///< No stroke usable, or an end pose (approach, contact, retract) is not.
};

constexpr size_t kNoLimitingStroke = std::numeric_limits<size_t>::max();

struct ColumnReachVerdict
{
  ColumnReachStatus status{ColumnReachStatus::kWhole};
  std::vector<bool> stroke_usable;       ///< One per stroke; empty = all usable.
  std::string limiting_pose;             ///< "approach" | "contact" | "stroke" | "retract" ("" when whole).
  std::string limiting_reason;           ///< "out_of_reach" | "low_margin" | "refused" | "singular" | "joint_limit".
  size_t limiting_stroke{kNoLimitingStroke};
  double min_score{0.0};                 ///< Lowest margin score over the usable poses, [0, 1].

  bool strokeUsable(size_t i) const {return stroke_usable.empty() || (i < stroke_usable.size() && stroke_usable[i]);}
};

class ColumnReachabilityPort
{
public:
  virtual ~ColumnReachabilityPort() = default;
  /// One verdict per input column, same order. An implementation that cannot answer returns an empty vector;
  /// the analyzer then treats every column as not checked.
  virtual std::vector<ColumnReachVerdict> check(const std::vector<ColumnReachInput> & columns) = 0;
};

}  // namespace manipulation_manager
