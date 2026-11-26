/**
 * @file coverage_plan_intents.hpp
 * @brief ROS-free conversion of a coverage plan (poses + stroke index ranges) into CoverageSegmentIntent.
 * @details Both CoverageSource implementations feed this one function, so the JSON file and the ROS topic give
 *          the same intents for the same plan: CoverageOutputJsonSource parses the file into a RawCoveragePlan,
 *          RosCoverageSource copies a coverage_planner_msgs/CoveragePlan into one. RawCoveragePlan is a plain struct
 *          with the fields of that message that the manager uses (success, message, frame, path poses, segment
 *          start/end indices), so host gtests can build a "message" by hand without any *_msgs header.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "core_types/pose.hpp"
#include "manipulation_manager/core/coverage_source.hpp"
#include "manipulation_manager/core/wall_direction.hpp"

namespace manipulation_manager
{

/// One stroke of the plan: inclusive indices into RawCoveragePlan::poses (MotionPlanSegment start/end_index).
struct RawPlanSegment
{
  size_t start_index{0};
  size_t end_index{0};
};

/// The plan as the planner emits it (coverage_planner_msgs/CoveragePlan, or final_lifted_and_expanded in the JSON).
struct RawCoveragePlan
{
  bool success{true};
  std::string message;
  std::string frame_id;                  ///< Frame of the path (informational; z_offset commits to the target frame).
  std::vector<core_types::Pose> poses;   ///< Path poses in plan order.
  std::vector<RawPlanSegment> segments;  ///< Optional; empty = no stroke information.
};

/// The settings the manager passes to either source (the meaning of each is documented on CoverageOutputJsonSource).
struct CoverageIntentOptions
{
  double z_offset{0.0};
  double z_step_m{0.1};
  double z_min_m{1.20};
  double z_max_m{1.60};
  double target_eef_speed{0.02};
  WallDirection wall_dir{};
  bool follow_plan_direction{false};
};

/// Throws std::runtime_error when the plan reports failure, has no poses, or a segment's indices are out of range.
void validateCoveragePlan(const RawCoveragePlan & plan);

/// A deterministic text of the plan's poses and segments (6 decimals); the mission ledger hashes it as the plan
/// identity where there is no file to hash.
std::string coveragePlanFingerprint(const RawCoveragePlan & plan);

/// Group the points into wall columns, split them into strokes, resample each at z_step_m inside [z_min_m, z_max_m].
std::vector<CoverageSegmentIntent> buildCoverageIntents(
  const RawCoveragePlan & plan, const CoverageIntentOptions & options);

/// The latest plan received, handed from a subscription callback to the code that waits for it.
class CoveragePlanInbox
{
public:
  void put(RawCoveragePlan plan);
  bool has() const;
  /// Calls `pump` (e.g. one spin of the node) and checks for a plan until one is there or `timeout_s` has passed.
  /// Returns false on timeout. The plan stays in the inbox (a later call returns the same plan until a new one arrives).
  bool waitFor(double timeout_s, const std::function<void()> & pump, RawCoveragePlan & out);

private:
  mutable std::mutex mutex_;
  RawCoveragePlan plan_;
  bool have_{false};
};

}  // namespace manipulation_manager
