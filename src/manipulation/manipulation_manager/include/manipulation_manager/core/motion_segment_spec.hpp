/**
 * @file motion_segment_spec.hpp
 * @brief Per-segment description of a motion, including its own planning
 *        parameters.
 * @details This is where per-segment planning params (eef_step, target speed,
 *          contact/wall bounds, ...) live in deployment -- fed later from
 *          ToolProfile/coverage output (§5/§7 of
 *          implementation_plan_manipulation_architecture.md). Replaces the
 *          idea of a single global request-defaults side-channel, which
 *          cannot express per-segment overrides.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "core_types/pose.hpp"

namespace manipulation_manager
{

/**
 * @struct MotionSegmentSpec
 * @brief One segment of a segmented motion plan, with its own routing hints
 *        and planning parameters.
 */
struct MotionSegmentSpec
{
  std::string label;                     ///< Segment label (used for auto-routing and reporting).
  std::vector<size_t> waypoint_indices;  ///< Indices into the caller's waypoint list.
  std::string mode_hint;                 ///< "cartesian"|"fsp"|"" (empty = auto-route from label).
  std::string planner_hint;              ///< "" | explicit planner name (e.g. "ompl").

  /// Direct joint-space goal, as an alternative to waypoint_indices/poses.
  /// Empty = pose-based segment (today's behavior). See PlanningRequest.
  /// target_joint_state for the full rationale -- FSP-only, ignored by
  /// CartesianPlanner.
  std::vector<double> target_joint_state;

  /// Per-segment planning parameters -- see PlanningRequest for field meanings.
  double eef_step{0.0};
  double revolute_jump_threshold{0.0};
  double prismatic_jump_threshold{0.0};
  double min_path_fraction{0.5};
  double target_eef_speed{0.0};
  bool allow_target_contact{false};
  std::string target_collision_object_id;
  core_types::Pose wall_reference_pose;
  double max_wall_penetration{0.0};
};

}  // namespace manipulation_manager
