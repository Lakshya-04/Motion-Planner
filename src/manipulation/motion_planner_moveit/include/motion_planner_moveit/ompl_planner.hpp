#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "motion_planner/core/direct_joint_path.hpp"
#include "motion_planner/core/path_quality.hpp"
#include "motion_planner/core/path_shortcut.hpp"
#include "motion_planner/core/planners/planner.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace moveit::planning_interface
{
class MoveGroupInterface;
}  // namespace moveit::planning_interface

namespace moveit::core
{
class RobotState;
}  // namespace moveit::core

namespace motion_planner
{

class MoveItPlanningBackend;

/// Optimal free-space mode (plugin params `free_space_optimize`, `free_space_time_budget_s`,
/// `free_space_num_candidates`, `free_space_objective`, `free_space_planners`).
struct FreeSpaceOptimizeConfig
{
  bool enabled{false};
  double time_budget_s{20.0};   ///< Total OMPL planning time, shared by the OMPL candidates.
  int num_candidates{5};        ///< Candidates in all, the direct joint path included.
  FreeSpaceObjective objective{FreeSpaceObjective::kLength};
  /// planner_configs of ompl_planning.yaml, used round robin for the OMPL candidates.
  std::vector<std::string> planners{
    "BITstarkConfigDefault", "RRTstarkConfigDefault", "AITstarkConfigDefault", "InformedRRTstarkConfigDefault"};
  PathShortcutConfig shortcut;
  /// The Cartesian straight line (continuity IK, seeded with the start) as one more candidate (param
  /// `free_space_cartesian_candidate`). Only accepted when it reaches the goal on the goal's own joint branch.
  bool cartesian_candidate{true};
  double cartesian_eef_step{0.01};          ///< m between interpolated tool poses.
  double cartesian_goal_tolerance{0.05};    ///< rad or m per joint between the line's end and the goal state.
  double cartesian_revolute_jump{0.3};      ///< rad, largest joint step between two samples.
  double cartesian_prismatic_jump{0.05};    ///< m.
};

class OmplPlanner : public Planner
{
public:
  /// backend must be non-null and, in production, the SAME instance injected
  /// into CartesianPlanner -- see CartesianPlanner's constructor comment and
  /// MoveItPlanningBackend's own file comment for the full rationale.
  explicit OmplPlanner(std::shared_ptr<MoveItPlanningBackend> backend);

  std::string name() const override;
  PlanningResult plan(const PlanningRequest & request) override;
  IkResult ik(const IkRequest & request) override;
  CollisionResult collision(const CollisionRequest & request) override;

  /// Opt-in fast path (plugin param `free_space_direct_first`): try the straight joint-space line to the goal (joint
  /// target, or a pose goal solved by IK from the start) before the OMPL pipeline. Every sample is checked through
  /// the planning scene; one invalid sample falls back to OMPL unchanged.
  void setDirectFirst(bool enabled, const DirectJointPathConfig & config, double default_acceleration);

  /// Optimal mode: asymptotically optimal OMPL runs plus the direct joint path, post-processed and the best chosen.
  void setFreeSpaceOptimize(const FreeSpaceOptimizeConfig & config, double default_acceleration);

  /// How the last plan() was made: "ompl" (default pipeline), "direct", or "optimal:<candidate source>"
  /// (candidate sources: direct, cartesian, or the OMPL planner config name). Empty before the first plan.
  const std::string & lastRoute() const {return last_route_;}

private:
  std::string last_route_;

  /// Fills `out` with the best of the candidates; false when none was found (the caller then plans as before).
  bool planOptimized(
    const std::vector<double> & start, const std::vector<double> & goal,
    moveit_msgs::msg::RobotTrajectory & out);

  FreeSpaceOptimizeConfig optimize_;

  /// The tool's straight line from the pose at `start` to the pose at `goal` (both joint states), IK seeded from
  /// the previous sample, every sample checked through the planning scene. True when the whole line was found, it
  /// ends on `goal` (same branch) and it was retimed into `out`.
  bool tryCartesianLine(
    const std::vector<double> & start, const std::vector<double> & goal,
    moveit_msgs::msg::RobotTrajectory & out);

  /// Straight-line attempt; true when moveit_plan was filled with a timed, valid trajectory.
  bool tryDirectJointPath(
    const std::vector<double> & start, const std::vector<double> & goal,
    moveit_msgs::msg::RobotTrajectory & out);

  bool direct_first_{false};
  DirectJointPathConfig direct_config_;
  double direct_default_acceleration_{2.0};

  /// Delegates to backend_->ensureMoveGroup(), then refreshes node_/
  /// move_group_/move_group_name_ from the backend's own accessors.
  bool ensureMoveGroup(const std::string & planning_group, std::string & error_message);

  std::shared_ptr<MoveItPlanningBackend> backend_;

  /// Cached views, refreshed from backend_ by ensureMoveGroup() -- still
  /// used directly by plan()'s setJointValueTarget/setPoseTarget/
  /// setStartState/move_group_->plan()/clearPoseTargets calls.
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string move_group_name_;
};

}  // namespace motion_planner
