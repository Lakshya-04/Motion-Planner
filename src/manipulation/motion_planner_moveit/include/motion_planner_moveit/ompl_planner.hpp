#pragma once

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "motion_planner/core/direct_joint_path.hpp"
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

private:
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
