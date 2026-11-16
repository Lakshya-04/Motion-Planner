#include "manipulation_manager/ros/execution/retiming_wiring.hpp"

#include <algorithm>

#include <moveit/robot_model_loader/robot_model_loader.hpp>

#include "manipulation_manager/core/execution/retiming_execution_port.hpp"
#include "motion_planner_moveit/moveit_time_parameterizer.hpp"

namespace manipulation_manager
{

std::shared_ptr<ExecutionPort> wrapWithRetiming(
  const rclcpp::Node::SharedPtr & node, std::shared_ptr<ExecutionPort> inner,
  const std::string & planning_group)
{
  if (!node->declare_parameter<bool>("retime", false)) {
    return inner;
  }
  const auto tcp_link = node->declare_parameter<std::string>("retime_tcp_link", "");
  const double accel = node->declare_parameter<double>("retime_accel", 2.0);
  const double seam_jerk = node->declare_parameter<double>("retime_seam_jerk", 20.0);
  const double transition_jerk = node->declare_parameter<double>("retime_transition_jerk", 0.0);
  const double vel_scale = std::clamp(node->declare_parameter<double>("retime_vel_scale", 1.0), 0.01, 1.0);

  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const auto model = loader.getModel();
  if (!model || !model->getJointModelGroup(planning_group)) {
    RCLCPP_ERROR(
      node->get_logger(), "retime: no robot model or group '%s'; retiming left off",
      planning_group.c_str());
    return inner;
  }
  auto retimer = std::make_shared<motion_planner::MoveItTimeParameterizer>(
    model, planning_group, tcp_link);
  auto seam = retimer->modelLimits(accel, seam_jerk);
  auto transition = retimer->modelLimits(accel, transition_jerk);
  for (auto * set : {&seam, &transition}) {
    for (auto & l : *set) {
      l.velocity *= vel_scale;
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "retime: on (group %s, accel %.2f, seam jerk %.1f, velocity scale %.2f)",
    planning_group.c_str(), accel, seam_jerk, vel_scale);
  return std::make_shared<RetimingExecutionPort>(
    std::move(inner), std::move(retimer), std::move(seam), std::move(transition));
}

}  // namespace manipulation_manager
