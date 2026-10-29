/**
 * @file jtac_execution_adapter.hpp
 * @brief ExecutionPort backed by JTAC's own FollowJointTrajectory action server.
 * @details sii's original ManipulationManager port only had a
 *          MoveItExecutionAdapter (MoveGroupInterface::execute()) -- this repo's own
 *          coverage_pass_demo_node.cpp already talks to
 *          /combined_arm_lift_controller/follow_joint_trajectory directly (JTAC is a
 *          JointTrajectoryController-family controller, not driven through MoveIt's own
 *          execution manager), so ManipulationManager needs this adapter instead of that one.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "manipulation_manager/core/execution/execution_port.hpp"

namespace manipulation_manager
{

class JtacExecutionAdapter : public ExecutionPort
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  explicit JtacExecutionAdapter(
    const rclcpp::Node::SharedPtr & node,
    const std::string & action_name = "/combined_arm_lift_controller/follow_joint_trajectory");

  ExecutionResult execute(const ExecutionRequest & request) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
};

}  // namespace manipulation_manager
