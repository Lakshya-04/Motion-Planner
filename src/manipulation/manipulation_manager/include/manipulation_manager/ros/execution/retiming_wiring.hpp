/**
 * @file retiming_wiring.hpp
 * @brief Opt-in wiring of the retiming ExecutionPort decorator for the manager node.
 * @details Declares the `retime*` parameters on the node and, when `retime` is true, wraps the
 *          execution port so each planned trajectory is retimed by segment role (TOTG for
 *          transitions, constant stroke speed plus Ruckig for seams). With `retime:=false`
 *          (default) it returns the port unchanged and nothing else is touched.
 *          Parameters: retime, retime_tcp_link (default: group tip), retime_accel (rad/s^2 where
 *          the model has no acceleration bound), retime_seam_jerk, retime_transition_jerk (0 =
 *          unbounded), retime_vel_scale (0 < s <= 1 on the model velocity bounds).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "manipulation_manager/core/execution/execution_port.hpp"

namespace manipulation_manager
{

std::shared_ptr<ExecutionPort> wrapWithRetiming(
  const rclcpp::Node::SharedPtr & node, std::shared_ptr<ExecutionPort> inner,
  const std::string & planning_group);

}  // namespace manipulation_manager
