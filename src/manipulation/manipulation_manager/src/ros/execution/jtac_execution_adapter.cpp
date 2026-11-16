#include "manipulation_manager/ros/execution/jtac_execution_adapter.hpp"

#include <chrono>
#include <future>

#include "ros_adapters/ros_conversions.hpp"

namespace manipulation_manager
{

using GoalHandle = rclcpp_action::ClientGoalHandle<JtacExecutionAdapter::FollowJointTrajectory>;

JtacExecutionAdapter::JtacExecutionAdapter(
  const rclcpp::Node::SharedPtr & node, const std::string & action_name)
: node_(node),
  client_(rclcpp_action::create_client<FollowJointTrajectory>(node, action_name))
{
}

ExecutionResult JtacExecutionAdapter::execute(const ExecutionRequest & request)
{
  ExecutionResult result;
  if (cancelled_) {
    result.message = "cancelled";
    result.failure_reason_code = "cancelled";
    return result;
  }
  if (request.trajectory.points.empty()) {
    result.message = "empty trajectory, nothing to execute";
    result.failure_reason_code = "empty_trajectory";
    return result;
  }
  if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
    result.message = "JTAC action server unavailable";
    result.failure_reason_code = "action_server_unavailable";
    return result;
  }

  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory = ros_adapters::toRos(request.trajectory);

  auto result_promise = std::make_shared<std::promise<bool>>();
  auto result_future = result_promise->get_future().share();

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
    [result_promise](const GoalHandle::WrappedResult & wrapped) {
      result_promise->set_value(wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  const auto start_time = std::chrono::steady_clock::now();
  auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    result.message = "send_goal call failed";
    result.failure_reason_code = "send_goal_failed";
    return result;
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    result.message = "goal rejected";
    result.failure_reason_code = "goal_rejected";
    return result;
  }

  current_goal_ = goal_handle;
  if (cancelled_) {
    cancelGoal(goal_handle);
  }
  while (rclcpp::ok() &&
    result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
  {
    rclcpp::spin_some(node_);
  }
  current_goal_.reset();
  result.success = result_future.get();
  result.execution_time_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count());
  if (!result.success) {
    result.message = "trajectory execution did not succeed";
    result.failure_reason_code = "execution_failed";
  }
  return result;
}

void JtacExecutionAdapter::setCancelled(bool cancelled)
{
  cancelled_ = cancelled;
}

void JtacExecutionAdapter::cancelCurrent()
{
  cancelled_ = true;
  if (current_goal_) {
    cancelGoal(current_goal_);
  }
}

void JtacExecutionAdapter::cancelGoal(
  const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr & goal)
{
  try {
    client_->async_cancel_goal(goal);
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    // the goal finished (its result was already handled) between the cancel request and now: nothing to cancel
  }
}

}  // namespace manipulation_manager
