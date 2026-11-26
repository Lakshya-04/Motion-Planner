#include "manipulation_manager/ros/coverage/ros_coverage_source.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation_manager
{

RawCoveragePlan rawPlanFromMsg(const coverage_planner_msgs::msg::CoveragePlan & msg)
{
  RawCoveragePlan plan;
  plan.success = msg.success;
  plan.message = msg.message;
  plan.frame_id = msg.path.header.frame_id;
  plan.poses.reserve(msg.path.poses.size());
  for (const auto & ps : msg.path.poses) {
    core_types::Pose p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    p.z = ps.pose.position.z;
    p.qx = ps.pose.orientation.x;
    p.qy = ps.pose.orientation.y;
    p.qz = ps.pose.orientation.z;
    p.qw = ps.pose.orientation.w;
    plan.poses.push_back(p);
  }
  for (const auto & s : msg.segments) {
    plan.segments.push_back({s.start_index, s.end_index});
  }
  return plan;
}

rclcpp::Subscription<coverage_planner_msgs::msg::CoveragePlan>::SharedPtr subscribeCoveragePlan(
  rclcpp::Node & node, const std::string & topic, std::shared_ptr<CoveragePlanInbox> inbox)
{
  // Same QoS as the publisher (wall_coverage_service_node): reliable + transient_local, so a plan published before
  // the manager started is still delivered when it subscribes.
  return node.create_subscription<coverage_planner_msgs::msg::CoveragePlan>(
    topic, rclcpp::QoS(1).reliable().transient_local(),
    [inbox, logger = node.get_logger()](coverage_planner_msgs::msg::CoveragePlan::ConstSharedPtr msg) {
      inbox->put(rawPlanFromMsg(*msg));
      RCLCPP_INFO(logger, "received a coverage plan with %zu pose(s)", msg->path.poses.size());
    });
}

RosCoverageSource::RosCoverageSource(
  std::shared_ptr<CoveragePlanInbox> inbox, std::function<void()> pump, double timeout_s,
  std::string frame_id, CoverageIntentOptions options)
: inbox_(std::move(inbox)),
  pump_(std::move(pump)),
  timeout_s_(timeout_s),
  frame_id_(std::move(frame_id)),
  options_(std::move(options))
{
}

std::vector<CoverageSegmentIntent> RosCoverageSource::getCoveragePlan()
{
  if (!inbox_->waitFor(timeout_s_, pump_, last_plan_)) {
    throw std::runtime_error("no coverage plan arrived on the coverage plan topic within the timeout");
  }
  validateCoveragePlan(last_plan_);
  return buildCoverageIntents(last_plan_, options_);
}

}  // namespace manipulation_manager
