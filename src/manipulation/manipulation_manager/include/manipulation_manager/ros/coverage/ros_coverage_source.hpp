/**
 * @file ros_coverage_source.hpp
 * @brief CoverageSource that takes the plan from the coverage planner over ROS.
 * @details wall_coverage_service_node publishes its finished plan once, as a coverage_planner_msgs/CoveragePlan on
 *          ~/coverage_plan (latched: transient_local, depth 1). The manager subscribes to that topic
 *          (`coverage_source: ros`, `coverage_plan_topic`), keeps the latest message in a CoveragePlanInbox, and
 *          this source waits for it, validates it and turns it into CoverageSegmentIntent with the same
 *          buildCoverageIntents() the JSON source uses. The message-to-RawCoveragePlan copy is the only ROS-typed
 *          code here; the conversion, the validation and the wait are ROS-free and host-tested.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "coverage_planner_msgs/msg/coverage_plan.hpp"
#include "manipulation_manager/core/coverage_plan_intents.hpp"
#include "manipulation_manager/core/coverage_source.hpp"

namespace manipulation_manager
{

/// Field-by-field copy of the message into the ROS-free struct (no logic, so nothing to test beyond the build).
RawCoveragePlan rawPlanFromMsg(const coverage_planner_msgs::msg::CoveragePlan & msg);

/// A transient_local (latched) subscription that puts every received plan into `inbox`.
rclcpp::Subscription<coverage_planner_msgs::msg::CoveragePlan>::SharedPtr subscribeCoveragePlan(
  rclcpp::Node & node, const std::string & topic, std::shared_ptr<CoveragePlanInbox> inbox);

class RosCoverageSource : public CoverageSource
{
public:
  /**
   * @param[in] inbox Filled by subscribeCoveragePlan().
   * @param[in] pump Called while waiting; the manager spins its node once here (it has no executor thread).
   * @param[in] timeout_s How long getCoveragePlan() waits for a plan if none has arrived yet.
   * @param[in] frame_id Frame the OUTPUT poses are expressed in.
   * @param[in] options Same resampling settings the JSON source takes.
   */
  RosCoverageSource(
    std::shared_ptr<CoveragePlanInbox> inbox, std::function<void()> pump, double timeout_s,
    std::string frame_id, CoverageIntentOptions options);

  /// Throws std::runtime_error on timeout or an invalid plan (the manager reports it as "cannot load the plan").
  std::vector<CoverageSegmentIntent> getCoveragePlan() override;
  std::string getFrameId() override {return frame_id_;}

  /// The plan the last getCoveragePlan() converted (for the mission ledger's plan identity).
  const RawCoveragePlan & lastPlan() const {return last_plan_;}

private:
  std::shared_ptr<CoveragePlanInbox> inbox_;
  std::function<void()> pump_;
  double timeout_s_;
  std::string frame_id_;
  CoverageIntentOptions options_;
  RawCoveragePlan last_plan_;
};

}  // namespace manipulation_manager
