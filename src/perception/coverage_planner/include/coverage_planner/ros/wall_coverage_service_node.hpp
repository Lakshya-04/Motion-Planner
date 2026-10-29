// Copyright 2026 Lakshya Agarwal
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef COVERAGE_PLANNER__WALL_COVERAGE_SERVICE_NODE_HPP_
#define COVERAGE_PLANNER__WALL_COVERAGE_SERVICE_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "coverage_planner/core/types.hpp"
#include <std_srvs/srv/trigger.hpp>

#include "coverage_planner/core/tool_coverage_profile.hpp"
#include "coverage_planner_msgs/msg/coverage_plan.hpp"
#include "coverage_planner_msgs/msg/wall_plane.hpp"
#include "coverage_planner_msgs/srv/generate_coverage_path.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace msg = coverage_planner_msgs::msg;
namespace srv = coverage_planner_msgs::srv;

/// A standing, trigger-controlled counterpart to wall_coverage_demo_node: loads
/// wall_scene_json/walls.pcd from its own parameters (not a service request -- ~/start is a
/// std_srvs/Trigger, which carries no fields), calls coverage_planner_node's own
/// GenerateCoveragePath service, and publishes the final lifted plan on ~/coverage_plan instead
/// of returning it synchronously.
///
/// ~/start acknowledges immediately once the request is under way; the plan itself always
/// arrives later, on the topic. This is deliberate, not a shortcut: GenerateCoveragePath is
/// called with async_send_request() rather than blocking, because a blocking
/// spin_until_future_complete() called from inside this node's own service callback would
/// deadlock -- the executor cannot service the future's completion while it is still busy
/// running the callback that is waiting on it.
///
/// ~/stop does not cancel an in-flight GenerateCoveragePath call (a plain ROS service call
/// cannot be cancelled once sent, the same as every other node in this codebase that wraps one).
/// What it does do: suppress the *publish* if the run currently in flight finishes after ~/stop
/// was called -- the one thing actually still undecided by the time ~/stop can react.
class WallCoverageServiceNode : public rclcpp::Node
{
public:
  explicit WallCoverageServiceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using GenerateCoveragePath = srv::GenerateCoveragePath;

  void startCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Builds this node's own ToolCoverageProfile from its parameters -- see
  /// wall_coverage_demo_node's own buildToolCoverageProfile() for the doc comment this mirrors.
  ToolCoverageProfile buildToolCoverageProfile() const;

  /// Loads/rasterizes the wall and sends the GenerateCoveragePath request. Throws on any failure
  /// (a bad path, an unknown wall id) -- caught by the caller (startCallback), since this is
  /// routine, expected-failure territory, not something that should propagate out of a service
  /// callback.
  void requestCoveragePath();

  /// The GenerateCoveragePath response callback -- runs the same lifting
  /// wall_coverage_demo_node applies, then publishes on ~/coverage_plan unless ~/stop fired
  /// first.
  void coveragePathResponseCallback(rclcpp::Client<GenerateCoveragePath>::SharedFuture future);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Client<GenerateCoveragePath>::SharedPtr coverage_client_;
  rclcpp::Publisher<msg::CoveragePlan>::SharedPtr plan_pub_;

  WallPlane wall_;  ///< Core type; converted to messages only at the ROS boundary.

  std::atomic<bool> busy_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__WALL_COVERAGE_SERVICE_NODE_HPP_
