// The motion_planner Planner port over cuRobo: a client of the curobo_bridge planner node.
//
// The node does the GPU work in its own process; this class turns PlanningRequest / world updates
// into service calls and the answers back into PlanningResult. The node passed in must be spun by
// someone else (an executor thread) because calls wait on the service future.
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "core_types/logger.hpp"
#include "motion_planner/core/planners/planner.hpp"
#include "motion_planner/core/scene_port.hpp"

namespace curobo_planner
{

struct Options
{
  std::string node_name{"/curobo_planner"};  ///< Name of the cuRobo planner node (its services' prefix).
  double wait_for_service_sec{5.0};
  double timeout_sec{30.0};                  ///< Per-call deadline; the first plan can be slow.
};

class CuroboClientPlanner : public motion_planner::Planner
{
public:
  CuroboClientPlanner(
    rclcpp::Node::SharedPtr node, const Options & options = Options{},
    std::shared_ptr<core_types::Logger> logger = nullptr);
  ~CuroboClientPlanner() override;

  std::string name() const override {return "curobo";}

  /// Replaces the planner's world (boxes + points, `points_origin` applied). False on failure.
  bool setWorld(const motion_planner::WorldGeometryUpdate & world, std::string * message = nullptr);
  bool clearWorld(std::string * message = nullptr);

  /// Joint-space move start_joint_state -> target_joint_state. With no target joint state and one
  /// target pose, plans to that pose (position + quaternion, robot base frame).
  motion_planner::PlanningResult plan(const motion_planner::PlanningRequest & request) override;
  /// Not provided by this client (failure code "unsupported").
  motion_planner::IkResult ik(const motion_planner::IkRequest & request) override;
  motion_planner::CollisionResult collision(const motion_planner::CollisionRequest & request) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace curobo_planner
