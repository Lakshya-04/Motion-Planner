#include "motion_planner_moveit/ompl_planner.hpp"

#include <chrono>
#include <exception>
#include <limits>

#include <Eigen/Geometry>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "rclcpp/rclcpp.hpp"

#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "motion_planner_moveit/moveit_time_parameterizer.hpp"
#include "motion_planner/core/planners/wall_penetration_check.hpp"
#include "ros_adapters/ros_conversions.hpp"

namespace motion_planner
{

OmplPlanner::OmplPlanner(std::shared_ptr<MoveItPlanningBackend> backend)
: backend_(std::move(backend))
{
}

void OmplPlanner::setDirectFirst(
  bool enabled, const DirectJointPathConfig & config, double default_acceleration)
{
  direct_first_ = enabled;
  direct_config_ = config;
  direct_default_acceleration_ = default_acceleration;
}

bool OmplPlanner::tryDirectJointPath(
  const std::vector<double> & start, const std::vector<double> & goal,
  moveit_msgs::msg::RobotTrajectory & out)
{
  const auto robot_model = move_group_->getRobotModel();
  const auto * jmg = robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
  const auto log = node_ ? node_->get_logger() : rclcpp::get_logger("ompl_planner");
  if (!jmg) {
    return false;
  }
  DirectJointPathConfig config = direct_config_;
  if (config.prismatic.empty()) {
    for (const auto * joint : jmg->getActiveJointModels()) {
      config.prismatic.push_back(joint->getType() == moveit::core::JointModel::PRISMATIC);
    }
  }

  std::string scene_error;
  if (!backend_->ensurePlanningSceneMonitor(move_group_name_, scene_error) ||
    !backend_->planningSceneMonitor())
  {
    RCLCPP_WARN(log, "direct path unavailable (no planning scene: %s), falling back to OMPL", scene_error.c_str());
    return false;
  }
  DirectJointPathResult path;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(backend_->planningSceneMonitor());
    if (!scene) {
      return false;
    }
    moveit::core::RobotState state(scene->getCurrentState());
    path = buildDirectJointPath(
      start, goal, config, [&](const std::vector<double> & q) {
        state.setJointGroupPositions(jmg, q);
        state.update();
        return scene->isStateValid(state, move_group_name_);
      });
  }
  if (path.status == DirectJointPathResult::Status::kInvalidSample) {
    RCLCPP_INFO(log, "direct path invalid at sample %zu, falling back to OMPL", path.invalid_index);
    return false;
  }
  if (path.status != DirectJointPathResult::Status::kOk) {
    return false;  // no motion or bad input: let OMPL answer as before
  }

  core_types::Trajectory piece;
  piece.joint_names = jmg->getActiveJointModelNames();
  for (const auto & q : path.samples) {
    core_types::TrajectoryPoint point;
    point.positions = q;
    piece.points.push_back(std::move(point));
  }
  MoveItTimeParameterizer retimer(robot_model, move_group_name_);
  std::string message;
  if (!retimer.retime(
      piece, SegmentRole::kTransition, retimer.modelLimits(direct_default_acceleration_, 0.0), 0.0,
      message))
  {
    RCLCPP_WARN(log, "direct path retiming failed (%s), falling back to OMPL", message.c_str());
    return false;
  }
  out.joint_trajectory = ros_adapters::toRos(piece);
  RCLCPP_INFO(
    log, "direct joint path: %zu points, %.2f s", piece.points.size(),
    piece.points.empty() ? 0.0 : piece.points.back().time_from_start_sec);
  return true;
}

std::string OmplPlanner::name() const
{
  return "ompl";
}

bool OmplPlanner::ensureMoveGroup(const std::string & planning_group, std::string & error_message)
{
  if (!backend_->ensureMoveGroup(planning_group, error_message)) {
    return false;
  }
  node_ = backend_->node();
  move_group_ = backend_->moveGroup();
  move_group_name_ = backend_->activeMoveGroupName();
  return true;
}

PlanningResult OmplPlanner::plan(const PlanningRequest & request)
{
  const auto planning_start = std::chrono::steady_clock::now();

  if (request.target_poses.empty() && request.target_joint_state.empty()) {
    PlanningResult invalid_request;
    invalid_request.success = false;
    invalid_request.status = "failed";
    invalid_request.message =
      "OMPL planning requires target_poses or target_joint_state; received neither.";
    invalid_request.waypoints_completed = 0;
    invalid_request.failure_reason_code = "missing_target_poses";
    invalid_request.failure_stage = FailureStage::kPrecheck;
    invalid_request.planning_time_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() -
        planning_start)
      .count());
    return invalid_request;
  }

  std::string initialization_error;
  if (!ensureMoveGroup(request.planning_group, initialization_error)) {
    PlanningResult init_failure;
    init_failure.success = false;
    init_failure.status = "failed";
    init_failure.message = "Failed to initialize OMPL MoveGroup: " + initialization_error;
    init_failure.waypoints_completed = 0;
    init_failure.failure_reason_code = "planner_init";
    init_failure.failure_stage = FailureStage::kPrecheck;
    init_failure.planning_time_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() -
        planning_start)
      .count());
    return init_failure;
  }

  PlanningResult result;

  // Joint-space goal takes priority when given -- it isn't a Cartesian-path
  // concept, so there's no pose to fall back to reading here anyway.
  bool used_joint_target = false;
  std::vector<double> direct_goal;
  geometry_msgs::msg::Pose direct_goal_pose;
  if (!request.target_joint_state.empty()) {
    const auto robot_model = move_group_->getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
    if (joint_model_group &&
      request.target_joint_state.size() == joint_model_group->getVariableCount())
    {
      move_group_->setJointValueTarget(request.target_joint_state);
      used_joint_target = true;
      direct_goal = request.target_joint_state;
    } else {
      PlanningResult invalid_target;
      invalid_target.success = false;
      invalid_target.status = "failed";
      invalid_target.message =
        "OmplPlanner: target_joint_state size mismatch for planning group '" +
        move_group_name_ + "'.";
      invalid_target.failure_reason_code = "invalid_target_joint_state";
      invalid_target.failure_stage = FailureStage::kPrecheck;
      invalid_target.planning_time_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() -
          planning_start)
        .count());
      return invalid_target;
    }
  } else {
    auto goal_pose = ros_adapters::toRos(request.target_poses.back());
    // goal_pose is expressed in request.frame_id -- resolve into the planning frame before
    // handing it to MoveGroupInterface, same fix as CartesianPlanner::plan() (see its own
    // comment for the confirmed-live gap this closes).
    if (!request.frame_id.empty()) {
      geometry_msgs::msg::Pose resolved_pose;
      std::string resolve_error;
      if (!backend_->resolvePoseToPlanningFrame(
          request.frame_id, goal_pose, resolved_pose, resolve_error))
      {
        PlanningResult frame_failure;
        frame_failure.success = false;
        frame_failure.status = "failed";
        frame_failure.message = "Failed to resolve target pose from frame '" + request.frame_id +
          "' into the planning frame: " + resolve_error;
        frame_failure.failure_reason_code = "frame_resolution_failed";
        frame_failure.failure_stage = FailureStage::kPrecheck;
        frame_failure.planning_time_ms = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() -
            planning_start)
          .count());
        return frame_failure;
      }
      goal_pose = resolved_pose;
    }
    move_group_->setPoseTarget(goal_pose);
    direct_goal_pose = goal_pose;
  }

  std::vector<double> direct_start;
  bool used_explicit_start_state = false;
  if (!request.start_joint_state.empty()) {
    const auto robot_model = move_group_->getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
    if (joint_model_group &&
      request.start_joint_state.size() == joint_model_group->getVariableCount())
    {
      moveit::core::RobotState start_state(robot_model);
      start_state.setToDefaultValues();
      start_state.setJointGroupPositions(joint_model_group, request.start_joint_state);
      // The measured state of a joint at its end stop is often a hair outside the range (a lift read -0.09 mm and
      // later -1.5 mm below 0 after a stow). MoveIt's CheckStartStateBounds rejects any such start state, so
      // every plan then fails in milliseconds and the arm cannot be moved again. Planning from the state clamped
      // into the range changes the start by that hair only.
      start_state.enforceBounds(joint_model_group);
      start_state.update();
      move_group_->setStartState(start_state);
      used_explicit_start_state = true;
      start_state.copyJointGroupPositions(joint_model_group, direct_start);
      if (direct_goal.empty() && direct_first_) {
        if (start_state.setFromIK(joint_model_group, direct_goal_pose, 0.1)) {
          start_state.copyJointGroupPositions(joint_model_group, direct_goal);
        }
      }
    } else if (node_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "OmplPlanner: request.start_joint_state size mismatch (got %zu); "
        "falling back to current state.",
        request.start_joint_state.size());
    }
  }
  if (!used_explicit_start_state) {
    // The same hair-outside-the-range problem as above, from the live state: a lift that reads -2e-14 m below its
    // lower limit made MoveIt's CheckStartStateBounds fail every plan in Gazebo. Take the current state, clamp it
    // into the range, and plan from that.
    const auto robot_model = move_group_->getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
    // The MoveGroupInterface's own state monitor lives on the backend node, which nothing spins: in Gazebo it never
    // received a joint state ("latest received state has time 0"), so this fell through to the unclamped server
    // state and 20 of 62 plans were refused as "Start state out of bounds" by 2e-14 m. The planning scene monitor
    // has its own spinner and is what CartesianPlanner already reads its start from: use it first.
    std::shared_ptr<moveit::core::RobotState> current;
    std::string scene_error;
    if (backend_->ensurePlanningSceneMonitor(move_group_name_, scene_error)) {
      const auto monitor = backend_->planningSceneMonitor();
      if (monitor) {
        planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
        if (scene) {
          current = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
        }
      }
    }
    if (!current) {
      current = move_group_->getCurrentState(1.0);
    }
    if (current && joint_model_group) {
      moveit::core::RobotState start_state(*current);
      start_state.enforceBounds(joint_model_group);
      start_state.update();
      move_group_->setStartState(start_state);
      start_state.copyJointGroupPositions(joint_model_group, direct_start);
      if (direct_goal.empty() && direct_first_) {
        if (start_state.setFromIK(joint_model_group, direct_goal_pose, 0.1)) {
          start_state.copyJointGroupPositions(joint_model_group, direct_goal);
        }
      }
    } else {
      move_group_->setStartStateToCurrentState();
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
  bool direct_ok = false;
  if (direct_first_ && !direct_start.empty() && direct_goal.size() == direct_start.size()) {
    direct_ok = tryDirectJointPath(direct_start, direct_goal, moveit_plan.trajectory);
  }
  const bool plan_result = direct_ok ? true : static_cast<bool>(move_group_->plan(moveit_plan));

  result.success = static_cast<bool>(plan_result);
  result.status = result.success ? "planned" : "failed";
  result.message = result.success ? "OMPL plan succeeded." : "OMPL plan failed.";
  // A joint-target segment has no waypoint_indices to count (there's no pose
  // array it indexes into) -- treat reaching the target configuration as one
  // unit of work, same as any single-pose segment.
  result.waypoints_completed = result.success ?
    (used_joint_target ? 1 : request.waypoint_indices.size()) : 0;
  result.path_fraction = result.success ? 1.0 : 0.0;
  result.failure_reason_code = result.success ? "none" : "planning_failed";
  result.failure_stage = result.success ? FailureStage::kNone : FailureStage::kPlanning;
  if (result.success) {
    result.trajectory = ros_adapters::toCore(moveit_plan.trajectory.joint_trajectory);
  }

  // Independent, ACM-blind wall-penetration bound -- FSP is a transit mode
  // with no legitimate reason to be near the wall, so its tolerance should
  // be near-zero. move_group_->plan() is a service call handled entirely
  // inside the separate move_group process; there is no per-state validity
  // callback available from this client, so this is a post-hoc check over
  // the planned trajectory (the same mechanism CartesianPlanner uses,
  // applied here for consistency and because it's the only one actually
  // reachable from this API) rather than something OMPL's own search can
  // route around mid-plan.
  // wall_reference_pose is expressed in request.frame_id too -- resolve it into the planning
  // frame here (once), same fix as CartesianPlanner::plan().
  core_types::Pose wall_reference_pose = request.wall_reference_pose;
  if (result.success && !request.frame_id.empty() && hasWallReference(wall_reference_pose)) {
    geometry_msgs::msg::Pose resolved_wall_pose;
    std::string resolve_error;
    if (backend_->resolvePoseToPlanningFrame(
        request.frame_id, ros_adapters::toRos(wall_reference_pose), resolved_wall_pose,
        resolve_error))
    {
      wall_reference_pose = ros_adapters::toCore(resolved_wall_pose);
    } else {
      result.success = false;
      result.status = "failed";
      result.message = "Failed to resolve wall_reference_pose from frame '" + request.frame_id +
        "' into the planning frame: " + resolve_error;
      result.failure_reason_code = "frame_resolution_failed";
      result.failure_stage = FailureStage::kPrecheck;
    }
  }

  if (result.success && hasWallReference(wall_reference_pose)) {
    // Synthetic model-default state (seeded with the trajectory's own first
    // waypoint) -- deliberately never queries move_group_->getCurrentState(),
    // which was found live to stall ~10s per call (and never actually
    // succeed beyond the very first call of a run) in an environment where
    // CurrentStateMonitor never sees a full joint set because some other
    // part of the robot's controller stack never publishes. Same fix as
    // CartesianPlanner::resolveReferenceState() -- this state is only an
    // FK/other-joint-value container for reading back the already-planned
    // trajectory below, not the actual planning start (governed
    // server-side), so it always fell through to this exact synthetic state
    // anyway; skipping the query outright changes nothing about the result,
    // only removes the wasted wait.
    std::shared_ptr<moveit::core::RobotState> reference_state;
    const auto robot_model = move_group_->getRobotModel();
    if (robot_model) {
      reference_state = std::make_shared<moveit::core::RobotState>(robot_model);
      reference_state->setToDefaultValues();
      const auto * joint_model_group = robot_model->getJointModelGroup(move_group_name_);
      if (joint_model_group && !moveit_plan.trajectory.joint_trajectory.points.empty()) {
        reference_state->setJointGroupPositions(
          joint_model_group, moveit_plan.trajectory.joint_trajectory.points.front().positions);
      }
      reference_state->update();
    }
    if (reference_state) {
      const std::string tool_tcp_link = move_group_->getEndEffectorLink();
      robot_trajectory::RobotTrajectory rt_check(move_group_->getRobotModel(), move_group_name_);
      rt_check.setRobotTrajectoryMsg(*reference_state, moveit_plan.trajectory);
      for (size_t index = 0; index < rt_check.getWayPointCount(); ++index) {
        const auto & waypoint_state = rt_check.getWayPoint(index);
        const auto & tcp_translation =
          waypoint_state.getGlobalLinkTransform(tool_tcp_link).translation();
        if (!isWithinWallPenetrationBound(
            wall_reference_pose,
            tcp_translation.x(), tcp_translation.y(), tcp_translation.z(),
            request.max_wall_penetration))
        {
          result.success = false;
          result.status = "failed";
          result.message = "Waypoint " + std::to_string(index) +
            " exceeds max_wall_penetration for fsp mode.";
          result.waypoints_completed = 0;
          result.path_fraction = 0.0;
          result.trajectory = core_types::Trajectory{};
          result.failure_reason_code = "wall_penetration_exceeded";
          result.failure_stage = FailureStage::kPlanning;
          break;
        }
      }
    }
  }

  result.planning_time_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() -
      planning_start)
    .count());

  move_group_->clearPoseTargets();
  return result;
}

IkResult OmplPlanner::ik(const IkRequest & request)
{
  return backend_->ik(request);
}

CollisionResult OmplPlanner::collision(const CollisionRequest & request)
{
  return backend_->collision(request);
}

}  // namespace motion_planner
