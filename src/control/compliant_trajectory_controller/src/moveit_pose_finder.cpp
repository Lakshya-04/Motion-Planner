/**
 * @file moveit_pose_finder.cpp
 * @brief Reusable diagnostic: finds a joint-space solution for a Cartesian pose target and prints
 *        it (never executes).
 * @details PLAN-ONLY -- unlike moveit_jtac_execution_probe.cpp (this package), this never calls
 *          execute(). Its purpose is finding a validated joint-space pose to hardcode into a raw-
 *          FollowJointTrajectory test (the same role pose_conditioning_probe/
 *          reference_pose_collision_probe already play for a hand-picked pose, extended to
 *          "start from a Cartesian target instead of guessing joint angles directly").
 *
 *          collision_aware (default true): calls move_group.plan() -- a successful result already
 *          implies the target is reachable, IK-solvable, AND collision-free against whatever
 *          planning scene is currently active (e.g. a real pushed wall octomap,
 *          cartesian_test.launch.py's mount_robot_to_wall:=true) -- a genuine validation step.
 *
 *          allow_wall_contact (default false, only meaningful with collision_aware:=true): before
 *          planning, temporarily sets an Allowed Collision Matrix entry (tcp_link vs
 *          contact_object_id) to ALWAYS-allowed, then restores whatever entry (or absence of one)
 *          existed before -- reusing the EXACT mechanism motion_planner::CartesianPlanner
 *          already uses in production for `allow_target_contact` segments
 *          (setWallContactAcmEntry()/restoreWallContactAcmEntry(), cartesian_planner.cpp), not a
 *          new invention: read the live ACM via GetPlanningScene, mutate one entry, push the FULL
 *          resulting ACM back via PlanningSceneInterface::applyPlanningScene() (never a partial
 *          diff -- avoids depending on how MoveIt merges vs. replaces ACM diffs, same reasoning
 *          cartesian_planner.cpp's own comment gives). contact_object_id defaults to "<octomap>",
 *          the literal MoveIt itself assigns to octomap world geometry (verified against the
 *          installed libmoveit_planning_scene.so -- hardcoded here rather than pulled from
 *          motion_planner::kWallOctomapCollisionObjectId, matching the SAME already-existing
 *          precedent of inlining this literal directly in the wall-cleaning application's
 *          action servers rather than adding a cross-tree header dependency for one string).
 *
 *          This is the CORRECT way to find a wall-approach pose that genuinely touches real
 *          collision geometry: it keeps MoveIt's own collision-aware planning in the loop (the
 *          resulting pose is provably reachable and self-collision-free, exactly as if a real
 *          contact segment planned it), rather than bypassing collision checking altogether. It
 *          replaces an earlier collision_aware:=false / setFromIK() attempt at solving the SAME
 *          problem (see below) that was found live to overshoot: a plain "push N cm past the
 *          fitted plane" target, found via unconstrained IK with no ACM involved at all, can end
 *          up penetrating clean through a thin, single-voxel-thick wall shell and the raw point
 *          cloud along with it -- there was nothing to stop it at "just inside," because nothing
 *          was checking against the real occupied volume it was supposed to be touching. The ACM
 *          route asks MoveIt itself for a pose that is reachable while permitted to touch (not
 *          required to avoid) the wall's real octomap -- the planner, not a hand-tuned standoff
 *          number, decides how far "in" the resulting pose actually lands. This mirrors what will
 *          actually happen on hardware: JTAC's real force feedback -- not a precomputed exact
 *          Cartesian target -- is what stops the approach at genuine contact; commanding a target
 *          slightly past the (imperfectly known) real surface and letting the geometric contact
 *          monitor (or, on hardware, real force) cut the motion short is the intended, correct
 *          shape of a move-until-contact approach, not a bug to eliminate by finding an exact
 *          non-penetrating pose.
 *
 *          collision_aware:=false: calls moveit::core::RobotState::setFromIK() directly instead,
 *          with NO validity/collision constraint -- pure kinematics, no motion planning, no ACM.
 *          Kept for other legitimate uses (e.g. probing IK reachability irrespective of the
 *          current scene) but superseded by allow_wall_contact for the wall-approach-pose use case
 *          specifically, for the overshoot reason above.
 *
 *          Reusable diagnostic, matching this package's own established pattern -- not a
 *          throwaway script.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <algorithm>
#include <cstdio>
#include <future>
#include <string>
#include <vector>

#include "moveit/collision_detection/collision_matrix.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "mobile_manipulator_profile/profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace
{

// Mirrors motion_planner::CartesianPlanner::setWallContactAcmEntry() (cartesian_planner.cpp):
// read the FULL current ACM, mutate one entry, push the FULL result back -- never a partial diff.
// Uses a plain future.wait_for() rather than rclcpp::spin_until_future_complete() because main()
// already spins `node` on a background executor thread for MoveGroupInterface's own use; spinning
// the same node again here would race that executor.
bool setContactAcmEntry(
  const rclcpp::Node::SharedPtr & node, const std::string & tool_link,
  const std::string & target_object_id, bool allowed, bool & had_previous_entry,
  bool & previous_allowed)
{
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "get_planning_scene service not available.");
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "get_planning_scene call timed out.");
    return false;
  }

  collision_detection::AllowedCollisionMatrix acm(future.get()->scene.allowed_collision_matrix);
  collision_detection::AllowedCollision::Type previous_type =
    collision_detection::AllowedCollision::NEVER;
  had_previous_entry = acm.getEntry(tool_link, target_object_id, previous_type);
  previous_allowed =
    had_previous_entry && (previous_type == collision_detection::AllowedCollision::ALWAYS);
  acm.setEntry(tool_link, target_object_id, allowed);

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  acm.getMessage(diff.allowed_collision_matrix);

  moveit::planning_interface::PlanningSceneInterface psi;
  if (!psi.applyPlanningScene(diff)) {
    RCLCPP_ERROR(node->get_logger(), "applyPlanningScene() rejected the ACM diff.");
    return false;
  }
  return true;
}

bool restoreContactAcmEntry(
  const rclcpp::Node::SharedPtr & node, const std::string & tool_link,
  const std::string & target_object_id, bool had_previous_entry, bool previous_allowed)
{
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "get_planning_scene service not available for ACM restore.");
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "get_planning_scene call timed out during ACM restore.");
    return false;
  }

  collision_detection::AllowedCollisionMatrix acm(future.get()->scene.allowed_collision_matrix);
  if (had_previous_entry) {
    acm.setEntry(tool_link, target_object_id, previous_allowed);
  } else {
    acm.removeEntry(tool_link, target_object_id);
  }

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  acm.getMessage(diff.allowed_collision_matrix);

  moveit::planning_interface::PlanningSceneInterface psi;
  if (!psi.applyPlanningScene(diff)) {
    RCLCPP_ERROR(node->get_logger(), "applyPlanningScene() rejected the ACM restore diff.");
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_pose_finder");

  // Names come from the robot profile (`profile` parameter, config/profiles/<name>.yaml); the
  // explicit parameters below still win when set.
  node->declare_parameter("profile", std::string(mobile_manipulator_profile::kDefaultProfile));
  std::string profile_warning;
  const auto profile = mobile_manipulator_profile::loadProfileOrDefaults(
    node->get_parameter("profile").as_string(), profile_warning);
  if (!profile_warning.empty()) {
    RCLCPP_WARN(node->get_logger(), "%s", profile_warning.c_str());
  }
  node->declare_parameter("planning_group", profile.whole_body_group);
  node->declare_parameter("tcp_link", profile.frameOr("tcp", "tcp_link"));
  node->declare_parameter("reference_frame", std::string(""));  // empty == move_group's default.
  node->declare_parameter("position_xyz", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter("orientation_rpy", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter("planning_time_s", 10.0);
  // false (default): plan only, never moves the real robot -- matching this tool's own name and
  // this package's other diagnostics. true: also execute() the plan -- NOTE: found live to
  // conflict with any OTHER process holding the same action server (e.g. cartesian_test.launch
  // .py's own motion_validator_node running its own concurrent mission); seed_joint_positions
  // below is the conflict-free alternative for chaining multiple calls.
  node->declare_parameter("execute", false);
  // 7 values (lift, joint1..6), or empty (default) to plan from the robot's actual live current
  // state. Lets a SECOND call be seeded from a FIRST call's own solution -- entirely offline
  // (setStartState(), no action-server interaction, no conflict with a concurrently-running
  // mission) -- rather than both calls independently solving IK from the same starting state and
  // landing on unrelated solution branches. Hit live finding a wall-approach + standoff pose
  // pair: two nearby Cartesian targets, planned independently, produced wildly different
  // elbow/wrist configurations with no continuity between them.
  node->declare_parameter("seed_joint_positions", std::vector<double>{});
  // true (default): move_group.plan() -- collision-checked, matching every other use of this
  // tool. false: moveit::core::RobotState::setFromIK() directly, with no validity/collision
  // constraint at all -- see this file's own doc comment for the one legitimate reason to want
  // this (probing raw IK reachability irrespective of the current scene; superseded by
  // allow_wall_contact below for finding a wall-approach pose specifically).
  node->declare_parameter("collision_aware", true);
  // false (default): plan() must avoid ALL collision geometry, unchanged behavior. true: only
  // meaningful with collision_aware:=true -- temporarily allows the tcp_link to touch
  // contact_object_id for the duration of this one plan() call, then restores the ACM exactly as
  // found. See this file's own doc comment for why this is the correct way to find a genuinely
  // wall-touching pose (reuses motion_planner::CartesianPlanner's own production mechanism).
  node->declare_parameter("allow_wall_contact", false);
  // The collision object id the ACM exclusion above applies to. Default matches the literal
  // MoveIt itself assigns to octomap world geometry (see this file's own doc comment) -- only
  // override this for a `boxes`-style named obstacle, never for the wall octomap itself.
  node->declare_parameter("contact_object_id", std::string("<octomap>"));
  // false (default): use move_group.plan()/setFromIK() per collision_aware above -- an OMPL
  // joint-space search to a freshly, independently IK-sampled goal, a poor match for "take one
  // short, nearly-straight step toward the wall." Found live: even with allow_wall_contact:=true
  // relaxing the ACM, plan() still failed repeatedly (3+ tries, up to 20s) for a target already
  // independently confirmed goal-state-valid via /check_state_validity -- almost certainly because
  // setPoseTarget() resamples IK for the goal itself each call and can land on a branch requiring
  // a large, obstacle-fraught joint-space detour for what is Cartesian-ly a 0.15m straight-line
  // move. true: use move_group.computeCartesianPath() instead -- the SAME primitive
  // motion_planner::CartesianPlanner's own contact segments use in production (see this
  // file's own doc comment), which walks the tip through explicit waypoints rather than
  // re-sampling IK from scratch, and reports a `fraction` diagnosing exactly how far it got.
  node->declare_parameter("cartesian_mode", false);
  // computeCartesianPath()'s own eef_step -- how finely the straight-line path is discretized.
  node->declare_parameter("cartesian_step_m", 0.005);
  // Minimum computeCartesianPath() fraction to accept as "planned." Not 1.0 by default: a
  // fraction just under 1.0 near the very end of a wall-contact segment is expected and fine.
  node->declare_parameter("cartesian_min_fraction", 0.95);
  // collision_aware:=false's own setFromIK() timeout. Found live: the original 0.1s default let a
  // tiny (2cm) step from an already-correctly-oriented seed converge to a DIFFERENT, wrongly-
  // oriented branch (tip local Z flipped ~90deg) -- almost certainly the underlying KDL solver
  // falling back to a randomized restart before the seed-local Newton search had time to converge.
  node->declare_parameter("ik_timeout_s", 1.0);
  // Per-joint +/- bound (radians) setFromIK() may move away from seed_joint_positions, in
  // kJointNames order (7 values), or empty (default) for no bound. Use this (not just a longer
  // timeout) to FORCE the solver to stay on the seed's own branch rather than silently converging
  // to a different, differently-oriented elbow/wrist configuration for the same Cartesian target.
  node->declare_parameter("ik_consistency_limits", std::vector<double>{});

  const auto planning_group = node->get_parameter("planning_group").as_string();
  const auto tcp_link = node->get_parameter("tcp_link").as_string();
  const auto reference_frame = node->get_parameter("reference_frame").as_string();
  const auto position_xyz = node->get_parameter("position_xyz").as_double_array();
  const auto orientation_rpy = node->get_parameter("orientation_rpy").as_double_array();
  const auto planning_time_s = node->get_parameter("planning_time_s").as_double();
  const auto should_execute = node->get_parameter("execute").as_bool();
  const auto seed_joint_positions =
    node->get_parameter("seed_joint_positions").as_double_array();
  const auto collision_aware = node->get_parameter("collision_aware").as_bool();
  const auto allow_wall_contact = node->get_parameter("allow_wall_contact").as_bool();
  const auto contact_object_id = node->get_parameter("contact_object_id").as_string();
  const auto cartesian_mode = node->get_parameter("cartesian_mode").as_bool();
  const auto cartesian_step_m = node->get_parameter("cartesian_step_m").as_double();
  const auto cartesian_min_fraction = node->get_parameter("cartesian_min_fraction").as_double();
  const auto ik_timeout_s = node->get_parameter("ik_timeout_s").as_double();
  const auto ik_consistency_limits =
    node->get_parameter("ik_consistency_limits").as_double_array();

  if (position_xyz.size() != 3 || orientation_rpy.size() != 3) {
    RCLCPP_FATAL(node->get_logger(), "position_xyz and orientation_rpy must each have 3 elements.");
    rclcpp::shutdown();
    return 1;
  }

  // Needed ONLY for collision_aware:=false's own moveit::core::RobotState::setFromIK() call --
  // that operates entirely LOCALLY on this node's OWN RobotModel (unlike move_group.plan(), which
  // just delegates the whole request to the REMOTE /move_group action server, already correctly
  // configured), so it needs its OWN local IK solver plugin loaded via robot_description_
  // kinematics parameters. Found live: without these, setFromIK() fails outright with "No
  // kinematics solver instantiated for group ..." even for a target already proven IK-solvable
  // by the collision-aware path. Values match robot_description/config/kinematics.yaml's own
  // whole-body group's entry -- MoveIt's own robot_description_kinematics.<group>.<key> convention, NOT
  // kinematics_interface's flat kinematics.* one this package's other tools use.
  node->declare_parameter(
    "robot_description_kinematics." + planning_group + ".kinematics_solver",
    std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
  node->declare_parameter(
    "robot_description_kinematics." + planning_group + ".kinematics_solver_search_resolution",
    0.005);
  node->declare_parameter(
    "robot_description_kinematics." + planning_group + ".kinematics_solver_timeout", 0.05);
  node->declare_parameter(
    "robot_description_kinematics." + planning_group + ".kinematics_solver_attempts", 5);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setPlanningTime(planning_time_s);
  move_group.setEndEffectorLink(tcp_link);
  if (!reference_frame.empty()) {
    move_group.setPoseReferenceFrame(reference_frame);
  }
  RCLCPP_INFO(
    node->get_logger(), "planning frame = '%s', end effector link = '%s'",
    move_group.getPlanningFrame().c_str(), move_group.getEndEffectorLink().c_str());

  // Force the CurrentStateMonitor to actually receive a /joint_states message before anything
  // below relies on "the robot's current state" -- found live that computeCartesianPath() (unlike
  // plan(), which apparently tolerates this better) used a stale/default all-near-zero-ish state
  // when called immediately after construction in this short-lived, one-shot process, silently
  // planning from the WRONG start and hitting an unrelated self-collision/limit at the same
  // fraction regardless of the actual target or any ACM change.
  auto current_state_check = move_group.getCurrentState(10.0);
  if (!current_state_check) {
    RCLCPP_FATAL(node->get_logger(), "Never received the robot's current state (/joint_states).");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  {
    std::string dbg = "[";
    for (const auto & jn : move_group.getActiveJoints()) {
      const double * p = current_state_check->getJointPositions(jn);
      dbg += jn + "=" + (p ? std::to_string(*p) : std::string("?")) + " ";
    }
    dbg += "]";
    RCLCPP_INFO(node->get_logger(), "getCurrentState() reports: %s", dbg.c_str());
  }

  // The group's own active joints, in group order (lift first, then the arm): any DOF.
  const std::vector<std::string> kJointNames = move_group.getActiveJoints();

  moveit::core::RobotState seed_state(move_group.getRobotModel());
  seed_state.setToDefaultValues();
  if (!seed_joint_positions.empty()) {
    if (seed_joint_positions.size() != kJointNames.size()) {
      RCLCPP_FATAL(
        node->get_logger(), "seed_joint_positions must have %zu elements, got %zu.",
        kJointNames.size(), seed_joint_positions.size());
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      seed_state.setJointPositions(kJointNames[i], &seed_joint_positions[i]);
    }
    seed_state.update();
    move_group.setStartState(seed_state);  // only used by the collision_aware:=true path below.
    RCLCPP_INFO(node->get_logger(), "planning from a seeded start state, not the robot's live one.");
  }

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = position_xyz[0];
  target_pose.position.y = position_xyz[1];
  target_pose.position.z = position_xyz[2];
  tf2::Quaternion q;
  q.setRPY(orientation_rpy[0], orientation_rpy[1], orientation_rpy[2]);
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  bool planned = false;
  std::vector<double> solution_positions;
  std::vector<std::string> solution_names;

  if (cartesian_mode) {
    bool had_previous_acm_entry = false;
    bool previous_acm_allowed = false;
    if (allow_wall_contact) {
      if (!setContactAcmEntry(
          node, tcp_link, contact_object_id, true, had_previous_acm_entry, previous_acm_allowed))
      {
        RCLCPP_FATAL(node->get_logger(), "Failed to set the wall-contact ACM entry.");
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return 1;
      }
      RCLCPP_INFO(
        node->get_logger(),
        "allow_wall_contact: '%s' vs '%s' temporarily ALWAYS-allowed for this "
        "computeCartesianPath() call.",
        tcp_link.c_str(), contact_object_id.c_str());
    }

    // computeCartesianPath() walks from the robot's actual CURRENT state (it does not honor
    // setStartState()/seed_joint_positions -- those only affect the collision_aware plan() path
    // above), so this mode is meant to be run with the real robot already sitting at a known
    // pose (e.g. kStandoffPose), not seeded offline.
    const std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group.computeCartesianPath(waypoints, cartesian_step_m, 0.0, trajectory);
    RCLCPP_INFO(
      node->get_logger(), "computeCartesianPath() fraction = %.4f (need >= %.4f)", fraction,
      cartesian_min_fraction);
    planned = fraction >= cartesian_min_fraction && !trajectory.joint_trajectory.points.empty();
    if (!trajectory.joint_trajectory.points.empty()) {
      solution_names = trajectory.joint_trajectory.joint_names;
      solution_positions = trajectory.joint_trajectory.points.back().positions;
      std::string last_reached_line = "[";
      for (size_t i = 0; i < solution_names.size(); ++i) {
        last_reached_line += solution_names[i] + "=" + std::to_string(solution_positions[i]) +
          (i + 1 < solution_names.size() ? ", " : "");
      }
      last_reached_line += "]";
      RCLCPP_INFO(
        node->get_logger(), "last reached joint state (may be a partial/truncated path): %s",
        last_reached_line.c_str());
    }

    if (allow_wall_contact) {
      if (!restoreContactAcmEntry(
          node, tcp_link, contact_object_id, had_previous_acm_entry, previous_acm_allowed))
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to restore the wall-contact ACM entry.");
      }
    }

    if (planned && should_execute) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = trajectory;
      const auto execute_result = move_group.execute(plan);
      const bool executed = execute_result == moveit::core::MoveItErrorCode::SUCCESS;
      RCLCPP_INFO(
        node->get_logger(), "execute() error_code = %d (SUCCESS = %d)", execute_result.val,
        moveit::core::MoveItErrorCode::SUCCESS);
      if (!executed) {
        planned = false;
      }
    }
  } else if (collision_aware) {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id =
      reference_frame.empty() ? move_group.getPlanningFrame() : reference_frame;
    target.pose = target_pose;
    move_group.setPoseTarget(target, tcp_link);

    bool had_previous_acm_entry = false;
    bool previous_acm_allowed = false;
    if (allow_wall_contact) {
      if (!setContactAcmEntry(
          node, tcp_link, contact_object_id, true, had_previous_acm_entry, previous_acm_allowed))
      {
        RCLCPP_FATAL(node->get_logger(), "Failed to set the wall-contact ACM entry.");
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return 1;
      }
      RCLCPP_INFO(
        node->get_logger(), "allow_wall_contact: '%s' vs '%s' temporarily ALWAYS-allowed for this "
        "plan() call.",
        tcp_link.c_str(), contact_object_id.c_str());
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    planned = static_cast<bool>(move_group.plan(plan));
    RCLCPP_INFO(node->get_logger(), "plan() success = %s", planned ? "true" : "false");
    if (planned && !plan.trajectory.joint_trajectory.points.empty()) {
      solution_names = plan.trajectory.joint_trajectory.joint_names;
      solution_positions = plan.trajectory.joint_trajectory.points.back().positions;
    }

    if (allow_wall_contact) {
      if (!restoreContactAcmEntry(
          node, tcp_link, contact_object_id, had_previous_acm_entry, previous_acm_allowed))
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to restore the wall-contact ACM entry.");
      }
    }

    if (planned && should_execute) {
      const auto execute_result = move_group.execute(plan);
      const bool executed = execute_result == moveit::core::MoveItErrorCode::SUCCESS;
      RCLCPP_INFO(
        node->get_logger(), "execute() error_code = %d (SUCCESS = %d)", execute_result.val,
        moveit::core::MoveItErrorCode::SUCCESS);
      if (!executed) {
        planned = false;
      }
    }
  } else {
    // See this file's own doc comment: pure kinematic IK, no validity/collision constraint at
    // all. seed_state above becomes the seed setFromIK() perturbs FROM, matching the
    // collision_aware path's own setStartState() role.
    RCLCPP_WARN(
      node->get_logger(),
      "collision_aware:=false -- solving pure IK with NO collision awareness. Only use this to "
      "deliberately find a pose that penetrates real geometry; never execute a solution found "
      "this way through a collision-checked path.");
    const moveit::core::JointModelGroup * jmg =
      move_group.getRobotModel()->getJointModelGroup(planning_group);
    if (!ik_consistency_limits.empty()) {
      if (ik_consistency_limits.size() != kJointNames.size()) {
        RCLCPP_FATAL(
          node->get_logger(), "ik_consistency_limits must have %zu elements, got %zu.",
          kJointNames.size(), ik_consistency_limits.size());
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return 1;
      }
      // Given in this file's own kJointNames (lift, j1..j6) order for consistency with
      // seed_joint_positions above, but setFromIK() needs the group's OWN internal variable
      // order (jmg->getVariableNames(), not necessarily the same) -- reorder here rather than
      // silently misapplying limits to the wrong joints.
      const auto jmg_variable_names = jmg->getVariableNames();
      std::vector<double> reordered_limits(jmg_variable_names.size());
      for (size_t i = 0; i < jmg_variable_names.size(); ++i) {
        const auto it = std::find(
          kJointNames.begin(), kJointNames.end(), jmg_variable_names[i]);
        reordered_limits[i] = ik_consistency_limits[static_cast<size_t>(it - kJointNames.begin())];
      }
      Eigen::Isometry3d target_isometry;
      tf2::fromMsg(target_pose, target_isometry);
      planned = seed_state.setFromIK(
        jmg, target_isometry, tcp_link, reordered_limits, ik_timeout_s);
    } else {
      planned = seed_state.setFromIK(jmg, target_pose, tcp_link, ik_timeout_s);
    }
    RCLCPP_INFO(node->get_logger(), "setFromIK() success = %s", planned ? "true" : "false");
    if (planned) {
      solution_names = jmg->getVariableNames();
      solution_positions.resize(solution_names.size());
      for (size_t i = 0; i < solution_names.size(); ++i) {
        solution_positions[i] = seed_state.getVariablePosition(solution_names[i]);
      }
    }
  }

  if (planned) {
    std::string joint_pose_line = "[";
    for (size_t i = 0; i < solution_names.size(); ++i) {
      joint_pose_line += solution_names[i] + "=" + std::to_string(solution_positions[i]) +
        (i + 1 < solution_names.size() ? ", " : "");
    }
    joint_pose_line += "]";
    RCLCPP_INFO(node->get_logger(), "solution joint pose: %s", joint_pose_line.c_str());

    // Same ordering convenience pose_conditioning_probe/reference_pose_collision_probe's own
    // joint_pose param expects, if this needs to be handed to either of them next.
    std::string csv_in_kjointnames_order;
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      const auto it = std::find(solution_names.begin(), solution_names.end(), kJointNames[i]);
      if (it == solution_names.end()) {
        csv_in_kjointnames_order = "(joint '" + kJointNames[i] + "' not in this group)";
        break;
      }
      csv_in_kjointnames_order +=
        std::to_string(solution_positions[static_cast<size_t>(it - solution_names.begin())]) +
        (i + 1 < kJointNames.size() ? ", " : "");
    }
    RCLCPP_INFO(
      node->get_logger(), "as [lift, j1..j6] CSV: [%s]", csv_in_kjointnames_order.c_str());
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return planned ? 0 : 1;
}
