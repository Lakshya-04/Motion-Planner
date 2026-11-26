/**
 * @file manipulation_manager_node.cpp
 * @brief Wires the ported ManipulationManager to this repo's real backend (MoveIt
 *        CartesianPlanner/OmplPlanner + JTAC's FollowJointTrajectory action) and drives it
 *        through coverage_planner's real coverage_output.json, one wall column at a
 *        time: dock (fsp) -> contact (cartesian, JTAC force mode engaged) -> retract (fsp).
 * @details Single fixed base stance, same scope as this repo's own coverage_pass_demo_node.cpp
 *          -- driving the base to reach columns outside this stance's own reach is a separate,
 *          not-yet-built outer sweep layer (see manipulation_manager's own README: nothing
 *          upstream ever did this either, confirmed against sii's wall_cleaning_action_server.cpp).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <cstdio>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <sstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "coverage_task_msgs/action/execute_coverage.hpp"
#include "coverage_task_msgs/srv/mission_command.hpp"
#include "manipulation_manager/core/mission_verify.hpp"
#include "stand_navigator_msgs/action/navigate_to_stand.hpp"
#include "stand_navigator_msgs/action/wall_dock.hpp"
#include "reachability_msgs/srv/plan_stands.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "core_types/pose.hpp"
#include "motion_planner/core/motion_planner.hpp"
#include "motion_planner/core/planning_pipeline.hpp"
#include "motion_planner/core/scene_port.hpp"
#include "motion_planner/ros/planning_config.hpp"
#include "mobile_manipulator_profile/profile.hpp"
#include "reachability/core/io.hpp"
#include "ros_adapters/rclcpp_logger.hpp"

#include "manipulation_manager/ros/execution/retiming_wiring.hpp"
#include "manipulation_manager/core/execution/tool_feed_execution_port.hpp"
#include "manipulation_manager/adapters/reachability_column_port.hpp"
#include "manipulation_manager/adapters/reachability_column_ik_port.hpp"
#include "manipulation_manager/adapters/reachability_path_quality_port.hpp"
#include "manipulation_manager/core/column_ik_gate.hpp"
#include "manipulation_manager/core/tool_trace.hpp"
#include "manipulation_manager/core/base_stance_check.hpp"
#include "manipulation_manager/core/coverage_task.hpp"
#include "manipulation_manager/core/column_ordering.hpp"
#include "manipulation_manager/core/column_retry.hpp"
#include "manipulation_manager/core/frame_transform.hpp"
#include "manipulation_manager/core/manipulation_manager.hpp"
#include "manipulation_manager/core/mission_fsm.hpp"
#include "manipulation_manager/core/mission_ledger.hpp"
#include "manipulation_manager/core/dock_ik_selector.hpp"
#include "manipulation_manager/core/recovery_branch.hpp"
#include "manipulation_manager/core/recovery_jog.hpp"
#include "manipulation_manager/core/stand_check.hpp"
#include "manipulation_manager/core/column_chain.hpp"
#include "manipulation_manager/core/contact_connector.hpp"
#include "manipulation_manager/core/serpentine.hpp"
#include "manipulation_manager/core/stand_gate.hpp"
#include "manipulation_manager/core/wall_dock_gate.hpp"
#include "manipulation_manager/core/tool_mode.hpp"
#include "manipulation_manager/core/joint_names.hpp"
#include "manipulation_manager/core/travel_pose.hpp"
#include "manipulation_manager/core/wall_direction.hpp"
#include "manipulation_manager/ros/coverage/coverage_output_json_source.hpp"
#include "manipulation_manager/ros/execution/jtac_execution_adapter.hpp"

namespace manipulation_manager
{

namespace
{
// PlanningRequest::start_joint_state defaults to querying MoveGroupInterface's own
// CurrentStateMonitor when empty -- observed live here (same class of issue
// sii's own original manipulation_manager_node.cpp already flagged, and this repo's
// coverage_pass_demo_node.cpp already worked around for its own retract segment): that query
// can resolve to a stale/default state rather than the arm's true live pose, producing an
// instantly-aborted (not a real search-timeout) planning failure regardless of the robot's
// actual configuration. Reading /joint_states directly and priming continuity with it (see
// primeContinuityFromLiveState() below) sidesteps the query entirely for the first segment.
// The joint names of the planning group, in the group's order, as /joint_states publishes them: the repo's robot
// (UR10e on the telescopic lift) first, then the names of an earlier arm. The `group_joint_names` parameter
// overrides both.
const std::vector<std::vector<std::string>> kJointNameCandidates = {
  {"lift_joint", "arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint",
    "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint"},
  {"lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}};

/// The group's live joint values, or empty when /joint_states does not carry the group's joints (within 2 s).
/// An earlier version filled a missing joint with 0.0: with names that did not match the robot the "live" state
/// was the all-zero pose, and moving the arm "back" to it stretched the UR10e out and put the lift on its stop.
std::vector<double> readCurrentJointState(
  const rclcpp::Node::SharedPtr & node, const std::vector<std::vector<std::string>> & candidates,
  std::vector<std::string> * names_out = nullptr)
{
  std::map<std::string, double> latest;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&latest](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        latest[msg->name[i]] = msg->position[i];
      }
    });
  std::vector<std::string> names;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::vector<std::string> available;
    for (const auto & kv : latest) {
      available.push_back(kv.first);
    }
    names = selectJointNames(candidates, available);
    if (!names.empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::vector<double> result;
  for (const auto & name : names) {
    result.push_back(latest.at(name));
  }
  if (names_out) {
    *names_out = names;
  }
  return result;
}
}  // namespace

namespace
{
// wall_reference_pose's local Z must be the wall's OUTWARD normal, opposite the tool's approach
// axis. WallDirection::outwardNormalQuaternion() gives it for any wall direction; for the default
// +y wall it equals the constant coverage_pass_demo_node.cpp uses.
}  // namespace

/// Feeds `tool_actuation_node` over its topics (`<node>/window`, `<node>/trajectory_time`, `<node>/cancel`).
class TopicToolWindowSink : public manipulation_manager::ToolWindowSink
{
public:
  TopicToolWindowSink(rclcpp::Node & node, const std::string & executor_node)
  {
    const auto qos = rclcpp::QoS(10).reliable();
    window_ = node.create_publisher<std_msgs::msg::Float64MultiArray>("/" + executor_node + "/window", qos);
    time_ = node.create_publisher<std_msgs::msg::Float64>("/" + executor_node + "/trajectory_time", qos);
    cancel_ = node.create_publisher<std_msgs::msg::Bool>("/" + executor_node + "/cancel", qos);
  }
  void startWindow(double on_s, double end_s) override
  {
    std_msgs::msg::Float64MultiArray m;
    m.data = {on_s, end_s};
    window_->publish(m);
  }
  void trajectoryTime(double t) override
  {
    std_msgs::msg::Float64 m;
    m.data = t;
    time_->publish(m);
  }
  void cancel() override
  {
    std_msgs::msg::Bool m;
    m.data = true;
    cancel_->publish(m);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr window_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cancel_;
};

class ManipulationManagerNode : public rclcpp::Node
{
public:
  ManipulationManagerNode()
  : Node("manipulation_manager")
  {
    // Names come from the robot profile (`profile` parameter); the explicit frame_id and
    // planning_group parameters still win when set (an empty planning_group means "profile").
    std::string profile_warning;
    const auto profile = mobile_manipulator_profile::loadProfileOrDefaults(
      this->declare_parameter<std::string>("profile", mobile_manipulator_profile::kDefaultProfile),
      profile_warning);
    if (!profile_warning.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s", profile_warning.c_str());
    }
    // The profile's per-joint movement cost (lift first). Empty when the profile has none: every consumer then keeps
    // its own lift weight (dock_ik_lift_cost_weight, the planners' lift_cost_weight).
    profile_joint_costs_ = profile.joint_costs;
    if (!profile_joint_costs_.empty()) {
      std::ostringstream costs;
      for (size_t i = 0; i < profile_joint_costs_.size(); ++i) {
        costs << (i ? ", " : "") << profile_joint_costs_[i];
      }
      RCLCPP_INFO(this->get_logger(), "joint_costs from profile %s: [%s]", profile.name.c_str(), costs.str().c_str());
    }
    frame_id_ = this->declare_parameter<std::string>(
      "frame_id", profile.frameOr("world_base", "base_footprint"));
    planning_group_ = this->declare_parameter<std::string>("planning_group", "");
    if (planning_group_.empty()) {
      planning_group_ = profile.whole_body_group;
    }
    // Off by default: the coverage poses are taken as they are, in frame_id (the base at the origin).
    // Set (e.g. "odom"): the poses are in this fixed frame and every column is transformed into
    // frame_id with the TF at the time it starts, so the pass is right wherever the base stopped
    // (after stand_navigator drove it to a stand). wall_direction is then a direction in this frame.
    world_frame_ = this->declare_parameter<std::string>("world_frame", "");
    arm_base_frame_ = this->declare_parameter<std::string>("arm_base_frame", "arm_base_link");
    coverage_json_path_ = this->declare_parameter<std::string>(
      "coverage_json_path", "/tmp/wall_export/coverage_output.json");
    // base_link -> base_footprint fixed offset (see coverage_pass_demo_node.cpp's own frame_id
    // comment) -- coverage_output.json is authored in base_link.
    z_offset_ = this->declare_parameter<double>("z_offset_m", 0.31);
    {
      // Which way the wall lies from the robot in frame_id (a vector, normalised here). The default
      // is the original +y wall; the MuJoCo/Gazebo scenes have the wall at +x: [1, 0, 0].
      const auto v = this->declare_parameter<std::vector<double>>(
        "wall_direction", std::vector<double>{0.0, 1.0, 0.0});
      if (v.size() != 3) {
        throw std::invalid_argument("wall_direction needs three numbers");
      }
      wall_dir_ = WallDirection::from(v[0], v[1], v[2]);
    }
    // Resampling of each wall column and its z limits in frame_id (the defaults are the original
    // wall-cleaning values), and the contact speed.
    coverage_z_step_ = this->declare_parameter<double>("coverage_z_step_m", 0.1);
    coverage_z_min_ = this->declare_parameter<double>("coverage_z_min_m", 1.20);
    coverage_z_max_ = this->declare_parameter<double>("coverage_z_max_m", 1.60);
    coverage_eef_speed_ = this->declare_parameter<double>("coverage_eef_speed_mps", 0.02);
    dock_at_stroke_start_ = this->declare_parameter<bool>("dock_at_stroke_start", false);
    dock_standoff_ = this->declare_parameter<double>("dock_standoff_m", 0.12);
    // OMPL is randomised; from a base that turned side-on to the wall a dock sometimes needs more tries.
    dock_attempts_ = this->declare_parameter<int>("dock_attempts", 3);
    // The pose goal of a dock is sampled by move_group with a random IK, so which arm branch (shoulder, elbow, wrist
    // winding) the dock lands on changes from run to run, and some branches leave the next columns of the stand in
    // self-collision. true: the goal configuration is solved here by an IK seeded with the previous docked
    // configuration (the stowed pose before the first dock) and handed to the planner as a joint goal; the pose goal
    // stays as the fallback of the later attempts.
    dock_ik_seeded_ = this->declare_parameter<bool>("dock_ik_seeded", true);
    // true: after an approach segment the tool must be within approach_tolerance_m of the approach pose, otherwise the
    // attempt counts as failed (and the arm is jogged back / the continuity re-primed) instead of docking from where
    // the arm happens to be.
    // true: the dock joint goal is chosen among IK solutions from several seeds (the previous dock, the live state,
    // dock_seed_joint_state): the one closest to the previous dock (the live state before the first) wins, a solution
    // on another branch (sign of a dock_ik_branch_joints joint flips) or with the dock_ik_wrist_joint outside
    // [dock_ik_wrist_min_rad, dock_ik_wrist_max_rad] is dropped. None left: the seeded IK above as before. Off by default.
    dock_ik_continuity_ = this->declare_parameter<bool>("dock_ik_continuity", false);
    {
      const auto to_idx = [](const std::vector<int64_t> & v) {
          std::vector<size_t> out;
          for (const auto i : v) {
            if (i >= 0) {
              out.push_back(static_cast<size_t>(i));
            }
          }
          return out;
        };
      dock_ik_sel_.linear_joints = to_idx(
        this->declare_parameter<std::vector<int64_t>>("dock_ik_linear_joints", std::vector<int64_t>{0}));
      dock_ik_sel_.branch_joints = to_idx(
        this->declare_parameter<std::vector<int64_t>>("dock_ik_branch_joints", std::vector<int64_t>{1, 3}));
      dock_ik_sel_.wrist_joint = static_cast<int>(this->declare_parameter<int>("dock_ik_wrist_joint", 6));
      dock_ik_sel_.wrist_min_rad = this->declare_parameter<double>("dock_ik_wrist_min_rad", -3.2);
      dock_ik_sel_.wrist_max_rad = this->declare_parameter<double>("dock_ik_wrist_max_rad", 3.2);
      dock_ik_sel_.max_dq_rad = this->declare_parameter<double>("dock_ik_max_dq_rad", 3.2);
      // Reach preference of the continuity choice (needs dock_ik_continuity and the TF arm_base_frame
      // in frame_id): > 0 keeps the tool at or below this fraction of dock_ik_reach_m from the arm base by using the
      // lift joint (dock_ik_lift_joint, clamped into [dock_ik_lift_min_m, dock_ik_lift_max_m]). 0 = off (default).
      // Cost multiplier of the lift's delta in the continuity cost: 2.0 = lift motion twice as costly as arm motion, so
      // the choice prefers arm motion (and, with the reach preference, a lift that moves only as far as needed). 1.0 = off.
      dock_ik_sel_.lift_cost_weight = this->declare_parameter<double>("dock_ik_lift_cost_weight", 2.0);
      // The profile's joint_costs, when it has them, replace the lift weight above (range-normalised, per joint).
      dock_ik_sel_.joint_costs = profile_joint_costs_;
      dock_ik_sel_.reach_target_fraction = this->declare_parameter<double>("dock_ik_reach_fraction", 0.0);
      dock_ik_sel_.reach_m = this->declare_parameter<double>("dock_ik_reach_m", 0.0);
      dock_ik_sel_.lift_joint = static_cast<size_t>(std::max<int64_t>(0, this->declare_parameter<int>("dock_ik_lift_joint", 0)));
      dock_ik_sel_.lift_min_m = this->declare_parameter<double>("dock_ik_lift_min_m", 0.0);
      dock_ik_sel_.lift_max_m = this->declare_parameter<double>("dock_ik_lift_max_m", 1.5);
    }
    // recovery_keeps_branch: a recovery (jog after a short approach or a failed dock) stays on the IK branch the arm
    // is on. The jog goes toward the last good configuration only when that is on the live branch (else the last
    // docked one, else the arm stays), and the attempts after a short approach keep the joint goal chosen from the live
    // state (selectRecoveryGoal) instead of falling back to a pose goal the planner solves from its own samples.
    // Uses the dock_ik_* branch / wrist limits. Off by default (behaviour as before).
    recovery_keeps_branch_ = this->declare_parameter<bool>("recovery_keeps_branch", false);
    verify_approach_ = this->declare_parameter<bool>("verify_approach", true);
    approach_tolerance_m_ = this->declare_parameter<double>("approach_tolerance_m", 0.05);
    // true: after the dock the tool must be on the standoff pose (position and orientation) before the press;
    // a tool that arrives tilted (a redundant arm's goal solved to the wrong orientation) would press on the wall
    // with its edge. A column that fails the check is skipped as a failed dock (retried only through column_retries).
    verify_dock_ = this->declare_parameter<bool>("verify_dock", true);
    dock_tolerance_m_ = this->declare_parameter<double>("dock_tolerance_m", 0.02);
    dock_tolerance_rad_ = this->declare_parameter<double>("dock_tolerance_rad", 0.17);
    // > 0: before the approach / dock checks the tool is waited on until it stops moving (its position changes by
    // less than tcp_settle_tolerance_m over 0.4 s), at most this many seconds. A simulated arm that tracks a short
    // trajectory with a lag of a second or more reports the trajectory done while the tool is still 3 to 6 cm short,
    // and the fixed 0.5 s wait then fails a dock that would have arrived. 0 keeps the fixed wait only.
    tcp_settle_timeout_s_ = this->declare_parameter<double>("tcp_settle_timeout_s", 0.0);
    tcp_settle_tolerance_m_ = this->declare_parameter<double>("tcp_settle_tolerance_m", 0.002);
    // The seed of that IK: a configuration of the arm docked at the wall (planning group order, lift first). Every
    // dock is then solved from the same seed, so the lift / shoulder / wrist branch does not creep from one column to
    // the next (a chain of seeds drifted into upper arm <-> lift carriage self-collision at the last column of a
    // stand). Empty: the seed is the previous docked configuration, the stowed pose before the first dock.
    dock_seed_ = this->declare_parameter<std::vector<double>>("dock_seed_joint_state", std::vector<double>{});
    // The free-space move of a dock stops this far short of the dock pose and a short straight move closes the
    // gap (0 = plan straight to the dock pose). Planning to a pose a few cm from the wall fails or times out
    // when the tool mount clips the wall voxels on the way in; see docs/sim_mujoco.md.
    dock_approach_extra_m_ = this->declare_parameter<double>("dock_approach_extra_m", 0.0);
    // Chained columns (default on; false: every approach is a randomized OMPL plan): after a column's retract the tool stands at that column's dock pose. When the
    // next column of the same stand starts within chain_max_shift_m of it, the approach is first tried as one
    // straight Cartesian move (deterministic, short) instead of a randomized OMPL plan with a joint goal; a failed or
    // short chain move falls back to the normal approach, planned from where the tool really is.
    chain_columns_ = this->declare_parameter<bool>("chain_columns", true);
    chain_max_shift_m_ = this->declare_parameter<double>("chain_max_shift_m", 0.4);
    // Serpentine columns (default on): each column keeps the direction the coverage plan gave it (up, then down, ...)
    // and the retract backs the tool off the wall at the END of the stroke, not at its start, so the next column starts
    // where this one finished. false: every column runs bottom to top and the retract returns to the column's start
    // (a full-length trip back down the wall after every stroke).
    serpentine_columns_ = this->declare_parameter<bool>("serpentine_columns", true);
    // Contact connectors (default on): between two adjacent strips of one stand the tool stays on the wall and slides
    // to the start of the next strip (one short Cartesian move with the force loop still engaged) instead of
    // retract + dock + approach. Needs serpentine_columns to be useful (the next strip must start where this one ends).
    // Any refusal or failure falls back to the normal retract and approach; the result message counts both.
    contact_connectors_ = this->declare_parameter<bool>("contact_connectors", true);
    connector_cfg_.max_step_m = this->declare_parameter<double>("contact_connector_max_step_m", 0.15);
    connector_cfg_.plane_tol_m = this->declare_parameter<double>("contact_connector_plane_tol_m", 0.02);
    if (serpentine_columns_ && !dock_at_stroke_start_) {
      RCLCPP_WARN(
        this->get_logger(), "serpentine_columns without dock_at_stroke_start: every column docks at its middle "
        "height, so the stroke end the retract backs off at is not where the next column docks");
    }
    contact_y_overshoot_ = this->declare_parameter<double>("contact_y_overshoot_m", 0.045);
    max_wall_penetration_ = this->declare_parameter<double>("max_wall_penetration_m", 0.12);
    target_force_n_ = this->declare_parameter<double>("target_force_n", 5.0);
    press_settle_s_ = this->declare_parameter<double>("press_settle_s", 2.0);
    // 0.02 (was 0.01): after the unload the tool rests 5 to 11 mm above the backed-off reference and the joint-space
    // retract then starts with a 2 to 4 mm move toward the wall; with 0.01 the pad reached the wall plane in one
    // xArm 7 run (46 to 73 N at the first retract sample of every column, docs/sim_mujoco.md).
    unload_m_ = this->declare_parameter<double>("unload_m", 0.02);
    // After a column the force target is set to 0 and the admittance is switched off only once the measured force has
    // decayed below unload_release_force_n (0 = the old fixed 1.5 s wait), at most unload_timeout_s later: switching it
    // off while the tool still presses releases the offset at once and the arm snaps (100 to 160 N spikes at the
    // contact-to-retract handover on some arm configurations, docs/sim_mujoco.md).
    unload_release_force_n_ = this->declare_parameter<double>("unload_release_force_n", 2.0);
    unload_timeout_s_ = this->declare_parameter<double>("unload_timeout_s", 4.0);
    // ... and the tool has stopped: it lags its reference along the wall (friction, the admittance's own time constant)
    // and switching the admittance off while it still creeps makes it jump to the reference with the pad on the wall
    // (a 6 mm jump in 0.1 s gave 90 to 175 N). 0 = do not wait for the tool to settle.
    unload_release_speed_mps_ = this->declare_parameter<double>("unload_release_speed_mps", 0.001);
    // Tool kind. true (default): the contact flow above. false: a non-contact (spray) tool -- no force
    // engage, no press, no wall-contact allowance, strokes at the plan's stand-off (+ spray_extra_standoff_m
    // further from the wall) and the valve (std_msgs/Bool on valve_topic) is open during the stroke only.
    contact_ = this->declare_parameter<bool>("contact", true);
    spray_extra_standoff_ = this->declare_parameter<double>("spray_extra_standoff_m", 0.0);
    valve_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->declare_parameter<std::string>("valve_topic", "/tool/valve"),
      rclcpp::QoS(10).reliable().transient_local());

    // Base stance check against the reachability planner: "off" (default), "warn" (log and go on)
    // or "enforce" (do not run the plan from a stance the planner says cannot reach it).
    base_check_mode_ = this->declare_parameter<std::string>("base_check", "off");
    base_check_service_ = this->declare_parameter<std::string>(
      "base_check_service", "/reachability/plan_stands");
    // How long a multi-stand mission waits for the stand planner's reply. 60 s suits ~/plan_stands; with
    // base_check_service:=/reachability/plan_verified_stands (IK-confirmed stands) a wall takes minutes: raise it.
    stand_service_timeout_s_ = this->declare_parameter<double>("stand_service_timeout_s", 60.0);
    seed_start_from_live_state_ = this->declare_parameter<bool>("seed_start_from_live_state", false);
    base_check_lift_min_ = this->declare_parameter<double>("base_check_lift_min_m", 0.0);
    base_check_lift_max_ = this->declare_parameter<double>("base_check_lift_max_m", 0.0);
    // The stow pose is kept this far inside the lift's travel (base_check_lift_min_m .. max_m): a lift sent to its
    // end stop settles beyond it and MoveIt then refuses every plan ("start state out of bounds"). 0 = as is.
    travel_lift_margin_ = this->declare_parameter<double>("travel_lift_margin_m", 0.01);
    base_check_arm_mount_z_ = this->declare_parameter<double>("base_check_arm_mount_z_m", 0.0);
    base_check_tol_xy_ = this->declare_parameter<double>("base_check_tol_xy_m", 0.15);
    base_check_tol_yaw_ = this->declare_parameter<double>("base_check_tol_yaw_rad", 0.1);

    // Column pose sequence check (approach, contact, strokes, retract against an RM4D map). Off unless a map file
    // is given. It uses the dock parameters above (dock_standoff_m, dock_approach_extra_m, dock_at_stroke_start,
    // wall_direction) and base_check_lift_min_m / _max_m / base_check_arm_mount_z_m. Needs world_frame (the arm
    // base position comes from TF). Advisory in a single-stand run: split and unreachable columns are logged,
    // nothing is skipped; also handed to ManipulationManager::analyzeFeasibleSegments.
    column_reach_map_file_ = this->declare_parameter<std::string>("column_reachability_map_file", "");
    column_reach_min_score_ = this->declare_parameter<double>("column_reachability_min_score", 0.15);
    column_reach_count_ref_ = this->declare_parameter<double>("column_reachability_count_ref", 8.0);
    column_reach_lift_step_ = this->declare_parameter<double>("column_reachability_lift_step_m", 0.05);
    // How far past the plan surface the tool presses; negative = contact_y_overshoot_m (0 for a non-contact tool).
    column_reach_press_depth_ = this->declare_parameter<double>("column_reachability_press_depth_m", -1.0);

    // Which planning backends to use: a YAML selection (see docs/backend_selection.md), the
    // packaged default when empty. The planners are built in init().
    planning_config_ = this->declare_parameter<std::string>("planning_config", "");
    manager_.setLogger(std::make_shared<ros_adapters::RclcppLogger>(this->get_logger()));

    // The JTAC controller's node name: its action, parameters and force topic hang off it.
    controller_name_ = this->declare_parameter<std::string>(
      "controller_name", "combined_arm_lift_controller");
    force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/" + controller_name_ + "/target_force_n", 10);

    // Task interface (action `execute_coverage`, coverage_task_msgs). true (default): the old behaviour, run the
    // plan of the parameters once at start and exit. false: serve goals until shut down.
    run_on_start_ = this->declare_parameter<bool>("run_on_start", true);
    // Multi-stand goals: tries per stand's navigation before its columns are skipped, the navigation's time limit,
    // and the arm's travel pose (lift + 6 joints in the group's order; empty = the pose when the task started).
    // The widest stretch of wall (m, along it) one stand is trusted with: the reachability map does not know the arm's
    // self-collision or how badly it is conditioned at the edge of its reach, so plan_stands is asked for stands
    // group by group. 0 = the whole plan at once.
    stand_max_span_ = this->declare_parameter<double>("stand_max_span_m", 0.0);
    // Consecutive stands closer than this (m, plan view) are merged when plan_stands finds one stand for both of
    // their target sets (grouped requests can return stands a few cm apart). 0 = off.
    stand_merge_tol_ = this->declare_parameter<double>("stand_merge_tol_m", 0.0);
    // Minimum chassis-to-wall clearance (m) asked of plan_stands in every request (its min_clearance field); 0 = the
    // planner's own default (and its stand_min_clearance_m floor).
    stand_min_clearance_ = this->declare_parameter<double>("stand_min_clearance_m", 0.0);
    // A shell command run after each arrival at a stand, before its columns (empty = none).
    arrival_command_ = this->declare_parameter<std::string>("stand_arrival_command", "");
    // A shell command that takes the wall out of the planning scene. With it (and the arrival command, which
    // puts the wall back) a failed retract or stow is retried once without the wall: a tool left in the
    // wall box has a start state in collision, and no planner will move it out of one.
    wall_clear_command_ = this->declare_parameter<std::string>("wall_clear_command", "");
    // Failure handling. A column that failed (dock or contact stroke did nothing) is retried this many times, then
    // skipped and the run goes on; the result lists it under columns_failed.
    column_retries_ = std::max<int>(0, this->declare_parameter<int>("column_retries", 1));
    // Second pass over the columns that are still failed or partial after the main pass (core/column_retry.hpp): each gets
    // max_column_retries more attempts from a fresh stow + prime, docking with a dock IK solution other than the earlier
    // ones; retry_replan_stand also re-plans a nearby stand for a navigation or dock failure (multi-stand runs).
    // Off by default; the verify result gets a column_retries block.
    retry_cfg_.enabled = this->declare_parameter<bool>("retry_failed_columns", false);
    retry_cfg_.max_retries = std::max<int>(0, this->declare_parameter<int>("max_column_retries", 1));
    retry_cfg_.replan_stand = this->declare_parameter<bool>("retry_replan_stand", false);
    retry_cfg_.allow_branch_change = this->declare_parameter<bool>("retry_allow_branch_change", false);
    retry_cfg_.stop_after_failed_columns = std::max<int>(0, this->declare_parameter<int>("retry_stop_after_failed", 3));
    // A start state the planners refuse (the arm in self-collision at the end of its reach, a joint on its limit)
    // is moved a few centimetres of joint space back toward the last configuration known to be good (a stand's
    // stow pose, a column's dock pose) before the retract or stow is tried again. recovery_jog_max_rad refuses a
    // jog where a joint would have to move further than that. Joint limits (same order as the group) are optional.
    recovery_jog_ = this->declare_parameter<bool>("recovery_jog", true);
    jog_params_.max_total_rad = this->declare_parameter<double>("recovery_jog_max_rad", 2.5);
    jog_params_.max_speed = this->declare_parameter<double>("recovery_jog_speed", 0.15);
    jog_params_.lower = this->declare_parameter<std::vector<double>>(
      "recovery_joint_lower", std::vector<double>{});
    jog_params_.upper = this->declare_parameter<std::vector<double>>(
      "recovery_joint_upper", std::vector<double>{});
    // Empty: only the lift (the group's first joint) is limited, to the base check's travel. A lift that settled
    // 1.5 mm below its lower stop is what makes MoveIt refuse every start state ("outside bounds"), the jog
    // moves it back inside by travel_lift_margin_m.
    if (jog_params_.lower.empty() && jog_params_.upper.empty() &&
      base_check_lift_max_ > base_check_lift_min_)
    {
      jog_params_.lower = {base_check_lift_min_};
      jog_params_.upper = {base_check_lift_max_};
    }
    jog_params_.limit_margin = std::max(0.0, travel_lift_margin_);
    // The order of the columns within a stand: "input" (the plan's order, default), "nearest" (nearest neighbour by
    // travel time between one stroke's end and the next one's start) or "nearest_2opt".
    column_order_ = normalizeColumnOrder(this->declare_parameter<std::string>("column_order", "input"));
    nav_attempts_ = this->declare_parameter<int>("stand_nav_attempts", 2);
    nav_timeout_s_ = this->declare_parameter<double>("stand_nav_timeout_s", 240.0);
    // Stand-check gate (default off): after the base arrives, its error is compared with a tolerance from the stand's
    // reach margin (clamp(scale * margin, min, max); margin < 0 = unknown -> fallback) before the arm moves. Out of
    // tolerance: drive to the stand again, up to max_redocks times, then the stand is failed (columns not run).
    stand_check_enabled_ = this->declare_parameter<bool>("stand_check", false);
    stand_check_reach_margin_ = this->declare_parameter<double>("stand_check_reach_margin_m", -1.0);
    {
      StandCheckConfig sc;
      sc.min_tol_m = this->declare_parameter<double>("stand_check_min_tol_m", sc.min_tol_m);
      sc.max_tol_m = this->declare_parameter<double>("stand_check_max_tol_m", sc.max_tol_m);
      sc.margin_scale = this->declare_parameter<double>("stand_check_margin_scale", sc.margin_scale);
      sc.fallback_tol_m = this->declare_parameter<double>("stand_check_fallback_tol_m", sc.fallback_tol_m);
      sc.yaw_tol_rad = this->declare_parameter<double>("stand_check_yaw_tol_rad", sc.yaw_tol_rad);
      sc.max_redocks = this->declare_parameter<int>("stand_check_max_redocks", sc.max_redocks);
      stand_check_ = StandCheck(sc);
    }
    // Wall dock step (default off): after a successful navigate_to_stand, servo to the standoff and square to the wall
    // with the WallDock action; the measured standoff/yaw error goes through the stand-check logic (go / dock again /
    // give the stand up) and into the verify result.
    wall_dock_enabled_ = this->declare_parameter<bool>("wall_dock_after_navigate", false);
    wall_dock_action_ = this->declare_parameter<std::string>("wall_dock_action", "wall_dock");
    wall_dock_standoff_m_ = this->declare_parameter<double>("wall_dock_standoff_m", 0.30);
    wall_dock_timeout_s_ = this->declare_parameter<double>("wall_dock_timeout_s", 0.0);
    wall_dock_planned_standoff_ = this->declare_parameter<bool>("wall_dock_planned_standoff", false);
    wall_dock_cfg_.max_attempts = this->declare_parameter<int>("wall_dock_max_attempts", wall_dock_cfg_.max_attempts);
    wall_dock_cfg_.standoff_tol_m = this->declare_parameter<double>("wall_dock_standoff_tol_m", wall_dock_cfg_.standoff_tol_m);
    wall_dock_cfg_.yaw_tol_rad = this->declare_parameter<double>("wall_dock_yaw_tol_rad", wall_dock_cfg_.yaw_tol_rad);
    travel_joint_state_ = this->declare_parameter<std::vector<double>>(
      "travel_joint_state", std::vector<double>{});
    // The planning group's joints in the group's order as /joint_states names them (empty = the repo's UR10e and
    // lift names, then the earlier arm's), used to read the live arm state.
    group_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "group_joint_names", std::vector<std::string>{});
    // Which names read the live state: the parameter when set (nothing else then), else the planning group's joints
    // from the profile (a 7-DOF arm has arm_joint1..7, which the fixed list below does not know), then the fixed
    // list (UR10e, an earlier arm) as the last resort.
    if (!group_joint_names_.empty()) {
      joint_name_candidates_ = {group_joint_names_};
      RCLCPP_INFO(this->get_logger(), "live joint state: group_joint_names parameter (%zu joints)",
        group_joint_names_.size());
    } else {
      const auto derived = manipulation_manager::groupJointNames(
        planning_group_, profile.whole_body_group, profile.arm_group, profile.lift_joint, profile.arm_joints);
      if (!derived.empty()) {
        joint_name_candidates_.push_back(derived);
      }
      joint_name_candidates_.insert(
        joint_name_candidates_.end(), kJointNameCandidates.begin(), kJointNameCandidates.end());
      RCLCPP_INFO(this->get_logger(),
        "live joint state: group_joint_names empty, %s (%zu joints from profile '%s', group '%s')",
        derived.empty() ? "no profile joints for this group, using the fixed fallback lists" :
        "using the profile's joints for the group, fixed lists as the last resort",
        derived.size(), profile.name.c_str(), planning_group_.c_str());
    }
    // Where the tool's force is read for the feedback's and result's peak ("" = not read, peak -1).
    const auto force_topic = this->declare_parameter<std::string>(
      "force_topic", "/ft_conditioner/wrench_conditioned");
    if (!force_topic.empty()) {
      force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        force_topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          const auto & f = msg->wrench.force;
          const double mag = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
          force_now_ = mag;   // always: the release of the admittance waits for it to decay
          if (verify_trace_) {
            trace_.setForce(mag);   // the contact flag of the tool trace
          }
          if (!tracking_peak_) {
            return;
          }
          peak_force_ = std::max(peak_force_, mag);
          column_peak_ = std::max(column_peak_, mag);
        });
    }
    // Mission ledger ("" = off) and the contact-overload limit (0 = off; above it a column is skipped, not retried).
    ledger_ = MissionLedger(this->declare_parameter<std::string>("ledger_path", ""));
    contact_overload_n_ = this->declare_parameter<double>("contact_overload_n", 0.0);
    // Verify result written when a task ends ("" = off); max_peak 0 = no force limit in it.
    verify_result_path_ = this->declare_parameter<std::string>("verify_result_path", "");
    verify_max_peak_n_ = this->declare_parameter<double>("verify_max_peak_n", 0.0);
    // Geometric coverage in the verify result (needs verify_result_path): the tool's TF position is sampled while a
    // stroke runs and compared with the planned patch (geometric_coverage.hpp). Off by default. The swath width is not
    // read from the tool. A sample works when the tool force is at least verify_contact_force_n (contact tools; the
    // spray tool has no contact, so every stroking sample works) and the tool is within verify_max_standoff_m of the plane
    // through the plan's first pose (plan poses press past the wall by contact_y_overshoot, hence the 4 cm default).
    verify_geometric_ = this->declare_parameter<bool>("verify_geometric_coverage", false) && !verify_result_path_.empty();
    verify_geo_cfg_.swath_m = this->declare_parameter<double>("verify_swath_m", 0.10);
    verify_geo_cfg_.cell_m = this->declare_parameter<double>("verify_cell_m", 0.01);
    verify_contact_force_n_ = this->declare_parameter<double>("verify_contact_force_n", 1.0);
    verify_geo_cfg_.max_gap_m = this->declare_parameter<double>("verify_max_gap_m", 0.05);
    verify_geo_cfg_.max_standoff_m = this->declare_parameter<double>("verify_max_standoff_m", 0.04);
    verify_min_geometric_percent_ = this->declare_parameter<double>("verify_min_geometric_percent", 0.0);
    verify_trace_hz_ = this->declare_parameter<double>("verify_trace_hz", 20.0);
    // Opt-in, report only: how close the executed joint path came to a singularity or a joint limit (warnings in the
    // verify result; it never fails a run). Needs a chain file of the arm joints (reachability/scripts/urdf_to_chain.py).
    verify_path_quality_ = this->declare_parameter<bool>("verify_path_quality", false) && !verify_result_path_.empty();
    verify_path_chain_file_ = this->declare_parameter<std::string>("verify_path_chain_file", "");
    verify_path_skip_joints_ = this->declare_parameter<int>("verify_path_skip_joints", 1);
    verify_path_cfg_.min_sigma = this->declare_parameter<double>("verify_path_min_sigma", 0.02);
    verify_path_cfg_.min_limit_margin = this->declare_parameter<double>("verify_path_min_limit_margin", 0.02);
    verify_trace_ = verify_geometric_ || verify_path_quality_;
    // Opt-in whole-column IK continuity check before a column is run (column_ik_gate.hpp): IK along approach, strokes and
    // retract from the current seed; on a branch break the same column is tried from other lift heights (the first that
    // is continuous seeds the dock IK), else the column is skipped (reason plan_failed). Off by default.
    column_ik_check_ = this->declare_parameter<bool>("column_ik_check", false);
    column_ik_lift_step_ = this->declare_parameter<double>("column_ik_lift_step_m", 0.1);
    column_ik_reject_on_refused_ = this->declare_parameter<bool>("column_ik_reject_on_refused", false);
    column_ik_ratio_limit_ = this->declare_parameter<double>("column_ik_ratio_limit", 6.0);
    column_ik_branch_joints_ = this->declare_parameter<std::vector<int64_t>>(
      "column_ik_branch_joints", std::vector<int64_t>{1, 3});
    column_ik_prismatic_joints_ = this->declare_parameter<std::vector<int64_t>>(
      "column_ik_prismatic_joints", std::vector<int64_t>{0});
    column_ik_continuous_joints_ = this->declare_parameter<std::vector<int64_t>>(
      "column_ik_continuous_joints", std::vector<int64_t>{});
    mission_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "mission_command", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "pause") {
          RCLCPP_INFO(this->get_logger(), "mission: pause %s", mission_fsm_.requestPause() ? "requested" : "ignored");
        } else if (msg->data == "resume") {
          RCLCPP_INFO(this->get_logger(), "mission: resume %s", mission_fsm_.resume() ? "accepted" : "ignored");
        }
      });
    // Typed counterpart of the mission_command topic (which stays): pause / resume / status with the FSM state back.
    mission_command_srv_ = this->create_service<MissionCommandSrv>(
      "mission_command", [this](
        const std::shared_ptr<MissionCommandSrv::Request> req, std::shared_ptr<MissionCommandSrv::Response> res) {
        const auto r = applyMissionCommand(mission_fsm_, req->command);
        res->accepted = r.accepted;
        res->state = r.state;
        res->pause_pending = r.pause_pending;
        res->message = r.message;
        RCLCPP_INFO(this->get_logger(), "mission command '%s': %s (%s)", req->command.c_str(),
          r.accepted ? "accepted" : "rejected", r.state.c_str());
      });
    nav_client_ = rclcpp_action::create_client<NavigateToStand>(this, "navigate_to_stand");
    if (wall_dock_enabled_) {
      wall_dock_client_ = rclcpp_action::create_client<WallDock>(this, wall_dock_action_);
    }
    action_server_ = rclcpp_action::create_server<ExecuteCoverage>(
      this, "execute_coverage",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteCoverage::Goal>) {
        if (busy_ || pending_goal_) {
          RCLCPP_WARN(this->get_logger(), "execute_coverage goal rejected: a task is already running");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        // Runs while the node is spun (inside every wait of the task): flags only, the task cleans up itself.
        cancel_requested_ = true;
        execution_adapter_->cancelCurrent();
        cancelNavigation();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {pending_goal_ = handle;});
  }

  /// Two-phase init: JtacExecutionAdapter needs a live node shared_ptr for its action client,
  /// which isn't available yet inside the constructor above (shared_from_this() requires the
  /// object to already be owned by a shared_ptr) -- call once right after make_shared().
  bool init()
  {
    if (!buildPlanners()) {
      return false;
    }
    {
      // TF is needed in every run (ARMSTATE tool pose, approach and dock verification), not only with a world
      // frame; its own spin thread: the node itself is only spun in bursts while a request waits
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), true);
    }
    if (!initColumnReachability()) {
      return false;
    }
    if (!initVerifyTrace()) {
      return false;
    }
    initColumnIkCheck();
    execution_adapter_ = std::make_shared<JtacExecutionAdapter>(
      shared_from_this(), "/" + controller_name_ + "/follow_joint_trajectory");
    // Opt-in: the tool actuation executor (tool_actuation_node) owns the valve during a spray stroke. The
    // manager then publishes the stroke's window and trajectory time to it and no longer opens the valve itself.
    std::shared_ptr<manipulation_manager::ExecutionPort> exec_port = execution_adapter_;
    if (this->declare_parameter<bool>("tool_actuation_feed", false) && !contact_) {
      tool_feed_ = std::make_shared<manipulation_manager::ToolFeedExecutionPort>(
        execution_adapter_,
        std::make_shared<TopicToolWindowSink>(
          *this, this->declare_parameter<std::string>("tool_actuation_node", "tool_actuation")));
      exec_port = tool_feed_;
      RCLCPP_INFO(this->get_logger(), "tool actuation feed on: the executor node drives the valve during a stroke");
    }
    manager_.setExecutionPort(wrapWithRetiming(shared_from_this(), exec_port, planning_group_));
    if (!contact_) {
      valve_.update(true);   // force the first command out: the valve is explicitly closed at start
      setValve(false);
    }

    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      shared_from_this(), "/" + controller_name_);
    if (!param_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "%s param service unavailable", controller_name_.c_str());
      return false;
    }
    return true;
  }

  ~ManipulationManagerNode() override
  {
    if (plugin_executor_) {
      plugin_executor_->cancel();
    }
    if (plugin_spin_thread_.joinable()) {
      plugin_spin_thread_.join();
    }
  }

  bool runOnStart() const {return run_on_start_;}

  /// Opens or closes the spray valve; only a change is published.
  void setValve(bool open)
  {
    if (tool_feed_) {
      // The executor opens the valve from the stroke's trajectory time; here only "off" goes out (always, as a
      // backstop that does not depend on the gate's state).
      if (open) {return;}
    } else if (!valve_.update(open)) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = open;
    valve_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "valve %s", open ? "ON" : "OFF");
  }

  void setAdmittanceEngaged(bool enabled)
  {
    std::vector<bool> axes = {false, false, enabled, false, false, false};
    auto future = param_client_->set_parameters({rclcpp::Parameter("admittance.selected_axes",
      axes)});
    rclcpp::spin_until_future_complete(shared_from_this(), future);
    admittance_engaged_ = enabled;
  }

  /**
   * @brief The old one-shot run (run_on_start): the plan of the parameters, once, from this stance:
   *        dock -> contact -> retract per wall column, in the column order CoverageOutputJsonSource
   *        already sorted left-to-right.
   */
  void runCoveragePlan()
  {
    const TaskResult result = executeTask(TaskRequest{});
    if (result.loaded) {
      RCLCPP_INFO(
        this->get_logger(), "Coverage plan complete: done=%zu partial=%zu failed=%zu",
        tracker_.done().size(), tracker_.partial().size(), tracker_.failed().size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Coverage plan not run: %s", result.message.c_str());
    }
  }

  /// Serves `execute_coverage` goals until shutdown. The node is spun only here and inside the task's waits (the
  /// manager's blocking calls spin it themselves), so a goal is run from this loop, one at a time.
  void serve()
  {
    RCLCPP_INFO(this->get_logger(), "serving the execute_coverage action");
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      if (pending_goal_) {
        const auto handle = pending_goal_;
        pending_goal_.reset();
        runGoal(handle);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

private:
  using ExecuteCoverage = coverage_task_msgs::action::ExecuteCoverage;
  using MissionCommandSrv = coverage_task_msgs::srv::MissionCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCoverage>;
  using NavigateToStand = stand_navigator_msgs::action::NavigateToStand;
  using WallDock = stand_navigator_msgs::action::WallDock;

  /// What a task is asked to do; the defaults are the parameters' (the old one-shot behaviour).
  struct TaskRequest
  {
    std::string json_path;            ///< empty: the parameter
    bool multi_stand{false};
    int tool_mode{0};                 ///< ExecuteCoverage::TOOL_*
    std::vector<size_t> columns;      ///< empty: all
    bool resume{false};               ///< keep the ledger of the same plan and skip its DONE columns
  };

  struct TaskResult
  {
    bool loaded{false};               ///< the plan was loaded and the run started (false: `message` says why)
    bool canceled{false};
    std::string message;
  };

  void runGoal(const std::shared_ptr<GoalHandle> & handle)
  {
    busy_ = true;
    const auto goal = handle->get_goal();
    auto result = std::make_shared<ExecuteCoverage::Result>();
    result->peak_force_n = -1.0;
    if (handle->is_canceling()) {
      result->message = "canceled";
      handle->canceled(result);
      busy_ = false;
      return;
    }
    TaskRequest req;
    req.json_path = goal->coverage_json_path;
    req.multi_stand = goal->multi_stand;
    req.tool_mode = goal->tool_mode;
    req.columns.assign(goal->columns.begin(), goal->columns.end());
    req.resume = goal->resume;
    feedback_sink_ = [handle](const std::shared_ptr<ExecuteCoverage::Feedback> & fb) {
        handle->publish_feedback(fb);
      };
    const TaskResult run = executeTask(req);
    feedback_sink_ = nullptr;

    result->columns_done.assign(tracker_.done().begin(), tracker_.done().end());
    result->columns_partial.assign(tracker_.partial().begin(), tracker_.partial().end());
    result->columns_failed.assign(tracker_.failed().begin(), tracker_.failed().end());
    for (const auto & s : tracker_.stands()) {
      coverage_task_msgs::msg::StandOutcome o;
      o.stand_index = static_cast<uint32_t>(s.index);
      o.reached = s.reached;
      o.message = s.message;
      o.columns_done.assign(s.done.begin(), s.done.end());
      o.columns_partial.assign(s.partial.begin(), s.partial.end());
      o.columns_failed.assign(s.failed.begin(), s.failed.end());
      result->stands.push_back(o);
    }
    result->peak_force_n = peak_force_;
    result->message = run.message;
    result->success = run.loaded && !run.canceled && tracker_.success();
    RCLCPP_INFO(
      this->get_logger(), "execute_coverage finished: %s (%s)",
      result->success ? "success" : "not successful", run.message.c_str());
    if (run.canceled || handle->is_canceling()) {
      handle->canceled(result);
    } else if (!run.loaded) {
      handle->abort(result);
    } else {
      handle->succeed(result);   // ran to its end: `success` and the column lists say how it went
    }
    busy_ = false;
  }

  bool stop() const {return cancel_requested_ || !rclcpp::ok();}

  /// Spins the node for `seconds` (so a cancel is heard); false when stopped early.
  bool waitFor(double seconds, bool honour_cancel = true)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(shared_from_this());
      if (honour_cancel && stop()) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true;
  }

  /// The fixed 0.5 s wait of the approach and dock checks, then (tcp_settle_timeout_s > 0) a wait until the tool
  /// has stopped: its position moved less than tcp_settle_tolerance_m over the last 0.4 s, or the timeout passed.
  void waitTcpSettled()
  {
    waitFor(0.5, false);
    if (tcp_settle_timeout_s_ <= 0.0) {
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    const auto limit = start + std::chrono::duration<double>(tcp_settle_timeout_s_);
    std::deque<std::pair<std::chrono::steady_clock::time_point, std::array<double, 3>>> hist;
    while (std::chrono::steady_clock::now() < limit) {
      FrameTransform tcp;
      if (!lookup(frame_id_, "tcp_link", tcp)) {
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      hist.emplace_back(now, std::array<double, 3>{tcp.t[0], tcp.t[1], tcp.t[2]});
      while (hist.size() > 1 && now - hist[1].first >= std::chrono::milliseconds(400)) {
        hist.pop_front();
      }
      if (now - hist.front().first >= std::chrono::milliseconds(400)) {
        double moved = 0.0;
        for (const auto & h : hist) {
          moved = std::max(
            moved, std::sqrt(
              std::pow(h.second[0] - tcp.t[0], 2) + std::pow(h.second[1] - tcp.t[1], 2) +
              std::pow(h.second[2] - tcp.t[2], 2)));
        }
        if (moved < tcp_settle_tolerance_m_) {
          RCLCPP_INFO(
            this->get_logger(), "tool settled after %.1f s",
            std::chrono::duration<double>(now - start).count() + 0.5);
          return;
        }
      }
      waitFor(0.05, false);
    }
    RCLCPP_WARN(this->get_logger(), "tool still moving after %.1f s of settling", tcp_settle_timeout_s_);
  }

  void setState(const std::string & state, int column = -2)
  {
    state_ = state;
    if (column != -2) {
      column_ = column;
    }
    if (verify_trace_) {
      trace_.setStroking(state_ == "stroking", static_cast<size_t>(std::max(0, column_)));
    }
    if (!feedback_sink_) {
      return;
    }
    auto fb = std::make_shared<ExecuteCoverage::Feedback>();
    fb->state = state_;
    fb->stand_index = static_cast<uint32_t>(stand_index_);
    fb->stand_count = static_cast<uint32_t>(stand_count_);
    fb->column = column_;
    fb->columns_done = static_cast<uint32_t>(tracker_.done().size());
    fb->columns_failed = static_cast<uint32_t>(tracker_.partial().size() + tracker_.failed().size());
    fb->peak_force_n = peak_force_;
    fb->mission_state = toString(mission_fsm_.state());
    feedback_sink_(fb);
  }

  /// After a cancel: no force target, the admittance off (after the unload ramp the normal flow also gives it),
  /// the spray valve closed. The arm stays where it stopped.
  void abortCleanup()
  {
    tracking_peak_ = false;
    {
      std_msgs::msg::Float64 zero;
      zero.data = 0.0;
      force_pub_->publish(zero);
    }
    if (admittance_engaged_) {
      waitFor(1.5, false);
      setAdmittanceEngaged(false);
    }
    setValve(false);
    RCLCPP_WARN(this->get_logger(), "task canceled: motion stopped, admittance off, force target 0");
  }

  /// Runs one task (executeTaskImpl) and then writes the verify result when `verify_result_path` is set.
  TaskResult executeTask(const TaskRequest & req)
  {
    column_peaks_.clear();
    verify_stand_columns_.clear();
    verify_wall_dock_.clear();
    retry_report_ = ColumnRetryReport{};
    column_dock_q_.clear();
    trace_.reset();
    const TaskResult out = executeTaskImpl(req);
    if (!verify_result_path_.empty()) {
      MissionVerifyInput in;
      in.loaded = out.loaded;
      in.canceled = out.canceled;
      in.tracker_success = tracker_.success();
      in.done = tracker_.done();
      in.partial = tracker_.partial();
      in.failed = tracker_.failed();
      in.peak_force_n = peak_force_;
      in.column_peaks_n = column_peaks_;
      in.max_peak_force_n = verify_max_peak_n_;
      in.message = out.message;
      in.report_path = ledger_.path();
      in.stand_columns = verify_stand_columns_;
      in.wall_dock = verify_wall_dock_;
      in.column_retry = retry_report_;
      addTraceToVerify(in);
      const auto result = buildMissionVerify(in);
      if (writeMissionVerify(verify_result_path_, missionVerifyToJson(result, in))) {
        RCLCPP_INFO(
          this->get_logger(), "verify result (%s, columns %.0f%%) written to %s",
          result.passed ? "passed" : "failed", result.coverage_percent, verify_result_path_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "cannot write the verify result to %s", verify_result_path_.c_str());
      }
    }
    return out;
  }

  /// Runs one task: loads the plan, then either the whole plan from here or, multi-stand, the stands in order.
  TaskResult executeTaskImpl(const TaskRequest & req)
  {
    TaskResult out;
    cancel_requested_ = false;
    execution_adapter_->setCancelled(false);
    peak_force_ = -1.0;
    tracking_peak_ = false;
    column_ = -1;
    stand_index_ = 0;
    stand_count_ = 1;
    tracker_.beginSingle();

    const bool saved_contact = contact_;
    if (req.tool_mode == ExecuteCoverage::Goal::TOOL_CONTACT) {
      contact_ = true;
    } else if (req.tool_mode == ExecuteCoverage::Goal::TOOL_NON_CONTACT) {
      contact_ = false;
    }
    if (!contact_ && saved_contact) {
      valve_.update(true);   // force the first command out: the valve is explicitly closed at start
      setValve(false);
    }
    const auto restore = [&]() {contact_ = saved_contact;};

    const std::string path = req.json_path.empty() ? coverage_json_path_ : req.json_path;
    std::vector<CoverageSegmentIntent> intents;
    try {
      // In a world frame the poses are loaded as they are and moved into frame_id column by column.
      CoverageOutputJsonSource source(
        path, world_frame_.empty() ? frame_id_ : world_frame_, z_offset_, coverage_z_step_,
        coverage_z_min_, coverage_z_max_, coverage_eef_speed_, wall_dir_);
      source.setFollowPlanDirection(serpentine_columns_);
      intents = source.getCoveragePlan();
    } catch (const std::exception & e) {
      out.message = std::string("cannot load the coverage plan ") + path + ": " + e.what();
      restore();
      return out;
    }
    RCLCPP_INFO(
      this->get_logger(), "Loaded %zu wall column(s) from %s", intents.size(), path.c_str());

    // The columns to run: their index in the plan, and the intents
    std::vector<size_t> ids;
    std::vector<CoverageSegmentIntent> sel;
    for (size_t c = 0; c < intents.size(); ++c) {
      if (intents[c].poses.empty()) {
        continue;
      }
      if (!req.columns.empty() &&
        std::find(req.columns.begin(), req.columns.end(), c) == req.columns.end())
      {
        continue;
      }
      ids.push_back(c);
      sel.push_back(intents[c]);
    }
    if (sel.empty()) {
      out.message = "no column to run";
      restore();
      return out;
    }

    // Ledger: a resume of the same plan drops the columns it holds as done; the rest run as usual.
    mission_fsm_.reset();
    mission_fsm_.start();
    if (ledger_.persistent()) {
      std::ifstream plan_in(path);
      std::stringstream plan_bytes;
      plan_bytes << plan_in.rdbuf();
      if (!ledger_.load()) {
        RCLCPP_WARN(
          this->get_logger(), "mission ledger %s was corrupted, moved aside, starting empty",
          ledger_.path().c_str());
      }
      ledger_.begin(missionIdFromBytes(plan_bytes.str()), intents.size(), req.resume);
      if (req.resume) {
        std::vector<size_t> keep_ids;
        std::vector<CoverageSegmentIntent> keep_sel;
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ledger_.resumeSkips(ids[i])) {
            if (ledger_.isDone(ids[i])) {
              tracker_.columnResumed(ids[i]);
            } else {
              tracker_.columnSkipped(ids[i]);   // skipped for an overload: not retried blindly
            }
          } else {
            keep_ids.push_back(ids[i]);
            keep_sel.push_back(sel[i]);
          }
        }
        RCLCPP_INFO(
          this->get_logger(), "resume: %zu column(s) already settled in the ledger, %zu to run",
          ids.size() - keep_ids.size(), keep_ids.size());
        ids = keep_ids;
        sel = keep_sel;
        if (sel.empty()) {
          out.loaded = true;
          out.message = "resume: nothing left to run (" + tracker_.summary() + ")";
          mission_fsm_.stop();
          restore();
          return out;
        }
      }
    }

    if (verify_trace_) {
      std::vector<PlannedStroke> planned;
      for (size_t i = 0; i < ids.size(); ++i) {
        PlannedStroke st;
        st.column = ids[i];
        for (const auto & p : sel[i].poses) {
          st.points.push_back({p.x, p.y, p.z});
        }
        planned.push_back(std::move(st));
      }
      trace_.setPlan(std::move(planned));
    }

    TaskStateMachine machine(nav_attempts_);
    connector_log_ = ConnectorLog{};
    connector_carry_ = false;
    next_in_stand_ = nullptr;
    if (!req.multi_stand) {
      machine.startSingle();
      logColumnReach(sel);
      if (!baseStanceAllowsRun(sel)) {
        out.message = "base stance check failed (base_check enforce)";
        restore();
        return out;
      }
      out.loaded = true;
      setState("priming");
      primeContinuityFromLiveState();
      std::vector<size_t> all(sel.size());
      for (size_t i = 0; i < all.size(); ++i) {
        all[i] = i;
      }
      last_good_state_ = readCurrentJointState(shared_from_this(), joint_name_candidates_);
      const auto order = orderColumns(all, sel);
      for (size_t k = 0; k < order.size(); ++k) {
        const size_t i = order[k];
        if (stop()) {
          break;
        }
        setNextInStand(k + 1 < order.size() ? &sel[order[k + 1]] : nullptr, k + 1 < order.size() ? ids[order[k + 1]] : 0);
        tracker_.columnFinished(0, ids[i], runColumnWithRetry(sel[i], ids[i]));
      }
      setNextInStand(nullptr, 0);
      retryFailedColumns(sel, ids, nullptr);
      if (!stop()) {
        machine.singleFinished();
      }
    } else {
      out.loaded = runStands(machine, sel, ids, out.message);
    }

    if (stop() && rclcpp::ok()) {
      machine.cancel();
    }
    if (machine.state() == TaskState::kCanceled) {
      abortCleanup();
      out.canceled = true;
      out.message = "canceled";
    } else {
      setValve(false);
      if (out.message.empty()) {
        out.message = tracker_.summary();
        if (contact_connectors_ && !connector_log_.summary().empty()) {
          out.message += "; " + connector_log_.summary();
        }
      }
    }
    setState(out.canceled ? "canceled" : "done", -1);
    mission_fsm_.stop();
    restore();
    return out;
  }

  /// The multi-stand loop: plan the stands, give each column to a stand, and for each stand drive there, run its
  /// columns, go on. Returns false (with `why`) when no stand could be planned.
  bool runStands(
    TaskStateMachine & machine, const std::vector<CoverageSegmentIntent> & sel,
    const std::vector<size_t> & ids, std::string & why)
  {
    if (world_frame_.empty()) {
      why = "multi_stand needs the world_frame parameter (the stands are poses in it)";
      return false;
    }
    machine.startMulti();
    setState("planning_stands");
    // The arm's pose before it first moves: what it goes back to before every drive.
    travel_state_ = !travel_joint_state_.empty() ?
      travel_joint_state_ : readCurrentJointState(shared_from_this(), joint_name_candidates_);
    if (travel_state_.empty()) {
      why = "no travel pose: /joint_states does not carry the planning group's joints (set group_joint_names or "
        "travel_joint_state)";
      return false;
    }
    travel_state_ = keepLiftOffEndStops(
      travel_state_, base_check_lift_min_, base_check_lift_max_, travel_lift_margin_);
    arm_deployed_ = false;

    std::vector<StanceStand> stands;
    std::vector<std::vector<size_t>> stand_targets;
    if (!planStands(sel, stands, stand_targets, why)) {
      machine.standsFailed();
      return false;
    }
    std::vector<size_t> sizes;
    for (const auto & c : sel) {
      sizes.push_back(c.poses.size());
    }
    const StanceWall plan_wall = stanceWall(sel);
    const auto assignment = assignColumnsToStands(sizes, stand_targets);
    // Stands that run no column are not visited.
    std::vector<StanceStand> visit;
    std::vector<std::vector<size_t>> visit_columns;   // indices into sel
    for (size_t s = 0; s < stands.size(); ++s) {
      if (!assignment.columns_per_stand[s].empty()) {
        visit.push_back(stands[s]);
        visit_columns.push_back(assignment.columns_per_stand[s]);
      }
    }
    stand_count_ = visit.size();
    tracker_.beginMulti(visit.size());
    for (const size_t c : assignment.unassigned) {
      RCLCPP_WARN(
        this->get_logger(), "column %zu (%s): no stand reaches it, not run", ids[c],
        sel[c].label.c_str());
      tracker_.columnSkipped(ids[c]);
    }
    if (!machine.standsPlanned(visit.size())) {
      why = "no stand covers the plan";
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "%zu stand(s) to visit", visit.size());
    verify_stand_columns_.clear();
    for (size_t vs = 0; vs < visit_columns.size(); ++vs) {
      for (const size_t c : visit_columns[vs]) {
        verify_stand_columns_[vs].push_back(ids[c]);
      }
    }

    while (!machine.terminal()) {
      if (stop()) {
        break;
      }
      const size_t s = machine.stand();
      stand_index_ = s;
      if (machine.state() == TaskState::kNavigating) {
        std::string msg;
        bool ok = true;
        bool gate_abort = false;
        if (arm_deployed_) {
          setState("stowing", -1);
          if (stowArm()) {
            arm_deployed_ = false;
          } else {
            ok = false;
            msg = stop() ? "canceled" : "could not stow the arm for the drive";
          }
        }
        if (ok) {
          setState("navigating", -1);
          ok = navigateToStand(visit[s], msg);
          if (ok && stand_check_enabled_) {
            ok = gateStand(visit[s], msg, gate_abort);
          }
          if (ok && wall_dock_enabled_) {
            ok = wallDockStand(s, plannedWallDockStandoff(visit[s], plan_wall), msg, gate_abort);
          }
        }
        if (stop()) {
          break;
        }
        const auto verdict = resolveNavigation(machine, ok, gate_abort);   // a stand the gate gave up is not retried
        if (verdict == NavVerdict::kRun) {
          tracker_.standReached(s, true, "");
          if (!arrival_command_.empty()) {
            // MoveIt keeps the obstacles in the robot's root frame, so after the base moved the wall has to be put
            // into its scene again (the sim does it with world_model_loader_node; a live feed would not need it).
            setState("scene", -1);
            const int rc = std::system(arrival_command_.c_str());
            if (rc != 0) {
              RCLCPP_WARN(this->get_logger(), "stand_arrival_command exited with %d", rc);
            }
            waitFor(1.0);
          }
        } else if (verdict == NavVerdict::kRetry) {
          RCLCPP_WARN(this->get_logger(), "stand %zu: %s, trying again", s, msg.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "stand %zu not reached (%s): its columns are not run", s, msg.c_str());
          tracker_.standReached(s, false, msg);
          for (const size_t c : visit_columns[s]) {
            tracker_.columnFinished(s, ids[c], ColumnOutcome::kFailed);
            ledger_.markSkipped(ids[c], static_cast<int>(s), SkipReason::kNavFailed);
          }
        }
      } else if (machine.state() == TaskState::kRunningColumns) {
        setState("priming", -1);
        logArmState("arrival");
        primeContinuityFromLiveState();
        last_good_state_ = travel_state_;   // the stowed arm: where a jog goes back to
        arm_stuck_ = false;
        chain_valid_ = false;
        if (serpentine_columns_) {
          // the arm was stowed for the drive and the base stands elsewhere: the docked branch of the last stand is no
          // reference for the first dock here (it made every candidate 'wrong branch' and fell back to a raw IK solution a
          // turn away from the stow pose)
          dock_branch_seed_.clear();
        }
        const auto order = orderColumns(visit_columns[s], sel);
        for (size_t k = 0; k < order.size(); ++k) {
          const size_t c = order[k];
          if (stop()) {
            break;
          }
          arm_deployed_ = true;
          setNextInStand(k + 1 < order.size() ? &sel[order[k + 1]] : nullptr, k + 1 < order.size() ? ids[order[k + 1]] : 0);
          tracker_.columnFinished(s, ids[c], runColumnWithRetry(sel[c], ids[c]));
        }
        setNextInStand(nullptr, 0);
        connector_carry_ = false;
        if (!stop()) {
          machine.standFinished();
        }
      } else {
        break;
      }
    }
    retryFailedColumns(sel, ids, &visit);
    return true;
  }

  /// plan_stands for the plan from where the arm base is now (world frame). `targets_per_stand` are indices into
  /// the concatenated poses of `sel`. With stand_max_span_m > 0 the columns are asked for in groups no wider than
  /// that along the wall (nearest end first, each group starting from the previous group's last stand), so one stand
  /// is never trusted with more than the span.
  bool planStands(
    const std::vector<CoverageSegmentIntent> & sel, std::vector<StanceStand> & stands,
    std::vector<std::vector<size_t>> & targets_per_stand, std::string & why)
  {
    FrameTransform arm_in_world, base_in_world;
    if (!lookup(world_frame_, arm_base_frame_, arm_in_world) ||
      !lookup(world_frame_, frame_id_, base_in_world))
    {
      why = "no TF " + world_frame_ + " -> " + arm_base_frame_ + " / " + frame_id_;
      return false;
    }
    auto client = this->create_client<reachability_msgs::srv::PlanStands>(base_check_service_);
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      why = "service " + base_check_service_ + " is not available";
      return false;
    }

    // position along the wall (the horizontal direction at right angles to wall_direction) of each column
    const double tx = -wall_dir_.d[1];
    const double ty = wall_dir_.d[0];
    std::vector<double> lateral;
    std::vector<size_t> first_pose;   // index of the column's first pose in the concatenation
    size_t n_poses = 0;
    for (const auto & c : sel) {
      const auto & p = c.poses.front();
      lateral.push_back(p.x * tx + p.y * ty);
      first_pose.push_back(n_poses);
      n_poses += c.poses.size();
    }
    auto groups = groupByLateralSpan(lateral, stand_max_span_);
    const double here = arm_in_world.t[0] * tx + arm_in_world.t[1] * ty;
    if (groups.size() > 1 &&
      std::abs(lateral[groups.back().front()] - here) < std::abs(lateral[groups.front().back()] - here))
    {
      std::reverse(groups.begin(), groups.end());
    }

    StanceStand start{arm_in_world.t[0], arm_in_world.t[1], base_in_world.yaw()};
    for (const auto & group : groups) {
      std::vector<CoverageSegmentIntent> part;
      for (const size_t i : group) {
        part.push_back(sel[i]);
      }
      auto req = makeStandRequest(stanceWall(part));
      req->has_start = true;
      req->start_x = start.x;
      req->start_y = start.y;
      req->start_yaw = start.yaw;
      auto future = client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::duration<double>(stand_service_timeout_s_)) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        why = "no reply from " + base_check_service_;
        return false;
      }
      const auto res = future.get();
      // the group's pose index -> the index in the concatenation of `sel`
      std::vector<size_t> to_global;
      for (const size_t i : group) {
        for (size_t k = 0; k < sel[i].poses.size(); ++k) {
          to_global.push_back(first_pose[i] + k);
        }
      }
      for (const auto & st : res->stands) {
        stands.push_back({st.x, st.y, st.yaw});
        std::vector<size_t> targets;
        for (const auto t : st.target_indices) {
          if (t < to_global.size()) {
            targets.push_back(to_global[t]);
          }
        }
        RCLCPP_INFO(
          this->get_logger(), "stand %zu: arm base (%.3f, %.3f) yaw %.3f reaches %zu pose(s)",
          stands.size() - 1, st.x, st.y, st.yaw, targets.size());
        targets_per_stand.push_back(targets);
        start = stands.back();
      }
      if (!res->success) {
        RCLCPP_WARN(
          this->get_logger(), "plan_stands: %zu target(s) unreachable, %zu not planned (%s)",
          res->unreachable.size(), res->not_planned.size(), res->message.c_str());
      }
    }
    if (stands.empty()) {
      why = "plan_stands returned no stand";
      return false;
    }
    if (stand_merge_tol_ > 0.0 && stands.size() > 1) {
      std::vector<core_types::Pose> all_poses;
      for (const auto & c : sel) {
        all_poses.insert(all_poses.end(), c.poses.begin(), c.poses.end());
      }
      std::vector<MergeableStand> ms;
      for (size_t i = 0; i < stands.size(); ++i) {
        ms.push_back({stands[i].x, stands[i].y, stands[i].yaw, targets_per_stand[i]});
      }
      const auto merger = [&](const std::vector<size_t> & idx) -> std::optional<MergeableStand> {
          std::vector<CoverageSegmentIntent> one(1);
          for (const size_t k : idx) {
            if (k >= all_poses.size()) {
              return std::nullopt;
            }
            one[0].poses.push_back(all_poses[k]);
          }
          auto req = makeStandRequest(stanceWall(one));
          auto future = client->async_send_request(req);
          if (rclcpp::spin_until_future_complete(
              shared_from_this(), future, std::chrono::duration<double>(stand_service_timeout_s_)) !=
            rclcpp::FutureReturnCode::SUCCESS)
          {
            return std::nullopt;
          }
          const auto res = future.get();
          if (!res->success || res->stands.size() != 1 || !res->unreachable.empty() || !res->not_planned.empty() ||
            res->stands[0].target_indices.size() != idx.size())
          {
            return std::nullopt;
          }
          return MergeableStand{res->stands[0].x, res->stands[0].y, res->stands[0].yaw, {}};
        };
      const size_t n = mergeNearDuplicateStands(ms, stand_merge_tol_, merger);
      if (n > 0) {
        stands.clear();
        targets_per_stand.clear();
        for (const auto & m : ms) {
          stands.push_back({m.x, m.y, m.yaw});
          targets_per_stand.push_back(m.targets);
        }
        RCLCPP_INFO(
          this->get_logger(), "merged %zu near-duplicate stand(s) (within %.2f m): %zu stand(s) left", n,
          stand_merge_tol_, stands.size());
      }
    }
    return true;
  }

  /// Drives the arm base to a stand with the navigate_to_stand action; false (with `message`) on failure.
  bool navigateToStand(const StanceStand & st, std::string & message)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      message = "navigate_to_stand is not available";
      return false;
    }
    NavigateToStand::Goal goal;
    goal.pose.header.frame_id = world_frame_;
    goal.pose.pose.position.x = st.x;
    goal.pose.pose.position.y = st.y;
    goal.pose.pose.orientation.z = std::sin(st.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(st.yaw / 2.0);

    struct Outcome
    {
      bool done{false};
      rclcpp_action::ResultCode code{rclcpp_action::ResultCode::UNKNOWN};
      NavigateToStand::Result::SharedPtr result;
    };
    auto outcome = std::make_shared<Outcome>();
    rclcpp_action::Client<NavigateToStand>::SendGoalOptions options;
    options.result_callback =
      [outcome](const rclcpp_action::ClientGoalHandle<NavigateToStand>::WrappedResult & w) {
        outcome->done = true;
        outcome->code = w.code;
        outcome->result = w.result;
      };
    auto sent = nav_client_->async_send_goal(goal, options);
    const auto t0 = std::chrono::steady_clock::now();
    const auto elapsed = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      };
    while (rclcpp::ok() && sent.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (elapsed() > 10.0) {
        message = "navigate_to_stand did not answer the goal";
        return false;
      }
    }
    if (!rclcpp::ok()) {
      message = "shutdown";
      return false;
    }
    nav_goal_ = sent.get();
    if (!nav_goal_) {
      message = "navigate_to_stand rejected the goal";
      return false;
    }
    bool cancel_sent = false;
    double cancel_at = 0.0;
    while (rclcpp::ok() && !outcome->done) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (!cancel_sent && (cancel_requested_ || elapsed() > nav_timeout_s_)) {
        message = cancel_requested_ ? "canceled" : "navigation timed out";
        cancelNavigation();
        cancel_sent = true;
        cancel_at = elapsed();
      }
      if (cancel_sent && elapsed() - cancel_at > 10.0) {
        break;   // the navigator did not confirm the cancel
      }
    }
    nav_goal_.reset();
    if (cancel_sent || !outcome->done) {
      return false;
    }
    if (outcome->code != rclcpp_action::ResultCode::SUCCEEDED || outcome->result->error_code != 0) {
      message = outcome->result && !outcome->result->error_msg.empty() ?
        outcome->result->error_msg : "navigation failed";
      return false;
    }
    last_arrival_pos_err_m_ = outcome->result->position_error_m;
    last_arrival_yaw_err_rad_ = outcome->result->yaw_error_rad;
    RCLCPP_INFO(
      this->get_logger(), "arrived at the stand (error %.1f mm, %.3f rad)",
      outcome->result->position_error_m * 1000.0, outcome->result->yaw_error_rad);
    return true;
  }

  /// Stand-check gate after an arrival: go, or drive to the stand again, or give the stand up (`abort` = true,
  /// the caller then marks it failed). A failed re-drive returns false with `message` and the normal retry applies.
  bool gateStand(const StanceStand & st, std::string & message, bool & abort)
  {
    struct Port final : StandNavigatorPort
    {
      Port(ManipulationManagerNode & n, const StanceStand & s) : node(n), stand(s) {}
      bool navigate(std::string & m) override {return node.navigateToStand(stand, m);}
      StandAchieved achieved() const override {return {node.last_arrival_pos_err_m_, node.last_arrival_yaw_err_rad_};}
      bool stopRequested() const override {return node.stop();}
      ManipulationManagerNode & node;
      const StanceStand & stand;
    } port(*this, st);
    const auto o = runStandGate(port, stand_check_, stand_check_reach_margin_);
    if (o.ok) {
      RCLCPP_INFO(this->get_logger(), "%s", o.message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "stand gate: %s (re-docks %d)", o.message.c_str(), o.redocks);
      message = o.message;
    }
    abort = o.abort;
    return o.ok;
  }

  /// Wall dock step after an arrival (opt-in): sends WallDock goals, judges the measured wall gap and yaw error with
  /// the stand-check logic (core/wall_dock_gate.hpp) and keeps the outcome for the verify result. False with `message`
  /// when the arm must not move; `abort` = true when the stand is given up.
  bool wallDockStand(size_t stand, double standoff_m, std::string & message, bool & abort)
  {
    struct Port final : WallDockPort
    {
      Port(ManipulationManagerNode & n, double s) : node(n), standoff_m(s) {}
      WallDockMeasured dock() override {return node.sendWallDock(standoff_m);}
      bool stopRequested() const override {return node.stop();}
      ManipulationManagerNode & node;
      double standoff_m;
    } port{*this, standoff_m};
    const auto o = runWallDock(port, wall_dock_cfg_);
    verify_wall_dock_[stand] = o;
    if (o.ok) {
      RCLCPP_INFO(
        this->get_logger(), "wall dock: standoff error %.1f mm, yaw error %.3f rad (%d goal(s))",
        o.standoff_error_m * 1000.0, o.yaw_error_rad, o.attempts);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "wall dock: %s (%d goal(s); standoff error %.1f mm, yaw error %.3f rad)",
        o.message.c_str(), o.attempts, o.standoff_error_m * 1000.0, o.yaw_error_rad);
      message = o.message;
    }
    abort = o.abort;
    return o.ok;
  }

  /// The standoff of the WallDock goal: the stand's planned one (arm base to the plan's wall) or the fixed parameter.
  double plannedWallDockStandoff(const StanceStand & stand, const StanceWall & wall) const
  {
    if (!wall_dock_planned_standoff_) {
      return wall_dock_standoff_m_;
    }
    return plannedStandoff(wall.wall_x, wall.wall_y, stand.x, stand.y, wall_dir_.d[0], wall_dir_.d[1]);
  }

  /// One WallDock goal, waited for; `docked` only when the action succeeded.
  WallDockMeasured sendWallDock(double standoff_m)
  {
    WallDockMeasured m;
    if (!wall_dock_client_->wait_for_action_server(std::chrono::seconds(5))) {
      m.message = "wall dock action is not available";
      return m;
    }
    WallDock::Goal goal;
    goal.standoff_m = standoff_m;
    goal.timeout_s = wall_dock_timeout_s_;
    auto outcome = std::make_shared<rclcpp_action::ClientGoalHandle<WallDock>::WrappedResult>();
    auto done = std::make_shared<bool>(false);
    rclcpp_action::Client<WallDock>::SendGoalOptions options;
    options.result_callback = [outcome, done](const rclcpp_action::ClientGoalHandle<WallDock>::WrappedResult & w) {
        *outcome = w;
        *done = true;
      };
    auto sent = wall_dock_client_->async_send_goal(goal, options);
    const auto t0 = std::chrono::steady_clock::now();
    const double limit = (wall_dock_timeout_s_ > 0.0 ? wall_dock_timeout_s_ : 60.0) + 15.0;
    const auto elapsed = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      };
    while (rclcpp::ok() && sent.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (elapsed() > 10.0) {
        m.message = "wall dock did not answer the goal";
        return m;
      }
    }
    auto handle = rclcpp::ok() ? sent.get() : nullptr;
    if (!handle) {
      m.message = "wall dock goal rejected";
      return m;
    }
    bool cancel_sent = false;
    while (rclcpp::ok() && !*done) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (!cancel_sent && (cancel_requested_ || elapsed() > limit)) {
        m.message = cancel_requested_ ? "canceled" : "wall dock timed out";
        try {
          wall_dock_client_->async_cancel_goal(handle);
        } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
        }
        cancel_sent = true;
      }
      if (cancel_sent && elapsed() > limit + 10.0) {
        break;
      }
    }
    if (cancel_sent || !*done) {
      if (m.message.empty()) {
        m.message = "wall dock not finished";
      }
      return m;
    }
    const auto & r = outcome->result;
    if (outcome->code != rclcpp_action::ResultCode::SUCCEEDED || !r || r->error_code != 0) {
      m.message = r && !r->error_msg.empty() ? r->error_msg : "wall dock failed";
      return m;
    }
    m.docked = true;
    m.standoff_error_m = r->standoff_error_m;
    m.yaw_error_rad = r->yaw_error_rad;
    return m;
  }

  void cancelNavigation()
  {
    if (nav_goal_) {
      try {
        nav_client_->async_cancel_goal(nav_goal_);
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
        // already finished: nothing to cancel
      }
    }
  }

  /// True when a failed motion can be retried with the wall out of the scene (both commands are set).
  bool canRecoverWithoutWall() const
  {
    return !wall_clear_command_.empty() && !arrival_command_.empty();
  }

  /// Runs `motion` with the wall taken out of the planning scene, then puts the wall back. For a tool that
  /// ended up inside the wall box: its start state is in collision and every planner refuses it. The move out
  /// is the same straight one the retract or stow plans anyway; only the wall is not an obstacle for it.
  template<typename Motion>
  bool withWallCleared(const char * what, Motion motion)
  {
    RCLCPP_WARN(this->get_logger(), "%s failed: retrying with the wall out of the planning scene", what);
    if (std::system(wall_clear_command_.c_str()) != 0) {
      RCLCPP_WARN(this->get_logger(), "wall_clear_command failed; the %s recovery is skipped", what);
      return false;
    }
    waitFor(0.5, false);
    const bool ok = motion();
    if (std::system(arrival_command_.c_str()) != 0) {
      RCLCPP_WARN(this->get_logger(), "stand_arrival_command exited with an error after the %s recovery", what);
    }
    waitFor(1.0, false);
    return ok;
  }

  /// One pass of the stow attempts (a free-space move to the travel pose).
  bool stowAttempts()
  {
    MotionSegmentSpec seg;
    seg.label = "stow";
    seg.mode_hint = "fsp";
    seg.target_joint_state = travel_state_;
    for (int attempt = 0; attempt < dock_attempts_ && !stop(); ++attempt) {
      if (manager_.executeMotionSegments(frame_id_, {seg}, {}, planning_group_).success) {
        return true;
      }
      RCLCPP_WARN(this->get_logger(), "stow attempt %d failed", attempt + 1);
    }
    return false;
  }

  /// The arm to its travel pose (a free-space move), so the base can drive without the tool near the wall.
  bool stowArm()
  {
    if (stowAttempts()) {
      return true;
    }
    if (stop()) {
      return false;
    }
    // A serpentine stand ends with the tool backed off at the top of the wall, where the planning model has the arm
    // touching the lift (the start state is refused). Come back down the standoff plane in one straight move, to
    // the height a column starts from in the bottom-up order, and stow from there.
    const auto lowering = decideStowLower(
      serpentine_columns_, chain_valid_, chain_last_standoff_, coverage_z_min_);
    if (lowering.lower) {
      MotionSegmentSpec down;
      down.label = "stow_lower";
      down.mode_hint = "cartesian";
      down.waypoint_indices = {0};
      down.min_path_fraction = 0.99;
      down.target_eef_speed = coverage_eef_speed_;
      if (manager_.executeMotionSegments(frame_id_, {down}, {lowering.target}, planning_group_).success) {
        chain_valid_ = false;
        if (stowAttempts()) {
          return true;
        }
      }
    }
    if (recoverArm("stow") && stowAttempts()) {
      return true;
    }
    if (stop() || !canRecoverWithoutWall()) {
      return false;
    }
    return withWallCleared("stow", [this]() {return stowAttempts();});
  }

  /// The plan_stands request for the targets of `wall`, with the lift range and arm mount of the base check.
  std::shared_ptr<reachability_msgs::srv::PlanStands::Request> makeStandRequest(const StanceWall & wall)
  {
    auto req = std::make_shared<reachability_msgs::srv::PlanStands::Request>();
    for (const auto & p : wall.targets) {
      geometry_msgs::msg::Pose m;
      m.position.x = p.x;
      m.position.y = p.y;
      m.position.z = p.z;
      m.orientation.x = p.qx;
      m.orientation.y = p.qy;
      m.orientation.z = p.qz;
      m.orientation.w = p.qw;
      req->targets_world.push_back(m);
    }
    // The wall lies along wall_direction from the base (the same direction the dock pose uses); the
    // normal points into the free side, back towards the robot.
    req->wall_point.x = wall.wall_x;
    req->wall_point.y = wall.wall_y;
    req->wall_point.z = wall.wall_z;
    req->wall_normal.x = -wall_dir_.d[0];
    req->wall_normal.y = -wall_dir_.d[1];
    req->wall_normal.z = -wall_dir_.d[2];
    if (stand_min_clearance_ > 0.0) {
      req->min_clearance = stand_min_clearance_;
    }
    req->lift_min = base_check_lift_min_;
    req->lift_max = base_check_lift_max_;
    req->arm_mount_offset_z = base_check_arm_mount_z_;
    return req;
  }

  /// Loads the RM4D map named by column_reachability_map_file and builds the column reachability port. Off (true)
  /// without a file; false when the file does not load, or world_frame is not set (no way to place the arm base).
  bool initColumnReachability()
  {
    if (column_reach_map_file_.empty()) {
      return true;
    }
    if (world_frame_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(), "column_reachability_map_file needs world_frame (the arm base position comes from TF)");
      return false;
    }
    std::shared_ptr<reachability::Map4D> map;
    try {
      map = std::make_shared<reachability::Map4D>(reachability::loadMap(column_reach_map_file_));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "column_reachability_map_file %s: %s", column_reach_map_file_.c_str(), e.what());
      return false;
    }
    ReachabilityColumnConfig cfg;
    cfg.sequence.wall_normal = reachability::Vec3(-wall_dir_.d[0], -wall_dir_.d[1], -wall_dir_.d[2]);
    cfg.sequence.standoff_m = dock_standoff_;
    cfg.sequence.approach_extra_m = std::fabs(dock_approach_extra_m_);
    cfg.sequence.press_depth_m =
      column_reach_press_depth_ >= 0.0 ? column_reach_press_depth_ : (contact_ ? contact_y_overshoot_ : 0.0);
    cfg.sequence.dock_at_first_stroke = dock_at_stroke_start_;
    cfg.margin.min_score = column_reach_min_score_;
    cfg.margin.count_ref = std::max(1.0, column_reach_count_ref_);
    cfg.lift_heights = liftHeights(base_check_lift_min_, base_check_lift_max_, column_reach_lift_step_);
    cfg.arm_mount_offset_z = base_check_arm_mount_z_;
    column_reach_ = std::make_unique<ReachabilityColumnPort>(
      map, cfg, [this]() -> std::optional<Eigen::Vector2d> {
        FrameTransform arm_in_world;
        if (!lookup(world_frame_, arm_base_frame_, arm_in_world)) {
          return std::nullopt;
        }
        return Eigen::Vector2d(arm_in_world.t[0], arm_in_world.t[1]);
      });
    manager_.setColumnReachability(column_reach_.get());
    RCLCPP_INFO(
      this->get_logger(), "column reachability check on: map %s, %zu lift height(s)",
      column_reach_map_file_.c_str(), cfg.lift_heights.size());
    return true;
  }

  /// Logs, per contact column, whether the arm base where it stands does the column's whole pose sequence
  /// (approach, contact, strokes, retract). Advisory: nothing is skipped or refused.
  void logColumnReach(const std::vector<CoverageSegmentIntent> & intents)
  {
    if (!column_reach_) {
      return;
    }
    std::vector<ColumnReachInput> inputs;
    for (const auto & it : intents) {
      if (it.requires_contact && !it.poses.empty()) {
        inputs.push_back({it.label, it.poses});
      }
    }
    const auto verdicts = column_reach_->check(inputs);
    if (verdicts.size() != inputs.size()) {
      RCLCPP_WARN(this->get_logger(), "column reachability: not checked (no arm base pose from TF)");
      return;
    }
    size_t whole = 0;
    for (size_t i = 0; i < verdicts.size(); ++i) {
      const auto & v = verdicts[i];
      if (v.status == ColumnReachStatus::kWhole) {
        ++whole;
        continue;
      }
      size_t usable = 0;
      for (size_t k = 0; k < inputs[i].strokes.size(); ++k) {
        usable += v.strokeUsable(k) ? 1 : 0;
      }
      RCLCPP_WARN(
        this->get_logger(), "column reachability: %s is %s from here (%zu/%zu strokes usable), limited by %s: %s",
        inputs[i].label.c_str(), v.status == ColumnReachStatus::kSplit ? "split" : "unreachable", usable,
        inputs[i].strokes.size(), v.limiting_pose.c_str(), v.limiting_reason.c_str());
    }
    RCLCPP_INFO(
      this->get_logger(), "column reachability: %zu of %zu column(s) whole (RM4D estimate)", whole,
      verdicts.size());
  }

  /// Asks the reachability planner whether the base, where it stands, reaches the whole plan.
  /// Returns false only in "enforce" mode when the stance fails; an unreachable service counts as
  /// a failed check (the answer is unknown), so enforce never runs blind.
  bool baseStanceAllowsRun(const std::vector<CoverageSegmentIntent> & intents)
  {
    if (base_check_mode_ == "off") {
      return true;
    }
    if (base_check_mode_ != "warn" && base_check_mode_ != "enforce") {
      RCLCPP_ERROR(
        this->get_logger(), "base_check must be off, warn or enforce (got '%s')",
        base_check_mode_.c_str());
      return false;
    }
    const bool enforce = base_check_mode_ == "enforce";
    auto fail = [&](const std::string & why) {
        RCLCPP_WARN(this->get_logger(), "base stance check: %s", why.c_str());
        if (enforce) {
          RCLCPP_ERROR(this->get_logger(), "base_check=enforce: not running the plan");
        }
        return !enforce;
      };

    const auto wall = stanceWall(intents);
    if (wall.targets.empty()) {
      return true;
    }
    auto client = this->create_client<reachability_msgs::srv::PlanStands>(base_check_service_);
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      return fail("service " + base_check_service_ + " is not available");
    }
    auto req = makeStandRequest(wall);
    // In a world frame the stands are world poses: start the planner where the arm base is, and
    // compare the stand with that pose (without a start the planner centres the stand on the wall
    // section, which is not necessarily the stand nearest to the base).
    StanceStand here;
    if (!world_frame_.empty()) {
      FrameTransform arm_in_world, base_in_world;
      if (!lookup(world_frame_, arm_base_frame_, arm_in_world) ||
        !lookup(world_frame_, frame_id_, base_in_world))
      {
        return fail("no TF " + world_frame_ + " -> " + arm_base_frame_ + " / " + frame_id_);
      }
      here = {arm_in_world.t[0], arm_in_world.t[1], base_in_world.yaw()};
      req->has_start = true;
      req->start_x = here.x;
      req->start_y = here.y;
      req->start_yaw = here.yaw;
    }
    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(
        shared_from_this(), future, std::chrono::seconds(30)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return fail("no reply from " + base_check_service_);
    }
    const auto res = future.get();
    std::vector<StanceStand> stands;
    for (const auto & st : res->stands) {
      stands.push_back({st.x, st.y, st.yaw});
    }
    StanceTolerance tol;
    tol.xy_m = base_check_tol_xy_;
    tol.yaw_rad = base_check_tol_yaw_;
    const auto verdict = evaluateStance(
      stands, res->unreachable.size(), res->not_planned.size(), tol, here);
    if (verdict.ok) {
      RCLCPP_INFO(this->get_logger(), "base stance check: %s", verdict.message.c_str());
      return true;
    }
    return fail(verdict.message);
  }

  /// Pose of `child` in `parent` from TF (latest), false when it does not resolve within 2 s.
  bool lookup(const std::string & parent, const std::string & child, FrameTransform & out)
  {
    if (!tf_buffer_) {
      // init() creates the buffer in every run; this only guards a lookup before init() or after a failed one:
      // report "no TF" instead of dereferencing a null buffer
      RCLCPP_DEBUG_ONCE(this->get_logger(), "TF lookup %s <- %s skipped: no TF buffer", parent.c_str(), child.c_str());
      return false;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(
        parent, child, tf2::TimePointZero, tf2::durationFromSec(2.0));
      const auto & t = tf.transform.translation;
      const auto & q = tf.transform.rotation;
      out.t = {t.x, t.y, t.z};
      out.q = {q.x, q.y, q.z, q.w};
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "TF %s <- %s: %s", parent.c_str(), child.c_str(), e.what());
      return false;
    }
  }

  /// Builds the planners the YAML selection names, through pluginlib. Plugins that need a ROS
  /// node (cuRobo) get a dedicated one, created and spun only if one of them asks for it.
  bool buildPlanners()
  {
    std::string config_path = planning_config_;
    if (config_path.empty()) {
      config_path = ament_index_cpp::get_package_share_directory("manipulation_manager") +
        "/config/planning.yaml";
    }
    motion_planner::PlanningSelection selection;
    std::string error;
    if (!motion_planner::loadPlanningSelection(config_path, selection, error)) {
      RCLCPP_ERROR(this->get_logger(), "planning_config: %s", error.c_str());
      return false;
    }

    // The profile's joint_costs reach the free-space planners as the plugin param `joint_costs`, unless the planning
    // config already sets one for that plugin.
    if (!profile_joint_costs_.empty()) {
      std::ostringstream costs;
      for (size_t i = 0; i < profile_joint_costs_.size(); ++i) {
        costs << (i ? "," : "") << profile_joint_costs_[i];
      }
      for (const auto * name : {&selection.free_space, &selection.free_space_fallback}) {
        if (!name->empty()) {
          selection.plugin_params[*name].emplace("joint_costs", costs.str());
        }
      }
    }

    plugin_registry_ = motion_planner::makePluginlibRegistry();
    plugin_context_.logger = std::make_shared<ros_adapters::RclcppLogger>(this->get_logger());
    plugin_context_.setFactory<rclcpp::Node>(
      "node", [this]() {
        auto node = std::make_shared<rclcpp::Node>("planning_plugins");
        plugin_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        plugin_executor_->add_node(node);
        plugin_spin_thread_ = std::thread([this]() {plugin_executor_->spin();});
        return node;
      });

    motion_planner::PipelineError pipeline_error;
    if (!pipeline_.build(selection, *plugin_registry_, plugin_context_, pipeline_error)) {
      RCLCPP_ERROR(
        this->get_logger(), "planning backend '%s' (%s) failed: [%s] %s (config %s)",
        pipeline_error.plugin.c_str(), pipeline_error.role.c_str(), pipeline_error.code.c_str(),
        pipeline_error.message.c_str(), config_path.c_str());
      return false;
    }
    manager_.setMotionPlanner(pipeline_.makeMotionPlanner());
    // seed_start_from_live_state: the first segment of each call starts from /joint_states instead of the planner's own
    // idea of the current state. Needed with the cuRobo backend (it needs an explicit start); off by default.
    if (seed_start_from_live_state_) {
      manager_.setStartStateProvider(
        [this]() {return readCurrentJointState(shared_from_this(), joint_name_candidates_);});
    }
    RCLCPP_INFO(
      this->get_logger(), "planning backends from %s: cartesian=%s free_space=%s ik=%s collision=%s",
      config_path.c_str(), selection.cartesian.c_str(), selection.free_space.c_str(),
      selection.ik.c_str(), selection.collision.c_str());
    return true;
  }

  /// The joint configuration for `pose` (frame_id_) from an IK seeded with the previous docked configuration, the
  /// last good one otherwise. Empty when the IK has no solution or is not available.
  std::vector<double> seededDockIk(const core_types::Pose & pose)
  {
    const auto ik = pipeline_.ik();
    const auto & seed = !dock_seed_.empty() ? dock_seed_ :
      (dock_branch_seed_.empty() ? last_good_state_ : dock_branch_seed_);
    // with the continuity choice the live state is a seed of its own, so an empty remembered seed (the first dock of a
    // run) must not stop it
    const bool exclude = dock_ik_exclude_on_;   // a column retry: a solution other than the earlier ones
    if (!ik || (seed.empty() && !dock_ik_continuity_ && !exclude)) {
      return {};
    }
    motion_planner::IkRequest req;
    req.frame_id = frame_id_;
    req.planning_group = planning_group_;
    req.target_pose = {pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw};
    if (dock_ik_continuity_ || exclude) {
      const auto live = readCurrentJointState(shared_from_this(), joint_name_candidates_);
      const std::vector<double> & ref = dockBranchReference(dock_branch_seed_, dock_seed_, live);
      // reach preference: where the tool goal lies relative to the arm base (TF frame_id <- arm_base_frame, the live lift
      // taken out of the arm base height)
      DockReachContext reach_ctx;
      const DockReachContext * reach_ptr = nullptr;
      if (dockReachPreferenceOn(dock_ik_sel_) && dock_ik_sel_.lift_joint < live.size()) {
        FrameTransform arm_in_frame;
        if (lookup(frame_id_, arm_base_frame_, arm_in_frame)) {
          const double dx = pose.x - arm_in_frame.t[0];
          const double dy = pose.y - arm_in_frame.t[1];
          reach_ctx.horizontal_m = std::hypot(dx, dy);
          reach_ctx.z_rel_m = pose.z - (arm_in_frame.t[2] - live[dock_ik_sel_.lift_joint]);
          reach_ptr = &reach_ctx;
        } else {
          RCLCPP_WARN(this->get_logger(), "DOCKGOAL reach preference skipped: no TF %s <- %s", frame_id_.c_str(), arm_base_frame_.c_str());
        }
      }
      if (jog_params_.lower.size() == live.size() && jog_params_.upper.size() == live.size()) {
        dock_ik_sel_.joint_lower = jog_params_.lower;
        dock_ik_sel_.joint_upper = jog_params_.upper;
      }
      const auto choice = selectDockIk(
        [&](const std::vector<double> & sd) -> std::optional<std::vector<double>> {
          auto r = req;
          r.joint_seed = sd;
          const auto res = ik->ik(r);
          if (!res.success) {
            return std::nullopt;
          }
          return res.joint_solution;
        },
        {dock_branch_seed_, live, dock_seed_, last_good_state_}, ref, dock_ik_sel_, reach_ptr,
        exclude ? &dock_ik_exclude_ : nullptr);
      if (choice.ok) {
        column_dock_q_[static_cast<size_t>(std::max(0, column_))].push_back(choice.q);
        RCLCPP_INFO(
          this->get_logger(),
          "DOCKGOAL continuity: seed %zu of %zu candidates, max dq %.3f rad (dropped: branch %zu, wrist %zu, jump %zu)",
          choice.seed_index, choice.candidates, choice.cost, choice.rejected_branch, choice.rejected_wrist,
          choice.rejected_cost);
        if (reach_ptr) {
          RCLCPP_INFO(
            this->get_logger(), "DOCKGOAL reach fraction %.2f (target %.2f)%s", choice.reach_fraction,
            dock_ik_sel_.reach_target_fraction, choice.reach_over_target ? ", over the target" : "");
        }
        // the approach starts at the live state: of the 2*pi-equivalent representatives take the one nearest to IT, not
        // to the remembered branch seed (which can be a turn away after a stow or a failed column)
        return live.size() == choice.q.size() ? rebaseDockIkChecked(choice.q, live, dock_ik_sel_) : choice.q;
      }
      RCLCPP_WARN(
        this->get_logger(),
        "DOCKGOAL continuity: no acceptable solution (%zu candidates; branch %zu, wrist %zu, jump %zu, already used %zu), seeded IK as before",
        choice.candidates, choice.rejected_branch, choice.rejected_wrist, choice.rejected_cost, choice.rejected_excluded);
      if (exclude) {
        return {};   // a retry must not dock with the solution that failed: the planner samples its own
      }
    }
    req.joint_seed = seed;
    const auto res = ik->ik(req);
    if (!res.success || res.joint_solution.size() != seed.size()) {
      RCLCPP_WARN(this->get_logger(), "seeded dock IK: no solution (%s), the planner samples its own", res.message.c_str());
      return {};
    }
    std::string txt;
    char buf[32];
    for (const double v : res.joint_solution) {
      std::snprintf(buf, sizeof(buf), "%s%.4f", txt.empty() ? "" : ", ", v);
      txt += buf;
    }
    RCLCPP_INFO(this->get_logger(), "DOCKGOAL seeded IK q=[%s]", txt.c_str());
    // same pose, representative nearest the live state (the IK may return a joint 2*pi away)
    {
      const auto live_now = readCurrentJointState(shared_from_this(), joint_name_candidates_);
      const auto q = rebaseDockIkChecked(
        res.joint_solution, live_now.size() == res.joint_solution.size() ? live_now : seed, dock_ik_sel_);
      column_dock_q_[static_cast<size_t>(std::max(0, column_))].push_back(q);
      return q;
    }
  }

  /// recovery_keeps_branch: the joint goal for `pose` chosen on the arm's live branch (selectRecoveryGoal, IK seeded
  /// from the live state and the last docked one). Empty when there is no IK or no acceptable solution: the caller
  /// keeps its previous goal.
  std::vector<double> recoveryGoalIk(const core_types::Pose & pose)
  {
    const auto ik = pipeline_.ik();
    const auto live = readCurrentJointState(shared_from_this(), joint_name_candidates_);
    if (!ik || live.empty()) {
      return {};
    }
    motion_planner::IkRequest req;
    req.frame_id = frame_id_;
    req.planning_group = planning_group_;
    req.target_pose = {pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw};
    const auto choice = selectRecoveryGoal(
      [&](const std::vector<double> & sd) -> std::optional<std::vector<double>> {
        auto r = req;
        r.joint_seed = sd;
        const auto res = ik->ik(r);
        if (!res.success) {
          return std::nullopt;
        }
        return res.joint_solution;
      },
      live, dock_branch_seed_, dock_ik_sel_);
    if (!choice.ok) {
      RCLCPP_WARN(
        this->get_logger(),
        "recovery goal: no solution on the live branch (%zu candidates; branch %zu, wrist %zu, jump %zu), unchanged",
        choice.candidates, choice.rejected_branch, choice.rejected_wrist, choice.rejected_cost);
      return {};
    }
    RCLCPP_INFO(
      this->get_logger(), "recovery goal on the live branch: seed %zu of %zu, max dq %.3f rad", choice.seed_index,
      choice.candidates, choice.cost);
    return choice.q;
  }

  /// True when the tool frame is within `tol` of `pose` (both in frame_id_); `err` is the distance. An unknown tool
  /// frame counts as near: the check must not block a run that has no tcp_link.
  bool tcpNear(const core_types::Pose & pose, double tol, double & err)
  {
    FrameTransform tcp;
    if (!lookup(frame_id_, "tcp_link", tcp)) {
      err = 0.0;
      return true;
    }
    err = std::sqrt(
      std::pow(tcp.t[0] - pose.x, 2) + std::pow(tcp.t[1] - pose.y, 2) + std::pow(tcp.t[2] - pose.z, 2));
    return err <= tol;
  }

  /// Builds the column IK check (column_ik_check): the reachability ColumnIkChain over the planner's IK port in the base
  /// frame. Off (nothing built) when the parameter is false or the pipeline has no IK.
  void initColumnIkCheck()
  {
    if (!column_ik_check_) {
      return;
    }
    if (!pipeline_.ik()) {
      RCLCPP_WARN(this->get_logger(), "column_ik_check is on but the planning pipeline has no IK backend: the check is off");
      column_ik_check_ = false;
      return;
    }
    const auto to_idx = [](const std::vector<int64_t> & v) {
        std::vector<size_t> out;
        for (const auto i : v) {
          if (i >= 0) {out.push_back(static_cast<size_t>(i));}
        }
        return out;
      };
    ReachabilityColumnIkConfig cfg;
    cfg.sequence.standoff_m = dock_standoff_;
    cfg.sequence.approach_extra_m = std::fabs(dock_approach_extra_m_);
    cfg.sequence.press_depth_m = contact_ ? contact_y_overshoot_ : 0.0;
    cfg.sequence.dock_at_first_stroke = dock_at_stroke_start_;
    cfg.chain.branch_joints = to_idx(column_ik_branch_joints_);
    cfg.chain.ratio_limit_revolute = column_ik_ratio_limit_;
    cfg.chain.ratio_limit_continuous = column_ik_ratio_limit_;
    cfg.prismatic_joints = to_idx(column_ik_prismatic_joints_);
    cfg.continuous_joints = to_idx(column_ik_continuous_joints_);
    column_ik_port_ = std::make_unique<ReachabilityColumnIkPort>(
      cfg, [this](const core_types::Pose & pose, const std::vector<double> & seed) -> std::optional<std::vector<double>> {
        const auto ik = pipeline_.ik();
        if (!ik) {
          return std::nullopt;
        }
        motion_planner::IkRequest req;
        req.frame_id = frame_id_;
        req.planning_group = planning_group_;
        req.target_pose = {pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw};
        req.joint_seed = seed;
        const auto res = ik->ik(req);
        if (!res.success || res.joint_solution.size() != seed.size()) {
          return std::nullopt;
        }
        return res.joint_solution;
      });
    ColumnIkGateConfig gc;
    gc.lift_joint = cfg.prismatic_joints.empty() ? 0 : cfg.prismatic_joints.front();
    gc.lift_candidates = liftHeights(base_check_lift_min_, base_check_lift_max_, column_ik_lift_step_);
    gc.reject_on_refused = column_ik_reject_on_refused_;
    column_ik_gate_ = ColumnIkGate(gc);
    RCLCPP_INFO(
      this->get_logger(), "column IK check on: %zu lift candidate(s), branch joints %zu, reject on refused %s",
      gc.lift_candidates.size(), cfg.chain.branch_joints.size(), gc.reject_on_refused ? "yes" : "no");
  }

  /// True when the column may run: the IK chain along it is continuous from the seed or from another lift height, or the
  /// check could not decide. False (the column is skipped) when every attempt breaks. A rescuing lift seeds the dock IK.
  bool columnIkAllows(const CoverageSegmentIntent & intent, const WallDirection & wall_dir)
  {
    if (!column_ik_check_ || !column_ik_port_) {
      return true;
    }
    std::vector<double> seed = !dock_seed_.empty() ? dock_seed_ :
      (dock_branch_seed_.empty() ? last_good_state_ : dock_branch_seed_);
    if (seed.empty()) {
      seed = readCurrentJointState(shared_from_this(), joint_name_candidates_);
    }
    const auto d = column_ik_gate_.decide(
      column_ik_port_.get(), intent.poses, {-wall_dir.d[0], -wall_dir.d[1], -wall_dir.d[2]}, seed);
    RCLCPP_INFO(
      this->get_logger(),
      "%s: column IK check %s (%zu attempt(s)): %s; splits %zu, branch changes %zu, max ratio %.2f, lift travel %.3f m",
      intent.label.c_str(), toString(d.action), d.attempts, d.reason.c_str(), d.verdict.split_count,
      d.verdict.branch_changes, d.verdict.max_continuity_ratio, d.verdict.lift_travel_m);
    if (d.action == ColumnIkAction::kGoWithSeed) {
      dock_branch_seed_ = d.seed;
    }
    if (d.action == ColumnIkAction::kReject) {
      RCLCPP_WARN(
        this->get_logger(), "%s: skipped before docking, no continuous IK chain (%s)", intent.label.c_str(),
        d.reason.c_str());
      column_reason_ = SkipReason::kPlanFailed;
      return false;
    }
    return true;
  }

  /// Opt-in verify trace (geometric coverage, path quality audit): a joint-state cache, a sampling timer and, for the audit,
  /// the chain model. False when a configured chain file does not load.
  bool initVerifyTrace()
  {
    if (!verify_trace_) {
      return true;
    }
    if (verify_path_quality_) {
      if (verify_path_chain_file_.empty()) {
        RCLCPP_WARN(
          this->get_logger(), "verify_path_quality is on without verify_path_chain_file: the audit will say it was not run");
      } else {
        try {
          path_quality_ = std::make_unique<ReachabilityPathQualityPort>(
            std::make_shared<reachability::ChainModel>(reachability::loadChainYaml(verify_path_chain_file_)),
            verify_path_cfg_, static_cast<size_t>(std::max(0, verify_path_skip_joints_)));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(), "verify_path_chain_file %s: %s", verify_path_chain_file_.c_str(), e.what());
          return false;
        }
      }
      joint_cache_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joint_cache_mutex_);
          for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            joint_cache_[msg->name[i]] = msg->position[i];
          }
        });
    }
    const double hz = std::max(1.0, verify_trace_hz_);
    trace_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz)),
      [this]() {sampleTrace();});
    RCLCPP_INFO(
      this->get_logger(), "verify trace on: geometric coverage %s, path quality %s, %.0f Hz (a spin of the node's wait loops)",
      verify_geometric_ ? "yes" : "no", verify_path_quality_ ? "yes" : "no", hz);
    return true;
  }

  /// One trace sample while a stroke runs: the tool position (TF, no waiting) and the planning group's joint values.
  void sampleTrace()
  {
    if (!trace_.stroking()) {
      return;
    }
    if (verify_geometric_ && tf_buffer_) {
      const std::string wf = world_frame_.empty() ? frame_id_ : world_frame_;
      try {
        if (tf_buffer_->canTransform(wf, "tcp_link", tf2::TimePointZero)) {
          const auto tf = tf_buffer_->lookupTransform(wf, "tcp_link", tf2::TimePointZero);
          trace_.addTool({tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z});
        }
      } catch (const std::exception &) {
        // no transform yet: skip this sample
      }
    }
    if (verify_path_quality_) {
      std::vector<double> q;
      {
        std::lock_guard<std::mutex> lock(joint_cache_mutex_);
        if (trace_joint_names_.empty()) {
          std::vector<std::string> available;
          for (const auto & kv : joint_cache_) {
            available.push_back(kv.first);
          }
          const std::vector<std::vector<std::string>> cand = group_joint_names_.empty() ?
            kJointNameCandidates : std::vector<std::vector<std::string>>{group_joint_names_};
          trace_joint_names_ = selectJointNames(cand, available);
        }
        for (const auto & name : trace_joint_names_) {
          q.push_back(joint_cache_.at(name));
        }
      }
      trace_.addJoints(q);
    }
  }

  /// Adds the geometric coverage and the path quality audit to the verify input (no-ops when not enabled).
  void addTraceToVerify(MissionVerifyInput & in)
  {
    if (verify_geometric_) {
      GeometricCoverageConfig cfg = verify_geo_cfg_;
      cfg.contact_force_n = contact_ ? verify_contact_force_n_ : -1.0;
      cfg.groups = verify_stand_columns_;
      cfg.wall_normal = {wall_dir_.d[0], wall_dir_.d[1], wall_dir_.d[2]};
      const auto plan = trace_.plan();
      if (!plan.empty() && !plan.front().points.empty()) {
        cfg.plane_origin = plan.front().points.front();
      }
      in.geometric = computeGeometricCoverage(plan, trace_.samples(), cfg);
      if (trace_.dropped() > 0) {
        in.geometric.note += (in.geometric.note.empty() ? "" : "; ") + std::to_string(trace_.dropped()) +
          " trace sample(s) dropped at the cap";
      }
      in.min_geometric_percent = verify_min_geometric_percent_;
      in.swath_m = cfg.swath_m;
    }
    if (verify_path_quality_) {
      in.path_quality_requested = true;
      if (path_quality_) {
        in.path_quality = path_quality_->audit(trace_.jointStrokes());
      } else {
        in.path_quality.note = "no verify_path_chain_file";
      }
    }
  }


  /// Position error (m) and orientation error (rad, the angle of the relative rotation) of the tool frame against `pose`
  /// (both in frame_id_). False when the tool frame is unknown: callers must not block a run that has no tcp_link.
  bool tcpPoseError(const core_types::Pose & pose, double & pos_err, double & ang_err)
  {
    FrameTransform tcp;
    if (!lookup(frame_id_, "tcp_link", tcp)) {
      pos_err = 0.0;
      ang_err = 0.0;
      return false;
    }
    pos_err = std::sqrt(
      std::pow(tcp.t[0] - pose.x, 2) + std::pow(tcp.t[1] - pose.y, 2) + std::pow(tcp.t[2] - pose.z, 2));
    // lookup() stores the quaternion as (x, y, z, w)
    const double dot = std::fabs(
      tcp.q[0] * pose.qx + tcp.q[1] * pose.qy + tcp.q[2] * pose.qz + tcp.q[3] * pose.qw);
    const double n1 = std::sqrt(tcp.q[0] * tcp.q[0] + tcp.q[1] * tcp.q[1] + tcp.q[2] * tcp.q[2] + tcp.q[3] * tcp.q[3]);
    const double n2 = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
    const double c = (n1 > 0.0 && n2 > 0.0) ? std::min(1.0, dot / (n1 * n2)) : 1.0;
    ang_err = 2.0 * std::acos(c);
    return true;
  }

  /// One diagnostic line: the planning group's live joint values (lift first) with a tag, for comparing runs.
  void logArmState(const std::string & tag)
  {
    const auto q = readCurrentJointState(shared_from_this(), joint_name_candidates_);
    std::string txt;
    char buf[32];
    for (const double v : q) {
      std::snprintf(buf, sizeof(buf), "%s%.4f", txt.empty() ? "" : ", ", v);
      txt += buf;
    }
    FrameTransform tcp;
    const std::string wf = world_frame_.empty() ? frame_id_ : world_frame_;
    const bool have_tcp = lookup(wf, "tcp_link", tcp);
    RCLCPP_INFO(
      this->get_logger(), "ARMSTATE %s stand %zu: q=[%s] tcp_in_%s=(%.3f, %.3f, %.3f)", tag.c_str(), stand_index_,
      txt.c_str(), wf.c_str(), have_tcp ? tcp.t[0] : 0.0, have_tcp ? tcp.t[1] : 0.0, have_tcp ? tcp.t[2] : 0.0);
  }

  /// A trivial joint-space "move to where the arm already is" segment -- its only purpose is
  /// getting a genuinely live joint state into ManipulationManager's own continuity tracking
  /// before the first real segment, see this file's own readCurrentJointState() comment for why.
  void primeContinuityFromLiveState()
  {
    const auto live_state = readCurrentJointState(shared_from_this(), joint_name_candidates_);
    if (live_state.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "primeContinuityFromLiveState: /joint_states does not carry the planning group's joints "
        "(group_joint_names) -- no priming, the first segment plans from the planner's own current state");
      return;
    }
    MotionSegmentSpec prime_seg;
    prime_seg.label = "prime_continuity";
    prime_seg.mode_hint = "fsp";
    prime_seg.target_joint_state = live_state;
    const auto result = manager_.executeMotionSegments(frame_id_, {prime_seg}, {}, planning_group_);
    if (!result.success) {
      RCLCPP_WARN(
        this->get_logger(),
        "primeContinuityFromLiveState: priming segment did not report success (status=%s) -- "
        "continuing anyway, first real segment may hit the same stale-start-state issue this "
        "was meant to avoid",
        result.status.c_str());
    }
  }

  /// `idx` (indices into `sel`) in the order set by column_order. "input" keeps it.
  std::vector<size_t> orderColumns(const std::vector<size_t> & idx, const std::vector<CoverageSegmentIntent> & sel)
  {
    if (column_order_ == "input" || idx.size() < 3) {
      return idx;
    }
    std::vector<ColumnEnds> ends;
    for (const size_t c : idx) {
      const auto & a = sel[c].poses.front();
      const auto & b = sel[c].poses.back();
      ends.push_back({{a.x, a.y, a.z}, {b.x, b.y, b.z}});
    }
    // the first column of the plan stays first (the arm is docked for it); the rest follow by travel time
    const auto local = orderColumnsByTime(ends, ends.front().start, column_order_ == "nearest_2opt");
    std::vector<size_t> out;
    for (const size_t l : local) {
      out.push_back(idx[l]);
    }
    RCLCPP_INFO(this->get_logger(), "column order (%s): %zu column(s) ordered", column_order_.c_str(), out.size());
    return out;
  }

  /// runColumn, and when the column failed (not done, not partial) once more per column_retries after the arm
  /// was freed if it was stuck. A column that still fails is skipped: the loop goes on with the next one.
  ColumnOutcome runColumnWithRetry(const CoverageSegmentIntent & world_intent, size_t column)
  {
    // Segment boundary: a requested pause takes effect here (cancel still ends the wait).
    while (mission_fsm_.atBoundary() && !stop()) {
      waitFor(0.2);
    }
    if (stop()) {
      return ColumnOutcome::kFailed;
    }
    const int stand = static_cast<int>(stand_index_);
    ledger_.markRunning(column, stand);
    column_peak_ = 0.0;
    column_reason_ = SkipReason::kPlanFailed;
    auto outcome = runColumn(world_intent, column);
    // A contact overload is not retried: the cause is on the wall, not in the plan.
    const auto overloaded = [&]() {
        return contact_overload_n_ > 0.0 && column_peak_ > contact_overload_n_ && outcome != ColumnOutcome::kDone;
      };
    for (int r = 0; r < column_retries_ && outcome == ColumnOutcome::kFailed && !stop() && !overloaded(); ++r) {
      RCLCPP_WARN(
        this->get_logger(), "column %zu failed, retry %d of %d", column, r + 1, column_retries_);
      if (arm_stuck_) {
        mission_fsm_.enterRecovery();
        if (!recoverArm("column retry")) {
          RCLCPP_WARN(this->get_logger(), "column %zu: the arm is stuck and could not be jogged free", column);
        }
        mission_fsm_.leaveRecovery();
      }
      primeContinuityFromLiveState();
      column_peak_ = 0.0;
      column_reason_ = SkipReason::kPlanFailed;
      outcome = runColumn(world_intent, column);
    }
    column_peaks_[column] = column_peak_;
    if (outcome == ColumnOutcome::kFailed && !stop()) {
      RCLCPP_ERROR(this->get_logger(), "column %zu failed, skipped", column);
    }
    if (!stop()) {   // a stopped column stays RUNNING in the ledger: a resume reruns it
      if (outcome == ColumnOutcome::kDone) {
        ledger_.markDone(column, stand);
      } else if (overloaded()) {
        RCLCPP_ERROR(
          this->get_logger(), "column %zu: contact overload (%.1f N > %.1f N), skipped without retry", column,
          column_peak_, contact_overload_n_);
        ledger_.markSkipped(column, stand, SkipReason::kContactOverload);
      } else if (outcome == ColumnOutcome::kFailed) {
        ledger_.markSkipped(column, stand, column_reason_);
      } else {
        ledger_.markFailed(column, stand);   // partial: rerun on a resume
      }
    }
    return outcome;
  }

  /// Opt-in second pass (retry_failed_columns): every column still failed or partial gets up to max_column_retries more
  /// attempts, each from a fresh stow + prime and with a dock IK solution other than the earlier ones; for a
  /// navigation or dock failure retry_replan_stand re-plans the stand from the column's cells (multi-stand runs; `visit`
  /// is the stand list of the run, null for a single stand). The policy is core/column_retry.hpp; this only moves.
  void retryFailedColumns(
    const std::vector<CoverageSegmentIntent> & sel, const std::vector<size_t> & ids,
    const std::vector<StanceStand> * visit)
  {
    retry_report_.enabled = retry_cfg_.enabled;
    retry_report_.max_retries = retry_cfg_.max_retries;
    if (!retry_cfg_.enabled || stop()) {
      return;
    }
    std::map<size_t, size_t> index_of;   // column id -> index in sel
    for (size_t i = 0; i < ids.size(); ++i) {
      index_of[ids[i]] = i;
    }
    std::vector<ColumnFailure> failures;
    const auto add = [&](size_t col, ColumnOutcome o) {
        const auto e = ledger_.get(col);
        if (index_of.find(col) == index_of.end() || (e.state != LedgerState::kFailed && e.state != LedgerState::kSkipped)) {
          return;   // never ran (no stand reaches it) or not tracked: not a retry candidate
        }
        ColumnFailure f;
        f.column = col;
        f.stand = visit ? e.stand : -1;
        f.outcome = o;
        f.kind = e.reason == SkipReason::kNavFailed ? RetryFailure::kNav :
          e.reason == SkipReason::kDockFailed ? RetryFailure::kDock :
          e.reason == SkipReason::kPlanFailed ? RetryFailure::kPlan :
          e.reason == SkipReason::kContactOverload ? RetryFailure::kOverload : RetryFailure::kOther;
        failures.push_back(f);
      };
    for (const size_t c : tracker_.failed()) {add(c, ColumnOutcome::kFailed);}
    for (const size_t c : tracker_.partial()) {add(c, ColumnOutcome::kPartial);}
    const auto plan = planColumnRetries(failures, retry_cfg_, &retry_report_.not_retried);
    if (plan.empty()) {
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "column retry pass: %zu column(s), up to %d attempt(s) each", plan.size(), retry_cfg_.max_retries);
    int at_stand = (visit && stand_index_ < tracker_.stands().size() && tracker_.stands()[stand_index_].reached) ?
      static_cast<int>(stand_index_) : -1;
    const auto attempt = [&](const ColumnRetryPlan & p, int n, const RetryVariant & v) {
        RetryAttemptResult r;
        const size_t i = index_of.at(p.column);
        RCLCPP_WARN(
          this->get_logger(), "column %zu retry %d (%s): %s", p.column, n, v.label().c_str(), p.first_outcome ==
          ColumnOutcome::kPartial ? "was partial" : "failed before");
        if (!travel_state_.empty() && arm_deployed_) {
          setState("stowing", -1);
          if (!stowArm()) {
            r.fatal = true;
            r.kind = RetryFailure::kOther;
            r.note = stop() ? "canceled" : "could not stow the arm";
            return r;
          }
          arm_deployed_ = false;
        }
        if (visit && p.stand >= 0 && static_cast<size_t>(p.stand) < visit->size()) {
          StanceStand target = (*visit)[static_cast<size_t>(p.stand)];
          bool need_drive = at_stand != p.stand;
          if (v.restand) {
            std::vector<StanceStand> ns;
            std::vector<std::vector<size_t>> nt;
            std::string why;
            if (planStands({sel[i]}, ns, nt, why) && !ns.empty()) {
              size_t best = 0;
              for (size_t k = 1; k < ns.size(); ++k) {
                if (nt[k].size() > nt[best].size()) {best = k;}
              }
              target = ns[best];
              need_drive = true;
              r.note = "new stand (" + std::to_string(target.x) + ", " + std::to_string(target.y) + ")";
            } else {
              r.note = "stand re-plan failed (" + why + "), kept the old stand";
            }
          }
          if (need_drive) {
            std::string msg;
            setState("navigating", -1);
            bool ok = navigateToStand(target, msg);
            if (ok && stand_check_enabled_) {
              bool gate_abort = false;
              ok = gateStand(target, msg, gate_abort);
            }
            if (stop()) {
              r.fatal = true;
              r.note = "canceled";
              return r;
            }
            if (!ok) {
              at_stand = -1;
              r.kind = RetryFailure::kNav;
              r.note += (r.note.empty() ? "" : "; ") + std::string("navigation failed: ") + msg;
              ledger_.markSkipped(p.column, p.stand, SkipReason::kNavFailed);
              return r;
            }
            at_stand = p.stand;
            if (!arrival_command_.empty()) {
              if (std::system(arrival_command_.c_str()) != 0) {
                RCLCPP_WARN(this->get_logger(), "stand_arrival_command failed");
              }
              waitFor(1.0);
            }
          }
        }
        setState("priming", -1);
        primeContinuityFromLiveState();
        if (!travel_state_.empty()) {
          last_good_state_ = travel_state_;
        }
        dock_branch_seed_.clear();   // clean pose: the continuity reference is the live state again
        arm_stuck_ = false;
        dock_ik_exclude_.used = column_dock_q_[p.column];
        dock_ik_exclude_.allow_branch_change = v.allow_branch_change;
        dock_ik_exclude_on_ = v.vary_ik && !dock_ik_exclude_.used.empty();
        arm_deployed_ = true;
        const auto outcome = runColumnWithRetry(sel[i], p.column);
        dock_ik_exclude_on_ = false;
        r.outcome = outcome;
        if (outcome != ColumnOutcome::kDone) {
          const auto e = ledger_.get(p.column);
          r.kind = e.reason == SkipReason::kNavFailed ? RetryFailure::kNav :
            e.reason == SkipReason::kDockFailed ? RetryFailure::kDock :
            e.reason == SkipReason::kPlanFailed ? RetryFailure::kPlan :
            e.reason == SkipReason::kContactOverload ? RetryFailure::kOverload : RetryFailure::kOther;
        }
        r.fatal = stop();
        tracker_.columnRetried(
          p.stand >= 0 ? static_cast<size_t>(p.stand) : tracker_.stands().size(), p.column, outcome);
        return r;
      };
    auto rep = runColumnRetries(plan, retry_cfg_, attempt, [this]() {return stop();});
    rep.not_retried.insert(
      rep.not_retried.begin(), retry_report_.not_retried.begin(), retry_report_.not_retried.end());
    retry_report_ = rep;
    RCLCPP_INFO(this->get_logger(), "column retry pass done: %s", tracker_.summary().c_str());
  }

  /// Remembers where the arm is as the configuration to jog back to.
  void markGoodState()
  {
    const auto live = readCurrentJointState(shared_from_this(), joint_name_candidates_);
    if (!live.empty()) {
      last_good_state_ = live;
    }
    arm_stuck_ = false;
  }

  /// A joint-space jog from the arm's live state toward last_good_state_ (planRecoveryJog), executed straight on
  /// the controller: the planners refuse a start state in self-collision, so there is no plan to ask for. Returns
  /// true when the jog ran and succeeded.
  bool recoverArm(const char * what)
  {
    if (!recovery_jog_ || (last_good_state_.empty() && !(recovery_keeps_branch_ && !dock_branch_seed_.empty())) ||
      stop())
    {
      return false;
    }
    std::vector<std::string> names;
    const auto live = readCurrentJointState(shared_from_this(), joint_name_candidates_, &names);
    if (live.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s: recovery jog skipped, no live joint state", what);
      return false;
    }
    std::vector<double> jog_target = last_good_state_;
    if (recovery_keeps_branch_) {
      const auto t = selectRecoveryJogTarget(live, last_good_state_, dock_branch_seed_, dock_ik_sel_);
      if (!t.ok) {
        RCLCPP_WARN(
          this->get_logger(), "%s: recovery jog skipped, no known configuration on the arm's current branch", what);
        return false;
      }
      if (t.from_branch_seed || t.last_good_rejected) {
        RCLCPP_WARN(
          this->get_logger(), "%s: last good configuration is on another branch, jogging toward %s", what,
          t.from_branch_seed ? "the last docked one" : "it anyway");
      }
      jog_target = t.q;
    }
    const auto jog = planRecoveryJog(live, jog_target, jog_params_);
    if (!jog.ok) {
      RCLCPP_WARN(this->get_logger(), "%s: recovery jog refused: %s", what, jog.reason.c_str());
      return false;
    }
    if (!jog.needed) {
      return false;   // already where it was good: the jog would change nothing
    }
    ExecutionRequest req;
    req.planning_group = planning_group_;
    req.trajectory.joint_names = names;
    core_types::TrajectoryPoint first;
    first.positions = live;
    req.trajectory.points.push_back(first);
    for (const auto & p : jog.points) {
      core_types::TrajectoryPoint tp;
      tp.positions = p.positions;
      tp.time_from_start_sec = p.time_from_start_s;
      req.trajectory.points.push_back(tp);
    }
    RCLCPP_WARN(
      this->get_logger(), "%s: recovery jog toward the last good configuration (%zu steps, %.1f s)", what,
      jog.points.size(), jog.points.back().time_from_start_s);
    setState("recovering");
    const auto res = execution_adapter_->execute(req);
    if (!res.success) {
      RCLCPP_ERROR(this->get_logger(), "%s: recovery jog failed: %s", what, res.message.c_str());
      return false;
    }
    primeContinuityFromLiveState();
    return true;
  }

  /// The column that follows in the same stand (nullptr: none), for contact_connectors. Cleared after the stand.
  void setNextInStand(const CoverageSegmentIntent * next, size_t id)
  {
    next_in_stand_ = contact_connectors_ ? next : nullptr;
    next_in_stand_id_ = id;
  }

  ColumnOutcome runColumn(const CoverageSegmentIntent & world_intent, size_t column)
  {
    setState("docking", static_cast<int>(column));
    // The previous column of this stand slid onto this column's first stroke pose in contact (contact_connectors): the
    // force loop is engaged and the tool is there, so the dock, the approach and the press are skipped.
    const bool carried = connector_carry_ && connector_carry_column_ == column;
    connector_carry_ = false;
    // The column as planned in world_frame, expressed in frame_id for where the base is now. The
    // wall direction is rotated with it (the base may have stopped at another yaw).
    CoverageSegmentIntent intent = world_intent;
    WallDirection wall_dir = wall_dir_;
    std::optional<CoverageSegmentIntent> next_intent;
    if (next_in_stand_ != nullptr && !next_in_stand_->poses.empty()) {
      next_intent = *next_in_stand_;
    }
    if (!world_frame_.empty()) {
      FrameTransform base_in_world;
      if (!lookup(world_frame_, frame_id_, base_in_world)) {
        RCLCPP_ERROR(this->get_logger(), "%s: no base pose, skipping column", intent.label.c_str());
        return ColumnOutcome::kFailed;
      }
      const auto world_in_base = base_in_world.inverse();
      intent = transformIntent(world_in_base, world_intent);
      wall_dir = world_in_base.apply(wall_dir_);
      if (next_intent) {next_intent = transformIntent(world_in_base, *next_intent);}
      RCLCPP_INFO(
        this->get_logger(), "%s: base at (%.3f, %.3f) yaw %.3f in %s; column starts at (%.3f, %.3f, %.3f) in %s",
        intent.label.c_str(), base_in_world.t[0], base_in_world.t[1], base_in_world.yaw(),
        world_frame_.c_str(), intent.poses.front().x, intent.poses.front().y, intent.poses.front().z,
        frame_id_.c_str());
    }
    if (!columnIkAllows(intent, wall_dir)) {
      return ColumnOutcome::kFailed;
    }
    logArmState("before_dock_" + intent.label);
    const bool chain_ok = chain_valid_;   // only a column that finished right before this one can be chained from
    chain_valid_ = false;
    const auto & rep = intent.poses.front();
    // Docking in the middle of the column is the original behaviour; at the stroke's first pose the
    // contact path starts from the arm configuration it docked in (no IK branch jump at the start).
    const double dock_z = dock_at_stroke_start_ ? intent.poses.front().z
                                                : intent.poses[intent.poses.size() / 2].z;

    core_types::Pose standoff_pose = rep;
    standoff_pose.z = dock_z;
    standoff_pose = wall_dir.shifted(standoff_pose, -dock_standoff_);
    standoff_pose.qx = rep.qx;
    standoff_pose.qy = rep.qy;
    standoff_pose.qz = rep.qz;
    standoff_pose.qw = rep.qw;

    if (!carried) {
      MotionSegmentSpec dock_seg;
      dock_seg.label = intent.label + "_dock";
      dock_seg.mode_hint = "fsp";
      dock_seg.waypoint_indices = {0};
      // MotionPlanner always tries Cartesian first for an fsp-hinted segment, keeping that
      // result if it clears min_path_fraction (default 0.5) -- too low for a single free-space
      // waypoint: a partial straight-line Cartesian attempt (observed live: 61.5%, truncated by
      // the same base/lift self-collision this repo's own contact tuning already works around)
      // leaves the arm short of the actual target, not usably "docked". Require near-full
      // completion so anything but a clean, direct Cartesian path properly falls back to OMPL.
      dock_seg.min_path_fraction = 0.99;
      // OMPL's own sampling is randomized -- coverage_pass_demo_node.cpp's own retract already
      // retries for exactly this reason (a genuine failure to find any path is worth one more
      // attempt before giving up); dock deserves the same tolerance.
      MotionExec dock_exec;
      const core_types::Pose first_goal = dock_approach_extra_m_ > 0.0 ?
        wall_dir.approachPose(standoff_pose, dock_approach_extra_m_) : standoff_pose;
      const auto dock_round = [&]() {
      // the first goal (the approach pose, or the dock pose without an approach) as a joint goal on the branch of
      // the previous dock; every second attempt falls back to the pose goal the planner samples itself
      std::vector<double> goal_q;
      if (dock_ik_seeded_ || dock_ik_continuity_) {
        goal_q = seededDockIk(first_goal);
      }
      bool pin_q = false;   // recovery_keeps_branch: after a short approach every attempt uses the joint goal
      for (int attempt = 0; attempt < dock_attempts_ && !stop(); ++attempt) {
        const bool use_q = !goal_q.empty() && (attempt % 2 == 0 || pin_q);
        bool did_chain = false;
        if (attempt == 0) {
          const auto cd = decideChain(
            chain_columns_, chain_ok, arm_stuck_, dock_approach_extra_m_ > 0.0,
            {chain_last_standoff_.x, chain_last_standoff_.y, chain_last_standoff_.z},
            {first_goal.x, first_goal.y, first_goal.z}, chain_max_shift_m_);
          if (cd.chain) {
            MotionSegmentSpec chain_seg = dock_seg;
            chain_seg.label = intent.label + "_chain";
            chain_seg.mode_hint = "cartesian";
            const auto chain_exec = manager_.executeMotionSegments(
              frame_id_, {chain_seg}, std::vector<core_types::Pose>{first_goal}, planning_group_);
            double chain_err = 0.0;
            if (chain_exec.success && (waitFor(0.5, false), tcpNear(first_goal, approach_tolerance_m_, chain_err))) {
              RCLCPP_INFO(
                this->get_logger(), "%s: chained from the previous column in one straight move (%.3f m)",
                intent.label.c_str(), cd.shift_m);
              did_chain = true;
            } else {
              RCLCPP_WARN(
                this->get_logger(), "%s: chain move %s, using the normal approach", intent.label.c_str(),
                chain_exec.success ? "ended short" : "failed");
            }
          }
        }
        if (dock_approach_extra_m_ > 0.0 && !did_chain) {
          MotionSegmentSpec approach_seg = dock_seg;
          approach_seg.label = intent.label + "_approach";
          if (use_q) {
            approach_seg.target_joint_state = goal_q;
          }
          const auto approach_exec = manager_.executeMotionSegments(
            frame_id_, {approach_seg}, use_q ? std::vector<core_types::Pose>{} :
            std::vector<core_types::Pose>{first_goal},
            planning_group_);
          if (!approach_exec.success) {
            RCLCPP_WARN(
              this->get_logger(), "%s: approach attempt %d failed, retrying...", intent.label.c_str(),
              attempt + 1);
            continue;
          }
          // "completed" only says the trajectory ran: an arm that did not move (the first plan after a drive that
          // is a no-op) or that rests on the wall's top edge half way stops short of the pose, and the dock from
          // there starts a Cartesian path from the wrong place and ends on another branch. Check where the tool is.
          if (verify_approach_) {
            waitTcpSettled();
            double err = 0.0;
            if (!tcpNear(first_goal, approach_tolerance_m_, err)) {
              RCLCPP_WARN(
                this->get_logger(), "%s: approach attempt %d ended %.3f m from the approach pose, not docking from there",
                intent.label.c_str(), attempt + 1, err);
              logArmState("approach_short_" + intent.label);
              if (!recoverArm("approach")) {
                primeContinuityFromLiveState();
              }
              if (recovery_keeps_branch_) {
                const auto g = recoveryGoalIk(first_goal);
                if (!g.empty()) {
                  goal_q = g;
                  pin_q = true;
                }
              }
              continue;
            }
          }
        }
        if (dock_approach_extra_m_ > 0.0 || !use_q) {
          dock_exec = manager_.executeMotionSegments(
            frame_id_, {dock_seg}, {standoff_pose}, planning_group_);
        } else {
          MotionSegmentSpec q_seg = dock_seg;
          q_seg.target_joint_state = goal_q;
          dock_exec = manager_.executeMotionSegments(frame_id_, {q_seg}, {}, planning_group_);
        }
        if (dock_exec.success) {
          break;
        }
        RCLCPP_WARN(
          this->get_logger(), "%s: dock attempt %d failed, retrying...", intent.label.c_str(),
          attempt + 1);
      }
      };
      dock_round();
      // Every try failed: a start state the planners refuse (self-collision at the end of the last reach) is
      // moved back toward the last good configuration once, then the tries are repeated.
      if (!dock_exec.success && !stop() && recoverArm("dock")) {
        dock_round();
      }
      if (!dock_exec.success) {
        RCLCPP_WARN(
          this->get_logger(), "%s: dock %s, skipping column", intent.label.c_str(),
          stop() ? "stopped" : "failed");
        column_reason_ = SkipReason::kDockFailed;
        return ColumnOutcome::kFailed;
      }

      if (stop()) {
        return ColumnOutcome::kFailed;
      }
      if (verify_dock_) {
        waitTcpSettled(); // let the controller and TF settle after the last point, as verify_approach does
        double pos_err = 0.0, ang_err = 0.0;
        if (tcpPoseError(standoff_pose, pos_err, ang_err) &&
          (pos_err > dock_tolerance_m_ || ang_err > dock_tolerance_rad_))
        {
          RCLCPP_WARN(
            this->get_logger(),
            "%s: docked %.3f m and %.1f deg off the standoff pose (limits %.3f m, %.1f deg), not pressing from there",
            intent.label.c_str(), pos_err, ang_err * 180.0 / M_PI, dock_tolerance_m_,
            dock_tolerance_rad_ * 180.0 / M_PI);
          logArmState("dock_off_pose_" + intent.label);
          column_reason_ = SkipReason::kDockFailed;
          return ColumnOutcome::kFailed;
        }
      }
      markGoodState();
      dock_branch_seed_ = last_good_state_;
      logArmState("docked_" + intent.label);
    }
    if (!contact_) {
      return runSprayStroke(intent, wall_dir, standoff_pose);
    }

    if (!carried) {setAdmittanceEngaged(true);}
    {
      std_msgs::msg::Float64 msg;
      msg.data = target_force_n_;
      force_pub_->publish(msg);
    }
    if (!carried && !waitFor(0.5)) {
      return ColumnOutcome::kFailed;   // canceled: the task's cleanup unloads and disengages
    }
    tracking_peak_ = true;
    setState("pressing");

    MotionSegmentSpec contact_seg;
    contact_seg.label = intent.label + "_contact";
    contact_seg.mode_hint = "cartesian";
    contact_seg.target_eef_speed = intent.target_eef_speed;
    contact_seg.min_path_fraction = 0.05;
    contact_seg.allow_target_contact = true;
    // The planner lifts the tool-vs-wall collision for the contact link only when it is told which object the wall is:
    // until now nothing set this, so the exclusion never applied, which went unnoticed while the planner's tool had no
    // geometry near the TCP. The tool now carries its contact pad (generic_tool.xacro) and touches the wall by design;
    // the wall-penetration bound on the TCP stays the safety limit of the contact path.
    contact_seg.target_collision_object_id = motion_planner::kWallOctomapCollisionObjectId;
    contact_seg.max_wall_penetration = max_wall_penetration_;
    contact_seg.wall_reference_pose.x = rep.x;
    contact_seg.wall_reference_pose.y = rep.y;
    contact_seg.wall_reference_pose.z = rep.z;
    const auto wall_q = wall_dir.outwardNormalQuaternion();
    contact_seg.wall_reference_pose.qx = wall_q[0];
    contact_seg.wall_reference_pose.qy = wall_q[1];
    contact_seg.wall_reference_pose.qz = wall_q[2];
    contact_seg.wall_reference_pose.qw = wall_q[3];

    std::vector<core_types::Pose> contact_waypoints;
    contact_waypoints.reserve(intent.poses.size());
    for (size_t i = 0; i < intent.poses.size(); ++i) {
      core_types::Pose p = wall_dir.shifted(
        intent.poses[i], columnSteps(true, contact_y_overshoot_, 0.0).stroke_shift_m);
      contact_waypoints.push_back(p);
    }

    // Press first, then stroke: the admittance needs a couple of seconds at rest to settle on the
    // target force; stroking straight out of the approach drags the first-contact creep (about
    // 13 N) along the wall.
    if (!carried && press_settle_s_ > 0.0 && !contact_waypoints.empty()) {
      MotionSegmentSpec press_seg = contact_seg;
      press_seg.label = intent.label + "_press";
      press_seg.waypoint_indices = {0};
      const auto press_exec = manager_.executeMotionSegments(
        frame_id_, {press_seg}, {contact_waypoints.front()}, planning_group_);
      if (press_exec.success) {
        waitFor(press_settle_s_);
      }
    }
    if (stop()) {
      return ColumnOutcome::kFailed;
    }
    setState("stroking");
    for (size_t i = 0; i < contact_waypoints.size(); ++i) {
      contact_seg.waypoint_indices.push_back(i);
    }
    const auto contact_exec = manager_.executeMotionSegments(
      frame_id_, {contact_seg}, contact_waypoints, planning_group_);
    RCLCPP_INFO(
      this->get_logger(), "%s: contact success=%d completed=%zu", intent.label.c_str(),
      contact_exec.success, contact_exec.waypoints_completed);

    tracking_peak_ = false;
    if (stop()) {
      return ColumnOutcome::kFailed;
    }
    // Contact connector: slide to the start of the next strip with the force loop engaged, no unload, retract or dock.
    if (contact_connectors_ && next_intent && !contact_waypoints.empty()) {
      const core_types::Pose next_start = wall_dir.shifted(
        next_intent->poses.front(), columnSteps(true, contact_y_overshoot_, 0.0).stroke_shift_m);
      const bool complete = contact_exec.success && contact_exec.waypoints_completed == intent.poses.size();
      const auto dec = decideContactConnector(
        true, true, complete, !arm_stuck_, contact_waypoints.back(), next_start, wall_dir, connector_cfg_);
      std::string why = dec.reason;
      bool slid = false;
      if (dec.eligible) {
        const auto saved_reason = column_reason_;
        const bool ik_ok = columnIkAllows(*next_intent, wall_dir);   // the IK chain of the next strip
        column_reason_ = saved_reason;
        if (!ik_ok) {
          why = "no continuous IK chain on the next strip";
        } else {
          const auto target = connectorTarget(contact_waypoints.back(), next_start, wall_dir);
          MotionSegmentSpec conn = contact_seg;
          conn.label = intent.label + "_connector";
          conn.waypoint_indices = {0};
          conn.min_path_fraction = 0.99;   // all of the slide or none of it
          setState("connecting");
          const auto conn_exec = manager_.executeMotionSegments(frame_id_, {conn}, {target}, planning_group_);
          double err = 0.0;
          if (!conn_exec.success) {
            why = "connector move failed";
          } else if (!tcpNear(target, approach_tolerance_m_, err)) {
            why = "connector ended " + std::to_string(err) + " m short";
          } else {
            slid = true;
          }
        }
      }
      if (slid) {
        connector_log_.used();
        RCLCPP_INFO(
          this->get_logger(), "%s: slid %.3f m along the wall to the next strip in contact", intent.label.c_str(),
          dec.step_m);
        connector_carry_ = true;
        connector_carry_column_ = next_in_stand_id_;
        chain_valid_ = false;
        return ColumnOutcome::kDone;
      }
      connector_log_.fellBack(why);
      RCLCPP_WARN(
        this->get_logger(), "%s: no contact connector (%s), retracting and approaching as usual", intent.label.c_str(),
        why.c_str());
    }
    // Back the reference off the wall while the admittance is still engaged: switching it off with the
    // reference 1.5 mm inside the wall snaps the arm onto that reference (27-31 N spikes at every
    // stroke end). The admittance releases its offset against a reference that no longer presses.
    if (unload_m_ > 0.0 && !contact_waypoints.empty()) {
      MotionSegmentSpec unload_seg = contact_seg;
      unload_seg.label = intent.label + "_unload";
      unload_seg.waypoint_indices = {0};
      unload_seg.allow_target_contact = false;
      unload_seg.min_path_fraction = 0.5;
      const auto unload_pose = wall_dir.shifted(contact_waypoints.back(), -(contact_y_overshoot_ + unload_m_));
      manager_.executeMotionSegments(frame_id_, {unload_seg}, {unload_pose}, planning_group_);
    }
    // Ramp the force target to zero and let the admittance unload the wall before it is switched off.
    {
      std_msgs::msg::Float64 zero;
      zero.data = 0.0;
      force_pub_->publish(zero);
    }
    if (unload_release_force_n_ <= 0.0 || force_now_ < 0.0) {
      waitFor(1.5, false);   // no force signal (or switched off): the old fixed wait
    } else {
      waitFor(0.5, false);   // the target has to reach the admittance and the tool start to move off
      const auto t0 = std::chrono::steady_clock::now();
      const auto elapsed = [&]() {
          return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        };
      // the tool speed from TF: the position change over the last sample; settled = below the limit for 0.3 s
      FrameTransform prev_tcp;
      bool have_prev = lookup(frame_id_, "tcp_link", prev_tcp);
      double prev_t = elapsed(), still_since = -1.0, speed = 0.0;
      const bool watch_speed = unload_release_speed_mps_ > 0.0 && have_prev;
      while (elapsed() < unload_timeout_s_) {
        waitFor(0.05, false);
        bool settled = true;
        if (watch_speed) {
          FrameTransform cur;
          const double now_t = elapsed();
          if (lookup(frame_id_, "tcp_link", cur) && now_t > prev_t) {
            speed = std::sqrt(
              std::pow(cur.t[0] - prev_tcp.t[0], 2) + std::pow(cur.t[1] - prev_tcp.t[1], 2) +
              std::pow(cur.t[2] - prev_tcp.t[2], 2)) / (now_t - prev_t);
            prev_tcp = cur;
            prev_t = now_t;
          }
          if (speed > unload_release_speed_mps_) {
            still_since = -1.0;
          } else if (still_since < 0.0) {
            still_since = now_t;
          }
          settled = still_since >= 0.0 && now_t - still_since >= 0.3;
        }
        if (force_now_ <= unload_release_force_n_ && settled) {
          break;
        }
      }
      const double waited = 0.5 + elapsed();
      if (force_now_ > unload_release_force_n_ || elapsed() >= unload_timeout_s_) {
        RCLCPP_WARN(
          this->get_logger(),
          "%s: force %.1f N (limit %.1f N), tool speed %.1f mm/s after %.1f s, releasing the admittance anyway",
          intent.label.c_str(), force_now_, unload_release_force_n_, speed * 1000.0, waited);
      } else {
        RCLCPP_INFO(
          this->get_logger(), "%s: force %.1f N, tool speed %.1f mm/s after %.1f s, releasing the admittance",
          intent.label.c_str(), force_now_, speed * 1000.0, waited);
      }
    }
    // The admittance stays engaged through the retract and is released only once the tool is off the wall: switching
    // it off while the pad is pressed on the wall lets the arm jump to its reference (88 to 233 N spikes on the xArm 7
    // in about half the runs, even after the force had decayed and the reference had been backed off).
    setState("retracting");
    // Back off at the end of the stroke for a serpentine plan (the next column starts there), else at its start.
    core_types::Pose retract_pose = chooseRetractPose(serpentine_columns_, standoff_pose, intent.poses);
    if (serpentine_columns_ && contact_exec.waypoints_completed < intent.poses.size()) {
      // a stroke that stopped early: back off from where the tool really is, not from the planned end of the stroke
      FrameTransform here;
      if (lookup(frame_id_, "tcp_link", here)) {retract_pose.z = here.t[2];}
    }
    MotionSegmentSpec retract_seg;
    retract_seg.label = intent.label + "_retract";
    // a serpentine retract leaves the wall from the stroke's end (often the top of the reach): the straight move away
    // from the wall is deterministic, where a randomized joint-space plan from there failed; fsp stays the fallback
    retract_seg.mode_hint = retractModeHint(serpentine_columns_);
    retract_seg.waypoint_indices = {0};
    retract_seg.min_path_fraction = 0.99;  // see dock_seg's own comment above for why
    const auto retract_exec = manager_.executeMotionSegments(
      frame_id_, {retract_seg}, {retract_pose}, planning_group_);
    setAdmittanceEngaged(false);
    if (!retract_exec.success) {
      RCLCPP_WARN(this->get_logger(), "%s: retract failed", intent.label.c_str());
      retract_seg.mode_hint = retractFallbackHint();
      const auto retract_again = [&]() {
          for (int attempt = 0; attempt < dock_attempts_ && !stop(); ++attempt) {
            if (manager_.executeMotionSegments(
                frame_id_, {retract_seg}, {retract_pose}, planning_group_).success)
            {
              return true;
            }
          }
          return false;
        };
      // a failed straight retract is first tried as one plain free-space plan from where the arm is; the jog is the
      // next resort
      bool freed = serpentine_columns_ && !stop() && manager_.executeMotionSegments(
        frame_id_, {retract_seg}, {retract_pose}, planning_group_).success;
      if (!freed && !stop() && recoverArm("retract")) {
        freed = retract_again();
      }
      if (!freed && !stop() && canRecoverWithoutWall()) {
        freed = withWallCleared("retract", retract_again);
      }
      if (freed) {
        arm_stuck_ = false;   // not a new good state: the pose after a retract can sit just inside a self-collision
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s: the retract recovery failed too", intent.label.c_str());
        arm_stuck_ = true;
      }
    } else {
      arm_stuck_ = false;
    }

    chain_valid_ = contact_exec.success && contact_exec.waypoints_completed > 0 && !arm_stuck_;
    chain_last_standoff_ = retract_pose;
    if (!contact_exec.success || contact_exec.waypoints_completed == 0) {
      return ColumnOutcome::kFailed;
    }
    return contact_exec.waypoints_completed == intent.poses.size()
      ? ColumnOutcome::kDone : ColumnOutcome::kPartial;
  }

  /// The non-contact column after the dock: move to the first stroke pose with the valve closed, open the
  /// valve, stroke along the column at the plan's stand-off (no force engage, no press, no contact
  /// allowance), close the valve, retract to the dock pose.
  ColumnOutcome runSprayStroke(
    const CoverageSegmentIntent & intent, const WallDirection & wall_dir,
    const core_types::Pose & standoff_pose)
  {
    const auto steps = columnSteps(false, 0.0, spray_extra_standoff_);
    std::vector<core_types::Pose> waypoints;
    waypoints.reserve(intent.poses.size());
    for (const auto & p : intent.poses) {
      waypoints.push_back(wall_dir.shifted(p, steps.stroke_shift_m));
    }

    MotionSegmentSpec stroke_seg;
    stroke_seg.label = intent.label + "_spray";
    stroke_seg.mode_hint = "cartesian";
    stroke_seg.target_eef_speed = intent.target_eef_speed;
    stroke_seg.min_path_fraction = 0.05;
    stroke_seg.allow_target_contact = steps.allow_wall_contact;   // false: the wall is an obstacle

    // Valve closed: take up the first stroke pose from the dock pose (a straight move).
    setValve(false);
    setState("stroking");
    MotionSegmentSpec start_seg = stroke_seg;
    start_seg.label = intent.label + "_start";
    start_seg.waypoint_indices = {0};
    manager_.executeMotionSegments(frame_id_, {start_seg}, {waypoints.front()}, planning_group_);

    for (size_t i = 0; i < waypoints.size(); ++i) {
      stroke_seg.waypoint_indices.push_back(i);
    }
    setValve(steps.valve);
    if (tool_feed_) {tool_feed_->arm(steps.valve);}   // this trajectory is the tool window
    const auto stroke_exec = manager_.executeMotionSegments(
      frame_id_, {stroke_seg}, waypoints, planning_group_);
    if (tool_feed_) {tool_feed_->arm(false);}
    setValve(false);
    RCLCPP_INFO(
      this->get_logger(), "%s: spray success=%d completed=%zu", intent.label.c_str(),
      stroke_exec.success, stroke_exec.waypoints_completed);
    if (stop()) {
      return ColumnOutcome::kFailed;
    }
    setState("retracting");

    // Back off at the end of the stroke for a serpentine plan (the next column starts there), else at its start.
    core_types::Pose retract_pose = chooseRetractPose(serpentine_columns_, standoff_pose, intent.poses);
    if (serpentine_columns_ && stroke_exec.waypoints_completed < intent.poses.size()) {
      // a stroke that stopped early: back off from where the tool really is, not from the planned end of the stroke
      FrameTransform here;
      if (lookup(frame_id_, "tcp_link", here)) {retract_pose.z = here.t[2];}
    }
    MotionSegmentSpec retract_seg;
    retract_seg.label = intent.label + "_retract";
    // a serpentine retract leaves the wall from the stroke's end (often the top of the reach): the straight move away
    // from the wall is deterministic, where a randomized joint-space plan from there failed; fsp stays the fallback
    retract_seg.mode_hint = retractModeHint(serpentine_columns_);
    retract_seg.waypoint_indices = {0};
    retract_seg.min_path_fraction = 0.99;
    const auto retract_exec = manager_.executeMotionSegments(
      frame_id_, {retract_seg}, {retract_pose}, planning_group_);
    if (!retract_exec.success) {
      RCLCPP_WARN(this->get_logger(), "%s: retract failed", intent.label.c_str());
      retract_seg.mode_hint = retractFallbackHint();
      // a start state the planners refuse: jog it free, then try the retract again
      const bool plain_fsp_ok = serpentine_columns_ && !stop() && manager_.executeMotionSegments(
        frame_id_, {retract_seg}, {retract_pose}, planning_group_).success;
      if (plain_fsp_ok || (!stop() && recoverArm("retract") &&
        manager_.executeMotionSegments(
          frame_id_, {retract_seg}, {retract_pose}, planning_group_).success))
      {
        arm_stuck_ = false;
      } else {
        arm_stuck_ = true;
      }
    } else {
      arm_stuck_ = false;
    }

    chain_valid_ = stroke_exec.success && stroke_exec.waypoints_completed > 0 && !arm_stuck_;
    chain_last_standoff_ = retract_pose;
    if (!stroke_exec.success || stroke_exec.waypoints_completed == 0) {
      return ColumnOutcome::kFailed;
    }
    return stroke_exec.waypoints_completed == intent.poses.size()
      ? ColumnOutcome::kDone : ColumnOutcome::kPartial;
  }

  // Plugin plumbing. Declared before manager_ so the manager (which owns the planners) is
  // destroyed first; the registry's plugin libraries stay loaded for the whole process.
  // Task state (all touched from the main thread only: the action callbacks run inside the node's spins).
  bool run_on_start_{true};
  int nav_attempts_{2};
  bool stand_check_enabled_{false};
  double stand_check_reach_margin_{-1.0};
  StandCheck stand_check_;
  double last_arrival_pos_err_m_{0.0};
  double last_arrival_yaw_err_rad_{0.0};
  double stand_max_span_{0.0};
  double stand_merge_tol_{0.15};
  double stand_min_clearance_{0.0};
  std::string arrival_command_;
  std::string wall_clear_command_;
  double nav_timeout_s_{240.0};
  bool wall_dock_enabled_{false};
  std::string wall_dock_action_{"wall_dock"};
  double wall_dock_standoff_m_{0.30};
  double wall_dock_timeout_s_{0.0};
  bool wall_dock_planned_standoff_{false};
  WallDockConfig wall_dock_cfg_;
  std::map<size_t, WallDockOutcome> verify_wall_dock_;   // stand -> last wall dock outcome of the run
  rclcpp_action::Client<WallDock>::SharedPtr wall_dock_client_;
  std::vector<double> travel_joint_state_;
  std::vector<std::string> group_joint_names_;
  std::vector<std::vector<std::string>> joint_name_candidates_;
  std::vector<double> travel_state_;
  std::vector<double> last_good_state_;   ///< the last arm configuration known to plan from (recovery target)
  bool arm_stuck_{false};                 ///< a retract or stow failed and no recovery got the arm out yet
  int column_retries_{1};
  ColumnRetryConfig retry_cfg_;                    ///< retry_failed_columns & co (column_retry.hpp)
  ColumnRetryReport retry_report_;                 ///< what the second pass did, for the verify result
  DockIkExclusion dock_ik_exclude_;                ///< dock solutions a retry must differ from (empty used = off)
  bool dock_ik_exclude_on_{false};
  std::map<size_t, std::vector<std::vector<double>>> column_dock_q_;   ///< column -> the dock solutions chosen so far
  bool recovery_jog_{true};
  RecoveryJogParams jog_params_;
  std::string column_order_{"input"};
  bool arm_deployed_{false};
  rclcpp_action::Server<ExecuteCoverage>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> pending_goal_;
  bool busy_{false};
  bool cancel_requested_{false};
  bool admittance_engaged_{false};
  bool tracking_peak_{false};
  double peak_force_{-1.0};
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp_action::Client<NavigateToStand>::SharedPtr nav_client_;
  rclcpp_action::ClientGoalHandle<NavigateToStand>::SharedPtr nav_goal_;
  std::function<void(const std::shared_ptr<ExecuteCoverage::Feedback> &)> feedback_sink_;
  TaskTracker tracker_;
  MissionLedger ledger_;
  MissionFsm mission_fsm_;
  double contact_overload_n_{0.0};
  double column_peak_{0.0};
  std::map<size_t, double> column_peaks_;
  std::string verify_result_path_;
  double verify_max_peak_n_{0.0};
  bool column_ik_check_{false};
  double column_ik_lift_step_{0.1};
  bool column_ik_reject_on_refused_{false};
  double column_ik_ratio_limit_{6.0};
  std::vector<int64_t> column_ik_branch_joints_, column_ik_prismatic_joints_, column_ik_continuous_joints_;
  std::unique_ptr<ReachabilityColumnIkPort> column_ik_port_;
  ColumnIkGate column_ik_gate_;
  std::map<size_t, std::vector<size_t>> verify_stand_columns_;   // stand -> column ids of the last multi-stand run
  bool verify_trace_{false};
  bool verify_geometric_{false};
  bool verify_path_quality_{false};
  GeometricCoverageConfig verify_geo_cfg_;
  double verify_contact_force_n_{1.0};
  double verify_min_geometric_percent_{0.0};
  double verify_trace_hz_{20.0};
  std::string verify_path_chain_file_;
  int verify_path_skip_joints_{1};
  reachability::PathQualityConfig verify_path_cfg_;
  ToolTrace trace_;
  std::unique_ptr<ReachabilityPathQualityPort> path_quality_;
  rclcpp::TimerBase::SharedPtr trace_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cache_sub_;
  std::mutex joint_cache_mutex_;
  std::map<std::string, double> joint_cache_;
  std::vector<std::string> trace_joint_names_;
  SkipReason column_reason_{SkipReason::kPlanFailed};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
  rclcpp::Service<MissionCommandSrv>::SharedPtr mission_command_srv_;
  std::string state_{"idle"};
  size_t stand_index_{0};
  size_t stand_count_{1};
  int column_{-1};

  std::string planning_config_;
  std::unique_ptr<motion_planner::PluginRegistry> plugin_registry_;
  motion_planner::PluginContext plugin_context_;
  motion_planner::PlanningPipeline pipeline_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> plugin_executor_;
  std::thread plugin_spin_thread_;

  ManipulationManager manager_;
  std::shared_ptr<JtacExecutionAdapter> execution_adapter_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valve_pub_;
  ValveGate valve_;
  std::shared_ptr<manipulation_manager::ToolFeedExecutionPort> tool_feed_;   // null unless tool_actuation_feed
  bool contact_{true};
  double spray_extra_standoff_{0.0};

  std::string frame_id_;
  std::string world_frame_;
  double stand_service_timeout_s_{60.0};
  bool seed_start_from_live_state_{false};
  std::string arm_base_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string planning_group_;
  std::string coverage_json_path_;
  double z_offset_{0.0};
  WallDirection wall_dir_;
  bool dock_at_stroke_start_{false};
  double coverage_z_step_{0.1};
  double coverage_z_min_{1.20};
  double coverage_z_max_{1.60};
  double coverage_eef_speed_{0.02};
  double dock_standoff_{0.0};
  int dock_attempts_{3};
  bool dock_ik_seeded_{true};
  bool dock_ik_continuity_{false};
  std::vector<double> profile_joint_costs_;   ///< the profile's `joint_costs`; empty = none given
  DockIkSelectorParams dock_ik_sel_;
  bool recovery_keeps_branch_{false};
  bool verify_approach_{true};
  bool verify_dock_{true};
  double tcp_settle_timeout_s_{0.0};
  double tcp_settle_tolerance_m_{0.002};
  double dock_tolerance_m_{0.02};
  double dock_tolerance_rad_{0.17};
  double approach_tolerance_m_{0.05};
  std::vector<double> dock_seed_;
  std::vector<double> dock_branch_seed_;   ///< the last docked configuration: the IK seed of the next dock
  double travel_lift_margin_{0.01};
  double dock_approach_extra_m_{0.0};
  bool chain_columns_{true};
  double chain_max_shift_m_{0.4};
  bool serpentine_columns_{true};
  bool contact_connectors_{true};
  ConnectorConfig connector_cfg_;
  ConnectorLog connector_log_;
  const CoverageSegmentIntent * next_in_stand_{nullptr};   ///< the next column of the stand being run, if any
  size_t next_in_stand_id_{0};
  bool connector_carry_{false};            ///< the tool slid onto the start of connector_carry_column_ and is in contact
  size_t connector_carry_column_{0};
  bool chain_valid_{false};                 ///< the tool is at chain_last_standoff_ after a finished column of this stand
  core_types::Pose chain_last_standoff_;
  std::string base_check_mode_{"off"};
  std::string base_check_service_;
  double base_check_lift_min_{0.0};
  double base_check_lift_max_{0.0};
  double base_check_arm_mount_z_{0.0};
  double base_check_tol_xy_{0.15};
  double base_check_tol_yaw_{0.1};
  std::string column_reach_map_file_;
  double column_reach_min_score_{0.15};
  double column_reach_count_ref_{8.0};
  double column_reach_lift_step_{0.05};
  double column_reach_press_depth_{-1.0};
  std::unique_ptr<ReachabilityColumnPort> column_reach_;
  double contact_y_overshoot_{0.0};
  double max_wall_penetration_{0.0};
  double target_force_n_{0.0};
  std::string controller_name_{"combined_arm_lift_controller"};
  double press_settle_s_{2.0};
  double unload_m_{0.02};
  double unload_release_force_n_{2.0};
  double unload_release_speed_mps_{0.001};
  double unload_timeout_s_{4.0};
  double force_now_{-1.0};   // latest conditioned force magnitude, -1 until one arrived
};

}  // namespace manipulation_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation_manager::ManipulationManagerNode>();
  if (!node->init()) {
    RCLCPP_ERROR(node->get_logger(), "Initialization failed, shutting down.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for 'move_group' node to start...");
  bool move_group_ready = false;
  while (rclcpp::ok()) {
    for (const auto & name : node->get_node_names()) {
      if (name.find("move_group") != std::string::npos) {
        move_group_ready = true;
        break;
      }
    }
    if (move_group_ready) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (move_group_ready && rclcpp::ok()) {
    if (node->runOnStart()) {
      node->runCoveragePlan();
    } else {
      node->serve();
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "Shutdown requested before move_group became ready.");
  }

  rclcpp::shutdown();
  return 0;
}
