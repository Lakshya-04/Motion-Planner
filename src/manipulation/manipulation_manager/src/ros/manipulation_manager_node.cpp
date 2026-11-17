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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <functional>
#include <map>
#include <memory>
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
#include "stand_navigator_msgs/action/navigate_to_stand.hpp"
#include "reachability_msgs/srv/plan_stands.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "core_types/pose.hpp"
#include "motion_planner/core/motion_planner.hpp"
#include "motion_planner/core/planning_pipeline.hpp"
#include "motion_planner/ros/planning_config.hpp"
#include "mobile_manipulator_profile/profile.hpp"
#include "ros_adapters/rclcpp_logger.hpp"

#include "manipulation_manager/ros/execution/retiming_wiring.hpp"
#include "manipulation_manager/core/base_stance_check.hpp"
#include "manipulation_manager/core/coverage_task.hpp"
#include "manipulation_manager/core/column_ordering.hpp"
#include "manipulation_manager/core/frame_transform.hpp"
#include "manipulation_manager/core/manipulation_manager.hpp"
#include "manipulation_manager/core/mission_fsm.hpp"
#include "manipulation_manager/core/mission_ledger.hpp"
#include "manipulation_manager/core/recovery_jog.hpp"
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
  const rclcpp::Node::SharedPtr & node, const std::vector<std::string> & configured_names,
  std::vector<std::string> * names_out = nullptr)
{
  std::map<std::string, double> latest;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&latest](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        latest[msg->name[i]] = msg->position[i];
      }
    });
  std::vector<std::vector<std::string>> candidates;
  if (!configured_names.empty()) {
    candidates.push_back(configured_names);
  } else {
    candidates = kJointNameCandidates;
  }
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
    // The free-space move of a dock stops this far short of the dock pose and a short straight move closes the
    // gap (0 = plan straight to the dock pose). Planning to a pose a few cm from the wall fails or times out
    // when the tool mount clips the wall voxels on the way in; see docs/sim_mujoco.md.
    dock_approach_extra_m_ = this->declare_parameter<double>("dock_approach_extra_m", 0.0);
    contact_y_overshoot_ = this->declare_parameter<double>("contact_y_overshoot_m", 0.045);
    max_wall_penetration_ = this->declare_parameter<double>("max_wall_penetration_m", 0.12);
    target_force_n_ = this->declare_parameter<double>("target_force_n", 5.0);
    press_settle_s_ = this->declare_parameter<double>("press_settle_s", 2.0);
    unload_m_ = this->declare_parameter<double>("unload_m", 0.0);
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
    base_check_lift_min_ = this->declare_parameter<double>("base_check_lift_min_m", 0.0);
    base_check_lift_max_ = this->declare_parameter<double>("base_check_lift_max_m", 0.0);
    // The stow pose is kept this far inside the lift's travel (base_check_lift_min_m .. max_m): a lift sent to its
    // end stop settles beyond it and MoveIt then refuses every plan ("start state out of bounds"). 0 = as is.
    travel_lift_margin_ = this->declare_parameter<double>("travel_lift_margin_m", 0.01);
    base_check_arm_mount_z_ = this->declare_parameter<double>("base_check_arm_mount_z_m", 0.0);
    base_check_tol_xy_ = this->declare_parameter<double>("base_check_tol_xy_m", 0.15);
    base_check_tol_yaw_ = this->declare_parameter<double>("base_check_tol_yaw_rad", 0.1);

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
    // A shell command run after each arrival at a stand, before its columns (empty = none).
    arrival_command_ = this->declare_parameter<std::string>("stand_arrival_command", "");
    // A shell command that takes the wall out of the planning scene. With it (and the arrival command, which
    // puts the wall back) a failed retract or stow is retried once without the wall: a tool left in the
    // wall box has a start state in collision, and no planner will move it out of one.
    wall_clear_command_ = this->declare_parameter<std::string>("wall_clear_command", "");
    // Failure handling. A column that failed (dock or contact stroke did nothing) is retried this many times, then
    // skipped and the run goes on; the result lists it under columns_failed.
    column_retries_ = std::max<int>(0, this->declare_parameter<int>("column_retries", 1));
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
    travel_joint_state_ = this->declare_parameter<std::vector<double>>(
      "travel_joint_state", std::vector<double>{});
    // The planning group's joints in the group's order as /joint_states names them (empty = the repo's UR10e and
    // lift names, then the earlier arm's), used to read the live arm state.
    group_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "group_joint_names", std::vector<std::string>{});
    // Where the tool's force is read for the feedback's and result's peak ("" = not read, peak -1).
    const auto force_topic = this->declare_parameter<std::string>(
      "force_topic", "/ft_conditioner/wrench_conditioned");
    if (!force_topic.empty()) {
      force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        force_topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          if (!tracking_peak_) {
            return;
          }
          const auto & f = msg->wrench.force;
          const double mag = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
          peak_force_ = std::max(peak_force_, mag);
          column_peak_ = std::max(column_peak_, mag);
        });
    }
    // Mission ledger ("" = off) and the contact-overload limit (0 = off; above it a column is skipped, not retried).
    ledger_ = MissionLedger(this->declare_parameter<std::string>("ledger_path", ""));
    contact_overload_n_ = this->declare_parameter<double>("contact_overload_n", 0.0);
    mission_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "mission_command", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "pause") {
          RCLCPP_INFO(this->get_logger(), "mission: pause %s", mission_fsm_.requestPause() ? "requested" : "ignored");
        } else if (msg->data == "resume") {
          RCLCPP_INFO(this->get_logger(), "mission: resume %s", mission_fsm_.resume() ? "accepted" : "ignored");
        }
      });
    nav_client_ = rclcpp_action::create_client<NavigateToStand>(this, "navigate_to_stand");
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
    if (!world_frame_.empty()) {
      // its own spin thread: the node itself is only spun in bursts while a request waits
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), true);
    }
    execution_adapter_ = std::make_shared<JtacExecutionAdapter>(
      shared_from_this(), "/" + controller_name_ + "/follow_joint_trajectory");
    manager_.setExecutionPort(wrapWithRetiming(shared_from_this(), execution_adapter_, planning_group_));
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
    if (!valve_.update(open)) {
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
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCoverage>;
  using NavigateToStand = stand_navigator_msgs::action::NavigateToStand;

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

  void setState(const std::string & state, int column = -2)
  {
    state_ = state;
    if (column != -2) {
      column_ = column;
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

  /// Runs one task: loads the plan, then either the whole plan from here or, multi-stand, the stands in order.
  TaskResult executeTask(const TaskRequest & req)
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

    TaskStateMachine machine(nav_attempts_);
    if (!req.multi_stand) {
      machine.startSingle();
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
      last_good_state_ = readCurrentJointState(shared_from_this(), group_joint_names_);
      for (const size_t i : orderColumns(all, sel)) {
        if (stop()) {
          break;
        }
        tracker_.columnFinished(0, ids[i], runColumnWithRetry(sel[i], ids[i]));
      }
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
      travel_joint_state_ : readCurrentJointState(shared_from_this(), group_joint_names_);
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

    while (!machine.terminal()) {
      if (stop()) {
        break;
      }
      const size_t s = machine.stand();
      stand_index_ = s;
      if (machine.state() == TaskState::kNavigating) {
        std::string msg;
        bool ok = true;
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
        }
        if (stop()) {
          break;
        }
        const auto verdict = machine.navigationResult(ok);
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
        primeContinuityFromLiveState();
        last_good_state_ = travel_state_;   // the stowed arm: where a jog goes back to
        arm_stuck_ = false;
        for (const size_t c : orderColumns(visit_columns[s], sel)) {
          if (stop()) {
            break;
          }
          arm_deployed_ = true;
          tracker_.columnFinished(s, ids[c], runColumnWithRetry(sel[c], ids[c]));
        }
        if (!stop()) {
          machine.standFinished();
        }
      } else {
        break;
      }
    }
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
          shared_from_this(), future, std::chrono::seconds(60)) != rclcpp::FutureReturnCode::SUCCESS)
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
    RCLCPP_INFO(
      this->get_logger(), "arrived at the stand (error %.1f mm, %.3f rad)",
      outcome->result->position_error_m * 1000.0, outcome->result->yaw_error_rad);
    return true;
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
    req->lift_min = base_check_lift_min_;
    req->lift_max = base_check_lift_max_;
    req->arm_mount_offset_z = base_check_arm_mount_z_;
    return req;
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
    RCLCPP_INFO(
      this->get_logger(), "planning backends from %s: cartesian=%s free_space=%s ik=%s collision=%s",
      config_path.c_str(), selection.cartesian.c_str(), selection.free_space.c_str(),
      selection.ik.c_str(), selection.collision.c_str());
    return true;
  }

  /// A trivial joint-space "move to where the arm already is" segment -- its only purpose is
  /// getting a genuinely live joint state into ManipulationManager's own continuity tracking
  /// before the first real segment, see this file's own readCurrentJointState() comment for why.
  void primeContinuityFromLiveState()
  {
    const auto live_state = readCurrentJointState(shared_from_this(), group_joint_names_);
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

  /// Remembers where the arm is as the configuration to jog back to.
  void markGoodState()
  {
    const auto live = readCurrentJointState(shared_from_this(), group_joint_names_);
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
    if (!recovery_jog_ || last_good_state_.empty() || stop()) {
      return false;
    }
    std::vector<std::string> names;
    const auto live = readCurrentJointState(shared_from_this(), group_joint_names_, &names);
    if (live.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s: recovery jog skipped, no live joint state", what);
      return false;
    }
    const auto jog = planRecoveryJog(live, last_good_state_, jog_params_);
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

  ColumnOutcome runColumn(const CoverageSegmentIntent & world_intent, size_t column)
  {
    setState("docking", static_cast<int>(column));
    // The column as planned in world_frame, expressed in frame_id for where the base is now. The
    // wall direction is rotated with it (the base may have stopped at another yaw).
    CoverageSegmentIntent intent = world_intent;
    WallDirection wall_dir = wall_dir_;
    if (!world_frame_.empty()) {
      FrameTransform base_in_world;
      if (!lookup(world_frame_, frame_id_, base_in_world)) {
        RCLCPP_ERROR(this->get_logger(), "%s: no base pose, skipping column", intent.label.c_str());
        return ColumnOutcome::kFailed;
      }
      const auto world_in_base = base_in_world.inverse();
      intent = transformIntent(world_in_base, world_intent);
      wall_dir = world_in_base.apply(wall_dir_);
      RCLCPP_INFO(
        this->get_logger(), "%s: base at (%.3f, %.3f) yaw %.3f in %s; column starts at (%.3f, %.3f, %.3f) in %s",
        intent.label.c_str(), base_in_world.t[0], base_in_world.t[1], base_in_world.yaw(),
        world_frame_.c_str(), intent.poses.front().x, intent.poses.front().y, intent.poses.front().z,
        frame_id_.c_str());
    }
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
    const auto dock_round = [&]() {
    for (int attempt = 0; attempt < dock_attempts_ && !stop(); ++attempt) {
      if (dock_approach_extra_m_ > 0.0) {
        MotionSegmentSpec approach_seg = dock_seg;
        approach_seg.label = intent.label + "_approach";
        const auto approach_exec = manager_.executeMotionSegments(
          frame_id_, {approach_seg}, {wall_dir.approachPose(standoff_pose, dock_approach_extra_m_)},
          planning_group_);
        if (!approach_exec.success) {
          RCLCPP_WARN(
            this->get_logger(), "%s: approach attempt %d failed, retrying...", intent.label.c_str(),
            attempt + 1);
          continue;
        }
      }
      dock_exec = manager_.executeMotionSegments(
        frame_id_, {dock_seg}, {standoff_pose}, planning_group_);
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
    markGoodState();
    if (!contact_) {
      return runSprayStroke(intent, wall_dir, standoff_pose);
    }

    setAdmittanceEngaged(true);
    {
      std_msgs::msg::Float64 msg;
      msg.data = target_force_n_;
      force_pub_->publish(msg);
    }
    if (!waitFor(0.5)) {
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
    if (press_settle_s_ > 0.0 && !contact_waypoints.empty()) {
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
    waitFor(1.5, false);
    setAdmittanceEngaged(false);

    setState("retracting");
    MotionSegmentSpec retract_seg;
    retract_seg.label = intent.label + "_retract";
    retract_seg.mode_hint = "fsp";
    retract_seg.waypoint_indices = {0};
    retract_seg.min_path_fraction = 0.99;  // see dock_seg's own comment above for why
    const auto retract_exec = manager_.executeMotionSegments(
      frame_id_, {retract_seg}, {standoff_pose}, planning_group_);
    if (!retract_exec.success) {
      RCLCPP_WARN(this->get_logger(), "%s: retract failed", intent.label.c_str());
      const auto retract_again = [&]() {
          for (int attempt = 0; attempt < dock_attempts_ && !stop(); ++attempt) {
            if (manager_.executeMotionSegments(
                frame_id_, {retract_seg}, {standoff_pose}, planning_group_).success)
            {
              return true;
            }
          }
          return false;
        };
      bool freed = false;
      if (!stop() && recoverArm("retract")) {
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
    const auto stroke_exec = manager_.executeMotionSegments(
      frame_id_, {stroke_seg}, waypoints, planning_group_);
    setValve(false);
    RCLCPP_INFO(
      this->get_logger(), "%s: spray success=%d completed=%zu", intent.label.c_str(),
      stroke_exec.success, stroke_exec.waypoints_completed);
    if (stop()) {
      return ColumnOutcome::kFailed;
    }
    setState("retracting");

    MotionSegmentSpec retract_seg;
    retract_seg.label = intent.label + "_retract";
    retract_seg.mode_hint = "fsp";
    retract_seg.waypoint_indices = {0};
    retract_seg.min_path_fraction = 0.99;
    const auto retract_exec = manager_.executeMotionSegments(
      frame_id_, {retract_seg}, {standoff_pose}, planning_group_);
    if (!retract_exec.success) {
      RCLCPP_WARN(this->get_logger(), "%s: retract failed", intent.label.c_str());
      // a start state the planners refuse: jog it free, then try the retract again
      if (!stop() && recoverArm("retract") &&
        manager_.executeMotionSegments(
          frame_id_, {retract_seg}, {standoff_pose}, planning_group_).success)
      {
        arm_stuck_ = false;
      } else {
        arm_stuck_ = true;
      }
    } else {
      arm_stuck_ = false;
    }

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
  double stand_max_span_{0.0};
  std::string arrival_command_;
  std::string wall_clear_command_;
  double nav_timeout_s_{240.0};
  std::vector<double> travel_joint_state_;
  std::vector<std::string> group_joint_names_;
  std::vector<double> travel_state_;
  std::vector<double> last_good_state_;   ///< the last arm configuration known to plan from (recovery target)
  bool arm_stuck_{false};                 ///< a retract or stow failed and no recovery got the arm out yet
  int column_retries_{1};
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
  SkipReason column_reason_{SkipReason::kPlanFailed};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
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
  bool contact_{true};
  double spray_extra_standoff_{0.0};

  std::string frame_id_;
  std::string world_frame_;
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
  double travel_lift_margin_{0.01};
  double dock_approach_extra_m_{0.0};
  std::string base_check_mode_{"off"};
  std::string base_check_service_;
  double base_check_lift_min_{0.0};
  double base_check_lift_max_{0.0};
  double base_check_arm_mount_z_{0.0};
  double base_check_tol_xy_{0.15};
  double base_check_tol_yaw_{0.1};
  double contact_y_overshoot_{0.0};
  double max_wall_penetration_{0.0};
  double target_force_n_{0.0};
  std::string controller_name_{"combined_arm_lift_controller"};
  double press_settle_s_{2.0};
  double unload_m_{0.0};
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
