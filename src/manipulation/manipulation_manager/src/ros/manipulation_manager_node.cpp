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

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "reachability_msgs/srv/plan_stands.hpp"
#include "std_msgs/msg/float64.hpp"

#include "core_types/pose.hpp"
#include "motion_planner/core/motion_planner.hpp"
#include "motion_planner/core/planning_pipeline.hpp"
#include "motion_planner/ros/planning_config.hpp"
#include "mobile_manipulator_profile/profile.hpp"
#include "ros_adapters/rclcpp_logger.hpp"

#include "manipulation_manager/core/base_stance_check.hpp"
#include "manipulation_manager/core/manipulation_manager.hpp"
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
const std::vector<std::string> kGroupJointOrder = {
  "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

std::vector<double> readCurrentJointState(const rclcpp::Node::SharedPtr & node)
{
  std::map<std::string, double> latest;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&latest](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        latest[msg->name[i]] = msg->position[i];
      }
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && latest.size() < kGroupJointOrder.size() &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::vector<double> result;
  for (const auto & name : kGroupJointOrder) {
    result.push_back(latest.count(name) ? latest.at(name) : 0.0);
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
    contact_y_overshoot_ = this->declare_parameter<double>("contact_y_overshoot_m", 0.045);
    max_wall_penetration_ = this->declare_parameter<double>("max_wall_penetration_m", 0.12);
    target_force_n_ = this->declare_parameter<double>("target_force_n", 5.0);
    press_settle_s_ = this->declare_parameter<double>("press_settle_s", 2.0);
    unload_m_ = this->declare_parameter<double>("unload_m", 0.0);

    // Base stance check against the reachability planner: "off" (default), "warn" (log and go on)
    // or "enforce" (do not run the plan from a stance the planner says cannot reach it).
    base_check_mode_ = this->declare_parameter<std::string>("base_check", "off");
    base_check_service_ = this->declare_parameter<std::string>(
      "base_check_service", "/reachability_node/plan_stands");
    base_check_lift_min_ = this->declare_parameter<double>("base_check_lift_min_m", 0.0);
    base_check_lift_max_ = this->declare_parameter<double>("base_check_lift_max_m", 0.0);
    base_check_arm_mount_z_ = this->declare_parameter<double>("base_check_arm_mount_z_m", 0.0);
    base_check_tol_xy_ = this->declare_parameter<double>("base_check_tol_xy_m", 0.15);
    base_check_tol_yaw_ = this->declare_parameter<double>("base_check_tol_yaw_rad", 0.1);

    // Which planning backends to use: a YAML selection (see docs/backend_selection.md), the
    // packaged default when empty. The planners are built in init().
    planning_config_ = this->declare_parameter<std::string>("planning_config", "");
    manager_.setLogger(std::make_shared<ros_adapters::RclcppLogger>(this->get_logger()));

    force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/combined_arm_lift_controller/target_force_n", 10);
  }

  /// Two-phase init: JtacExecutionAdapter needs a live node shared_ptr for its action client,
  /// which isn't available yet inside the constructor above (shared_from_this() requires the
  /// object to already be owned by a shared_ptr) -- call once right after make_shared().
  bool init()
  {
    if (!buildPlanners()) {
      return false;
    }
    execution_adapter_ = std::make_shared<JtacExecutionAdapter>(shared_from_this());
    manager_.setExecutionPort(execution_adapter_);

    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      shared_from_this(), "/combined_arm_lift_controller");
    if (!param_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "combined_arm_lift_controller param service unavailable");
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

  void setAdmittanceEngaged(bool enabled)
  {
    std::vector<bool> axes = {false, false, enabled, false, false, false};
    auto future = param_client_->set_parameters({rclcpp::Parameter("admittance.selected_axes",
      axes)});
    rclcpp::spin_until_future_complete(shared_from_this(), future);
  }

  /**
   * @brief Run the whole coverage plan from this stance: dock -> contact -> retract per wall
   *        column, in the column order CoverageOutputJsonSource already sorted left-to-right.
   */
  void runCoveragePlan()
  {
    CoverageOutputJsonSource source(
      coverage_json_path_, frame_id_, z_offset_, coverage_z_step_, coverage_z_min_, coverage_z_max_,
      coverage_eef_speed_, wall_dir_);
    const auto intents = source.getCoveragePlan();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu wall column(s) from %s",
      intents.size(), coverage_json_path_.c_str());

    if (!baseStanceAllowsRun(intents)) {
      return;
    }

    primeContinuityFromLiveState();

    size_t columns_done = 0;
    size_t columns_partial = 0;
    size_t columns_failed = 0;
    for (const auto & intent : intents) {
      if (intent.poses.empty()) {
        continue;
      }
      const auto outcome = runColumn(intent);
      if (outcome == ColumnOutcome::kDone) {
        ++columns_done;
      } else if (outcome == ColumnOutcome::kPartial) {
        ++columns_partial;
      } else {
        ++columns_failed;
      }
    }
    RCLCPP_INFO(
      this->get_logger(), "Coverage plan complete: done=%zu partial=%zu failed=%zu",
      columns_done, columns_partial, columns_failed);
  }

private:
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
    const auto verdict = evaluateStance(stands, res->unreachable.size(), res->not_planned.size(), tol);
    if (verdict.ok) {
      RCLCPP_INFO(this->get_logger(), "base stance check: %s", verdict.message.c_str());
      return true;
    }
    return fail(verdict.message);
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

  enum class ColumnOutcome { kDone, kPartial, kFailed };

  /// A trivial joint-space "move to where the arm already is" segment -- its only purpose is
  /// getting a genuinely live joint state into ManipulationManager's own continuity tracking
  /// before the first real segment, see this file's own readCurrentJointState() comment for why.
  void primeContinuityFromLiveState()
  {
    const auto live_state = readCurrentJointState(shared_from_this());
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

  ColumnOutcome runColumn(const CoverageSegmentIntent & intent)
  {
    const auto & rep = intent.poses.front();
    // Docking in the middle of the column is the original behaviour; at the stroke's first pose the
    // contact path starts from the arm configuration it docked in (no IK branch jump at the start).
    const double dock_z = dock_at_stroke_start_ ? intent.poses.front().z
                                                : intent.poses[intent.poses.size() / 2].z;

    core_types::Pose standoff_pose = rep;
    standoff_pose.z = dock_z;
    standoff_pose = wall_dir_.shifted(standoff_pose, -dock_standoff_);
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
    for (int attempt = 0; attempt < 3; ++attempt) {
      dock_exec = manager_.executeMotionSegments(
        frame_id_, {dock_seg}, {standoff_pose}, planning_group_);
      if (dock_exec.success) {
        break;
      }
      RCLCPP_WARN(
        this->get_logger(), "%s: dock attempt %d failed, retrying...", intent.label.c_str(),
        attempt + 1);
    }
    if (!dock_exec.success) {
      RCLCPP_WARN(this->get_logger(), "%s: dock failed, skipping column", intent.label.c_str());
      return ColumnOutcome::kFailed;
    }

    setAdmittanceEngaged(true);
    {
      std_msgs::msg::Float64 msg;
      msg.data = target_force_n_;
      force_pub_->publish(msg);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

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
    const auto wall_q = wall_dir_.outwardNormalQuaternion();
    contact_seg.wall_reference_pose.qx = wall_q[0];
    contact_seg.wall_reference_pose.qy = wall_q[1];
    contact_seg.wall_reference_pose.qz = wall_q[2];
    contact_seg.wall_reference_pose.qw = wall_q[3];

    std::vector<core_types::Pose> contact_waypoints;
    contact_waypoints.reserve(intent.poses.size());
    for (size_t i = 0; i < intent.poses.size(); ++i) {
      core_types::Pose p = wall_dir_.shifted(intent.poses[i], contact_y_overshoot_);
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
        std::this_thread::sleep_for(std::chrono::duration<double>(press_settle_s_));
      }
    }
    for (size_t i = 0; i < contact_waypoints.size(); ++i) {
      contact_seg.waypoint_indices.push_back(i);
    }
    const auto contact_exec = manager_.executeMotionSegments(
      frame_id_, {contact_seg}, contact_waypoints, planning_group_);
    RCLCPP_INFO(
      this->get_logger(), "%s: contact success=%d completed=%zu", intent.label.c_str(),
      contact_exec.success, contact_exec.waypoints_completed);

    // Back the reference off the wall while the admittance is still engaged: switching it off with the
    // reference 1.5 mm inside the wall snaps the arm onto that reference (27-31 N spikes at every
    // stroke end). The admittance releases its offset against a reference that no longer presses.
    if (unload_m_ > 0.0 && !contact_waypoints.empty()) {
      MotionSegmentSpec unload_seg = contact_seg;
      unload_seg.label = intent.label + "_unload";
      unload_seg.waypoint_indices = {0};
      unload_seg.allow_target_contact = false;
      unload_seg.min_path_fraction = 0.5;
      const auto unload_pose = wall_dir_.shifted(contact_waypoints.back(), -(contact_y_overshoot_ + unload_m_));
      manager_.executeMotionSegments(frame_id_, {unload_seg}, {unload_pose}, planning_group_);
    }
    // Ramp the force target to zero and let the admittance unload the wall before it is switched off.
    {
      std_msgs::msg::Float64 zero;
      zero.data = 0.0;
      force_pub_->publish(zero);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    setAdmittanceEngaged(false);

    MotionSegmentSpec retract_seg;
    retract_seg.label = intent.label + "_retract";
    retract_seg.mode_hint = "fsp";
    retract_seg.waypoint_indices = {0};
    retract_seg.min_path_fraction = 0.99;  // see dock_seg's own comment above for why
    const auto retract_exec = manager_.executeMotionSegments(
      frame_id_, {retract_seg}, {standoff_pose}, planning_group_);
    if (!retract_exec.success) {
      RCLCPP_WARN(this->get_logger(), "%s: retract failed", intent.label.c_str());
    }

    if (!contact_exec.success || contact_exec.waypoints_completed == 0) {
      return ColumnOutcome::kFailed;
    }
    return contact_exec.waypoints_completed == intent.poses.size()
      ? ColumnOutcome::kDone : ColumnOutcome::kPartial;
  }

  // Plugin plumbing. Declared before manager_ so the manager (which owns the planners) is
  // destroyed first; the registry's plugin libraries stay loaded for the whole process.
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

  std::string frame_id_;
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
    node->runCoveragePlan();
  } else {
    RCLCPP_WARN(node->get_logger(), "Shutdown requested before move_group became ready.");
  }

  rclcpp::shutdown();
  return 0;
}
