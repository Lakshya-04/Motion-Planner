/**
 * @file jtac_engage_disengage_test_node.cpp
 * @brief Engage/disengage test: drives the REAL force_core::ForceStageSequencer through a full APPROACH ->
 *        CARTESIAN -> RETRACT group against JtacControllerCompliancePort, talking over ROS to an
 *        ALREADY-RUNNING JtacController instance (jtac_controller_test.launch.py) -- the
 *        production integration the raw-trajectory test left as a "not yet built" item. Also covers
 *        protectiveStop()
 *        cancelling an in-flight FollowJointTrajectory goal, not just unloading the target force.
 * @details the sequencer-admittance test proved the SAME sequencer against AdmittanceRuleCompliancePort's in-process
 *          AdmittanceRule (a synthetic chain, no controller_manager). This proves the SAME
 *          sequencing logic against the REAL, deployed controller from a SEPARATE process, using
 *          REAL wall-clock dwell timing (ForceStageSequencer's own default now_ns_fn/sleep_fn --
 *          std::chrono::steady_clock + std::this_thread::sleep_for -- not the virtual/injected
 *          clock the sequencer-admittance test used), and REAL FollowJointTrajectory goals sent over the actual
 *          action server for each segment's execute() step.
 *
 *          Contact is scripted, not derived from a real environment model -- same reasoning as
 *          the sequencer-admittance test's own doc: publishing directly to ft_bridge_hardware's injection topic
 *          simulates "the wall pushed back with this much force" without needing a real contact-
 *          dynamics model, which is exactly what a proper wall-contact model is scoped to build properly later.
 *
 *          Run against jtac_controller_test.launch.py (which already starts the FT bridge in
 *          injection_mode and JtacController with admittance.selected_axes starting all-false --
 *          this test's whole point is proving engage() is what turns compliance on, not a
 *          startup default).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "compliant_trajectory_controller/jtac_controller_compliance_port.hpp"
#include "core_types/tool_profile.hpp"
#include "force_core/force_stage_sequencer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{

using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;

const std::vector<std::string> kJointNames{
  "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

// Same validated pose the raw-trajectory test (send_test_trajectory_and_inject_wrench.py)
// already proved collision-free, in-bounds, and well-conditioned -- reused, not re-derived.
const std::vector<double> kHomePose{0.40, 0.0, -0.15, -0.15, 0.0, -0.15, 0.0};
const std::vector<double> kApproachPose{0.40, 0.0, -0.35, -0.15, 0.0, -0.15, 0.0};
const std::vector<double> kLateralPose{0.40, 0.0, -0.35, -0.05, 0.0, -0.15, 0.0};

constexpr double kTargetForceN = 6.0;

// Diagnostic gains -- see jtac_controller.yaml's own comment on why these are 4x the admittance integration tests'
// original values (a moving reference erodes the M > K*(zeta*dt)^2 margin those were derived
// against). Kept identical here so this test exercises the SAME, already-validated physics.
constexpr double kMassZ = 8.0;
constexpr double kStiffnessZ = 300.0;
constexpr double kDampingRatioZ = 6.0;
constexpr double kMassXY = 16.0;
constexpr double kStiffnessXY = 1000.0;
constexpr double kDampingRatioXY = 5.59;

double rawDamping(double ratio, double mass, double stiffness)
{
  return ratio * 2.0 * std::sqrt(mass * stiffness);
}

core_types::ToolProfile buildTestToolProfile()
{
  core_types::ToolProfile profile;
  profile.name = "t8_test_tool";
  profile.task.contact = true;
  profile.task.target_force_n = kTargetForceN;
  profile.task.force_band_n = 1.5;
  profile.task.force_zero_threshold_n = 1.0;
  profile.task.settle_timeout_s = 10.0;
  profile.task.overload_abort_n = 40.0;
  profile.task.tangential_torque_abort_nm = 10.0;
  profile.task.unexpected_contact_abort_n = 0.0;

  profile.control.controller = "admittance_chain";
  profile.control.admittance.selected_axes = {false, false, true, false, false, false};
  profile.control.admittance.mass = {kMassXY, kMassXY, kMassZ};
  profile.control.admittance.stiffness = {kStiffnessXY, kStiffnessXY, kStiffnessZ};
  profile.control.admittance.damping = {
    rawDamping(kDampingRatioXY, kMassXY, kStiffnessXY),
    rawDamping(kDampingRatioXY, kMassXY, kStiffnessXY),
    rawDamping(kDampingRatioZ, kMassZ, kStiffnessZ)};
  profile.control.admittance.wrench_reference_z_n = kTargetForceN;
  profile.control.admittance.contact_threshold_n = 2.0;
  profile.control.admittance.max_offset_m = 0.05;

  profile.control.ft.lpf_cutoff_hz = 0.0;
  profile.control.ft.deadband_n = 0.1;
  profile.control.ft.clamp_n = 100.0;
  profile.control.ft.tare_per_goal = false;
  return profile;
}

/// Sends one FollowJointTrajectory goal and blocks until it completes. A thin, focused re-
/// implementation of send_test_trajectory_and_inject_wrench.py's own sendTrajectory() in C++,
/// since this test needs the C++-only ForceStageSequencer/CompliancePort types alongside it.
bool sendTrajectoryAndWait(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<FollowJTrajAction>::SharedPtr & client,
  const std::vector<double> & target_positions, double duration_s)
{
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available.");
    return false;
  }

  FollowJTrajAction::Goal goal;
  goal.trajectory.joint_names = kJointNames;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = target_positions;
  point.velocities.assign(kJointNames.size(), 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
  goal.trajectory.points = {point};

  auto send_goal_future = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, send_goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
    return false;
  }
  const auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected.");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::duration<double>(duration_s + 5.0)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for trajectory result.");
    return false;
  }
  const auto result = result_future.get();
  return result.result->error_code == FollowJTrajAction::Result::SUCCESSFUL;
}

/// Publishes a scripted wrench on ft_bridge_hardware's own injection topic for duration_s,
/// simulating "the wall pushed back with this much force" -- see this file's own doc for why
/// this is scripted rather than derived from a real contact model (same reasoning as the sequencer-admittance test).
///
/// Takes its OWN dedicated node/publisher rather than sharing the caller's -- this runs on a
/// background thread concurrently with the main thread's own spin_until_future_complete() calls
/// (sendTrajectoryAndWait), and rclcpp does not support spinning the same node from two threads
/// at once. A plain publish() needs no spinning at all, so this function never spins anything.
void injectWrenchFor(
  const rclcpp::Node::SharedPtr & injection_node,
  const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr & publisher,
  double force_z, double duration_s, double rate_hz = 50.0,
  std::shared_ptr<std::atomic<bool>> stop_flag = nullptr)
{
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.frame_id = "link_tcp";
  msg.wrench.force.z = force_z;
  const int ticks = static_cast<int>(duration_s * rate_hz);
  for (int i = 0; i < ticks; ++i) {
    if (stop_flag && stop_flag->load()) {
      return;
    }
    msg.header.stamp = injection_node->now();
    publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / rate_hz));
  }
}

#define CHECK(condition, message) \
  do { \
    if (!(condition)) { \
      RCLCPP_ERROR(node->get_logger(), "FAILED: %s", message); \
      rclcpp::shutdown(); \
      return 1; \
    } \
    RCLCPP_INFO(node->get_logger(), "OK: %s", message); \
  } while (0)

/// Same as CHECK, but also prints outcome.failure_reason_code/message on failure -- for
/// checking a force_core::ForceSequenceOutcome specifically, since "proceed == false" alone
/// doesn't say WHY (abort vs. timeout vs. a real bug).
// Parameter deliberately NOT named "message" -- it collides with ForceSequenceOutcome's own
// .message field below, and the preprocessor performs plain textual substitution: it would
// replace the ".message" member access too, turning "_outcome.message.c_str()" into
// "_outcome.<the passed string literal>.c_str()" (hit live as a baffling "expected unqualified-id
// before string constant" at the call site, not here).
#define CHECK_OUTCOME(outcome_expr, check_label) \
  do { \
    const auto & _outcome = (outcome_expr); \
    if (!_outcome.proceed) { \
      RCLCPP_ERROR( \
        node->get_logger(), "FAILED: %s (%s: %s)", check_label, \
        _outcome.failure_reason_code.c_str(), _outcome.message.c_str()); \
      rclcpp::shutdown(); \
      return 1; \
    } \
    RCLCPP_INFO(node->get_logger(), "OK: %s", check_label); \
  } while (0)

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("jtac_engage_disengage_test_node");

  const auto tool_profile = buildTestToolProfile();

  compliant_trajectory_controller::JtacControllerCompliancePort::Options port_options;
  port_options.controller_node_name = "combined_arm_lift_controller";
  compliant_trajectory_controller::JtacControllerCompliancePort port(port_options);

  // Explicit now_ns_fn matching JtacControllerCompliancePort::status().stamp_ns's own ROS
  // wall-clock epoch (node->now(), the same convention JtacController's own update() clock
  // uses) rather than ForceStageSequencer's default std::chrono::steady_clock -- keeping both
  // sides on one epoch by construction, rather than relying on elapsedNs()'s underflow guard
  // (force_stage_fsm.cpp) to paper over a real mismatch if one is ever introduced elsewhere.
  auto now_ns_fn = [&node]() {return static_cast<uint64_t>(node->now().nanoseconds());};
  force_core::ForceStageSequencer sequencer(port, now_ns_fn);

  auto trajectory_client = rclcpp_action::create_client<FollowJTrajAction>(
    node, "combined_arm_lift_controller/follow_joint_trajectory");

  // Dedicated node for wrench injection -- see injectWrenchFor()'s own doc for why this must not
  // share `node` (it runs on a background thread concurrent with `node`'s own spin calls).
  auto injection_node = std::make_shared<rclcpp::Node>("jtac_test_wrench_injector");
  auto injection_publisher = injection_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/ft_bridge_hardware_uf_ft_sensor/injected_wrench", rclcpp::SystemDefaultsQoS());

  // Home first -- known-good starting pose, force-free.
  CHECK(
    sendTrajectoryAndWait(node, trajectory_client, kHomePose, 4.0),
    "moved to home pose");

  // --- APPROACH: engage() (real live parameter set on the running controller), then a real
  //     trajectory toward the (virtual) wall, then scripted contact. ---
  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  auto outcome = sequencer.beforeSegment(approach_intent, tool_profile);
  CHECK_OUTCOME(outcome, "beforeSegment(APPROACH) engaged compliance");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kApproach,
    "FSM entered kApproach");

  CHECK(
    sendTrajectoryAndWait(node, trajectory_client, kApproachPose, 3.0),
    "executed APPROACH trajectory");

  injectWrenchFor(injection_node, injection_publisher, -kTargetForceN, 1.0);

  outcome = sequencer.afterSegment(true);
  CHECK_OUTCOME(outcome, "afterSegment(APPROACH) confirmed contact");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kApproachContactConfirmed,
    "FSM confirmed contact");

  // --- CARTESIAN: dwell (REAL wall-clock time) until force converges, then hold through a
  //     lateral "coverage path" move while re-publishing the same contact wrench. ---
  //
  // Captures injection_node/injection_publisher/stop_flag BY VALUE (its own shared_ptr copies,
  // keeping those objects alive independent of main()'s own locals) and is detached rather than
  // joined on every path -- any CHECK_OUTCOME/CHECK between here and where this test signals
  // hold_stop_flag below returns early on failure, and a still-joinable std::thread whose
  // destructor runs on that unwind calls std::terminate() (hit live). A short-lived detached
  // thread outliving a diagnostic test's early-abort path is an acceptable, disclosed tradeoff
  // for test tooling, not production code. Duration is a generous upper bound (30s), not the
  // real stop signal -- hold_stop_flag is, so the RETRACT section's own 0 N injection below
  // never races against this thread still publishing -kTargetForceN.
  auto hold_stop_flag = std::make_shared<std::atomic<bool>>(false);
  std::thread hold_thread([injection_node, injection_publisher, hold_stop_flag]() {
      injectWrenchFor(injection_node, injection_publisher, -kTargetForceN, 30.0, 50.0,
        hold_stop_flag);
    });
  hold_thread.detach();

  core_types::ForceIntent cartesian_intent;
  cartesian_intent.stage = core_types::ForceStage::kCartesian;
  outcome = sequencer.beforeSegment(cartesian_intent, tool_profile);
  CHECK_OUTCOME(outcome, "beforeSegment(CARTESIAN) settled onto target_force_n");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kCartesianFollow,
    "FSM converged to kCartesianFollow");

  CHECK(
    sendTrajectoryAndWait(node, trajectory_client, kLateralPose, 2.0),
    "executed CARTESIAN (lateral) trajectory while loaded");

  outcome = sequencer.afterSegment(true);
  CHECK_OUTCOME(outcome, "afterSegment(CARTESIAN) accepted");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kCartesianDone,
    "FSM reached kCartesianDone");

  // Stop the hold thread and give its in-flight loop iteration (at most one 1/50s sleep) a
  // moment to actually exit before this test starts publishing a conflicting value -- see
  // hold_thread's own comment on why this is a flag, not a join().
  hold_stop_flag->store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // --- RETRACT: unload to ~0 N, then peel back to home, then disengage(). ---
  injectWrenchFor(injection_node, injection_publisher, 0.0, 1.0);

  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  outcome = sequencer.beforeSegment(retract_intent, tool_profile);
  CHECK_OUTCOME(outcome, "beforeSegment(RETRACT) unloaded");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kRetractFollow,
    "FSM unloaded to kRetractFollow");

  CHECK(
    sendTrajectoryAndWait(node, trajectory_client, kHomePose, 3.0),
    "executed RETRACT (peel-off) trajectory");

  outcome = sequencer.afterSegment(true);
  CHECK_OUTCOME(outcome, "afterSegment(RETRACT) disengaged");
  CHECK(sequencer.state() == force_core::ForceFsmState::kIdle, "FSM returned to kIdle");

  const auto & telemetry = sequencer.telemetry();
  RCLCPP_INFO(
    node->get_logger(), "max_abs_force_at_switch_n = %.3f (bumpless-transfer bound: < 3.0)",
    telemetry.max_abs_force_at_switch_n);
  CHECK(
    telemetry.max_abs_force_at_switch_n < 3.0, "disengaged within the bumpless-transfer bound");
  CHECK(!telemetry.compliance_engaged_at_exit, "compliance is OFF after the full group");

  RCLCPP_INFO(node->get_logger(), "=== T8 PASSED: full engage/disengage cycle verified live ===");

  // --- Backlog item 4: protectiveStop() cancels an in-flight FollowJointTrajectory goal. ---
  // engage() with all axes deselected -- matches disengage()'s own convention (no active spring
  // dynamics on any axis), so this section exercises ONLY the new cancel behavior, not force
  // physics the engage/disengage test above already proved. Only needed because protectiveStop() early-returns
  // "not engaged" otherwise.
  // admittance.mass/stiffness must be within admittance_controller's own validated bounds
  // ([0.0001, 1e6]) even with every axis deselected -- a zero mass/stiffness is rejected at
  // set_parameters() time regardless of selected_axes, so this reuses tool_profile's own
  // already-valid values with selected_axes all false (matches disengage()'s own convention: a
  // deselected axis has no active spring dynamics regardless of its mass/stiffness/damping).
  core_types::ComplianceConfig cancel_test_config;
  cancel_test_config.selected_axes = {false, false, false, false, false, false};
  cancel_test_config.mass = tool_profile.control.admittance.mass;
  cancel_test_config.stiffness = tool_profile.control.admittance.stiffness;
  cancel_test_config.damping = tool_profile.control.admittance.damping;
  std::string engage_error;
  const bool engage_ok = port.engage(cancel_test_config, engage_error);
  if (!engage_ok) {
    RCLCPP_ERROR(node->get_logger(), "engage() error_message: %s", engage_error.c_str());
  }
  CHECK(engage_ok, "re-engaged for protectiveStop() test");

  // A goal whose commanded duration (20s) is far longer than this test should ever wait --
  // proves protectiveStop() actually stops it early, not that it happens to finish on its own.
  FollowJTrajAction::Goal long_goal;
  long_goal.trajectory.joint_names = kJointNames;
  trajectory_msgs::msg::JointTrajectoryPoint long_point;
  long_point.positions = kApproachPose;
  long_point.velocities.assign(kJointNames.size(), 0.0);
  long_point.time_from_start = rclcpp::Duration::from_seconds(20.0);
  long_goal.trajectory.points = {long_point};

  CHECK(
    trajectory_client->wait_for_action_server(std::chrono::seconds(5)),
    "action server available for cancellation test");
  auto long_goal_future = trajectory_client->async_send_goal(long_goal);
  CHECK(
    rclcpp::spin_until_future_complete(node, long_goal_future) ==
    rclcpp::FutureReturnCode::SUCCESS,
    "sent the long-duration goal to be cancelled");
  const auto long_goal_handle = long_goal_future.get();
  CHECK(long_goal_handle != nullptr, "long-duration goal accepted");

  // Give the controller a moment to genuinely start executing before stopping it -- this is a
  // cancellation test, not a rejection-at-accept-time test.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  const auto protective_stop_start = node->now();
  std::string protective_stop_error;
  CHECK(
    port.protectiveStop(protective_stop_error),
    "protectiveStop() called on the in-flight goal");

  auto long_result_future = trajectory_client->async_get_result(long_goal_handle);
  CHECK(
    rclcpp::spin_until_future_complete(node, long_result_future, std::chrono::seconds(5)) ==
    rclcpp::FutureReturnCode::SUCCESS,
    "long-duration goal reached a terminal state after protectiveStop()");
  const auto long_result = long_result_future.get();
  const double elapsed_s = (node->now() - protective_stop_start).seconds();
  RCLCPP_INFO(
    node->get_logger(), "goal reached terminal state %d in %.2fs after protectiveStop() (goal "
    "duration was 20s)", static_cast<int>(long_result.code), elapsed_s);
  CHECK(
    long_result.code == rclcpp_action::ResultCode::CANCELED,
    "goal was actually CANCELED, not left running to completion");
  CHECK(elapsed_s < 5.0, "goal stopped promptly, not merely eventually");

  std::string disengage_error;
  CHECK(port.disengage(disengage_error), "disengaged after protectiveStop() test");

  RCLCPP_INFO(
    node->get_logger(),
    "=== protectiveStop() trajectory-cancellation check PASSED ===");
  rclcpp::shutdown();
  return 0;
}
