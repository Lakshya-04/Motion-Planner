/**
 * @file jtac_controller.hpp
 * @brief A real ros2_control controller_interface::ControllerInterface plugin: fused joint-
 *        trajectory-following + admittance for the 7-DOF arm+lift chain ("JTAC").
 * @details Replaces joint_trajectory_controller/JointTrajectoryController outright for the
 *          arm+lift chain -- this is the (B) "fused" design from
 *          force_control_backend_admittance_vs_diffik.md section 11, chosen over the (A) chained
 *          admittance_controller-under-JTC design specifically because chaining only gives the
 *          arm's 6 joints admittance authority (the lift is claimed directly by JTC's hardware
 *          command_joints and never sees a force error) -- it cannot reproduce the genuine 7-DOF
 *          redundancy split (lift absorbing ~98% of a vertical push) that
 *          test_admittance_rule_integration.cpp (T4a) and AdmittanceRuleCompliancePort (T6)
 *          already proved. This controller runs the SAME real
 *          admittance_controller::AdmittanceRule across all 7 joints instead.
 *
 *          Two upstream ros2_control libraries are reused directly rather than reimplemented,
 *          both confirmed exported/linkable from their installed .so (the same check T4a already
 *          did for AdmittanceRule):
 *            - joint_trajectory_controller::Trajectory -- time-sampling (linear/cubic/quintic
 *              spline, matching stock JTC) of the multi-waypoint JointTrajectory goal into a
 *              per-cycle reference point, which admittance then applies its compliant offset to.
 *            - admittance_controller::AdmittanceRule -- the compliance law itself, unchanged from
 *              T4a/T6, reused via the SAME admittance_controller::ParamListener mechanism so this
 *              controller's own `admittance.*`/`kinematics.*`/`ft_sensor.*` ROS parameters are
 *              exactly admittance_controller's own parameter schema, addressable the normal way
 *              (`ros2 param ... /jtac_controller admittance.mass`) -- not hidden behind an
 *              internal, unaddressable node the way AdmittanceRuleCompliancePort's own throwaway-
 *              node-per-engage() pattern does (correct for that class's in-process-test role, not
 *              for a real deployed controller).
 *
 *          The FollowJointTrajectory action-server plumbing is NOT similarly reusable (it lives as
 *          protected, non-mixin members on JointTrajectoryController itself) and is hand-rolled
 *          here against the same RT-safe handoff pattern JTC itself uses:
 *          realtime_tools::RealtimeServerGoalHandle + RealtimeBuffer, with a non-RT wall timer
 *          draining goal-handle state transitions outside the update() cycle. This keeps
 *          MoveItExecutionAdapter (manipulation_manager) working completely unmodified: it
 *          only ever calls MoveGroupInterface::execute(), which resolves to whatever action server
 *          is named in 7dof_moveit_controllers_combined.yaml -- pointing that at this
 *          controller's name is the entire integration surface.
 *
 *          Per-segment engage()/disengage() (JtacControllerCompliancePort, this package) works
 *          with ZERO new mechanism here: AdmittanceRule::update() already calls its own
 *          apply_parameters_update() every cycle when `enable_parameter_update_without_
 *          reactivation` is true (verified against the installed admittance_rule_impl.hpp), which
 *          picks up a live `ros2 param set` on admittance.mass/stiffness/damping_ratio/
 *          selected_axes/control.frame.id with no reactivation needed. `selected_axes: all false`
 *          is this controller's own startup default -- matching core_types::ForceStage::kNone
 *          (and PickNik's own `admittance_parameters_no_admittance.yaml` preset, confirmed in
 *          their public `moveit_pro_example_ws`) -- so compliance is genuinely off until an
 *          engage() call turns specific axes on. `setTargetForce()` is NOT a controller parameter
 *          at all: it publishes to ft_bridge_hardware's own existing setpoint-bias topic
 *          (`raw - setpoint`), the same "give the zero-seeking law a target" trick
 *          AdmittanceRuleCompliancePort implements locally -- here it's delegated to
 *          already-built, already-tested infrastructure instead of duplicated.
 *
 *          CORRECTION, found live: `setTargetForce()` does NOT publish to ft_bridge_hardware's
 *          setpoint topic after all, despite that being this class's original design (and despite
 *          that bridge component's bias mechanism being real and separately useful). The FT
 *          bridge's bias is applied BEFORE the ros2_control state interfaces are exported, so
 *          EVERY consumer reading those interfaces -- including this controller's own
 *          measured_wrench_publisher_ status topic -- would see the already-biased
 *          "error from target" value, not the raw physical force. force_core::ForceStageFsm
 *          needs the RAW force for its own contact_threshold_n / force_band_n checks (it computes
 *          its OWN error against target_force_n internally); double-applying the bias silently
 *          corrupts contact detection (hit live: APPROACH's contact check never fired, because a
 *          fully-biased reading reads as ~0 N exactly AT the target force, not above threshold on
 *          the way there). Fix: the bias is applied HERE, inside update(), on a local copy fed to
 *          admittance_rule_->update() only -- target_force_n_ (settable via ~/target_force_n,
 *          std_msgs::msg::Float64) never touches the wrench this controller reports as status,
 *          matching AdmittanceRuleCompliancePort's own (correct) precedent. Independently
 *          confirmed as the right layering by upstream ros2_controllers' OWN current (6.9.0,
 *          rolling -- far ahead of Humble's installed 2.52.1) admittance_controller, which added
 *          exactly this as a controller-level `~/wrench_reference` topic, not a hardware/sensor
 *          concept -- a real future option (vendor that version) if this hand-rolled bias ever
 *          needs replacing, tracked as a named next step rather than attempted here.
 *
 *          `measured_wrench_pub_` publishes the RAW per-cycle FT reading (WrenchStamped, frame_id
 *          = ft_sensor.frame.id, pre-bias) so a CompliancePort adapter -- running in a different
 *          process -- has something to build ComplianceStatus::wrench from; the RT state itself
 *          is only visible inside this controller's own update() cycle. CONSTRAINT, not yet
 *          handled: this is only meaningful as "wrench in control_frame" when the deployment sets
 *          ft_sensor.frame.id == control.frame.id (both link_tcp today) -- a real rotation would
 *          be needed if they ever differ.
 *
 *          Path/goal tolerance checking (JTC's own PATH_TOLERANCE_VIOLATED/GOAL_TOLERANCE_VIOLATED)
 *          is still not implemented -- goals run to completion by trajectory time and report
 *          SUCCESSFUL unconditionally. Real hardware tare() and offset-saturation reporting
 *          (ComplianceStatus::normal_offset_m) are also still open -- see
 *          JtacControllerCompliancePort's own header doc for exactly what it does and doesn't
 *          cover.
 *
 *          T11 -- per-joint redundancy weighting (`enable_redundancy_weighting`,
 *          `redundancy_weights`, both plain params on THIS controller's own node, read once at
 *          on_configure(), NOT routed through admittance_controller::ParamListener's own schema):
 *          ported from jtac_compliance_demo_node.cpp's own prototype (tickWallCeilingWeighted())
 *          after it was live-verified there across the full weighting range. Same reason for
 *          existing as a standalone re-implementation rather than a parameter on
 *          AdmittanceRule::update(): that call computes the Cartesian spring law AND converts to
 *          joint space via the UNWEIGHTED convert_cartesian_deltas_to_joint_deltas() in the same
 *          function body, with no seam to swap in a weighted conversion from outside -- see
 *          force_control_backend_admittance_vs_diffik.md's T11 section for the full derivation,
 *          the literature it's drawn from (Whitney 1969, Khatib's dynamically-consistent
 *          generalized inverse, Chan & Dubey 1995), and the live-verified range.
 *
 *          updateWeighted() replicates AdmittanceRule::update()'s OWN spring law faithfully
 *          (including the K/D-rotated-into-base-frame vs. mass-applied-directly-in-base-frame
 *          asymmetry the real implementation has, and the accumulated joint-space OFFSET-onto-
 *          reference convention, NOT the demo's simpler offset-onto-current form -- the demo's
 *          reference pose never moves mid-phase, this controller's does, continuously, from a
 *          real trajectory), replacing only the final Cartesian-to-joint-space step with a
 *          weighted pseudo-inverse. DISCLOSED SCOPE LIMIT, enforced both at configure time and
 *          every update() cycle: translation-only (no rotational admittance.selected_axes
 *          support -- a live engage() selecting rx/ry/rz while this path is active freezes at the
 *          current measured position rather than silently applying zero torque along that axis),
 *          and requires kinematics.tip == control.frame.id == ft_sensor.frame.id (asserted once
 *          at on_configure(); true of every config this repo ships today, all three set to
 *          link_tcp). Owns its own separate kinematics_interface::KinematicsInterface instance
 *          (same duplication jtac_compliance_demo_node.cpp already accepted -- AdmittanceRule's
 *          own instance is a private member, not exposed). Defaults to
 *          enable_redundancy_weighting:=false (byte-identical to before this landed) and, when
 *          enabled, redundancy_weights:=[10,1,1,1,1,1,1] -- the lift weighted 10x each arm joint,
 *          i.e. the arm is kept MORE compliant than the lift. A DELIBERATE, REVERSIBLE design
 *          choice, not a physically-derived number -- see the decision doc for the full
 *          live-verified range this sits inside and swap freely via the same parameter.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "admittance_controller/admittance_rule.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "joint_trajectory_controller/interpolation_methods.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace compliant_trajectory_controller
{

/**
 * @brief The weighted pseudo-inverse J_W+ = W^-1 J^T (J W^-1 J^T)^-1, W = diag(weights).
 * @details Extracted as its own free function (rather than inlined in JtacController::
 *          updateWeighted()) specifically so it is unit-testable in isolation from the rest of
 *          that method's frame-rotation/masking logic, which needs a real kinematics instance to
 *          exercise meaningfully and is verified LIVE instead (see force_control_backend_
 *          admittance_vs_diffik.md's T11 section) -- matching this codebase's own established
 *          convention for logic that is live-verified rather than unit-covered (KNOWN_ISSUES.md
 *          #18 names the same tradeoff for CartesianPlanner). See test/test_redundancy_weighting
 *          .cpp for what IS pinned here: equal weights reduce to the plain Moore-Penrose
 *          pseudo-inverse exactly, and a heavily-weighted column's contribution to the solution
 *          collapses toward zero relative to an unweighted solve on the same input.
 * @param j_lin The 3xN linear-velocity-only Jacobian (translation rows only -- this class's own
 *        header doc names the translation-only scope this is used within).
 * @param weights N per-joint weights, all > 0 -- a larger weight makes that joint contribute LESS
 *        to the Cartesian target for the same input; all-equal reduces to the unweighted case.
 * @return The Nx3 weighted pseudo-inverse.
 */
Eigen::MatrixXd weightedPseudoInverse(
  const Eigen::Matrix<double, 3, Eigen::Dynamic> & j_lin, const Eigen::VectorXd & weights);

class JtacController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  rclcpp_action::GoalResponse goalReceivedCallback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);
  rclcpp_action::CancelResponse goalCancelledCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  void goalAcceptedCallback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  void readStateFromStateInterfaces(trajectory_msgs::msg::JointTrajectoryPoint & state);
  void writeCommandsToCommandInterfaces(
    const trajectory_msgs::msg::JointTrajectoryPoint & command);
  geometry_msgs::msg::Wrench readMeasuredWrench();
  void onTargetForce(const std_msgs::msg::Float64::SharedPtr msg);
  void onMaxOffset(const std_msgs::msg::Float64::SharedPtr msg);

  /// Hard safety clamp: found live (2026-08-21) that ComplianceConfig::max_offset_m -- documented
  /// as "backend must clamp" -- was never actually enforced anywhere. Twice in one session, an
  /// APPROACH that never found real contact (free air within reach of the compliant offset, but
  /// no wall) ran the FULL settle_timeout_s dwell with the offset growing UNBOUNDED (nothing
  /// stopped it), driving the arm into self-collision -- once folding it down toward the AMR base
  /// entirely. Called from update() after EITHER admittance_rule_->update() or updateWeighted()
  /// fills state_desired_: FKs both state_desired_ and state_reference_ via offset_clamp_
  /// kinematics_ (a dedicated, ALWAYS-initialized instance -- unlike redundancy_kinematics_,
  /// which only exists when enable_redundancy_weighting_ is on, this clamp must apply
  /// unconditionally, on whichever path actually ran), and if the Cartesian offset's own norm
  /// exceeds max_offset_m_, scales the JOINT-SPACE offset (state_desired_ - state_reference_)
  /// back to that bound before it is ever written to the command interfaces. A no-op (state_
  /// desired_ left untouched) if the FK call fails or the offset is already within bound.
  void clampCartesianOffset();

  /// Real-actuator rate limiting -- a defense-in-depth safety net, not the primary fix for the
  /// instability below (that fix is tool_profiles.yaml's own admittance.damping values -- see its
  /// own comment there for the actual root cause). Clamps state_desired_ to what the joint's own
  /// real max_velocity/max_acceleration (joint_limits.yaml) actually allow it to reach from
  /// state_current_ in one control period, and falls back to holding at state_current_ (zero
  /// velocity) if either value is non-finite. A real actuator cannot physically teleport
  /// regardless of what its control loop computes; this keeps ANY future misconfiguration of this
  /// same class (an admittance profile re-tuned back into instability, a different tool profile
  /// nobody re-checked) from ever reaching the command interfaces as a NaN/Inf or a physically
  /// impossible jump, without touching the admittance law itself. Called from update() after
  /// clampCartesianOffset(), immediately before writeCommandsToCommandInterfaces().
  void enforceJointRateLimits(const rclcpp::Duration & period);

  /// Loaded once at on_configure() from this node's own `joint_limits.<joint_name>.max_velocity`/
  /// `max_acceleration` parameters (see cartesian_test.launch.py / 7dof_serum_moveit.launch.py
  /// for how robot_description's own joint_limits.yaml gets wired onto this controller's node
  /// under its own name -- the same canonical file MoveIt's own time parameterization already
  /// uses, not a duplicated copy). A joint missing from that file falls back to a generous but
  /// finite default (never literally unbounded) and logs a one-time warning, matching max_offset_m_
  /// 's own established precedent for "safe if a caller forgets to configure this."
  std::vector<double> max_joint_velocity_;
  std::vector<double> max_joint_acceleration_;
  /// The velocity actually written last cycle (post-clamp) -- needed to bound this cycle's
  /// acceleration (Δv/dt), which state_desired_.velocities alone doesn't give us since that's
  /// admittance's own unclamped output, not what previously reached the hardware.
  std::vector<double> previous_command_velocity_;

  /// A SEPARATE instance from admittance_rule_'s own internal kinematics_ (private, not exposed)
  /// and from redundancy_kinematics_ (only initialized when enable_redundancy_weighting_ is on) --
  /// same duplication-for-independence reasoning already established elsewhere in this file: this
  /// safety clamp must work regardless of which update path is active.
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
    offset_clamp_kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> offset_clamp_kinematics_;
  /// Settable live via ~/max_offset_m (JtacControllerCompliancePort::engage() publishes
  /// ComplianceConfig::max_offset_m here, mirroring target_force_n_'s own exact mechanism --
  /// NOT a plain declared parameter, since get_parameter() is not RT-safe to call every cycle).
  /// Default 0.1m: generous enough not to interfere with any test in this repo that already sets
  /// a real value, but finite -- never literally unbounded even if a caller forgets to publish.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr max_offset_subscriber_;
  realtime_tools::RealtimeBuffer<double> max_offset_m_{0.1};

  /// T11 -- see this class's own header doc for the full mechanism and disclosed scope limits.
  /// Fills state_desired_ directly (mirroring AdmittanceRule::update()'s own output convention:
  /// reference + an accumulated joint-space offset), in place of calling admittance_rule_->
  /// update(). Returns false on a kinematics failure or a disallowed rotational-axis selection
  /// (both freeze state_desired_ at a safe value rather than leaving it stale).
  bool updateWeighted(const geometry_msgs::msg::Wrench & biased_wrench,
    const rclcpp::Duration & period);

  // -- Parameters / admittance -- see this file's own doc for why ParamListener/AdmittanceRule
  // are reused directly against get_node() rather than via AdmittanceRuleCompliancePort.
  std::shared_ptr<admittance_controller::ParamListener> param_listener_;
  admittance_controller::Params params_;
  std::unique_ptr<admittance_controller::AdmittanceRule> admittance_rule_;
  size_t num_joints_{0};
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;  ///< e.g. ["position", "velocity"].
  std::vector<std::string> state_interface_types_;

  // -- T11: per-joint redundancy weighting -- see this class's own header doc. --
  bool enable_redundancy_weighting_{false};
  std::vector<double> redundancy_weights_;
  /// A SEPARATE instance from admittance_rule_'s own internal kinematics_ (private, not exposed)
  /// -- same duplication jtac_compliance_demo_node.cpp already accepted for the same reason.
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
    redundancy_kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> redundancy_kinematics_;
  /// Persistent joint-space offset integrator state (analogous to AdmittanceRule's own private
  /// admittance_state_.joint_pos/joint_vel) -- zeroed at on_configure(), matching AdmittanceRule's
  /// own reset convention (never reset on activate/deactivate).
  std::vector<double> weighted_joint_pos_offset_;
  std::vector<double> weighted_joint_vel_offset_;

  // -- FT sensor (optional -- zero wrench, i.e. pure trajectory-following, when unset) --
  std::unique_ptr<semantic_components::ForceTorqueSensor> ft_sensor_;
  bool has_ft_sensor_{false};

  /// Publishes the RAW (pre-bias) per-cycle FT reading so a CompliancePort adapter running in a
  /// different process (JtacControllerCompliancePort) has something to build ComplianceStatus::
  /// wrench from -- see this class's own header doc for the frame-matching constraint this
  /// relies on, and for why this must stay pre-bias (never target_force_n_-adjusted).
  using WrenchStatePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr measured_wrench_publisher_;
  std::unique_ptr<WrenchStatePublisher> measured_wrench_rt_publisher_;

  /// Force-setpoint bias -- see this class's own header doc's CORRECTION note for why this lives
  /// here (applied to a local copy fed to AdmittanceRule only) rather than upstream in
  /// ft_bridge_hardware. Settable live via ~/target_force_n; 0.0 (the default) makes
  /// AdmittanceRule's own zero-seeking law behave exactly as before this existed.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_force_subscriber_;
  realtime_tools::RealtimeBuffer<double> target_force_n_{0.0};

  /// Diagnostic-only: [reference_z, desired_z, offset_norm] every clampCartesianOffset() call, in
  /// params_.kinematics.tip's own frame -- added live to chase a repeating contact-force bounce
  /// (see motion_planner/coverage_pass_demo_node.cpp's own header) whose cause wasn't visible
  /// from measured_wrench alone. Not part of this controller's stable public interface.
  using DebugOffsetPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_offset_publisher_;
  std::unique_ptr<DebugOffsetPublisher> debug_offset_rt_publisher_;

  // -- Interface references, populated in on_activate() via controller_interface::
  //    get_ordered_interfaces() -- the same lookup-by-name helper semantic_components use
  //    internally, rather than assuming a fixed positional order in command_interfaces_. --
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;
  bool has_velocity_state_interface_{false};

  // -- Trajectory reference generation (reuses joint_trajectory_controller::Trajectory -- see
  //    this file's own doc for why this needed no reimplementation). --
  std::shared_ptr<joint_trajectory_controller::Trajectory> traj_external_point_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_msg_external_point_ptr_;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> current_active_traj_msg_;
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  /// True on any cycle where update() sampled a live trajectory. Lets the no-active-trajectory
  /// branch freeze state_reference_ exactly once, on the cycle idle begins, instead of every
  /// idle cycle -- see update()'s own doc on why re-deriving it from state_current_ every cycle
  /// is unsound (state_current_ already carries the previous cycle's own compliant offset, so an
  /// every-cycle reset permanently banks it, turning admittance into an unbounded integrator
  /// under any sustained, unconverged force error).
  bool had_active_trajectory_last_cycle_{false};

  // -- Action server: RT-safe handoff pattern matching joint_trajectory_controller's own (see
  //    this file's own doc on why this part is hand-rolled, not reused). --
  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> rt_active_goal_;
  /// The goal handle a terminal transition (setSucceeded/setCanceled) was already sent for.
  /// RealtimeServerGoalHandle::runNonRealtime() resets req_succeed_/req_cancel_ back to false
  /// once it actually calls gh_->succeed()/canceled() -- calling setSucceeded() unconditionally
  /// every update() cycle after completion therefore re-arms and re-fires it every single cycle
  /// forever (hit live: "goal_handle attempted invalid transition from state SUCCEEDED with
  /// event SUCCEED" at 100Hz, which alone stalled the RT loop badly enough to blow up
  /// admittance's own dt-dependent stability bound). Comparing against this pointer makes the
  /// terminal transition fire exactly once per goal.
  RealtimeGoalHandlePtr terminal_goal_handled_;
  std::atomic<bool> rt_has_pending_goal_{false};
  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  const rclcpp::Duration action_monitor_period_{std::chrono::milliseconds(50)};

  // -- Working state (preallocated once num_joints_ is known -- realtime-safe). --
  trajectory_msgs::msg::JointTrajectoryPoint state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint state_reference_;
  trajectory_msgs::msg::JointTrajectoryPoint state_desired_;
};

}  // namespace compliant_trajectory_controller
