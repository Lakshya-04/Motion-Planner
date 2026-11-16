/**
 * @file jtac_controller_compliance_port.hpp
 * @brief A real core_types::CompliancePort implementation that is a ROS CLIENT of a RUNNING
 *        JtacController instance -- the missing piece connecting force_core::
 *        ForceStageSequencer's per-segment engage()/disengage()/setTargetForce() calls to the
 *        actually-deployed controller (the raw-trajectory test), rather than the in-process
 *        AdmittanceRuleCompliancePort (the sequencer-admittance test), which owns its own throwaway AdmittanceRule instance
 *        and cannot reach a controller running in a different process.
 * @details The sequencer and the controller are two separate processes/nodes once JtacController
 *          is real (unlike AdmittanceRuleCompliancePort, which ticks AdmittanceRule in-process) --
 *          this class's whole job is bridging that gap over ordinary ROS interfaces, with NO new
 *          mechanism added to JtacController itself beyond what the raw-trajectory test already built:
 *
 *            - engage()/disengage() call rclcpp::SyncParametersClient::set_parameters() against
 *              the controller's own node (admittance.selected_axes/mass/damping_ratio/stiffness/
 *              control.frame.id) -- picked up live by AdmittanceRule::update()'s own
 *              apply_parameters_update() call (gated on enable_parameter_update_without_
 *              reactivation, which jtac_controller.yaml sets true), so no controller code needed
 *              to react to this. disengage() sets selected_axes to all-false, matching
 *              core_types::ForceStage::kNone's own convention (unselected axes' springs
 *              relax to zero regardless of geometric coupling -- proven in test_admittance_rule_
 *              integration.cpp's ForceOnADeselectedAxisProducesOnlyItsSelectedAxisProjection).
 *            - engage() ALSO publishes ComplianceConfig::max_offset_m on JtacController's own
 *              ~/max_offset_m topic (added 2026-08-21) -- found live that this field was never
 *              enforced anywhere despite being documented "backend must clamp": an APPROACH that
 *              never found contact ran the compliant offset unbounded for the whole settle_
 *              timeout_s dwell, twice causing real self-collision. See JtacController::
 *              clampCartesianOffset()'s own doc comment for the actual enforcement.
 *            - setTargetForce() publishes to JtacController's OWN ~/target_force_n topic (a plain
 *              std_msgs::msg::Float64) -- NOT a controller parameter, and NOT ft_bridge_
 *              hardware's setpoint-bias topic despite that being this class's original design.
 *              Found live: the FT bridge's bias is applied BEFORE ros2_control state interfaces
 *              are exported, so publishing the target there would make JtacController's own
 *              measured_wrench status topic report the ALREADY-BIASED "error from target" value
 *              instead of the raw physical force -- corrupting force_core::ForceStageFsm's
 *              own separate contact_threshold_n/force_band_n checks, which need the raw force
 *              (they compute their own error against target_force_n internally). AdmittanceRule
 *              has no native force setpoint; JtacController applies this bias itself, on a local
 *              copy fed only to AdmittanceRule::update() -- see that class's own header doc.
 *            - status() subscribes to JtacController's own measured_wrench_publisher_ topic (the
 *              raw per-cycle FT reading) on a dedicated background-spun node/executor, matching
 *              ft_bridge_hardware's own established threading pattern for exactly this reason
 *              (ForceStageSequencer's dwell loop calls status() synchronously and spins nothing
 *              itself).
 *
 *          DISCLOSED, NOT HIDDEN, gaps in this first increment:
 *            - tare(): a documented no-op (returns true if not engaged) -- real hardware zeroing
 *              would need a service call to the vendor driver; out of scope for the mock-only
 *              path this class targets today.
 *            - ComplianceStatus::normal_offset_m / offset_saturated: always 0.0/false. Computing
 *              the real Cartesian offset needs either JtacController exposing AdmittanceRule's
 *              own admittance_position (control_msgs::msg::AdmittanceControllerState), or a
 *              frame-rotated computation on this class's own side -- deferred.
 *            - protectiveStop(): unloads the target force (publishes 0 N) AND cancels every
 *              in-flight FollowJointTrajectory goal on the controller's own action server
 *              (async_cancel_all_goals(), fire-and-forget -- matching this method's own
 *              "immediate" characterization, not something a guard trip should block waiting to
 *              confirm). Not the safety function of last resort (CompliancePort's own doc): the
 *              backend's own hardware force/collision limit is. Verified against JtacController's
 *              OWN actual cancel handling (jtac_controller.cpp): on is_canceling(), it freezes the
 *              reference at the CURRENT measured position instantly (not a smoothed decel ramp --
 *              JtacController does not implement one) -- so a cancelled goal stops the arm from
 *              tracking any further commanded motion immediately, with compliance still live.
 *            - FT hygiene (deadband_n/clamp_n from ComplianceConfig): not applied anywhere in this
 *              path -- these fields have no home in admittance_controller OR ft_bridge_
 *              hardware yet, a known gap to close before real hardware.
 *
 *          CONSTRAINT this class relies on rather than works around: ComplianceStatus::wrench is
 *          only meaningful as "in control_frame" when the deployment sets ft_sensor.frame.id ==
 *          control.frame.id (both link_tcp in jtac_controller.yaml today) -- see
 *          JtacController's own header doc for the same note.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <thread>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "core_types/compliance_port.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"

namespace compliant_trajectory_controller
{

class JtacControllerCompliancePort : public core_types::CompliancePort
{
public:
  struct Options
  {
    std::string controller_node_name{"combined_arm_lift_controller"};
    /// How stale a measured_wrench sample may be before status() reports wrench_valid=false --
    /// guards against silently trusting a wrench from a controller that has stopped publishing.
    double wrench_staleness_timeout_s{0.5};
    /// Blocking timeout for the parameter-set round-trip in engage()/disengage().
    double parameter_set_timeout_s{2.0};
  };

  explicit JtacControllerCompliancePort(Options options);
  ~JtacControllerCompliancePort() override;

  bool tare(std::string & error_message) override;
  bool engage(const core_types::ComplianceConfig & config, std::string & error_message)
  override;
  bool setTargetForce(double target_force_n, std::string & error_message) override;
  bool disengage(std::string & error_message) override;
  core_types::ComplianceStatus status() override;
  bool protectiveStop(std::string & error_message) override;

private:
  void onMeasuredWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  bool setSelectedAxes(const std::array<bool, 6> & selected_axes, std::string & error_message);
  void publishTargetForce(double target_force_n);
  void stopExecutor();

  Options options_;
  rclcpp::Node::SharedPtr node_;
  /// A SEPARATE node from node_, used ONLY as SyncParametersClient's target: that class adds
  /// whatever node it's given to its OWN internal executor for the duration of each blocking
  /// call, and rclcpp rejects adding a node to two executors at once -- node_ is already owned by
  /// executor_ below (for the wrench subscription's background spin). Hit live: "Node ... has
  /// already been added to an executor" the first time this shared a single node with node_.
  rclcpp::Node::SharedPtr param_client_node_;
  std::shared_ptr<rclcpp::SyncParametersClient> param_client_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_force_publisher_;
  /// engage() publishes ComplianceConfig::max_offset_m here -- JtacController's own real safety
  /// clamp (found live to be missing entirely until 2026-08-21; see that class's own
  /// clampCartesianOffset() doc comment for the incident that surfaced it). Same mechanism as
  /// target_force_publisher_ above, for the same RT-safety reason.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr max_offset_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr measured_wrench_subscriber_;
  /// Used only by protectiveStop() to cancel every in-flight goal on the controller's own
  /// FollowJointTrajectory action server -- see this class's own header doc on protectiveStop().
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;

  // -- Cross-thread handoff: written by the background executor's subscription callback, read by
  //    status() on whatever thread the caller (ForceStageSequencer) runs on -- same threading
  //    model as ft_bridge_hardware's own, and for the same reason. --
  std::mutex wrench_mutex_;
  geometry_msgs::msg::WrenchStamped latest_wrench_;
  bool has_received_wrench_{false};

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spin_{false};

  bool engaged_{false};
};

}  // namespace compliant_trajectory_controller
