/**
 * @file admittance_rule_compliance_port.hpp
 * @brief A real core_types::CompliancePort implementation, backed by ros2_control's
 *        AdmittanceRule -- the JTAC compliance mechanism proven in test_admittance_rule_
 *        integration.cpp (T4a), now wired to the real port interface so it can be driven by
 *        force_core::ForceStageSequencer.
 * @details Lives here (platform), not capabilities, because CompliancePort implementations
 *          are adapters over rclcpp/admittance_controller/kinematics_interface -- see
 *          compliance_port.hpp's own doc comment on why the port itself sits in core_types.
 *
 *          The installed Humble AdmittanceRule drives toward measured-wrench == 0 and has no
 *          force setpoint (see force_control_backend_admittance_vs_diffik.md's Context section,
 *          option (a)). This class supplies the setpoint the same way a real FT bridge would:
 *          it biases the measured wrench by target_force_n_ along the tool's own +Z before
 *          feeding AdmittanceRule::update(), so "drive to zero" becomes "drive to target". It
 *          also owns the FT hygiene (low-pass filter, deadband, clamp) ComplianceConfig carries --
 *          per the plan doc's Context section, "the setpoint, deadband, clamp and tare land in
 *          the FT bridge", and there is no separate bridge component yet.
 *
 *          engage() rebuilds the internal AdmittanceRule/kinematics stack from scratch against the
 *          new ComplianceConfig rather than attempting live parameter reconfiguration -- engage()
 *          IS the controller switch (this class's own doc on CompliancePort::engage says so), so
 *          reconstructing internal state at that boundary is the architecturally honest choice,
 *          and it reuses the exact construction sequence jtac_compliance_demo_node.cpp already
 *          proved live, rather than a new, unverified reconfiguration path.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "admittance_controller/admittance_rule.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "core_types/compliance_port.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace compliant_trajectory_controller
{

/**
 * @class AdmittanceRuleCompliancePort
 * @brief CompliancePort backed by AdmittanceRule, ticked in-process (no controller_manager).
 */
class AdmittanceRuleCompliancePort : public core_types::CompliancePort
{
public:
  /**
   * @struct Options
   * @brief The kinematic chain this port drives -- fixed for the port's lifetime, unlike
   *        ComplianceConfig (which changes per engage() call as tools/tasks change).
   */
  struct Options
  {
    std::string robot_description;                ///< Flat (already xacro-rendered) URDF.
    std::vector<std::string> joint_names;          ///< Positional order matches the URDF chain.
    std::string kinematics_base{"base_link"};      ///< Must not include the compliant joints' root
                                                    ///< above the chain -- see kinematics.base note
                                                    ///< in force_control_backend_admittance_vs_diffik.md.
    std::string kinematics_tip{"link_tcp"};        ///< Controlled tool point (CLAUDE.md convention).
    double dt_s{0.01};                             ///< AdmittanceRule tick period (100 Hz default).
  };

  AdmittanceRuleCompliancePort(Options options, std::vector<double> initial_joint_positions);

  bool tare(std::string & error_message) override;
  bool engage(const core_types::ComplianceConfig & config, std::string & error_message)
  override;
  bool setTargetForce(double target_force_n, std::string & error_message) override;
  bool disengage(std::string & error_message) override;
  core_types::ComplianceStatus status() override;
  bool protectiveStop(std::string & error_message) override;

  /**
   * @brief TEST/SIMULATION-ONLY SEAM -- not part of CompliancePort.
   * @details Production trajectory time is driven by a real JTC through a real ExecutionPort;
   *          there is no ExecutionPort here to drive it, since this integration test exercises
   *          ForceStageSequencer directly (see test/test_force_stage_sequencer_admittance_
   *          integration.cpp's own doc comment for why). This substitutes for that JTC: it moves
   *          the AdmittanceRule reference linearly from its current value toward
   *          target_joint_positions over duration_s, ticking update() at dt_s and sampling
   *          measured_wrench_provider() once per tick (in the kinematics tip frame) so the caller
   *          controls exactly when simulated contact begins. Returns the resulting (compliance-
   *          shifted) joint positions.
   */
  std::vector<double> simulateTrajectoryExecution(
    const std::vector<double> & target_joint_positions, double duration_s,
    const std::function<core_types::Wrench()> & measured_wrench_provider);

  std::vector<double> currentJointPositions() const {return current_.positions;}
  std::vector<double> referenceJointPositions() const {return reference_.positions;}

  /**
   * @brief The port's own tick counter, in nanoseconds -- what status().stamp_ns reports.
   * @details NOT wall-clock time: it advances by exactly options_.dt_s per tickOnce() call, so a
   *          caller driving both this port AND force_core::ForceStageSequencer (whose FSM
   *          derives every timeout from ComplianceStatus::stamp_ns, per force_stage_fsm.hpp's own
   *          doc) can inject THIS SAME counter as the sequencer's now_ns_fn. Sharing one clock is
   *          required, not a convenience: mixing this status()'s stamp with a separately-advancing
   *          wall clock for settle_entry_ns_ would make every settle_timeout_s/convergence_ticks
   *          comparison meaningless, and would also make the dwell loop's real duration equal
   *          settle_timeout_s of actual wall-clock waiting rather than of simulated ticks.
   */
  uint64_t simulatedStampNs() const {return simulated_stamp_ns_;}

private:
  void buildRule(const core_types::ComplianceConfig & config);
  void tickOnce(const core_types::Wrench & measured_wrench);
  double normalOffsetM() const;
  trajectory_msgs::msg::JointTrajectoryPoint makePoint(
    const std::vector<double> & positions) const;
  static uint64_t nowNs();

  Options options_;
  size_t num_joints_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> rule_node_;
  std::shared_ptr<admittance_controller::ParamListener> param_listener_;
  std::unique_ptr<admittance_controller::AdmittanceRule> rule_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
  Eigen::Isometry3d base_to_tip_at_reference_;  ///< Snapshot at the moment reference_ was last set.

  trajectory_msgs::msg::JointTrajectoryPoint current_;
  trajectory_msgs::msg::JointTrajectoryPoint reference_;

  bool engaged_{false};
  double target_force_n_{0.0};
  double max_offset_m_{0.0};
  double deadband_n_{0.0};
  double clamp_n_{0.0};
  double lpf_alpha_{1.0};             ///< 1.0 == no filtering (lpf_cutoff_hz == 0 means disabled).
  double filtered_force_z_{0.0};
  bool filter_initialized_{false};
  uint64_t simulated_stamp_ns_{1'000'000ULL};  ///< See simulatedStampNs()'s own doc.
};

}  // namespace compliant_trajectory_controller
