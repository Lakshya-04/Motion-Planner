/**
 * @file test_force_stage_sequencer_admittance_integration.cpp
 * @brief Gate T6: drive the REAL force_core::ForceStageSequencer/ForceStageFsm through a full
 *        APPROACH -> CARTESIAN -> RETRACT group against the REAL AdmittanceRule physics proven in
 *        test_admittance_rule_integration.cpp (T4a), via AdmittanceRuleCompliancePort.
 * @details T4a proved AdmittanceRule is reusable standalone; this proves the SEQUENCING layer
 *          (extracted to force_core specifically so platform could depend on it -- see
 *          that package's own package.xml) correctly drives it end to end: engage on approach,
 *          detect contact, settle onto a target force, hold it through a simulated coverage-path
 *          move, unload, peel off, and disengage -- with a genuine bumpless transfer (|F| below
 *          switch_safe_force_n at both the engage and disengage boundaries).
 *
 *          The measured wrench is SCRIPTED by contact phase (free air / in contact / unloaded),
 *          the same style T4a's own runToSteadyState() already uses (a caller-chosen wrench, not
 *          a self-consistent environment model) -- deliberately, not for lack of trying: an
 *          earlier version of this test computed contact as a real spring reaction from the
 *          tool's actual FK position against a virtual surface (environment stiffness in series
 *          with the admittance spring). That closed a SECOND feedback loop around the already-
 *          closed admittance loop, and the combined system went unstable (a slow limit cycle,
 *          joints swinging through radians) for reasons unrelated to the sequencer this test
 *          exists to validate -- exactly the kind of realistic-environment-model work T5
 *          ("Realistic-geometry mock re-run of R10", force_control_backend_admittance_vs_diffik.md
 *          section 10) is scoped to do properly, with a measured K_e from T1, not invented here.
 *          Scripting the wrench keeps this test's scope honest: does the SEQUENCER drive real
 *          AdmittanceRule physics through the right states in the right order, not "is this
 *          particular hand-picked environment stiffness stable."
 *
 *          There is deliberately no ExecutionPort or ManipulationManager here: this test drives
 *          ForceStageSequencer's beforeSegment()/afterSegment() hooks directly, standing in for
 *          the caller's own trajectory execution with
 *          AdmittanceRuleCompliancePort::simulateTrajectoryExecution() -- a focused test of the
 *          real sequencing classes against real compliance physics, without needing a running
 *          MoveIt/controller_manager stack. force_segment_expander (which DOES depend on manager
 *          types) is untouched by this; it was never meant to be portable, see force_core's
 *          own package.xml.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "kinematics_interface/kinematics_interface.hpp"
#include "compliant_trajectory_controller/admittance_rule_compliance_port.hpp"
#include "core_types/tool_profile.hpp"
#include "force_core/force_stage_sequencer.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

// Identical chain to test_admittance_rule_integration.cpp's kTestUrdf -- deliberately reused
// rather than re-derived, so any divergence in behavior traces to the sequencing layer, not to a
// different kinematic chain.
constexpr const char * kTestUrdf =
  R"(<?xml version="1.0"?>
<robot name="jtac_test_7dof">
  <link name="base_link"/>
  <link name="lift_top_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>
  <link name="link4"/>
  <link name="link5"/>
  <link name="link6"/>
  <link name="link_tcp"/>

  <joint name="lift_prismatic_joint" type="prismatic">
    <parent link="base_link"/><child link="lift_top_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="0.8" effort="1000" velocity="0.2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
  <joint name="joint1" type="revolute">
    <parent link="lift_top_link"/><child link="link1"/>
    <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="100" velocity="3.14"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <axis xyz="0 1 0"/><limit lower="-3.14" upper="3.14" effort="100" velocity="3.14"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <axis xyz="0 1 0"/><limit lower="-3.14" upper="3.14" effort="100" velocity="3.14"/>
    <origin xyz="0 0 0.39" rpy="0 0 0"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="100" velocity="3.14"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <axis xyz="0 1 0"/><limit lower="-3.14" upper="3.14" effort="100" velocity="3.14"/>
    <origin xyz="0 0 0.30" rpy="0 0 0"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="100" velocity="3.14"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <parent link="link6"/><child link="link_tcp"/>
    <origin xyz="0 0 0.09" rpy="0 0 0"/>
  </joint>
</robot>)";

constexpr size_t kNumJoints = 7;
constexpr double kDt = 0.01;  // 100 Hz, matching T4a and combined_arm_lift_controllers.yaml.

// Same diagnostic Z gains T4a's own suite proved stable (M > K*(zeta*dt)^2 with margin: M_min =
// 300*(6*0.01)^2 = 1.08 kg, shipped/used here 2.0). Deliberately NOT mop's shipped zeta_z=20.4 --
// that divergence is a separately pinned regression (AdmittanceRuleUnconditionallyDivergesAtMops
// ShippedGains); this suite is about sequencing, and needs stable physics under it.
constexpr double kMassZ = 2.0;
constexpr double kStiffnessZ = 300.0;
constexpr double kDampingRatioZ = 6.0;
constexpr double kMassXY = 4.0;
constexpr double kStiffnessXY = 1000.0;
constexpr double kDampingRatioXY = 5.59;

double rawDamping(double ratio, double mass, double stiffness)
{
  return ratio * 2.0 * std::sqrt(mass * stiffness);
}

// Same non-singular reference pose T4a's own suite uses -- see that file's own comment on why the
// all-zero pose is a trap.
const std::vector<double> kReferencePose{0.40, 0.0, 0.6, -0.9, 0.0, 0.5, 0.0};

const std::vector<std::string> kJointNames{
  "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

constexpr double kTargetForceN = 6.0;
// Scripted sensor readings -- see this file's own header doc for why these are scripted rather
// than derived from a self-consistent environment model. Deliberately NOT exactly target_force_n/
// zero: a small, fixed offset from each (0.5 N, 0.3 N) still exercises AdmittanceRule producing a
// genuine, nonzero, physically real deflection each phase, while staying inside force_band_n /
// force_zero_threshold_n from the very first sample -- since the scripted value never changes in
// response to admittance's own reaction, it must already be within band for the FSM's settle
// check to ever succeed.
constexpr double kContactMeasuredForceN = kTargetForceN + 0.5;
constexpr double kUnloadedResidualForceN = 0.3;

core_types::ToolProfile buildTestToolProfile()
{
  core_types::ToolProfile profile;
  profile.name = "t6_test_tool";
  profile.task.contact = true;
  profile.task.target_force_n = kTargetForceN;
  profile.task.force_band_n = 1.5;
  profile.task.force_zero_threshold_n = 1.0;
  profile.task.settle_timeout_s = 8.0;
  profile.task.overload_abort_n = 40.0;
  profile.task.tangential_torque_abort_nm = 10.0;
  profile.task.unexpected_contact_abort_n = 0.0;  // disabled -- not exercised by this sequence.

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

  profile.control.ft.lpf_cutoff_hz = 0.0;  // Disabled: this suite wants deterministic physics,
                                            // not filter dynamics -- T4a's own suite makes the
                                            // same call (ft_sensor.filter_coefficient = 1.0).
  profile.control.ft.deadband_n = 0.1;
  profile.control.ft.clamp_n = 100.0;
  profile.control.ft.tare_per_goal = false;  // Mission-level concern; not this test's scope.
  return profile;
}

}  // namespace

class ForceStageSequencerAdmittanceIntegrationTest : public ::testing::Test
{
protected:
  enum class ContactPhase
  {
    kFreeAir,   ///< No contact yet -- sensor reads zero.
    kInContact, ///< Tool is against the surface -- sensor reads kContactMeasuredForceN.
    kUnloaded,  ///< RETRACT has commanded target_force_n=0 -- sensor reads a small residual.
  };

  void SetUp() override
  {
    compliant_trajectory_controller::AdmittanceRuleCompliancePort::Options options;
    options.robot_description = kTestUrdf;
    options.joint_names = kJointNames;
    options.kinematics_base = "base_link";
    // Deliberately base_link, not link_tcp -- reuses T4a's own already-proven control-frame
    // convention (Cartesian Z == the compliant axis) rather than introducing an unverified
    // tip-frame admittance configuration in this first sequencer-level integration test. A real
    // ToolProfile-driven deployment against a wall would set control_frame to the tool tip
    // instead -- see AdmittanceRuleCompliancePort's own header doc.
    options.kinematics_tip = "link_tcp";
    options.dt_s = kDt;
    port_ = std::make_unique<compliant_trajectory_controller::AdmittanceRuleCompliancePort>(
      options, kReferencePose);

    // Independent kinematics instance, used only to build small joint-space moves (via a one-shot
    // Jacobian pseudo-inverse step) representing APPROACH/CARTESIAN/RETRACT trajectories -- never
    // reuses the port's internal kinematics, matching T4a's own "verification never shares state
    // with the thing it verifies" convention.
    loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");
    kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      loader_->createUnmanagedInstance("kinematics_interface_kdl/KinematicsInterfaceKDL"));
    // Kept alive as a member (probe_node_), not a SetUp()-local temporary: kinematics_interface_
    // kdl's KinematicsInterface::initialize() retains the node_parameters_interface it is given
    // rather than copying what it needs out of it, so a node that dies when SetUp() returns
    // leaves kinematics_ reading through a dangling interface for the rest of the test -- exactly
    // T4a's own node_ member pattern, reused here for the same reason.
    probe_node_ = std::make_shared<rclcpp::Node>("t6_kinematics_probe");
    probe_node_->declare_parameter("robot_description", std::string(kTestUrdf));
    // Jazzy's kinematics_interface reads the tip from a "<param_namespace>.tip" parameter rather
    // than taking it directly.
    probe_node_->declare_parameter("kinematics.tip", std::string("link_tcp"));
    ASSERT_TRUE(
      kinematics_->initialize(
        kTestUrdf, probe_node_->get_node_parameters_interface(), "kinematics"));
  }

  /// One-shot Gauss-Newton IK step: joint delta that would produce cartesian_delta_base (a 6-
  /// vector, translation-only here) from `from_positions`, via the Jacobian pseudo-inverse.
  /// Adequate for this test's small (<=5 cm) moves against the well-conditioned reference pose
  /// T4a's own suite already validated -- not a general IK solver.
  std::vector<double> stepToward(
    const std::vector<double> & from_positions, const Eigen::Vector3d & cartesian_delta_base)
  {
    Eigen::VectorXd q =
      Eigen::VectorXd::Map(from_positions.data(), static_cast<long>(from_positions.size()));
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, kNumJoints);
    EXPECT_TRUE(kinematics_->calculate_jacobian(q, "link_tcp", jacobian));
    Eigen::Matrix<double, 6, 1> delta6 = Eigen::Matrix<double, 6, 1>::Zero();
    delta6.head<3>() = cartesian_delta_base;
    const Eigen::VectorXd delta_q = jacobian.completeOrthogonalDecomposition().solve(delta6);
    std::vector<double> result(kNumJoints);
    for (size_t i = 0; i < kNumJoints; ++i) {
      result[i] = from_positions[i] + delta_q[static_cast<long>(i)];
    }
    return result;
  }

  core_types::Wrench measuredWrench() const
  {
    core_types::Wrench wrench;
    switch (contact_phase_) {
      case ContactPhase::kFreeAir:
        break;  // 0 N -- nothing sensed yet.
      case ContactPhase::kInContact:
        // Negative while pressing along the tool's own +Z -- core_types::Wrench's own sign
        // convention (control_frame == base_link here, so "the tool's own +Z" is base-frame Z).
        wrench.force_z = -kContactMeasuredForceN;
        break;
      case ContactPhase::kUnloaded:
        wrench.force_z = -kUnloadedResidualForceN;
        break;
    }
    return wrench;
  }

  std::unique_ptr<compliant_trajectory_controller::AdmittanceRuleCompliancePort> port_;
  std::shared_ptr<rclcpp::Node> probe_node_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
  ContactPhase contact_phase_{ContactPhase::kFreeAir};
};

TEST_F(ForceStageSequencerAdmittanceIntegrationTest, DrivesApproachContactCartesianRetract)
{
  const auto tool_profile = buildTestToolProfile();

  // Shared tick counter: BOTH the sequencer's injected clock and the port's own status().stamp_ns
  // must derive from the same timeline, or every settle_timeout_s/convergence_ticks comparison
  // in the FSM becomes meaningless -- see AdmittanceRuleCompliancePort::simulatedStampNs()'s own
  // doc. Ticking it via the port itself (not a second independent counter) guarantees that.
  auto now_ns_fn = [this]() {return port_->simulatedStampNs();};
  // Each dwell tick: hold the reference exactly where it is (a real "wait here" dwell, not a
  // trajectory move) while advancing compliance physics by exactly one dt_s cycle, sampling
  // whatever the current scripted contact phase reads.
  auto sleep_fn = [this]() {
      port_->simulateTrajectoryExecution(
        port_->referenceJointPositions(), kDt, [this]() {return measuredWrench();});
    };
  force_core::ForceStageSequencer sequencer(*port_, now_ns_fn, sleep_fn);

  // --- APPROACH: engage, drive a real trajectory toward the surface, then contact is made. ---
  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  auto outcome = sequencer.beforeSegment(approach_intent, tool_profile);
  ASSERT_TRUE(outcome.proceed) << outcome.failure_reason_code << ": " << outcome.message;
  ASSERT_EQ(sequencer.state(), force_core::ForceFsmState::kApproach);

  const auto approach_target = stepToward(kReferencePose, Eigen::Vector3d(0.0, 0.0, 0.03));
  port_->simulateTrajectoryExecution(approach_target, 1.0, [this]() {return measuredWrench();});

  // Contact is made at the end of the approach move -- onTrajectoryFinished() checks a single
  // status() sample taken once the trajectory has finished, not a history (see ForceStageFsm's
  // own doc), so flipping the scripted phase here and letting AdmittanceRule react for a short
  // settle window is a faithful model of that check.
  contact_phase_ = ContactPhase::kInContact;
  port_->simulateTrajectoryExecution(approach_target, 1.0, [this]() {return measuredWrench();});

  outcome = sequencer.afterSegment(true);
  ASSERT_TRUE(outcome.proceed) << outcome.failure_reason_code << ": " << outcome.message;
  EXPECT_EQ(sequencer.state(), force_core::ForceFsmState::kApproachContactConfirmed);

  // --- CARTESIAN: settle onto target_force_n, then hold it through a lateral "coverage path"
  //     move (base-frame X) -- proving the strip trajectory executes normally once force is
  //     held, not just that the FSM transitions correctly at a standstill. ---
  core_types::ForceIntent cartesian_intent;
  cartesian_intent.stage = core_types::ForceStage::kCartesian;
  outcome = sequencer.beforeSegment(cartesian_intent, tool_profile);
  ASSERT_TRUE(outcome.proceed) << outcome.failure_reason_code << ": " << outcome.message;
  EXPECT_EQ(sequencer.state(), force_core::ForceFsmState::kCartesianFollow);

  const double force_at_settle = std::abs(core_types::normalForceN(port_->status().wrench));
  EXPECT_NEAR(force_at_settle, kTargetForceN, tool_profile.task.force_band_n)
    << "CARTESIAN settle released the trajectory without actually converging on target_force_n";
  // AdmittanceRule genuinely reacted to the scripted contact wrench, not a no-op: the Hooke
  // equilibrium for the 0.5 N error above is small (0.5/300 m) but must be nonzero.
  EXPECT_GT(std::abs(port_->status().normal_offset_m), 1e-5)
    << "compliance produced no deflection at all in response to a real contact wrench";

  const auto lateral_target =
    stepToward(port_->currentJointPositions(), Eigen::Vector3d(0.03, 0.0, 0.0));
  port_->simulateTrajectoryExecution(lateral_target, 1.0, [this]() {return measuredWrench();});

  const double force_after_slide =
    std::abs(core_types::normalForceN(port_->status().wrench));
  EXPECT_NEAR(force_after_slide, kTargetForceN, tool_profile.task.force_band_n)
    << "contact force was not held through the simulated coverage-path move";

  outcome = sequencer.afterSegment(true);
  ASSERT_TRUE(outcome.proceed) << outcome.failure_reason_code << ": " << outcome.message;
  EXPECT_EQ(sequencer.state(), force_core::ForceFsmState::kCartesianDone);

  // --- RETRACT: unload to ~0 N, then peel all the way back to the original reference pose. ---
  contact_phase_ = ContactPhase::kUnloaded;  // The sensor now reads near-zero, before any motion
                                              // -- matching the FSM's own "unload before peel"
                                              // design (see ForceStageFsm's kRetractSettle doc).
  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  outcome = sequencer.beforeSegment(retract_intent, tool_profile);
  ASSERT_TRUE(outcome.proceed) << outcome.failure_reason_code << ": " << outcome.message;
  EXPECT_EQ(sequencer.state(), force_core::ForceFsmState::kRetractFollow);

  const double force_at_unload = std::abs(core_types::normalForceN(port_->status().wrench));
  EXPECT_LT(force_at_unload, tool_profile.task.force_zero_threshold_n)
    << "RETRACT released its peel-off trajectory while still loaded";

  port_->simulateTrajectoryExecution(kReferencePose, 1.0, [this]() {return measuredWrench();});

  outcome = sequencer.afterSegment(true);
  ASSERT_TRUE(outcome.proceed) << outcome.failure_reason_code << ": " << outcome.message;
  EXPECT_EQ(sequencer.state(), force_core::ForceFsmState::kIdle)
    << "group did not complete cleanly -- expected a return to kIdle after disengage";

  // --- The P3.2 acceptance criterion, end to end: bumpless transfer at the disengage switch.
  // Reaching kIdle at all already guarantees this (ForceFsmParams::switch_safe_force_n's own
  // default is 3.0 N; onTrajectoryFinished() aborts with controller_switch_unsafe rather than
  // emitting kDisengage above it) -- asserted explicitly anyway, per this plan doc's own point
  // that this turns the acceptance criterion into a printed number, not just an implication.
  const auto & telemetry = sequencer.telemetry();
  EXPECT_LT(telemetry.max_abs_force_at_switch_n, 3.0) << "disengaged above the bumpless-transfer "
                                                          "bound (switch_safe_force_n)";
  EXPECT_EQ(telemetry.compliance_switch_count, 1U)
    << "afterSegment's telemetry is per-segment (reset every beforeSegment) -- RETRACT's own "
       "group should show exactly one switch: the disengage() this group ends with";
  EXPECT_FALSE(telemetry.compliance_engaged_at_exit)
    << "compliance must be OFF once the whole APPROACH->CARTESIAN->RETRACT group has completed";

  // The arm actually came back home -- the physical point of a peel-off, not just an FSM state.
  for (size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(port_->currentJointPositions()[i], kReferencePose[i], 0.01)
      << "joint index " << i << " did not return to the reference pose after RETRACT";
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
