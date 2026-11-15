/**
 * @file test_admittance_rule_integration.cpp
 * @brief Admittance-rule integration test: drive ros2_control's AdmittanceRule from OUTSIDE its own controller.
 * @details The fused-controller ("JTAC") plan rests on one assumption: that the maintained
 *          upstream compliance law is reusable standalone, so we build a trajectory host rather
 *          than reimplementing physics. This suite proves that against a MIXED
 *          prismatic+revolute 7-joint chain -- the configuration we could find no public
 *          precedent for.
 *
 *          It also pins the SEMANTICS we are relying on, not merely that it links:
 *            - a zero wrench must be a byte-exact no-op on the reference (the off-switch property)
 *            - a constant force must settle at the Hooke equilibrium x = F/K
 *            - a force on a de-selected axis must produce no motion
 *            - which joint absorbs a vertical push -- the redundancy-weighting question that
 *              stock admittance gives us no knob for
 *
 *          Runs fully offline: no hardware, no controller_manager, no robot_state_publisher.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "admittance_controller/admittance_rule.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

/// Mixed prismatic + revolute 7-joint chain at roughly UF850+lift scale. Deliberately synthetic
/// and self-contained so this stays a hermetic unit test; the realistic-geometry run against the
/// production URDF is a separate gate (a proper wall-contact model).
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
constexpr double kStiffnessZ = 300.0;  // mop's re-derived K_z = target_force_n / max_offset_m.
constexpr double kDt = 0.01;           // 100 Hz, the real controller_manager rate.

// DIAGNOSTIC gains, NOT mop's shipped production values -- see
// AdmittanceRuleUnconditionallyDivergesAtMopsShippedGains below for why. AdmittanceRule integrates
// with explicit Euler (admittance_rule_impl.hpp: joint_vel += joint_acc*dt; joint_pos +=
// joint_vel*dt), whose stability condition for a critically/over-damped spring-mass-damper is
// D*dt/M < 2, i.e. M > K*(zeta*dt)^2 for EVERY one of the six Cartesian channels independently,
// since the geometric (Jacobian) coupling between them spreads roundoff noise from an unstable
// channel into all the others even when only one axis carries applied force.
//
// mop's shipped values fail this on TWO axes, not one:
//   Z (normal): K_z=300,  zeta_z=20.4 -> M_min=12.48 kg  (shipped M_z=2.0 -- ~6x too light)
//   X/Y:        K=1000,   zeta=5.59   -> M_min=3.125 kg  (shipped M_xy=2.0 -- ~1.6x too light)
// This was found the hard way: an initial fix that only raised zeta_z's stability margin still
// diverged, because the unforced X-channel's own homogeneous dynamics (K=1000, zeta=5.59, M=2)
// were independently unstable and grew from floating-point roundoff alone -- visible in a cycle
// trace as admittance_velocity.x alternating sign and roughly doubling every single 10 ms cycle,
// the textbook signature of an unstable discrete pole.
//
// The diagnostic values below satisfy M > K*(zeta*dt)^2 on all three translational axes with
// margin (Z: M_min=1.08 kg; X/Y: M_min=3.125 kg with M=4.0), so the Hooke/linearity/redundancy
// tests below are validating the CONTROL LAW's semantics, not accidentally exercising this bug.
constexpr double kDiagnosticMassXY = 4.0;
constexpr double kDiagnosticDampingRatioZ = 6.0;

/// Deliberately non-singular: the all-zero pose stacks every link along a common axis and
/// collapses the Jacobian pseudo-inverse (a failure mode already hit live).
const std::vector<double> kReferencePose{0.40, 0.0, 0.6, -0.9, 0.0, 0.5, 0.0};

std::vector<rclcpp::Parameter> buildParameters(
  const std::vector<bool> & selected_axes = {false, false, true, false, false, false})
{
  return {
    rclcpp::Parameter("robot_description", std::string(kTestUrdf)),
    rclcpp::Parameter(
      "joints",
      std::vector<std::string>{"lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4",
        "joint5", "joint6"}),
    rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position", "velocity"}),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position", "velocity"}),
    rclcpp::Parameter(
      "chainable_command_interfaces", std::vector<std::string>{"position", "velocity"}),
    rclcpp::Parameter(
      "kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL")),
    rclcpp::Parameter("kinematics.plugin_package", std::string("kinematics_interface")),
    rclcpp::Parameter("kinematics.base", std::string("base_link")),
    rclcpp::Parameter("kinematics.tip", std::string("link_tcp")),
    rclcpp::Parameter("kinematics.alpha", 0.0005),
    rclcpp::Parameter("ft_sensor.name", std::string("test_ft")),
    rclcpp::Parameter("ft_sensor.frame.id", std::string("link6")),
    // 1.0 == pass-through: the wrench filter is not what this suite is testing, and smoothing it
    // would only stretch the settling time.
    rclcpp::Parameter("ft_sensor.filter_coefficient", 1.0),
    // control frame == kinematics base keeps the stiffness axes aligned with the base frame, so
    // the Hooke assertion below is unambiguous rather than entangled with a tool rotation.
    rclcpp::Parameter("control.frame.id", std::string("base_link")),
    rclcpp::Parameter("fixed_world_frame.frame.id", std::string("base_link")),
    rclcpp::Parameter("gravity_compensation.frame.id", std::string("link6")),
    rclcpp::Parameter("gravity_compensation.CoG.pos", std::vector<double>{0.0, 0.0, 0.0}),
    // Must stay 0.0: UFACTORY's ft_ext_force already removes payload/gravity, so a non-zero value
    // double-compensates (recorded as a landmine in tool_profiles.yaml).
    rclcpp::Parameter("gravity_compensation.CoG.force", 0.0),
    rclcpp::Parameter("admittance.selected_axes", selected_axes),
    rclcpp::Parameter(
      "admittance.mass",
      std::vector<double>{
        kDiagnosticMassXY, kDiagnosticMassXY, 2.0, 0.5, 0.5, 0.5}),
    rclcpp::Parameter(
      "admittance.stiffness", std::vector<double>{1000.0, 1000.0, kStiffnessZ, 50.0, 50.0, 50.0}),
    rclcpp::Parameter(
      "admittance.damping_ratio",
      std::vector<double>{5.59, 5.59, kDiagnosticDampingRatioZ, 1.0, 1.0, 1.0}),
    rclcpp::Parameter("admittance.joint_damping", 5.0),
    rclcpp::Parameter("enable_parameter_update_without_reactivation", false),
  };
}

trajectory_msgs::msg::JointTrajectoryPoint makePoint(const std::vector<double> & positions)
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities.assign(kNumJoints, 0.0);
  point.accelerations.assign(kNumJoints, 0.0);
  return point;
}

}  // namespace

class AdmittanceRuleIntegrationTest : public ::testing::Test
{
protected:
  void SetUpWithAxes(const std::vector<bool> & selected_axes)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(buildParameters(selected_axes));
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("jtac_rule_test", options);

    param_listener_ = std::make_shared<admittance_controller::ParamListener>(
      node_->get_node_parameters_interface(), node_->get_logger());
    rule_ = std::make_unique<admittance_controller::AdmittanceRule>(param_listener_);

    // An independent kinematics instance, so verification FK/Jacobian never reuses the rule's own
    // internal state -- otherwise a bug in the rule could hide behind the same bug in the check.
    loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");
    // Initialized at "link6", not "link_tcp": calculate_jacobian() only builds a solver for the
    // exact tip passed here (unlike calculate_link_transform(), which resolves any segment from
    // the full map) -- and every check in this file targets link6, matching the FT sensor.
    verify_kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      loader_->createUnmanagedInstance("kinematics_interface_kdl/KinematicsInterfaceKDL"));
    // Jazzy's kinematics_interface reads the tip from a "<param_namespace>.tip" parameter rather
    // than taking it directly -- a separate namespace from the rule's own "kinematics.tip"
    // (declared at "link_tcp" above), since this instance needs "link6" instead.
    if (!node_->has_parameter("verify_kinematics.tip")) {
      node_->declare_parameter("verify_kinematics.tip", std::string("link6"));
    }
    ASSERT_TRUE(verify_kinematics_->initialize(
        kTestUrdf, node_->get_node_parameters_interface(), "verify_kinematics"));
  }

  void SetUp() override {SetUpWithAxes({false, false, true, false, false, false});}

  /// Runs the compliance law closed-loop, feeding its own output back as the measured state --
  /// i.e. modelling a robot that tracks its commands perfectly. The displacement the spring
  /// "feels" is FK(current) - FK(reference), so without this feedback the spring would never see
  /// any deflection and nothing would ever converge.
  trajectory_msgs::msg::JointTrajectoryPoint runToSteadyState(
    const geometry_msgs::msg::Wrench & wrench, double duration_sec)
  {
    auto reference = makePoint(kReferencePose);
    auto current = makePoint(kReferencePose);
    auto desired = makePoint(kReferencePose);
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);

    const int iterations = static_cast<int>(duration_sec / kDt);
    for (int i = 0; i < iterations; ++i) {
      EXPECT_EQ(
        rule_->update(current, wrench, reference, period, desired),
        controller_interface::return_type::OK);
      current.positions = desired.positions;
      current.velocities = desired.velocities;
    }
    return current;
  }

  Eigen::Vector3d ftTranslation(const std::vector<double> & positions)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Map(positions.data(), static_cast<long>(positions.size()));
    Eigen::Isometry3d transform;
    EXPECT_TRUE(verify_kinematics_->calculate_link_transform(q, "link6", transform));
    return transform.translation();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<admittance_controller::ParamListener> param_listener_;
  std::unique_ptr<admittance_controller::AdmittanceRule> rule_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> verify_kinematics_;
};

// ---------------------------------------------------------------------------------------------
// the admittance-rule integration test proper: the rule is constructible, configurable, and runnable outside its own controller.
// ---------------------------------------------------------------------------------------------

TEST_F(AdmittanceRuleIntegrationTest, ConfiguresStandaloneAgainstMixedPrismaticRevoluteChain)
{
  // The whole fused-controller plan hinges on this call succeeding with a node we own rather than
  // one admittance_controller created for itself.
  ASSERT_EQ(
    rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  auto reference = makePoint(kReferencePose);
  auto current = makePoint(kReferencePose);
  auto desired = makePoint(kReferencePose);
  geometry_msgs::msg::Wrench wrench;

  EXPECT_EQ(
    rule_->update(current, wrench, reference, rclcpp::Duration::from_seconds(kDt), desired),
    controller_interface::return_type::OK);
  EXPECT_EQ(desired.positions.size(), kNumJoints);
}

TEST_F(AdmittanceRuleIntegrationTest, ZeroWrenchLeavesTheReferenceUntouched)
{
  ASSERT_EQ(rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  geometry_msgs::msg::Wrench zero_wrench;
  const auto settled = runToSteadyState(zero_wrench, 5.0);

  // The off-switch property: with no force, a compliant controller must be indistinguishable from
  // a plain trajectory controller. Anything else means compliance leaks into free-space motion.
  for (size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(settled.positions[i], kReferencePose[i], 1e-9) << "joint index " << i;
  }
}

// ---------------------------------------------------------------------------------------------
// Regression: mop's SHIPPED gains (tool_profiles.yaml, K_z=300, zeta_z=20.4, M_z=2.0) are
// unconditionally unstable at 100 Hz under this integrator, independent of contact stiffness or
// rig geometry. This was originally (and incorrectly) attributed to the mock rig's toy-arm link
// lengths during live verification; running the identical gains here, against a realistic-scale
// mixed 7-joint chain, reproduces the same divergence -- proving the real cause is the integrator
// stability bound above, not geometry. Keep this test failing (i.e. keep asserting divergence)
// until tool_profiles.yaml's mop.control.admittance is re-derived to satisfy M > K*(zeta*dt)^2;
// flip it to assert convergence at that point, don't delete it.
// ---------------------------------------------------------------------------------------------

TEST_F(AdmittanceRuleIntegrationTest, AdmittanceRuleUnconditionallyDivergesAtMopsShippedGains)
{
  // SetUp() already ran with the stable diagnostic zeta_z; overwrite it with mop's actual
  // production number before configure() so the rule picks it up.
  rclcpp::Parameter production_damping(
    "admittance.damping_ratio", std::vector<double>{5.59, 5.59, 20.4, 1.0, 1.0, 1.0});
  node_->set_parameter(production_damping);
  ASSERT_EQ(rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  geometry_msgs::msg::Wrench wrench;
  wrench.force.z = 6.0;
  const auto settled = runToSteadyState(wrench, 5.0);  // Diverges within well under 1 s in practice.

  bool any_non_finite = false;
  for (double p : settled.positions) {
    if (!std::isfinite(p)) {
      any_non_finite = true;
      break;
    }
  }
  EXPECT_TRUE(any_non_finite)
    << "mop's shipped zeta_z=20.4 at M_z=2.0 converged at 100 Hz -- if tool_profiles.yaml or the "
    << "control rate changed, update this test's fixture and flip it to assert convergence "
    << "instead of deleting it.";
}

// ---------------------------------------------------------------------------------------------
// Semantics we are relying on when we host this rule ourselves.
// ---------------------------------------------------------------------------------------------

TEST_F(AdmittanceRuleIntegrationTest, ConstantForceSettlesAtTheHookeEquilibrium)
{
  ASSERT_EQ(rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  geometry_msgs::msg::Wrench wrench;
  wrench.force.z = 6.0;  // Well inside max_offset_m: 6/300 = 2 cm of expected deflection.

  // Heavily overdamped by design (zeta_z = 20.4), so the slow pole is ~K/D = 0.3 1/s: allow many
  // time constants rather than assuming a fast settle.
  const auto settled = runToSteadyState(wrench, 60.0);

  const auto & state = rule_->get_controller_state();
  const Eigen::Vector3d displacement =
    ftTranslation(settled.positions) - ftTranslation(kReferencePose);

  // At equilibrium the spring balances the applied force: K * x = F, per axis, in the base frame
  // (control frame == base here). Compare against the rule's OWN base-frame wrench so the check is
  // independent of how the sensor-frame wrench happened to rotate.
  const double force_z_base = state.wrench_base.wrench.force.z;
  ASSERT_GT(std::abs(force_z_base), 1.0) << "test force did not survive into the base frame";
  EXPECT_NEAR(kStiffnessZ * displacement.z(), force_z_base, 0.05 * std::abs(force_z_base));

  // Unselected axes carry no applied force, so their springs must relax back to zero.
  EXPECT_NEAR(displacement.x(), 0.0, 2e-3);
  EXPECT_NEAR(displacement.y(), 0.0, 2e-3);
}

TEST_F(AdmittanceRuleIntegrationTest, ForceOnADeselectedAxisProducesOnlyItsSelectedAxisProjection)
{
  // Select Z only, then push along the FT sensor's own X axis. The wrench passed to update() is
  // in the SENSOR frame (link6), which is rotated relative to base/control at this bent
  // reference pose -- so a pure sensor-frame X push does NOT land purely on base-frame X. What
  // selected_axes actually masks is the CONTROL-frame axis, so this test predicts the expected
  // Cartesian response by measuring that rotation directly, rather than asserting "near zero" and
  // hoping the projection happens to be small.
  ASSERT_EQ(rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  Eigen::Isometry3d base_to_ft;
  ASSERT_TRUE(
    verify_kinematics_->calculate_link_transform(
      Eigen::VectorXd::Map(kReferencePose.data(), static_cast<long>(kReferencePose.size())),
      "link6",
      base_to_ft));
  const Eigen::Vector3d sensor_x_in_base = base_to_ft.rotation() * Eigen::Vector3d::UnitX();

  geometry_msgs::msg::Wrench wrench;
  wrench.force.x = 6.0;
  const auto settled = runToSteadyState(wrench, 60.0);

  const auto & state = rule_->get_controller_state();
  const Eigen::Vector3d displacement =
    ftTranslation(settled.positions) - ftTranslation(kReferencePose);
  const double force_z_base = state.wrench_base.wrench.force.z;

  // The only masking selected_axes performs is on the CONTROL-frame Z component -- so the
  // predicted Z displacement is exactly the Hooke response to whatever Z-component the sensor
  // rotation produced (6.0 * sensor_x_in_base.z()), not zero.
  const double predicted_force_z = 6.0 * sensor_x_in_base.z();
  EXPECT_NEAR(force_z_base, predicted_force_z, 0.05 * std::max(1.0, std::abs(predicted_force_z)));
  if (std::abs(force_z_base) > 0.5) {
    EXPECT_NEAR(kStiffnessZ * displacement.z(), force_z_base, 0.1 * std::abs(force_z_base));
  }

  // X and Y are genuinely masked in the control frame -- their springs must relax to zero
  // regardless of how the sensor-frame force happened to be oriented.
  EXPECT_NEAR(displacement.x(), 0.0, 2e-3);
  EXPECT_NEAR(displacement.y(), 0.0, 2e-3);
}

TEST_F(AdmittanceRuleIntegrationTest, DoublingTheForceDoublesTheDeflection)
{
  ASSERT_EQ(rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  geometry_msgs::msg::Wrench small;
  small.force.z = 3.0;
  const double deflection_small =
    (ftTranslation(runToSteadyState(small, 60.0).positions) - ftTranslation(kReferencePose)).z();

  rule_->reset(kNumJoints);

  geometry_msgs::msg::Wrench large;
  large.force.z = 6.0;
  const double deflection_large =
    (ftTranslation(runToSteadyState(large, 60.0).positions) - ftTranslation(kReferencePose)).z();

  // Linearity is what makes K_z = target_force_n / max_offset_m a valid way to size the gain at
  // all; if this failed, the whole "seek distance is inherently bounded" argument would fail too.
  ASSERT_GT(std::abs(deflection_small), 1e-4);
  EXPECT_NEAR(deflection_large / deflection_small, 2.0, 0.1);
}

// ---------------------------------------------------------------------------------------------
// Diagnostic: quantifies the redundancy-weighting gap that stock admittance gives us no knob for.
// ---------------------------------------------------------------------------------------------

TEST_F(AdmittanceRuleIntegrationTest, ReportsWhichJointAbsorbsAVerticalPush)
{
  ASSERT_EQ(rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  geometry_msgs::msg::Wrench wrench;
  wrench.force.z = 6.0;
  const auto settled = runToSteadyState(wrench, 60.0);

  // Per-joint offsets are in mixed units (metres for the lift, radians for the arm), so comparing
  // them directly is meaningless. Project each through the Jacobian instead to get the vertical
  // Cartesian motion each joint actually contributes.
  Eigen::VectorXd q =
    Eigen::VectorXd::Map(kReferencePose.data(), static_cast<long>(kReferencePose.size()));
  // calculate_jacobian() requires the output already sized to (6, num_joints) -- it validates
  // rather than resizes.
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, kNumJoints);
  ASSERT_TRUE(verify_kinematics_->calculate_jacobian(q, "link6", jacobian));

  double total_abs = 0.0;
  std::vector<double> contribution(kNumJoints, 0.0);
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double offset = settled.positions[i] - kReferencePose[i];
    contribution[i] = jacobian(2, static_cast<long>(i)) * offset;  // row 2 == Cartesian Z
    total_abs += std::abs(contribution[i]);
  }
  ASSERT_GT(total_abs, 1e-6);

  const double lift_share = std::abs(contribution[0]) / total_abs;
  RecordProperty("lift_share_of_vertical_motion", std::to_string(lift_share));
  for (size_t i = 0; i < kNumJoints; ++i) {
    RecordProperty("z_contribution_joint_" + std::to_string(i), std::to_string(contribution[i]));
  }

  // Deliberately a wide bound, not a tuned expectation: this test exists to MEASURE the split and
  // make a future change to it visible, since stock admittance exposes no per-joint weighting.
  EXPECT_GE(lift_share, 0.0);
  EXPECT_LE(lift_share, 1.0);
}

// ---------------------------------------------------------------------------------------------
// The counterpart to the test above: a WALL-like correction (Cartesian X, horizontal) rather than
// a CEILING-like one (Cartesian Z, vertical -- the same direction the lift itself translates in).
// Together these two tests are the direct evidence for "the lift dominates a vertical correction,
// but a horizontal one is the arm's job alone" -- not an assumption, a measured contrast.
// ---------------------------------------------------------------------------------------------

TEST_F(AdmittanceRuleIntegrationTest, LiftJacobianColumnIsPurelyVerticalByConstruction)
{
  // This does not need a running rule at all: it is a property of the URDF's kinematic structure.
  // lift_prismatic_joint is the FIRST joint in the chain (nothing precedes it to reorient its
  // axis), so its Jacobian column is exactly its own joint axis in world/base frame, for EVERY
  // configuration -- unlike every arm joint's column, which rotates with the pose. A prismatic
  // joint's column also has no rotational part.
  SetUpWithAxes({true, true, true, false, false, false});  // axis choice irrelevant to this check.
  Eigen::VectorXd q =
    Eigen::VectorXd::Map(kReferencePose.data(), static_cast<long>(kReferencePose.size()));
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, kNumJoints);
  ASSERT_TRUE(verify_kinematics_->calculate_jacobian(q, "link6", jacobian));

  constexpr int kLiftColumn = 0;
  EXPECT_NEAR(jacobian(0, kLiftColumn), 0.0, 1e-9) << "lift column has NO Cartesian-X authority";
  EXPECT_NEAR(jacobian(1, kLiftColumn), 0.0, 1e-9) << "lift column has NO Cartesian-Y authority";
  EXPECT_NEAR(std::abs(jacobian(2, kLiftColumn)), 1.0, 1e-9) << "lift column IS pure Cartesian-Z";
  for (int row = 3; row < 6; ++row) {
    EXPECT_NEAR(jacobian(row, kLiftColumn), 0.0, 1e-9) << "prismatic joints impart no rotation";
  }
}

TEST_F(AdmittanceRuleIntegrationTest, ReportsWhichJointAbsorbsAHorizontalPush)
{
  // Select Cartesian X only (the wall case: contact normal is horizontal, not aligned with the
  // lift's own travel direction) and construct a sensor-frame wrench that produces a clean push
  // along base-frame X, the same way ForceOnADeselectedAxis... above worked out the reverse
  // direction.
  SetUpWithAxes({true, false, false, false, false, false});
  ASSERT_EQ(rule_->configure(node_, kNumJoints, kTestUrdf), controller_interface::return_type::OK);

  Eigen::Isometry3d base_to_ft;
  ASSERT_TRUE(
    verify_kinematics_->calculate_link_transform(
      Eigen::VectorXd::Map(kReferencePose.data(), static_cast<long>(kReferencePose.size())),
      "link6",
      base_to_ft));
  const Eigen::Vector3d desired_base_force(6.0, 0.0, 0.0);
  const Eigen::Vector3d sensor_force = base_to_ft.rotation().transpose() * desired_base_force;

  geometry_msgs::msg::Wrench wrench;
  wrench.force.x = sensor_force.x();
  wrench.force.y = sensor_force.y();
  wrench.force.z = sensor_force.z();
  const auto settled = runToSteadyState(wrench, 60.0);

  Eigen::VectorXd q =
    Eigen::VectorXd::Map(kReferencePose.data(), static_cast<long>(kReferencePose.size()));
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, kNumJoints);
  ASSERT_TRUE(verify_kinematics_->calculate_jacobian(q, "link6", jacobian));

  double total_abs = 0.0;
  std::vector<double> contribution(kNumJoints, 0.0);
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double offset = settled.positions[i] - kReferencePose[i];
    contribution[i] = jacobian(0, static_cast<long>(i)) * offset;  // row 0 == Cartesian X
    total_abs += std::abs(contribution[i]);
  }
  ASSERT_GT(total_abs, 1e-6) << "the correction produced no horizontal motion at all";

  const double lift_share = std::abs(contribution[0]) / total_abs;
  RecordProperty("lift_share_of_horizontal_motion", std::to_string(lift_share));

  // Not "small" -- EXACTLY zero, up to floating point, because the previous test proved the
  // lift's Jacobian column has no X component at all. No pseudo-inverse weighting can route
  // Cartesian-X motion through a joint whose column is orthogonal to X.
  EXPECT_NEAR(lift_share, 0.0, 1e-6);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
