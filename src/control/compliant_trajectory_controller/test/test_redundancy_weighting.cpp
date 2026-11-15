/**
 * @file test_redundancy_weighting.cpp
 * @brief Redundancy-weighting test: pins weightedPseudoInverse() (JtacController's per-joint redundancy weighting) against the general
 *        algebraic properties the whole feature rests on.
 * @details Deliberately tests weightedPseudoInverse() in isolation from JtacController::
 *          updateWeighted()'s surrounding frame-rotation/masking logic (K/D rotated control-frame
 *          -> base-frame, selected_axes masked BEFORE that rotation) -- that logic needs a real
 *          kinematics_interface instance and a real pose to exercise meaningfully, and is
 *          verified LIVE instead (see the redundancy weighting section for the exact numbers: 55.8% ceiling lift
 *          share at the [10,1,1,1,1,1,1] default, ~93-96% unweighted, 1.2%/100% at the two
 *          extremes tried). Same tradeoff this codebase already names for CartesianPlanner's own
 *          diagnose_truncation/setFromIK recovery: live-verified where a
 *          real kinematics chain is the point, unit-tested where a general algebraic property is.
 *
 *          Runs fully offline: no hardware, no controller_manager, no kinematics_interface, no
 *          URDF -- weightedPseudoInverse() takes a plain Jacobian, so a hand-built one is enough
 *          to pin the two properties that actually matter: (1) equal weights reduce EXACTLY to
 *          the plain Moore-Penrose pseudo-inverse (the "off switch" this feature must have when
 *          nobody asked for a bias), and (2) a heavily-weighted joint's contribution to the
 *          solution collapses toward zero relative to an unweighted solve on the SAME input (the
 *          actual mechanism the feature exists to provide).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "compliant_trajectory_controller/jtac_controller.hpp"

namespace
{

using compliant_trajectory_controller::weightedPseudoInverse;

/// A 3x7 Jacobian shaped like the real mixed prismatic+revolute 7-DOF chain this feature targets:
/// column 0 (the lift) is PURE Cartesian-Z (matching LiftJacobianColumnIsPurelyVerticalByConstruction
/// in test_admittance_rule_integration.cpp -- a prismatic joint at the chain root has this
/// property for every configuration, not just this one hand-picked matrix), and the six arm
/// columns are a representative mix of contributions to all three axes, none of them axis-aligned
/// (as a real, non-singular arm pose's columns generally aren't).
Eigen::Matrix<double, 3, Eigen::Dynamic> testJacobian()
{
  Eigen::Matrix<double, 3, Eigen::Dynamic> j(3, 7);
  j.col(0) << 0.0, 0.0, 1.0;    // lift: pure Z, by construction.
  j.col(1) << 0.3, -0.2, 0.5;
  j.col(2) << -0.4, 0.6, 0.3;
  j.col(3) << 0.2, 0.1, -0.3;
  j.col(4) << -0.1, 0.3, 0.4;
  j.col(5) << 0.5, -0.3, 0.2;
  j.col(6) << -0.2, 0.4, -0.1;
  return j;
}

Eigen::VectorXd equalWeights()
{
  return Eigen::VectorXd::Constant(7, 1.0);
}

}  // namespace

// -------------------------------------------------------------------------------------------
// Property 1: the "off switch". redundancy_weights defaults to [10,1,1,1,1,1,1], but a caller
// who genuinely wants unweighted behavior (equal weights) must get EXACTLY the standard
// Moore-Penrose result, not an approximation of it -- otherwise "redundancy weighting disabled"
// and "redundancy weighting enabled with equal weights" would silently diverge.
// -------------------------------------------------------------------------------------------

TEST(RedundancyWeighting, EqualWeightsReduceToPlainMoorePenrosePseudoInverse)
{
  const auto j = testJacobian();
  const Eigen::MatrixXd weighted = weightedPseudoInverse(j, equalWeights());

  // Eigen's own JacobiSVD-based pseudo-inverse -- an INDEPENDENT computation path from
  // weightedPseudoInverse()'s own W^-1 J^T (J W^-1 J^T)^-1 formula, so this genuinely checks the
  // two agree rather than checking the same formula against itself.
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(j, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd moore_penrose = svd.solve(Eigen::Matrix3d::Identity());

  ASSERT_EQ(weighted.rows(), moore_penrose.rows());
  ASSERT_EQ(weighted.cols(), moore_penrose.cols());
  for (int r = 0; r < weighted.rows(); ++r) {
    for (int c = 0; c < weighted.cols(); ++c) {
      EXPECT_NEAR(weighted(r, c), moore_penrose(r, c), 1e-9)
        << "at (" << r << "," << c << ")";
    }
  }
}

TEST(RedundancyWeighting, EqualWeightsRecoverTheOriginalCartesianTargetExactly)
{
  // A pseudo-inverse solution J_W+ * x_ddot must satisfy J * (J_W+ * x_ddot) == x_ddot for ANY
  // weighting (the whole point of a pseudo-inverse of a full-row-rank, wide Jacobian: it always
  // finds SOME joint-space vector that reproduces the requested Cartesian motion exactly, only
  // the REDUNDANT extra degrees of freedom differ by weight). If this fails, the "weighting only
  // redistributes, never changes the net Cartesian result" claim in the decision doc is false.
  const auto j = testJacobian();
  const Eigen::Vector3d x_ddot(0.4, -0.2, 0.6);

  for (const Eigen::VectorXd & weights :
    {equalWeights(), (Eigen::VectorXd(7) << 1000.0, 1, 1, 1, 1, 1, 1).finished(),
      (Eigen::VectorXd(7) << 1.0, 1000, 1000, 1000, 1000, 1000, 1000).finished()})
  {
    const Eigen::MatrixXd pinv = weightedPseudoInverse(j, weights);
    const Eigen::Vector3d reproduced = j * (pinv * x_ddot);
    EXPECT_NEAR(reproduced.x(), x_ddot.x(), 1e-9);
    EXPECT_NEAR(reproduced.y(), x_ddot.y(), 1e-9);
    EXPECT_NEAR(reproduced.z(), x_ddot.z(), 1e-9);
  }
}

// -------------------------------------------------------------------------------------------
// Property 2: the actual mechanism. A joint given an extreme weight relative to the others must
// have its own contribution to the solution collapse toward zero -- this is the literal
// definition of "less compliant" the whole feature exists to provide, and it is what was
// live-verified against the real robot (1.2%/100% lift share at the two 1000:1 extremes tried)
// -- here pinned as a deterministic,
// offline property of the same function, not dependent on any live URDF/pose.
//
// x_ddot is PURE Z here, deliberately -- not a mixed direction. The real controller only ever
// commands a target along the single axis admittance.selected_axes has enabled at a time (mop's
// own profile: Z only); the lift's own column is EXACTLY (0,0,1) (LiftJacobianColumnIsPurely
// VerticalByConstruction in test_admittance_rule_integration.cpp -- a prismatic root joint has
// this property for every pose), so it alone CAN satisfy a pure-Z target regardless of weight.
// Found live while writing this test: a MIXED target (nonzero X/Y too) breaks the "arm-biased
// extreme makes the lift dominate" direction outright, because the lift's single scalar DOF
// mathematically cannot solve a 3-component target alone no matter how it's weighted --
// EqualWeightsRecoverTheOriginalCartesianTargetExactly (above) already proves
// weightedPseudoInverse() always finds an EXACT solution to J*q=x_ddot, so an infeasible-for-the-
// lift-alone mixed target forces the arm to keep contributing regardless of weight. A real
// bug-shaped test-design mistake, not a bug in weightedPseudoInverse() itself.
// -------------------------------------------------------------------------------------------

TEST(RedundancyWeighting, HeavilyWeightedJointsCollapseTowardZeroContribution)
{
  const auto j = testJacobian();
  const Eigen::Vector3d x_ddot(0.0, 0.0, 0.6);

  const Eigen::MatrixXd unweighted = weightedPseudoInverse(j, equalWeights());
  const double unweighted_lift_component = std::abs((unweighted * x_ddot)(0));
  ASSERT_GT(unweighted_lift_component, 1e-6)
    << "test setup problem: the unweighted solution must use the lift at all for this to be a "
       "meaningful comparison";

  Eigen::VectorXd lift_biased = equalWeights();
  lift_biased(0) = 1000.0;  // lift weighted 1000x every arm joint -- the arm-biased extreme.
  const Eigen::MatrixXd weighted = weightedPseudoInverse(j, lift_biased);
  const double weighted_lift_component = std::abs((weighted * x_ddot)(0));

  EXPECT_LT(weighted_lift_component, 0.05 * unweighted_lift_component)
    << "an extreme 1000:1 lift weight should nearly eliminate the lift's own contribution, "
       "matching the 1.2% (down from ~93-96%) live-verified at this same ratio";
}

TEST(RedundancyWeighting, LightlyWeightingOneJointMakesItDominateTheSolution)
{
  // The mirror image: weighting every OTHER joint heavily relative to the lift should make the
  // lift do NEARLY ALL of the work -- matching the opposite live-verified extreme (100% lift
  // share at the 1:1000 ratio). Pure-Z target -- see this section's own comment above for why.
  const auto j = testJacobian();
  const Eigen::Vector3d x_ddot(0.0, 0.0, 0.6);

  Eigen::VectorXd arm_biased = equalWeights();
  for (int i = 1; i < 7; ++i) {
    arm_biased(i) = 1000.0;
  }
  const Eigen::MatrixXd weighted = weightedPseudoInverse(j, arm_biased);
  const Eigen::VectorXd solution = weighted * x_ddot;

  const double lift_component = std::abs(solution(0));
  double arm_total = 0.0;
  for (int i = 1; i < 7; ++i) {
    arm_total += std::abs(solution(i));
  }
  EXPECT_GT(lift_component, 20.0 * arm_total)
    << "an extreme 1:1000 arm weight should make the lift carry nearly all the solution, "
       "matching the 100% lift share live-verified at this same ratio";
}

TEST(RedundancyWeighting, IntermediateWeightRatioGivesAnIntermediateSplit)
{
  // The [10,1,1,1,1,1,1] chosen default (the arm kept MORE compliant than the lift, a deliberate
  // and reversible design choice -- see jtac_controller.hpp's own redundancy weighting header doc) should sit
  // strictly between the unweighted split and the extreme arm-biased split, not collapse to
  // either endpoint -- pins that "10x" is a genuinely intermediate, non-degenerate choice.
  // Pure-Z target -- see this section's own comment above for why.
  const auto j = testJacobian();
  const Eigen::Vector3d x_ddot(0.0, 0.0, 0.6);

  const double unweighted_lift = std::abs((weightedPseudoInverse(j, equalWeights()) * x_ddot)(0));

  Eigen::VectorXd default_weights = equalWeights();
  default_weights(0) = 10.0;
  const double default_lift =
    std::abs((weightedPseudoInverse(j, default_weights) * x_ddot)(0));

  Eigen::VectorXd extreme_weights = equalWeights();
  extreme_weights(0) = 1000.0;
  const double extreme_lift =
    std::abs((weightedPseudoInverse(j, extreme_weights) * x_ddot)(0));

  EXPECT_LT(default_lift, unweighted_lift)
    << "the chosen default must reduce the lift's contribution relative to unweighted";
  EXPECT_GT(default_lift, extreme_lift)
    << "the chosen default must not already be as extreme as the 1000:1 case";
}
