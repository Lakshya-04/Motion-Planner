#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "chain_helper.hpp"
#include "diff_ik_core/jacobian_quality.hpp"

using namespace diff_ik_core;
using namespace chain_test;

TEST(JacobianQuality, DiagonalSingularValues)
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 3);
  J.diagonal() << 4.0, 2.0, 0.5;
  const auto q = jacobianQuality(J);
  EXPECT_NEAR(q.sigma_min, 0.5, 1e-12);
  EXPECT_NEAR(q.sigma_max, 4.0, 1e-12);
  EXPECT_NEAR(q.condition, 8.0, 1e-12);
  EXPECT_NEAR(q.manipulability, 4.0, 1e-12);
}

TEST(JacobianQuality, AnalyticJacobianMatchesFiniteDifference)
{
  for (const Chain & c : {arm6(), arm7(), liftArm6()}) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(c.dof());
    for (int i = 0; i < c.dof(); ++i) {q(i) = 0.3 + 0.17 * i;}
    EXPECT_LT((c.jacobian(q) - c.numericJacobian(q)).cwiseAbs().maxCoeff(), 1e-4);
  }
}

TEST(JacobianQuality, PlanarTwoLinkManipulabilityIsAnalytic)
{
  const Chain c = planar({1.0, 0.7});
  const double q2 = 0.9;
  const Eigen::MatrixXd J = c.jacobian(vec({0.4, q2})).topRows(2);
  EXPECT_NEAR(jacobianQuality(J).manipulability, std::abs(1.0 * 0.7 * std::sin(q2)), 1e-9);
}

TEST(JacobianQuality, SingularArmHasInfiniteConditionAndZeroManipulability)
{
  const Chain c = planar({1.0, 0.7});
  const Eigen::MatrixXd J = c.jacobian(vec({0.4, 0.0})).topRows(2);   // fully stretched
  const auto q = jacobianQuality(J);
  EXPECT_LT(q.sigma_min, 1e-9);
  EXPECT_LT(q.manipulability, 1e-9);
  EXPECT_GT(q.condition, 1e8);
}

TEST(JacobianQuality, MoreRowsThanColumnsHasZeroSigmaMin)
{
  const Chain c = planar({1.0, 0.7});
  const Eigen::MatrixXd J = c.jacobian(vec({0.4, 0.9}));   // 6 x 2
  const auto q = jacobianQuality(J);
  EXPECT_EQ(q.sigma_min, 0.0);
  EXPECT_EQ(q.manipulability, 0.0);
  EXPECT_TRUE(std::isinf(q.condition));
  EXPECT_EQ(minSingularValue(J), 0.0);
}

TEST(JacobianQuality, EmptyJacobianIsAllZero)
{
  const auto q = jacobianQuality(Eigen::MatrixXd());
  EXPECT_EQ(q.sigma_max, 0.0);
  EXPECT_EQ(q.manipulability, 0.0);
}

TEST(DirectionalGain, DiagonalEllipsoidAxes)
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 2);
  J.diagonal() << 2.0, 1.0;
  EXPECT_NEAR(directionalGain(J, vec({1, 0})), 2.0, 1e-12);
  EXPECT_NEAR(directionalGain(J, vec({0, 3})), 1.0, 1e-12);   // direction is normalised
  // 45 degrees: 1 / sqrt(0.5/4 + 0.5/1)
  EXPECT_NEAR(directionalGain(J, vec({1, 1})), 1.0 / std::sqrt(0.5 / 4.0 + 0.5), 1e-12);
}

TEST(DirectionalGain, DirectionOutsideRangeIsZero)
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 2);
  J(0, 0) = 1.0;
  J(1, 1) = 1.0;
  EXPECT_EQ(directionalGain(J, vec({0, 0, 1})), 0.0);
  EXPECT_EQ(directionalGain(J, vec({0, 0, 0})), 0.0);
  EXPECT_EQ(directionalGain(J, vec({1, 0})), 0.0);   // wrong size
}

TEST(DirectionalGain, BoundedBySingularValuesAndGoodAlongSafeDirectionNearSingularity)
{
  const Chain c = planar({1.0, 0.7});
  const Eigen::MatrixXd J = c.jacobian(vec({0.4, 0.02})).topRows(2);   // nearly stretched
  const auto q = jacobianQuality(J);
  std::mt19937 rng(1);
  std::normal_distribution<double> nd;
  for (int k = 0; k < 50; ++k) {
    const double g = directionalGain(J, vec({nd(rng), nd(rng)}));
    EXPECT_GE(g, q.sigma_min - 1e-9);
    EXPECT_LE(g, q.sigma_max + 1e-9);
  }
  // Tangential to the stretched arm (perpendicular to the radial line) the gain stays large.
  const Eigen::Vector2d radial(std::cos(0.4 + 0.01), std::sin(0.4 + 0.01));
  const double g_tan = directionalGain(J, vec({-radial.y(), radial.x()}));
  const double g_rad = directionalGain(J, vec({radial.x(), radial.y()}));
  EXPECT_GT(g_tan, 10.0 * g_rad);
}

TEST(JointLimitMargin, ContinuousJointsAreIgnored)
{
  const double pi = M_PI;
  EXPECT_EQ(jointLimitMargin({0, 0}, {-pi, -pi}, {pi, pi}), 0.5);
  EXPECT_FALSE(isLimitedJoint(-pi, pi));
  EXPECT_FALSE(isLimitedJoint(0.0, 0.0));
  EXPECT_TRUE(isLimitedJoint(-1.0, 1.0));
}

TEST(JointLimitMargin, ShareOfRangeAndAtLimit)
{
  EXPECT_NEAR(jointLimitMargin({0.0, 0.9}, {-1.0, -1.0}, {1.0, 1.0}), 0.05, 1e-12);
  EXPECT_EQ(jointLimitMargin({1.0}, {-1.0}, {1.0}), 0.0);
  EXPECT_EQ(jointLimitMargin({2.0}, {-1.0}, {1.0}), 0.0);   // past the limit
  EXPECT_EQ(jointLimitMargin({0.0, 0.0}, {-1.0}, {1.0}), -1.0);   // size mismatch
}
