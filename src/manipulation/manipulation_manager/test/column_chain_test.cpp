#include <gtest/gtest.h>

#include <limits>

#include "manipulation_manager/core/column_chain.hpp"

using namespace manipulation_manager;

TEST(ColumnChain, NeighbourWithinTheShiftChains)
{
  const auto d = decideChain(true, true, false, true, {0.94, 0.30, 1.0}, {0.84, 0.40, 1.0}, 0.4);
  EXPECT_TRUE(d.chain);
  EXPECT_NEAR(d.shift_m, 0.1414, 1e-3);
}

TEST(ColumnChain, OffByDefaultOrNotValidOrStuckOrNoApproach)
{
  const std::array<double, 3> a{0, 0, 0}, b{0.1, 0, 0};
  EXPECT_FALSE(decideChain(false, true, false, true, a, b, 0.4).chain);
  EXPECT_FALSE(decideChain(true, false, false, true, a, b, 0.4).chain);
  EXPECT_FALSE(decideChain(true, true, true, true, a, b, 0.4).chain);
  EXPECT_FALSE(decideChain(true, true, false, false, a, b, 0.4).chain);
}

TEST(ColumnChain, FarColumnDoesNotChain)
{
  EXPECT_FALSE(decideChain(true, true, false, true, {0, 0, 0}, {0.0, 0.9, 0.0}, 0.4).chain);
}

TEST(ColumnChain, NonFinitePoseNeverChains)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(decideChain(true, true, false, true, {0, 0, 0}, {nan, 0, 0}, 1e9).chain);
}

TEST(ColumnChain, ExactlyAtTheLimitChains)
{
  EXPECT_TRUE(decideChain(true, true, false, true, {0, 0, 0}, {0.4, 0, 0}, 0.4).chain);
}
