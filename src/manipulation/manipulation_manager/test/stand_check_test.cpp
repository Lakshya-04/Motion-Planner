#include <gtest/gtest.h>
#include <limits>

#include "manipulation_manager/core/stand_check.hpp"

namespace manipulation_manager
{
namespace
{
// Fake values only: no measured navigator error or reach margin behind these numbers.
StandCheckConfig cfg()
{
  StandCheckConfig c;
  c.min_tol_m = 0.02;
  c.max_tol_m = 0.15;
  c.margin_scale = 0.5;
  c.fallback_tol_m = 0.10;
  c.yaw_tol_rad = 0.1;
  c.max_redocks = 2;
  return c;
}
}  // namespace

TEST(StandCheckTest, ToleranceFollowsTheMarginAndIsClamped)
{
  const StandCheck sc(cfg());
  EXPECT_NEAR(sc.toleranceM(0.12), 0.06, 1e-12);
  EXPECT_NEAR(sc.toleranceM(0.01), 0.02, 1e-12);   // floor
  EXPECT_NEAR(sc.toleranceM(1.0), 0.15, 1e-12);    // ceiling
  EXPECT_NEAR(sc.toleranceM(-1.0), 0.10, 1e-12);   // unknown margin
}

TEST(StandCheckTest, WithinToleranceGoes)
{
  const StandCheck sc(cfg());
  const auto r = sc.evaluate({0.05, -0.05}, 0.12, 0);
  EXPECT_EQ(r.decision, StandDecision::kGo);
  EXPECT_EQ(r.reason, StandReason::kWithinTolerance);
  EXPECT_FALSE(r.stand_failed);
}

TEST(StandCheckTest, SmallMarginTightensTheGate)
{
  const StandCheck sc(cfg());
  EXPECT_EQ(sc.evaluate({0.05, 0.0}, 0.12, 0).decision, StandDecision::kGo);
  const auto r = sc.evaluate({0.05, 0.0}, 0.06, 0);   // tol 0.03
  EXPECT_EQ(r.decision, StandDecision::kRedock);
  EXPECT_EQ(r.reason, StandReason::kPositionOutOfTolerance);
}

TEST(StandCheckTest, YawAndBothAreCodedApart)
{
  const StandCheck sc(cfg());
  EXPECT_EQ(sc.evaluate({0.01, 0.2}, 0.12, 0).reason, StandReason::kYawOutOfTolerance);
  EXPECT_EQ(sc.evaluate({0.01, -0.2}, 0.12, 0).reason, StandReason::kYawOutOfTolerance);
  EXPECT_EQ(sc.evaluate({0.10, 0.2}, 0.12, 0).reason, StandReason::kPositionAndYawOutOfTolerance);
}

TEST(StandCheckTest, RedocksThenAbortMarksTheStandFailed)
{
  const StandCheck sc(cfg());
  EXPECT_EQ(sc.evaluate({0.1, 0.0}, 0.12, 0).decision, StandDecision::kRedock);
  EXPECT_EQ(sc.evaluate({0.1, 0.0}, 0.12, 1).decision, StandDecision::kRedock);
  const auto r = sc.evaluate({0.1, 0.0}, 0.12, 2);
  EXPECT_EQ(r.decision, StandDecision::kAbort);
  EXPECT_EQ(r.reason, StandReason::kRedocksExhausted);
  EXPECT_TRUE(r.stand_failed);
  // a good arrival after re-docks still goes
  EXPECT_EQ(sc.evaluate({0.01, 0.0}, 0.12, 2).decision, StandDecision::kGo);
}

TEST(StandCheckTest, ZeroRedocksAbortsOnTheFirstMiss)
{
  auto c = cfg();
  c.max_redocks = 0;
  const auto r = StandCheck(c).evaluate({0.1, 0.0}, 0.12, 0);
  EXPECT_EQ(r.decision, StandDecision::kAbort);
  EXPECT_TRUE(r.stand_failed);
}

TEST(StandCheckTest, InvalidMeasurementAborts)
{
  const StandCheck sc(cfg());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(sc.evaluate({nan, 0.0}, 0.12, 0).reason, StandReason::kInvalidMeasurement);
  EXPECT_EQ(sc.evaluate({0.0, nan}, 0.12, 0).decision, StandDecision::kAbort);
  EXPECT_EQ(sc.evaluate({-0.01, 0.0}, 0.12, 0).decision, StandDecision::kAbort);
}

TEST(StandCheckTest, BadConfigIsSanitised)
{
  StandCheckConfig c;
  c.min_tol_m = 0.2;
  c.max_tol_m = 0.1;   // below min
  c.max_redocks = -3;
  const StandCheck sc(c);
  EXPECT_NEAR(sc.toleranceM(1.0), 0.2, 1e-12);
  EXPECT_EQ(sc.config().max_redocks, 0);
}

TEST(StandCheckTest, ReasonStringsAreStable)
{
  EXPECT_STREQ(toString(StandReason::kRedocksExhausted), "redocks_exhausted");
  EXPECT_STREQ(toString(StandReason::kWithinTolerance), "within_tolerance");
}

}  // namespace manipulation_manager
