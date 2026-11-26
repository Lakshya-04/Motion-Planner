#include <gtest/gtest.h>

#include <limits>

#include "manipulation_manager/core/contact_connector.hpp"

using namespace manipulation_manager;

namespace
{
core_types::Pose at(double x, double y, double z)
{
  core_types::Pose p;
  p.x = x; p.y = y; p.z = z;
  p.qx = 0.0; p.qy = 0.0; p.qz = 0.0; p.qw = 1.0;
  return p;
}
const WallDirection kWall = WallDirection::from(0, 1, 0);   // wall on +y: strips run in x-z
}  // namespace

TEST(ContactConnector, AdjacentStripOnTheSamePlaneIsEligible)
{
  const auto d = decideContactConnector(true, true, true, true, at(0, 0.5, 1.8), at(0.10, 0.5, 1.8), kWall);
  EXPECT_TRUE(d.eligible) << d.reason;
  EXPECT_NEAR(d.step_m, 0.10, 1e-9);
}

TEST(ContactConnector, EveryGuardRefuses)
{
  const auto a = at(0, 0.5, 1.8), b = at(0.1, 0.5, 1.8);
  EXPECT_FALSE(decideContactConnector(false, true, true, true, a, b, kWall).eligible);
  EXPECT_FALSE(decideContactConnector(true, false, true, true, a, b, kWall).eligible);
  EXPECT_FALSE(decideContactConnector(true, true, false, true, a, b, kWall).eligible);
  EXPECT_FALSE(decideContactConnector(true, true, true, false, a, b, kWall).eligible);
}

TEST(ContactConnector, NonAdjacentStripIsRefused)
{
  const auto d = decideContactConnector(true, true, true, true, at(0, 0.5, 1.8), at(0.40, 0.5, 1.8), kWall);
  EXPECT_FALSE(d.eligible);
  EXPECT_EQ(d.reason, "strips not adjacent");
}

TEST(ContactConnector, StepAtTheLimitIsEligible)
{
  ConnectorConfig cfg;
  cfg.max_step_m = 0.15;
  EXPECT_TRUE(decideContactConnector(true, true, true, true, at(0, 0.5, 0), at(0.15, 0.5, 0), kWall, cfg).eligible);
}

TEST(ContactConnector, OtherWallPlaneIsRefused)
{
  const auto d = decideContactConnector(true, true, true, true, at(0, 0.5, 1.8), at(0.10, 0.6, 1.8), kWall);
  EXPECT_FALSE(d.eligible);
  EXPECT_EQ(d.reason, "not the same wall plane");
}

TEST(ContactConnector, DifferentToolOrientationIsRefused)
{
  auto b = at(0.1, 0.5, 1.8);
  b.qx = 1.0; b.qw = 0.0;   // half turn
  EXPECT_FALSE(decideContactConnector(true, true, true, true, at(0, 0.5, 1.8), b, kWall).eligible);
}

TEST(ContactConnector, NonFinitePoseIsRefused)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(decideContactConnector(true, true, true, true, at(0, 0.5, 1.8), at(nan, 0.5, 1.8), kWall).eligible);
}

TEST(ContactConnector, TargetSlidesInThePlaneOfTheStrokeEnd)
{
  const auto t = connectorTarget(at(0, 0.5, 1.8), at(0.10, 0.51, 0.3), kWall);
  EXPECT_NEAR(t.x, 0.10, 1e-12);
  EXPECT_NEAR(t.y, 0.50, 1e-12);   // the 1 cm offset along the normal is removed
  EXPECT_NEAR(t.z, 0.30, 1e-12);
}

TEST(ContactConnector, LogSummary)
{
  ConnectorLog log;
  EXPECT_EQ(log.summary(), "");
  log.used();
  log.fellBack("connector move failed");
  EXPECT_EQ(log.usedCount(), 1);
  EXPECT_EQ(log.fellBackCount(), 1);
  EXPECT_NE(log.summary().find("1 used, 1 fell back"), std::string::npos);
  EXPECT_NE(log.summary().find("connector move failed"), std::string::npos);
}
