#include <gtest/gtest.h>

#include <limits>

#include "compliant_trajectory_controller/integration_period.hpp"

using compliant_trajectory_controller::cappedIntegrationPeriod;

TEST(IntegrationPeriod, NominalCycleIsUnchanged)
{
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(0.01, 0.015), 0.01);
}

TEST(IntegrationPeriod, OverrunIsCapped)
{
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(0.022, 0.015), 0.015);
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(1.0, 0.015), 0.015);
}

TEST(IntegrationPeriod, CapOffLeavesThePeriodAlone)
{
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(0.05, 0.0), 0.05);
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(0.05, -1.0), 0.05);
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(0.05, std::numeric_limits<double>::quiet_NaN()), 0.05);
}

TEST(IntegrationPeriod, BadPeriodNeverRunsBackwards)
{
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(-0.01, 0.015), 0.0);
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(std::numeric_limits<double>::quiet_NaN(), 0.015), 0.0);
  EXPECT_DOUBLE_EQ(cappedIntegrationPeriod(std::numeric_limits<double>::infinity(), 0.015), 0.0);
}
