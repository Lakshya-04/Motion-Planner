#include <gtest/gtest.h>

#include "motion_planner_moveit/start_state_source.hpp"

using motion_planner_moveit::chooseStartStateSource;
using motion_planner_moveit::StartStateSource;

TEST(StartStateSource, FreshMonitorIsUsed)
{
  EXPECT_EQ(chooseStartStateSource(true, true), StartStateSource::kPlanningSceneMonitor);
}

TEST(StartStateSource, StaleMonitorFallsBackToMoveGroup)
{
  EXPECT_EQ(chooseStartStateSource(true, false), StartStateSource::kMoveGroup);
}

TEST(StartStateSource, MissingMonitorFallsBackToMoveGroup)
{
  EXPECT_EQ(chooseStartStateSource(false, true), StartStateSource::kMoveGroup);
  EXPECT_EQ(chooseStartStateSource(false, false), StartStateSource::kMoveGroup);
}
