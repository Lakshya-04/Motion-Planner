/**
 * @file test_trajectory.cpp
 * @brief Unit tests for core_types::Trajectory.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <gtest/gtest.h>

#include "core_types/trajectory.hpp"

TEST(Trajectory, DefaultsToEmpty)
{
  core_types::Trajectory trajectory;
  EXPECT_TRUE(trajectory.joint_names.empty());
  EXPECT_TRUE(trajectory.points.empty());
}

TEST(TrajectoryPoint, DefaultsToZeroTimeAndEmptyVectors)
{
  core_types::TrajectoryPoint point;
  EXPECT_TRUE(point.positions.empty());
  EXPECT_TRUE(point.velocities.empty());
  EXPECT_TRUE(point.accelerations.empty());
  EXPECT_DOUBLE_EQ(point.time_from_start_sec, 0.0);
}

TEST(Trajectory, FieldsAreIndependentlyAssignable)
{
  core_types::Trajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2"};

  core_types::TrajectoryPoint point;
  point.positions = {0.1, 0.2};
  point.velocities = {0.0, 0.0};
  point.accelerations = {0.0, 0.0};
  point.time_from_start_sec = 1.5;
  trajectory.points.push_back(point);

  ASSERT_EQ(trajectory.joint_names.size(), 2u);
  EXPECT_EQ(trajectory.joint_names[0], "joint1");
  EXPECT_EQ(trajectory.joint_names[1], "joint2");

  ASSERT_EQ(trajectory.points.size(), 1u);
  EXPECT_EQ(trajectory.points[0].positions, (std::vector<double>{0.1, 0.2}));
  EXPECT_DOUBLE_EQ(trajectory.points[0].time_from_start_sec, 1.5);
}
