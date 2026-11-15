/**
 * @file test_pose.cpp
 * @brief Unit tests for core_types::Pose.
 * @details Also serves as the litmus test that this package builds and runs
 *          its tests with ROS not sourced (the pure-core rule in
 *          docs/design_principles.md).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <gtest/gtest.h>

#include "core_types/pose.hpp"

TEST(Pose, DefaultsToIdentity)
{
  core_types::Pose pose;
  EXPECT_DOUBLE_EQ(pose.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.qx, 0.0);
  EXPECT_DOUBLE_EQ(pose.qy, 0.0);
  EXPECT_DOUBLE_EQ(pose.qz, 0.0);
  EXPECT_DOUBLE_EQ(pose.qw, 1.0);
}

TEST(Pose, FieldsAreIndependentlyAssignable)
{
  core_types::Pose pose;
  pose.x = 1.0;
  pose.y = 2.0;
  pose.z = 3.0;
  pose.qx = 0.1;
  pose.qy = 0.2;
  pose.qz = 0.3;
  pose.qw = 0.9;

  EXPECT_DOUBLE_EQ(pose.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.z, 3.0);
  EXPECT_DOUBLE_EQ(pose.qx, 0.1);
  EXPECT_DOUBLE_EQ(pose.qy, 0.2);
  EXPECT_DOUBLE_EQ(pose.qz, 0.3);
  EXPECT_DOUBLE_EQ(pose.qw, 0.9);
}
