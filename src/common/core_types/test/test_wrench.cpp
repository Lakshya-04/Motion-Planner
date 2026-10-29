/**
 * @file test_wrench.cpp
 * @brief Unit tests for core_types::Wrench and the force_stage helpers.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <gtest/gtest.h>

#include "core_types/force_stage.hpp"
#include "core_types/wrench.hpp"

TEST(Wrench, NormalForceIsNegatedForceZ)
{
  core_types::Wrench wrench;
  wrench.force_z = -15.0;  // pressing into the wall along tool +Z
  EXPECT_DOUBLE_EQ(core_types::normalForceN(wrench), 15.0);
}

TEST(Wrench, TangentialTorqueIgnoresTwistAboutNormal)
{
  core_types::Wrench wrench;
  wrench.torque_x = 3.0;
  wrench.torque_y = 4.0;
  wrench.torque_z = 100.0;  // twist about the contact normal -- must not contribute
  EXPECT_DOUBLE_EQ(core_types::tangentialTorqueNm(wrench), 5.0);
}

TEST(ForceStage, IsContactStageTrueOnlyForApproachAndCartesian)
{
  using core_types::ForceStage;
  using core_types::isContactStage;
  EXPECT_FALSE(isContactStage(ForceStage::kNone));
  EXPECT_TRUE(isContactStage(ForceStage::kApproach));
  EXPECT_TRUE(isContactStage(ForceStage::kCartesian));
  EXPECT_FALSE(isContactStage(ForceStage::kRetract));
}

TEST(ForceStage, WireValueDegradesUnknownToNone)
{
  using core_types::ForceStage;
  using core_types::forceStageFromWireValue;
  EXPECT_EQ(forceStageFromWireValue(0), ForceStage::kNone);
  EXPECT_EQ(forceStageFromWireValue(1), ForceStage::kApproach);
  EXPECT_EQ(forceStageFromWireValue(2), ForceStage::kCartesian);
  EXPECT_EQ(forceStageFromWireValue(3), ForceStage::kRetract);
  EXPECT_EQ(forceStageFromWireValue(4), ForceStage::kNone);
  EXPECT_EQ(forceStageFromWireValue(255), ForceStage::kNone);
}
