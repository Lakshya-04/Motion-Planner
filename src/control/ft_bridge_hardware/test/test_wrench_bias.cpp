#include <gtest/gtest.h>

#include "ft_bridge_hardware/wrench_bias.hpp"

namespace ft_bridge_hardware
{
namespace
{

TEST(WrenchBias, ZeroSetpointIsANoOp)
{
  const WrenchArray raw = {1.0, 2.0, -3.0, 0.1, -0.2, 0.3};
  const WrenchArray setpoint = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  EXPECT_EQ(applyWrenchBias(raw, setpoint), raw);
}

TEST(WrenchBias, SubtractsComponentWise)
{
  const WrenchArray raw = {0.0, 0.0, -15.0, 0.0, 0.0, 0.0};
  const WrenchArray setpoint = {0.0, 0.0, -15.0, 0.0, 0.0, 0.0};

  // Pressing into the wall at exactly the target force -- after bias, the "measured" wrench
  // admittance_controller sees is zero, matching its own built-in zero-seeking law.
  const auto biased = applyWrenchBias(raw, setpoint);
  for (double component : biased) {
    EXPECT_DOUBLE_EQ(component, 0.0);
  }
}

TEST(WrenchBias, BelowTargetLeavesANegativeResidualOnTheForceAxis)
{
  // Free air (raw force_z = 0) with a -15N setpoint: residual = 0 - (-15) = +15, which
  // normalForceN()'s convention (force_z negated) would read as -15N of "missing" force --
  // i.e. still hungry for contact, exactly the seek-until-contact behavior this bias enables.
  const WrenchArray raw = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const WrenchArray setpoint = {0.0, 0.0, -15.0, 0.0, 0.0, 0.0};

  const auto biased = applyWrenchBias(raw, setpoint);
  EXPECT_DOUBLE_EQ(biased[2], 15.0);
}

}  // namespace
}  // namespace ft_bridge_hardware
