#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "core_types/command_guard.hpp"

using core_types::CommandGuard;
using core_types::CommandGuardConfig;
using core_types::GuardFault;

namespace
{
CommandGuardConfig cfg(int debounce = 3)
{
  CommandGuardConfig c;
  c.lower = {-1.0, -2.0};
  c.upper = {1.0, 2.0};
  c.debounce_cycles = debounce;
  return c;
}
const double kNan = std::numeric_limits<double>::quiet_NaN();
}  // namespace

TEST(CommandGuard, GoodCommandPassesThrough)
{
  CommandGuard g(cfg());
  auto r = g.apply({0.5, 1.0}, {0.4, 0.9});
  EXPECT_FALSE(r.bad);
  EXPECT_EQ(r.positions, (std::vector<double>{0.5, 1.0}));
}

TEST(CommandGuard, NanIsHeldAtLastGoodNotWritten)
{
  CommandGuard g(cfg());
  g.apply({0.5, 1.0}, {0.5, 1.0});
  auto r = g.apply({kNan, 1.0}, {0.5, 1.0});
  EXPECT_TRUE(r.bad);
  EXPECT_FALSE(r.faulted);
  EXPECT_EQ(r.code, GuardFault::NON_FINITE_COMMAND);
  EXPECT_EQ(r.positions, (std::vector<double>{0.5, 1.0}));
}

TEST(CommandGuard, HoldsMeasuredBeforeAnyGoodCommand)
{
  CommandGuard g(cfg());
  auto r = g.apply({kNan, 0.0}, {0.1, 0.2});
  EXPECT_EQ(r.positions, (std::vector<double>{0.1, 0.2}));
}

TEST(CommandGuard, LimitViolationHeld)
{
  CommandGuard g(cfg());
  g.apply({0.0, 0.0}, {0.0, 0.0});
  auto r = g.apply({1.5, 0.0}, {0.0, 0.0});
  EXPECT_TRUE(r.bad);
  EXPECT_EQ(r.code, GuardFault::LIMIT_VIOLATION);
  EXPECT_EQ(r.positions, (std::vector<double>{0.0, 0.0}));
}

TEST(CommandGuard, DebouncedLatchAfterThreeBadCycles)
{
  CommandGuard g(cfg(3));
  g.apply({0.0, 0.0}, {0.0, 0.0});
  EXPECT_FALSE(g.apply({9.0, 0.0}, {0.0, 0.0}).faulted);
  EXPECT_FALSE(g.apply({9.0, 0.0}, {0.0, 0.0}).faulted);
  auto r = g.apply({9.0, 0.0}, {0.0, 0.0});
  EXPECT_TRUE(r.faulted);
  EXPECT_TRUE(g.faulted());
  EXPECT_EQ(g.faultCode(), GuardFault::LIMIT_VIOLATION);
}

TEST(CommandGuard, GoodCycleClearsTheDebounceCount)
{
  CommandGuard g(cfg(3));
  g.apply({9.0, 0.0}, {0.0, 0.0});
  g.apply({9.0, 0.0}, {0.0, 0.0});
  g.apply({0.1, 0.0}, {0.0, 0.0});
  EXPECT_EQ(g.badCycles(), 0);
  g.apply({9.0, 0.0}, {0.0, 0.0});
  EXPECT_FALSE(g.faulted());
}

TEST(CommandGuard, LatchHoldsEvenWhenCommandsRecover)
{
  CommandGuard g(cfg(1));
  g.apply({0.3, 0.0}, {0.3, 0.0});
  g.apply({kNan, 0.0}, {0.3, 0.0});
  EXPECT_TRUE(g.faulted());
  auto r = g.apply({0.9, 0.0}, {0.3, 0.0});  // a good command is still refused
  EXPECT_TRUE(r.faulted);
  EXPECT_EQ(r.code, GuardFault::NON_FINITE_COMMAND);
  EXPECT_EQ(r.positions, (std::vector<double>{0.3, 0.0}));
  g.reset();
  EXPECT_FALSE(g.apply({0.9, 0.0}, {0.3, 0.0}).bad);
}

TEST(CommandGuard, DeadAxisIsNeverReportedHealthy)
{
  CommandGuard g(cfg(2));
  g.apply({0.0, 0.0}, {0.0, 0.0});
  g.apply({0.0, 0.0}, {0.0, kNan});
  auto r = g.apply({0.0, 0.0}, {0.0, kNan});
  EXPECT_TRUE(r.faulted);
  EXPECT_EQ(r.code, GuardFault::NON_FINITE_STATE);
}

TEST(CommandGuard, SizeMismatch)
{
  CommandGuard g(cfg(1));
  auto r = g.apply({0.0}, {0.0, 0.0});
  EXPECT_EQ(r.code, GuardFault::SIZE_MISMATCH);
  EXPECT_TRUE(r.faulted);
}

TEST(CommandGuard, NothingSafeToWriteIsEmpty)
{
  CommandGuard g(cfg());
  auto r = g.apply({kNan, 0.0}, {kNan, 0.0});
  EXPECT_TRUE(r.positions.empty());
}

TEST(CommandGuard, NoLimitsOnlyChecksFinite)
{
  CommandGuardConfig c;
  CommandGuard g(c);
  EXPECT_FALSE(g.apply({100.0}, {0.0}).bad);
  EXPECT_TRUE(g.apply({INFINITY}, {0.0}).bad);
}
