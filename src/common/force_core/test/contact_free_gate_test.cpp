#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "force_core/contact_free_gate.hpp"

using force_core::ContactFreeGate;
using force_core::ContactFreeGateConfig;
using force_core::ForceFsmState;

namespace
{
ContactFreeGateConfig cfg()
{
  ContactFreeGateConfig c;
  c.free_below_n = 0.5;
  c.not_free_above_n = 1.0;
  c.debounce_ticks = 3;
  c.min_settle_s = 1.0;
  return c;
}

// Drive `n` updates at 10 ms spacing from t; returns last result and advances t.
bool run(ContactFreeGate & g, ForceFsmState s, double f, int n, double & t, bool motion = false)
{
  bool r = false;
  for (int i = 0; i < n; ++i) {
    t += 0.01;
    r = g.update(s, f, t, motion);
  }
  return r;
}
}  // namespace

TEST(ContactFreeGate, OnlyIdleAndApproachAreFreeStates)
{
  EXPECT_TRUE(ContactFreeGate::isFreeState(ForceFsmState::kIdle));
  EXPECT_TRUE(ContactFreeGate::isFreeState(ForceFsmState::kApproach));
  for (auto s : {ForceFsmState::kApproachContactConfirmed, ForceFsmState::kCartesianSettle,
      ForceFsmState::kCartesianFollow, ForceFsmState::kCartesianDone, ForceFsmState::kRetractSettle,
      ForceFsmState::kRetractFollow, ForceFsmState::kAborted})
  {
    EXPECT_FALSE(ContactFreeGate::isFreeState(s));
  }
}

TEST(ContactFreeGate, NotFreeBeforeSettleTimeThenFree)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 0.0, 50, t));  // 0.5 s < 1 s
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.0, 60, t));   // past 1 s
}

TEST(ContactFreeGate, ContactStatesAreNeverFreeEvenAtZeroForce)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.0, 200, t));
  for (auto s : {ForceFsmState::kApproachContactConfirmed, ForceFsmState::kCartesianSettle,
      ForceFsmState::kCartesianFollow, ForceFsmState::kCartesianDone, ForceFsmState::kRetractSettle,
      ForceFsmState::kRetractFollow, ForceFsmState::kAborted})
  {
    EXPECT_FALSE(run(g, s, 0.0, 300, t));
  }
}

TEST(ContactFreeGate, SettleTimeRestartsAfterRetractCompletes)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  run(g, ForceFsmState::kCartesianFollow, 5.0, 10, t);
  run(g, ForceFsmState::kRetractFollow, 0.0, 300, t);
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 0.0, 50, t));
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 0.0, 40, t));  // 0.9 s
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.0, 20, t));
}

TEST(ContactFreeGate, ApproachIsFreeAfterSettleAndLostOnContact)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  EXPECT_TRUE(run(g, ForceFsmState::kApproach, 0.1, 200, t));
  EXPECT_FALSE(run(g, ForceFsmState::kApproach, 3.0, 1, t));  // immediate loss
  EXPECT_FALSE(run(g, ForceFsmState::kApproachContactConfirmed, 3.0, 5, t));
}

TEST(ContactFreeGate, HysteresisBandKeepsFreeButDoesNotGrantIt)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 0.75, 300, t));  // in band, never became free
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.1, 3, t));
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.75, 50, t));    // stays free inside the band
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 1.2, 1, t));     // above the exit threshold
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 0.75, 50, t));   // band does not re-grant
}

TEST(ContactFreeGate, DebounceNeedsConsecutiveLowTicks)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  run(g, ForceFsmState::kIdle, 0.0, 150, t);
  g.update(ForceFsmState::kIdle, 2.0, t += 0.01);
  EXPECT_FALSE(g.update(ForceFsmState::kIdle, 0.0, t += 0.01));
  EXPECT_FALSE(g.update(ForceFsmState::kIdle, 0.0, t += 0.01));
  EXPECT_FALSE(g.update(ForceFsmState::kIdle, 0.9, t += 0.01));  // breaks the run
  EXPECT_FALSE(g.update(ForceFsmState::kIdle, 0.0, t += 0.01));
  EXPECT_FALSE(g.update(ForceFsmState::kIdle, 0.0, t += 0.01));
  EXPECT_TRUE(g.update(ForceFsmState::kIdle, 0.0, t += 0.01));
}

TEST(ContactFreeGate, NanAndInfForceAreNotFree)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  run(g, ForceFsmState::kIdle, 0.0, 200, t);
  EXPECT_TRUE(g.contactFree());
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, std::nan(""), 1, t));
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, std::numeric_limits<double>::infinity(), 1, t));
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 0.0, 2, t));  // debounce restarts
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.0, 1, t));
}

TEST(ContactFreeGate, CommandedMotionOnlyMattersWhenRequired)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.0, 200, t, true));  // default: ignored
  auto c = cfg();
  c.require_no_motion = true;
  ContactFreeGate h(c);
  t = 0.0;
  EXPECT_FALSE(run(h, ForceFsmState::kIdle, 0.0, 200, t, true));
  EXPECT_TRUE(run(h, ForceFsmState::kIdle, 0.0, 5, t, false));
}

TEST(ContactFreeGate, NegativeForceUsesMagnitude)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, -3.0, 200, t));
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, -0.1, 5, t));
}

TEST(ContactFreeGate, BackwardsOrNonFiniteTimeFailsSafe)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  run(g, ForceFsmState::kIdle, 0.0, 200, t);
  EXPECT_FALSE(g.update(ForceFsmState::kIdle, 0.0, t - 5.0));
  EXPECT_FALSE(g.update(ForceFsmState::kIdle, 0.0, std::nan("")));
  EXPECT_FALSE(g.contactFree());
}

TEST(ContactFreeGate, InvertedThresholdsAreRepaired)
{
  auto c = cfg();
  c.not_free_above_n = 0.1;  // below free_below_n
  ContactFreeGate g(c);
  double t = 0.0;
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.0, 200, t));
  EXPECT_TRUE(run(g, ForceFsmState::kIdle, 0.45, 5, t));  // band collapses to 0.5
}

TEST(ContactFreeGate, ResetClearsFree)
{
  ContactFreeGate g(cfg());
  double t = 0.0;
  run(g, ForceFsmState::kIdle, 0.0, 200, t);
  g.reset();
  EXPECT_FALSE(g.contactFree());
  t = 0.0;
  EXPECT_FALSE(run(g, ForceFsmState::kIdle, 0.0, 50, t));
}
