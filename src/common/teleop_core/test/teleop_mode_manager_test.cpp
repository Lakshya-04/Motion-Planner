#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "teleop_core/teleop_mode_manager.hpp"

using teleop_core::Mode;
using teleop_core::Reason;
using teleop_core::TeleopModeConfig;
using teleop_core::TeleopModeManager;

namespace
{
TeleopModeManager inTeleop(double t = 0.0)
{
  TeleopModeManager m;
  EXPECT_TRUE(m.requestTeleop(true, t).accepted);
  return m;
}
}  // namespace

TEST(TeleopModeManager, StartsInAuto)
{
  TeleopModeManager m;
  EXPECT_EQ(m.mode(), Mode::kAuto);
  EXPECT_EQ(m.refusedCount(), 0u);
}

TEST(TeleopModeManager, AutoToHoldOnStop)
{
  TeleopModeManager m;
  auto r = m.onStop(false, 1.0);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.mode, Mode::kHold);
  EXPECT_EQ(r.reason, Reason::kStopRequested);
}

TEST(TeleopModeManager, AutoToHoldOnSafetyHasSafetyReason)
{
  TeleopModeManager m;
  EXPECT_EQ(m.onStop(true, 1.0).reason, Reason::kSafetyStop);
  EXPECT_EQ(m.mode(), Mode::kHold);
}

TEST(TeleopModeManager, AutoToTeleopRefusedWhileSegmentRunning)
{
  TeleopModeManager m;
  auto r = m.requestTeleop(false, 0.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, Reason::kSegmentStillRunning);
  EXPECT_EQ(m.mode(), Mode::kAuto);
  EXPECT_EQ(m.refusedCount(), 1u);
}

TEST(TeleopModeManager, AutoToTeleopWhenHalted)
{
  TeleopModeManager m;
  auto r = m.requestTeleop(true, 0.0);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.mode, Mode::kTeleop);
  EXPECT_EQ(r.reason, Reason::kOperatorRequest);
  EXPECT_TRUE(m.needsReplan());
}

TEST(TeleopModeManager, TeleopToHoldOnStopAndSafety)
{
  auto m = inTeleop();
  EXPECT_EQ(m.onStop(true, 1.0).mode, Mode::kHold);
  auto n = inTeleop();
  EXPECT_EQ(n.onStop(false, 1.0).reason, Reason::kStopRequested);
}

TEST(TeleopModeManager, TeleopToAutoNeedsConfirm)
{
  auto m = inTeleop();
  auto r = m.requestAuto(false, true, 1.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, Reason::kNeedsConfirm);
  EXPECT_EQ(m.mode(), Mode::kTeleop);
}

TEST(TeleopModeManager, TeleopToAutoNeedsReplanAck)
{
  auto m = inTeleop();
  auto r = m.requestAuto(true, false, 1.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, Reason::kNeedsReplanAck);
  EXPECT_EQ(m.mode(), Mode::kTeleop);
}

TEST(TeleopModeManager, TeleopToAutoWithBoth)
{
  auto m = inTeleop();
  auto r = m.requestAuto(true, true, 1.0);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.mode, Mode::kAuto);
  EXPECT_EQ(r.reason, Reason::kOperatorConfirmed);
  EXPECT_FALSE(m.needsReplan());
}

TEST(TeleopModeManager, HoldToTeleopByRequest)
{
  TeleopModeManager m;
  m.onStop(false, 0.0);
  auto r = m.requestTeleop(false, 1.0);  // halted flag irrelevant in HOLD
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.mode, Mode::kTeleop);
}

TEST(TeleopModeManager, HoldFromAutoToAutoByRequestOnly)
{
  TeleopModeManager m;
  m.onStop(false, 0.0);
  auto r = m.requestAuto(false, false, 1.0);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.mode, Mode::kAuto);
}

TEST(TeleopModeManager, HoldAfterTeleopToAutoNeedsReplanAck)
{
  auto m = inTeleop();
  m.onStop(false, 1.0);
  auto r = m.requestAuto(false, false, 2.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, Reason::kNeedsReplanAck);
  EXPECT_TRUE(m.requestAuto(false, true, 3.0).accepted);
}

TEST(TeleopModeManager, TimeoutFallsBackToHold)
{
  TeleopModeConfig c;
  c.input_timeout_s = 2.0;
  TeleopModeManager m(c);
  m.requestTeleop(true, 10.0);
  EXPECT_FALSE(m.tick(11.9).accepted);
  EXPECT_EQ(m.mode(), Mode::kTeleop);
  auto r = m.tick(12.1);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.mode, Mode::kHold);
  EXPECT_EQ(r.reason, Reason::kInputTimeout);
}

TEST(TeleopModeManager, InputRefreshesTimeout)
{
  TeleopModeManager m;
  m.requestTeleop(true, 0.0);
  m.noteInput(1.5);
  m.noteInput(3.0);
  EXPECT_EQ(m.tick(4.5).mode, Mode::kTeleop);
  EXPECT_EQ(m.tick(5.1).mode, Mode::kHold);
}

TEST(TeleopModeManager, TickNeverChangesAutoOrHold)
{
  TeleopModeManager m;
  EXPECT_FALSE(m.tick(1000.0).accepted);
  EXPECT_EQ(m.mode(), Mode::kAuto);
  m.onStop(false, 0.0);
  EXPECT_FALSE(m.tick(1000.0).accepted);
  EXPECT_EQ(m.mode(), Mode::kHold);
}

TEST(TeleopModeManager, NonFiniteTimeInTeleopFallsToHold)
{
  auto m = inTeleop();
  EXPECT_EQ(m.tick(std::nan("")).mode, Mode::kHold);
}

TEST(TeleopModeManager, SameModeRequestsAreNoOpsNotIllegal)
{
  TeleopModeManager m;
  EXPECT_EQ(m.requestAuto(true, true, 0.0).reason, Reason::kAlreadyInMode);
  m.requestTeleop(true, 0.0);
  EXPECT_EQ(m.requestTeleop(true, 0.0).reason, Reason::kAlreadyInMode);
  m.onStop(false, 0.0);
  EXPECT_EQ(m.onStop(true, 0.0).reason, Reason::kAlreadyInMode);
  EXPECT_EQ(m.refusedCount(), 0u);
}

TEST(TeleopModeManager, RefusedAndAcceptedAreCounted)
{
  TeleopModeManager m;
  m.requestTeleop(false, 0.0);     // refused
  m.requestTeleop(false, 0.1);     // refused
  m.requestTeleop(true, 0.2);      // accepted
  m.requestAuto(false, false, 0.3);  // refused
  m.requestAuto(true, true, 0.4);    // accepted
  EXPECT_EQ(m.refusedCount(), 3u);
  EXPECT_EQ(m.acceptedCount(), 2u);
}

TEST(TeleopModeManager, RefusalDoesNotChangeMode)
{
  auto m = inTeleop();
  m.requestAuto(false, false, 1.0);
  EXPECT_EQ(m.mode(), Mode::kTeleop);
}

TEST(TeleopModeManager, FullCycleAutoTeleopAuto)
{
  TeleopModeManager m;
  EXPECT_TRUE(m.requestTeleop(true, 0.0).accepted);
  EXPECT_TRUE(m.requestAuto(true, true, 1.0).accepted);
  EXPECT_EQ(m.mode(), Mode::kAuto);
  EXPECT_TRUE(m.requestTeleop(true, 2.0).accepted);  // again, needs_replan set afresh
  EXPECT_TRUE(m.needsReplan());
}

TEST(TeleopModeManager, DeterministicSequence)
{
  auto run = []() {
      TeleopModeManager m;
      std::size_t h = 0;
      auto mix = [&h](const teleop_core::TransitionResult & r) {
          h = h * 31 + static_cast<std::size_t>(r.mode) * 7 + static_cast<std::size_t>(r.reason) +
            (r.accepted ? 1000 : 0);
        };
      mix(m.requestTeleop(false, 0.0));
      mix(m.requestTeleop(true, 0.1));
      mix(m.tick(5.0));
      mix(m.requestAuto(true, false, 5.1));
      mix(m.requestAuto(true, true, 5.2));
      mix(m.onStop(true, 5.3));
      return h;
    };
  EXPECT_EQ(run(), run());
}

// ---- fail-safe time handling and safety-stop confirm (review fixes)

TEST(TeleopModeManager, BackwardsClockFallsToHold)
{
  auto m = inTeleop(100.0);
  const auto r = m.tick(1.0);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.reason, Reason::kInputTimeout);
  EXPECT_EQ(m.mode(), Mode::kHold);
}

TEST(TeleopModeManager, NanEntryTimeRefusedAndStaysSafe)
{
  TeleopModeManager m;
  const auto r = m.requestTeleop(true, std::nan(""));
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, Reason::kInvalidTime);
  EXPECT_EQ(m.mode(), Mode::kAuto);
  m.onStop(false, 0.0);
  EXPECT_FALSE(m.requestTeleop(true, std::nan("")).accepted);
  EXPECT_EQ(m.mode(), Mode::kHold);
  EXPECT_EQ(m.tick(1000.0).accepted, false);
}

TEST(TeleopModeManager, FutureInputStampDoesNotKeepTeleopAlive)
{
  auto m = inTeleop(0.0);
  m.noteInput(1e9);
  EXPECT_EQ(m.tick(100.0).reason, Reason::kInputTimeout);
  EXPECT_EQ(m.mode(), Mode::kHold);
}

TEST(TeleopModeManager, NanTickFallsToHold)
{
  auto m = inTeleop(0.0);
  EXPECT_EQ(m.tick(std::nan("")).reason, Reason::kInputTimeout);
  EXPECT_EQ(m.mode(), Mode::kHold);
}

TEST(TeleopModeManager, BadConfigThrows)
{
  for (double bad : {std::nan(""), std::numeric_limits<double>::infinity(), 0.0, -1.0}) {
    TeleopModeConfig c;
    c.input_timeout_s = bad;
    EXPECT_THROW(TeleopModeManager{c}, std::invalid_argument);
  }
}

TEST(TeleopModeManager, SafetyHoldToAutoNeedsConfirm)
{
  TeleopModeManager m;
  m.onStop(true, 0.0);
  auto r = m.requestAuto(false, false, 1.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, Reason::kNeedsConfirm);
  EXPECT_EQ(m.mode(), Mode::kHold);
  EXPECT_TRUE(m.requestAuto(true, false, 1.0).accepted);  // no teleop happened: no replan needed
}

TEST(TeleopModeManager, SafetyHoldFromTeleopNeedsConfirmAndReplanAck)
{
  auto m = inTeleop(0.0);
  m.onStop(true, 1.0);
  EXPECT_EQ(m.requestAuto(false, true, 2.0).reason, Reason::kNeedsConfirm);
  EXPECT_EQ(m.requestAuto(true, false, 2.0).reason, Reason::kNeedsReplanAck);
  EXPECT_TRUE(m.requestAuto(true, true, 2.0).accepted);
}

TEST(TeleopModeManager, PlainStopHoldToAutoStillNoConfirm)
{
  TeleopModeManager m;
  m.onStop(false, 0.0);
  EXPECT_TRUE(m.requestAuto(false, false, 1.0).accepted);
}
