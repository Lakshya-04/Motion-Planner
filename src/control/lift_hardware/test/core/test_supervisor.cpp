#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "lift_hardware/core/device_transport.hpp"
#include "lift_hardware/core/lift_supervisor.hpp"
#include "lift_hardware/core/mock_transport.hpp"
#include "lift_hardware/core/scripted_transport.hpp"

namespace lift_hardware
{
namespace
{

constexpr double kDt = 0.02;

struct Rig
{
  explicit Rig(const LiftConfig & config = LiftConfig{}, double stroke = 0.8, double start = 0.3)
  {
    auto t = std::make_unique<ScriptedTransport>(stroke, start);
    transport = t.get();
    supervisor = std::make_unique<LiftSupervisor>(std::move(t), config);
  }

  /// One control cycle: poll, then command (when a target is given).
  void cycle(double target = std::nan(""))
  {
    supervisor->poll(now);
    supervisor->command(now, target);
    now += kDt;
  }

  void run(double seconds, double target = std::nan(""))
  {
    const int n = static_cast<int>(seconds / kDt);
    for (int i = 0; i < n; ++i) {
      cycle(target);
    }
  }

  void until_ready(double target = std::nan(""))
  {
    for (int i = 0; i < 100 && snap().phase != Phase::kReady; ++i) {
      cycle(target);
    }
    ASSERT_EQ(snap().phase, Phase::kReady);
  }

  const Snapshot & snap() const { return supervisor->snapshot(); }

  ScriptedTransport * transport{nullptr};
  std::unique_ptr<LiftSupervisor> supervisor;
  double now{0.0};
};

LiftConfig config()
{
  LiftConfig c;
  c.max_velocity_mps = 0.1;
  c.max_acceleration_mps2 = 0.2;
  c.send_period_s = 0.0;
  c.resend_deadband_m = 0.0;
  c.move_timeout_s = 2.0;
  c.home_timeout_s = 3.0;
  c.max_io_failures = 3;
  return c;
}

// ---- connect and home ------------------------------------------------------------------------

TEST(Supervisor, HomesOnConnectAndIgnoresCommandsUntilThen)
{
  Rig rig(config());
  rig.transport->set_follow_moves(false);
  rig.cycle(0.6);  // connects, starts homing
  EXPECT_EQ(rig.snap().phase, Phase::kHoming);
  EXPECT_EQ(rig.transport->connect_calls(), 1);
  ASSERT_EQ(rig.transport->moves().size(), 1u);
  EXPECT_DOUBLE_EQ(rig.transport->moves()[0], 0.0);  // the home height, not the 0.6 command
  rig.run(0.5, 0.6);
  EXPECT_EQ(rig.snap().phase, Phase::kHoming);
  EXPECT_FALSE(rig.snap().homed);
  EXPECT_EQ(rig.transport->moves().size(), 1u);  // commands did not reach the device

  rig.transport->set_height(0.0);
  rig.until_ready();
  EXPECT_TRUE(rig.snap().homed);
  EXPECT_EQ(rig.snap().fault, Fault::kNone);
}

TEST(Supervisor, HomesToTheConfiguredHeightAndWaitsForTheDeviceToBeIdle)
{
  LiftConfig c = config();
  c.home_height_m = 0.05;
  Rig rig(c);
  rig.until_ready();
  EXPECT_NEAR(rig.snap().height_m, 0.05, 1e-9);
  ASSERT_FALSE(rig.transport->moves().empty());
  EXPECT_DOUBLE_EQ(rig.transport->moves().front(), 0.05);
}

TEST(Supervisor, StrokeIsTheDeviceOneOrASmallerConfiguredOne)
{
  Rig a(config(), 0.8);
  a.cycle();
  EXPECT_DOUBLE_EQ(a.snap().stroke_m, 0.8);

  LiftConfig c = config();
  c.stroke_m = 0.5;
  Rig b(c, 0.8);
  b.cycle();
  EXPECT_DOUBLE_EQ(b.snap().stroke_m, 0.5);

  c.stroke_m = 1.2;  // configured above the device: the device wins
  Rig d(c, 0.8);
  d.cycle();
  EXPECT_DOUBLE_EQ(d.snap().stroke_m, 0.8);
}

TEST(Supervisor, ADeviceReportingNoStrokeIsNotUsable)
{
  Rig rig(config(), 0.0);
  rig.cycle();
  EXPECT_EQ(rig.snap().phase, Phase::kDisconnected);
  EXPECT_EQ(rig.snap().fault, Fault::kLinkLost);
  EXPECT_FALSE(rig.transport->connected());
}

TEST(Supervisor, HomingThatNeverArrivesHoldsWithATimeout)
{
  Rig rig(config());
  rig.transport->set_follow_moves(false);
  rig.run(4.0);
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);
  EXPECT_EQ(rig.snap().fault, Fault::kTimeout);
  EXPECT_GE(rig.transport->stop_calls(), 1);
  EXPECT_FALSE(rig.snap().homed);
}

TEST(Supervisor, AFailedFirstHomeCommandIsRetried)
{
  Rig rig(config());
  rig.transport->inject(Op::kMoveTo, Fault::kTimeout, 1);
  rig.until_ready();
  EXPECT_EQ(rig.transport->moves().size(), 1u);  // the retry got through
  EXPECT_TRUE(rig.snap().homed);
}

// ---- command shaping -------------------------------------------------------------------------

TEST(Supervisor, ClampsCommandsToTheStroke)
{
  Rig rig(config());
  rig.until_ready();
  rig.run(20.0, 5.0);
  EXPECT_DOUBLE_EQ(rig.supervisor->clamped_target_m(), 0.8);
  EXPECT_LE(*std::max_element(rig.transport->moves().begin(), rig.transport->moves().end()), 0.8);
  EXPECT_DOUBLE_EQ(rig.transport->moves().back(), 0.8);

  rig.run(20.0, -3.0);
  EXPECT_DOUBLE_EQ(rig.supervisor->clamped_target_m(), 0.0);
  EXPECT_GE(*std::min_element(rig.transport->moves().begin(), rig.transport->moves().end()), 0.0);
  EXPECT_DOUBLE_EQ(rig.transport->moves().back(), 0.0);
  EXPECT_EQ(rig.snap().phase, Phase::kReady);
}

TEST(Supervisor, RateLimitsWhatItSendsToTheDevice)
{
  Rig rig(config());
  rig.until_ready();
  const size_t before = rig.transport->moves().size();
  rig.run(12.0, 0.6);  // a step command of 0.6 m
  const auto & moves = rig.transport->moves();
  ASSERT_GT(moves.size(), before + 50);
  double previous = moves[before - 1];
  double previous_velocity = 0.0;
  double peak_velocity = 0.0;
  for (size_t i = before; i < moves.size(); ++i) {
    const double velocity = (moves[i] - previous) / kDt;
    EXPECT_LE(velocity, 0.1 + 1e-9);
    EXPECT_LE(std::abs(velocity - previous_velocity), 0.2 * kDt * 2.0 + 1e-9);
    peak_velocity = std::max(peak_velocity, velocity);
    previous_velocity = velocity;
    previous = moves[i];
  }
  EXPECT_NEAR(peak_velocity, 0.1, 1e-6);
  EXPECT_DOUBLE_EQ(moves.back(), 0.6);
  EXPECT_NEAR(rig.snap().height_m, 0.6, 1e-9);
}

TEST(Supervisor, NonFiniteCommandsAreIgnored)
{
  Rig rig(config());
  rig.until_ready();
  rig.run(0.2, 0.2);
  const double target = rig.supervisor->clamped_target_m();
  const size_t sent = rig.transport->moves().size();
  rig.supervisor->command(rig.now, std::nan(""));
  rig.supervisor->command(rig.now, INFINITY);
  EXPECT_EQ(rig.supervisor->clamped_target_m(), target);
  EXPECT_EQ(rig.transport->moves().size(), sent);
}

TEST(Supervisor, ResendsOnlyOnAMoveBeyondTheDeadbandAndPeriod)
{
  LiftConfig c = config();
  c.send_period_s = 0.1;
  c.resend_deadband_m = 0.001;
  Rig rig(c);
  rig.until_ready();
  rig.run(10.0, 0.3);
  const size_t settled = rig.transport->moves().size();
  rig.run(2.0, 0.3);  // same command, already there
  EXPECT_EQ(rig.transport->moves().size(), settled);
  rig.run(1.0, 0.6);
  // at most one send per 0.1 s period
  EXPECT_LE(rig.transport->moves().size() - settled, 11u);
}

TEST(Supervisor, EstimatesVelocityFromTheMeasurements)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->set_follow_moves(false);
  double height = rig.snap().height_m;
  for (int i = 0; i < 100; ++i) {  // 2 s at 0.05 m/s
    height += 0.05 * kDt;
    rig.transport->set_height(height);
    rig.cycle(0.7);
  }
  EXPECT_NEAR(rig.snap().velocity_mps, 0.05, 0.005);
}

TEST(Supervisor, ReadyWithAMockDeviceFollowsThroughMotion)
{
  double clock = 0.0;
  MockConfig mock;
  mock.stroke_m = 0.8;
  mock.initial_height_m = 0.4;
  LiftConfig c;
  c.max_velocity_mps = mock.max_velocity_mps;
  c.max_acceleration_mps2 = mock.max_acceleration_mps2;
  auto transport = std::make_unique<MockTransport>(mock, [&clock]() {return clock;});
  LiftSupervisor sup(std::move(transport), c);
  auto cycle = [&](double target) {
    sup.poll(clock);
    sup.command(clock, target);
    clock += 0.01;
  };
  for (int i = 0; i < 6000 && sup.snapshot().phase != Phase::kReady; ++i) {
    cycle(0.5);
  }
  ASSERT_EQ(sup.snapshot().phase, Phase::kReady);  // homed from 0.4 down to 0 at <= 0.1 m/s
  EXPECT_GT(clock, 4.0);
  for (int i = 0; i < 1500; ++i) {
    cycle(0.5);
  }
  EXPECT_NEAR(sup.snapshot().height_m, 0.5, c.tolerance_m);
  EXPECT_EQ(sup.snapshot().phase, Phase::kReady);
}

// ---- faults ----------------------------------------------------------------------------------

TEST(Supervisor, ADeviceFaultStopsHoldsAndReports)
{
  Rig rig(config());
  rig.until_ready();
  rig.run(1.0, 0.5);
  const int stops = rig.transport->stop_calls();
  rig.transport->set_device_fault(Fault::kDevice);
  rig.cycle(0.7);
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);
  EXPECT_EQ(rig.snap().fault, Fault::kDevice);
  EXPECT_TRUE(rig.snap().stop_confirmed);
  EXPECT_EQ(rig.transport->stop_calls(), stops + 1);
  EXPECT_DOUBLE_EQ(rig.snap().velocity_mps, 0.0);

  rig.cycle();  // the held lift settles on where the device really stands
  const size_t sent = rig.transport->moves().size();
  const double height = rig.snap().height_m;
  rig.run(1.0, 0.1);
  EXPECT_EQ(rig.transport->moves().size(), sent);  // commands ignored while held
  EXPECT_DOUBLE_EQ(rig.snap().height_m, height);
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);
}

TEST(Supervisor, ResetClearsTheFaultAndHomesAgain)
{
  Rig rig(config());
  rig.until_ready();
  rig.run(1.0, 0.5);
  rig.transport->set_device_fault(Fault::kDevice);
  rig.cycle();
  ASSERT_EQ(rig.snap().phase, Phase::kHeld);

  EXPECT_TRUE(rig.supervisor->reset_fault(rig.now));  // the fault is still active on the device
  rig.cycle();
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);           // and is seen again at once
  EXPECT_EQ(rig.snap().fault, Fault::kDevice);

  rig.transport->set_device_fault(Fault::kNone);
  EXPECT_TRUE(rig.supervisor->reset_fault(rig.now));
  EXPECT_EQ(rig.snap().phase, Phase::kHoming);
  EXPECT_FALSE(rig.snap().homed);
  rig.until_ready();
  EXPECT_NEAR(rig.snap().height_m, 0.0, 1e-9);
  EXPECT_EQ(rig.snap().fault, Fault::kNone);
}

TEST(Supervisor, ResetOutsideAHoldDoesNothing)
{
  Rig rig(config());
  EXPECT_FALSE(rig.supervisor->reset_fault(0.0));
  rig.until_ready();
  EXPECT_FALSE(rig.supervisor->reset_fault(rig.now));
  EXPECT_EQ(rig.snap().phase, Phase::kReady);
}

TEST(Supervisor, AnOutOfRangeReadingHolds)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->set_follow_moves(false);
  rig.transport->set_height(0.95);  // beyond the 0.8 m stroke
  rig.cycle(0.5);
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);
  EXPECT_EQ(rig.snap().fault, Fault::kOutOfRange);
  EXPECT_GE(rig.transport->stop_calls(), 1);
}

TEST(Supervisor, ADeviceThatRejectsATargetHolds)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->inject(Op::kMoveTo, Fault::kOutOfRange, 1);
  rig.run(0.5, 0.5);
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);
  EXPECT_EQ(rig.snap().fault, Fault::kOutOfRange);
}

TEST(Supervisor, ADeviceThatNeverArrivesTimesOutAfterTheReferenceGetsThere)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->set_follow_moves(false);  // frozen at 0
  // Reference needs 0.2 m / 0.1 m/s plus ramps, about 2.5 s; then move_timeout_s = 2 s more.
  rig.run(3.5, 0.2);
  EXPECT_EQ(rig.snap().phase, Phase::kReady);
  rig.run(2.5, 0.2);
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);
  EXPECT_EQ(rig.snap().fault, Fault::kTimeout);
}

TEST(Supervisor, ASlowButArrivingDeviceDoesNotTimeOut)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->set_follow_moves(false);
  std::vector<double> reference_history;
  for (int i = 0; i < 600; ++i) {  // the device trails the reference by 1 s, inside the allowance
    const size_t lag = 50;
    rig.transport->set_height(reference_history.size() > lag ?
      reference_history[reference_history.size() - lag] : 0.0);
    rig.cycle(0.2);
    reference_history.push_back(rig.snap().reference_m);
  }
  EXPECT_EQ(rig.snap().phase, Phase::kReady);
}

TEST(Supervisor, StopIsRetriedUntilTheDeviceAcceptsIt)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->inject(Op::kStop, Fault::kTimeout, 2);
  rig.transport->set_device_fault(Fault::kDevice);
  rig.cycle();
  EXPECT_EQ(rig.snap().phase, Phase::kHeld);
  EXPECT_FALSE(rig.snap().stop_confirmed);
  rig.cycle();
  EXPECT_FALSE(rig.snap().stop_confirmed);
  rig.cycle();
  EXPECT_TRUE(rig.snap().stop_confirmed);
  const int stops = rig.transport->stop_calls();
  rig.run(0.5);
  EXPECT_EQ(rig.transport->stop_calls(), stops);  // no more once accepted
}

TEST(Supervisor, AStopThatNeverGoesThroughEndsInALostLink)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->inject(Op::kStop, Fault::kTimeout, -1);
  rig.transport->set_device_fault(Fault::kDevice);
  rig.run(0.5);
  EXPECT_EQ(rig.snap().phase, Phase::kDisconnected);
  EXPECT_EQ(rig.snap().fault, Fault::kLinkLost);
}

// ---- link loss and reconnect -----------------------------------------------------------------

TEST(Supervisor, ShortRunsOfTimeoutsAreRiddenOut)
{
  Rig rig(config());  // max_io_failures = 3
  rig.until_ready();
  rig.transport->inject(Op::kRead, Fault::kTimeout, 2);
  rig.run(0.2, 0.2);
  EXPECT_EQ(rig.snap().phase, Phase::kReady);
  EXPECT_EQ(rig.transport->disconnect_calls(), 0);
}

TEST(Supervisor, ARunOfTimeoutsLosesTheLink)
{
  Rig rig(config());
  rig.until_ready();
  rig.transport->inject(Op::kRead, Fault::kTimeout, 3);
  rig.cycle();
  rig.cycle();
  EXPECT_EQ(rig.snap().phase, Phase::kReady);
  rig.cycle();
  EXPECT_EQ(rig.snap().phase, Phase::kDisconnected);
  EXPECT_EQ(rig.snap().fault, Fault::kLinkLost);
  EXPECT_FALSE(rig.snap().homed);
  EXPECT_DOUBLE_EQ(rig.snap().velocity_mps, 0.0);
}

TEST(Supervisor, ALinkLostReportIsActedOnAtOnceAndHomesAgainAfterReconnecting)
{
  Rig rig(config());
  rig.until_ready();
  rig.run(1.0, 0.4);
  rig.transport->inject(Op::kRead, Fault::kLinkLost, 1);
  rig.cycle(0.4);
  EXPECT_EQ(rig.snap().phase, Phase::kDisconnected);
  EXPECT_FALSE(rig.transport->connected());
  const int connects = rig.transport->connect_calls();

  rig.run(1.0);  // the first retry is due after reconnect_initial_s = 0.5 s
  EXPECT_GT(rig.transport->connect_calls(), connects);
  rig.until_ready();
  EXPECT_TRUE(rig.snap().homed);
  EXPECT_NEAR(rig.snap().height_m, 0.0, 1e-9);  // went home again, not straight back to 0.4
  EXPECT_EQ(rig.snap().fault, Fault::kNone);
}

TEST(Supervisor, ReconnectDelaysGrowThenCap)
{
  LiftConfig c = config();  // 0.5 s, factor 2, cap 8 s
  Rig rig(c);
  rig.transport->inject(Op::kConnect, Fault::kTimeout, -1);
  std::vector<double> attempts;
  int seen = 0;
  for (int i = 0; i < 4000; ++i) {
    rig.cycle();
    if (rig.transport->connect_calls() != seen) {
      seen = rig.transport->connect_calls();
      attempts.push_back(rig.now - kDt);
    }
  }
  ASSERT_GE(attempts.size(), 7u);
  const double expected_gaps[] = {0.5, 1.0, 2.0, 4.0, 8.0, 8.0};
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(attempts[i + 1] - attempts[i], expected_gaps[i], kDt + 1e-9) << i;
  }
  EXPECT_EQ(rig.snap().phase, Phase::kDisconnected);
}

TEST(Supervisor, TheDelayStartsOverAfterASuccessfulConnect)
{
  Rig rig(config());
  rig.transport->inject(Op::kConnect, Fault::kTimeout, 3);  // gaps 0.5, 1, 2 then success
  rig.run(6.0);
  ASSERT_NE(rig.snap().phase, Phase::kDisconnected);
  rig.until_ready();
  const int connects = rig.transport->connect_calls();
  rig.transport->inject(Op::kRead, Fault::kLinkLost, 1);
  rig.transport->inject(Op::kConnect, Fault::kTimeout, 1);
  rig.cycle();
  ASSERT_EQ(rig.snap().phase, Phase::kDisconnected);
  const double lost_at = rig.now;
  double first_attempt = 0.0;
  double second_attempt = 0.0;
  for (int i = 0; i < 400 && second_attempt == 0.0; ++i) {
    rig.cycle();
    const int calls = rig.transport->connect_calls() - connects;
    if (calls >= 1 && first_attempt == 0.0) {
      first_attempt = rig.now - kDt;
    }
    if (calls >= 2) {
      second_attempt = rig.now - kDt;
    }
  }
  EXPECT_NEAR(first_attempt - lost_at, 0.5, 2 * kDt);  // not the 4 s a grown delay would be
  EXPECT_NEAR(second_attempt - first_attempt, 1.0, 2 * kDt);  // the loss itself grew it once
}

TEST(Supervisor, TheUnimplementedDeviceSkeletonNeverMovesAndSaysWhy)
{
  LiftSupervisor sup(std::make_unique<DeviceTransport>(), config());
  for (int i = 0; i < 100; ++i) {
    sup.poll(i * kDt);
    sup.command(i * kDt, 0.5);
  }
  EXPECT_EQ(sup.snapshot().phase, Phase::kDisconnected);
  EXPECT_EQ(sup.snapshot().fault, Fault::kUnsupported);
  EXPECT_FALSE(sup.snapshot().homed);
}

TEST(Supervisor, ShutdownStopsDisconnectsAndGoesQuiet)
{
  Rig rig(config());
  rig.until_ready();
  const int stops = rig.transport->stop_calls();
  rig.supervisor->shutdown();
  EXPECT_EQ(rig.transport->stop_calls(), stops + 1);
  EXPECT_FALSE(rig.transport->connected());
  const int connects = rig.transport->connect_calls();
  const size_t sent = rig.transport->moves().size();
  rig.run(2.0, 0.5);
  EXPECT_EQ(rig.transport->connect_calls(), connects);
  EXPECT_EQ(rig.transport->moves().size(), sent);
  EXPECT_FALSE(rig.snap().homed);
}

}  // namespace
}  // namespace lift_hardware
