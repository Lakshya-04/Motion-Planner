#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "lift_hardware/core/mock_transport.hpp"

namespace lift_hardware
{
namespace
{

struct FakeClock
{
  double now{100.0};
  MockTransport::Clock fn() { return [this]() {return now;}; }
};

MockConfig config()
{
  MockConfig c;
  c.stroke_m = 0.8;
  c.max_velocity_mps = 0.1;
  c.max_acceleration_mps2 = 0.2;
  c.initial_height_m = 0.25;
  return c;
}

TEST(MockTransport, EverythingButConnectFailsWhileDisconnected)
{
  FakeClock clock;
  MockTransport t(config(), clock.fn());
  double stroke = 0.0;
  Reading r;
  EXPECT_EQ(t.read_stroke(stroke), Fault::kLinkLost);
  EXPECT_EQ(t.move_to(0.1), Fault::kLinkLost);
  EXPECT_EQ(t.stop(), Fault::kLinkLost);
  EXPECT_EQ(t.read(r), Fault::kLinkLost);
  EXPECT_EQ(t.connect(), Fault::kNone);
  EXPECT_EQ(t.read_stroke(stroke), Fault::kNone);
  EXPECT_DOUBLE_EQ(stroke, 0.8);
}

TEST(MockTransport, StartsWhereConfiguredAndReportsIdle)
{
  FakeClock clock;
  MockTransport t(config(), clock.fn());
  t.connect();
  Reading r;
  ASSERT_EQ(t.read(r), Fault::kNone);
  EXPECT_DOUBLE_EQ(r.height_m, 0.25);
  EXPECT_EQ(r.state, DeviceState::kIdle);
}

TEST(MockTransport, MovesAtTheLimitsAndArrivesExactly)
{
  FakeClock clock;
  MockTransport t(config(), clock.fn());
  t.connect();
  Reading r;
  t.read(r);
  ASSERT_EQ(t.move_to(0.55), Fault::kNone);
  double peak = 0.0;
  double previous = 0.25;
  bool moving_seen = false;
  for (int i = 0; i < 2000; ++i) {
    clock.now += 0.01;
    ASSERT_EQ(t.read(r), Fault::kNone);
    EXPECT_GE(r.height_m, previous - 1e-12);  // monotone
    peak = std::max(peak, (r.height_m - previous) / 0.01);
    previous = r.height_m;
    moving_seen = moving_seen || r.state == DeviceState::kMoving;
    if (r.state == DeviceState::kIdle) {
      break;
    }
  }
  EXPECT_TRUE(moving_seen);
  EXPECT_LE(peak, 0.1 + 1e-6);
  EXPECT_GT(peak, 0.09);
  EXPECT_DOUBLE_EQ(r.height_m, 0.55);
  EXPECT_EQ(r.state, DeviceState::kIdle);
}

TEST(MockTransport, RejectsTargetsOutsideTheTravelAndKeepsTheOldOne)
{
  FakeClock clock;
  MockTransport t(config(), clock.fn());
  t.connect();
  EXPECT_EQ(t.move_to(-0.01), Fault::kOutOfRange);
  EXPECT_EQ(t.move_to(0.81), Fault::kOutOfRange);
  EXPECT_EQ(t.move_to(std::nan("")), Fault::kOutOfRange);
  clock.now += 10.0;
  Reading r;
  t.read(r);
  EXPECT_DOUBLE_EQ(r.height_m, 0.25);
}

TEST(MockTransport, StopBrakesToARestShortOfTheOldTarget)
{
  FakeClock clock;
  MockTransport t(config(), clock.fn());
  t.connect();
  Reading r;
  t.read(r);
  t.move_to(0.8);
  for (int i = 0; i < 200; ++i) {  // 2 s: at full speed by now
    clock.now += 0.01;
    t.read(r);
  }
  const double at_stop = r.height_m;
  ASSERT_EQ(t.stop(), Fault::kNone);
  clock.now += 5.0;
  t.read(r);
  EXPECT_GT(r.height_m, at_stop);   // braking distance covered
  EXPECT_LT(r.height_m, at_stop + 0.1 * 0.1 / (2.0 * 0.2) + 0.01);
  EXPECT_LT(r.height_m, 0.8);
  EXPECT_EQ(r.state, DeviceState::kIdle);
}

TEST(MockTransport, ReconnectKeepsThePositionAndDoesNotJumpTime)
{
  FakeClock clock;
  MockTransport t(config(), clock.fn());
  t.connect();
  t.move_to(0.5);
  t.disconnect();
  clock.now += 1000.0;  // time passing while disconnected must not move the carriage
  t.connect();
  Reading r;
  t.read(r);
  EXPECT_NEAR(r.height_m, 0.25, 1e-3);
}

}  // namespace
}  // namespace lift_hardware
