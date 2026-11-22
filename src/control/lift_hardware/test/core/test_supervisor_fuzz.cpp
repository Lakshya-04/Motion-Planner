// Property / fuzz test of LiftSupervisor against a faulty simulated device: seeded random command streams (NaN, inf,
// out of range), random transport faults, link drops, device faults, wild readings and clock jumps.
// Invariants, checked every cycle:
//   - every height sent to the device is finite and inside [0, min(configured stroke, device stroke)]
//   - the shaped reference stays inside [0, stroke] and moves no faster than max_velocity (clock gap capped at 0.25 s)
//   - while Held no move is sent (a stop wins), a held supervisor keeps stopping until a stop is accepted,
//     and once a stop is confirmed the device is not moving
//   - command() outside Ready, after shutdown, or with a non-finite height sends nothing
//   - published height / velocity / stroke are finite; Ready implies homed
//   - liveness: once the faults stop, the supervisor (with reset_fault when held) reaches Ready, homed, and follows a command
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "fuzz_util.hpp"
#include "lift_hardware/core/lift_config.hpp"
#include "lift_hardware/core/lift_supervisor.hpp"
#include "lift_hardware/core/lift_transport.hpp"
#include "lift_hardware/core/profile.hpp"
#include "lift_hardware/core/scripted_transport.hpp"

namespace lift_hardware
{
namespace
{
using fuzz::kCases;
using fuzz::Rng;

/// A faulty device: moves toward its target a share per read, drops calls, reports wild values.
struct FaultyDevice : LiftTransport
{
  explicit FaultyDevice(Rng * r) : rng(r) {}

  Fault roll(double scale = 1.0)
  {
    if (!faults_on || !rng->chance(p_fault * scale)) {return Fault::kNone;}
    switch (rng->integer(0, 4)) {
      case 0: return Fault::kTimeout;
      case 1: return Fault::kTimeout;
      case 2: return Fault::kLinkLost;
      case 3: return Fault::kDevice;
      default: return rng->chance(0.3) ? Fault::kUnsupported : Fault::kOutOfRange;
    }
  }
  Fault connect() override
  {
    ++connects;
    const Fault f = roll();
    if (f != Fault::kNone) {return f;}
    connected = true;
    return Fault::kNone;
  }
  void disconnect() override
  {
    connected = false;
    ++calls_after_disconnect_ok;
  }
  Fault read_stroke(double & s) override
  {
    if (!connected) {return Fault::kLinkLost;}
    const Fault f = roll();
    if (f != Fault::kNone) {return f;}
    stroke = faults_on && rng->chance(0.05) ? rng->wild(0.0, 0.0, 1.0) : stroke_nominal * rng->uni(0.7, 1.0) ;
    if (!faults_on) {stroke = stroke_nominal;}
    s = stroke;
    return Fault::kNone;
  }
  Fault move_to(double h) override
  {
    if (!connected) {return Fault::kLinkLost;}
    ++move_calls;
    sent.push_back(h);
    if (!std::isfinite(h) || h < 0.0 || h > stroke + 1e-12) {++bad_moves;}
    const Fault f = roll();
    if (f != Fault::kNone) {return f;}
    target = h;
    moving = true;
    return Fault::kNone;
  }
  Fault stop() override
  {
    ++stop_calls;
    if (!connected) {return Fault::kLinkLost;}
    const Fault f = roll(2.0);
    if (f != Fault::kNone) {return f;}
    moving = false;
    return Fault::kNone;
  }
  Fault read(Reading & r) override
  {
    if (!connected) {return Fault::kLinkLost;}
    const Fault f = roll();
    if (f != Fault::kNone) {return f;}
    if (moving) {
      height += 0.5 * (target - height);
      if (std::fabs(target - height) < 1e-4) {
        height = target;
        moving = false;
      }
    }
    r.height_m = height;
    r.state = moving ? DeviceState::kMoving : DeviceState::kIdle;
    r.fault = Fault::kNone;
    if (faults_on && rng->chance(0.01)) {r.height_m = rng->wild(-0.2, 2.0, 0.5);}
    if (faults_on && rng->chance(0.01)) {
      r.state = DeviceState::kFaulted;
      r.fault = rng->chance(0.5) ? Fault::kDevice : Fault::kNone;
    }
    return Fault::kNone;
  }

  Rng * rng;
  bool faults_on{true};
  double p_fault{0.03};
  double stroke_nominal{0.8};
  double stroke{0.8};
  double height{0.3}, target{0.3};
  bool connected{false}, moving{false};
  int connects{0}, move_calls{0}, stop_calls{0}, bad_moves{0}, calls_after_disconnect_ok{0};
  std::vector<double> sent;
};

LiftConfig randomConfig(Rng & rng)
{
  LiftConfig c;
  c.stroke_m = rng.chance(0.4) ? rng.uni(0.2, 1.2) : 0.0;
  c.max_velocity_mps = rng.uni(0.02, 0.5);
  c.max_acceleration_mps2 = rng.uni(0.05, 1.0);
  c.home_height_m = rng.uni(0.0, 1.2);
  if (c.stroke_m > 0.0) {c.home_height_m = std::min(c.home_height_m, c.stroke_m);}
  c.home_timeout_s = rng.uni(2.0, 60.0);
  c.tolerance_m = rng.uni(0.001, 0.02);
  c.move_timeout_s = rng.uni(1.0, 10.0);
  c.send_period_s = rng.chance(0.3) ? 0.0 : rng.uni(0.0, 0.2);
  c.resend_deadband_m = rng.uni(0.0, 0.002);
  c.reconnect_initial_s = rng.uni(0.05, 1.0);
  c.reconnect_max_s = c.reconnect_initial_s + rng.uni(0.0, 8.0);
  c.reconnect_factor = rng.uni(1.0, 3.0);
  c.max_io_failures = rng.integer(1, 6);
  c.velocity_filter_s = rng.uni(0.0, 0.3);
  return c;
}
}  // namespace

TEST(SupervisorFuzz, InvariantsHoldUnderRandomFaultsAndCommands)
{
  Rng rng(0x11F7001);
  const int cases = 6000;   // 6000 cases x ~300 cycles = 1.8M supervisor cycles (kept under 5 s with the sanitizers)
  long cycles = 0, held_cycles = 0, ready_cycles = 0, reconnects = 0, resets = 0;
  for (int c = 0; c < cases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    const LiftConfig cfg = randomConfig(rng);
    ASSERT_EQ(validate(cfg), "");
    auto dev_owner = std::make_unique<FaultyDevice>(&rng);
    FaultyDevice * dev = dev_owner.get();
    dev->p_fault = rng.chance(0.3) ? 0.0 : rng.uni(0.0, 0.12);
    dev->stroke_nominal = rng.uni(0.3, 1.5);
    dev->height = rng.uni(0.0, dev->stroke_nominal);
    dev->target = dev->height;
    LiftSupervisor sup(std::move(dev_owner), cfg);

    double now = rng.uni(0.0, 100.0);
    double target = rng.uni(0.0, 1.0);
    bool prev_ready = false;
    double prev_ref = 0.0, last_cmd_t = 0.0;
    size_t held_moves = 0;
    bool was_held = false;
    int prev_stops = 0;
    bool prev_unconfirmed = false;
    Phase prev_phase = Phase::kDisconnected;

    auto step = [&](bool recovery) -> void {
        // clock: usually small steps, sometimes a long jump
        now += rng.chance(0.01) ? rng.uni(0.3, 30.0) : rng.uni(0.005, 0.1);
        if (rng.chance(0.05)) {target = rng.wild(-0.5, 2.0, 0.25);}
        if (recovery && !std::isfinite(target)) {target = 0.4;}
        const int moves_before = dev->move_calls;
        const int stops_before = dev->stop_calls;
        const Phase before = sup.snapshot().phase;
        if (before == Phase::kHeld && recovery && rng.chance(0.5)) {
          sup.reset_fault(now);
          ++resets;
        }
        sup.poll(now);
        const int moves_poll = dev->move_calls;
        const Phase mid = sup.snapshot().phase;
        sup.command(now, target);
        const auto & s = sup.snapshot();
        const Phase after = s.phase;

        // the device was never asked for an impossible height
        ASSERT_EQ(dev->bad_moves, 0) << "a height outside [0, stroke] (or non-finite) was sent: " << dev->sent.back() << " dev stroke " << dev->stroke << " snap stroke " << s.stroke_m << " phase " << to_string(after) << " before " << to_string(before) << " cfgstroke " << cfg.stroke_m << " home " << cfg.home_height_m << " ref " << s.reference_m << " target " << target << " mid " << to_string(mid) << " h " << s.height_m << " nmoves " << dev->sent.size();
        if (cfg.stroke_m > 0.0 && !dev->sent.empty()) {ASSERT_LE(dev->sent.back(), cfg.stroke_m + 1e-12);}
        // a non-finite command, or a command outside Ready, moves nothing
        if (!std::isfinite(target) || mid != Phase::kReady) {ASSERT_EQ(dev->move_calls, moves_poll);}
        // published numbers
        ASSERT_TRUE(std::isfinite(s.height_m));
        ASSERT_TRUE(std::isfinite(s.velocity_mps)) << s.velocity_mps << " now " << now << " h " << s.height_m << " filt " << cfg.velocity_filter_s;
        ASSERT_TRUE(std::isfinite(s.reference_m));
        ASSERT_TRUE(std::isfinite(s.stroke_m));
        ASSERT_GE(s.stroke_m, 0.0);
        if (after == Phase::kReady) {
          ASSERT_TRUE(s.homed);
          ASSERT_GT(s.stroke_m, 0.0);
          ASSERT_GE(s.reference_m, -1e-12);
          ASSERT_LE(s.reference_m, s.stroke_m + 1e-12);
          if (cfg.stroke_m > 0.0) {ASSERT_LE(s.stroke_m, cfg.stroke_m + 1e-12);}
          ASSERT_LE(s.stroke_m, dev->stroke + 1e-12);
          // rate limit of the shaped reference
          if (prev_ready && std::isfinite(target)) {
            const double gap = std::min(0.25, std::max(0.0, now - last_cmd_t));
            ASSERT_LE(std::fabs(s.reference_m - prev_ref), cfg.max_velocity_mps * gap + 1e-9)
              << "reference jumped " << s.reference_m - prev_ref << " in " << gap << " s; target " << target << " ref " << s.reference_m << " prev " << prev_ref << " vmax " << cfg.max_velocity_mps << " amax " << cfg.max_acceleration_mps2 << " before " << to_string(before) << " mid " << to_string(mid) << " clamped " << sup.clamped_target_m() << " h " << s.height_m;
          }
          if (std::isfinite(target)) {last_cmd_t = now;}
          // the shaping (re)starts from the measured height at the first accepted command of a Ready episode, so the
          // rate limit is checked between two accepted commands only
          if (std::isfinite(target)) {
            prev_ref = s.reference_m;
            prev_ready = true;
          }
          ++ready_cycles;
        } else {
          prev_ready = false;
          ASSERT_FALSE(after == Phase::kDisconnected && s.homed);
        }
                // a stop wins: while held nothing is sent
        if (after == Phase::kHeld) {
          ++held_cycles;
          if (was_held && before == Phase::kHeld) {ASSERT_EQ(dev->move_calls, moves_before) << "move while held";}
          if (!s.stop_confirmed && before == Phase::kHeld) {
            ASSERT_GT(dev->stop_calls, stops_before) << "held with an unconfirmed stop and no stop was tried";
          }
          if (s.stop_confirmed && dev->connected) {ASSERT_FALSE(dev->moving) << "stop confirmed but the device still moves";}
          was_held = true;
          held_moves = static_cast<size_t>(dev->move_calls);
        } else {
          was_held = false;
        }
        if (before == Phase::kDisconnected && after != Phase::kDisconnected) {++reconnects;}
        (void)held_moves; (void)prev_stops; (void)prev_unconfirmed; (void)prev_phase;
        ++cycles;
      };

    for (int i = 0; i < 300 && !::testing::Test::HasFatalFailure(); ++i) {
      step(false);
      if (::testing::Test::HasFatalFailure()) {return;}
      if (rng.chance(0.02) && sup.snapshot().phase == Phase::kHeld) {
        sup.reset_fault(now);
        ++resets;
      }
    }
    // liveness: the faults stop; the supervisor must end up Ready, homed and following a finite command
    dev->faults_on = false;
    dev->stroke = dev->stroke_nominal;
    target = rng.uni(0.0, 1.0);
    bool ok = false;
    for (int i = 0; i < 4000 && !ok; ++i) {
      now += 0.05;
      if (sup.snapshot().phase == Phase::kHeld) {sup.reset_fault(now);}
      sup.poll(now);
      sup.command(now, target);
      ok = sup.snapshot().phase == Phase::kReady && std::fabs(sup.snapshot().reference_m - sup.clamped_target_m()) < 1e-9 &&
        std::fabs(sup.snapshot().height_m - sup.clamped_target_m()) < 2 * cfg.tolerance_m;
    }
    ASSERT_TRUE(ok) << "stuck in phase " << to_string(sup.snapshot().phase) << " after the faults stopped (home "
                    << cfg.home_height_m << ", target " << target << ", height " << sup.snapshot().height_m << ")";
    ASSERT_TRUE(sup.snapshot().homed);
    ASSERT_NEAR(sup.clamped_target_m(), std::clamp(target, 0.0, sup.snapshot().stroke_m), 1e-12);
    // shutdown stops the device and nothing more is sent
    sup.shutdown();
    ASSERT_FALSE(dev->moving);
    ASSERT_FALSE(dev->connected);
    const int calls = dev->move_calls + dev->stop_calls + dev->connects;
    sup.poll(now + 1.0);
    sup.command(now + 1.0, 0.1);
    ASSERT_FALSE(sup.reset_fault(now + 1.0));
    ASSERT_EQ(dev->move_calls + dev->stop_calls + dev->connects, calls);
  }
  RecordProperty("cycles", static_cast<int>(cycles));
  EXPECT_GT(held_cycles, 1000);     // the generator really holds, reconnects and runs Ready
  EXPECT_GT(ready_cycles, 10000);
  EXPECT_GT(reconnects, 500);
  EXPECT_GT(resets, 100);
}

TEST(SupervisorFuzz, RegressionLongStepSnapRespectsTheSpeedLimit)
{
  // dt = 0.25 s, a = 1: the "land exactly" snap covered up to 0.5 * a * dt^2 = 0.031 m with max_velocity 0.05 m/s
  // (limit 0.0125 m in that step).
  const Kinematics k = advance_toward({0.40, 0.0}, 0.419, 0.05, 1.0, 0.25);
  EXPECT_LE(std::fabs(k.position - 0.40), 0.05 * 0.25 + 1e-12);
}

TEST(SupervisorFuzz, RegressionReversalNearTheStrokeEndNeverSendsAHeightBeyondIt)
{
  // Running up to the stroke end and then commanding a far lower target: the reference braked past the end.
  LiftConfig cfg;
  cfg.max_velocity_mps = 0.3;
  cfg.max_acceleration_mps2 = 0.2;
  cfg.send_period_s = 0.0;
  cfg.resend_deadband_m = 0.0;
  auto t = std::make_unique<ScriptedTransport>(0.8, 0.3);
  ScriptedTransport * dev = t.get();
  LiftSupervisor sup(std::move(t), cfg);
  double now = 0.0;
  for (int i = 0; i < 20; ++i) {sup.poll(now); sup.command(now, 0.8); now += 0.02;}
  ASSERT_EQ(sup.snapshot().phase, Phase::kReady);
  while (sup.snapshot().reference_m < 0.79) {sup.poll(now); sup.command(now, 0.8); now += 0.02;}
  for (int i = 0; i < 100; ++i) {sup.poll(now); sup.command(now, 0.0); now += 0.02;}
  for (const double m : dev->moves()) {
    EXPECT_LE(m, 0.8 + 1e-12);
    EXPECT_GE(m, 0.0);
  }
}

TEST(SupervisorFuzz, RegressionAbsurdStrokeIsRefused)
{
  auto t = std::make_unique<ScriptedTransport>(1e300, 0.3);
  LiftSupervisor sup(std::move(t), LiftConfig{});
  for (int i = 0; i < 50; ++i) {sup.poll(i * 1.0);}
  EXPECT_EQ(sup.snapshot().phase, Phase::kDisconnected);
}

TEST(SupervisorFuzz, ConfigParsingNeverYieldsAnUnusableConfig)
{
  Rng rng(0x11F7002);
  static const char * const texts[] = {"nan", "inf", "-inf", "1e999", "-1", "0", "1", "0.5", " 2", "3 ", "0x10", "1e-3", "", "abc", "1.5.2",
    "2.5", "1e308", "-0", "+0.1", "5"};
  static const char * const keys[] = {"stroke_m", "max_velocity_mps", "max_acceleration_mps2", "home_height_m", "home_timeout_s",
    "tolerance_m", "move_timeout_s", "send_period_s", "resend_deadband_m", "reconnect_initial_s", "reconnect_max_s",
    "reconnect_factor", "max_io_failures", "velocity_filter_s", "unknown_key"};
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    std::unordered_map<std::string, std::string> params;
    for (const char * k : keys) {
      if (rng.chance(0.4)) {params[k] = texts[rng.integer(0, static_cast<int>(sizeof(texts) / sizeof(texts[0])) - 1)];}
    }
    std::string err;
    const LiftConfig cfg = parse_lift_config(params, &err);
    // either a usable config, or the defaults with the reason
    ASSERT_EQ(validate(cfg), "") << err;
    if (!err.empty()) {
      ASSERT_EQ(cfg.max_velocity_mps, LiftConfig{}.max_velocity_mps);
      ASSERT_EQ(cfg.stroke_m, LiftConfig{}.stroke_m);
    }
  }
}

}  // namespace lift_hardware
