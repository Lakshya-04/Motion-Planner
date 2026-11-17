#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include "force_core/bias_tracker.hpp"

using force_core::BiasTracker;
using force_core::BiasTrackerConfig;
using force_core::BiasUpdateStatus;
using core_types::Wrench;

namespace
{
Wrench fz(double f)
{
  Wrench w;
  w.force_z = f;
  return w;
}
}  // namespace

TEST(BiasTracker, FollowsSlowDriftWithNoise)
{
  BiasTrackerConfig c;
  c.process_noise_force = 1e-6;     // allows the drift below
  BiasTracker t(fz(0.20), c);
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 0.1);
  double drift = 0.20, worst_late = 0.0;
  for (int i = 0; i < 6000; ++i) {                       // 100 Hz, 60 s
    drift += 0.01 / 60.0 / 100.0 * 10.0;                 // 0.1 N over 60 s
    t.update(fz(drift + n(rng)), true, true);
    if (i > 3000) {worst_late = std::max(worst_late, std::abs(t.bias().force_z - drift));}
  }
  std::printf("[ bias ] drift %.3f N tracked, worst error after 30 s %.3f N\n", drift - 0.2, worst_late);
  EXPECT_LT(worst_late, 0.05);
  EXPECT_NEAR(t.bias().force_z, drift, 0.04);
}

TEST(BiasTracker, DoesNotUpdateInContactOrWhileMoving)
{
  BiasTracker t(fz(0.0));
  EXPECT_EQ(t.update(fz(0.3), false, true), BiasUpdateStatus::kSkippedNotFree);
  EXPECT_EQ(t.update(fz(0.3), true, false), BiasUpdateStatus::kSkippedNotFree);
  EXPECT_DOUBLE_EQ(t.bias().force_z, 0.0);
  EXPECT_EQ(t.accepted(), 0u);
}

TEST(BiasTracker, RejectsLargeInnovationAndFlagsRecalibration)
{
  BiasTracker t(fz(0.0));
  for (int i = 0; i < 19; ++i) {EXPECT_EQ(t.update(fz(8.0), true, true), BiasUpdateStatus::kRejectedInnovation);}
  EXPECT_FALSE(t.needsRecalibration());
  t.update(fz(8.0), true, true);
  EXPECT_TRUE(t.needsRecalibration());
  EXPECT_DOUBLE_EQ(t.bias().force_z, 0.0);
  t.update(fz(0.01), true, true);               // a good sample clears the run
  EXPECT_FALSE(t.needsRecalibration());
}

TEST(BiasTracker, CapsTotalCorrection)
{
  BiasTrackerConfig c;
  c.reject_threshold_force = 5.0;
  c.max_total_correction_force = 0.5;
  BiasTracker t(fz(0.0), c);
  BiasUpdateStatus last = BiasUpdateStatus::kUpdated;
  for (int i = 0; i < 2000; ++i) {last = t.update(fz(0.9), true, true);}
  EXPECT_EQ(last, BiasUpdateStatus::kRejectedCap);
  EXPECT_LE(std::abs(t.bias().force_z), 0.5);
}

TEST(BiasTracker, ContactSpikesDoNotLeakIntoTheEstimate)
{
  BiasTracker t(fz(0.1));
  std::mt19937 rng(5);
  std::normal_distribution<double> n(0.0, 0.05);
  for (int i = 0; i < 3000; ++i) {
    const bool spike = (i % 50) < 3;                       // missed contact bursts of 5 N
    t.update(fz(0.1 + n(rng) + (spike ? 5.0 : 0.0)), true, true);
  }
  EXPECT_NEAR(t.bias().force_z, 0.1, 0.02);
}
