// Reduced Monte Carlo split benchmark (planar fake arms, see apps/split_benchmark.hpp): ordering and determinism.
#include <gtest/gtest.h>

#include "split_benchmark.hpp"

using namespace reachability::split_bench;  // NOLINT

TEST(SplitBenchmark, ChainBreaksNoMoreThanPerPoseFirst)
{
  for (const auto & s : defaultSettings()) {
    const auto r = runSetting(s, 50, 7);
    ASSERT_EQ(r.size(), static_cast<size_t>(kMethodCount));
    EXPECT_EQ(r[kChain].columns, 50) << s.name;
    EXPECT_LE(r[kChain].breaks, r[kFirstFixed].breaks) << s.name;
    EXPECT_LE(r[kChain].breaks, r[kFirstRandom].breaks) << s.name;
    EXPECT_GE(r[kChain].whole, r[kFirstFixed].whole) << s.name;
  }
}

TEST(SplitBenchmark, Deterministic)
{
  for (const auto & s : defaultSettings()) {
    EXPECT_EQ(runSetting(s, 50, 11), runSetting(s, 50, 11)) << s.name;
  }
}
