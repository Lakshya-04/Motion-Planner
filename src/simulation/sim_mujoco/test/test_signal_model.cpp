#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sim_mujoco/signal_model.hpp"

using sim_mujoco::SignalConfig;
using sim_mujoco::SignalModel;

namespace
{
constexpr double kDt = 0.002;  // 500 Hz
double mean(const std::vector<double> & v)
{
  double s = 0;
  for (double x : v) {s += x;}
  return s / v.size();
}
double stdev(const std::vector<double> & v)
{
  const double m = mean(v);
  double s = 0;
  for (double x : v) {s += (x - m) * (x - m);}
  return std::sqrt(s / (v.size() - 1));
}
}  // namespace

TEST(SignalModel, DefaultIsAnExactPassThrough)
{
  SignalConfig cfg;
  EXPECT_FALSE(cfg.enabled());
  SignalModel m(cfg, kDt);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(m.step(0.1 * i), 0.1 * i);
  }
}

TEST(SignalModel, WhiteNoiseMatchesDensityAndIsZeroMean)
{
  SignalConfig cfg;
  cfg.noise_density = 0.1;  // N/sqrt(Hz) -> std = 0.1 / sqrt(2 * 0.002) = 1.581 N
  SignalModel m(cfg, kDt);
  std::vector<double> y;
  for (int i = 0; i < 200000; ++i) {y.push_back(m.step(5.0));}
  EXPECT_NEAR(mean(y), 5.0, 0.02);
  EXPECT_NEAR(stdev(y), 0.1 / std::sqrt(2 * kDt), 0.02);
}

TEST(SignalModel, BiasRandomWalkGrowsWithSqrtTime)
{
  SignalConfig cfg;
  cfg.bias_walk = 0.2;  // N/sqrt(s)
  const double T = 10.0;
  const int n = static_cast<int>(T / kDt);
  std::vector<double> end;
  for (uint64_t seed = 1; seed <= 400; ++seed) {
    cfg.seed = seed;
    SignalModel m(cfg, kDt);
    for (int i = 0; i < n; ++i) {m.step(0.0);}
    end.push_back(m.bias());
  }
  EXPECT_NEAR(stdev(end), 0.2 * std::sqrt(T), 0.2 * std::sqrt(T) * 0.12);  // 400 runs: 3.5 % + margin
  EXPECT_NEAR(mean(end), 0.0, 0.2 * std::sqrt(T) * 0.2);
}

TEST(SignalModel, LowpassStepResponseHasTheTimeConstant)
{
  SignalConfig cfg;
  cfg.lowpass_hz = 10.0;
  SignalModel m(cfg, kDt);
  m.step(0.0);
  double y = 0;
  const double tau = 1.0 / (2 * M_PI * 10.0);
  const int k = static_cast<int>(tau / kDt);
  for (int i = 0; i < k; ++i) {y = m.step(1.0);}
  EXPECT_NEAR(y, 1.0 - std::exp(-k * kDt / tau), 1e-9);  // exact discrete step response (~63 % at tau)
  for (int i = 0; i < 2000; ++i) {y = m.step(1.0);}
  EXPECT_NEAR(y, 1.0, 1e-6);
}

TEST(SignalModel, DelayShiftsByWholePeriods)
{
  SignalConfig cfg;
  cfg.delay_s = 0.010;  // 5 periods
  SignalModel m(cfg, kDt);
  EXPECT_EQ(m.delaySamples(), 5u);
  std::vector<double> y;
  for (int i = 0; i < 20; ++i) {y.push_back(m.step(i));}
  for (int i = 5; i < 20; ++i) {EXPECT_EQ(y[i], i - 5);}
  for (int i = 0; i < 5; ++i) {EXPECT_EQ(y[i], 0.0);}  // first sample held while the line fills
}

TEST(SignalModel, QuantisationSnapsToTheStep)
{
  SignalConfig cfg;
  cfg.quantum = 0.05;
  SignalModel m(cfg, kDt);
  EXPECT_NEAR(m.step(1.012), 1.0, 1e-12);
  EXPECT_NEAR(m.step(1.04), 1.05, 1e-12);
  EXPECT_NEAR(m.step(-0.3), -0.3, 1e-12);
}

TEST(SignalModel, NanCommandsPassThrough)
{
  SignalConfig cfg;
  cfg.delay_s = 0.01;
  SignalModel m(cfg, kDt);
  EXPECT_TRUE(std::isnan(m.step(std::nan(""))));
  EXPECT_EQ(m.step(1.0), 1.0);
}

TEST(SignalModel, SameSeedSameSequence)
{
  SignalConfig cfg;
  cfg.noise_density = 0.1;
  cfg.bias_walk = 0.1;
  SignalModel a(cfg, kDt), b(cfg, kDt);
  for (int i = 0; i < 100; ++i) {EXPECT_EQ(a.step(0.0), b.step(0.0));}
}
