// Seeded random helpers for the property/fuzz tests of the ROS-free decision cores (deterministic, header-only).
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace fuzz
{

constexpr int kCases = 20000;   ///< random cases per fuzz test

class Rng
{
public:
  explicit Rng(uint64_t seed) : e_(seed) {}
  double uni(double lo, double hi) {return std::uniform_real_distribution<double>(lo, hi)(e_);}
  int integer(int lo, int hi) {return std::uniform_int_distribution<int>(lo, hi)(e_);}
  bool chance(double p) {return uni(0.0, 1.0) < p;}
  /// A value from [lo, hi], now and then replaced by an adversarial one: NaN, +-inf, +-0, huge, tiny, negative.
  double wild(double lo, double hi, double p_bad = 0.08)
  {
    if (!chance(p_bad)) {return uni(lo, hi);}
    static const double bad[] = {
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(), 0.0, -0.0, 1e300, -1e300, 1e-300, 1e18, -1e18,
      std::numeric_limits<double>::max(), std::numeric_limits<double>::denorm_min()};
    return bad[static_cast<size_t>(integer(0, static_cast<int>(sizeof(bad) / sizeof(bad[0])) - 1))];
  }
  std::vector<double> vec(size_t n, double lo, double hi, double p_bad = 0.0)
  {
    std::vector<double> v(n);
    for (auto & x : v) {x = p_bad > 0.0 ? wild(lo, hi, p_bad) : uni(lo, hi);}
    return v;
  }
  std::mt19937_64 & engine() {return e_;}

private:
  std::mt19937_64 e_;
};

}  // namespace fuzz
