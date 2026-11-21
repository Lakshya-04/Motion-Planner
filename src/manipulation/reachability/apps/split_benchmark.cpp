// Monte Carlo benchmark: how well does ColumnIkChain avoid split columns on planar fake arms?
//
//   split_benchmark [--n 1000] [--seed 20240611] [--csv split_benchmark.csv]
//
// Prints a table and writes a CSV. See split_benchmark.hpp for the model and its limits (planar fakes, not a
// real arm, not hardware). ROS-free.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

#include "split_benchmark.hpp"

int main(int argc, char ** argv)
{
  int n = 1000;
  uint64_t seed = 20240611;
  std::string csv = "split_benchmark.csv";
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--n")) {n = std::atoi(argv[i + 1]);}
    else if (!std::strcmp(argv[i], "--seed")) {seed = std::strtoull(argv[i + 1], nullptr, 10);}
    else if (!std::strcmp(argv[i], "--csv")) {csv = argv[i + 1];}
    else {std::fprintf(stderr, "unknown option %s\n", argv[i]); return 2;}
  }
  std::ofstream f(csv);
  f << "setting,method,columns,discarded,whole,whole_pct,breaks,branch_changes,mean_ratio,p95_ratio,"
       "mean_lift_m,near_singular,near_limit,worse_than_chain,better_than_chain\n";
  std::printf("%-12s %-13s %6s %6s %7s %7s %7s %7s %7s %6s %6s %6s %7s %7s\n", "setting", "method", "cols",
    "whole", "whole%", "breaks", "brchg", "ratio", "p95", "lift", "nsing", "nlim", "worse", "better");
  uint64_t k = 0;
  for (const auto & s : reachability::split_bench::defaultSettings()) {
    for (const auto & r : reachability::split_bench::runSetting(s, n, seed + 1000 * k)) {
      const double pct = r.columns ? 100.0 * r.whole / r.columns : 0.0;
      std::printf("%-12s %-13s %6d %6d %6.1f%% %7d %7d %7.2f %7.2f %6.3f %6d %6d %7d %7d\n", r.setting.c_str(),
        r.method.c_str(), r.columns, r.whole, pct, r.breaks, r.branch_changes, r.mean_ratio, r.p95_ratio,
        r.mean_lift_m, r.near_singular, r.near_limit, r.worse_than_chain, r.better_than_chain);
      f << r.setting << ',' << r.method << ',' << r.columns << ',' << r.discarded << ',' << r.whole << ','
        << pct << ',' << r.breaks << ',' << r.branch_changes << ',' << r.mean_ratio << ',' << r.p95_ratio
        << ',' << r.mean_lift_m << ',' << r.near_singular << ',' << r.near_limit << ','
        << r.worse_than_chain << ',' << r.better_than_chain << '\n';
    }
    ++k;
  }
  std::printf("seed base %llu, n %d kept columns per setting; csv: %s\n", static_cast<unsigned long long>(seed), n,
    csv.c_str());
  return 0;
}
