// score_run: score a recorded wall run from a sample CSV and a wall JSON (see run_scorecard_io.hpp).
//   score_run --samples run.csv --wall wall.json [--swath 0.2] [--resolution 0.01] [--no-contact]
//             [--target-force 10] [--band 1] [--engaged-force 1]
//             [--spray-standoff 0.3 --spray-half-angle 20 [--spray-sigma 0.25] [--max-thickness-cov 0]]
//             [--min-coverage 0.95] [--max-rms 0] [--max-peak 0] [--min-in-band 0]
// Prints the scorecard as JSON. Exit code 0 = passed, 1 = failed a limit, 2 = bad input.
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "coverage_planner/core/run_scorecard_io.hpp"

int main(int argc, char ** argv)
{
  std::map<std::string, std::string> a;
  bool contact = true;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "--no-contact") {contact = false; continue;}
    if (k.rfind("--", 0) != 0 || i + 1 >= argc) {
      std::cerr << "usage: score_run --samples run.csv --wall wall.json [options]\n";
      return 2;
    }
    a[k] = argv[++i];
  }
  try {
    if (!a.count("--samples") || !a.count("--wall")) {throw std::runtime_error("--samples and --wall are required");}
    std::ifstream csv(a["--samples"]), wall_file(a["--wall"]);
    if (!csv) {throw std::runtime_error("cannot read " + a["--samples"]);}
    if (!wall_file) {throw std::runtime_error("cannot read " + a["--wall"]);}
    const auto samples = coverage_planner::parseRunCsv(csv);
    const auto wall = coverage_planner::wallFromJson(nlohmann::json::parse(wall_file));
    auto num = [&](const char * key, double dflt) {return a.count(key) ? std::stod(a[key]) : dflt;};
    coverage_planner::ScorecardOptions o;
    o.contact = contact;
    o.swath_m = num("--swath", o.swath_m);
    o.resolution_m = num("--resolution", o.resolution_m);
    o.engaged_force_n = num("--engaged-force", o.engaged_force_n);
    o.target_force_n = num("--target-force", o.target_force_n);
    o.force_band_n = num("--band", o.force_band_n);
    if (!contact && a.count("--spray-standoff")) {
      o.spray = coverage_planner::SprayFootprint::fromCone(
        std::stod(a["--spray-standoff"]), num("--spray-half-angle", 20.0), num("--spray-sigma", 0.25));
    }
    coverage_planner::ScorecardLimits l;
    l.max_thickness_cov = num("--max-thickness-cov", l.max_thickness_cov);
    l.min_covered_fraction = num("--min-coverage", l.min_covered_fraction);
    l.max_force_rms_error_n = num("--max-rms", l.max_force_rms_error_n);
    l.max_force_peak_n = num("--max-peak", l.max_force_peak_n);
    l.min_in_band_fraction = num("--min-in-band", l.min_in_band_fraction);
    const auto card = coverage_planner::scoreRun(wall, {}, samples, o, l);
    std::cout << coverage_planner::scorecardToJson(card).dump(2) << "\n";
    return card.passed ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << "score_run: " << e.what() << "\n";
    return 2;
  }
}
