// score_run: score a recorded wall run from a sample CSV and a wall JSON (see run_scorecard_io.hpp).
//   score_run --samples run.csv --wall wall.json [--swath 0.2] [--resolution 0.01] [--no-contact]
//             [--target-force 10] [--band 1] [--engaged-force 1]
//             [--spray-standoff 0.3 --spray-half-angle 20 [--spray-sigma 0.25] [--max-thickness-cov 0]]
//             [--keepout-margin 0.07] [--obstacle-margin 0] [--obstacle-tol 0.003] [--max-obstacle-contacts N]
//             [--max-wall-gap 0.01] [--max-tilt 10] [--nominal-tilt 0] [--max-bad-pose 0.05]  (tool on the wall and square to it; needs tilt_deg;
//             --max-bad-pose needs a gate, --nominal-tilt needs --max-tilt, --max-wall-gap is ignored with --no-contact)
//             [--min-coverage 0.95] [--max-rms 0] [--max-peak 0] [--min-in-band 0]
//             [--mode contact|non_contact|spray]  (per-mode default limits; explicit flags override)
//             [--plan coverage_output.json]  (adds a "plan_redundancy" field: see plan_redundancy.hpp)
//             [--segments segments.json] [--result-out result.json] [--report-out card.json]
// Prints the scorecard as JSON; --result-out writes the verify-result contract (passed, coverage_percent,
// failed_segment_ids, report_path, source, failures). Exit code 0 = passed, 1 = failed a limit, 2 = bad input.
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "coverage_planner/core/plan_redundancy.hpp"
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
    const auto wall_json = nlohmann::json::parse(wall_file);
    const auto wall = coverage_planner::wallFromJson(wall_json);
    const auto obstacles = coverage_planner::obstaclesFromJson(wall_json, wall.id);
    auto num = [&](const char * key, double dflt) {return a.count(key) ? std::stod(a[key]) : dflt;};
    coverage_planner::ScorecardOptions o;
    o.contact = contact;
    o.swath_m = num("--swath", o.swath_m);
    o.resolution_m = num("--resolution", o.resolution_m);
    o.engaged_force_n = num("--engaged-force", o.engaged_force_n);
    o.target_force_n = num("--target-force", o.target_force_n);
    o.force_band_n = num("--band", o.force_band_n);
    o.keepout_margin_m = num("--keepout-margin", o.keepout_margin_m);
    o.obstacle_margin_m = num("--obstacle-margin", o.obstacle_margin_m);
    o.obstacle_contact_tol_m = num("--obstacle-tol", o.obstacle_contact_tol_m);
    o.max_wall_gap_m = num("--max-wall-gap", o.max_wall_gap_m);
    o.max_tilt_deg = num("--max-tilt", o.max_tilt_deg);
    o.nominal_tilt_deg = num("--nominal-tilt", o.nominal_tilt_deg);
    if (!contact && a.count("--spray-standoff")) {
      o.spray = coverage_planner::SprayFootprint::fromCone(
        std::stod(a["--spray-standoff"]), num("--spray-half-angle", 20.0), num("--spray-sigma", 0.25));
    }
    coverage_planner::ScorecardLimits l;
    if (a.count("--mode")) {
      const auto mode = coverage_planner::runModeFromString(a["--mode"]);
      l = coverage_planner::limitsForMode(mode);
      if (mode != coverage_planner::RunMode::Contact && contact) {
        throw std::runtime_error("--mode " + a["--mode"] + " needs --no-contact");
      }
    }
    l.max_thickness_cov = num("--max-thickness-cov", l.max_thickness_cov);
    l.min_covered_fraction = num("--min-coverage", l.min_covered_fraction);
    l.max_force_rms_error_n = num("--max-rms", l.max_force_rms_error_n);
    l.max_force_peak_n = num("--max-peak", l.max_force_peak_n);
    l.min_in_band_fraction = num("--min-in-band", l.min_in_band_fraction);
    l.max_bad_pose_fraction = num("--max-bad-pose", l.max_bad_pose_fraction);
    if (a.count("--max-obstacle-contacts")) {l.max_obstacle_contacts = std::stoi(a["--max-obstacle-contacts"]);}
    coverage_planner::validateGateOptions(o, l);
    std::vector<coverage_planner::SegmentWindow> segments;
    if (a.count("--segments")) {
      std::ifstream sf(a["--segments"]);
      if (!sf) {throw std::runtime_error("cannot read " + a["--segments"]);}
      segments = coverage_planner::segmentsFromJson(nlohmann::json::parse(sf));
    }
    coverage_planner::RunScorecard card;
    const auto result = coverage_planner::verifyFromTelemetry(
      wall, obstacles, samples, segments, o, l, a.count("--report-out") ? a["--report-out"] : "", &card);
    auto card_obj = coverage_planner::scorecardToJson(card);
    if (a.count("--plan")) {
      std::ifstream pf(a["--plan"]);
      if (!pf) {throw std::runtime_error("cannot read " + a["--plan"]);}
      const auto plan = coverage_planner::expandedCoverageFromJson(nlohmann::json::parse(pf));
      card_obj["plan_redundancy"] = coverage_planner::redundancyReportToJson(
        coverage_planner::analyzePlanRedundancy(plan.path, plan.segments));
    }
    const auto card_json = card_obj.dump(2);
    std::cout << card_json << "\n";
    if (a.count("--report-out")) {std::ofstream(a["--report-out"]) << card_json << "\n";}
    if (a.count("--result-out")) {
      std::ofstream(a["--result-out"]) << coverage_planner::verifyResultToJson(result).dump(2) << "\n";
    }
    return result.passed ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << "score_run: " << e.what() << "\n";
    return 2;
  }
}
