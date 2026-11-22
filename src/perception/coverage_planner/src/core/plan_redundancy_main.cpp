// plan_redundancy_report: does a coverage plan move for nothing?
//   plan_redundancy_report coverage_output.json [--pitch 0.2] [--bottom-z 0] [--top-z 2] [--json-out report.json]
// Prints a table (stdout) and writes the JSON (--json-out). Reads `final_lifted_and_expanded` of the file.
// Exit code 0 = no jumps, alternation OK and no double sweep; 1 = redundancy found; 2 = bad input.
#include <fstream>
#include <iostream>
#include <string>

#include "coverage_planner/core/plan_redundancy.hpp"

int main(int argc, char ** argv)
{
  try {
    std::string in, json_out;
    coverage_planner::RedundancyOptions o;
    for (int i = 1; i < argc; ++i) {
      const std::string k = argv[i];
      if (k.rfind("--", 0) != 0) {in = k; continue;}
      if (i + 1 >= argc) {throw std::runtime_error("missing value for " + k);}
      const std::string v = argv[++i];
      if (k == "--pitch") {o.pitch_m = std::stod(v);}
      else if (k == "--bottom-z") {o.wall_bottom_z = std::stod(v);}
      else if (k == "--top-z") {o.wall_top_z = std::stod(v);}
      else if (k == "--json-out") {json_out = v;}
      else {throw std::runtime_error("unknown option " + k);}
    }
    if (in.empty()) {throw std::runtime_error("usage: plan_redundancy_report coverage_output.json [options]");}
    std::ifstream f(in);
    if (!f) {throw std::runtime_error("cannot read " + in);}
    const auto plan = coverage_planner::expandedCoverageFromJson(nlohmann::json::parse(f));
    const auto r = coverage_planner::analyzePlanRedundancy(plan.path, plan.segments, o);
    std::cout << coverage_planner::redundancyReportToTable(r);
    if (!json_out.empty()) {std::ofstream(json_out) << coverage_planner::redundancyReportToJson(r).dump(2) << "\n";}
    return (r.jumps == 0 && r.alternation_ok && r.cells_swept_more_than_expected == 0) ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << "plan_redundancy_report: " << e.what() << "\n";
    return 2;
  }
}
