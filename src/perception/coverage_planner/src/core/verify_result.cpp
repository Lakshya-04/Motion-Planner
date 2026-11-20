#include "coverage_planner/core/verify_result.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace coverage_planner
{

ScorecardLimits limitsForMode(RunMode mode)
{
  ScorecardLimits l;
  l.min_covered_fraction = 0.95;
  switch (mode) {
    case RunMode::Contact:
      l.max_force_peak_n = 60.0;
      l.min_in_band_fraction = 0.8;
      break;
    case RunMode::NonContact:
      l.max_force_peak_n = 5.0;
      break;
    case RunMode::Spray:
      l.max_force_peak_n = 5.0;
      l.max_thickness_cov = 0.5;
      break;
  }
  return l;
}

RunMode runModeFromString(const std::string & name)
{
  if (name == "contact") {return RunMode::Contact;}
  if (name == "non_contact") {return RunMode::NonContact;}
  if (name == "spray") {return RunMode::Spray;}
  throw std::invalid_argument("unknown run mode '" + name + "' (contact | non_contact | spray)");
}

VerifyResult verifyFromTelemetry(
  const WallPlane & wall, const std::vector<ObstacleRegion> & obstacles,
  const std::vector<RunSample> & samples, const std::vector<SegmentWindow> & segments,
  const ScorecardOptions & options, const ScorecardLimits & limits,
  const std::string & report_path, RunScorecard * card_out)
{
  const RunScorecard card = scoreRun(wall, obstacles, samples, options, limits);
  VerifyResult r;
  r.coverage_percent = 100.0 * card.coverage.covered_fraction;
  r.report_path = report_path;
  r.failures = card.failures;

  std::set<uint32_t> failed;
  for (const auto & seg : segments) {
    size_t engaged = 0, in_band = 0;
    double peak = 0.0;
    for (const auto & s : samples) {
      if (s.t_s < seg.t_start_s || s.t_s > seg.t_end_s) {continue;}
      peak = std::max(peak, s.force_n);
      const bool working = options.contact ? s.force_n >= options.engaged_force_n : s.tool_active;
      if (!working) {continue;}
      ++engaged;
      if (options.contact && std::fabs(s.force_n - options.target_force_n) <= options.force_band_n) {++in_band;}
    }
    std::ostringstream why;
    bool bad = false;
    if (engaged == 0) {
      bad = true; why << "segment " << seg.id << ": tool never worked";
    } else if (limits.max_force_peak_n > 0.0 && peak > limits.max_force_peak_n) {
      bad = true; why << "segment " << seg.id << ": force peak " << peak << " N > " << limits.max_force_peak_n;
    } else if (options.contact && limits.min_in_band_fraction > 0.0 &&
      static_cast<double>(in_band) / static_cast<double>(engaged) < limits.min_in_band_fraction)
    {
      bad = true; why << "segment " << seg.id << ": in-band fraction under " << limits.min_in_band_fraction;
    }
    if (bad) {
      failed.insert(seg.id);
      r.failures.push_back(why.str());
    }
  }
  r.failed_segment_ids.assign(failed.begin(), failed.end());
  r.passed = card.passed && failed.empty();
  if (card_out) {*card_out = card;}
  return r;
}

}  // namespace coverage_planner
