/**
 * @file mission_verify.hpp
 * @brief The verify result the manager can write at the end of a mission, from what it knows.
 * @details ROS-free, header-only. Same JSON keys as coverage_planner's `VerifyResult`
 *          (`verifyResultToJson`: passed, coverage_percent, failed_segment_ids, report_path, source,
 *          failures), with `source` "manager". `coverage_percent` is the share of the columns asked for that finished DONE
 *          (`coverage_basis` says "columns"). When the node sampled the tool path (`verify_geometric_coverage`), the
 *          result also carries `geometric_coverage_percent`: the covered share of the planned patch from the tool
 *          positions actually reached (`geometric_coverage.hpp`), with its cell counts, per-column shares and the sample
 *          counts; `verify_min_geometric_percent` (0 = not checked) fails the run below it. It is a TF-sampled trace at
 *          the node's rate, not a recording-grade score: `score_run` on a recording is still the reference number.
 *          An opt-in path quality audit (`path_quality_port.hpp`) adds a `path_quality` block and `warnings`; it is
 *          report only and never changes `passed`.
 *            - a segment is a wall column; it fails when it ended partial or failed, or (when a peak
 *              limit is set and the force topic is read) when the tool force in it passed the limit.
 *            - the run passes when the task loaded, was not canceled, every column and stand
 *              succeeded and the overall force peak is within the limit.
 *          Not computed: standoff, in-band fraction, thickness, obstacle contacts, "no controller
 *          switch under load".
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <algorithm>
#include <cstdio>
#include <cstddef>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "manipulation_manager/core/geometric_coverage.hpp"
#include "manipulation_manager/core/path_quality_port.hpp"

namespace manipulation_manager
{

struct MissionVerifyInput
{
  bool loaded{false};                ///< the plan was loaded and the run started
  bool canceled{false};
  bool tracker_success{false};       ///< every column done and every stand reached
  std::vector<size_t> done, partial, failed;
  double peak_force_n{-1.0};         ///< overall; -1 = no force topic read
  std::map<size_t, double> column_peaks_n;   ///< per column, only columns that ran
  double max_peak_force_n{0.0};      ///< 0 = not checked
  std::string message;               ///< the task's message, kept as a failure when the run did not load
  std::string report_path;           ///< e.g. the ledger or a scorecard written elsewhere; may be empty
  /// Geometric coverage of the planned patch from the sampled tool path (`available` false = not computed).
  GeometricCoverageResult geometric;
  double min_geometric_percent{0.0}; ///< 0 = not checked; otherwise the run fails below it (or when it is unavailable)
  double swath_m{0.0};               ///< recorded in the report
  PathQualitySummary path_quality;   ///< report only (`available` false = not computed)
  bool path_quality_requested{false};
};

struct MissionVerifyResult
{
  bool passed{false};
  double coverage_percent{0.0};              ///< columns done / columns asked for, 0..100
  std::vector<size_t> failed_segment_ids;    ///< ascending, no duplicates
  std::vector<std::string> failures;
  std::vector<std::string> warnings;         ///< report-only findings (path quality); never change `passed`
};

inline MissionVerifyResult buildMissionVerify(const MissionVerifyInput & in)
{
  MissionVerifyResult r;
  const size_t asked = in.done.size() + in.partial.size() + in.failed.size();
  r.coverage_percent = asked == 0 ? 0.0 : 100.0 * static_cast<double>(in.done.size()) / static_cast<double>(asked);

  std::set<size_t> bad(in.partial.begin(), in.partial.end());
  bad.insert(in.failed.begin(), in.failed.end());
  const bool limit = in.max_peak_force_n > 0.0;
  if (limit) {
    for (const auto & [col, peak] : in.column_peaks_n) {
      if (peak > in.max_peak_force_n) {
        bad.insert(col);
        std::ostringstream s;
        s << "segment " << col << ": force peak " << peak << " N > " << in.max_peak_force_n;
        r.failures.push_back(s.str());
      }
    }
  }
  r.failed_segment_ids.assign(bad.begin(), bad.end());
  for (const size_t c : in.partial) {r.failures.push_back("segment " + std::to_string(c) + ": partial");}
  for (const size_t c : in.failed) {r.failures.push_back("segment " + std::to_string(c) + ": failed");}

  bool peak_ok = true;
  if (limit && in.peak_force_n > in.max_peak_force_n) {
    peak_ok = false;
    std::ostringstream s;
    s << "run force peak " << in.peak_force_n << " N > " << in.max_peak_force_n;
    r.failures.push_back(s.str());
  }
  bool geometry_ok = true;
  if (in.min_geometric_percent > 0.0) {
    if (!in.geometric.available) {
      geometry_ok = false;
      r.failures.push_back("geometric coverage not available: " + (in.geometric.note.empty() ? std::string("not computed") : in.geometric.note));
    } else if (in.geometric.percent < in.min_geometric_percent) {
      geometry_ok = false;
      std::ostringstream s;
      s << "geometric coverage " << in.geometric.percent << "% < " << in.min_geometric_percent << "%";
      r.failures.push_back(s.str());
    }
  }
  if (in.path_quality_requested) {
    if (!in.path_quality.available) {
      r.warnings.push_back("path quality not audited: " + (in.path_quality.note.empty() ? std::string("no result") : in.path_quality.note));
    } else {
      for (const auto & c : in.path_quality.columns) {
        if (c.reason == "near_singular" || c.reason == "near_limit") {
          std::ostringstream s;
          s << "segment " << c.column << ": " << c.reason << " (min sigma " << c.min_sigma << ", min limit margin "
            << c.min_limit_margin << ", worst waypoint " << c.worst_index << ")";
          r.warnings.push_back(s.str());
        }
      }
    }
  }
  if (!in.loaded) {r.failures.push_back("run did not start: " + in.message);}
  if (in.canceled) {r.failures.push_back("run canceled");}
  if (in.loaded && !in.canceled && !in.tracker_success && bad.empty()) {
    r.failures.push_back("a stand was not reached or a column was not run");
  }
  r.passed = in.loaded && !in.canceled && in.tracker_success && peak_ok && bad.empty() && geometry_ok;
  return r;
}

inline nlohmann::json missionVerifyToJson(const MissionVerifyResult & r, const MissionVerifyInput & in)
{
  nlohmann::json j = {
    {"passed", r.passed},
    {"coverage_percent", r.coverage_percent},
    {"coverage_basis", "columns"},
    {"failed_segment_ids", r.failed_segment_ids},
    {"report_path", in.report_path},
    {"source", "manager"},
    {"failures", r.failures},
    {"warnings", r.warnings},
    {"peak_force_n", in.peak_force_n}};
  if (in.geometric.available) {
    nlohmann::json per_column = nlohmann::json::object();
    for (const auto & [col, pct] : in.geometric.column_percent) {per_column[std::to_string(col)] = pct;}
    j["geometric_coverage_percent"] = in.geometric.percent;
    j["geometric"] = {
      {"basis", "sampled tool path against the planned patch, wall plane grid"},
      {"planned_cells", in.geometric.planned_cells},
      {"covered_cells", in.geometric.covered_cells},
      {"cell_m", in.geometric.cell_m},
      {"swath_m", in.swath_m},
      {"samples", in.geometric.samples},
      {"working_samples", in.geometric.working_samples},
      {"off_plane_samples", in.geometric.off_plane_samples},
      {"column_percent", per_column},
      {"note", in.geometric.note}};
  } else {
    j["geometric_coverage_percent"] = nullptr;
    j["geometric"] = {{"available", false}, {"note", in.geometric.note}};
  }
  if (in.path_quality_requested) {
    nlohmann::json cols = nlohmann::json::array();
    for (const auto & c : in.path_quality.columns) {
      cols.push_back({{"column", c.column}, {"reason", c.reason}, {"min_sigma", c.min_sigma},
        {"min_limit_margin", c.min_limit_margin}, {"worst_index", c.worst_index}});
    }
    j["path_quality"] = {
      {"available", in.path_quality.available},
      {"note", in.path_quality.note},
      {"near_singular", in.path_quality.near_singular},
      {"near_limit", in.path_quality.near_limit},
      {"min_sigma", in.path_quality.min_sigma},
      {"min_limit_margin", in.path_quality.min_limit_margin},
      {"sigma_threshold", in.path_quality.sigma_threshold},
      {"limit_margin_threshold", in.path_quality.limit_margin_threshold},
      {"columns", cols}};
  }
  return j;
}

/// Write the result as JSON via a temporary file and a rename. False when the file cannot be written.
inline bool writeMissionVerify(const std::string & path, const nlohmann::json & j)
{
  if (path.empty()) {return false;}
  const std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp);
    if (!f) {return false;}
    f << j.dump(2) << "\n";
    if (!f) {return false;}
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

}  // namespace manipulation_manager
