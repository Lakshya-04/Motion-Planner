/**
 * @file mission_verify.hpp
 * @brief The verify result the manager can write at the end of a mission, from what it knows.
 * @details ROS-free, header-only. Same JSON keys as coverage_planner's `VerifyResult`
 *          (`verifyResultToJson`: passed, coverage_percent, failed_segment_ids, report_path, source,
 *          failures), with `source` "manager". The manager has no tool-position telemetry and no wall
 *          model, so it cannot compute geometric coverage:
 *            - `coverage_percent` is the share of the columns asked for that finished DONE
 *              (`coverage_basis` says "columns"), not a coverage raster. `score_run` on a recording
 *              gives the geometric number.
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
};

struct MissionVerifyResult
{
  bool passed{false};
  double coverage_percent{0.0};              ///< columns done / columns asked for, 0..100
  std::vector<size_t> failed_segment_ids;    ///< ascending, no duplicates
  std::vector<std::string> failures;
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
  if (!in.loaded) {r.failures.push_back("run did not start: " + in.message);}
  if (in.canceled) {r.failures.push_back("run canceled");}
  if (in.loaded && !in.canceled && !in.tracker_success && bad.empty()) {
    r.failures.push_back("a stand was not reached or a column was not run");
  }
  r.passed = in.loaded && !in.canceled && in.tracker_success && peak_ok && bad.empty();
  return r;
}

inline nlohmann::json missionVerifyToJson(const MissionVerifyResult & r, const MissionVerifyInput & in)
{
  return {
    {"passed", r.passed},
    {"coverage_percent", r.coverage_percent},
    {"coverage_basis", "columns"},
    {"failed_segment_ids", r.failed_segment_ids},
    {"report_path", in.report_path},
    {"source", "manager"},
    {"failures", r.failures},
    {"peak_force_n", in.peak_force_n}};
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
