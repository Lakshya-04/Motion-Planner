#include "coverage_planner/core/run_scorecard_io.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <tuple>

#include "coverage_planner/core/wall_basis.hpp"

namespace coverage_planner
{

std::vector<RunSample> parseRunCsv(std::istream & in)
{
  std::vector<RunSample> out;
  std::string line;
  bool header_seen = false;
  size_t columns = 6;
  size_t line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    if (line.empty() || line[0] == '#') {continue;}
    if (!header_seen) {
      if (line == "t,x,y,z,force,active") {
        columns = 6;
      } else if (line == "t,x,y,z,force,active,tilt_deg") {
        columns = 7;
      } else {
        throw std::runtime_error("sample csv: the header must be t,x,y,z,force,active[,tilt_deg]");
      }
      header_seen = true;
      continue;
    }
    std::vector<double> v;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      try {
        size_t used = 0;
        v.push_back(std::stod(cell, &used));
        if (used != cell.size()) {throw std::invalid_argument(cell);}
      } catch (const std::exception &) {
        throw std::runtime_error("sample csv line " + std::to_string(line_no) + ": '" + cell + "' is not a number");
      }
    }
    if (v.size() != columns) {
      throw std::runtime_error(
              "sample csv line " + std::to_string(line_no) + ": expected " + std::to_string(columns) + " columns");
    }
    RunSample s;
    s.t_s = v[0]; s.x = v[1]; s.y = v[2]; s.z = v[3]; s.force_n = v[4]; s.tool_active = v[5] != 0.0;
    if (columns == 7) {s.tilt_deg = v[6];}
    out.push_back(s);
  }
  if (!header_seen) {throw std::runtime_error("sample csv: empty, no header");}
  return out;
}

namespace
{
Eigen::Vector3d vec3(const nlohmann::json & j, const char * key)
{
  if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 3) {
    throw std::runtime_error(std::string("wall json: '") + key + "' must be an array of 3 numbers");
  }
  return {j.at(key)[0].get<double>(), j.at(key)[1].get<double>(), j.at(key)[2].get<double>()};
}
std::pair<double, double> range(const nlohmann::json & j, const char * key)
{
  if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 2) {
    throw std::runtime_error(std::string("wall json: '") + key + "' must be [min, max]");
  }
  const double lo = j.at(key)[0].get<double>(), hi = j.at(key)[1].get<double>();
  if (!(hi > lo)) {throw std::runtime_error(std::string("wall json: '") + key + "' needs max > min");}
  return {lo, hi};
}
}  // namespace

WallPlane wallFromJson(const nlohmann::json & j)
{
  const Eigen::Vector3d origin = vec3(j, "origin"), u = vec3(j, "u"), v = vec3(j, "v");
  WallPlane wall;
  wall.id = j.value("id", 1U);
  try {
    wall.pose.pose = encodeWallBasis(origin, u, v, u.cross(v));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(std::string("wall json: ") + e.what());
  }
  std::tie(wall.u_min, wall.u_max) = range(j, "u_range");
  std::tie(wall.v_min, wall.v_max) = range(j, "v_range");
  return wall;
}

std::vector<ObstacleRegion> obstaclesFromJson(const nlohmann::json & j, uint32_t wall_id)
{
  std::vector<ObstacleRegion> out;
  if (!j.contains("obstacles")) {return out;}
  uint32_t next_id = 0;
  for (const auto & o : j.at("obstacles")) {
    ObstacleRegion r;
    r.id = o.value("id", next_id);
    next_id = r.id + 1;
    r.parent_wall = wall_id;
    std::tie(r.u_min, r.u_max) = range(o, "u");
    std::tie(r.v_min, r.v_max) = range(o, "v");
    r.height_m = o.value("height_m", 0.0);
    out.push_back(r);
  }
  return out;
}

void validateGateOptions(const ScorecardOptions & options, const ScorecardLimits & limits)
{
  const bool gap_gate = options.contact && options.max_wall_gap_m > 0.0;   // ignored for a non-contact tool
  if (limits.max_bad_pose_fraction >= 0.0 && !gap_gate && options.max_tilt_deg <= 0.0) {
    throw std::runtime_error(
      "--max-bad-pose needs --max-wall-gap (contact runs only) or --max-tilt, otherwise no sample can be a bad pose");
  }
  if (options.nominal_tilt_deg != 0.0 && options.max_tilt_deg <= 0.0) {
    throw std::runtime_error("--nominal-tilt needs --max-tilt, otherwise the tilt is not checked");
  }
}

nlohmann::json scorecardToJson(const RunScorecard & c)
{
  nlohmann::json j = {
    {"passed", c.passed},
    {"failures", c.failures},
    {"coverage", {
      {"covered_fraction", c.coverage.covered_fraction},
      {"target_cells", c.coverage.target_cells},
      {"covered_cells", c.coverage.covered_cells},
      {"missed_cells", c.coverage.missed.size()},
      {"mean_overlap", c.coverage.mean_overlap},
      {"outside_cells", c.coverage.outside_cells}}},
    {"force", {
      {"rms_error_n", c.force_rms_error_n},
      {"peak_n", c.force_peak_n},
      {"in_band_fraction", c.in_band_fraction}}},
    {"time", {{"duration_s", c.duration_s}, {"engaged_s", c.engaged_s}}},
    {"obstacle", {{"contacts", c.obstacle_contacts}, {"contact_samples", c.obstacle_contact_samples}}},
    {"pose", {
      {"bad_pose_samples", c.bad_pose_samples}, {"off_wall_samples", c.off_wall_samples},
      {"tilted_samples", c.tilted_samples}, {"bad_pose_fraction", c.bad_pose_fraction},
      {"tilt_mean_deg", c.tilt_mean_deg}, {"tilt_max_deg", c.tilt_max_deg},
      {"wall_gap_mean_m", c.wall_gap_mean_m}, {"wall_gap_max_abs_m", c.wall_gap_max_abs_m}}},
    {"strokes", c.strokes}};
  if (c.tcp_speed.windows > 0) {
    const auto & t = c.tcp_speed;
    j["tcp_speed"] = {
      {"windows", t.windows}, {"mean_mps", t.mean_mps}, {"median_mps", t.median_mps}, {"std_mps", t.std_mps},
      {"p5_mps", t.p5_mps}, {"p95_mps", t.p95_mps}};
    if (t.target_mps > 0.0) {
      j["tcp_speed"]["target_mps"] = t.target_mps;
      j["tcp_speed"]["median_rel_dev"] = t.median_rel_dev;
    }
  }
  if (c.coverage.thickness.valid) {
    j["thickness"] = {
      {"mean", c.coverage.thickness.mean}, {"cov", c.coverage.thickness.cov},
      {"min", c.coverage.thickness.min}, {"max", c.coverage.thickness.max}};
  }
  return j;
}

std::vector<SegmentWindow> segmentsFromJson(const nlohmann::json & j)
{
  if (!j.is_array()) {throw std::runtime_error("segments: expected an array");}
  std::vector<SegmentWindow> out;
  for (const auto & e : j) {
    if (!e.is_object() || !e.contains("id") || !e.contains("t_start") || !e.contains("t_end")) {
      throw std::runtime_error("segments: each entry needs id, t_start and t_end");
    }
    SegmentWindow w;
    w.id = e.at("id").get<uint32_t>();
    w.t_start_s = e.at("t_start").get<double>();
    w.t_end_s = e.at("t_end").get<double>();
    if (w.t_end_s < w.t_start_s) {throw std::runtime_error("segments: t_end before t_start");}
    out.push_back(w);
  }
  return out;
}

nlohmann::json verifyResultToJson(const VerifyResult & r)
{
  return {
    {"passed", r.passed},
    {"coverage_percent", r.coverage_percent},
    {"failed_segment_ids", r.failed_segment_ids},
    {"report_path", r.report_path},
    {"source", r.source},
    {"failures", r.failures}};
}

}  // namespace coverage_planner
