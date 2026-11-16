#include "coverage_planner/core/run_scorecard.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "coverage_planner/core/wall_basis.hpp"

namespace coverage_planner
{

RunScorecard scoreRun(
  const WallPlane & wall, const std::vector<ObstacleRegion> & obstacles,
  const std::vector<RunSample> & samples, const ScorecardOptions & options,
  const ScorecardLimits & limits)
{
  if (!(options.swath_m > 0.0) || !(options.resolution_m > 0.0)) {
    throw std::invalid_argument("scorecard: swath_m and resolution_m must be > 0");
  }
  const WallBasis basis = decomposeWallBasis(wall);

  // Target: the whole wall rectangle minus the obstacles on it.
  CoverageVerifier probe(wall.u_min, wall.u_max, wall.v_min, wall.v_max, options.resolution_m);
  std::vector<bool> target(static_cast<size_t>(probe.width()) * probe.height(), true);
  for (const auto & o : obstacles) {
    if (o.parent_wall != wall.id) {continue;}
    for (int r = 0; r < probe.height(); ++r) {
      const double v = wall.v_min + (r + 0.5) * options.resolution_m;
      const double k = options.keepout_margin_m;
      if (v < o.v_min - k || v > o.v_max + k) {continue;}
      for (int c = 0; c < probe.width(); ++c) {
        const double u = wall.u_min + (c + 0.5) * options.resolution_m;
        if (u >= o.u_min - k && u <= o.u_max + k) {target[static_cast<size_t>(r) * probe.width() + c] = false;}
      }
    }
  }
  CoverageVerifier verifier(wall.u_min, wall.u_max, wall.v_min, wall.v_max, options.resolution_m, target);

  RunScorecard card;
  ToolStroke stroke;
  stroke.swath_m = options.swath_m;
  auto flush = [&]() {
      if (!stroke.samples.empty()) {
        if (options.spray) {verifier.addSprayStroke(stroke, *options.spray);} else {verifier.addStroke(stroke);}
        ++card.strokes;
        stroke.samples.clear();
      }
    };

  double sq_err = 0.0;
  size_t engaged_n = 0, in_band = 0;
  bool in_obstacle = false;
  for (size_t i = 0; i < samples.size(); ++i) {
    const auto & s = samples[i];
    card.force_peak_n = std::max(card.force_peak_n, s.force_n);
    {
      // obstacle contact: the tool point inside an obstacle's footprint and not above its top
      const Eigen::Vector3d q = Eigen::Vector3d(s.x, s.y, s.z) - basis.origin;
      const double pu = q.dot(basis.basis_u), pv = q.dot(basis.basis_v), height = -q.dot(basis.normal_outward);
      bool inside = false;
      for (const auto & o : obstacles) {
        const double m = options.obstacle_margin_m;
        if (o.parent_wall == wall.id && pu >= o.u_min - m && pu <= o.u_max + m && pv >= o.v_min - m &&
          pv <= o.v_max + m && height < o.height_m + options.obstacle_contact_tol_m)
        {
          inside = true;
          break;
        }
      }
      if (inside) {
        ++card.obstacle_contact_samples;
        if (!in_obstacle) {++card.obstacle_contacts;}
      }
      in_obstacle = inside;
    }
    const bool engaged = options.contact ? s.force_n >= options.engaged_force_n : s.tool_active;
    if (!engaged) {
      flush();   // a disengaged sample ends the stroke
      continue;
    }
    const Eigen::Vector3d p = Eigen::Vector3d(s.x, s.y, s.z) - basis.origin;
    stroke.samples.push_back({p.dot(basis.basis_u), p.dot(basis.basis_v), true});
    ++engaged_n;
    if (i > 0) {card.engaged_s += std::max(0.0, s.t_s - samples[i - 1].t_s);}
    if (options.contact) {
      const double err = s.force_n - options.target_force_n;
      sq_err += err * err;
      if (std::fabs(err) <= options.force_band_n) {++in_band;}
    }
  }
  flush();

  card.coverage = verifier.report();
  if (samples.size() > 1) {card.duration_s = samples.back().t_s - samples.front().t_s;}
  if (options.contact && engaged_n > 0) {
    card.force_rms_error_n = std::sqrt(sq_err / static_cast<double>(engaged_n));
    card.in_band_fraction = static_cast<double>(in_band) / static_cast<double>(engaged_n);
  }

  auto fail = [&](const std::string & what) {card.failures.push_back(what);};
  std::ostringstream m;
  if (card.coverage.covered_fraction < limits.min_covered_fraction) {
    m << "coverage " << card.coverage.covered_fraction << " < " << limits.min_covered_fraction;
    fail(m.str());
  }
  if (options.contact) {
    if (limits.max_force_rms_error_n > 0.0 && card.force_rms_error_n > limits.max_force_rms_error_n) {
      m.str(""); m << "force RMS error " << card.force_rms_error_n << " N > " << limits.max_force_rms_error_n;
      fail(m.str());
    }
    if (limits.min_in_band_fraction > 0.0 && card.in_band_fraction < limits.min_in_band_fraction) {
      m.str(""); m << "in-band fraction " << card.in_band_fraction << " < " << limits.min_in_band_fraction;
      fail(m.str());
    }
  }
  if (limits.max_thickness_cov > 0.0 && card.coverage.thickness.valid &&
    card.coverage.thickness.cov > limits.max_thickness_cov)
  {
    m.str(""); m << "thickness CoV " << card.coverage.thickness.cov << " > " << limits.max_thickness_cov;
    fail(m.str());
  }
  if (limits.max_force_peak_n > 0.0 && card.force_peak_n > limits.max_force_peak_n) {
    m.str(""); m << "force peak " << card.force_peak_n << " N > " << limits.max_force_peak_n;
    fail(m.str());
  }
  if (limits.max_obstacle_contacts >= 0 && card.obstacle_contacts > static_cast<size_t>(limits.max_obstacle_contacts)) {
    m.str(""); m << "obstacle contacts " << card.obstacle_contacts << " > " << limits.max_obstacle_contacts;
    fail(m.str());
  }
  card.passed = card.failures.empty();
  return card;
}

}  // namespace coverage_planner
