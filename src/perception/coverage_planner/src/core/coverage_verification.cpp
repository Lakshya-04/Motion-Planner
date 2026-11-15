#include "coverage_planner/core/coverage_verification.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace coverage_planner
{

CoverageVerifier::CoverageVerifier(
  double u_min, double u_max, double v_min, double v_max, double resolution,
  const std::vector<bool> & target)
: u_min_(u_min), v_min_(v_min), res_(resolution)
{
  if (!(resolution > 0.0) || !(u_max > u_min) || !(v_max > v_min)) {
    throw std::invalid_argument("coverage verification: resolution and the u/v extent must be positive");
  }
  width_ = static_cast<int>(std::ceil((u_max - u_min) / resolution));
  height_ = static_cast<int>(std::ceil((v_max - v_min) / resolution));
  const size_t n = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  if (!target.empty() && target.size() != n) {
    throw std::invalid_argument("coverage verification: target mask size must be width * height");
  }
  target_ = target.empty() ? std::vector<bool>(n, true) : target;
  hits_.assign(n, 0);
  stamp_.assign(n, 0);
}

uint16_t CoverageVerifier::hits(int column, int row) const
{
  if (column < 0 || row < 0 || column >= width_ || row >= height_) {return 0;}
  return hits_[static_cast<size_t>(row) * width_ + column];
}

void CoverageVerifier::markSegment(double u0, double v0, double u1, double v1, double radius)
{
  const double lo_u = std::min(u0, u1) - radius, hi_u = std::max(u0, u1) + radius;
  const double lo_v = std::min(v0, v1) - radius, hi_v = std::max(v0, v1) + radius;
  const int c0 = std::max(0, static_cast<int>(std::floor((lo_u - u_min_) / res_)));
  const int c1 = std::min(width_ - 1, static_cast<int>(std::floor((hi_u - u_min_) / res_)));
  const int r0 = std::max(0, static_cast<int>(std::floor((lo_v - v_min_) / res_)));
  const int r1 = std::min(height_ - 1, static_cast<int>(std::floor((hi_v - v_min_) / res_)));
  const double du = u1 - u0, dv = v1 - v0;
  const double len2 = du * du + dv * dv;
  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      const double pu = u_min_ + (c + 0.5) * res_;
      const double pv = v_min_ + (r + 0.5) * res_;
      double t = len2 > 0.0 ? ((pu - u0) * du + (pv - v0) * dv) / len2 : 0.0;
      t = std::clamp(t, 0.0, 1.0);
      const double dx = pu - (u0 + t * du), dy = pv - (v0 + t * dv);
      if (dx * dx + dy * dy > radius * radius) {continue;}
      const size_t i = static_cast<size_t>(r) * width_ + c;
      if (stamp_[i] == stroke_id_) {continue;}   // already counted for this stroke
      stamp_[i] = stroke_id_;
      ++hits_[i];
    }
  }
}

void CoverageVerifier::addStroke(const ToolStroke & stroke)
{
  if (!(stroke.swath_m > 0.0)) {
    throw std::invalid_argument("coverage verification: swath_m must be > 0");
  }
  ++stroke_id_;
  const double radius = 0.5 * stroke.swath_m;
  const auto & s = stroke.samples;
  for (size_t i = 0; i < s.size(); ++i) {
    if (!s[i].engaged) {continue;}
    if (i + 1 < s.size() && s[i + 1].engaged) {
      markSegment(s[i].u, s[i].v, s[i + 1].u, s[i + 1].v, radius);
    } else if (i == 0 || !s[i - 1].engaged) {
      markSegment(s[i].u, s[i].v, s[i].u, s[i].v, radius);   // a lone engaged sample
    }
  }
}

VerificationReport CoverageVerifier::report() const
{
  VerificationReport r;
  size_t overlap_sum = 0;
  for (size_t i = 0; i < hits_.size(); ++i) {
    if (target_[i]) {
      ++r.target_cells;
      if (hits_[i] > 0) {
        ++r.covered_cells;
        overlap_sum += hits_[i];
      } else {
        r.missed.push_back(i);
      }
    } else if (hits_[i] > 0) {
      ++r.outside_cells;
    }
  }
  r.covered_fraction = r.target_cells == 0 ? 1.0 :
    static_cast<double>(r.covered_cells) / static_cast<double>(r.target_cells);
  r.mean_overlap = r.covered_cells == 0 ? 0.0 :
    static_cast<double>(overlap_sum) / static_cast<double>(r.covered_cells);
  return r;
}

}  // namespace coverage_planner
