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

double CoverageVerifier::thickness(int column, int row) const
{
  if (thickness_.empty() || column < 0 || row < 0 || column >= width_ || row >= height_) {return 0.0;}
  return thickness_[static_cast<size_t>(row) * width_ + column];
}

void CoverageVerifier::depositSegment(
  double u0, double v0, double u1, double v1, const SprayFootprint & fp)
{
  const double du = u1 - u0, dv = v1 - v0;
  const double len = std::sqrt(du * du + dv * dv);
  if (!(len > 0.0)) {return;}
  const double h = fp.half_width_m;
  const double lo_u = std::min(u0, u1) - h, hi_u = std::max(u0, u1) + h;
  const double lo_v = std::min(v0, v1) - h, hi_v = std::max(v0, v1) + h;
  const int c0 = std::max(0, static_cast<int>(std::floor((lo_u - u_min_) / res_)));
  const int c1 = std::min(width_ - 1, static_cast<int>(std::floor((hi_u - u_min_) / res_)));
  const int r0 = std::max(0, static_cast<int>(std::floor((lo_v - v_min_) / res_)));
  const int r1 = std::min(height_ - 1, static_cast<int>(std::floor((hi_v - v_min_) / res_)));
  const double nu = du / len, nv = dv / len;   // unit travel direction
  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      const double pu = u_min_ + (c + 0.5) * res_ - u0;
      const double pv = v_min_ + (r + 0.5) * res_ - v0;
      const double along = pu * nu + pv * nv;
      // half-open [0, len): consecutive segments of a polyline never count a cell twice
      if (along < 0.0 || along >= len) {continue;}
      const double across = -pu * nv + pv * nu;
      thickness_[static_cast<size_t>(r) * width_ + c] += fp.density(across) * len;
    }
  }
}

void CoverageVerifier::addSprayStroke(const ToolStroke & stroke, const SprayFootprint & footprint)
{
  addStroke(stroke);
  if (thickness_.empty()) {thickness_.assign(hits_.size(), 0.0);}
  const auto & s = stroke.samples;
  for (size_t i = 0; i + 1 < s.size(); ++i) {
    if (s[i].engaged && s[i + 1].engaged) {depositSegment(s[i].u, s[i].v, s[i + 1].u, s[i + 1].v, footprint);}
  }
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
  if (!thickness_.empty()) {
    double sum = 0.0, sq = 0.0, lo = 0.0, hi = 0.0;
    size_t n = 0;
    for (size_t i = 0; i < hits_.size(); ++i) {
      if (!target_[i] || hits_[i] == 0) {continue;}
      const double t = thickness_[i];
      lo = n == 0 ? t : std::min(lo, t);
      hi = n == 0 ? t : std::max(hi, t);
      sum += t;
      sq += t * t;
      ++n;
    }
    r.thickness.valid = true;
    if (n > 0) {
      const double mean = sum / static_cast<double>(n);
      const double var = std::max(0.0, sq / static_cast<double>(n) - mean * mean);
      r.thickness.mean = mean;
      r.thickness.cov = mean > 0.0 ? std::sqrt(var) / mean : 0.0;
      r.thickness.min = lo;
      r.thickness.max = hi;
    }
  }
  r.mean_overlap = r.covered_cells == 0 ? 0.0 :
    static_cast<double>(overlap_sum) / static_cast<double>(r.covered_cells);
  return r;
}

}  // namespace coverage_planner
