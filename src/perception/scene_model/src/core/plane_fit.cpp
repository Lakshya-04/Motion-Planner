#include "scene_model/core/plane_fit.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>

namespace scene_model
{

Transform PlaneFit::pose() const
{
  Transform T = Transform::Identity();
  T.linear().col(0) = u;
  T.linear().col(1) = v;
  T.linear().col(2) = normal;
  T.translation() = origin;
  return T;
}

namespace
{

// Least-squares plane through `pts` (indices into `all`): centroid + smallest-eigenvector normal.
bool pcaPlane(const std::vector<Vec3> & all, const std::vector<size_t> & idx, Vec3 & centroid, Vec3 & normal)
{
  if (idx.size() < 3) {
    return false;
  }
  centroid.setZero();
  for (size_t i : idx) {centroid += all[i];}
  centroid /= static_cast<double>(idx.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (size_t i : idx) {
    const Vec3 d = all[i] - centroid;
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  normal = solver.eigenvectors().col(0);  // eigenvalues ascending: col 0 is the smallest
  return true;
}

}  // namespace

std::optional<PlaneFit> fitPlane(const std::vector<Vec3> & points, const PlaneFitParams & p)
{
  const size_t n = points.size();
  if (n < std::max<size_t>(p.min_inliers, 3)) {
    return std::nullopt;
  }

  std::mt19937 rng(p.seed);
  std::uniform_int_distribution<size_t> pick(0, n - 1);
  std::vector<size_t> best;
  for (int it = 0; it < p.ransac_iterations; ++it) {
    const size_t a = pick(rng), b = pick(rng), c = pick(rng);
    if (a == b || b == c || a == c) {continue;}
    Vec3 nrm = (points[b] - points[a]).cross(points[c] - points[a]);
    const double len = nrm.norm();
    if (len < 1e-12) {continue;}
    nrm /= len;
    std::vector<size_t> in;
    for (size_t i = 0; i < n; ++i) {
      if (std::abs(nrm.dot(points[i] - points[a])) <= p.inlier_threshold) {in.push_back(i);}
    }
    if (in.size() > best.size()) {best = std::move(in);}
  }
  if (best.size() < p.min_inliers) {
    return std::nullopt;
  }

  // Refine on the inliers, then re-collect inliers once against the refined plane.
  Vec3 centroid, normal;
  if (!pcaPlane(points, best, centroid, normal)) {
    return std::nullopt;
  }
  std::vector<size_t> refined;
  for (size_t i = 0; i < n; ++i) {
    if (std::abs(normal.dot(points[i] - centroid)) <= p.inlier_threshold) {refined.push_back(i);}
  }
  if (refined.size() >= p.min_inliers && pcaPlane(points, refined, centroid, normal)) {
    best = std::move(refined);
  }

  // Orient the normal.
  if (p.normal_hint.norm() > 0.0) {
    if (normal.dot(p.normal_hint) < 0.0) {normal = -normal;}
  } else if (normal.dot(p.up) < 0.0 && std::abs(normal.dot(p.up)) > 0.5) {
    normal = -normal;  // near-horizontal plane: point up
  }

  // In-plane frame: u horizontal along the wall, v = normal x u so that u x v = normal.
  Vec3 u = p.up.cross(normal);
  if (u.norm() < 1e-6) {  // plane is horizontal: fall back to a fixed axis
    u = Vec3::UnitX() - normal * normal.x();
  }
  u.normalize();
  Vec3 v = normal.cross(u);

  PlaneFit fit;
  fit.origin = centroid;
  fit.u = u;
  fit.v = v;
  fit.normal = normal;
  fit.inliers = best;
  fit.u_min = fit.v_min = std::numeric_limits<double>::infinity();
  fit.u_max = fit.v_max = -std::numeric_limits<double>::infinity();
  std::vector<double> dist;
  dist.reserve(best.size());
  double sum_sq = 0.0;
  for (size_t i : best) {
    const Vec3 d = points[i] - centroid;
    fit.u_min = std::min(fit.u_min, d.dot(u));
    fit.u_max = std::max(fit.u_max, d.dot(u));
    fit.v_min = std::min(fit.v_min, d.dot(v));
    fit.v_max = std::max(fit.v_max, d.dot(v));
    const double h = std::abs(d.dot(normal));
    dist.push_back(h);
    sum_sq += h * h;
  }
  fit.rms = std::sqrt(sum_sq / static_cast<double>(best.size()));
  std::sort(dist.begin(), dist.end());
  auto pct = [&](double q) {
      return dist[std::min(dist.size() - 1, static_cast<size_t>(q * static_cast<double>(dist.size())))];
    };
  fit.spread_p95 = pct(0.95);
  fit.spread_p99 = pct(0.99);
  return fit;
}

std::optional<PlaneFit> fitPlaneForLabel(
  const PointCloud & cloud, uint32_t label, const PlaneFitParams & params)
{
  std::vector<Vec3> pts;
  std::vector<size_t> source;
  const bool labelled = cloud.labels.size() == cloud.points.size();
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if (!labelled || cloud.labels[i] == label) {
      pts.push_back(cloud.points[i]);
      source.push_back(i);
    }
  }
  auto fit = fitPlane(pts, params);
  if (fit) {
    for (auto & i : fit->inliers) {i = source[i];}  // map back to the full cloud
  }
  return fit;
}

}  // namespace scene_model
