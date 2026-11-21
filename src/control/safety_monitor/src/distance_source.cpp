/**
 * @file distance_source.cpp
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include "safety_monitor/distance_source.hpp"

#include <algorithm>
#include <cmath>

namespace safety_monitor
{

namespace
{
Vec3 unitOr(const Vec3 & v, const Vec3 & fallback)
{
  const double n = v.norm();
  return n > 1e-12 ? Vec3(v / n) : fallback;
}
}  // namespace

void StaticPrimitiveSource::addSphere(const Vec3 & center, double radius)
{
  Prim p;
  p.type = Prim::kSphere;
  p.pose.translation() = center;
  p.s = radius;
  prims_.push_back(p);
}

void StaticPrimitiveSource::addBox(const Eigen::Isometry3d & pose, const Vec3 & half_extents)
{
  Prim p;
  p.type = Prim::kBox;
  p.pose = pose;
  p.a = half_extents;
  prims_.push_back(p);
}

void StaticPrimitiveSource::addPlane(const Vec3 & normal, double offset)
{
  Prim p;
  p.type = Prim::kPlane;
  const double n = normal.norm();
  p.a = normal / n;
  p.s = offset / n;
  prims_.push_back(p);
}

DistanceQuery StaticPrimitiveSource::query(const Vec3 & pt, double) const
{
  DistanceQuery best;
  for (const auto & pr : prims_) {
    double d = 0.0;
    Vec3 g = Vec3::UnitZ();
    switch (pr.type) {
      case Prim::kSphere: {
        const Vec3 v = pt - pr.pose.translation();
        d = v.norm() - pr.s;
        g = unitOr(v, Vec3::UnitZ());
        break;
      }
      case Prim::kPlane:
        d = pr.a.dot(pt) - pr.s;
        g = pr.a;
        break;
      case Prim::kBox: {
        const Vec3 l = pr.pose.inverse() * pt;
        const Vec3 q = l.cwiseAbs() - pr.a;
        const Vec3 out = q.cwiseMax(0.0);
        Vec3 gl;
        if (out.norm() > 0.0) {
          d = out.norm();
          gl = unitOr(out.cwiseProduct(l.cwiseSign()), Vec3::UnitZ());
        } else {
          // inside: distance to the nearest face, gradient towards it
          int axis;
          d = q.maxCoeff(&axis);  // negative
          gl = Vec3::Zero();
          gl[axis] = l[axis] >= 0.0 ? 1.0 : -1.0;
        }
        g = pr.pose.linear() * gl;
        break;
      }
    }
    if (!best.valid || d < best.distance) {
      best.valid = true;
      best.distance = d;
      best.gradient = g;
    }
  }
  return best;
}

EsdfGrid::EsdfGrid(const Vec3 & origin, double voxel, int nx, int ny, int nz)
: origin_(origin), voxel_(voxel), nx_(nx), ny_(ny), nz_(nz),
  values_(static_cast<size_t>(std::max(nx, 0)) * std::max(ny, 0) * std::max(nz, 0), 0.0f)
{
}

DistanceQuery EsdfGrid::query(const Vec3 & p, double) const
{
  DistanceQuery q;
  if (values_.empty() || nx_ < 2 || ny_ < 2 || nz_ < 2 || !p.allFinite()) {return q;}
  const Vec3 g = (p - origin_) / voxel_;
  const double hi[3] = {nx_ - 1.0, ny_ - 1.0, nz_ - 1.0};
  for (int i = 0; i < 3; ++i) {
    if (g[i] < 0.0 || g[i] > hi[i]) {return q;}
  }
  int i0[3];
  double f[3];
  for (int i = 0; i < 3; ++i) {
    i0[i] = std::min(static_cast<int>(std::floor(g[i])), static_cast<int>(hi[i]) - 1);
    f[i] = g[i] - i0[i];
  }
  double c[2][2][2];
  for (int dz = 0; dz < 2; ++dz) {
    for (int dy = 0; dy < 2; ++dy) {
      for (int dx = 0; dx < 2; ++dx) {
        c[dz][dy][dx] = values_[index(i0[0] + dx, i0[1] + dy, i0[2] + dz)];
      }
    }
  }
  auto lerp = [](double a, double b, double t) {return a + (b - a) * t;};
  double cx[2][2], d;
  for (int dz = 0; dz < 2; ++dz) {
    for (int dy = 0; dy < 2; ++dy) {cx[dz][dy] = lerp(c[dz][dy][0], c[dz][dy][1], f[0]);}
  }
  const double cy0 = lerp(cx[0][0], cx[0][1], f[1]);
  const double cy1 = lerp(cx[1][0], cx[1][1], f[1]);
  d = lerp(cy0, cy1, f[2]);
  // analytic gradient of the trilinear interpolant
  double dxv[2][2];
  for (int dz = 0; dz < 2; ++dz) {
    for (int dy = 0; dy < 2; ++dy) {dxv[dz][dy] = c[dz][dy][1] - c[dz][dy][0];}
  }
  const double gx = lerp(lerp(dxv[0][0], dxv[0][1], f[1]), lerp(dxv[1][0], dxv[1][1], f[1]), f[2]);
  const double gy = lerp(cx[0][1] - cx[0][0], cx[1][1] - cx[1][0], f[2]);
  const double gz = cy1 - cy0;
  q.valid = std::isfinite(d);
  q.distance = d;
  q.gradient = unitOr(Vec3(gx, gy, gz), Vec3::UnitZ());
  return q;
}

DistanceQuery MovingCapsuleSource::query(const Vec3 & p, double t_ahead) const
{
  DistanceQuery best;
  for (const auto & c : capsules_) {
    const Vec3 shift = c.velocity * t_ahead;
    const Vec3 a = c.a + shift;
    const Vec3 ab = c.b + shift - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 1e-18 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    const Vec3 v = p - (a + t * ab);
    const double d = v.norm() - c.radius;
    if (!best.valid || d < best.distance) {
      best.valid = true;
      best.distance = d;
      best.gradient = unitOr(v, Vec3::UnitZ());
      best.velocity = c.velocity;
    }
  }
  return best;
}

}  // namespace safety_monitor
