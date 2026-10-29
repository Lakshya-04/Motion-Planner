#include "scene_model/core/voxel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace scene_model
{

VoxelGrid voxelize(
  const PointCloud & cloud, double voxel_size, int min_points, double margin, size_t max_voxels)
{
  if (voxel_size <= 0.0) {
    throw std::invalid_argument("voxelize: voxel_size must be positive");
  }
  VoxelGrid grid;
  grid.voxel_size = voxel_size;
  if (cloud.points.empty()) {
    return grid;
  }
  Aabb box;
  for (const auto & p : cloud.points) {box.extend(p);}
  const Vec3 lo = box.min - Vec3::Constant(margin);
  const Vec3 hi = box.max + Vec3::Constant(margin);
  grid.origin = lo;  // centre of voxel (0,0,0)
  // Points snap to the nearest voxel centre (lround below), so the extent must use the same
  // rounding; floor() undercounts by one when extent / voxel_size is 2.9999999999999996.
  grid.nx = static_cast<int>(std::lround((hi.x() - lo.x()) / voxel_size)) + 1;
  grid.ny = static_cast<int>(std::lround((hi.y() - lo.y()) / voxel_size)) + 1;
  grid.nz = static_cast<int>(std::lround((hi.z() - lo.z()) / voxel_size)) + 1;
  const double total = static_cast<double>(grid.nx) * grid.ny * grid.nz;
  if (total > static_cast<double>(max_voxels)) {
    throw std::runtime_error(
            "voxelize: grid would have " + std::to_string(static_cast<long long>(total)) +
            " voxels; raise voxel_size or crop the cloud");
  }
  std::vector<int> count(static_cast<size_t>(total), 0);
  for (const auto & p : cloud.points) {
    const Vec3 r = (p - grid.origin) / voxel_size;
    const int i = std::clamp(static_cast<int>(std::lround(r.x())), 0, grid.nx - 1);
    const int j = std::clamp(static_cast<int>(std::lround(r.y())), 0, grid.ny - 1);
    const int k = std::clamp(static_cast<int>(std::lround(r.z())), 0, grid.nz - 1);
    ++count[grid.index(i, j, k)];
  }
  grid.occupied.resize(count.size());
  for (size_t n = 0; n < count.size(); ++n) {
    grid.occupied[n] = count[n] >= min_points ? 1 : 0;
  }
  return grid;
}

PointCloud occupiedCentres(const VoxelGrid & grid)
{
  PointCloud out;
  for (int k = 0; k < grid.nz; ++k) {
    for (int j = 0; j < grid.ny; ++j) {
      for (int i = 0; i < grid.nx; ++i) {
        if (grid.occupied[grid.index(i, j, k)] == kVoxelOccupied) {
          out.points.push_back(grid.origin + grid.voxel_size * Vec3(i, j, k));
        }
      }
    }
  }
  return out;
}

namespace
{

constexpr double kInf = 1e20;

// 1D squared Euclidean distance transform of `f` (Felzenszwalb & Huttenlocher), in place.
void dt1d(std::vector<double> & f, std::vector<double> & d, std::vector<int> & v, std::vector<double> & z)
{
  const int n = static_cast<int>(f.size());
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    double s;
    while (true) {
      s = ((f[q] + static_cast<double>(q) * q) - (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
        (2.0 * q - 2.0 * v[k]);
      if (s <= z[k] && k > 0) {
        --k;
      } else {
        break;
      }
    }
    if (s <= z[k]) {
      v[k] = q;
      z[k] = -kInf;
      z[k + 1] = kInf;
    } else {
      ++k;
      v[k] = q;
      z[k] = s;
      z[k + 1] = kInf;
    }
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    const double dq = static_cast<double>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
  f = d;
}

// Squared distance (in voxel units) from every voxel to the nearest voxel whose `seed` is set.
std::vector<double> squaredDistanceTo(const VoxelGrid & g, const std::vector<uint8_t> & occ, bool seed_value)
{
  const size_t n = static_cast<size_t>(g.nx) * g.ny * g.nz;
  std::vector<double> sq(n);
  for (size_t i = 0; i < n; ++i) {
    sq[i] = ((occ[i] != 0) == seed_value) ? 0.0 : kInf;
  }
  // x
  for (int k = 0; k < g.nz; ++k) {
    for (int j = 0; j < g.ny; ++j) {
      std::vector<double> f(g.nx), d(g.nx), z(g.nx + 1);
      std::vector<int> v(g.nx);
      for (int i = 0; i < g.nx; ++i) {f[i] = sq[g.index(i, j, k)];}
      dt1d(f, d, v, z);
      for (int i = 0; i < g.nx; ++i) {sq[g.index(i, j, k)] = f[i];}
    }
  }
  // y
  for (int k = 0; k < g.nz; ++k) {
    for (int i = 0; i < g.nx; ++i) {
      std::vector<double> f(g.ny), d(g.ny), z(g.ny + 1);
      std::vector<int> v(g.ny);
      for (int j = 0; j < g.ny; ++j) {f[j] = sq[g.index(i, j, k)];}
      dt1d(f, d, v, z);
      for (int j = 0; j < g.ny; ++j) {sq[g.index(i, j, k)] = f[j];}
    }
  }
  // z
  for (int j = 0; j < g.ny; ++j) {
    for (int i = 0; i < g.nx; ++i) {
      std::vector<double> f(g.nz), d(g.nz), z(g.nz + 1);
      std::vector<int> v(g.nz);
      for (int k = 0; k < g.nz; ++k) {f[k] = sq[g.index(i, j, k)];}
      dt1d(f, d, v, z);
      for (int k = 0; k < g.nz; ++k) {sq[g.index(i, j, k)] = f[k];}
    }
  }
  return sq;
}

}  // namespace

EsdfGrid computeEsdf(const VoxelGrid & grid, UnobservedPolicy policy)
{
  EsdfGrid esdf;
  esdf.voxel_size = grid.voxel_size;
  esdf.origin = grid.origin;
  esdf.nx = grid.nx;
  esdf.ny = grid.ny;
  esdf.nz = grid.nz;
  const size_t n = static_cast<size_t>(grid.nx) * grid.ny * grid.nz;
  esdf.distance.assign(n, 0.0F);
  if (n == 0) {
    return esdf;
  }
  // Resolve unknown voxels to free / occupied according to the policy before the transform.
  std::vector<uint8_t> occ(n);
  for (size_t i = 0; i < n; ++i) {
    occ[i] = grid.occupied[i] == kVoxelOccupied ||
      (grid.occupied[i] == kVoxelUnknown && policy == UnobservedPolicy::kOccupied);
  }
  const auto to_occupied = squaredDistanceTo(grid, occ, true);
  const auto to_free = squaredDistanceTo(grid, occ, false);
  const float big = static_cast<float>(
    grid.voxel_size * std::sqrt(static_cast<double>(grid.nx) * grid.nx + static_cast<double>(grid.ny) * grid.ny +
    static_cast<double>(grid.nz) * grid.nz));
  for (size_t i = 0; i < n; ++i) {
    if (policy == UnobservedPolicy::kMarkUnobserved && grid.occupied[i] == kVoxelUnknown) {
      esdf.distance[i] = std::numeric_limits<float>::quiet_NaN();
    } else if (occ[i]) {
      // No free voxel anywhere -> a fully occupied grid: report the grid diagonal as "deep".
      esdf.distance[i] = to_free[i] >= kInf / 2 ? -big :
        -static_cast<float>(grid.voxel_size * std::sqrt(to_free[i]));
    } else {
      esdf.distance[i] = to_occupied[i] >= kInf / 2 ? big :
        static_cast<float>(grid.voxel_size * std::sqrt(to_occupied[i]));
    }
  }
  return esdf;
}

}  // namespace scene_model
