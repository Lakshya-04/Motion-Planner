#include "wall_detector/core/octree_esdf.hpp"

#include <algorithm>
#include <cmath>

#include "scene_model/core/voxel.hpp"

namespace wall_detector
{

scene_model::EsdfGrid octreeToEsdf(const octomap::OcTree & tree, const EsdfWindow & w)
{
  scene_model::VoxelGrid grid;
  grid.voxel_size = w.voxel_size;
  grid.nx = std::max(1, static_cast<int>(std::lround(w.dims.x() / w.voxel_size)));
  grid.ny = std::max(1, static_cast<int>(std::lround(w.dims.y() / w.voxel_size)));
  grid.nz = std::max(1, static_cast<int>(std::lround(w.dims.z() / w.voxel_size)));
  grid.origin = w.center - 0.5 * Eigen::Vector3d(grid.nx, grid.ny, grid.nz) * w.voxel_size;
  grid.occupied.assign(static_cast<size_t>(grid.nx) * grid.ny * grid.nz, 0);
  const int n[3] = {grid.nx, grid.ny, grid.nz};

  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it)) {
      continue;
    }
    const double half = it.getSize() * 0.5;
    const double c[3] = {it.getX(), it.getY(), it.getZ()};
    int lo[3], hi[3];
    for (int a = 0; a < 3; ++a) {
      // voxel centres inside the leaf cube [c - half, c + half)
      lo[a] = std::max(
        0, static_cast<int>(std::ceil((c[a] - half - grid.origin[a]) / w.voxel_size - 0.5)));
      hi[a] = std::min(
        n[a] - 1,
        static_cast<int>(std::ceil((c[a] + half - grid.origin[a]) / w.voxel_size - 0.5)) - 1);
    }
    for (int k = lo[2]; k <= hi[2]; ++k) {
      for (int j = lo[1]; j <= hi[1]; ++j) {
        for (int i = lo[0]; i <= hi[0]; ++i) {
          grid.occupied[grid.index(i, j, k)] = 1;
        }
      }
    }
  }
  return scene_model::computeEsdf(grid);
}

}  // namespace wall_detector
