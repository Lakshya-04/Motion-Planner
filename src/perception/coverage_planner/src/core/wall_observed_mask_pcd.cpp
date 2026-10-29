// Point-cloud (.pcd) loading for the wall observed-mask. Kept in its own translation unit so
// the rest of the core builds without PCL.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "coverage_planner/core/wall_basis.hpp"
#include "coverage_planner/core/wall_rasterizer.hpp"

namespace coverage_planner
{

std::vector<bool> loadWallObservedMask(
  const std::string & pcd_path, uint32_t wall_id, const WallPlane & wall, int width,
  int height, double resolution)
{
  pcl::PointCloud<pcl::PointXYZL> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZL>(pcd_path, cloud) < 0) {
    throw std::runtime_error("loadWallObservedMask: failed to load point cloud from " + pcd_path);
  }

  const WallBasis basis = decomposeWallBasis(wall);

  std::vector<bool> mask(static_cast<size_t>(width) * static_cast<size_t>(height), false);
  for (const auto & point : cloud.points) {
    if (point.label != wall_id) {
      continue;
    }
    const Eigen::Vector3d p(point.x, point.y, point.z);
    const double u = (p - basis.origin).dot(basis.basis_u);
    const double v = (p - basis.origin).dot(basis.basis_v);
    const int x = static_cast<int>(std::floor((u - wall.u_min) / resolution));
    const int y = static_cast<int>(std::floor((v - wall.v_min) / resolution));
    if (x >= 0 && x < width && y >= 0 && y < height) {
      mask[static_cast<size_t>(y) * width + x] = true;
    }
  }
  return mask;
}

}  // namespace coverage_planner
