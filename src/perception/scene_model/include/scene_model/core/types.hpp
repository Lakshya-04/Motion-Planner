// Backend-neutral scene types. Plain C++17 + Eigen, no ROS, no planner or sensor dependency.
//
// One SceneModel holds everything the stack knows about the world, in whatever form it arrived
// (point cloud, mesh, primitive, voxel grid, signed distance field). Adapters convert it to
// what each backend wants (MoveIt collision objects, a cuRobo scene dict / ESDF, VAMP primitives
// + point cloud) -- see converters.hpp.
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Geometry>

namespace scene_model
{

using Vec3 = Eigen::Vector3d;
using Transform = Eigen::Isometry3d;

/// What a scene object means to the task. Kept separate on purpose: a contact surface (the wall
/// the tool may touch) must never be merged with obstacles, or a contact exclusion could
/// silently disable a real obstacle.
enum class Role
{
  kObstacle,
  kContactSurface,
};

struct Box
{
  Vec3 size{Vec3::Zero()};  ///< Full extents along the object's local x, y, z.
};

struct Sphere
{
  double radius{0.0};
};

/// Axis along the object's local z.
struct Cylinder
{
  double radius{0.0};
  double height{0.0};
};

/// Capsule along the object's local z, centred at the origin; `length` is the straight section.
struct Capsule
{
  double radius{0.0};
  double length{0.0};
};

struct Mesh
{
  std::vector<Vec3> vertices;
  std::vector<std::array<int, 3>> faces;  ///< Triangles, indices into `vertices`.
  std::string source_path;                ///< File the mesh came from, if any.
};

struct PointCloud
{
  std::vector<Vec3> points;
  std::vector<uint32_t> labels;                 ///< Empty, or one per point (e.g. wall id).
  std::vector<std::array<uint8_t, 3>> colors;   ///< Empty, or one per point.
};

/// Voxel states. A grid that comes from a sensor (not just a point cloud) distinguishes space
/// that was observed free from space never observed; planners must not treat the two alike.
constexpr uint8_t kVoxelFree = 0;
constexpr uint8_t kVoxelOccupied = 1;
constexpr uint8_t kVoxelUnknown = 2;

/// Occupancy grid (kVoxelFree / kVoxelOccupied / kVoxelUnknown). Voxel (i, j, k) has its centre
/// at origin + voxel_size * (i, j, k); index is i + nx * (j + ny * k), x fastest.
struct VoxelGrid
{
  double voxel_size{0.0};
  Vec3 origin{Vec3::Zero()};
  int nx{0};
  int ny{0};
  int nz{0};
  std::vector<uint8_t> occupied;

  size_t index(int i, int j, int k) const
  {
    return static_cast<size_t>(i) + static_cast<size_t>(nx) *
           (static_cast<size_t>(j) + static_cast<size_t>(ny) * static_cast<size_t>(k));
  }
};

/// Signed distance field on the same grid convention as VoxelGrid: metres, positive in free
/// space (distance to the nearest occupied voxel), negative inside obstacles.
struct EsdfGrid
{
  double voxel_size{0.0};
  Vec3 origin{Vec3::Zero()};
  int nx{0};
  int ny{0};
  int nz{0};
  std::vector<float> distance;

  size_t index(int i, int j, int k) const
  {
    return static_cast<size_t>(i) + static_cast<size_t>(nx) *
           (static_cast<size_t>(j) + static_cast<size_t>(ny) * static_cast<size_t>(k));
  }
};

using Geometry = std::variant<Box, Sphere, Cylinder, Capsule, Mesh, PointCloud, VoxelGrid, EsdfGrid>;

struct SceneObject
{
  std::string id;                  ///< Unique within a scene.
  std::string frame_id;            ///< Frame `pose` is expressed in; empty = the scene's frame.
  Transform pose{Transform::Identity()};  ///< Geometry-local -> frame_id.
  Geometry geometry;
  Role role{Role::kObstacle};
  bool is_static{true};            ///< False for dynamic content that is re-sensed.
};

struct Aabb
{
  Vec3 min{Vec3::Constant(std::numeric_limits<double>::infinity())};
  Vec3 max{Vec3::Constant(-std::numeric_limits<double>::infinity())};
  bool empty() const { return (min.array() > max.array()).any(); }
  void extend(const Vec3 & p)
  {
    min = min.cwiseMin(p);
    max = max.cwiseMax(p);
  }
};

}  // namespace scene_model
