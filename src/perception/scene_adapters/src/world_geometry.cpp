#include "scene_adapters/world_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <variant>

namespace scene_adapters
{
namespace sm = scene_model;
namespace mp = motion_planner;

namespace
{
using Vec3 = Eigen::Vector3d;

core_types::Pose toPose(const sm::Transform & t)
{
  const Eigen::Quaterniond q(t.linear());
  core_types::Pose p;
  p.x = t.translation().x();
  p.y = t.translation().y();
  p.z = t.translation().z();
  p.qx = q.x();
  p.qy = q.y();
  p.qz = q.z();
  p.qw = q.w();
  return p;
}

mp::WorldGeometryBox makeBox(const std::string & id, const sm::Transform & pose, const Vec3 & size)
{
  const core_types::Pose p = toPose(pose);
  mp::WorldGeometryBox b;
  b.id = id;
  b.center_x = p.x;
  b.center_y = p.y;
  b.center_z = p.z;
  b.orientation_x = p.qx;
  b.orientation_y = p.qy;
  b.orientation_z = p.qz;
  b.orientation_w = p.qw;
  b.size_x = size.x();
  b.size_y = size.y();
  b.size_z = size.z();
  return b;
}

// Points on the six faces of a box (local frame), spaced at most `spacing` apart.
void sampleBoxSurface(const Vec3 & size, double spacing, std::vector<Vec3> & out)
{
  const Vec3 h = size / 2.0;
  auto steps = [spacing](double len) {return std::max(1, static_cast<int>(std::ceil(len / spacing)));};
  for (int axis = 0; axis < 3; ++axis) {
    const int a = (axis + 1) % 3, b = (axis + 2) % 3;
    const int na = steps(size[a]), nb = steps(size[b]);
    for (const double side : {-1.0, 1.0}) {
      for (int i = 0; i <= na; ++i) {
        for (int j = 0; j <= nb; ++j) {
          Vec3 p;
          p[axis] = side * h[axis];
          p[a] = -h[a] + size[a] * i / na;
          p[b] = -h[b] + size[b] * j / nb;
          out.push_back(p);
        }
      }
    }
  }
}

void sampleTriangle(const Vec3 & a, const Vec3 & b, const Vec3 & c, double spacing, std::vector<Vec3> & out)
{
  const double longest = std::max({(b - a).norm(), (c - b).norm(), (a - c).norm()});
  const int n = std::max(1, static_cast<int>(std::ceil(longest / spacing)));
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n - i; ++j) {
      out.push_back(a + (b - a) * (static_cast<double>(i) / n) + (c - a) * (static_cast<double>(j) / n));
    }
  }
}

// Bounding box (local frame) used to stand in for a non-box primitive; false for other shapes.
bool primitiveExtents(const sm::Geometry & g, Vec3 & size, std::string & what)
{
  if (const auto * s = std::get_if<sm::Sphere>(&g)) {
    size = Vec3::Constant(2.0 * s->radius);
    what = "sphere -> cube";
    return true;
  }
  if (const auto * c = std::get_if<sm::Cylinder>(&g)) {
    size = Vec3(2.0 * c->radius, 2.0 * c->radius, c->height);
    what = "cylinder -> box";
    return true;
  }
  if (const auto * c = std::get_if<sm::Capsule>(&g)) {
    size = Vec3(2.0 * c->radius, 2.0 * c->radius, c->length + 2.0 * c->radius);
    what = "capsule -> box";
    return true;
  }
  return false;
}

// World-space points of any point-like geometry (cloud, mesh surface, occupied voxels).
bool worldPoints(
  const sm::SceneObject & o, const WorldGeometryOptions & opt, std::vector<Vec3> & out,
  size_t * bad_faces = nullptr)
{
  if (const auto * pc = std::get_if<sm::PointCloud>(&o.geometry)) {
    for (const auto & p : pc->points) {out.push_back(o.pose * p);}
    return true;
  }
  if (const auto * m = std::get_if<sm::Mesh>(&o.geometry)) {
    std::vector<Vec3> local;
    const auto n = static_cast<long>(m->vertices.size());
    for (const auto & f : m->faces) {
      if (f[0] < 0 || f[1] < 0 || f[2] < 0 || f[0] >= n || f[1] >= n || f[2] >= n) {
        if (bad_faces) {++*bad_faces;}
        continue;
      }
      sampleTriangle(m->vertices[f[0]], m->vertices[f[1]], m->vertices[f[2]], opt.surface_spacing, local);
    }
    for (const auto & p : local) {out.push_back(o.pose * p);}
    return true;
  }
  if (const auto * v = std::get_if<sm::VoxelGrid>(&o.geometry)) {
    for (int k = 0; k < v->nz; ++k) {
      for (int j = 0; j < v->ny; ++j) {
        for (int i = 0; i < v->nx; ++i) {
          const uint8_t s = v->occupied[v->index(i, j, k)];
          if (s == sm::kVoxelOccupied || (s == sm::kVoxelUnknown && opt.unknown_voxels_occupied)) {
            out.push_back(o.pose * (v->origin + v->voxel_size * Vec3(i, j, k)));
          }
        }
      }
    }
    return true;
  }
  return false;
}

struct Aabb
{
  Vec3 lo;
  Vec3 hi;
};

// World-space extent of every occupied voxel (a voxel is centred on origin + voxel_size * (i, j, k)).
void voxelExtents(
  const sm::SceneObject & o, const sm::VoxelGrid & v, const WorldGeometryOptions & opt,
  std::vector<Aabb> & out)
{
  const double h = v.voxel_size / 2.0;
  for (int k = 0; k < v.nz; ++k) {
    for (int j = 0; j < v.ny; ++j) {
      for (int i = 0; i < v.nx; ++i) {
        const uint8_t s = v.occupied[v.index(i, j, k)];
        if (!(s == sm::kVoxelOccupied || (s == sm::kVoxelUnknown && opt.unknown_voxels_occupied))) {
          continue;
        }
        const Vec3 c = v.origin + v.voxel_size * Vec3(i, j, k);
        Aabb box{Vec3::Constant(1e300), Vec3::Constant(-1e300)};
        for (int corner = 0; corner < 8; ++corner) {   // rotated voxel: take the box of its corners
          const Vec3 w = o.pose * (c + Vec3((corner & 1) ? h : -h, (corner & 2) ? h : -h, (corner & 4) ? h : -h));
          box.lo = box.lo.cwiseMin(w);
          box.hi = box.hi.cwiseMax(w);
        }
        out.push_back(box);
      }
    }
  }
}

// Occupied-cube boxes for a set of extents, on a world-aligned lattice. Every lattice cell the
// extents touch becomes a cube, and the cube doubles until the count fits `max_boxes`, so the
// obstacle only ever gets bigger, never smaller.
std::vector<mp::WorldGeometryBox> cubesFor(
  const std::string & id, const std::vector<Aabb> & extents, double voxel, size_t max_boxes, double * used)
{
  std::set<std::array<long, 3>> cells;
  double size = std::max(voxel, 1e-6);
  for (int attempt = 0; attempt < 16; ++attempt) {
    cells.clear();
    for (const auto & e : extents) {
      std::array<long, 3> lo, hi;
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::lround(std::floor(e.lo[a] / size));
        hi[a] = std::max(lo[a], std::lround(std::ceil(e.hi[a] / size - 1e-9)) - 1);
      }
      for (long x = lo[0]; x <= hi[0] && cells.size() <= max_boxes; ++x) {
        for (long y = lo[1]; y <= hi[1] && cells.size() <= max_boxes; ++y) {
          for (long z = lo[2]; z <= hi[2] && cells.size() <= max_boxes; ++z) {cells.insert({x, y, z});}
        }
      }
      if (cells.size() > max_boxes) {break;}
    }
    if (cells.size() <= max_boxes) {break;}
    size *= 2.0;
  }
  *used = size;
  std::vector<mp::WorldGeometryBox> boxes;
  size_t n = 0;
  for (const auto & c : cells) {
    sm::Transform t = sm::Transform::Identity();
    t.translation() = Vec3((c[0] + 0.5) * size, (c[1] + 0.5) * size, (c[2] + 0.5) * size);
    boxes.push_back(makeBox(id + "_" + std::to_string(n++), t, Vec3::Constant(size)));
  }
  return boxes;
}
}  // namespace

mp::WorldGeometryUpdate toWorldGeometry(
  const sm::SceneModel & scene, const WorldGeometryOptions & requested, ConversionReport * report)
{
  mp::WorldGeometryUpdate out;
  out.frame_id = scene.frameId();
  ConversionReport local;
  ConversionReport & rep = report ? *report : local;
  rep = ConversionReport{};

  WorldGeometryOptions options = requested;
  const WorldGeometryOptions defaults;
  if (!(options.surface_spacing > 0.0)) {
    options.surface_spacing = defaults.surface_spacing;
    rep.approximated.push_back("surface_spacing must be > 0, using " + std::to_string(defaults.surface_spacing));
  }
  if (!(options.obstacle_voxel > 0.0)) {
    options.obstacle_voxel = defaults.obstacle_voxel;
    rep.approximated.push_back("obstacle_voxel must be > 0, using " + std::to_string(defaults.obstacle_voxel));
  }

  for (const auto & o : scene.objects()) {
    std::vector<Vec3> pts;
    if (std::holds_alternative<sm::EsdfGrid>(o.geometry)) {
      rep.skipped.push_back(o.id + ": esdf grid has no box/point form (use the planner's ESDF channel)");
      continue;
    }
    Vec3 prim_size;
    std::string prim_what;
    const bool is_primitive = primitiveExtents(o.geometry, prim_size, prim_what);

    if (o.role == sm::Role::kContactSurface) {
      // contact surface -> points, whatever the shape
      if (const auto * b = std::get_if<sm::Box>(&o.geometry)) {
        std::vector<Vec3> local;
        sampleBoxSurface(b->size, options.surface_spacing, local);
        for (const auto & p : local) {pts.push_back(o.pose * p);}
      } else if (is_primitive) {
        std::vector<Vec3> local;
        sampleBoxSurface(prim_size, options.surface_spacing, local);
        for (const auto & p : local) {pts.push_back(o.pose * p);}
        rep.approximated.push_back(o.id + ": " + prim_what + " (surface points)");
      } else {
        size_t bad = 0;
        worldPoints(o, options, pts, &bad);
        if (bad > 0) {
          rep.skipped.push_back(o.id + ": " + std::to_string(bad) + " mesh faces with out-of-range vertex indices");
        }
      }
      for (const auto & p : pts) {out.points.push_back({p.x(), p.y(), p.z()});}
      rep.points += pts.size();
      continue;
    }

    // obstacle -> boxes
    if (const auto * b = std::get_if<sm::Box>(&o.geometry)) {
      out.boxes.push_back(makeBox(o.id, o.pose, b->size));
      ++rep.boxes;
    } else if (is_primitive) {
      out.boxes.push_back(makeBox(o.id, o.pose, prim_size));
      ++rep.boxes;
      rep.approximated.push_back(o.id + ": " + prim_what);
    } else {
      std::vector<Aabb> extents;
      if (const auto * v = std::get_if<sm::VoxelGrid>(&o.geometry)) {
        voxelExtents(o, *v, options, extents);   // the whole voxel, not just its centre
      } else {
        size_t bad = 0;
        if (!worldPoints(o, options, pts, &bad)) {
          rep.skipped.push_back(o.id + ": " + sm::geometryName(o.geometry) + " has no box/point form");
          continue;
        }
        if (bad > 0) {
          rep.skipped.push_back(o.id + ": " + std::to_string(bad) + " mesh faces with out-of-range vertex indices");
        }
        for (const auto & p : pts) {extents.push_back({p, p});}
      }
      const size_t before = extents.size();
      extents.erase(
        std::remove_if(extents.begin(), extents.end(), [](const Aabb & e) {
          return !(e.lo.allFinite() && e.hi.allFinite());
        }), extents.end());
      if (extents.size() != before) {
        rep.skipped.push_back(o.id + ": " + std::to_string(before - extents.size()) + " non-finite points or voxels dropped");
      }
      double used = options.obstacle_voxel;
      auto cubes = cubesFor(o.id, extents, options.obstacle_voxel, options.max_obstacle_boxes, &used);
      if (used > options.obstacle_voxel) {
        rep.coarsened.push_back(o.id + ": cubes grown to " + std::to_string(used) + " m");
      }
      rep.boxes += cubes.size();
      rep.approximated.push_back(o.id + ": " + sm::geometryName(o.geometry) + " -> occupied cubes");
      out.boxes.insert(out.boxes.end(), cubes.begin(), cubes.end());
    }
  }
  return out;
}

}  // namespace scene_adapters
