#include "scene_model/core/scene_model.hpp"

#include <algorithm>
#include <cmath>

namespace scene_model
{

bool SceneModel::add(SceneObject object)
{
  if (object.id.empty() || find(object.id) != nullptr) {
    return false;
  }
  objects_.push_back(std::move(object));
  return true;
}

void SceneModel::upsert(SceneObject object)
{
  for (auto & existing : objects_) {
    if (existing.id == object.id) {
      existing = std::move(object);
      return;
    }
  }
  objects_.push_back(std::move(object));
}

bool SceneModel::remove(const std::string & id)
{
  const auto it = std::find_if(
    objects_.begin(), objects_.end(), [&](const SceneObject & o) {return o.id == id;});
  if (it == objects_.end()) {
    return false;
  }
  objects_.erase(it);
  return true;
}

const SceneObject * SceneModel::find(const std::string & id) const
{
  for (const auto & o : objects_) {
    if (o.id == id) {
      return &o;
    }
  }
  return nullptr;
}

std::vector<const SceneObject *> SceneModel::withRole(Role role) const
{
  std::vector<const SceneObject *> out;
  for (const auto & o : objects_) {
    if (o.role == role) {
      out.push_back(&o);
    }
  }
  return out;
}

void SceneModel::removeDynamic()
{
  objects_.erase(
    std::remove_if(objects_.begin(), objects_.end(), [](const SceneObject & o) {return !o.is_static;}),
    objects_.end());
}

Aabb SceneModel::bounds() const
{
  Aabb box;
  for (const auto & o : objects_) {
    const Aabb b = objectBounds(o);
    if (!b.empty()) {
      box.extend(b.min);
      box.extend(b.max);
    }
  }
  return box;
}

void SceneModel::merge(const SceneModel & other)
{
  for (SceneObject o : other.objects_) {
    if (find(o.id) != nullptr) {
      const std::string base = o.id;
      int n = 1;
      do {
        o.id = base + "_" + std::to_string(n++);
      } while (find(o.id) != nullptr);
    }
    objects_.push_back(std::move(o));
  }
}

namespace
{

void extendWithBox(Aabb & out, const Transform & pose, const Vec3 & half)
{
  for (int i = 0; i < 8; ++i) {
    const Vec3 corner(
      (i & 1) ? half.x() : -half.x(), (i & 2) ? half.y() : -half.y(),
      (i & 4) ? half.z() : -half.z());
    out.extend(pose * corner);
  }
}

}  // namespace

Aabb objectBounds(const SceneObject & object)
{
  Aabb out;
  const Transform & T = object.pose;
  struct Visitor
  {
    Aabb & out;
    const Transform & T;
    void operator()(const Box & b) const {extendWithBox(out, T, 0.5 * b.size);}
    void operator()(const Sphere & s) const
    {
      extendWithBox(out, T, Vec3::Constant(s.radius));
    }
    void operator()(const Cylinder & c) const
    {
      extendWithBox(out, T, Vec3(c.radius, c.radius, 0.5 * c.height));
    }
    void operator()(const Capsule & c) const
    {
      extendWithBox(out, T, Vec3(c.radius, c.radius, 0.5 * c.length + c.radius));
    }
    void operator()(const Mesh & m) const
    {
      for (const auto & v : m.vertices) {
        out.extend(T * v);
      }
    }
    void operator()(const PointCloud & p) const
    {
      for (const auto & v : p.points) {
        out.extend(T * v);
      }
    }
    void operator()(const VoxelGrid & g) const
    {
      if (g.nx <= 0 || g.ny <= 0 || g.nz <= 0) {
        return;
      }
      const Vec3 lo = g.origin - Vec3::Constant(0.5 * g.voxel_size);
      const Vec3 hi = g.origin + g.voxel_size * Vec3(g.nx - 0.5, g.ny - 0.5, g.nz - 0.5);
      extendWithBox(out, T * Eigen::Translation3d(0.5 * (lo + hi)), 0.5 * (hi - lo));
    }
    void operator()(const EsdfGrid & g) const
    {
      if (g.nx <= 0 || g.ny <= 0 || g.nz <= 0) {
        return;
      }
      const Vec3 lo = g.origin - Vec3::Constant(0.5 * g.voxel_size);
      const Vec3 hi = g.origin + g.voxel_size * Vec3(g.nx - 0.5, g.ny - 0.5, g.nz - 0.5);
      extendWithBox(out, T * Eigen::Translation3d(0.5 * (lo + hi)), 0.5 * (hi - lo));
    }
  };
  std::visit(Visitor{out, T}, object.geometry);
  return out;
}

const char * geometryName(const Geometry & geometry)
{
  struct Name
  {
    const char * operator()(const Box &) const {return "box";}
    const char * operator()(const Sphere &) const {return "sphere";}
    const char * operator()(const Cylinder &) const {return "cylinder";}
    const char * operator()(const Capsule &) const {return "capsule";}
    const char * operator()(const Mesh &) const {return "mesh";}
    const char * operator()(const PointCloud &) const {return "pointcloud";}
    const char * operator()(const VoxelGrid &) const {return "voxel";}
    const char * operator()(const EsdfGrid &) const {return "esdf";}
  };
  return std::visit(Name{}, geometry);
}

}  // namespace scene_model
