#include "scene_model/ros/conversions.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

#include <geometry_msgs/msg/point.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace scene_model
{
namespace ros_conv
{
namespace
{

const sensor_msgs::msg::PointField * findField(const sensor_msgs::msg::PointCloud2 & m, const char * name)
{
  for (const auto & f : m.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

template<typename T>
T readAt(const uint8_t * p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

}  // namespace

geometry_msgs::msg::Pose toRos(const Transform & pose)
{
  geometry_msgs::msg::Pose m;
  m.position.x = pose.translation().x();
  m.position.y = pose.translation().y();
  m.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  m.orientation.w = q.w();
  return m;
}

Transform fromRos(const geometry_msgs::msg::Pose & m)
{
  Transform T = Transform::Identity();
  T.translation() = Vec3(m.position.x, m.position.y, m.position.z);
  Eigen::Quaterniond q(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
  if (q.norm() < 1e-9) {
    q = Eigen::Quaterniond::Identity();  // an all-zero quaternion is the common "unset" value
  }
  q.normalize();
  T.linear() = q.toRotationMatrix();
  return T;
}

PointCloud fromRos(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::vector<size_t> unused;
  return fromRos(msg, unused);
}

PointCloud fromRos(const sensor_msgs::msg::PointCloud2 & msg, std::vector<size_t> & source_index)
{
  source_index.clear();
  const auto * fx = findField(msg, "x");
  const auto * fy = findField(msg, "y");
  const auto * fz = findField(msg, "z");
  if (!fx || !fy || !fz) {
    throw std::runtime_error("PointCloud2 has no x/y/z fields");
  }
  using PF = sensor_msgs::msg::PointField;
  if (fx->datatype != PF::FLOAT32 || fy->datatype != PF::FLOAT32 || fz->datatype != PF::FLOAT32) {
    throw std::runtime_error("PointCloud2 x/y/z must be float32");
  }
  const auto * frgb = findField(msg, "rgb");
  if (!frgb) {
    frgb = findField(msg, "rgba");
  }
  const auto * flabel = findField(msg, "label");
  const bool has_label = flabel && (flabel->datatype == PF::UINT32 || flabel->datatype == PF::INT32);
  const bool has_color = frgb != nullptr;

  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  if (msg.data.size() < n * msg.point_step) {
    throw std::runtime_error("PointCloud2 data is shorter than width * height * point_step");
  }
  PointCloud out;
  out.points.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * row = msg.data.data() + i * msg.point_step;
    const double x = readAt<float>(row + fx->offset);
    const double y = readAt<float>(row + fy->offset);
    const double z = readAt<float>(row + fz->offset);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    out.points.emplace_back(x, y, z);
    source_index.push_back(i);
    if (has_color) {
      const uint32_t rgb = readAt<uint32_t>(row + frgb->offset);
      out.colors.push_back({static_cast<uint8_t>((rgb >> 16) & 0xFF), static_cast<uint8_t>((rgb >> 8) & 0xFF),
        static_cast<uint8_t>(rgb & 0xFF)});
    }
    if (has_label) {
      out.labels.push_back(readAt<uint32_t>(row + flabel->offset));
    }
  }
  return out;
}

sensor_msgs::msg::PointCloud2 toRos(const PointCloud & cloud, const std::string & frame_id)
{
  using PF = sensor_msgs::msg::PointField;
  const size_t n = cloud.points.size();
  const bool color = cloud.colors.size() == n && n > 0;
  const bool label = cloud.labels.size() == n && n > 0;

  sensor_msgs::msg::PointCloud2 m;
  m.header.frame_id = frame_id;
  m.height = 1;
  m.width = static_cast<uint32_t>(n);
  m.is_bigendian = false;
  m.is_dense = true;
  uint32_t offset = 0;
  auto add = [&](const char * name, uint8_t type) {
      PF f;
      f.name = name;
      f.offset = offset;
      f.datatype = type;
      f.count = 1;
      m.fields.push_back(f);
      offset += 4;
    };
  add("x", PF::FLOAT32);
  add("y", PF::FLOAT32);
  add("z", PF::FLOAT32);
  if (color) {add("rgb", PF::FLOAT32);}
  if (label) {add("label", PF::UINT32);}
  m.point_step = offset;
  m.row_step = offset * m.width;
  m.data.resize(static_cast<size_t>(m.row_step));
  for (size_t i = 0; i < n; ++i) {
    uint8_t * row = m.data.data() + i * offset;
    const float xyz[3] = {static_cast<float>(cloud.points[i].x()), static_cast<float>(cloud.points[i].y()),
      static_cast<float>(cloud.points[i].z())};
    std::memcpy(row, xyz, sizeof(xyz));
    uint32_t at = 12;
    if (color) {
      const uint32_t rgb = (static_cast<uint32_t>(cloud.colors[i][0]) << 16) |
        (static_cast<uint32_t>(cloud.colors[i][1]) << 8) | cloud.colors[i][2];
      std::memcpy(row + at, &rgb, 4);
      at += 4;
    }
    if (label) {
      std::memcpy(row + at, &cloud.labels[i], 4);
    }
  }
  return m;
}

sensor_msgs::msg::PointCloud2 keepPoints(
  const sensor_msgs::msg::PointCloud2 & msg, const std::vector<bool> & removed)
{
  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  if (removed.size() != n) {
    throw std::runtime_error("keepPoints: the mask must have one entry per point");
  }
  if (msg.data.size() < n * msg.point_step) {
    throw std::runtime_error("keepPoints: data is shorter than width * height * point_step");
  }
  sensor_msgs::msg::PointCloud2 out;
  out.header = msg.header;
  out.fields = msg.fields;
  out.is_bigendian = msg.is_bigendian;
  out.point_step = msg.point_step;
  out.height = 1;
  size_t kept = 0;
  for (size_t i = 0; i < n; ++i) {
    if (!removed[i]) {++kept;}
  }
  out.data.reserve(kept * msg.point_step);
  for (size_t i = 0; i < n; ++i) {
    if (removed[i]) {
      continue;
    }
    const auto * row = msg.data.data() + i * msg.point_step;
    out.data.insert(out.data.end(), row, row + msg.point_step);
  }
  out.width = static_cast<uint32_t>(kept);
  out.row_step = out.point_step * out.width;
  out.is_dense = msg.is_dense;
  return out;
}

std::vector<moveit_msgs::msg::CollisionObject> toRos(const SceneModel & scene, std::vector<std::string> * skipped)
{
  using SP = shape_msgs::msg::SolidPrimitive;
  std::vector<moveit_msgs::msg::CollisionObject> out;
  for (const auto & o : scene.objects()) {
    moveit_msgs::msg::CollisionObject co;
    co.id = o.id;
    co.header.frame_id = o.frame_id.empty() ? scene.frameId() : o.frame_id;
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    const auto pose = toRos(o.pose);
    bool ok = true;
    if (const auto * b = std::get_if<Box>(&o.geometry)) {
      SP p;
      p.type = SP::BOX;
      p.dimensions = {b->size.x(), b->size.y(), b->size.z()};
      co.primitives.push_back(p);
      co.primitive_poses.push_back(pose);
    } else if (const auto * s = std::get_if<Sphere>(&o.geometry)) {
      SP p;
      p.type = SP::SPHERE;
      p.dimensions = {s->radius};
      co.primitives.push_back(p);
      co.primitive_poses.push_back(pose);
    } else if (const auto * c = std::get_if<Cylinder>(&o.geometry)) {
      SP p;
      p.type = SP::CYLINDER;
      p.dimensions = {c->height, c->radius};  // MoveIt order: height, radius
      co.primitives.push_back(p);
      co.primitive_poses.push_back(pose);
    } else if (const auto * cap = std::get_if<Capsule>(&o.geometry)) {
      SP p;  // MoveIt has no capsule: the cylinder over the full extent (conservative)
      p.type = SP::CYLINDER;
      p.dimensions = {cap->length + 2.0 * cap->radius, cap->radius};
      co.primitives.push_back(p);
      co.primitive_poses.push_back(pose);
    } else if (const auto * m = std::get_if<Mesh>(&o.geometry)) {
      shape_msgs::msg::Mesh mesh;
      for (const auto & v : m->vertices) {
        geometry_msgs::msg::Point pt;
        pt.x = v.x();
        pt.y = v.y();
        pt.z = v.z();
        mesh.vertices.push_back(pt);
      }
      for (const auto & f : m->faces) {
        shape_msgs::msg::MeshTriangle t;
        t.vertex_indices = {static_cast<uint32_t>(f[0]), static_cast<uint32_t>(f[1]), static_cast<uint32_t>(f[2])};
        mesh.triangles.push_back(t);
      }
      co.meshes.push_back(mesh);
      co.mesh_poses.push_back(pose);
    } else {
      ok = false;
    }
    if (ok) {
      out.push_back(std::move(co));
    } else if (skipped) {
      skipped->push_back(o.id + " (" + geometryName(o.geometry) + ")");
    }
  }
  return out;
}

std::vector<SceneObject> fromRos(const moveit_msgs::msg::CollisionObject & co)
{
  using SP = shape_msgs::msg::SolidPrimitive;
  std::vector<SceneObject> out;
  const Transform base = fromRos(co.pose);
  auto add = [&](Geometry g, const Transform & local) {
      SceneObject o;
      o.geometry = std::move(g);
      o.frame_id = co.header.frame_id;
      o.pose = base * local;
      out.push_back(std::move(o));
    };
  for (size_t i = 0; i < co.primitives.size(); ++i) {
    const auto & p = co.primitives[i];
    const Transform local = i < co.primitive_poses.size() ? fromRos(co.primitive_poses[i]) : Transform::Identity();
    if (p.type == SP::BOX && p.dimensions.size() >= 3) {
      add(Box{Vec3(p.dimensions[0], p.dimensions[1], p.dimensions[2])}, local);
    } else if (p.type == SP::SPHERE && p.dimensions.size() >= 1) {
      add(Sphere{p.dimensions[0]}, local);
    } else if (p.type == SP::CYLINDER && p.dimensions.size() >= 2) {
      add(Cylinder{p.dimensions[1], p.dimensions[0]}, local);  // message order: height, radius
    }
  }
  for (size_t i = 0; i < co.meshes.size(); ++i) {
    Mesh m;
    for (const auto & v : co.meshes[i].vertices) {m.vertices.emplace_back(v.x, v.y, v.z);}
    for (const auto & t : co.meshes[i].triangles) {
      m.faces.push_back({static_cast<int>(t.vertex_indices[0]), static_cast<int>(t.vertex_indices[1]),
        static_cast<int>(t.vertex_indices[2])});
    }
    add(std::move(m), i < co.mesh_poses.size() ? fromRos(co.mesh_poses[i]) : Transform::Identity());
  }
  if (out.size() == 1) {
    out[0].id = co.id;
  } else {
    for (size_t k = 0; k < out.size(); ++k) {out[k].id = co.id + "#" + std::to_string(k);}
  }
  return out;
}

SceneModel fromRos(const moveit_msgs::msg::PlanningScene & scene)
{
  SceneModel model(scene.world.collision_objects.empty() ? "" : scene.world.collision_objects[0].header.frame_id);
  for (const auto & co : scene.world.collision_objects) {
    for (auto & o : fromRos(co)) {
      model.upsert(std::move(o));
    }
  }
  return model;
}

}  // namespace ros_conv
}  // namespace scene_model
