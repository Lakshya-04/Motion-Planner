#include "scene_model/core/export.hpp"

#include <variant>

#include <yaml-cpp/yaml.h>

namespace scene_model
{
namespace
{

YAML::Node curoboPose(const Transform & T)
{
  const Eigen::Quaterniond q(T.linear());
  YAML::Node pose;
  pose.SetStyle(YAML::EmitterStyle::Flow);
  for (double v : {T.translation().x(), T.translation().y(), T.translation().z(), q.w(), q.x(), q.y(), q.z()}) {
    pose.push_back(v);
  }
  return pose;
}

YAML::Node flow3(const Vec3 & v)
{
  YAML::Node n;
  n.SetStyle(YAML::EmitterStyle::Flow);
  n.push_back(v.x());
  n.push_back(v.y());
  n.push_back(v.z());
  return n;
}

}  // namespace

std::string toCuroboWorldYaml(const SceneModel & scene, std::vector<std::string> * skipped)
{
  YAML::Node root(YAML::NodeType::Map);
  auto & r = root;
  for (const auto & o : scene.objects()) {
    YAML::Node n;
    n["pose"] = curoboPose(o.pose);
    if (const auto * b = std::get_if<Box>(&o.geometry)) {
      n["dims"] = flow3(b->size);
      r["cuboid"][o.id] = n;
    } else if (const auto * s = std::get_if<Sphere>(&o.geometry)) {
      n["radius"] = s->radius;
      r["sphere"][o.id] = n;
    } else if (const auto * c = std::get_if<Cylinder>(&o.geometry)) {
      n["radius"] = c->radius;
      n["height"] = c->height;
      r["cylinder"][o.id] = n;
    } else if (const auto * cap = std::get_if<Capsule>(&o.geometry)) {
      // Pose carries the placement; base and tip are along local z in that frame.
      n["radius"] = cap->radius;
      n["base"] = flow3(Vec3(0, 0, -0.5 * cap->length));
      n["tip"] = flow3(Vec3(0, 0, 0.5 * cap->length));
      r["capsule"][o.id] = n;
    } else if (const auto * m = std::get_if<Mesh>(&o.geometry)) {
      if (!m->source_path.empty()) {
        n["file_path"] = m->source_path;
      } else {
        YAML::Node verts(YAML::NodeType::Sequence), faces(YAML::NodeType::Sequence);
        for (const auto & v : m->vertices) {verts.push_back(flow3(v));}
        for (const auto & f : m->faces) {
          YAML::Node t;
          t.SetStyle(YAML::EmitterStyle::Flow);
          for (int i : f) {t.push_back(i);}
          faces.push_back(t);
        }
        n["vertices"] = verts;
        n["faces"] = faces;
      }
      r["mesh"][o.id] = n;
    } else if (skipped) {
      skipped->push_back(o.id + " (" + geometryName(o.geometry) + ")");
    }
  }
  YAML::Emitter out;
  out << root;
  return std::string(out.c_str()) + "\n";
}

}  // namespace scene_model
