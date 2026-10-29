#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "scene_model/core/io.hpp"

namespace scene_model
{
namespace
{

[[noreturn]] void fail(const std::string & why)
{
  throw std::runtime_error("scene description: " + why);
}

std::string dirOf(const std::string & path)
{
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? "" : path.substr(0, slash);
}

std::string resolve(const std::string & base_dir, const std::string & file)
{
  if (file.empty() || file[0] == '/' || base_dir.empty()) {
    return file;
  }
  return base_dir + "/" + file;
}

Vec3 vec3(const YAML::Node & n, const char * what)
{
  if (!n || !n.IsSequence() || n.size() != 3) {
    fail(std::string(what) + " must be a list of 3 numbers");
  }
  return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

Transform poseFromNative(const YAML::Node & pose)
{
  Transform T = Transform::Identity();
  if (!pose) {
    return T;
  }
  if (pose["position"]) {
    T.translation() = vec3(pose["position"], "pose.position");
  }
  if (pose["orientation_xyzw"]) {
    const auto q = pose["orientation_xyzw"];
    if (!q.IsSequence() || q.size() != 4) {
      fail("pose.orientation_xyzw must be a list of 4 numbers");
    }
    Eigen::Quaterniond quat(q[3].as<double>(), q[0].as<double>(), q[1].as<double>(), q[2].as<double>());
    if (quat.norm() < 1e-9) {
      fail("pose.orientation_xyzw is a zero quaternion");
    }
    quat.normalize();
    T.linear() = quat.toRotationMatrix();
  }
  return T;
}

// cuRobo poses are [x, y, z, qw, qx, qy, qz].
Transform poseFromCurobo(const YAML::Node & pose)
{
  Transform T = Transform::Identity();
  if (!pose) {
    return T;
  }
  if (!pose.IsSequence() || pose.size() != 7) {
    fail("pose must be [x, y, z, qw, qx, qy, qz]");
  }
  T.translation() = Vec3(pose[0].as<double>(), pose[1].as<double>(), pose[2].as<double>());
  Eigen::Quaterniond quat(
    pose[3].as<double>(), pose[4].as<double>(), pose[5].as<double>(), pose[6].as<double>());
  if (quat.norm() < 1e-9) {
    fail("pose has a zero quaternion");
  }
  quat.normalize();
  T.linear() = quat.toRotationMatrix();
  return T;
}

Role roleFrom(const YAML::Node & n)
{
  if (!n) {
    return Role::kObstacle;
  }
  const std::string s = n.as<std::string>();
  if (s == "obstacle") {return Role::kObstacle;}
  if (s == "contact_surface") {return Role::kContactSurface;}
  fail("role must be 'obstacle' or 'contact_surface', got '" + s + "'");
}

void loadNative(const YAML::Node & root, const std::string & base_dir, SceneModel & scene)
{
  if (root["frame_id"]) {
    scene.setFrameId(root["frame_id"].as<std::string>());
  }
  for (const auto & n : root["objects"]) {
    SceneObject o;
    if (!n["id"]) {fail("object without an id");}
    o.id = n["id"].as<std::string>();
    if (!n["type"]) {fail("object '" + o.id + "' has no type");}
    const std::string type = n["type"].as<std::string>();
    o.pose = poseFromNative(n["pose"]);
    o.role = roleFrom(n["role"]);
    o.is_static = n["static"] ? n["static"].as<bool>() : true;
    if (n["frame_id"]) {o.frame_id = n["frame_id"].as<std::string>();}

    if (type == "box") {
      o.geometry = Box{vec3(n["size"], "size")};
    } else if (type == "sphere") {
      o.geometry = Sphere{n["radius"].as<double>()};
    } else if (type == "cylinder") {
      o.geometry = Cylinder{n["radius"].as<double>(), n["height"].as<double>()};
    } else if (type == "capsule") {
      o.geometry = Capsule{n["radius"].as<double>(), n["length"].as<double>()};
    } else if (type == "mesh") {
      if (!n["file"]) {fail("mesh '" + o.id + "' needs a file");}
      Mesh m = loadMesh(resolve(base_dir, n["file"].as<std::string>()));
      if (n["scale"]) {
        const double s = n["scale"].as<double>();
        for (auto & v : m.vertices) {v *= s;}
      }
      o.geometry = std::move(m);
    } else if (type == "pointcloud") {
      if (!n["file"]) {fail("pointcloud '" + o.id + "' needs a file");}
      o.geometry = loadPointCloud(resolve(base_dir, n["file"].as<std::string>()));
    } else {
      fail("object '" + o.id + "' has unknown type '" + type + "'");
    }
    if (!scene.add(std::move(o))) {
      fail("duplicate or empty object id '" + n["id"].as<std::string>() + "'");
    }
  }
}

void loadCurobo(const YAML::Node & root, const std::string & base_dir, SceneModel & scene)
{
  auto addAll = [&](const char * key, auto make) {
      const YAML::Node group = root[key];
      if (!group) {return;}
      for (const auto & kv : group) {
        SceneObject o;
        o.id = kv.first.as<std::string>();
        o.pose = poseFromCurobo(kv.second["pose"]);
        make(kv.second, o);
        if (!scene.add(std::move(o))) {
          fail("duplicate object id '" + kv.first.as<std::string>() + "'");
        }
      }
    };
  addAll("cuboid", [](const YAML::Node & n, SceneObject & o) {
      o.geometry = Box{vec3(n["dims"], "dims")};
    });
  addAll("sphere", [](const YAML::Node & n, SceneObject & o) {
      o.geometry = Sphere{n["radius"].as<double>()};
      if (n["position"]) {o.pose.translation() = vec3(n["position"], "position");}
    });
  addAll("cylinder", [](const YAML::Node & n, SceneObject & o) {
      o.geometry = Cylinder{n["radius"].as<double>(), n["height"].as<double>()};
    });
  addAll("capsule", [](const YAML::Node & n, SceneObject & o) {
      // cuRobo gives base/tip points; our capsule is centred with its axis on local z.
      const Vec3 base = vec3(n["base"], "base"), tip = vec3(n["tip"], "tip");
      const Vec3 axis = tip - base;
      const double length = axis.norm();
      o.geometry = Capsule{n["radius"].as<double>(), length};
      Transform T = o.pose;
      const Vec3 centre = 0.5 * (base + tip);
      Transform local = Transform::Identity();
      local.translation() = centre;
      if (length > 1e-12) {
        local.linear() =
          Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), axis.normalized()).toRotationMatrix();
      }
      o.pose = T * local;
    });
  addAll("mesh", [&](const YAML::Node & n, SceneObject & o) {
      if (!n["file_path"]) {fail("mesh '" + o.id + "' needs file_path");}
      Mesh m = loadMesh(resolve(base_dir, n["file_path"].as<std::string>()));
      if (n["scale"]) {
        if (n["scale"].IsSequence()) {
          const Vec3 s = vec3(n["scale"], "scale");
          for (auto & v : m.vertices) {v = v.cwiseProduct(s);}
        } else {
          const double s = n["scale"].as<double>();
          for (auto & v : m.vertices) {v *= s;}
        }
      }
      o.geometry = std::move(m);
    });
}

}  // namespace

SceneModel loadSceneYaml(const std::string & yaml_text, const std::string & base_dir)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    fail(std::string("cannot parse: ") + e.what());
  }
  if (!root.IsMap()) {
    fail("top level must be a mapping");
  }
  SceneModel scene;
  try {
    if (root["objects"]) {
      loadNative(root, base_dir, scene);
    } else {
      loadCurobo(root, base_dir, scene);
    }
  } catch (const YAML::Exception & e) {
    fail(std::string("invalid field: ") + e.what());
  }
  return scene;
}

SceneModel loadSceneFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error(path + ": cannot open file");
  }
  std::stringstream ss;
  ss << in.rdbuf();
  try {
    return loadSceneYaml(ss.str(), dirOf(path));
  } catch (const std::exception & e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

}  // namespace scene_model
