#include "robot_self_filter/core/robot_model.hpp"

#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

#include <tinyxml2.h>

namespace robot_self_filter
{
namespace
{

[[noreturn]] void fail(const std::string & why) {throw std::runtime_error("URDF: " + why);}

std::vector<double> numbers(const char * text, size_t expected, const char * what)
{
  std::vector<double> out;
  if (text == nullptr) {
    return out;
  }
  std::istringstream ss(text);
  double v;
  while (ss >> v) {out.push_back(v);}
  if (out.size() != expected) {
    fail(std::string(what) + " must have " + std::to_string(expected) + " numbers, got '" + text + "'");
  }
  return out;
}

Pose originOf(const tinyxml2::XMLElement * parent)
{
  Pose T = Pose::Identity();
  const auto * o = parent ? parent->FirstChildElement("origin") : nullptr;
  if (!o) {
    return T;
  }
  if (o->Attribute("xyz")) {
    const auto v = numbers(o->Attribute("xyz"), 3, "origin xyz");
    T.translation() = Vec3(v[0], v[1], v[2]);
  }
  if (o->Attribute("rpy")) {
    const auto v = numbers(o->Attribute("rpy"), 3, "origin rpy");
    T.linear() = (Eigen::AngleAxisd(v[2], Vec3::UnitZ()) * Eigen::AngleAxisd(v[1], Vec3::UnitY()) *
      Eigen::AngleAxisd(v[0], Vec3::UnitX())).toRotationMatrix();
  }
  return T;
}

}  // namespace

UrdfRobot UrdfRobot::fromUrdf(const std::string & xml, const MeshBoundsFn & mesh_bounds)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
    fail(std::string("cannot parse: ") + doc.ErrorStr());
  }
  const auto * robot = doc.FirstChildElement("robot");
  if (!robot) {
    fail("no <robot> element");
  }

  UrdfRobot out;
  std::set<std::string> links;
  for (const auto * l = robot->FirstChildElement("link"); l; l = l->NextSiblingElement("link")) {
    const char * name = l->Attribute("name");
    if (!name) {
      fail("a <link> without a name");
    }
    links.insert(name);
    out.link_index_[name] = static_cast<int>(out.link_index_.size());
    LinkShapes ls;
    ls.link = name;
    for (const auto * c = l->FirstChildElement("collision"); c; c = c->NextSiblingElement("collision")) {
      const auto * g = c->FirstChildElement("geometry");
      if (!g) {
        continue;
      }
      Shape s;
      s.local = originOf(c);
      if (const auto * b = g->FirstChildElement("box")) {
        s.kind = Shape::Kind::kBox;
        const auto v = numbers(b->Attribute("size"), 3, "box size");
        s.size = Vec3(v[0], v[1], v[2]);
      } else if (const auto * sp = g->FirstChildElement("sphere")) {
        s.kind = Shape::Kind::kSphere;
        s.radius = sp->DoubleAttribute("radius");
      } else if (const auto * cy = g->FirstChildElement("cylinder")) {
        s.kind = Shape::Kind::kCylinder;
        s.radius = cy->DoubleAttribute("radius");
        s.height = cy->DoubleAttribute("length");
      } else if (const auto * m = g->FirstChildElement("mesh")) {
        const std::string uri = m->Attribute("filename") ? m->Attribute("filename") : "";
        const auto bounds = mesh_bounds ? mesh_bounds(uri) : std::nullopt;
        if (!bounds) {
          out.skipped_meshes_.push_back(std::string(name) + ": " + uri);
          continue;
        }
        Vec3 scale = Vec3::Ones();
        if (m->Attribute("scale")) {
          const auto v = numbers(m->Attribute("scale"), 3, "mesh scale");
          scale = Vec3(v[0], v[1], v[2]);
        }
        const Vec3 lo = bounds->min.cwiseProduct(scale), hi = bounds->max.cwiseProduct(scale);
        s.kind = Shape::Kind::kBox;
        s.size = (hi - lo).cwiseAbs();
        s.local.translation() += s.local.linear() * (0.5 * (lo + hi));
      } else {
        continue;
      }
      ls.shapes.push_back(s);
    }
    out.shapes_.push_back(std::move(ls));
  }

  std::set<std::string> children;
  std::vector<Joint> joints;
  for (const auto * j = robot->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
    Joint jt;
    const char * name = j->Attribute("name");
    const char * type = j->Attribute("type");
    if (!name || !type) {
      fail("a <joint> without a name or type");
    }
    jt.name = name;
    const std::string t = type;
    if (t == "fixed") {jt.type = Joint::Type::kFixed;}
    else if (t == "revolute") {jt.type = Joint::Type::kRevolute;}
    else if (t == "continuous") {jt.type = Joint::Type::kContinuous;}
    else if (t == "prismatic") {jt.type = Joint::Type::kPrismatic;}
    else {fail("joint '" + jt.name + "' has unsupported type '" + t + "'");}
    const auto * p = j->FirstChildElement("parent");
    const auto * c = j->FirstChildElement("child");
    if (!p || !c || !p->Attribute("link") || !c->Attribute("link")) {
      fail("joint '" + jt.name + "' needs <parent link> and <child link>");
    }
    jt.parent = p->Attribute("link");
    jt.child = c->Attribute("link");
    if (!links.count(jt.parent) || !links.count(jt.child)) {
      fail("joint '" + jt.name + "' refers to a missing link");
    }
    jt.origin = originOf(j);
    if (const auto * a = j->FirstChildElement("axis")) {
      const auto v = numbers(a->Attribute("xyz"), 3, "axis xyz");
      jt.axis = Vec3(v[0], v[1], v[2]);
      if (jt.axis.norm() < 1e-9) {
        fail("joint '" + jt.name + "' has a zero axis");
      }
      jt.axis.normalize();
    }
    if (const auto * m = j->FirstChildElement("mimic")) {
      jt.mimic_joint = m->Attribute("joint") ? m->Attribute("joint") : "";
      jt.mimic_multiplier = m->DoubleAttribute("multiplier", 1.0);
      jt.mimic_offset = m->DoubleAttribute("offset", 0.0);
    }
    children.insert(jt.child);
    joints.push_back(jt);
  }

  for (const auto & l : links) {
    if (!children.count(l)) {
      if (!out.root_.empty()) {
        fail("more than one root link ('" + out.root_ + "' and '" + l + "')");
      }
      out.root_ = l;
    }
  }
  if (out.root_.empty()) {
    fail("no root link");
  }

  // Order joints so a parent's pose is known before its children.
  std::set<std::string> placed = {out.root_};
  std::vector<bool> used(joints.size(), false);
  for (size_t done = 0; done < joints.size();) {
    bool progressed = false;
    for (size_t i = 0; i < joints.size(); ++i) {
      if (!used[i] && placed.count(joints[i].parent)) {
        used[i] = true;
        placed.insert(joints[i].child);
        out.joints_.push_back(joints[i]);
        ++done;
        progressed = true;
      }
    }
    if (!progressed) {
      fail("the joints do not form a tree from '" + out.root_ + "'");
    }
  }
  return out;
}

const std::vector<std::string> & UrdfRobot::jointNames() const
{
  if (joint_names_.empty()) {
    for (const auto & j : joints_) {
      if (j.type != Joint::Type::kFixed) {joint_names_.push_back(j.name);}
    }
  }
  return joint_names_;
}

LinkPoses UrdfRobot::linkPoses(const JointPositions & q) const
{
  LinkPoses poses;
  poses[root_] = Pose::Identity();
  auto value = [&](const Joint & j) {
      if (!j.mimic_joint.empty()) {
        const auto it = q.find(j.mimic_joint);
        const double master = it == q.end() ? 0.0 : it->second;
        return j.mimic_multiplier * master + j.mimic_offset;
      }
      const auto it = q.find(j.name);
      return it == q.end() ? 0.0 : it->second;
    };
  for (const auto & j : joints_) {
    Pose motion = Pose::Identity();
    if (j.type == Joint::Type::kRevolute || j.type == Joint::Type::kContinuous) {
      motion.linear() = Eigen::AngleAxisd(value(j), j.axis).toRotationMatrix();
    } else if (j.type == Joint::Type::kPrismatic) {
      motion.translation() = value(j) * j.axis;
    }
    poses[j.child] = poses.at(j.parent) * j.origin * motion;
  }
  return poses;
}

}  // namespace robot_self_filter
