#include "gravity_core/urdf_loader.hpp"

#include <tinyxml2.h>

#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>

namespace gravity_core {
namespace {

Eigen::Vector3d parseVec(const char* text, const Eigen::Vector3d& fallback) {
  if (!text) return fallback;
  std::istringstream in(text);
  Eigen::Vector3d v;
  in >> v[0] >> v[1] >> v[2];
  if (in.fail()) throw std::runtime_error(std::string("gravity_core: bad vector '") + text + "'");
  return v;
}

Eigen::Isometry3d parseOrigin(const tinyxml2::XMLElement* parent) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  const auto* o = parent ? parent->FirstChildElement("origin") : nullptr;
  if (!o) return t;
  const Eigen::Vector3d rpy = parseVec(o->Attribute("rpy"), Eigen::Vector3d::Zero());
  t.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  t.translation() = parseVec(o->Attribute("xyz"), Eigen::Vector3d::Zero());
  return t;
}

struct RawJoint {
  Joint joint;
  std::string parent_link;
  std::string child_link;
  std::string mimic_name;
};

}  // namespace

GravityModel loadUrdf(const std::string& urdf_xml, const std::vector<std::string>& actuated) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_xml.c_str(), urdf_xml.size()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("gravity_core: URDF is not valid XML");
  }
  const auto* robot = doc.FirstChildElement("robot");
  if (!robot) throw std::runtime_error("gravity_core: no <robot> element");

  std::map<std::string, Link> raw_links;
  for (const auto* l = robot->FirstChildElement("link"); l; l = l->NextSiblingElement("link")) {
    Link link;
    link.name = l->Attribute("name");
    if (const auto* in = l->FirstChildElement("inertial")) {
      if (const auto* m = in->FirstChildElement("mass")) link.mass = m->DoubleAttribute("value");
      link.com = parseOrigin(in).translation();
    }
    raw_links[link.name] = link;
  }

  std::vector<RawJoint> raw_joints;
  std::map<std::string, bool> is_child;
  for (const auto* j = robot->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
    RawJoint r;
    r.joint.name = j->Attribute("name");
    const std::string type = j->Attribute("type");
    if (type == "revolute" || type == "continuous") {
      r.joint.type = JointType::Revolute;
    } else if (type == "prismatic") {
      r.joint.type = JointType::Prismatic;
    } else if (type == "fixed") {
      r.joint.type = JointType::Fixed;
    } else {
      throw std::runtime_error("gravity_core: unsupported joint type " + type + " on " + r.joint.name);
    }
    r.parent_link = j->FirstChildElement("parent")->Attribute("link");
    r.child_link = j->FirstChildElement("child")->Attribute("link");
    r.joint.origin = parseOrigin(j);
    if (const auto* a = j->FirstChildElement("axis")) {
      r.joint.axis = parseVec(a->Attribute("xyz"), Eigen::Vector3d::UnitX());
    }
    if (const auto* m = j->FirstChildElement("mimic")) {
      r.mimic_name = m->Attribute("joint");
      r.joint.mimic_multiplier = m->DoubleAttribute("multiplier", 1.0);
      r.joint.mimic_offset = m->DoubleAttribute("offset", 0.0);
    }
    if (!raw_links.count(r.parent_link) || !raw_links.count(r.child_link)) {
      throw std::runtime_error("gravity_core: joint " + r.joint.name + " names an unknown link");
    }
    is_child[r.child_link] = true;
    raw_joints.push_back(r);
  }

  std::string root;
  for (const auto& kv : raw_links) {
    if (!is_child.count(kv.first)) root = kv.first;
  }
  if (root.empty()) throw std::runtime_error("gravity_core: no root link");

  // Breadth first from the root so every parent comes before its children.
  std::vector<Link> links;
  std::vector<Joint> joints;
  std::map<std::string, int> link_index;
  std::map<std::string, int> joint_index;
  std::vector<std::string> mimic_names;
  links.push_back(raw_links[root]);
  link_index[root] = 0;
  std::queue<std::string> todo;
  todo.push(root);
  while (!todo.empty()) {
    const std::string cur = todo.front();
    todo.pop();
    for (const auto& r : raw_joints) {
      if (r.parent_link != cur) continue;
      Joint j = r.joint;
      j.parent = link_index[cur];
      j.child = static_cast<int>(links.size());
      link_index[r.child_link] = j.child;
      links.push_back(raw_links[r.child_link]);
      joint_index[j.name] = static_cast<int>(joints.size());
      joints.push_back(j);
      mimic_names.push_back(r.mimic_name);
      todo.push(r.child_link);
    }
  }
  for (size_t k = 0; k < joints.size(); ++k) {
    if (mimic_names[k].empty()) continue;
    const auto it = joint_index.find(mimic_names[k]);
    if (it == joint_index.end()) throw std::runtime_error("gravity_core: mimic of unknown joint " + mimic_names[k]);
    joints[k].mimic = it->second;
  }
  return GravityModel(std::move(links), std::move(joints), actuated);
}

}  // namespace gravity_core
