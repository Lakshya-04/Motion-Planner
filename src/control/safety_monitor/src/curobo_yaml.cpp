/**
 * @file curobo_yaml.cpp
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include "safety_monitor/curobo_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <set>
#include <stdexcept>
#include <vector>

namespace safety_monitor
{

namespace
{
YAML::Node findKey(const YAML::Node & doc, const char * key)
{
  for (const char * path : {"robot_cfg", ""}) {
    YAML::Node n = *path ? doc[path] : doc;
    if (!n.IsMap()) {continue;}
    if (n["kinematics"].IsMap() && n["kinematics"][key]) {return n["kinematics"][key];}
    if (n[key]) {return n[key];}
  }
  return YAML::Node();
}
}  // namespace

SphereModel loadSphereModelFromCuroboYaml(
  const std::string & yaml_text, const std::map<std::string, std::string> & parents)
{
  const YAML::Node doc = YAML::Load(yaml_text);
  const YAML::Node table = findKey(doc, "collision_spheres");
  if (!table.IsMap()) {throw std::runtime_error("cuRobo yml: no inline collision_spheres table");}

  std::vector<std::string> order;
  if (const YAML::Node names = findKey(doc, "collision_link_names"); names.IsSequence()) {
    for (const auto & n : names) {
      const auto s = n.as<std::string>();
      if (table[s]) {order.push_back(s);}
    }
  }
  std::set<std::string> have(order.begin(), order.end());
  for (const auto & kv : table) {
    const auto s = kv.first.as<std::string>();
    if (!have.count(s)) {order.push_back(s); have.insert(s);}
  }

  auto spheresOf = [&](const std::string & link) {
      std::vector<Sphere> out;
      const YAML::Node list = table[link];
      if (!list.IsSequence()) {return out;}
      for (const auto & s : list) {
        const YAML::Node c = s["center"] ? s["center"] : s["position"];
        if (!c.IsSequence() || c.size() != 3 || !s["radius"]) {
          throw std::runtime_error("cuRobo yml: malformed sphere on link " + link);
        }
        out.push_back({Vec3(c[0].as<double>(), c[1].as<double>(), c[2].as<double>()),
            s["radius"].as<double>()});
      }
      return out;
    };

  auto nearestParent = [&](const std::string & link) -> std::string {
      std::string cur = link;
      for (size_t hops = 0; hops <= parents.size(); ++hops) {
        const auto it = parents.find(cur);
        if (it == parents.end()) {return "";}
        cur = it->second;
        if (have.count(cur)) {return cur;}
      }
      throw std::runtime_error("cuRobo yml: parent cycle at " + link);
    };

  SphereModel model;
  std::set<std::string> added;
  size_t remaining = order.size();
  while (remaining > 0) {
    const size_t before = remaining;
    for (const auto & link : order) {
      if (added.count(link)) {continue;}
      const std::string par = nearestParent(link);
      if (!par.empty() && !added.count(par)) {continue;}
      model.addLink(link, par.empty() ? -1 : model.linkIndex(par), spheresOf(link));
      added.insert(link);
      --remaining;
    }
    if (remaining == before) {throw std::runtime_error("cuRobo yml: parent cycle");}
  }

  if (const YAML::Node ign = findKey(doc, "self_collision_ignore"); ign.IsMap()) {
    for (const auto & kv : ign) {
      const auto a = kv.first.as<std::string>();
      if (!added.count(a) || !kv.second.IsSequence()) {continue;}
      for (const auto & b : kv.second) {
        if (added.count(b.as<std::string>())) {model.allowPair(a, b.as<std::string>());}
      }
    }
  }
  return model;
}

}  // namespace safety_monitor
