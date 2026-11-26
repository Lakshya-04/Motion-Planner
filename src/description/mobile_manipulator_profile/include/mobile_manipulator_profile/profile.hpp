// Header-only reader of the robot profile (mobile_manipulator_description/config/profiles/*.yaml),
// the C++ twin of mobile_manipulator_description.profile. Nodes take group names, joint lists,
// DOF and frame names from here instead of repeating them.
//
//   auto p = mobile_manipulator_profile::loadProfile("ur10e_telescopic");
//   p.whole_body_group;   // "lift_arm"
//   p.dof();              // 7 (lift + 6 arm joints); 8 for the 7-DOF arm
//   p.joints();           // lift joint first, then the arm joints in kinematic order
#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace mobile_manipulator_profile
{

inline constexpr char kDefaultProfile[] = "ur10e_telescopic";

/// One sensor on the base (profile `sensors:` entry). `spec` holds the numeric fields of the type.
struct SensorSpec
{
  std::string type;  ///< lidar2d | rgbd | imu | lidar3d
  std::string name;
  std::string frame;  ///< child of base_link
  std::array<double, 3> xyz{0, 0, 0};  ///< mount in base_link
  std::array<double, 3> rpy{0, 0, 0};
  bool factory = false;
  std::string source;
  std::map<std::string, double> spec;
  std::string depth_noise_note;  ///< rgbd only
};
/// Movement cost of the lift when a profile gives no `joint_costs`; an arm joint is 1.0.
inline constexpr double kDefaultLiftCost = 2.0;

struct Profile
{
  std::string name;
  std::string robot_name;
  std::string arm;  ///< xacro `arm` argument, e.g. "ur10e", "generic_6dof", "panda".
  std::string lift_joint;  ///< Empty: no actuated lift (lift:=none / fixed).
  std::vector<std::string> arm_joints;
  std::vector<std::string> mimic_joints;
  std::map<std::string, double> home;
  /// `joint_costs` of the profile, in joints() order. Empty when the profile has none; use jointCosts().
  std::vector<double> joint_costs;
  std::map<std::string, std::string> frames;
  std::string arm_group = "arm";
  std::string whole_body_group = "lift_arm";
  std::string tool_group = "tool";
  std::string controller_family;
  std::vector<SensorSpec> sensors;  ///< Empty when the profile has no `sensors:` block.

  size_t armDof() const {return arm_joints.size();}
  /// Whole-body DOF: the lift (if actuated) plus the arm.
  size_t dof() const {return joints().size();}
  /// Actuated joints of the whole-body group, lift first.
  std::vector<std::string> joints() const
  {
    std::vector<std::string> all;
    if (!lift_joint.empty()) {all.push_back(lift_joint);}
    all.insert(all.end(), arm_joints.begin(), arm_joints.end());
    return all;
  }
  /// Movement cost per joint in joints() order: how much a joint's travel (as a fraction of its range) counts
  /// against another's when planners and the dock IK choice compare paths. 1.0 is the plain range-normalised
  /// travel. Without `joint_costs` in the profile: the lift kDefaultLiftCost, every arm joint 1.0.
  std::vector<double> jointCosts() const
  {
    if (!joint_costs.empty()) {return joint_costs;}
    std::vector<double> c(dof(), 1.0);
    if (!lift_joint.empty()) {c[0] = kDefaultLiftCost;}
    return c;
  }
  /// Home pose in joints() order.
  std::vector<double> homePositions() const
  {
    std::vector<double> q;
    for (const auto & j : joints()) {q.push_back(home.at(j));}
    return q;
  }
  const std::string & tcpLink() const {return frames.at("tcp");}
  const std::string & frame(const std::string & key) const {return frames.at(key);}
  std::string frameOr(const std::string & key, const std::string & fallback) const
  {
    const auto it = frames.find(key);
    return it == frames.end() ? fallback : it->second;
  }
};

/// Directory with the profile files: $MOBILE_MANIPULATOR_PROFILE_DIR if set, else the installed
/// share directory of mobile_manipulator_description.
inline std::string profilesDir()
{
  if (const char * env = std::getenv("MOBILE_MANIPULATOR_PROFILE_DIR")) {
    if (*env != '\0') {return env;}
  }
  return ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
         "/config/profiles";
}

/// Read and validate the optional `sensors:` list into p.sensors; errors name the profile and entry.
inline void parseSensors(const YAML::Node & root, Profile & p)
{
  const auto & node = root["sensors"];
  if (!node || node.IsNull()) {return;}
  auto fail = [&p](const std::string & what) {
      throw std::runtime_error("profile " + p.name + ": " + what);
    };
  if (!node.IsSequence()) {fail("sensors must be a list");}
  struct Rule
  {
    std::vector<std::string> required;
    std::vector<std::string> optional;
  };
  const std::map<std::string, Rule> rules = {
    {"lidar2d", {{"angle_min_rad", "angle_max_rad", "angle_increment_rad", "range_min_m",
      "range_max_m", "rate_hz"}, {"range_noise_std_m"}}},
    {"rgbd", {{"width", "height", "hfov_rad", "range_min_m", "range_max_m", "rate_hz"}, {}}},
    {"imu", {{"rate_hz"}, {}}},
    {"lidar3d", {{"channels", "hfov_rad", "vfov_rad", "range_min_m", "range_max_m", "rate_hz"},
      {}}},
  };
  std::set<std::string> names, frames;
  size_t index = 0;
  for (const auto & e : node) {
    const std::string where = "sensors[" + std::to_string(index++) + "]" +
      ((e.IsMap() && e["name"]) ? " (" + e["name"].as<std::string>() + ")" : "");
    auto bad = [&](const std::string & what) {fail(where + ": " + what);};
    if (!e.IsMap()) {bad("must be a map");}
    auto text = [&](const char * key) {
        if (!e[key] || !e[key].IsScalar() || e[key].as<std::string>().empty()) {
          bad(std::string("missing ") + key);
        }
        return e[key].as<std::string>();
      };
    auto number = [&](const YAML::Node & n, const std::string & what) {
        double v = 0;
        try {v = n.as<double>();} catch (const YAML::Exception &) {bad(what + " is not a number");}
        if (!std::isfinite(v)) {bad(what + " is not finite");}
        return v;
      };
    SensorSpec s;
    s.type = text("type");
    s.name = text("name");
    s.frame = text("frame");
    s.source = text("source");
    const auto rule = rules.find(s.type);
    if (rule == rules.end()) {bad("unknown type " + s.type);}
    if (!names.insert(s.name).second) {bad("duplicate name");}
    if (s.frame == "base_link" || !frames.insert(s.frame).second) {
      bad("frame must be unique and not base_link");
    }
    if (!e["factory"] || !e["factory"].IsScalar()) {bad("missing factory");}
    try {s.factory = e["factory"].as<bool>();} catch (const YAML::Exception &) {
      bad("factory must be true or false");
    }
    const auto & mount = e["mount"];
    if (!mount || !mount.IsMap()) {bad("missing mount");}
    for (const char * key : {"xyz", "rpy"}) {
      if (!mount[key] || !mount[key].IsSequence() || mount[key].size() != 3) {
        bad(std::string("mount.") + key + " needs three numbers");
      }
      auto & dst = std::string(key) == "xyz" ? s.xyz : s.rpy;
      for (size_t i = 0; i < 3; ++i) {
        dst[i] = number(mount[key][i], std::string("mount.") + key);
      }
    }
    const auto & spec = e["spec"];
    if (!spec || !spec.IsMap()) {bad("missing spec");}
    for (const auto & key : rule->second.required) {
      if (!spec[key]) {bad("spec." + key + " missing");}
      s.spec[key] = number(spec[key], "spec." + key);
    }
    for (const auto & key : rule->second.optional) {
      if (spec[key]) {s.spec[key] = number(spec[key], "spec." + key);}
    }
    for (const auto & kv : s.spec) {
      // angle bounds may be zero or negative; the noise std may be zero
      if (kv.first == "angle_min_rad" || kv.first == "angle_max_rad") {continue;}
      if (kv.first == "range_noise_std_m" ? kv.second < 0 : kv.second <= 0) {
        const bool noise = kv.first == "range_noise_std_m";
        bad("spec." + kv.first + (noise ? " is negative" : " must be positive"));
      }
    }
    if (s.type == "lidar2d") {
      if (s.spec["angle_min_rad"] >= s.spec["angle_max_rad"]) {
        bad("angle_min_rad must be below angle_max_rad");
      }
    }
    if (s.spec.count("range_max_m") && s.spec["range_max_m"] <= s.spec["range_min_m"]) {
      bad("range_max_m must exceed range_min_m");
    }
    if (s.type == "rgbd") {
      if (s.spec["hfov_rad"] >= 3.14159265358979323846) {bad("hfov_rad must be below pi");}
      if (e["spec"]["depth_noise_note"]) {
        s.depth_noise_note = e["spec"]["depth_noise_note"].as<std::string>();
      }
      if (s.depth_noise_note.empty()) {bad("spec.depth_noise_note missing");}
    }
    p.sensors.push_back(s);
  }
}

/// Validate and read a parsed profile. Throws std::runtime_error on a missing or inconsistent field.
inline Profile parseProfile(const YAML::Node & root, const std::string & name)
{
  auto fail = [&name](const std::string & what) {
      throw std::runtime_error("profile " + name + ": " + what);
    };
  Profile p;
  p.name = name;
  if (!root["robot"] || !root["robot"]["xacro_args"] || !root["robot"]["xacro_args"]["arm"]) {
    fail("missing robot.xacro_args.arm");
  }
  p.robot_name = root["robot"]["name"] ? root["robot"]["name"].as<std::string>() :
    "mobile_manipulator";
  p.arm = root["robot"]["xacro_args"]["arm"].as<std::string>();
  if (!root["joints"] || !root["joints"]["arm"]) {fail("missing joints.arm");}
  if (root["joints"]["lift"] && !root["joints"]["lift"].IsNull()) {
    p.lift_joint = root["joints"]["lift"].as<std::string>();
  }
  p.arm_joints = root["joints"]["arm"].as<std::vector<std::string>>();
  if (root["joints"]["mimic"] && !root["joints"]["mimic"].IsNull()) {
    p.mimic_joints = root["joints"]["mimic"].as<std::vector<std::string>>();
  }
  if (p.arm_joints.empty()) {fail("joints.arm is empty");}
  std::set<std::string> unique(p.arm_joints.begin(), p.arm_joints.end());
  if (!p.lift_joint.empty()) {unique.insert(p.lift_joint);}
  if (unique.size() != p.arm_joints.size() + (p.lift_joint.empty() ? 0 : 1)) {
    fail("duplicate joint names");
  }
  if (!root["frames"]) {fail("missing frames");}
  for (const auto & kv : root["frames"]) {
    p.frames[kv.first.as<std::string>()] = kv.second.as<std::string>();
  }
  for (const char * key : {"world_base", "base", "arm_base", "flange", "tool_mount", "tcp"}) {
    if (!p.frames.count(key)) {fail(std::string("frames.") + key + " missing");}
  }
  if (root["groups"]) {
    const auto g = root["groups"];
    if (g["arm"]) {p.arm_group = g["arm"].as<std::string>();}
    if (g["whole_body"]) {p.whole_body_group = g["whole_body"].as<std::string>();}
    if (g["tool"]) {p.tool_group = g["tool"].as<std::string>();}
  }
  p.controller_family = p.arm.rfind("ur", 0) == 0 ? "ur" : p.arm;
  if (root["arm"]) {
    if (root["arm"]["controller_family"]) {
      p.controller_family = root["arm"]["controller_family"].as<std::string>();
    }
    if (root["arm"]["dof"] && root["arm"]["dof"].as<size_t>() != p.arm_joints.size()) {
      fail("arm.dof does not match joints.arm");
    }
  }
  if (root["joint_costs"] && !root["joint_costs"].IsNull()) {
    p.joint_costs = root["joint_costs"].as<std::vector<double>>();
    if (p.joint_costs.size() != p.dof()) {
      fail("joint_costs has " + std::to_string(p.joint_costs.size()) + " entries, joints() has " +
        std::to_string(p.dof()) + " (lift first, then the arm)");
    }
    for (const double c : p.joint_costs) {
      if (!(std::isfinite(c) && c > 0.0)) {fail("joint_costs entries must be finite and above 0");}
    }
  }
  if (root["home"]) {
    for (const auto & kv : root["home"]) {
      p.home[kv.first.as<std::string>()] = kv.second.as<double>();
    }
  }
  for (const auto & j : p.joints()) {
    if (!p.home.count(j)) {fail("home pose misses joint " + j);}
  }
  parseSensors(root, p);
  return p;
}

/// Load by profile name ("ur10e_telescopic") or by path to a .yaml file.
inline Profile loadProfile(const std::string & name_or_path = kDefaultProfile)
{
  const bool is_path = name_or_path.find('/') != std::string::npos ||
    (name_or_path.size() > 5 && name_or_path.substr(name_or_path.size() - 5) == ".yaml");
  const std::string path = is_path ? name_or_path : profilesDir() + "/" + name_or_path + ".yaml";
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot read profile " + path + ": " + e.what());
  }
  std::string stem = path.substr(path.find_last_of('/') == std::string::npos ? 0 :
      path.find_last_of('/') + 1);
  if (stem.size() > 5 && stem.substr(stem.size() - 5) == ".yaml") {stem.resize(stem.size() - 5);}
  return parseProfile(root, stem);
}

/// For nodes that only need names: the profile if it loads, else a Profile holding just the
/// built-in default group names (no joints, no frames; use frameOr) and `warning` set. A node
/// then keeps working where the description package is not installed.
inline Profile loadProfileOrDefaults(const std::string & name_or_path, std::string & warning)
{
  warning.clear();
  try {
    return loadProfile(name_or_path.empty() ? std::string(kDefaultProfile) : name_or_path);
  } catch (const std::exception & e) {
    warning = std::string("using built-in default group names: ") + e.what();
    Profile p;
    p.name = name_or_path;
    return p;
  }
}

}  // namespace mobile_manipulator_profile
