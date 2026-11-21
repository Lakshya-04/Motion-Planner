/**
 * @file curobo_yaml.hpp
 * @brief Builds a SphereModel from the collision-sphere table the planners already use (a cuRobo
 *        robot yml, as written by docker/tools/generate_curobo_robot.py).
 * @details Reads `kinematics.collision_spheres` (`link: [{center: [x, y, z], radius: r}, ...]`, also
 *          under `robot_cfg`, or at the top level), the optional `kinematics.collision_link_names`
 *          (link order; default: the order of the sphere table) and the optional
 *          `kinematics.self_collision_ignore` (`link: [links]`, loaded as allowed pairs). A cuRobo
 *          yml does not carry the kinematic tree, so the caller may pass `parents`
 *          (link -> parent link name, e.g. from the URDF); the nearest ancestor present in the sphere
 *          table becomes the parent (parent-child pairs are then excluded). A sphere table given as a
 *          file name instead of inline is not followed: pass that file's text. Throws
 *          std::runtime_error on a missing table, a malformed sphere or a parent cycle.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <map>
#include <string>

#include "safety_monitor/sphere_model.hpp"

namespace safety_monitor
{

SphereModel loadSphereModelFromCuroboYaml(
  const std::string & yaml_text, const std::map<std::string, std::string> & parents = {});

}  // namespace safety_monitor
