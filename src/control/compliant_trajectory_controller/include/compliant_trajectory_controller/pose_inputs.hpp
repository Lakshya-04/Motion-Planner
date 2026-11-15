// Input checks shared by the probe / test nodes that take joint names and poses.
#pragma once

#include <string>
#include <vector>

#include "mobile_manipulator_profile/profile.hpp"

namespace compliant_trajectory_controller
{

/// Fill empty `joint_names` / `pose` from the profile (its joints in order, home pose). Returns an
/// empty string on success, otherwise the reason the inputs cannot be used. A profile without
/// joints (failed load) or without a pose for the names is an error, not an empty verdict.
inline std::string resolveJointInputs(
  const mobile_manipulator_profile::Profile & profile, std::vector<std::string> & joint_names,
  std::vector<double> & pose)
{
  if (joint_names.empty()) {
    if (profile.arm_joints.empty()) {
      return "no joint_names given and the profile has no joints (failed to load?); "
             "pass joint_names and joint_pose or a valid profile";
    }
    joint_names = profile.joints();
    if (pose.empty()) {
      pose = profile.homePositions();
    }
  }
  if (joint_names.empty()) {
    return "joint_names is empty";
  }
  if (pose.size() != joint_names.size()) {
    return "joint_pose (" + std::to_string(pose.size()) + ") and joint_names (" +
           std::to_string(joint_names.size()) + ") must be the same length";
  }
  return "";
}

/// The hand-derived poses of the wall approach test belong to one chain. Returns an empty string
/// when `active_joints` is exactly `expected` (names and order) and `pose` has one value each.
inline std::string checkPoseChain(
  const std::vector<std::string> & active_joints, const std::vector<std::string> & expected,
  const std::vector<double> & pose)
{
  if (active_joints != expected || pose.size() != expected.size()) {
    return "the built-in poses are for a " + std::to_string(expected.size()) +
           "-joint chain (" + (expected.empty() ? std::string() : expected.front()) +
           ", ...); the planning group has " + std::to_string(active_joints.size()) +
           " active joints, so they do not apply to this arm";
  }
  return "";
}

}  // namespace compliant_trajectory_controller
