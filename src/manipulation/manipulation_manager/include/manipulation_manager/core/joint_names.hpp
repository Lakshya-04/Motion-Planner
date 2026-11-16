#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace manipulation_manager
{

/// The first candidate joint list whose every name is in `available` (the names a /joint_states message
/// carries); an empty vector when none is. The manager reads the arm's live joint values by name. With a
/// fixed list that does not match the robot (the repo's UR10e + telescopic lift is lift_joint,
/// arm_shoulder_pan_joint ..., an earlier arm was lift_prismatic_joint, joint1 ...), a lookup that falls back to 0.0
/// for a missing name silently turns "the live state" into the all-zero pose, and the "prime" move then
/// drives the arm there (UR10e at zero: fully stretched, lift on its end stop).
inline std::vector<std::string> selectJointNames(
  const std::vector<std::vector<std::string>> & candidates, const std::vector<std::string> & available)
{
  for (const auto & names : candidates) {
    if (names.empty()) {
      continue;
    }
    const bool all = std::all_of(
      names.begin(), names.end(), [&available](const std::string & n) {
        return std::find(available.begin(), available.end(), n) != available.end();
      });
    if (all) {
      return names;
    }
  }
  return {};
}

}  // namespace manipulation_manager
