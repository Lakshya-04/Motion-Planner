#pragma once

#include <string>
#include <vector>

#include "gravity_core/gravity_model.hpp"

namespace gravity_core {

// Reads link mass and centre of mass, and revolute, continuous, prismatic, fixed and mimic joints,
// from URDF text. Throws std::runtime_error on bad input or an unknown actuated joint.
GravityModel loadUrdf(const std::string& urdf_xml, const std::vector<std::string>& actuated);

}  // namespace gravity_core
