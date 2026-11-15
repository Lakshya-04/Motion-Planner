/**
 * @file wrench_bias.hpp
 * @brief Pure force-setpoint bias math for the FT bridge.
 * @details admittance_controller drives toward a measured wrench of zero -- it has no force
 *          setpoint of its own (verified against the installed Humble 2.52.1 source). Subtracting the active setpoint
 *          from the raw measured wrench before exporting it turns that zero-seeking law into a
 *          target-seeking one with no controller changes at all. Kept as a free function,
 *          independent of core_types::Wrench, because this component is a ROS adapter and
 *          works directly with geometry_msgs types at its boundary, per this codebase's
 *          core/adapter split -- pulling in core_types here for a two-line subtraction isn't
 *          warranted.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <array>
#include <cstddef>

namespace ft_bridge_hardware
{

/// Six-component wrench as force.x, force.y, force.z, torque.x, torque.y, torque.z --
/// the exact order and naming ros2_control state interfaces for an FT sensor use.
using WrenchArray = std::array<double, 6>;

/**
 * @brief raw - setpoint, component-wise.
 * @details A zero setpoint (the default, until something publishes on setpoint_topic) makes
 *          this an exact no-op -- the bridge behaves identically to a plain pass-through until a
 *          tool profile's target force is actually published.
 */
inline WrenchArray applyWrenchBias(const WrenchArray & raw, const WrenchArray & setpoint)
{
  WrenchArray result{};
  for (size_t i = 0; i < result.size(); ++i) {
    result[i] = raw[i] - setpoint[i];
  }
  return result;
}

}  // namespace ft_bridge_hardware
