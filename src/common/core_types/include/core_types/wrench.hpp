/**
 * @file wrench.hpp
 * @brief Pure force/torque triple, with no ROS message dependency.
 * @details Adapters (e.g. the F/T bridge hardware component) translate to/from
 *          geometry_msgs::msg::Wrench at the ROS boundary -- this type never crosses that
 *          boundary itself, matching core_types::Pose's discipline.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cmath>

namespace core_types
{

/**
 * @struct Wrench
 * @brief Force/torque triple, expressed in whatever frame the caller documents.
 * @details SIGN CONVENTION for contact tasks (asserted once at the adapter boundary, never
 *          re-derived downstream): this is the wrench the ENVIRONMENT exerts ON the tool,
 *          expressed in the tool control frame (link_tcp). Pressing the tool into a wall along
 *          its own +Z therefore yields force_z ~= -target_force_n. Use normalForceN() rather
 *          than reading force_z directly, so "15 N of contact" reads as +15 everywhere the
 *          force layer does arithmetic.
 */
struct Wrench
{
  double force_x{0.0};   ///< N.
  double force_y{0.0};   ///< N.
  double force_z{0.0};   ///< N; negative while pressing along the tool's own +Z (see convention above).
  double torque_x{0.0};  ///< Nm.
  double torque_y{0.0};  ///< Nm.
  double torque_z{0.0};  ///< Nm.
};

/// Contact force along the tool's own +Z, positive while loaded against the surface.
inline double normalForceN(const Wrench & wrench)
{
  return -wrench.force_z;
}

/// Magnitude of the tangential (in-plane, i.e. about the tool's X/Y axes) torque -- the
/// bending/tilting load a tool takes from friction or a snag, as distinct from torque_z (twist
/// about the contact normal itself). Drives ToolTask::tangential_torque_abort_nm.
inline double tangentialTorqueNm(const Wrench & wrench)
{
  return std::sqrt(wrench.torque_x * wrench.torque_x + wrench.torque_y * wrench.torque_y);
}

}  // namespace core_types
