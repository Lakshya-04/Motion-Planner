/**
 * @file force_stage.hpp
 * @brief Per-segment force-control intent, independent of ROS.
 * @details Mirrors reference_docs/13_cleaning_modes_spec.md §2.2's proposed
 *          MotionPlanSegment.msg force_stage field (FORCE_STAGE_NONE/APPROACH/CARTESIAN/RETRACT),
 *          kept here as the pure-core representation adapters convert the wire uint8 into.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstdint>

namespace core_types
{

/**
 * @enum ForceStage
 * @brief Which force-control obligation, if any, a segment carries.
 */
enum class ForceStage : uint8_t
{
  kNone = 0,      ///< No force control -- today's exact behavior.
  kApproach = 1,  ///< Advance along the tool normal, seeking contact.
  kCartesian = 2, ///< Track the coverage path while holding target_force_n.
  kRetract = 3,   ///< Unload to zero, then peel off the surface.
};

/**
 * @struct ForceIntent
 * @brief Force-control intent attached to one segment/execution request.
 */
struct ForceIntent
{
  ForceStage stage{ForceStage::kNone};  ///< kNone == today's exact behavior, everywhere.
  double target_force_n{0.0};           ///< 0 = use the active ToolProfile's task.target_force_n.
};

/// True for the two stages where the tool is expected to be in physical contact with the
/// surface -- the producer-side assertion reference_docs/13_cleaning_modes_spec.md §2.2 proposes
/// as "requires_contact == (force_stage in {APPROACH, CARTESIAN})", shared by both producer and
/// consumer so they cannot drift apart.
inline bool isContactStage(ForceStage stage)
{
  return stage == ForceStage::kApproach || stage == ForceStage::kCartesian;
}

/// Converts a wire (uint8) force_stage value to the enum. Any value outside the four known ones
/// degrades to kNone rather than freezing or aborting -- see
/// reference_docs/13_cleaning_modes_spec.md's explicit requirement that an unrecognized stage
/// must fail safe into "no force control", not into an undefined state.
inline ForceStage forceStageFromWireValue(uint8_t value)
{
  switch (value) {
    case static_cast<uint8_t>(ForceStage::kApproach):
      return ForceStage::kApproach;
    case static_cast<uint8_t>(ForceStage::kCartesian):
      return ForceStage::kCartesian;
    case static_cast<uint8_t>(ForceStage::kRetract):
      return ForceStage::kRetract;
    default:
      return ForceStage::kNone;
  }
}

}  // namespace core_types
