/**
 * @file coverage_source.hpp
 * @brief Seam for "where does the Actual coverage path come from."
 * @details Pure (no ROS deps), matching core_types::Pose's discipline.
 *          Implementations: CoverageOutputJsonSource (coverage_output.json) and RosCoverageSource
 *          (the latched CoveragePlan topic).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>
#include <vector>

#include "core_types/pose.hpp"

namespace manipulation_manager
{

/**
 * @struct CoverageSegmentIntent
 * @brief One segment of a coverage plan, in the shape the coverage planner is
 *        already committed to emitting (strip -> cartesian + requires_contact,
 *        connector/gap -> fsp + requires_retraction). requires_contact and
 *        requires_retraction are carried through as data for now -- nothing
 *        yet acts on them automatically (no force-layer gating, no
 *        auto-inserted retract segments); that lands with §7/§8 separately.
 */
struct CoverageSegmentIntent
{
  std::string label;                        ///< Segment label (for routing/reporting).
  std::vector<core_types::Pose> poses;  ///< This segment's own poses, in order.
  std::string mode_hint;                    ///< "cartesian" | "fsp".
  bool requires_contact{false};             ///< Intentional surface contact along this segment.
  bool requires_retraction{false};          ///< Needs a retract/approach move around it.
  double target_eef_speed{0.0};             ///< Constant EE speed (m/s) for this segment; 0 = off.
};

/**
 * @class CoverageSource
 * @brief Produces the full, ideal coverage plan ("Actual") to attempt.
 */
class CoverageSource
{
public:
  virtual ~CoverageSource() = default;

  /**
   * @brief Return the full coverage plan as an ordered list of segments.
   */
  virtual std::vector<CoverageSegmentIntent> getCoveragePlan() = 0;

  /**
   * @brief Reference frame all poses in the coverage plan are expressed in.
   */
  virtual std::string getFrameId() = 0;
};

}  // namespace manipulation_manager
