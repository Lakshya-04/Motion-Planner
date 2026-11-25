/**
 * @file coverage_output_json_source.hpp
 * @brief CoverageSource backed by coverage_planner's own coverage_output.json.
 * @details sii's original CoverageSource only ever had a hardcoded
 *          YamlCoverageSource -- the real coverage-planner-backed implementation was
 *          scoped as follow-on work there and never landed. coverage_planner here
 *          emits a JSON file (final_lifted_and_expanded.path.poses, base_link frame), not a
 *          live topic/service, so this reads that file directly.
 *
 *          The raw JSON is a boustrophedon path: each unique x groups a vertical "column"
 *          of (usually just 2, top+bottom) points on the wall. A CoverageSegmentIntent per
 *          raw point pair would jump straight between them -- this source instead resamples
 *          each column at z_step_m intervals within [z_min_m, z_max_m] (an ABSOLUTE clamp in
 *          the output frame, not a margin off the column's own raw extent -- the safe band is
 *          asymmetric: low z self-collides with the base/lift links, high z clips the wall's
 *          own top edge into a near-zero-penetration, ~1000N-force physics-engine instability,
 *          confirmed live in this repo; a single symmetric margin can't express that).
 *          Defaults to [1.20m, 1.60m], the exact band already proven reachable/stable for this
 *          tool+wall+base geometry. CoverageFeasibilityAnalyzer decides which of the resampled
 *          points are actually reachable from wherever the base currently is -- this class only
 *          emits candidates.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>
#include <vector>

#include "manipulation_manager/core/coverage_source.hpp"
#include "manipulation_manager/core/wall_direction.hpp"

namespace manipulation_manager
{

class CoverageOutputJsonSource : public CoverageSource
{
public:
  /**
   * @param[in] json_path Path to coverage_planner's coverage_output.json.
   * @param[in] frame_id Frame the OUTPUT poses are expressed in (e.g. "base_footprint") --
   *            this class doesn't re-derive it from the JSON, since z_offset already commits
   *            the caller to a specific target frame.
   * @param[in] z_offset Added to every point's z -- converts the JSON's own frame
   *            (base_link, per wall_scene.json's own frame_id) into frame_id above (e.g.
   *            +0.31m for base_footprint).
   * @param[in] z_step_m Spacing between resampled contact points within one column.
   * @param[in] z_min_m Absolute lower z clamp (output frame) -- see this class's own doc
   *            comment for why this isn't a margin off each column's own raw extent.
   * @param[in] z_max_m Absolute upper z clamp (output frame).
   * @param[in] target_eef_speed A controlled, non-crash approach/contact speed (m/s).
   * @param[in] wall_direction Which way the wall lies from the robot. A column is the set of
   *            points sharing the same position along the wall's horizontal tangent (direction x up),
   *            ordered by ascending tangent coordinate; for the default +y wall that is x, as before.
   */
  CoverageOutputJsonSource(
    std::string json_path,
    std::string frame_id,
    double z_offset,
    double z_step_m = 0.1,
    double z_min_m = 1.20,
    double z_max_m = 1.60,
    double target_eef_speed = 0.02,
    WallDirection wall_direction = WallDirection{});

  /// true: every stroke keeps the direction the plan gave it (the plan alternates up and down, so the next
  /// strip starts where the last one ended). false (default): every stroke runs bottom to top.
  void setFollowPlanDirection(bool follow) {follow_plan_direction_ = follow;}

  std::vector<CoverageSegmentIntent> getCoveragePlan() override;
  std::string getFrameId() override;

private:
  std::string json_path_;
  std::string frame_id_;
  double z_offset_;
  double z_step_m_;
  double z_min_m_;
  double z_max_m_;
  double target_eef_speed_;
  WallDirection wall_dir_;
  bool follow_plan_direction_{false};
};

}  // namespace manipulation_manager
