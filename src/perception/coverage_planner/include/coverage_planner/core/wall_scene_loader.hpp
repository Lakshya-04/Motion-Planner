#ifndef COVERAGE_PLANNER__WALL_SCENE_LOADER_HPP_
#define COVERAGE_PLANNER__WALL_SCENE_LOADER_HPP_

#include "coverage_planner/core/types.hpp"
#include <cstdint>
#include <string>
#include <vector>


namespace coverage_planner
{


/**
 * @brief Flatness/reliability gate applied to a wall's own plane-fit-quality metrics before it is
 *        handed to the 2D coverage core. A wall whose reconstruction spread exceeds these bounds
 *        is not flat enough for the (u, v) approximation the rest of this pipeline relies on.
 */
struct WallFlatnessThresholds
{
  double max_plane_rms_mm{15.0};
  double max_spread_p95_mm{50.0};
  double max_spread_p99_mm{75.0};
  /// If true, a threshold violation is logged by the caller (via WallSceneQuery::flatness_warning)
  /// instead of loadWallScene() throwing. Defaults to false: refuse loudly, don't silently plan.
  bool allow_override{false};
};

struct WallSceneQuery
{
  WallPlane wall;
  std::vector<ObstacleRegion> obstacles;
  /// Set when `allow_override` let a flatness-threshold violation through instead of throwing;
  /// empty otherwise. Callers should log this loudly even though loading succeeded.
  std::string flatness_warning;
};

/**
 * @brief Parse one wall (plus its obstacles) out of a wall_scene.json-shaped export.
 * @details Schema matches `test/data/wall_scene.json` (top-level `walls[]`/`obstacles[]`, each
 *          obstacle's `parent_wall` selecting which wall it belongs to). Throws
 *          std::runtime_error on: unreadable/unparsable file, no wall with `wall_id`, or (unless
 *          `thresholds.allow_override`) the wall's flatness metrics exceeding `thresholds`. Only
 *          the u/v-relevant obstacle fields are kept -- the 3D fields belong to the separate 3D
 *          collision-scene pathway, not here.
 */
WallSceneQuery loadWallScene(
  const std::string & json_path, uint32_t wall_id,
  const WallFlatnessThresholds & thresholds = WallFlatnessThresholds{});

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__WALL_SCENE_LOADER_HPP_
