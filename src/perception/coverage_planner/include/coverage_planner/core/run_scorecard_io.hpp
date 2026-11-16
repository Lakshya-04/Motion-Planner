/**
 * @file run_scorecard_io.hpp
 * @brief Text formats for scoring a recorded run: the sample CSV, the wall JSON, the card JSON.
 * @details Sample CSV: a header line `t,x,y,z,force,active` and one row per sample (seconds, metres,
 *          newtons along the wall normal pressing on the wall, 0/1 for the valve or pump). `#` lines
 *          and blank lines are skipped. Wall JSON: `{"origin": [x,y,z], "u": [..], "v": [..],
 *          "u_range": [min,max], "v_range": [min,max]}`; the normal is u x v, `u` and `v` must be
 *          orthonormal. Positions are in the same frame as the wall. Optional `"obstacles": [{"id": 0,
 *          "u": [min,max], "v": [min,max], "height_m": 0.04}]` (u, v in the same coordinates as the
 *          ranges; the height stands off the wall against u x v, which points into the wall).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <istream>
#include <vector>

#include <nlohmann/json.hpp>

#include "coverage_planner/core/run_scorecard.hpp"

namespace coverage_planner
{

/// @throws std::runtime_error on a missing or wrong header, or a row that is not 6 numbers.
std::vector<RunSample> parseRunCsv(std::istream & in);

/// @throws std::runtime_error for a missing field, a wrong length, or a basis that is not orthonormal.
WallPlane wallFromJson(const nlohmann::json & j);

/// The wall JSON's `obstacles` (none if the key is missing), parented to `wall_id`.
/// @throws std::runtime_error for an entry with a missing or reversed range.
std::vector<ObstacleRegion> obstaclesFromJson(const nlohmann::json & j, uint32_t wall_id);

nlohmann::json scorecardToJson(const RunScorecard & card);

}  // namespace coverage_planner
