/**
 * @file path_shortcut.hpp
 * @brief Randomised joint-space shortcutting and corner-cutting smoothing of a planned path, ROS-free.
 * @details A sampling planner returns a path with detours and corners. Both passes here only ever replace part of
 *          the path by a straight joint-space segment whose every densified sample passed the validity callback
 *          (the caller binds it to the planning scene), so the result is valid whenever the input was and is never
 *          longer. Shortcutting joins two random waypoints directly; smoothing cuts each interior corner at a
 *          fraction of the adjacent segments (the corner-cutting step of a quadratic B-spline, repeated), keeping
 *          a cut only when its segments are valid. A fixed seed makes it reproducible.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "motion_planner/core/direct_joint_path.hpp"
#include "motion_planner/core/path_quality.hpp"

namespace motion_planner
{

struct PathShortcutConfig
{
  std::size_t shortcut_iterations{300};
  std::size_t smoothing_rounds{4};   ///< Corner-cutting rounds; 0 = off.
  double corner_fraction{0.25};      ///< Cut each corner at this fraction (0, 0.5) of the adjacent segments.
  std::uint32_t seed{1};
  DirectJointPathConfig sampling;    ///< Step limits for the validity check of a replacement segment.
};

/// True when every densified sample of the segment a->b (ends included) is valid.
inline bool segmentValid(
  const std::vector<double> & a, const std::vector<double> & b, const DirectJointPathConfig & cfg,
  const JointStateValidity & validity)
{
  for (const auto & q : densify(a, b, cfg)) {
    if (!validity(q)) {
      return false;
    }
  }
  return true;
}

/// Randomised shortcutting: join waypoint i and j (j > i + 1) directly when the straight segment is valid.
inline std::vector<std::vector<double>> shortcutPath(
  std::vector<std::vector<double>> path, const PathShortcutConfig & cfg, const JointStateValidity & validity)
{
  std::mt19937 rng(cfg.seed);
  for (std::size_t it = 0; it < cfg.shortcut_iterations && path.size() > 2; ++it) {
    std::uniform_int_distribution<std::size_t> pick(0, path.size() - 1);
    std::size_t i = pick(rng);
    std::size_t j = pick(rng);
    if (i > j) {
      std::swap(i, j);
    }
    if (j < i + 2) {
      continue;
    }
    if (segmentValid(path[i], path[j], cfg.sampling, validity)) {
      path.erase(path.begin() + static_cast<std::ptrdiff_t>(i) + 1, path.begin() + static_cast<std::ptrdiff_t>(j));
    }
  }
  return path;
}

/// Corner cutting: each interior waypoint p becomes two points on its adjacent segments; kept when valid.
inline std::vector<std::vector<double>> smoothPath(
  std::vector<std::vector<double>> path, const PathShortcutConfig & cfg, const JointStateValidity & validity)
{
  const double f = cfg.corner_fraction;
  if (!(f > 0.0 && f < 0.5)) {
    return path;
  }
  for (std::size_t round = 0; round < cfg.smoothing_rounds && path.size() > 2; ++round) {
    std::vector<std::vector<double>> out;
    out.push_back(path.front());
    for (std::size_t k = 1; k + 1 < path.size(); ++k) {
      std::vector<double> in(path[k].size());
      std::vector<double> next(path[k].size());
      for (std::size_t j = 0; j < path[k].size(); ++j) {
        in[j] = path[k][j] + f * (path[k - 1][j] - path[k][j]);
        next[j] = path[k][j] + f * (path[k + 1][j] - path[k][j]);
      }
      const std::vector<double> & prev = out.back();
      if (segmentValid(prev, in, cfg.sampling, validity) && segmentValid(in, next, cfg.sampling, validity)) {
        out.push_back(std::move(in));
        out.push_back(std::move(next));
      } else {
        out.push_back(path[k]);
      }
    }
    out.push_back(path.back());
    // The cut segment must also reach the next waypoint; when the following corner was kept uncut, check the join.
    bool ok = true;
    for (std::size_t k = 1; k < out.size() && ok; ++k) {
      ok = segmentValid(out[k - 1], out[k], cfg.sampling, validity);
    }
    if (!ok) {
      break;
    }
    path = std::move(out);
  }
  return path;
}

/// Shortcut then smooth, then keep the result only when it is not longer (weighted) than the input.
inline std::vector<std::vector<double>> improvePath(
  const std::vector<std::vector<double>> & path, const PathShortcutConfig & cfg,
  const PathQualityConfig & quality, const JointStateValidity & validity)
{
  if (path.size() < 2 || !validity) {
    return path;
  }
  auto out = smoothPath(shortcutPath(path, cfg, validity), cfg, validity);
  return weightedPathLength(out, quality) <= weightedPathLength(path, quality) + 1e-12 ? out : path;
}

}  // namespace motion_planner
