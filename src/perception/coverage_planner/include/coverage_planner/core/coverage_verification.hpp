/**
 * @file coverage_verification.hpp
 * @brief What a run actually covered: a per-cell raster built from executed tool strokes.
 * @details Works in the wall's own (u, v) plane, ROS-free. A stroke is a sequence of tool
 *          positions with the tool working (contact held, valve open). The footprint of a stroke
 *          is a band of width `swath_m` around its path (the tool's fan, or the mop's width, runs
 *          perpendicular to travel). A cell counts once per stroke however many samples fall on
 *          it, so `mean_overlap` measures how many separate passes reached a cell, not how
 *          densely the path was sampled.
 *
 *          Limits: a geometric model. It has no deposition thickness, no spray falloff across the
 *          swath and no timing; a spray tool's quality needs the thickness model on top of it.
 *          Samples with `engaged == false` (transit, approach, retreat) never count.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace coverage_planner
{

/// One tool position on the wall plane.
struct ToolSample
{
  double u{0.0};
  double v{0.0};
  bool engaged{true};   ///< The tool is working here (in contact / valve open).
};

/// One continuous stroke of the tool and the swath it works with.
struct ToolStroke
{
  std::vector<ToolSample> samples;
  double swath_m{0.0};  ///< Footprint width perpendicular to travel, > 0.
};

struct VerificationReport
{
  size_t target_cells{0};      ///< Cells that had to be covered.
  size_t covered_cells{0};     ///< Target cells reached by at least one stroke.
  double covered_fraction{0.0};  ///< covered / target (1 when there is no target).
  double mean_overlap{0.0};    ///< Mean number of strokes over the covered target cells.
  size_t outside_cells{0};     ///< Cells worked that were not target (overspray / off the wall).
  std::vector<size_t> missed;  ///< Row-major indices of the target cells nobody reached.
};

/// A raster over a wall's (u, v) rectangle.
class CoverageVerifier
{
public:
  /// @param target Row-major width*height mask of the cells that must be covered; empty = all.
  /// @throws std::invalid_argument for a non-positive resolution or extent, or a mask of the
  ///         wrong size.
  CoverageVerifier(
    double u_min, double u_max, double v_min, double v_max, double resolution,
    const std::vector<bool> & target = {});

  /// Add one stroke: every cell within swath/2 of its path counts once. Consecutive samples are
  /// joined only when both are engaged; a lone engaged sample marks a disc.
  /// @throws std::invalid_argument if swath_m <= 0.
  void addStroke(const ToolStroke & stroke);

  VerificationReport report() const;

  int width() const {return width_;}
  int height() const {return height_;}
  /// Strokes that reached cell (column, row); 0 outside the raster.
  uint16_t hits(int column, int row) const;

private:
  void markSegment(double u0, double v0, double u1, double v1, double radius);

  double u_min_, v_min_, res_;
  int width_, height_;
  std::vector<bool> target_;
  std::vector<uint16_t> hits_;
  std::vector<uint32_t> stamp_;   ///< Last stroke that touched a cell (count once per stroke).
  uint32_t stroke_id_{0};
  size_t outside_{0};
};

}  // namespace coverage_planner
