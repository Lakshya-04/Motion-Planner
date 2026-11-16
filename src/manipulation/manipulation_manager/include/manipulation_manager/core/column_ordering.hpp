#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace manipulation_manager
{

/// Where a column's stroke starts and ends (same frame for all, metres).
struct ColumnEnds
{
  std::array<double, 3> start{0.0, 0.0, 0.0};
  std::array<double, 3> end{0.0, 0.0, 0.0};
};

/// Travel time between two points: the slowest axis' distance over its speed (axes move at the same time, like
/// the joints of a free-space move do). `axis_speed` of 0 on an axis ignores that axis.
inline double travelTime(
  const std::array<double, 3> & a, const std::array<double, 3> & b,
  const std::array<double, 3> & axis_speed = {1.0, 1.0, 1.0})
{
  double t = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    if (axis_speed[i] > 0.0) {
      t = std::max(t, std::fabs(a[i] - b[i]) / axis_speed[i]);
    }
  }
  return t;
}

/// Total time of visiting `order` (indices into `columns`) in turn, strokes in their own direction: the moves
/// from `from` to the first start and from each end to the next start (stroke time itself does not depend on
/// the order). Used by tests and by the 2-opt pass.
inline double orderTravelTime(
  const std::vector<ColumnEnds> & columns, const std::vector<size_t> & order,
  const std::array<double, 3> & from, const std::array<double, 3> & axis_speed = {1.0, 1.0, 1.0})
{
  double total = 0.0;
  std::array<double, 3> at = from;
  for (const size_t c : order) {
    total += travelTime(at, columns[c].start, axis_speed);
    at = columns[c].end;
  }
  return total;
}

/// The order to run the columns of one stand in, by a time metric: nearest neighbour from `from` (the
/// travel time to the next stroke's start), then an optional 2-opt pass that reverses a stretch of the order
/// when that is faster (strokes keep their own direction: the contact stroke is the same way every time).
/// The result is a permutation of 0 .. n-1.
inline std::vector<size_t> orderColumnsByTime(
  const std::vector<ColumnEnds> & columns, const std::array<double, 3> & from, bool two_opt = true,
  const std::array<double, 3> & axis_speed = {1.0, 1.0, 1.0})
{
  const size_t n = columns.size();
  std::vector<size_t> order;
  order.reserve(n);
  std::vector<bool> used(n, false);
  std::array<double, 3> at = from;
  for (size_t k = 0; k < n; ++k) {
    size_t best = n;
    double best_t = 0.0;
    for (size_t c = 0; c < n; ++c) {
      if (used[c]) {
        continue;
      }
      const double t = travelTime(at, columns[c].start, axis_speed);
      if (best == n || t < best_t) {   // strict: ties keep the input order
        best = c;
        best_t = t;
      }
    }
    used[best] = true;
    order.push_back(best);
    at = columns[best].end;
  }
  if (two_opt && n > 2) {
    bool improved = true;
    int guard = 0;
    while (improved && guard++ < 100) {
      improved = false;
      const double base = orderTravelTime(columns, order, from, axis_speed);
      for (size_t i = 0; i + 1 < n && !improved; ++i) {
        for (size_t j = i + 1; j < n && !improved; ++j) {
          auto candidate = order;
          std::reverse(candidate.begin() + static_cast<long>(i), candidate.begin() + static_cast<long>(j) + 1);
          if (orderTravelTime(columns, candidate, from, axis_speed) < base - 1e-9) {
            order = candidate;
            improved = true;
          }
        }
      }
    }
  }
  return order;
}

/// The column order policy name from the `column_order` parameter: "input" (default, the plan's order),
/// "nearest" (nearest neighbour), "nearest_2opt" (nearest neighbour then 2-opt). Unknown names give "input".
inline std::string normalizeColumnOrder(const std::string & name)
{
  if (name == "nearest" || name == "nearest_2opt") {
    return name;
  }
  return "input";
}

}  // namespace manipulation_manager
