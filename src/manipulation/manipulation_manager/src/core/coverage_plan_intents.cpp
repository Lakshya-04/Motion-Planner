#include "manipulation_manager/core/coverage_plan_intents.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <thread>

namespace manipulation_manager
{

namespace
{

struct RawPoint
{
  double x, y, z, qx, qy, qz, qw;
  int segment{-1};   // index into the plan's `segments`, -1 if the plan has none
  size_t order{0};   // position in the plan's path
};

}  // namespace

void validateCoveragePlan(const RawCoveragePlan & plan)
{
  if (!plan.success) {
    throw std::runtime_error(
      "the coverage planner reported failure: " + (plan.message.empty() ? std::string("no message") : plan.message));
  }
  if (plan.poses.empty()) {
    throw std::runtime_error("the coverage plan has no poses");
  }
  for (const auto & s : plan.segments) {
    if (s.start_index > s.end_index || s.end_index >= plan.poses.size()) {
      throw std::runtime_error(
        "a coverage plan segment has indices " + std::to_string(s.start_index) + ".." +
        std::to_string(s.end_index) + " outside its " + std::to_string(plan.poses.size()) + " poses");
    }
  }
}

std::string coveragePlanFingerprint(const RawCoveragePlan & plan)
{
  std::string out;
  char buf[160];
  for (const auto & p : plan.poses) {
    std::snprintf(buf, sizeof(buf), "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw);
    out += buf;
  }
  for (const auto & s : plan.segments) {
    out += "s " + std::to_string(s.start_index) + " " + std::to_string(s.end_index) + "\n";
  }
  return out;
}

std::vector<CoverageSegmentIntent> buildCoverageIntents(
  const RawCoveragePlan & plan, const CoverageIntentOptions & opt)
{
  // Group raw points by their position along the wall's horizontal tangent (direction x up; x for
  // the +y wall)
  const auto & wd = opt.wall_dir.d;
  const double tx = wd[1], ty = -wd[0];
  // Group raw points by x (rounded to kill float noise between two points on the same
  // column) -- the boustrophedon path visits each column as a top/bottom pair, sorted here
  // left-to-right (ascending x) so a caller sweeping the wall naturally starts at the left.
  // The plan's segments say which points belong to one stroke: an obstacle splits a column into several strokes
  // whose ends are far apart, and the manager must stroke each of them, not the whole column through the obstacle.
  const auto & poses = plan.poses;
  std::vector<int> segment_of_pose(poses.size(), -1);
  {
    int seg = 0;
    for (const auto & s : plan.segments) {
      for (size_t i = s.start_index; i <= s.end_index && i < segment_of_pose.size(); ++i) {
        segment_of_pose[i] = seg;
      }
      ++seg;
    }
  }
  std::map<long, std::vector<RawPoint>> columns_by_x_key;
  constexpr double kXQuantum = 1e-3;
  // Half the length of the short stroke given to a one-cell gap. A free cell lies wholly outside the inflated
  // obstacle, so a stroke within half a cell of the cell centre cannot enter the keep-out clearance. 0.01 m is
  // half the 0.02 m grid resolution of the demo scenes; a coarser grid is safe, a finer one needs this lowered.
  constexpr double kPointStrokeHalfM = 0.01;
  size_t pose_index = 0;
  for (const auto & p : poses) {
    RawPoint pt;
    pt.order = pose_index;
    pt.segment = segment_of_pose[pose_index++];
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z + opt.z_offset;
    pt.qx = p.qx;
    pt.qy = p.qy;
    pt.qz = p.qz;
    pt.qw = p.qw;
    const long key = static_cast<long>(std::round((pt.x * tx + pt.y * ty) / kXQuantum));
    columns_by_x_key[key].push_back(pt);
  }

  std::vector<CoverageSegmentIntent> segments;
  segments.reserve(columns_by_x_key.size());
  size_t column_index = 0;
  for (auto & [key, column] : columns_by_x_key) {
    std::sort(column.begin(), column.end(), [](const RawPoint & a, const RawPoint & b) {
        return a.z < b.z;
      });
    // z span of every stroke in the column (passes over the same stretch share one span); without segment
    // information the whole column is one span
    struct Span {double lo, hi; bool descending;};
    std::vector<Span> spans;
    if (column.front().segment < 0) {
      // no segment information: the whole column, direction of the first and last pose in plan order
      const auto first = std::min_element(column.begin(), column.end(), [](const RawPoint & a, const RawPoint & b) {
          return a.order < b.order;});
      const auto last = std::max_element(column.begin(), column.end(), [](const RawPoint & a, const RawPoint & b) {
          return a.order < b.order;});
      spans.push_back({column.front().z, column.back().z, first->z > last->z});
    } else {
      struct Acc {double lo, hi; size_t first_order, last_order; double first_z, last_z;};
      std::map<int, Acc> by_segment;
      for (const auto & pt : column) {
        auto it = by_segment.find(pt.segment);
        if (it == by_segment.end()) {
          by_segment[pt.segment] = {pt.z, pt.z, pt.order, pt.order, pt.z, pt.z};
        } else {
          auto & a = it->second;
          a.lo = std::min(a.lo, pt.z);
          a.hi = std::max(a.hi, pt.z);
          if (pt.order < a.first_order) {a.first_order = pt.order; a.first_z = pt.z;}
          if (pt.order > a.last_order) {a.last_order = pt.order; a.last_z = pt.z;}
        }
      }
      for (const auto & [seg, a] : by_segment) {
        bool seen = false;
        for (const auto & e : spans) {
          if (std::fabs(e.lo - a.lo) < 1e-3 && std::fabs(e.hi - a.hi) < 1e-3) {seen = true;}
        }
        if (!seen) {spans.push_back({a.lo, a.hi, a.first_z > a.last_z});}
      }
      std::sort(spans.begin(), spans.end(), [](const Span & a, const Span & b) {
          return a.lo < b.lo || (a.lo == b.lo && a.hi < b.hi);});
    }
    const auto & rep = column.front();
    for (const auto & span : spans) {
      double z_min = std::max(opt.z_min_m, span.lo);
      double z_max = std::min(opt.z_max_m, span.hi);
      // The planner's gap strokes beside a small obstacle can be a single grid cell (first == last): a stroke of
      // zero length would be dropped and the strip next to the obstacle never wiped. Give it a short stroke
      // around the point instead, kept inside the free cell (see kPointStrokeHalfM).
      if (span.hi - span.lo < 1e-6 && z_max >= z_min) {
        z_min = std::max(opt.z_min_m, span.lo - kPointStrokeHalfM);
        z_max = std::min(opt.z_max_m, span.lo + kPointStrokeHalfM);
      }

      CoverageSegmentIntent intent;
      intent.label = "wall_column_" + std::to_string(column_index);
      intent.mode_hint = "cartesian";
      intent.requires_contact = true;
      intent.requires_retraction = true;
      intent.target_eef_speed = opt.target_eef_speed;

      if (z_max > z_min) {
        for (double z = z_min; z <= z_max + 1e-9; z += opt.z_step_m) {
          core_types::Pose pose;
          pose.x = rep.x;
          pose.y = rep.y;
          pose.z = z;
          pose.qx = rep.qx;
          pose.qy = rep.qy;
          pose.qz = rep.qz;
          pose.qw = rep.qw;
          intent.poses.push_back(pose);
        }
      }

      if (opt.follow_plan_direction && span.descending) {
        std::reverse(intent.poses.begin(), intent.poses.end());
      }
      if (!intent.poses.empty()) {
        segments.push_back(std::move(intent));
      }
      ++column_index;
    }
  }
  return segments;
}

void CoveragePlanInbox::put(RawCoveragePlan plan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_ = std::move(plan);
  have_ = true;
}

bool CoveragePlanInbox::has() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return have_;
}

bool CoveragePlanInbox::waitFor(
  double timeout_s, const std::function<void()> & pump, RawCoveragePlan & out)
{
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(std::max(0.0, timeout_s)));
  for (;;) {
    if (pump) {pump();}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_) {
        out = plan_;
        return true;
      }
    }
    if (Clock::now() >= deadline) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}  // namespace manipulation_manager
