/**
 * @file tool_trace.hpp
 * @brief Thread-safe recorder of the tool path and joint states while a column strokes.
 * @details ROS-free, header-only. The node feeds it from three places (the state changes in `setState`, the force
 *          callback and a timer that reads TF and the joint states) and reads it once, after the run, to build the
 *          geometric coverage (`geometric_coverage.hpp`) and the path quality audit input (`path_quality_port.hpp`).
 *          Only samples taken while a stroke is active are kept; each carries the latest force reading and the column
 *          that was running. At most `max_samples` tool samples and as many joint samples are kept (later ones are
 *          dropped and counted), so a very long run cannot grow it without bound.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cstddef>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

#include "manipulation_manager/core/geometric_coverage.hpp"
#include "manipulation_manager/core/path_quality_port.hpp"

namespace manipulation_manager
{

class ToolTrace
{
public:
  explicit ToolTrace(size_t max_samples = 500000) : max_samples_(max_samples) {}

  /// Forget everything (a new task).
  void reset()
  {
    std::lock_guard<std::mutex> lock(m_);
    plan_.clear();
    samples_.clear();
    joints_.clear();
    dropped_ = 0;
    stroking_ = false;
    force_ = -1.0;
  }

  void setPlan(std::vector<PlannedStroke> plan)
  {
    std::lock_guard<std::mutex> lock(m_);
    plan_ = std::move(plan);
  }

  /// A stroke starts (`stroking` true, for `column`) or ends.
  void setStroking(bool stroking, size_t column)
  {
    std::lock_guard<std::mutex> lock(m_);
    stroking_ = stroking;
    column_ = column;
  }

  bool stroking() const
  {
    std::lock_guard<std::mutex> lock(m_);
    return stroking_;
  }

  /// Latest tool force magnitude (N); negative = not read.
  void setForce(double force_n)
  {
    std::lock_guard<std::mutex> lock(m_);
    force_ = force_n;
  }

  /// Adds a tool sample for the running column; false when no stroke is active or the cap is reached.
  bool addTool(const GeoPoint & p)
  {
    std::lock_guard<std::mutex> lock(m_);
    if (!stroking_) {return false;}
    if (samples_.size() >= max_samples_) {
      ++dropped_;
      return false;
    }
    samples_.push_back({column_, p, force_});
    return true;
  }

  /// Adds a joint sample (group joint order) for the running column.
  bool addJoints(const std::vector<double> & q)
  {
    std::lock_guard<std::mutex> lock(m_);
    if (!stroking_ || q.empty()) {return false;}
    auto & v = joints_[column_];
    if (v.size() >= max_samples_) {
      ++dropped_;
      return false;
    }
    v.push_back(q);
    return true;
  }

  std::vector<PlannedStroke> plan() const
  {
    std::lock_guard<std::mutex> lock(m_);
    return plan_;
  }

  std::vector<ToolSample> samples() const
  {
    std::lock_guard<std::mutex> lock(m_);
    return samples_;
  }

  /// One stroke per column that has joint samples, columns ascending.
  std::vector<PathQualityStroke> jointStrokes() const
  {
    std::lock_guard<std::mutex> lock(m_);
    std::vector<PathQualityStroke> out;
    for (const auto & [col, wp] : joints_) {
      out.push_back({col, wp});
    }
    return out;
  }

  size_t dropped() const
  {
    std::lock_guard<std::mutex> lock(m_);
    return dropped_;
  }

private:
  mutable std::mutex m_;
  size_t max_samples_;
  std::vector<PlannedStroke> plan_;
  std::vector<ToolSample> samples_;
  std::map<size_t, std::vector<std::vector<double>>> joints_;
  size_t dropped_{0};
  bool stroking_{false};
  size_t column_{0};
  double force_{-1.0};
};

}  // namespace manipulation_manager
