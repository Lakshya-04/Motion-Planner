/**
 * @file tool_feed_execution_port.hpp
 * @brief ExecutionPort decorator that feeds the tool actuation executor a window and the trajectory time.
 * @details ROS-free. While armed, a trajectory handed to `execute` is a tool window: the decorator
 *          tells the sink the window (0 to the trajectory's last `time_from_start_sec`), then, on its own
 *          thread at `rate_hz`, the trajectory time since the inner execution started (wall clock,
 *          clamped to the window end). The inner `execute` blocks as before.
 *
 *          - Success: the final trajectory time (the window end) is sent, so the executor closes the tool
 *            and finishes.
 *          - Failure or cancel: `sink.cancel()`, which drives the tool off.
 *          - Not armed, or an empty trajectory: the inner port is called untouched and the sink is not.
 *
 *          Limits: the trajectory time is the elapsed wall time, an estimate of how far the controller
 *          is along the trajectory (it ignores a controller that stalls or runs slow, and the sim's
 *          real-time factor). Pauses are column-boundary only today, so `paused` is never raised here.
 *          The tool executor that consumes this is `tool_actuation_node`; the sink that talks to it
 *          over ROS is in the node.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "manipulation_manager/core/execution/execution_port.hpp"

namespace manipulation_manager
{

/// Where the window and the trajectory time go (ROS topics in the node, a fake in tests).
class ToolWindowSink
{
public:
  virtual ~ToolWindowSink() = default;
  virtual void startWindow(double on_s, double end_s) = 0;
  virtual void trajectoryTime(double traj_time_s) = 0;
  virtual void cancel() = 0;
};

class ToolFeedExecutionPort : public ExecutionPort
{
public:
  ToolFeedExecutionPort(std::shared_ptr<ExecutionPort> inner, std::shared_ptr<ToolWindowSink> sink, double rate_hz = 50.0)
  : inner_(std::move(inner)), sink_(std::move(sink)), period_s_(1.0 / std::max(1.0, rate_hz)) {}

  /// While armed, every non-empty trajectory is a tool window. Set around the stroke segment.
  void arm(bool armed) {armed_ = armed;}
  bool armed() const {return armed_;}

  ExecutionResult execute(const ExecutionRequest & request) override
  {
    if (!armed_ || request.trajectory.points.empty()) {
      return inner_->execute(request);
    }
    const double end_s = request.trajectory.points.back().time_from_start_sec;
    const auto t0 = std::chrono::steady_clock::now();
    sink_->startWindow(0.0, end_s);

    std::mutex m;
    std::condition_variable cv;
    bool done = false;
    std::thread feeder([&]() {
        std::unique_lock<std::mutex> lock(m);
        while (!done) {
          const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
          sink_->trajectoryTime(std::min(t, end_s));
          cv.wait_for(lock, std::chrono::duration<double>(period_s_), [&]() {return done;});
        }
      });
    ExecutionResult result;
    try {
      result = inner_->execute(request);
    } catch (...) {
      stop(m, cv, done, feeder);
      sink_->cancel();
      throw;
    }
    stop(m, cv, done, feeder);
    if (result.success) {
      sink_->trajectoryTime(end_s);
    } else {
      sink_->cancel();
    }
    return result;
  }

private:
  static void stop(std::mutex & m, std::condition_variable & cv, bool & done, std::thread & feeder)
  {
    {
      std::lock_guard<std::mutex> lock(m);
      done = true;
    }
    cv.notify_all();
    feeder.join();
  }

  std::shared_ptr<ExecutionPort> inner_;
  std::shared_ptr<ToolWindowSink> sink_;
  double period_s_;
  std::atomic<bool> armed_{false};
};

}  // namespace manipulation_manager
