#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/execution/tool_feed_execution_port.hpp"

namespace manipulation_manager
{
namespace
{

class RecordingSink : public ToolWindowSink
{
public:
  void startWindow(double on_s, double end_s) override
  {
    std::lock_guard<std::mutex> l(m);
    windows.emplace_back(on_s, end_s);
    order.push_back('w');
  }
  void trajectoryTime(double t) override
  {
    std::lock_guard<std::mutex> l(m);
    times.push_back(t);
    order.push_back('t');
  }
  void cancel() override
  {
    std::lock_guard<std::mutex> l(m);
    ++cancels;
    order.push_back('c');
  }
  std::mutex m;
  std::vector<std::pair<double, double>> windows;
  std::vector<double> times;
  std::vector<char> order;
  int cancels{0};
};

class SleepingPort : public ExecutionPort
{
public:
  SleepingPort(double seconds, bool ok, bool throws = false)
  : seconds_(seconds), ok_(ok), throws_(throws) {}
  ExecutionResult execute(const ExecutionRequest &) override
  {
    ++calls;
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds_));
    if (throws_) {throw std::runtime_error("boom");}
    ExecutionResult r;
    r.success = ok_;
    return r;
  }
  std::atomic<int> calls{0};

private:
  double seconds_;
  bool ok_, throws_;
};

ExecutionRequest request(double duration_s)
{
  ExecutionRequest r;
  r.trajectory.joint_names = {"a"};
  for (double t : {0.0, duration_s / 2.0, duration_s}) {
    core_types::TrajectoryPoint p;
    p.positions = {t};
    p.time_from_start_sec = t;
    r.trajectory.points.push_back(p);
  }
  return r;
}

}  // namespace

TEST(ToolFeedExecutionPort, ArmedStrokeSendsTheWindowThenAdvancingTimeEndingAtTheEnd)
{
  auto sink = std::make_shared<RecordingSink>();
  auto inner = std::make_shared<SleepingPort>(0.25, true);
  ToolFeedExecutionPort port(inner, sink, 100.0);
  port.arm(true);
  const auto r = port.execute(request(2.0));
  EXPECT_TRUE(r.success);
  EXPECT_EQ(inner->calls, 1);
  ASSERT_EQ(sink->windows.size(), 1U);
  EXPECT_DOUBLE_EQ(sink->windows[0].first, 0.0);
  EXPECT_DOUBLE_EQ(sink->windows[0].second, 2.0);
  EXPECT_EQ(sink->order.front(), 'w');
  ASSERT_GE(sink->times.size(), 5U);          // about 25 ticks at 100 Hz over 0.25 s
  for (size_t i = 1; i < sink->times.size(); ++i) {EXPECT_GE(sink->times[i], sink->times[i - 1]);}
  EXPECT_LT(sink->times[1], 0.5);             // elapsed time, far from the 2 s end while running
  EXPECT_DOUBLE_EQ(sink->times.back(), 2.0);  // success closes the window
  EXPECT_EQ(sink->cancels, 0);
}

TEST(ToolFeedExecutionPort, FailedExecutionCancelsTheTool)
{
  auto sink = std::make_shared<RecordingSink>();
  ToolFeedExecutionPort port(std::make_shared<SleepingPort>(0.05, false), sink);
  port.arm(true);
  EXPECT_FALSE(port.execute(request(1.0)).success);
  EXPECT_EQ(sink->cancels, 1);
  EXPECT_LT(sink->times.back(), 1.0);   // never told the window ended
  EXPECT_EQ(sink->order.back(), 'c');
}

TEST(ToolFeedExecutionPort, ThrowingInnerPortCancelsAndRethrows)
{
  auto sink = std::make_shared<RecordingSink>();
  ToolFeedExecutionPort port(std::make_shared<SleepingPort>(0.02, true, true), sink);
  port.arm(true);
  EXPECT_THROW(port.execute(request(1.0)), std::runtime_error);
  EXPECT_EQ(sink->cancels, 1);
}

TEST(ToolFeedExecutionPort, NotArmedOrEmptyTrajectoryNeverTouchesTheSink)
{
  auto sink = std::make_shared<RecordingSink>();
  auto inner = std::make_shared<SleepingPort>(0.01, true);
  ToolFeedExecutionPort port(inner, sink);
  EXPECT_TRUE(port.execute(request(1.0)).success);   // transition: not armed
  port.arm(true);
  ExecutionRequest empty;
  port.execute(empty);
  EXPECT_EQ(inner->calls, 2);
  EXPECT_TRUE(sink->order.empty());
  port.arm(false);
  port.execute(request(1.0));
  EXPECT_TRUE(sink->order.empty());
}

TEST(ToolFeedExecutionPort, TimeIsClampedToTheWindowEnd)
{
  auto sink = std::make_shared<RecordingSink>();
  ToolFeedExecutionPort port(std::make_shared<SleepingPort>(0.2, true), sink, 100.0);
  port.arm(true);
  port.execute(request(0.05));   // the trajectory is shorter than the execution took
  for (double t : sink->times) {EXPECT_LE(t, 0.05 + 1e-12);}
  EXPECT_DOUBLE_EQ(sink->times.back(), 0.05);
}

}  // namespace manipulation_manager
