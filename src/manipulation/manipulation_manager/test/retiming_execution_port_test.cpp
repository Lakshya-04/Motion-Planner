#include <memory>

#include <gtest/gtest.h>

#include "manipulation_manager/core/execution/retiming_execution_port.hpp"

namespace manipulation_manager
{
namespace
{

class RecordingPort : public ExecutionPort
{
public:
  ExecutionResult execute(const ExecutionRequest & r) override
  {
    ++calls;
    last = r;
    ExecutionResult ok;
    ok.success = true;
    return ok;
  }
  int calls{0};
  ExecutionRequest last;
};

// Joint-space constant speed at the stroke speed, 0.1 rad/s for a transition.
class FakeRetimer : public motion_planner::TimeParameterizer
{
public:
  bool retime(
    core_types::Trajectory & piece, motion_planner::SegmentRole role,
    const std::vector<motion_planner::JointLimit> &, double stroke, std::string &) override
  {
    roles.push_back(role);
    motion_planner::constantSpeedStamps(piece, stroke > 0.0 ? stroke : 0.1);
    return true;
  }
  std::vector<motion_planner::SegmentRole> roles;
};

ExecutionRequest request(double stroke, bool reversal)
{
  ExecutionRequest r;
  r.stroke_speed = stroke;
  r.trajectory.joint_names = {"a"};
  const std::vector<double> q = reversal ? std::vector<double>{0.0, 0.1, 0.2, 0.1} :
    std::vector<double>{0.0, 0.1, 0.2, 0.3};
  for (double x : q) {
    core_types::TrajectoryPoint p;
    p.positions = {x};
    r.trajectory.points.push_back(p);
  }
  return r;
}

struct Fixture
{
  std::shared_ptr<RecordingPort> inner = std::make_shared<RecordingPort>();
  std::shared_ptr<FakeRetimer> retimer = std::make_shared<FakeRetimer>();
  std::vector<motion_planner::JointLimit> limits{{1.0, 2.0, 5.0}};
  RetimingExecutionPort port{inner, retimer, limits, limits};
};

TEST(RetimingExecutionPortTest, StrokeSpeedMakesASeamElseATransition)
{
  Fixture f;
  EXPECT_TRUE(f.port.execute(request(0.05, false)).success);
  EXPECT_TRUE(f.port.execute(request(0.0, false)).success);
  ASSERT_EQ(f.retimer->roles.size(), 2u);
  EXPECT_EQ(f.retimer->roles[0], motion_planner::SegmentRole::kContactSeam);
  EXPECT_EQ(f.retimer->roles[1], motion_planner::SegmentRole::kTransition);
  EXPECT_GT(f.inner->last.trajectory.points.back().time_from_start_sec, 0.0);
}

TEST(RetimingExecutionPortTest, ReversalIsJoinedIntoOneTrajectoryWithAStop)
{
  Fixture f;
  ASSERT_TRUE(f.port.execute(request(0.0, true)).success);
  EXPECT_EQ(f.port.lastTiming().pieces.size(), 2u);
  EXPECT_EQ(f.inner->last.trajectory.points.size(), 4u);
}

TEST(RetimingExecutionPortTest, SeamOverTheLimitFailsWithoutExecuting)
{
  Fixture f;
  // 0.1 rad per 0.01 s step at stroke 10 -> 10 rad/s on a 1 rad/s joint
  const auto r = f.port.execute(request(10.0, false));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "time_limit_violation");
  EXPECT_EQ(f.inner->calls, 0);
}

TEST(RetimingExecutionPortTest, BadLimitsAreARetimingFailure)
{
  Fixture f;
  RetimingExecutionPort bad(
    f.inner, f.retimer, std::vector<motion_planner::JointLimit>{{1.0, 0.0, 5.0}}, f.limits);
  const auto r = bad.execute(request(0.05, false));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "retiming_failed");
}

}  // namespace
}  // namespace manipulation_manager
