#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "motion_planner/core/planners/ik_then_plan_planner.hpp"

namespace motion_planner
{
namespace
{

class FakeIk : public IkPort
{
public:
  using Fn = std::function<IkResult(const IkRequest &)>;
  explicit FakeIk(Fn fn)
  : fn_(std::move(fn)) {}
  IkResult ik(const IkRequest & request) override
  {
    ++calls;
    requests.push_back(request);
    return fn_(request);
  }
  int calls{0};
  std::vector<IkRequest> requests;

private:
  Fn fn_;
};

class FakeCollision : public CollisionPort
{
public:
  using Fn = std::function<CollisionResult(const CollisionRequest &)>;
  explicit FakeCollision(Fn fn)
  : fn_(std::move(fn)) {}
  CollisionResult collision(const CollisionRequest & request) override
  {
    ++calls;
    checked.push_back(request.joint_values);
    return fn_(request);
  }
  int calls{0};
  std::vector<std::vector<double>> checked;

private:
  Fn fn_;
};

struct InnerLog
{
  int plan_calls{0};
  PlanningRequest last;
};

class FakeInner : public Planner
{
public:
  FakeInner(std::shared_ptr<InnerLog> log, PlanningResult result)
  : log_(std::move(log)), result_(std::move(result)) {}
  std::string name() const override {return "fake_inner";}
  PlanningResult plan(const PlanningRequest & request) override
  {
    ++log_->plan_calls;
    log_->last = request;
    return result_;
  }
  IkResult ik(const IkRequest &) override {return {};}
  CollisionResult collision(const CollisionRequest &) override {return {};}

private:
  std::shared_ptr<InnerLog> log_;
  PlanningResult result_;
};

IkResult solved(std::vector<double> q)
{
  IkResult r;
  r.success = true;
  r.joint_solution = std::move(q);
  return r;
}

IkResult unsolved()
{
  IkResult r;
  r.failure_reason_code = "no_solution";
  r.message = "unreachable";
  return r;
}

CollisionResult freeState()
{
  CollisionResult r;
  r.success = true;
  r.collision_free = true;
  return r;
}

CollisionResult hit()
{
  CollisionResult r;
  r.success = true;
  r.collision_free = false;
  return r;
}

PlanningResult planned()
{
  PlanningResult r;
  r.success = true;
  r.status = "success";
  r.path_fraction = 1.0;
  return r;
}

PlanningRequest poseRequest()
{
  PlanningRequest r;
  r.planning_group = "arm";
  r.frame_id = "base";
  r.waypoint_indices = {0};
  core_types::Pose p;
  p.x = 0.4;
  p.z = 0.5;
  r.target_poses = {p};
  r.start_joint_state = {0.0, 0.0, 0.0};
  return r;
}

struct Rig
{
  std::shared_ptr<InnerLog> log{std::make_shared<InnerLog>()};
  std::shared_ptr<FakeIk> ik;
  std::shared_ptr<FakeCollision> coll;
  std::unique_ptr<IkThenPlanPlanner> planner;

  Rig(FakeIk::Fn ik_fn, FakeCollision::Fn coll_fn, IkThenPlanConfig cfg, PlanningResult inner = planned())
  {
    ik = std::make_shared<FakeIk>(std::move(ik_fn));
    if (coll_fn) {coll = std::make_shared<FakeCollision>(std::move(coll_fn));}
    planner = std::make_unique<IkThenPlanPlanner>(
      std::make_unique<FakeInner>(log, std::move(inner)), ik, coll, std::move(cfg));
  }
};

TEST(IkThenPlan, JointGoalPassesThroughUntouched)
{
  Rig rig([](const IkRequest &) {return solved({9, 9, 9});}, [](const CollisionRequest &) {return freeState();}, {});
  PlanningRequest req;
  req.start_joint_state = {0, 0, 0};
  req.target_joint_state = {0.1, 0.2, 0.3};
  const auto r = rig.planner->plan(req);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(rig.ik->calls, 0);
  EXPECT_EQ(rig.coll->calls, 0);
  EXPECT_EQ(rig.log->plan_calls, 1);
  EXPECT_EQ(rig.log->last.target_joint_state, req.target_joint_state);
}

TEST(IkThenPlan, PoseGoalPlansToTheIkSolution)
{
  const std::vector<double> q{0.3, -0.2, 0.9};
  Rig rig([&](const IkRequest &) {return solved(q);}, [](const CollisionRequest &) {return freeState();}, {});
  const auto r = rig.planner->plan(poseRequest());
  ASSERT_TRUE(r.success);
  EXPECT_EQ(rig.log->plan_calls, 1);
  EXPECT_EQ(rig.log->last.target_joint_state, q);
  EXPECT_TRUE(rig.log->last.target_poses.empty());
  EXPECT_EQ(rig.log->last.start_joint_state, poseRequest().start_joint_state);
  ASSERT_EQ(rig.ik->requests.size(), 1u);
  EXPECT_EQ(rig.ik->requests[0].joint_seed, poseRequest().start_joint_state);
  ASSERT_EQ(rig.ik->requests[0].target_pose.size(), 7u);
  EXPECT_DOUBLE_EQ(rig.ik->requests[0].target_pose[0], 0.4);
}

TEST(IkThenPlan, IkFailureNeverReachesTheInnerPlanner)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 2;
  Rig rig([](const IkRequest &) {return unsolved();}, nullptr, cfg);
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "ik_failed");
  EXPECT_EQ(r.message.rfind("ik stage:", 0), 0u);
  EXPECT_EQ(rig.ik->calls, 3);
  EXPECT_EQ(rig.log->plan_calls, 0);
}

TEST(IkThenPlan, OutOfLimitsSolutionIsSkippedForTheNextSeed)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 1;
  cfg.home_pose = {0.5, 0.5, 0.5};
  cfg.lower_limits = {-1, -1, -1};
  cfg.upper_limits = {1, 1, 1};
  Rig rig(
    [](const IkRequest & r) {return r.joint_seed[0] == 0.5 ? solved({0.2, 0.2, 0.2}) : solved({3.0, 0, 0});},
    nullptr, cfg);
  const auto r = rig.planner->plan(poseRequest());
  ASSERT_TRUE(r.success);
  EXPECT_EQ(rig.ik->calls, 2);
  EXPECT_EQ(rig.log->last.target_joint_state, (std::vector<double>{0.2, 0.2, 0.2}));
}

TEST(IkThenPlan, AllSolutionsOutOfLimitsIsIkFailed)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 1;
  cfg.lower_limits = {-1, -1, -1};
  cfg.upper_limits = {1, 1, 1};
  Rig rig([](const IkRequest &) {return solved({3.0, 0, 0});}, nullptr, cfg);
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_EQ(r.failure_reason_code, "ik_failed");
  EXPECT_EQ(rig.log->plan_calls, 0);
}

TEST(IkThenPlan, GoalInCollisionTriesNextSeedThenFails)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 2;
  int n = 0;
  Rig rig(
    [&](const IkRequest &) {return solved({0.1 * ++n, 0, 0});},
    [](const CollisionRequest &) {return hit();}, cfg);
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_EQ(r.failure_reason_code, "ik_goal_in_collision");
  EXPECT_EQ(r.message.rfind("ik stage:", 0), 0u);
  EXPECT_EQ(rig.ik->calls, 3);
  EXPECT_EQ(rig.coll->calls, 3);
  EXPECT_EQ(rig.log->plan_calls, 0);
}

TEST(IkThenPlan, SecondSeedOutOfCollisionIsUsed)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 2;
  int n = 0;
  Rig rig(
    [&](const IkRequest &) {return solved({1.0 * ++n, 0, 0});},
    [](const CollisionRequest & c) {return c.joint_values[0] == 1.0 ? hit() : freeState();}, cfg);
  const auto r = rig.planner->plan(poseRequest());
  ASSERT_TRUE(r.success);
  EXPECT_EQ(rig.log->last.target_joint_state[0], 2.0);
  EXPECT_EQ(rig.ik->calls, 2);
}

TEST(IkThenPlan, InnerFailureIsPassedThroughWithItsCode)
{
  PlanningResult bad;
  bad.failure_reason_code = "no_path";
  bad.failure_stage = FailureStage::kPlanning;
  bad.message = "ran out of iterations";
  Rig rig([](const IkRequest &) {return solved({0.1, 0.1, 0.1});}, nullptr, {}, bad);
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "no_path");
  EXPECT_EQ(r.failure_stage, FailureStage::kPlanning);
  EXPECT_EQ(r.message, "plan stage: ran out of iterations");
}

TEST(IkThenPlan, ToleranceNeedsAnIkPortThatTakesIt)
{
  IkThenPlanConfig cfg;
  cfg.orientation_tolerance_rad = 0.05;
  Rig rig([](const IkRequest &) {return solved({0.1, 0.1, 0.1});}, nullptr, cfg);
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_EQ(r.failure_reason_code, "unsupported_constraint");
  EXPECT_EQ(rig.ik->calls, 0);
  EXPECT_EQ(rig.log->plan_calls, 0);
}

TEST(IkThenPlan, ToleranceIsCarriedToTheIkPort)
{
  IkThenPlanConfig cfg;
  cfg.position_tolerance_m = 0.002;
  cfg.orientation_tolerance_rad = 0.05;
  cfg.ik_honors_tolerance = true;
  Rig rig([](const IkRequest &) {return solved({0.1, 0.1, 0.1});}, nullptr, cfg);
  ASSERT_TRUE(rig.planner->plan(poseRequest()).success);
  EXPECT_DOUBLE_EQ(rig.ik->requests[0].position_tolerance, 0.002);
  EXPECT_DOUBLE_EQ(rig.ik->requests[0].orientation_tolerance, 0.05);
}

TEST(IkThenPlan, SeedsAreDeterministicAndOrdered)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 4;
  cfg.home_pose = {0.7, 0.7, 0.7};
  Rig a([](const IkRequest &) {return unsolved();}, nullptr, cfg);
  Rig b([](const IkRequest &) {return unsolved();}, nullptr, cfg);
  const std::vector<double> start{0.0, 0.1, 0.2};
  const auto sa = a.planner->seedsFor(start);
  const auto sb = b.planner->seedsFor(start);
  ASSERT_EQ(sa.size(), 5u);
  EXPECT_EQ(sa, sb);
  EXPECT_EQ(sa[0], start);
  EXPECT_EQ(sa[1], cfg.home_pose);
  EXPECT_NE(sa[2], start);
  EXPECT_NE(sa[2], sa[3]);
}

TEST(IkThenPlan, SeedsReachTheIkPortInThatOrder)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 3;
  Rig rig([](const IkRequest &) {return unsolved();}, nullptr, cfg);
  rig.planner->plan(poseRequest());
  const auto expected = rig.planner->seedsFor(poseRequest().start_joint_state);
  ASSERT_EQ(rig.ik->requests.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(rig.ik->requests[i].joint_seed, expected[i]);
  }
}

TEST(IkThenPlan, CancelStopsBeforeAnotherSeed)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 5;
  int ik_seen = 0;
  Rig* rig_ptr = nullptr;
  cfg.cancel = [&]() {return rig_ptr && rig_ptr->ik->calls >= 1;};
  Rig rig([&](const IkRequest &) {++ik_seen; return unsolved();}, nullptr, cfg);
  rig_ptr = &rig;
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_EQ(r.failure_reason_code, "cancelled");
  EXPECT_EQ(rig.ik->calls, 1);
  EXPECT_EQ(rig.log->plan_calls, 0);
}

TEST(IkThenPlan, TimeBudgetStopsTheSeedLoop)
{
  IkThenPlanConfig cfg;
  cfg.max_ik_seeds = 5;
  cfg.time_budget_s = 1.0;
  double clock = 0.0;
  cfg.now_s = [&]() {return clock;};
  Rig rig([&](const IkRequest &) {clock += 0.6; return unsolved();}, nullptr, cfg);
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_EQ(r.failure_reason_code, "ik_timeout");
  EXPECT_EQ(rig.ik->calls, 2);
  EXPECT_EQ(rig.log->plan_calls, 0);
}

TEST(IkThenPlan, MoreThanOnePoseIsRejected)
{
  Rig rig([](const IkRequest &) {return solved({0, 0, 0});}, nullptr, {});
  auto req = poseRequest();
  req.target_poses.push_back(req.target_poses[0]);
  const auto r = rig.planner->plan(req);
  EXPECT_EQ(r.failure_reason_code, "invalid_request");
  EXPECT_EQ(rig.ik->calls, 0);
}

TEST(IkThenPlan, FailedCollisionCheckIsNotTreatedAsFree)
{
  Rig rig(
    [](const IkRequest &) {return solved({0, 0, 0});},
    [](const CollisionRequest &) {CollisionResult r; r.message = "no scene"; return r;}, {});
  const auto r = rig.planner->plan(poseRequest());
  EXPECT_EQ(r.failure_reason_code, "collision_check_failed");
  EXPECT_EQ(rig.log->plan_calls, 0);
}

}  // namespace
}  // namespace motion_planner
