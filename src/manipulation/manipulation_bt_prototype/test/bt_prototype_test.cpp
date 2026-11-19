#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "manipulation_bt_prototype/bt_prototype.hpp"

using namespace manipulation_bt;
using manipulation_manager::LedgerState;
using manipulation_manager::MissionState;

namespace
{

std::string readFile(const char * path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

struct Script
{
  int polls_running{2};   ///< polls that answer RUNNING before the result
  bool fail{false};
  SkipReason reason{SkipReason::kNone};
  bool throws{false};
};

class FakeExecutor : public ISegmentExecutor
{
public:
  std::map<size_t, Script> script;
  std::vector<size_t> starts;
  int cancels{0};

  void start(size_t s) override {current_ = s; polls_ = 0; starts.push_back(s);}
  SegmentPoll poll() override
  {
    const Script sc = script.count(current_) ? script[current_] : Script{};
    if (sc.throws) {throw std::runtime_error("executor exploded");}
    if (polls_++ < sc.polls_running) {return {};}
    SegmentPoll p;
    p.status = sc.fail ? SegmentPoll::Status::kFailed : SegmentPoll::Status::kDone;
    p.reason = sc.reason;
    return p;
  }
  void cancel() override {++cancels;}

private:
  size_t current_{0};
  int polls_{0};
};

struct Fixture
{
  BT::BehaviorTreeFactory factory;
  MissionContextPtr ctx = std::make_shared<MissionContext>();
  std::shared_ptr<FakeExecutor> exec = std::make_shared<FakeExecutor>();
  BT::Tree tree;

  explicit Fixture(size_t segments)
  {
    registerNodes(factory);
    ctx->executor = exec;
    ctx->segment_count = segments;
    ctx->fsm.start();
    auto bb = BT::Blackboard::create();
    bb->set("ctx", ctx);
    tree = factory.createTreeFromText(readFile(BT_PROTOTYPE_TREE_FILE), bb);
  }

  /// Ticks until the tree stops running; `hook(tick)` runs before each tick (commands arrive between ticks).
  template<class Hook>
  BT::NodeStatus run(Hook hook, int max_ticks = 200)
  {
    BT::NodeStatus st = BT::NodeStatus::RUNNING;
    for (int i = 0; i < max_ticks && st == BT::NodeStatus::RUNNING; ++i) {
      hook(i);
      st = tree.tickOnce();
    }
    return st;
  }
  BT::NodeStatus run() {return run([](int) {});}

  size_t count(const std::string & prefix) const
  {
    return std::count_if(
      ctx->events.begin(), ctx->events.end(),
      [&](const std::string & e) {return e.rfind(prefix, 0) == 0;});
  }
};

}  // namespace

TEST(BtPrototype, SuccessPathRunsEverySegmentAndReportsOnce)
{
  Fixture f(3);
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.exec->starts, (std::vector<size_t>{0, 1, 2}));
  EXPECT_EQ(f.exec->cancels, 0);
  EXPECT_EQ(f.ctx->report.count, 1);
  EXPECT_EQ(f.ctx->report.outcome, "finished");
  EXPECT_EQ(f.ctx->report.done, 3u);
  EXPECT_EQ(f.ctx->report.skipped, 0u);
  for (size_t i = 0; i < 3; ++i) {EXPECT_TRUE(f.ctx->ledger.isDone(i));}
  EXPECT_EQ(f.ctx->events.back(), "report:finished");
}

TEST(BtPrototype, FailedSegmentIsSkippedWithItsTypedReasonAndTheRunContinues)
{
  Fixture f(4);
  f.exec->script[1] = {1, true, SkipReason::kDockFailed};
  f.exec->script[3] = {0, true, SkipReason::kContactOverload};
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);   // skips do not fail the mission
  EXPECT_EQ(f.exec->starts, (std::vector<size_t>{0, 1, 2, 3}));
  EXPECT_TRUE(f.ctx->ledger.isDone(0));
  EXPECT_TRUE(f.ctx->ledger.isDone(2));
  EXPECT_EQ(f.ctx->ledger.get(1).state, LedgerState::kSkipped);
  EXPECT_EQ(f.ctx->ledger.get(1).reason, SkipReason::kDockFailed);
  EXPECT_EQ(f.ctx->ledger.get(3).state, LedgerState::kSkipped);
  EXPECT_EQ(f.ctx->ledger.get(3).reason, SkipReason::kContactOverload);
  ASSERT_EQ(f.ctx->report.skips.size(), 2u);
  EXPECT_EQ(f.ctx->report.skips[0], std::make_pair(size_t{1}, SkipReason::kDockFailed));
  EXPECT_EQ(f.ctx->report.done, 2u);
  EXPECT_EQ(f.ctx->report.count, 1);
  // a resume must rerun the dock failure but not the contact overload
  EXPECT_FALSE(f.ctx->ledger.resumeSkips(1));
  EXPECT_TRUE(f.ctx->ledger.resumeSkips(3));
}

TEST(BtPrototype, UntypedFailureIsStillSkipped)
{
  Fixture f(1);
  f.exec->script[0] = {0, true, SkipReason::kNone};   // the executor gave no reason
  // ExecuteSegment copies the executor's reason; kNone is recorded as such, so adapters must always type it
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.ctx->ledger.get(0).state, LedgerState::kSkipped);
}

TEST(BtPrototype, HaltMidSegmentCancelsTheExecutorAndLeavesTheSegmentRunning)
{
  Fixture f(3);
  // run until segment 1 is in flight
  f.run([&](int) {}, 1);
  while (!f.ctx->segment_in_flight || f.ctx->current_segment != 1) {
    ASSERT_EQ(f.tree.tickOnce(), BT::NodeStatus::RUNNING);
  }
  f.tree.haltTree();
  EXPECT_EQ(f.exec->cancels, 1);
  EXPECT_FALSE(f.ctx->segment_in_flight);
  EXPECT_EQ(f.ctx->ledger.get(1).state, LedgerState::kRunning);
  EXPECT_TRUE(f.ctx->ledger.isDone(0));
  EXPECT_EQ(f.ctx->report.count, 0);   // a halt is not a report: the host reports the cancel
  EXPECT_EQ(f.count("cancel:1"), 1u);

  // ticking again resumes from the ledger: segment 0 is not rerun, 1 and 2 are
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.exec->starts, (std::vector<size_t>{0, 1, 1, 2}));
  EXPECT_EQ(f.ctx->report.outcome, "finished");
}

TEST(BtPrototype, FailureInsideTheWorkStillEndsInReportStatus)
{
  Fixture f(3);
  f.ctx->navigation_ok = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::FAILURE);   // ForceFailure: the tree still fails
  EXPECT_EQ(f.ctx->report.count, 1);
  EXPECT_EQ(f.ctx->report.outcome, "aborted");
  EXPECT_EQ(f.ctx->report.work_failure, SkipReason::kNavFailed);
  EXPECT_TRUE(f.exec->starts.empty());
  EXPECT_EQ(f.ctx->events.back(), "report:aborted");
}

TEST(BtPrototype, ExceptionInANodeBypassesTheFallbackAndTheReport)
{
  // Known limit of the convention: Fallback reacts to FAILURE, not to an exception. Nodes must catch their own.
  Fixture f(2);
  f.exec->script[0].throws = true;
  EXPECT_ANY_THROW(f.run());
  EXPECT_EQ(f.ctx->report.count, 0);
}

TEST(BtPrototype, PauseTakesEffectAtTheSegmentBoundary)
{
  Fixture f(3);
  bool paused_seen = false;
  bool resumed = false;
  int paused_ticks = 0;
  const auto st = f.run(
    [&](int) {
      if (f.ctx->segment_in_flight && f.ctx->current_segment == 0 && !paused_seen && f.ctx->fsm.state() ==
      MissionState::kActive && !f.ctx->fsm.pauseRequested())
      {
        EXPECT_TRUE(f.ctx->fsm.requestPause());   // mid-segment 0
      }
      if (f.ctx->fsm.state() == MissionState::kPaused) {
        paused_seen = true;
        if (++paused_ticks == 10 && !resumed) {
          resumed = true;
          EXPECT_EQ(f.count("start:1"), 0u);   // 10 ticks paused: segment 1 did not start
          EXPECT_TRUE(f.ctx->fsm.resume());
        }
      }
    });
  EXPECT_EQ(st, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(paused_seen);
  EXPECT_TRUE(resumed);
  EXPECT_EQ(f.exec->starts, (std::vector<size_t>{0, 1, 2}));
  EXPECT_EQ(f.exec->cancels, 0);   // pausing never cancels the segment in flight
  EXPECT_EQ(f.ctx->report.done, 3u);
}

TEST(BtPrototype, ControllerSwitchesOnlyBetweenSegments)
{
  Fixture f(3);
  f.exec->script[1] = {1, true, SkipReason::kPlanFailed};
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  bool in_flight = false;
  int switches = 0;
  for (const auto & e : f.ctx->events) {
    if (e.rfind("start:", 0) == 0) {in_flight = true;}
    if (e.rfind("done:", 0) == 0 || e.rfind("failed:", 0) == 0) {in_flight = false;}
    if (e.rfind("switch:", 0) == 0) {
      EXPECT_FALSE(in_flight) << e;
      ++switches;
    }
  }
  EXPECT_EQ(switches, 3);
}

TEST(BtPrototype, SwitchControllerRefusesWhileASegmentIsInFlight)
{
  BT::BehaviorTreeFactory factory;
  registerNodes(factory);
  auto ctx = std::make_shared<MissionContext>();
  ctx->segment_in_flight = true;
  auto bb = BT::Blackboard::create();
  bb->set("ctx", ctx);
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="T"><SwitchController mode="contact"/></BehaviorTree></root>)", bb);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(ctx->events.empty());
}

namespace
{
template<const char * kPort1, const char * kPort2 = nullptr>
class StubDecorator : public BT::DecoratorNode
{
public:
  StubDecorator(const std::string & n, const BT::NodeConfig & c) : BT::DecoratorNode(n, c) {}
  static BT::PortsList providedPorts()
  {
    BT::PortsList p{BT::OutputPort<std::string>(kPort1)};
    if (kPort2) {p.insert(BT::InputPort<std::string>(kPort2));}
    return p;
  }
  BT::NodeStatus tick() override {return child_node_->executeTick();}
};
constexpr char kStand[] = "stand";
constexpr char kColumn[] = "column";

class StubLeaf : public BT::SyncActionNode
{
public:
  StubLeaf(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("stand"), BT::InputPort<std::string>("column"),
      BT::InputPort<std::string>("reason")};
  }
  BT::NodeStatus tick() override {return BT::NodeStatus::SUCCESS;}
};
}  // namespace

TEST(BtPrototype, TargetTreeInDocsIsWellFormedAndEveryNodeIsDeclared)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ReportStatus>("ReportStatus");
  factory.registerNodeType<SwitchController>("SwitchController");
  for (const char * n : {"LoadPlan", "PlanStands", "StowForDrive", "NavigateToStand", "PrimeScene", "DockColumn",
      "PressStrokeRetract", "ContactOverloaded", "RecoverArm", "SafeState", "SetSkipReason"})
  {
    factory.registerNodeType<StubLeaf>(n);
  }
  factory.registerNodeType<StubDecorator<kStand>>("ForEachStand");
  factory.registerNodeType<StubDecorator<kColumn, kStand>>("ForEachColumn");
  const std::string xml = readFile(BT_TARGET_TREE_FILE);
  if (xml.empty()) {
    GTEST_SKIP() << "docs tree not present in this checkout (image builds copy only src/): "
                 << BT_TARGET_TREE_FILE;
  }
  factory.registerBehaviorTreeFromText(xml);
  auto bb = BT::Blackboard::create();
  bb->set("ctx", std::make_shared<MissionContext>());
  EXPECT_NO_THROW(factory.createTree("CoverageMission", bb));
}
