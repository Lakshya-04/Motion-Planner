/**
 * @file mission_fakes.hpp
 * @brief Scripted fake implementations of every mission port, for host tests and for trying the tree without ROS.
 * @details Every fake appends to the shared event log (`MissionCtx::events`) so a test can assert the order of calls
 *          across ports (for example the cancel cleanup order). An operation is scripted per key: a queue of
 *          `Script` entries consumed one per start, the default being "answer RUNNING for `polls_running` polls,
 *          then succeed".
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <deque>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_bt_prototype/mission_types.hpp"

namespace manipulation_bt
{
namespace mission
{

struct Script
{
  int polls_running{1};
  bool fail{false};
  Reason reason{Reason::kNone};
  double value{0.0};
};

/// A queue of scripts per key. next() pops the front; an empty queue gives the default (success).
template<class Key>
class ScriptBook
{
public:
  void add(const Key & k, const Script & s, int times = 1)
  {
    for (int i = 0; i < times; ++i) {queues_[k].push_back(s);}
  }
  void failAlways(const Key & k, Reason why) {always_[k] = Script{0, true, why, 0.0};}
  Script next(const Key & k)
  {
    const auto a = always_.find(k);
    if (a != always_.end()) {return a->second;}
    auto it = queues_.find(k);
    if (it == queues_.end() || it->second.empty()) {return Script{};}
    const Script s = it->second.front();
    it->second.pop_front();
    return s;
  }

private:
  std::map<Key, std::deque<Script>> queues_;
  std::map<Key, Script> always_;
};

/// The in-flight operation of a fake: counts polls down to the scripted answer.
class Op
{
public:
  void begin(const Script & s) {s_ = s; polls_ = 0;}
  OpPoll poll()
  {
    if (polls_++ < s_.polls_running) {return OpPoll{};}
    OpPoll p;
    p.status = s_.fail ? OpStatus::kFailed : OpStatus::kDone;
    p.reason = s_.reason;
    p.value = s_.value;
    return p;
  }

private:
  Script s_;
  int polls_{0};
};

class FakeBase
{
public:
  explicit FakeBase(std::vector<std::string> * log) : log_(log) {}

protected:
  void log(const std::string & e) {if (log_) {log_->push_back(e);}}

private:
  std::vector<std::string> * log_;
};

class FakePerception : public PerceptionPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  ScriptBook<int> script;   ///< key 0
  int starts{0};
  void startScan() override {++starts; op_.begin(script.next(0)); log("perception.scan");}
  OpPoll poll() override {return op_.poll();}
  void cancel() override {log("perception.cancel");}

private:
  Op op_;
};

class FakePlanner : public PlannerPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  CoveragePlan plan{4};
  bool plan_ok{true};
  Reason plan_reason{Reason::kPlanFailed};
  std::vector<StandPlan> stands;                    ///< what selectStands returns
  bool stands_ok{true};
  std::map<int, StandPlan> alternate_for_stand;     ///< key: failed stand id
  std::map<size_t, StandPlan> stand_for_column;     ///< key: column
  std::map<size_t, size_t> split_pieces;            ///< key: column, absent: cannot split
  std::vector<std::pair<size_t, Pose3>> replans;
  std::vector<size_t> wanted_seen;
  int restand_calls{0};

  bool planCoverage(CoveragePlan & out, Reason & why) override
  {
    log("planner.coverage");
    out = plan;
    why = plan_reason;
    return plan_ok;
  }
  bool selectStands(
    const CoveragePlan &, const std::vector<size_t> & wanted, std::vector<StandPlan> & out, Reason & why) override
  {
    log("planner.stands");
    wanted_seen = wanted;
    out = stands;
    why = Reason::kNoStands;
    return stands_ok;
  }
  bool alternateStand(const StandPlan & failed, Reason, StandPlan & out) override
  {
    ++restand_calls;
    const auto it = alternate_for_stand.find(failed.id);
    if (it == alternate_for_stand.end()) {return false;}
    out = it->second;
    alternate_for_stand.erase(it);   // each alternative is offered once
    return true;
  }
  bool standForColumn(size_t column, const StandPlan &, StandPlan & out) override
  {
    ++restand_calls;
    const auto it = stand_for_column.find(column);
    if (it == stand_for_column.end()) {return false;}
    out = it->second;
    stand_for_column.erase(it);
    return true;
  }
  size_t splitColumn(size_t column) override
  {
    const auto it = split_pieces.find(column);
    return it == split_pieces.end() ? 1 : it->second;
  }
  void replanFromPose(size_t column, const Pose3 & p) override
  {
    replans.emplace_back(column, p);
    log("planner.replan:" + std::to_string(column));
  }
};

class FakeNavigation : public NavigationPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  ScriptBook<int> script;   ///< key: stand id
  std::vector<int> starts;
  void startNavigate(const StandPlan & s) override
  {
    starts.push_back(s.id);
    op_.begin(script.next(s.id));
    log("nav.go:" + std::to_string(s.id));
  }
  OpPoll poll() override {return op_.poll();}
  void cancel() override {log("nav.cancel");}

private:
  Op op_;
};

class FakeDock : public DockPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  ScriptBook<int> script;   ///< key: stand id; Script::value is the achieved pose error in m
  std::vector<int> starts;
  void startDock(const StandPlan & s) override
  {
    starts.push_back(s.id);
    op_.begin(script.next(s.id));
    log("dock.go:" + std::to_string(s.id));
  }
  OpPoll poll() override {return op_.poll();}
  void cancel() override {log("dock.cancel");}

private:
  Op op_;
};

class FakeArm : public ArmMotionPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  using Key = std::pair<ArmPhase, size_t>;   ///< phase, column
  ScriptBook<Key> script;
  std::vector<Key> starts;
  std::vector<double> speed_scales;
  double peak_force{-1.0};
  int holds{0};
  int resumes{0};

  void start(ArmPhase phase, size_t column, size_t piece) override
  {
    starts.emplace_back(phase, column);
    op_.begin(script.next({phase, column}));
    log(std::string("arm.start:") + toString(phase) + ":" + std::to_string(column) + ":" + std::to_string(piece));
  }
  OpPoll poll() override {return op_.poll();}
  void cancelTrajectory() override {log("cancel_trajectory");}
  void admittanceOff() override {log("admittance_off");}
  void forceTargetZero() override {log("force_zero");}
  void hold() override {++holds; log("arm.hold");}
  void resume() override {++resumes; log("arm.resume");}
  void setSpeedScale(double s) override {speed_scales.push_back(s);}
  double peakForceN() override {return peak_force;}

  size_t startsOf(ArmPhase phase, size_t column) const
  {
    size_t n = 0;
    for (const auto & k : starts) {n += (k.first == phase && k.second == column) ? 1 : 0;}
    return n;
  }

private:
  Op op_;
};

class FakeVerify : public VerifyPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  ScriptBook<size_t> script;   ///< key: column
  void startVerify(size_t column, size_t piece) override
  {
    op_.begin(script.next(column));
    log("verify:" + std::to_string(column) + ":" + std::to_string(piece));
  }
  OpPoll poll() override {return op_.poll();}
  void cancel() override {log("verify.cancel");}

private:
  Op op_;
};

class FakeSafety : public SafetyPort
{
public:
  SafetyState state;
  SafetyState read() override {return state;}
};

class FakeTeleop : public TeleopPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  TeleopMode current{TeleopMode::kAuto};
  bool confirm{false};
  Pose3 pose{0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
  int handovers{0};
  TeleopMode mode() override {return current;}
  bool takeConfirm() override {const bool c = confirm; confirm = false; return c;}
  Pose3 toolPose() override {return pose;}
  void onHandoverStarted() override {++handovers; confirm = false; log("teleop.handover");}
};

class FakeCalibration : public CalibrationPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  CalibrationRecord rec;
  ScriptBook<int> script;       ///< key 0: the touch sequence
  bool touch_makes_valid{true}; ///< a finished touch writes a valid record
  int touches{0};
  CalibrationRecord record() override {return rec;}
  void startTouch() override {++touches; op_.begin(script.next(0)); log("calibration.touch");}
  OpPoll poll() override
  {
    const OpPoll p = op_.poll();
    if (p.status == OpStatus::kDone && touch_makes_valid) {rec.valid = true;}
    return p;
  }
  void cancelTouch() override {log("calibration.cancel");}

private:
  Op op_;
};

class FakeTool : public ToolPort, public FakeBase
{
public:
  using FakeBase::FakeBase;
  bool is_contact{true};
  int offs{0};
  bool contact() override {return is_contact;}
  void off() override {++offs; log("tool_off");}
  void closeValve() override {log("valve_closed");}
};

class FakeSink : public ReportSink
{
public:
  std::vector<MissionReport> reports;
  std::vector<std::string> states;
  void publish(const MissionReport & r) override {reports.push_back(r);}
  void state(const std::string & s) override {states.push_back(s);}
};

/// All fakes wired into a context.
struct FakeWorld
{
  std::shared_ptr<FakePerception> perception;
  std::shared_ptr<FakePlanner> planner;
  std::shared_ptr<FakeNavigation> navigation;
  std::shared_ptr<FakeDock> dock;
  std::shared_ptr<FakeArm> arm;
  std::shared_ptr<FakeVerify> verify;
  std::shared_ptr<FakeSafety> safety;
  std::shared_ptr<FakeTeleop> teleop;
  std::shared_ptr<FakeCalibration> calibration;
  std::shared_ptr<FakeTool> tool;
  std::shared_ptr<FakeSink> sink;
};

/// Creates the fakes, plugs them into `ctx->ports` and gives the planner `stand_count` stands of
/// `columns_per_stand` columns each (stand ids 0.., columns numbered on).
inline FakeWorld makeFakeWorld(MissionCtx & ctx, size_t stand_count = 1, size_t columns_per_stand = 4)
{
  FakeWorld w;
  auto * log = &ctx.events;
  w.perception = std::make_shared<FakePerception>(log);
  w.planner = std::make_shared<FakePlanner>(log);
  w.navigation = std::make_shared<FakeNavigation>(log);
  w.dock = std::make_shared<FakeDock>(log);
  w.arm = std::make_shared<FakeArm>(log);
  w.verify = std::make_shared<FakeVerify>(log);
  w.safety = std::make_shared<FakeSafety>();
  w.teleop = std::make_shared<FakeTeleop>(log);
  w.calibration = std::make_shared<FakeCalibration>(log);
  w.tool = std::make_shared<FakeTool>(log);
  w.sink = std::make_shared<FakeSink>();
  w.calibration->rec.valid = true;
  size_t next_column = 0;
  for (size_t s = 0; s < stand_count; ++s) {
    StandPlan sp;
    sp.id = static_cast<int>(s);
    sp.pose = Pose2{static_cast<double>(s), 0.0, 0.0};
    for (size_t c = 0; c < columns_per_stand; ++c) {sp.columns.push_back(next_column++);}
    w.planner->stands.push_back(sp);
  }
  w.planner->plan.column_count = next_column;
  ctx.ports.perception = w.perception;
  ctx.ports.planner = w.planner;
  ctx.ports.navigation = w.navigation;
  ctx.ports.dock = w.dock;
  ctx.ports.arm = w.arm;
  ctx.ports.verify = w.verify;
  ctx.ports.safety = w.safety;
  ctx.ports.teleop = w.teleop;
  ctx.ports.calibration = w.calibration;
  ctx.ports.tool = w.tool;
  ctx.ports.sink = w.sink;
  return w;
}

}  // namespace mission
}  // namespace manipulation_bt
