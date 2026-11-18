/**
 * @file bt_prototype.hpp
 * @brief BehaviorTree.CPP 4 evaluation prototype of the manager's mission flow over fake segment executors.
 * @details ROS-free. Uses the manager's MissionFsm and MissionLedger unchanged. Nodes: NavigateToStand, SegmentLoop
 *          (control: one child per segment, pause guard at the boundary, typed skip, resume skip), SwitchController
 *          (refuses while a segment is in flight), ExecuteSegment (stateful; halt cancels the executor) and
 *          ReportStatus (always-report). Not wired into the manager. See docs/behavior_tree_evaluation.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "manipulation_manager/core/mission_fsm.hpp"
#include "manipulation_manager/core/mission_ledger.hpp"

namespace manipulation_bt
{

using manipulation_manager::MissionFsm;
using manipulation_manager::MissionLedger;
using manipulation_manager::SkipReason;

struct SegmentPoll
{
  enum class Status { kRunning, kDone, kFailed };
  Status status{Status::kRunning};
  SkipReason reason{SkipReason::kNone};   ///< typed reason when kFailed
};

/// What the real manager does through JtacExecutionAdapter, reduced to three calls.
class ISegmentExecutor
{
public:
  virtual ~ISegmentExecutor() = default;
  virtual void start(size_t segment) = 0;
  virtual SegmentPoll poll() = 0;
  /// Halt cleanup: cancel the trajectory, tool off, admittance off, force target 0.
  virtual void cancel() = 0;
};

struct Report
{
  int count{0};
  std::string outcome;
  size_t done{0};
  size_t skipped{0};
  std::vector<std::pair<size_t, SkipReason>> skips;
  SkipReason work_failure{SkipReason::kNone};
};

/// Shared state of one mission, on the blackboard under "ctx".
struct MissionContext
{
  MissionFsm fsm;
  MissionLedger ledger;
  std::shared_ptr<ISegmentExecutor> executor;
  size_t segment_count{0};
  std::string mission_id{"mission"};

  bool navigation_ok{true};
  bool segment_in_flight{false};
  size_t current_segment{0};
  SkipReason segment_reason{SkipReason::kNone};
  Report report;
  std::vector<std::string> events;   ///< ordered log used by the tests
};

using MissionContextPtr = std::shared_ptr<MissionContext>;

inline MissionContextPtr contextOf(const BT::TreeNode & node)
{
  MissionContextPtr ctx;
  if (!node.config().blackboard || !node.config().blackboard->get("ctx", ctx) || !ctx) {
    throw BT::RuntimeError("blackboard entry 'ctx' (MissionContext) is missing");
  }
  return ctx;
}

class NavigateToStand : public BT::SyncActionNode
{
public:
  NavigateToStand(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    auto ctx = contextOf(*this);
    ctx->events.push_back("navigate");
    if (!ctx->navigation_ok) {
      ctx->report.work_failure = SkipReason::kNavFailed;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class SwitchController : public BT::SyncActionNode
{
public:
  SwitchController(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("mode")};}
  BT::NodeStatus tick() override
  {
    auto ctx = contextOf(*this);
    if (ctx->segment_in_flight) {
      return BT::NodeStatus::FAILURE;   // controllers switch at segment boundaries only
    }
    std::string mode;
    getInput("mode", mode);
    ctx->events.push_back("switch:" + mode);
    return BT::NodeStatus::SUCCESS;
  }
};

class ExecuteSegment : public BT::StatefulActionNode
{
public:
  ExecuteSegment(const std::string & name, const BT::NodeConfig & cfg) : BT::StatefulActionNode(name, cfg) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    auto ctx = contextOf(*this);
    ctx->segment_in_flight = true;
    ctx->events.push_back("start:" + std::to_string(ctx->current_segment));
    ctx->executor->start(ctx->current_segment);
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    auto ctx = contextOf(*this);
    const auto p = ctx->executor->poll();
    switch (p.status) {
      case SegmentPoll::Status::kDone:
        ctx->segment_in_flight = false;
        ctx->events.push_back("done:" + std::to_string(ctx->current_segment));
        return BT::NodeStatus::SUCCESS;
      case SegmentPoll::Status::kFailed:
        ctx->segment_in_flight = false;
        ctx->segment_reason = p.reason;
        ctx->events.push_back("failed:" + std::to_string(ctx->current_segment));
        return BT::NodeStatus::FAILURE;
      case SegmentPoll::Status::kRunning:
        break;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    auto ctx = contextOf(*this);
    ctx->executor->cancel();
    ctx->segment_in_flight = false;
    ctx->events.push_back("cancel:" + std::to_string(ctx->current_segment));
  }
};

/// Runs its single child once per segment. A failing child marks the segment SKIPPED with the typed reason and the
/// loop goes on; a pause requested on the FSM takes effect here, between segments. Returns RUNNING after each
/// segment (one segment per tick at most) and SUCCESS after the last one: skipped segments do not fail the loop.
class SegmentLoop : public BT::ControlNode
{
public:
  SegmentLoop(const std::string & name, const BT::NodeConfig & cfg) : BT::ControlNode(name, cfg) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    auto ctx = contextOf(*this);
    if (children_nodes_.size() != 1) {
      throw BT::LogicError("SegmentLoop needs exactly one child");
    }
    setStatus(BT::NodeStatus::RUNNING);
    if (!begun_) {
      ctx->ledger.begin(ctx->mission_id, ctx->segment_count, resume_);
      begun_ = true;
    }
    while (index_ < ctx->segment_count) {
      if (!in_segment_) {
        if (ctx->fsm.atBoundary()) {
          return BT::NodeStatus::RUNNING;   // PAUSED: wait at the boundary
        }
        if (ctx->ledger.resumeSkips(index_)) {
          ++index_;
          continue;
        }
        ctx->current_segment = index_;
        ctx->segment_reason = SkipReason::kPlanFailed;   // default, as the manager does; the executor refines it
        ctx->ledger.markRunning(index_, 0);
        in_segment_ = true;
      }
      const auto st = children_nodes_[0]->executeTick();
      if (st == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }
      if (st == BT::NodeStatus::SUCCESS) {
        ctx->ledger.markDone(index_, 0);
        ++ctx->report.done;
      } else {
        ctx->ledger.markSkipped(index_, 0, ctx->segment_reason);
        ctx->report.skips.emplace_back(index_, ctx->segment_reason);
        ++ctx->report.skipped;
      }
      resetChild(0);
      in_segment_ = false;
      ++index_;
      return index_ < ctx->segment_count ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    // A halted segment stays RUNNING in the ledger (the manager does the same), so a resume reruns it.
    index_ = 0;
    in_segment_ = false;
    begun_ = false;
    resume_ = true;   // the next run of this node keeps the ledger
    ControlNode::halt();
  }

private:
  size_t index_{0};
  bool in_segment_{false};
  bool begun_{false};
  bool resume_{false};
};

/// Always-report leaf: copies the ledger counts into the report. Wrapped in ForceFailure on the failure branch.
class ReportStatus : public BT::SyncActionNode
{
public:
  ReportStatus(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("outcome", "finished")};}
  BT::NodeStatus tick() override
  {
    auto ctx = contextOf(*this);
    getInput("outcome", ctx->report.outcome);
    ++ctx->report.count;
    ctx->events.push_back("report:" + ctx->report.outcome);
    return BT::NodeStatus::SUCCESS;
  }
};

inline void registerNodes(BT::BehaviorTreeFactory & f)
{
  f.registerNodeType<NavigateToStand>("NavigateToStand");
  f.registerNodeType<SwitchController>("SwitchController");
  f.registerNodeType<ExecuteSegment>("ExecuteSegment");
  f.registerNodeType<SegmentLoop>("SegmentLoop");
  f.registerNodeType<ReportStatus>("ReportStatus");
}

}  // namespace manipulation_bt
