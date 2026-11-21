/**
 * @file mission_nodes.hpp
 * @brief BehaviorTree.CPP 4 node implementations of the full mission tree (trees/mission.xml), over the abstract ports
 *        of mission_types.hpp.
 * @details ROS-free. Leaves call ports; control and decorator nodes carry the policy (per stand and per column loops,
 *          pause, safety, teleop handover, always-report). Every node gets the shared `MissionCtx` when its type is
 *          registered (`registerMissionNodes`). Cleanup contract: a leaf that drives the arm cleans up in the order
 *          cancel trajectory, tool off, admittance off, force target zero, valve closed when it is halted and when it
 *          fails (`MissionCtx::safeState`). Catalogue and mapping to the manager: docs/behavior_trees/mission.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "manipulation_bt_prototype/mission_types.hpp"

namespace manipulation_bt
{
namespace mission
{

// =============================================================================== leaves over async ports

/// A leaf that starts a port operation and polls it. Success, failure and halt are logged in `MissionCtx::events` as
/// start:<tag>, done:<tag>, failed:<tag>:<REASON>, halt:<tag>. A failure without a reason becomes `fallback_reason`.
class PortAction : public BT::StatefulActionNode
{
public:
  PortAction(
    const std::string & name, const BT::NodeConfig & cfg, MissionCtxPtr ctx, std::string tag, std::string state,
    Reason fallback_reason)
  : BT::StatefulActionNode(name, cfg), ctx_(std::move(ctx)), tag_(std::move(tag)), state_(std::move(state)),
    fallback_(fallback_reason) {}

  BT::NodeStatus onStart() override
  {
    ctx_->log("start:" + tag_);
    if (!state_.empty()) {ctx_->setState(state_);}
    begin();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    OpPoll p = poll();
    if (p.status == OpStatus::kRunning) {return BT::NodeStatus::RUNNING;}
    finished(p);
    if (p.status == OpStatus::kDone) {
      ctx_->log("done:" + tag_);
      return BT::NodeStatus::SUCCESS;
    }
    ctx_->reason = p.reason == Reason::kNone ? fallback_ : p.reason;
    ctx_->log("failed:" + tag_ + ":" + toString(ctx_->reason));
    cleanup();
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    ctx_->log("halt:" + tag_);
    cleanup();
  }

protected:
  virtual void begin() = 0;
  virtual OpPoll poll() = 0;
  /// Called when halted and when the operation failed (not on success).
  virtual void cleanup() = 0;
  /// Called on any end of the operation before the status is decided; may turn a Done into a Failed.
  virtual void finished(OpPoll &) {}

  MissionCtxPtr ctx_;

private:
  std::string tag_;
  std::string state_;
  Reason fallback_;
};

class PerceiveSurface : public PortAction
{
public:
  PerceiveSurface(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : PortAction(n, c, std::move(x), "perceive", "perceiving", Reason::kPerceptionFailed) {}
  static BT::PortsList providedPorts() {return {};}
  void begin() override {ctx_->ports.perception->startScan();}
  OpPoll poll() override {return ctx_->ports.perception->poll();}
  void cleanup() override {ctx_->ports.perception->cancel();}
};

class NavigateToStand : public PortAction
{
public:
  NavigateToStand(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : PortAction(n, c, std::move(x), "navigate", "navigating", Reason::kNavFailed) {}
  static BT::PortsList providedPorts() {return {};}
  void begin() override {ctx_->ports.navigation->startNavigate(ctx_->stands.at(ctx_->stand));}
  OpPoll poll() override {return ctx_->ports.navigation->poll();}
  void cleanup() override {ctx_->ports.navigation->cancel();}
};

/// Wall docking, then the stand-pose check: the achieved pose error must be within the stand's reachability margin
/// before any arm motion is allowed (STAND_CHECK_FAILED otherwise).
class DockBase : public PortAction
{
public:
  DockBase(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : PortAction(n, c, std::move(x), "dock", "docking", Reason::kDockFailed) {}
  static BT::PortsList providedPorts() {return {};}
  void begin() override {ctx_->ports.dock->startDock(ctx_->stands.at(ctx_->stand));}
  OpPoll poll() override {return ctx_->ports.dock->poll();}
  void cleanup() override {ctx_->ports.dock->cancel();}
  void finished(OpPoll & p) override
  {
    if (p.status == OpStatus::kDone && p.value > ctx_->stands.at(ctx_->stand).margin_m) {
      p.status = OpStatus::kFailed;
      p.reason = Reason::kStandCheckFailed;
    }
  }
};

/// Base of the arm leaves: approach (ArmDock), press, stroke, retract, stow, recover.
class ArmLeaf : public PortAction
{
public:
  ArmLeaf(
    const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x, ArmPhase phase, std::string tag,
    std::string state)
  : PortAction(n, c, std::move(x), std::move(tag), std::move(state), Reason::kPlanFailed), phase_(phase) {}
  static BT::PortsList providedPorts() {return {};}
  void begin() override {ctx_->ports.arm->start(phase_, ctx_->column, ctx_->piece);}
  OpPoll poll() override {return ctx_->ports.arm->poll();}
  void cleanup() override {ctx_->safeState();}
  void finished(OpPoll &) override
  {
    ctx_->report.peak_force_n = std::max(ctx_->report.peak_force_n, ctx_->ports.arm->peakForceN());
  }

protected:
  ArmPhase phase_;
};

class ArmDock : public ArmLeaf
{
public:
  ArmDock(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : ArmLeaf(n, c, std::move(x), ArmPhase::kApproach, "approach", "docking_arm") {}
};

/// Skipped (SUCCESS) for a non-contact tool: there is nothing to press.
class ContactPress : public ArmLeaf
{
public:
  ContactPress(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : ArmLeaf(n, c, std::move(x), ArmPhase::kPress, "press", "pressing") {}
  BT::NodeStatus onStart() override
  {
    if (!ctx_->ports.tool->contact()) {
      ctx_->log("skip:press");
      return BT::NodeStatus::SUCCESS;
    }
    return ArmLeaf::onStart();
  }
};

class StrokeColumn : public ArmLeaf
{
public:
  StrokeColumn(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : ArmLeaf(n, c, std::move(x), ArmPhase::kStroke, "stroke", "stroking") {}
};

class Retract : public ArmLeaf
{
public:
  Retract(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : ArmLeaf(n, c, std::move(x), ArmPhase::kRetract, "retract", "retracting") {}
};

/// Not stowed after a safety timeout: the stop is still in force and the stow would move the arm.
class StowArm : public ArmLeaf
{
public:
  StowArm(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : ArmLeaf(n, c, std::move(x), ArmPhase::kStow, "stow", "stowing") {}
  BT::NodeStatus onStart() override
  {
    if (ctx_->abort_reason == Reason::kSafetyTimeout) {
      ctx_->log("skip:stow:safety");
      return BT::NodeStatus::FAILURE;
    }
    return ArmLeaf::onStart();
  }
};

/// Frees a stuck arm between retries. The mission FSM is in RECOVERY for as long as it runs.
class RecoverArm : public ArmLeaf
{
public:
  RecoverArm(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : ArmLeaf(n, c, std::move(x), ArmPhase::kRecover, "recover", "recovering") {}
  void begin() override
  {
    ctx_->fsm.enterRecovery();
    ++ctx_->report.retries;
    ArmLeaf::begin();
  }
  void finished(OpPoll & p) override
  {
    ctx_->fsm.leaveRecovery();
    ArmLeaf::finished(p);
  }
  void cleanup() override
  {
    ctx_->fsm.leaveRecovery();
    ArmLeaf::cleanup();
  }
};

class VerifyColumn : public PortAction
{
public:
  VerifyColumn(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : PortAction(n, c, std::move(x), "verify", "verifying", Reason::kVerifyFailed) {}
  static BT::PortsList providedPorts() {return {};}
  void begin() override {ctx_->ports.verify->startVerify(ctx_->column, ctx_->piece);}
  OpPoll poll() override {return ctx_->ports.verify->poll();}
  void cleanup() override {ctx_->ports.verify->cancel();}
};

/// Touch sequence of the calibration stage. A touch that ends with an invalid record is a failure.
class TouchCalibrate : public PortAction
{
public:
  TouchCalibrate(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : PortAction(n, c, std::move(x), "touch_calibrate", "calibrating", Reason::kCalibrationInvalid) {}
  static BT::PortsList providedPorts() {return {};}
  void begin() override {ctx_->ports.calibration->startTouch();}
  OpPoll poll() override {return ctx_->ports.calibration->poll();}
  void cleanup() override
  {
    ctx_->ports.calibration->cancelTouch();
    ctx_->safeState();
  }
  void finished(OpPoll & p) override
  {
    if (p.status == OpStatus::kDone && !ctx_->ports.calibration->record().valid) {
      p.status = OpStatus::kFailed;
      p.reason = Reason::kCalibrationInvalid;
    }
  }
};

// =============================================================================== synchronous leaves

class StartMission : public BT::SyncActionNode
{
public:
  StartMission(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    c.fsm.start();
    c.report = MissionReport{};
    c.report_sent = false;
    c.abort_requested = false;
    c.abort_reason = Reason::kNone;
    c.reason = Reason::kNone;
    c.deferred.clear();
    c.stands.clear();
    c.log("start:mission");
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

/// Plans the coverage and opens the ledger (`resume` keeps it, as ExecuteCoverage.resume).
class PlanCoverage : public BT::SyncActionNode
{
public:
  PlanCoverage(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    c.setState("planning_coverage");
    Reason why = Reason::kPlanFailed;
    if (!c.ports.planner->planCoverage(c.plan, why) || c.plan.column_count == 0) {
      c.reason = why == Reason::kNone ? Reason::kPlanFailed : why;
      c.log(std::string("failed:plan_coverage:") + toString(c.reason));
      return BT::NodeStatus::FAILURE;
    }
    const bool kept = c.ledger.begin(c.mission_id, c.plan.column_count, c.resume);
    c.log(std::string("plan:coverage:") + std::to_string(c.plan.column_count) + (kept ? ":resume" : ":fresh"));
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

class SelectStands : public BT::SyncActionNode
{
public:
  SelectStands(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    c.setState("planning_stands");
    Reason why = Reason::kNoStands;
    c.stands.clear();
    if (!c.ports.planner->selectStands(c.plan, c.wanted_columns, c.stands, why) || c.stands.empty()) {
      c.reason = (why == Reason::kNone) ? Reason::kNoStands : why;
      c.log(std::string("failed:select_stands:") + toString(c.reason));
      return BT::NodeStatus::FAILURE;
    }
    c.log("plan:stands:" + std::to_string(c.stands.size()));
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

/// Condition: a non-contact tool needs no calibration; a contact tool needs a valid record.
class ToolCalibrationValid : public BT::ConditionNode
{
public:
  ToolCalibrationValid(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::ConditionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    if (!ctx_->ports.tool->contact()) {
      ctx_->log("calibration:not_required");
      return BT::NodeStatus::SUCCESS;
    }
    if (ctx_->ports.calibration->record().valid) {
      ctx_->log("calibration:valid");
      return BT::NodeStatus::SUCCESS;
    }
    ctx_->reason = Reason::kCalibrationInvalid;
    ctx_->log("calibration:invalid");
    return BT::NodeStatus::FAILURE;
  }

private:
  MissionCtxPtr ctx_;
};

/// Condition: the last failure can be cured by another attempt or an escalation step.
class Retryable : public BT::ConditionNode
{
public:
  Retryable(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::ConditionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return (!ctx_->abort_requested && isRetryable(ctx_->reason)) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  MissionCtxPtr ctx_;
};

/// Another stand pose for the current stand (scope "stand": after nav or dock failed) or for the current column
/// (scope "column": escalation). FAILURE without touching the failure reason when the planner has none.
class ReStand : public BT::SyncActionNode
{
public:
  ReStand(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("scope", "stand", "stand | column")};
  }
  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    std::string scope = "stand";
    getInput("scope", scope);
    if (c.abort_requested) {return BT::NodeStatus::FAILURE;}
    auto & cur = c.stands.at(c.stand);
    StandPlan alt;
    const bool ok = scope == "column" ? c.ports.planner->standForColumn(c.column, cur, alt) :
      c.ports.planner->alternateStand(cur, c.reason, alt);
    if (!ok) {
      c.log("restand:none:" + scope);
      return BT::NodeStatus::FAILURE;
    }
    cur.id = alt.id;
    cur.pose = alt.pose;
    cur.margin_m = alt.margin_m;
    ++c.report.restands;
    c.log("restand:" + scope + ":" + std::to_string(alt.id));
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

/// Re-order escalation: put the column behind the others of its stand, once. FAILURE when it was already moved or
/// nothing is queued behind it.
class DeferColumn : public BT::SyncActionNode
{
public:
  DeferColumn(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    if (c.abort_requested || c.columns_after == 0 || c.deferred.count(c.column)) {
      return BT::NodeStatus::FAILURE;
    }
    c.deferred.insert(c.column);
    c.column_deferred = true;
    c.log("defer:" + std::to_string(c.column));
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

/// Split escalation: asks the planner for pieces; output `pieces` feeds a Repeat. FAILURE when it cannot split.
class SplitColumn : public BT::SyncActionNode
{
public:
  SplitColumn(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {BT::OutputPort<int>("pieces")};}
  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    if (c.abort_requested) {return BT::NodeStatus::FAILURE;}
    const size_t n = c.ports.planner->splitColumn(c.column);
    if (n < 2) {
      c.log("split:none:" + std::to_string(c.column));
      return BT::NodeStatus::FAILURE;
    }
    c.pieces = n;
    c.piece = 0;
    ++c.report.splits;
    setOutput("pieces", static_cast<int>(n));
    c.log("split:" + std::to_string(c.column) + ":" + std::to_string(n));
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

class NextPiece : public BT::SyncActionNode
{
public:
  NextPiece(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    ++ctx_->piece;
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

/// Tool off and valve closed (the end-of-mission step; the same calls are in the cancel cleanup).
class ToolOff : public BT::SyncActionNode
{
public:
  ToolOff(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    ctx_->ports.tool->off();
    ctx_->ports.tool->closeValve();
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

/// Publishes the result, at most once per mission run (`outcome`: finished | aborted).
class ReportResult : public BT::SyncActionNode
{
public:
  ReportResult(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::SyncActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("outcome", "finished", "finished | aborted")};
  }
  BT::NodeStatus tick() override
  {
    std::string outcome = "finished";
    getInput("outcome", outcome);
    ctx_->reportOnce(outcome);
    return BT::NodeStatus::SUCCESS;
  }

private:
  MissionCtxPtr ctx_;
};

/// Waits at a column boundary while the mission FSM is PAUSED (a pause never cancels the motion in flight).
class PauseGuard : public BT::StatefulActionNode
{
public:
  PauseGuard(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::StatefulActionNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override
  {
    if (!ctx_->fsm.atBoundary()) {return BT::NodeStatus::SUCCESS;}
    ctx_->log("pause:wait");
    ctx_->setState("paused");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (ctx_->fsm.atBoundary()) {return BT::NodeStatus::RUNNING;}
    ctx_->log("pause:resume");
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {}

private:
  MissionCtxPtr ctx_;
};

// =============================================================================== loops

/// One child run per stand. The child's FAILURE is a skipped stand: its unfinished columns are SKIPPED in the ledger
/// with the reason in `ctx.reason` and the loop goes on. Returns FAILURE only when a guard requested a mission abort.
/// A stand whose columns the ledger already holds as done is not driven to on a resume.
class ForEachStand : public BT::ControlNode
{
public:
  ForEachStand(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::ControlNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    if (children_nodes_.size() != 1) {throw BT::LogicError("ForEachStand needs exactly one child");}
    setStatus(BT::NodeStatus::RUNNING);
    while (true) {
      if (!in_stand_) {
        if (c.abort_requested) {return finish(BT::NodeStatus::FAILURE);}
        if (index_ >= c.stands.size()) {return finish(BT::NodeStatus::SUCCESS);}
        const auto & s = c.stands[index_];
        const bool all_done = !s.columns.empty() && std::all_of(
          s.columns.begin(), s.columns.end(), [&](size_t col) {return c.ledger.resumeSkips(col);});
        if (all_done) {
          c.log("resume_skip:stand:" + std::to_string(index_));
          c.report.stands.push_back({s.id, true, Reason::kNone, 0});
          ++index_;
          continue;
        }
        c.stand = index_;
        c.reason = Reason::kNone;
        restands_at_start_ = c.report.restands;
        in_stand_ = true;
      }
      const auto st = children_nodes_[0]->executeTick();
      if (st == BT::NodeStatus::RUNNING) {return BT::NodeStatus::RUNNING;}
      resetChildren();
      in_stand_ = false;
      if (c.abort_requested) {return finish(BT::NodeStatus::FAILURE);}
      auto & s = c.stands[index_];
      StandOutcome o{s.id, st == BT::NodeStatus::SUCCESS, Reason::kNone, c.report.restands - restands_at_start_};
      if (st != BT::NodeStatus::SUCCESS) {
        o.reason = c.reason == Reason::kNone ? Reason::kNavFailed : c.reason;
        for (const size_t col : s.columns) {
          const auto state = c.ledger.get(col).state;
          if (state == LedgerState::kPending || state == LedgerState::kRunning) {
            c.ledger.markSkipped(col, s.id, toSkipReason(o.reason));
            c.report.columns_skipped.emplace_back(col, o.reason);
          }
        }
        c.log(std::string("stand_skipped:") + std::to_string(s.id) + ":" + toString(o.reason));
      } else {
        c.log("stand_done:" + std::to_string(s.id));
      }
      c.report.stands.push_back(o);
      ++index_;
      if (index_ >= c.stands.size()) {return finish(BT::NodeStatus::SUCCESS);}
      return BT::NodeStatus::RUNNING;
    }
  }

  void halt() override
  {
    index_ = 0;
    in_stand_ = false;
    ControlNode::halt();
  }

private:
  BT::NodeStatus finish(BT::NodeStatus st)
  {
    index_ = 0;
    in_stand_ = false;
    return st;
  }

  MissionCtxPtr ctx_;
  size_t index_{0};
  bool in_stand_{false};
  size_t restands_at_start_{0};
};

/// One child run per column of the current stand: ledger bookkeeping as the manager's runColumnWithRetry. SUCCESS of
/// the child marks the column DONE, FAILURE marks it SKIPPED with the typed reason, a deferred column goes to the end
/// of the queue. A halted column stays RUNNING in the ledger, so a resume reruns it. Returns FAILURE only on abort.
class ForEachColumn : public BT::ControlNode
{
public:
  ForEachColumn(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::ControlNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    if (children_nodes_.size() != 1) {throw BT::LogicError("ForEachColumn needs exactly one child");}
    setStatus(BT::NodeStatus::RUNNING);
    if (!begun_) {
      queue_ = c.stands.at(c.stand).columns;
      pos_ = 0;
      in_column_ = false;
      begun_ = true;
    }
    while (true) {
      if (!in_column_) {
        if (c.abort_requested) {return finish(BT::NodeStatus::FAILURE);}
        if (pos_ >= queue_.size()) {return finish(BT::NodeStatus::SUCCESS);}
        const size_t col = queue_[pos_];
        if (c.ledger.resumeSkips(col)) {
          c.log("resume_skip:column:" + std::to_string(col));
          ++pos_;
          continue;
        }
        c.column = col;
        c.reason = Reason::kPlanFailed;   // default of a failure that gives none, as the manager does
        c.piece = 0;
        c.pieces = 1;
        c.column_deferred = false;
        c.columns_after = queue_.size() - pos_ - 1;
        c.ledger.markRunning(col, c.stands.at(c.stand).id);
        c.log("column:" + std::to_string(col));
        in_column_ = true;
      }
      const auto st = children_nodes_[0]->executeTick();
      if (st == BT::NodeStatus::RUNNING) {return BT::NodeStatus::RUNNING;}
      resetChildren();
      in_column_ = false;
      if (c.abort_requested) {return finish(BT::NodeStatus::FAILURE);}
      const size_t col = queue_[pos_];
      const int stand_id = c.stands.at(c.stand).id;
      if (c.column_deferred) {
        c.column_deferred = false;
        queue_.push_back(col);
        ++c.report.reorders;
      } else if (st == BT::NodeStatus::SUCCESS) {
        c.ledger.markDone(col, stand_id);
        c.log("column_done:" + std::to_string(col));
      } else {
        c.ledger.markSkipped(col, stand_id, toSkipReason(c.reason));
        c.report.columns_skipped.emplace_back(col, c.reason);
        c.log("column_skipped:" + std::to_string(col) + ":" + toString(c.reason));
      }
      ++pos_;
      if (pos_ >= queue_.size()) {return finish(BT::NodeStatus::SUCCESS);}
      return BT::NodeStatus::RUNNING;
    }
  }

  void halt() override
  {
    begun_ = false;
    in_column_ = false;
    ControlNode::halt();
  }

private:
  BT::NodeStatus finish(BT::NodeStatus st)
  {
    begun_ = false;
    in_column_ = false;
    pos_ = 0;
    return st;
  }

  MissionCtxPtr ctx_;
  std::vector<size_t> queue_;
  size_t pos_{0};
  bool begun_{false};
  bool in_column_{false};
};

// =============================================================================== guards and always-report

/// Safety gate. Reads the safety port every tick. A stop request (or stale data) holds the arm and stops ticking the
/// child: RUNNING until it clears, then the motion resumes. A stop longer than `timeout_s` halts the child (cleanup
/// order) and aborts the mission (SAFETY_TIMEOUT). The speed scale is passed to the arm on every tick. It never
/// lets the child continue while a stop stands.
class SafetyGate : public BT::DecoratorNode
{
public:
  SafetyGate(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::DecoratorNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout_s", 30.0, "seconds a stop may last before the mission aborts, 0: no limit")};
  }

  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    setStatus(BT::NodeStatus::RUNNING);
    const SafetyState s = c.ports.safety->read();
    const double now = c.now();
    if (s.stop_request || !s.fresh) {
      if (!stopped_) {
        stopped_ = true;
        since_ = now;
        c.ports.arm->hold();
        ++c.report.safety_stops;
        c.log(s.fresh ? "safety:stop" : "safety:stale");
        c.setState("safety_stop");
      }
      double timeout = 30.0;
      getInput("timeout_s", timeout);
      if (timeout > 0.0 && now - since_ >= timeout) {
        stopped_ = false;
        haltChild();   // leaf cleanup runs, in the defined order
        c.reason = Reason::kSafetyTimeout;
        c.abort_reason = Reason::kSafetyTimeout;
        c.abort_requested = true;
        c.log("safety:timeout");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    if (stopped_) {
      stopped_ = false;
      c.ports.arm->resume();
      c.log("safety:resume");
    }
    c.ports.arm->setSpeedScale(std::min(1.0, std::max(0.0, s.speed_scale)));
    const auto st = child_node_->executeTick();
    if (st != BT::NodeStatus::RUNNING) {resetChild();}
    return st;
  }

  void halt() override
  {
    stopped_ = false;
    DecoratorNode::halt();
  }

private:
  MissionCtxPtr ctx_;
  bool stopped_{false};
  double since_{0.0};
};

/// AUTO / TELEOP / HOLD handover. Any mode but AUTO halts the running child at once (leaf cleanup) and keeps the
/// node RUNNING. Back in AUTO nothing resumes until the operator's explicit confirm; then the column is re-planned from
/// the tool's current pose and the child restarts from its beginning. `timeout_s` > 0 aborts the mission
/// (TELEOP_TIMEOUT) when the handover lasts longer.
class TeleopGuard : public BT::DecoratorNode
{
public:
  TeleopGuard(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::DecoratorNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout_s", 0.0, "seconds a handover may last before the mission aborts, 0: no limit")};
  }

  BT::NodeStatus tick() override
  {
    auto & c = *ctx_;
    auto & tp = *c.ports.teleop;
    setStatus(BT::NodeStatus::RUNNING);
    const bool auto_mode = tp.mode() == TeleopMode::kAuto;
    if (!auto_mode && !suspended_) {
      suspended_ = true;
      awaiting_logged_ = false;
      since_ = c.now();
      haltChild();   // leaf cleanup runs, in the defined order
      ++c.report.teleop_handovers;
      tp.onHandoverStarted();
      c.setState("teleop");
      c.log("teleop:halt");
    }
    if (suspended_) {
      double timeout = 0.0;
      getInput("timeout_s", timeout);
      if (timeout > 0.0 && c.now() - since_ >= timeout) {
        suspended_ = false;
        c.reason = Reason::kTeleopTimeout;
        c.abort_reason = Reason::kTeleopTimeout;
        c.abort_requested = true;
        c.log("teleop:timeout");
        return BT::NodeStatus::FAILURE;
      }
      if (!auto_mode) {
        tp.takeConfirm();   // a confirm given before the mode is back to AUTO does not count
        return BT::NodeStatus::RUNNING;
      }
      if (!tp.takeConfirm()) {
        if (!awaiting_logged_) {
          awaiting_logged_ = true;
          c.log("teleop:await_confirm");
        }
        return BT::NodeStatus::RUNNING;
      }
      suspended_ = false;
      c.ports.planner->replanFromPose(c.column, tp.toolPose());
      ++c.report.replans;
      c.log("teleop:replan");
    }
    const auto st = child_node_->executeTick();
    if (st != BT::NodeStatus::RUNNING) {resetChild();}
    return st;
  }

  void halt() override
  {
    suspended_ = false;
    DecoratorNode::halt();
  }

private:
  MissionCtxPtr ctx_;
  bool suspended_{false};
  bool awaiting_logged_{false};
  double since_{0.0};
};

/// Root decorator of the cancel path. Ticks its child; when it is halted (goal canceled) it halts the child first, so
/// every running leaf cleans up in the defined order, and then publishes the result with outcome "canceled". On the
/// normal paths the report comes from ReportResult (Wrapup).
class ReportOnHalt : public BT::DecoratorNode
{
public:
  ReportOnHalt(const std::string & n, const BT::NodeConfig & c, MissionCtxPtr x)
  : BT::DecoratorNode(n, c), ctx_(std::move(x)) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);
    const auto st = child_node_->executeTick();
    if (st != BT::NodeStatus::RUNNING) {resetChild();}
    return st;
  }

  void halt() override
  {
    const bool was_running = status() == BT::NodeStatus::RUNNING;
    DecoratorNode::halt();
    if (was_running) {
      ctx_->reason = Reason::kCanceled;
      ctx_->reportOnce("canceled");
    }
  }

private:
  MissionCtxPtr ctx_;
};

/// Registers every node of trees/mission.xml; each one receives `ctx`.
inline void registerMissionNodes(BT::BehaviorTreeFactory & f, const MissionCtxPtr & ctx)
{
  f.registerNodeType<StartMission>("StartMission", ctx);
  f.registerNodeType<PerceiveSurface>("PerceiveSurface", ctx);
  f.registerNodeType<PlanCoverage>("PlanCoverage", ctx);
  f.registerNodeType<SelectStands>("SelectStands", ctx);
  f.registerNodeType<ToolCalibrationValid>("ToolCalibrationValid", ctx);
  f.registerNodeType<TouchCalibrate>("TouchCalibrate", ctx);
  f.registerNodeType<ForEachStand>("ForEachStand", ctx);
  f.registerNodeType<ForEachColumn>("ForEachColumn", ctx);
  f.registerNodeType<NavigateToStand>("NavigateToStand", ctx);
  f.registerNodeType<DockBase>("DockBase", ctx);
  f.registerNodeType<ReStand>("ReStand", ctx);
  f.registerNodeType<PauseGuard>("PauseGuard", ctx);
  f.registerNodeType<TeleopGuard>("TeleopGuard", ctx);
  f.registerNodeType<SafetyGate>("SafetyGate", ctx);
  f.registerNodeType<ArmDock>("ArmDock", ctx);
  f.registerNodeType<ContactPress>("ContactPress", ctx);
  f.registerNodeType<StrokeColumn>("StrokeColumn", ctx);
  f.registerNodeType<Retract>("Retract", ctx);
  f.registerNodeType<VerifyColumn>("VerifyColumn", ctx);
  f.registerNodeType<Retryable>("Retryable", ctx);
  f.registerNodeType<RecoverArm>("RecoverArm", ctx);
  f.registerNodeType<DeferColumn>("DeferColumn", ctx);
  f.registerNodeType<SplitColumn>("SplitColumn", ctx);
  f.registerNodeType<NextPiece>("NextPiece", ctx);
  f.registerNodeType<StowArm>("StowArm", ctx);
  f.registerNodeType<ToolOff>("ToolOff", ctx);
  f.registerNodeType<ReportResult>("ReportResult", ctx);
  f.registerNodeType<ReportOnHalt>("ReportOnHalt", ctx);
}

}  // namespace mission
}  // namespace manipulation_bt
