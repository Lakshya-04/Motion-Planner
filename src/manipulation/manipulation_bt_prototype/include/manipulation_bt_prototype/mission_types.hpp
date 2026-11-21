/**
 * @file mission_types.hpp
 * @brief Vocabulary, port interfaces and the shared context of the full mission behaviour tree.
 * @details ROS-free. The tree (trees/mission.xml) is made of leaf nodes that call the abstract ports declared here;
 *          the later ROS wiring implements the ports over the existing action clients and services, the host tests
 *          implement them as fakes (mission_fakes.hpp). Failure reasons extend the manager's `SkipReason`
 *          vocabulary: every reason maps to a `SkipReason` for the ledger (`toSkipReason`).
 *          The shared state of one mission is the typed `MissionCtx` (its fields are the "blackboard keys",
 *          see docs/behavior_trees/mission.md). It is handed to every node when the node type is registered, not looked
 *          up on a blackboard, so it is visible inside subtrees without port remapping.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_manager/core/mission_fsm.hpp"
#include "manipulation_manager/core/mission_ledger.hpp"

namespace manipulation_bt
{
namespace mission
{

using manipulation_manager::LedgerState;
using manipulation_manager::MissionFsm;
using manipulation_manager::MissionLedger;
using manipulation_manager::SkipReason;

/// Failure reasons. The first five are the manager's SkipReason values; the others are new and map onto one of them.
enum class Reason
{
  kNone,
  kNavFailed,          ///< manager: NAV_FAILED
  kDockFailed,         ///< manager: DOCK_FAILED
  kPlanFailed,         ///< manager: PLAN_FAILED (also the default of a failure that gave no reason)
  kContactOverload,    ///< manager: CONTACT_OVERLOAD (never retried)
  kPerceptionFailed,   ///< scan of the surface failed
  kStandCheckFailed,   ///< achieved stand pose outside the reachability margin (plan section 7)
  kVerifyFailed,       ///< column verification below the accepted coverage
  kSafetyTimeout,      ///< safety stop lasted longer than the allowed time: mission abort
  kTeleopTimeout,      ///< teleop / hold lasted longer than the allowed time: mission abort
  kCalibrationInvalid, ///< contact tool without a valid calibration record
  kNoStands,           ///< no stand covers the plan
  kCanceled            ///< goal canceled
};

inline const char * toString(Reason r)
{
  switch (r) {
    case Reason::kNone: return "NONE";
    case Reason::kNavFailed: return "NAV_FAILED";
    case Reason::kDockFailed: return "DOCK_FAILED";
    case Reason::kPlanFailed: return "PLAN_FAILED";
    case Reason::kContactOverload: return "CONTACT_OVERLOAD";
    case Reason::kPerceptionFailed: return "PERCEPTION_FAILED";
    case Reason::kStandCheckFailed: return "STAND_CHECK_FAILED";
    case Reason::kVerifyFailed: return "VERIFY_FAILED";
    case Reason::kSafetyTimeout: return "SAFETY_TIMEOUT";
    case Reason::kTeleopTimeout: return "TELEOP_TIMEOUT";
    case Reason::kCalibrationInvalid: return "CALIBRATION_INVALID";
    case Reason::kNoStands: return "NO_STANDS";
    case Reason::kCanceled: return "CANCELED";
  }
  return "NONE";
}

/// The ledger only knows the manager's vocabulary.
inline SkipReason toSkipReason(Reason r)
{
  switch (r) {
    case Reason::kNone: return SkipReason::kNone;
    case Reason::kNavFailed: return SkipReason::kNavFailed;
    case Reason::kDockFailed:
    case Reason::kStandCheckFailed: return SkipReason::kDockFailed;
    case Reason::kContactOverload: return SkipReason::kContactOverload;
    default: return SkipReason::kPlanFailed;
  }
}

/// A failure that another attempt or an escalation step cannot cure: the cause is on the wall, in the operator's
/// hands or in the safety layer, not in the plan.
inline bool isRetryable(Reason r)
{
  switch (r) {
    case Reason::kContactOverload:
    case Reason::kSafetyTimeout:
    case Reason::kTeleopTimeout:
    case Reason::kCalibrationInvalid:
    case Reason::kNoStands:
    case Reason::kCanceled: return false;
    default: return true;
  }
}

struct Pose2 {double x{0.0}; double y{0.0}; double yaw{0.0};};
struct Pose3 {double x{0.0}; double y{0.0}; double z{0.0}; double roll{0.0}; double pitch{0.0}; double yaw{0.0};};

/// A stand: where the base parks and which columns (ledger segments) it covers.
struct StandPlan
{
  int id{0};
  Pose2 pose;
  std::vector<size_t> columns;
  double margin_m{0.05};   ///< largest accepted distance between the achieved and the planned stand pose
};

struct CoveragePlan {size_t column_count{0};};

enum class OpStatus { kRunning, kDone, kFailed };

/// Answer of a poll on a long-running port operation. `value` is operation specific (dock: pose error in m,
/// verify: covered fraction).
struct OpPoll
{
  OpStatus status{OpStatus::kRunning};
  Reason reason{Reason::kNone};
  double value{0.0};
};

// ----------------------------------------------------------------------------- ports

class PerceptionPort
{
public:
  virtual ~PerceptionPort() = default;
  virtual void startScan() = 0;
  virtual OpPoll poll() = 0;
  virtual void cancel() = 0;
};

/// Planning calls are short and blocking (a tick waits for them), as the manager's service calls are today.
class PlannerPort
{
public:
  virtual ~PlannerPort() = default;
  virtual bool planCoverage(CoveragePlan & out, Reason & why) = 0;
  /// reachability plan_stands; `wanted` = ExecuteCoverage.columns (empty: all).
  virtual bool selectStands(
    const CoveragePlan & plan, const std::vector<size_t> & wanted, std::vector<StandPlan> & out, Reason & why) = 0;
  /// Another stand for the same columns after `failed` could not be reached or docked.
  virtual bool alternateStand(const StandPlan & failed, Reason why, StandPlan & out) = 0;
  /// A stand from which `column` is reachable, other than `current` (escalation step re-stand).
  virtual bool standForColumn(size_t column, const StandPlan & current, StandPlan & out) = 0;
  /// Number of pieces `column` can be split into; 1 means it cannot be split.
  virtual size_t splitColumn(size_t column) = 0;
  /// Re-plan `column` from where the tool is now (after a teleop handover).
  virtual void replanFromPose(size_t column, const Pose3 & tool_pose) = 0;
};

class NavigationPort
{
public:
  virtual ~NavigationPort() = default;
  virtual void startNavigate(const StandPlan & stand) = 0;
  virtual OpPoll poll() = 0;
  virtual void cancel() = 0;
};

/// Wall docking of the base at the stand. `OpPoll::value` is the achieved stand-pose error in m.
class DockPort
{
public:
  virtual ~DockPort() = default;
  virtual void startDock(const StandPlan & stand) = 0;
  virtual OpPoll poll() = 0;
  virtual void cancel() = 0;
};

enum class ArmPhase { kApproach, kPress, kStroke, kRetract, kStow, kRecover };

inline const char * toString(ArmPhase p)
{
  switch (p) {
    case ArmPhase::kApproach: return "approach";
    case ArmPhase::kPress: return "press";
    case ArmPhase::kStroke: return "stroke";
    case ArmPhase::kRetract: return "retract";
    case ArmPhase::kStow: return "stow";
    case ArmPhase::kRecover: return "recover";
  }
  return "approach";
}

/// The arm side of a column. The cleanup calls are the manager's `abortCleanup()` split into its steps.
class ArmMotionPort
{
public:
  virtual ~ArmMotionPort() = default;
  virtual void start(ArmPhase phase, size_t column, size_t piece) = 0;
  virtual OpPoll poll() = 0;
  virtual void cancelTrajectory() = 0;
  virtual void admittanceOff() = 0;
  virtual void forceTargetZero() = 0;
  /// Stop moving but keep the motion resumable (safety stop).
  virtual void hold() = 0;
  virtual void resume() = 0;
  virtual void setSpeedScale(double scale) = 0;
  virtual double peakForceN() = 0;   ///< largest force magnitude so far, -1 when no force topic is read
};

class VerifyPort
{
public:
  virtual ~VerifyPort() = default;
  virtual void startVerify(size_t column, size_t piece) = 0;
  virtual OpPoll poll() = 0;   ///< value: covered fraction
  virtual void cancel() = 0;
};

struct SafetyState
{
  double speed_scale{1.0};     ///< 0..1, /safety/speed_scale
  bool stop_request{false};    ///< /safety/stop_request
  bool fresh{true};            ///< false when the data is stale: treated as a stop (fail safe)
};

class SafetyPort
{
public:
  virtual ~SafetyPort() = default;
  virtual SafetyState read() = 0;
};

enum class TeleopMode { kAuto, kTeleop, kHold };

class TeleopPort
{
public:
  virtual ~TeleopPort() = default;
  virtual TeleopMode mode() = 0;
  /// True once when the operator pressed the explicit confirm; the call consumes it.
  virtual bool takeConfirm() = 0;
  virtual Pose3 toolPose() = 0;
  /// Called once when the guard has halted the automatic motion for a handover.
  virtual void onHandoverStarted() = 0;
};

struct CalibrationRecord
{
  bool valid{false};
  double residual_mm{0.0};
};

class CalibrationPort
{
public:
  virtual ~CalibrationPort() = default;
  virtual CalibrationRecord record() = 0;
  virtual void startTouch() = 0;
  virtual OpPoll poll() = 0;
  virtual void cancelTouch() = 0;
};

class ToolPort
{
public:
  virtual ~ToolPort() = default;
  virtual bool contact() = 0;      ///< true: contact tool (needs a calibration record)
  virtual void off() = 0;
  virtual void closeValve() = 0;
};

struct StandOutcome
{
  int stand{0};
  bool reached{false};
  Reason reason{Reason::kNone};
  size_t restands{0};
};

/// Content of ExecuteCoverage.Result plus the counters of the tree's own policy.
struct MissionReport
{
  std::string outcome;   ///< finished | aborted | canceled
  bool success{false};   ///< every column asked for is done and every stand reached
  std::string message;
  std::vector<size_t> columns_done;
  std::vector<size_t> columns_failed;
  std::vector<std::pair<size_t, Reason>> columns_skipped;   ///< this run, with the full reason
  std::vector<StandOutcome> stands;
  Reason abort_reason{Reason::kNone};
  double peak_force_n{-1.0};
  size_t restands{0};
  size_t splits{0};
  size_t reorders{0};
  size_t retries{0};
  size_t replans{0};
  size_t safety_stops{0};
  size_t teleop_handovers{0};
};

class ReportSink
{
public:
  virtual ~ReportSink() = default;
  virtual void publish(const MissionReport & report) = 0;   ///< exactly once per mission run
  virtual void state(const std::string & state) = 0;        ///< ExecuteCoverage feedback `state`
};

struct MissionPorts
{
  std::shared_ptr<PerceptionPort> perception;
  std::shared_ptr<PlannerPort> planner;
  std::shared_ptr<NavigationPort> navigation;
  std::shared_ptr<DockPort> dock;
  std::shared_ptr<ArmMotionPort> arm;
  std::shared_ptr<VerifyPort> verify;
  std::shared_ptr<SafetyPort> safety;
  std::shared_ptr<TeleopPort> teleop;
  std::shared_ptr<CalibrationPort> calibration;
  std::shared_ptr<ToolPort> tool;
  std::shared_ptr<ReportSink> sink;
};

/// Shared state of one mission. Fields are the typed blackboard keys of the tree.
struct MissionCtx
{
  MissionPorts ports;
  MissionFsm fsm;
  MissionLedger ledger;

  // Goal (ExecuteCoverage fields)
  std::string mission_id{"mission"};
  bool resume{false};                   ///< ExecuteCoverage.resume
  std::vector<size_t> wanted_columns;   ///< ExecuteCoverage.columns, empty: all

  // Plan
  CoveragePlan plan;
  std::vector<StandPlan> stands;

  // Cursor
  size_t stand{0};      ///< index into `stands`
  size_t column{0};     ///< current column (ledger segment)
  size_t piece{0};      ///< current piece of a split column
  size_t pieces{1};     ///< pieces of the current column (1: not split)

  // Failure bookkeeping
  Reason reason{Reason::kNone};      ///< reason of the last failure inside the current stand or column
  bool abort_requested{false};       ///< a guard ended the mission (safety or teleop timeout)
  Reason abort_reason{Reason::kNone};
  bool column_deferred{false};       ///< set by DeferColumn, read by ForEachColumn
  size_t columns_after{0};           ///< columns still queued behind the current one in this stand
  std::set<size_t> deferred;         ///< columns already re-ordered once

  MissionReport report;
  bool report_sent{false};
  std::vector<std::string> events;   ///< ordered log (nodes and fakes append); the tests read it

  std::function<double()> clock;     ///< seconds; tests inject a fake one
  double now() const
  {
    if (clock) {return clock();}
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
  }

  void log(const std::string & e) {events.push_back(e);}

  void setState(const std::string & s)
  {
    log("state:" + s);
    if (ports.sink) {ports.sink->state(s);}
  }

  /// The cancel / abort cleanup, in the defined order (the manager's abortCleanup()).
  void safeState()
  {
    ports.arm->cancelTrajectory();
    ports.tool->off();
    ports.arm->admittanceOff();
    ports.arm->forceTargetZero();
    ports.tool->closeValve();
  }

  /// Builds the result and publishes it, at most once per mission run.
  void reportOnce(const std::string & outcome)
  {
    if (report_sent) {return;}
    report_sent = true;
    auto & r = report;
    r.outcome = outcome;
    r.columns_done = ledger.columnsIn(LedgerState::kDone);
    r.columns_failed = ledger.columnsIn(LedgerState::kFailed);
    r.abort_reason = abort_reason;
    bool all_stands = true;
    for (const auto & s : r.stands) {all_stands = all_stands && s.reached;}
    r.success = outcome == "finished" && r.columns_skipped.empty() && r.columns_failed.empty() && all_stands &&
      !r.columns_done.empty();
    if (outcome == "finished") {
      r.message = r.success ? "ok" : "finished with skipped columns";
    } else if (outcome == "canceled") {
      r.message = "canceled";
      fsm.stop();
    } else {
      r.message = std::string("aborted: ") + toString(abort_reason != Reason::kNone ? abort_reason : reason);
    }
    log("report:" + outcome);
    if (ports.sink) {ports.sink->publish(r);}
  }
};

using MissionCtxPtr = std::shared_ptr<MissionCtx>;

}  // namespace mission
}  // namespace manipulation_bt
