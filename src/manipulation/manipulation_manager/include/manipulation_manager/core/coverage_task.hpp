/**
 * @file coverage_task.hpp
 * @brief Pure logic of the manager's coverage task: columns grouped per stand, the task state machine, and the
 *        record of per-stand / per-column outcomes that becomes the action result.
 * @details ROS-free. The node (manipulation_manager_node.cpp) does the I/O -- plan_stands, NavigateToStand, the column
 *          flow -- and asks these types what comes next. BehaviorTree.CPP (what Nav2 uses for task flow) is the
 *          swap if the flow ever needs more than this sequence; see docs/stand_navigation.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace manipulation_manager
{

enum class ColumnOutcome { kDone, kPartial, kFailed };

/// Which stand runs which column.
struct StandAssignment
{
  /// Per stand (in visiting order): the columns it runs, ascending.
  std::vector<std::vector<size_t>> columns_per_stand;
  /// Columns no stand covers any pose of (they are not run).
  std::vector<size_t> unassigned;
};

/**
 * @brief Group columns per stand.
 * @param poses_per_column number of poses of each column, in plan order (the request's `targets_world` is
 *        the poses of the columns, concatenated in that order).
 * @param stand_targets per stand the target indices it reaches (reachability Stand.target_indices).
 * @param wanted columns to consider; empty = all.
 * @details A column goes to the stand that reaches most of its poses; on a tie the stand visited first. A column
 *          whose poses are split between stands is therefore run once, by the better stand, and may come out partial.
 */
StandAssignment assignColumnsToStands(
  const std::vector<size_t> & poses_per_column,
  const std::vector<std::vector<size_t>> & stand_targets,
  const std::vector<size_t> & wanted = {});

/**
 * @brief Split columns into groups no wider than `max_span` along the wall (the stand planner is asked for each group
 *        in turn, which caps what one stand is trusted with).
 * @param lateral position of each column along the wall.
 * @return groups of column indices; each group is ascending in lateral position and its columns lie within `max_span`
 *         of the group's first. Groups come in ascending order. `max_span` <= 0: one group of everything.
 */
std::vector<std::vector<size_t>> groupByLateralSpan(const std::vector<double> & lateral, double max_span);

enum class TaskState
{
  kIdle,
  kPlanningStands,
  kNavigating,
  kRunningColumns,
  kSucceeded,      ///< the loop ran to its end (columns may still have failed: see the tracker)
  kFailed,         ///< could not start (no stands)
  kCanceled,
};

const char * toString(TaskState s);

/// What to do after a navigation attempt.
enum class NavVerdict
{
  kRun,       ///< arrived: run the stand's columns
  kRetry,     ///< failed, attempts left: send the goal again
  kSkip,      ///< failed for good: this stand's columns are not run, drive to the next stand
  kFinish,    ///< failed for good and no stand is left
};

/**
 * @class TaskStateMachine
 * @brief idle -> (planning stands -> navigating -> running columns -> navigating -> ...) -> done, cancel from anywhere.
 */
class TaskStateMachine
{
public:
  explicit TaskStateMachine(int nav_attempts = 2);

  TaskState state() const {return state_;}
  size_t stand() const {return stand_;}
  size_t standCount() const {return stand_count_;}
  bool terminal() const;

  /// One stand, no navigation: idle -> running columns.
  bool startSingle();
  /// idle -> planning stands.
  bool startMulti();
  /// planning stands -> navigating to stand 0 (or failed when there is no stand).
  bool standsPlanned(size_t stand_count);
  bool standsFailed();
  /// navigating -> see NavVerdict. Attempts are counted per stand.
  NavVerdict navigationResult(bool reached);
  /// running columns -> navigating to the next stand, or succeeded after the last. Returns false out of order.
  bool standFinished();
  /// running columns of a single-stand task -> succeeded.
  bool singleFinished();
  /// Any non-terminal state -> canceled.
  bool cancel();

private:
  void nextStand();

  int nav_attempts_;
  int attempts_made_{0};
  TaskState state_{TaskState::kIdle};
  size_t stand_{0};
  size_t stand_count_{0};
};

/// The outcome record of one run: per column, per stand, and the totals the action reports.
class TaskTracker
{
public:
  struct Stand
  {
    size_t index{0};
    bool reached{false};
    std::string message;
    std::vector<size_t> done;
    std::vector<size_t> partial;
    std::vector<size_t> failed;
  };

  /// Single-stand run: one implicit stand (not listed in `stands()`).
  void beginSingle();
  /// Multi-stand run with `n` stands, none reached yet.
  void beginMulti(size_t n);

  void standReached(size_t stand, bool reached, const std::string & message);
  void columnFinished(size_t stand, size_t column, ColumnOutcome outcome);
  /// Columns that were not run for a reason that is not a stand (no stand covers them).
  void columnSkipped(size_t column);
  /// A column the mission ledger already holds as done (resume): counts as done, ran on no stand this time.
  void columnResumed(size_t column) {done_.push_back(column);}

  const std::vector<size_t> & done() const {return done_;}
  const std::vector<size_t> & partial() const {return partial_;}
  const std::vector<size_t> & failed() const {return failed_;}
  const std::vector<Stand> & stands() const {return stands_;}
  bool multi() const {return multi_;}

  /// Every column accounted for as done and every stand reached.
  bool success() const;
  /// "3 done, 1 partial, 0 failed" (+ the stands that were not reached).
  std::string summary() const;

private:
  bool multi_{false};
  std::vector<Stand> stands_;
  std::vector<size_t> done_, partial_, failed_;
};

}  // namespace manipulation_manager
