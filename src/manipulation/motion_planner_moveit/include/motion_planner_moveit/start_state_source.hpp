#pragma once

namespace motion_planner_moveit
{

/// Where the OMPL start state of a plan without an explicit start comes from.
enum class StartStateSource
{
  kPlanningSceneMonitor,  ///< The monitor's current state: it has its own spinner and is the one kept up to date.
  kMoveGroup              ///< MoveGroupInterface::getCurrentState (the earlier behaviour).
};

/// A monitor state is trusted only when the monitor exists and its joint state is complete and recent. A stale or
/// partial one (no joint_states for seconds) would plan from where the arm used to be, so fall back instead.
inline StartStateSource chooseStartStateSource(bool monitor_present, bool monitor_state_fresh)
{
  return (monitor_present && monitor_state_fresh) ? StartStateSource::kPlanningSceneMonitor :
         StartStateSource::kMoveGroup;
}

}  // namespace motion_planner_moveit
