# BehaviorTree.CPP for the manager: evaluation and prototype

Scope: ROADMAP section 18 asks for the manager action contract on BehaviorTree.CPP with BehaviorTree.ROS2 action nodes, the always-report
convention, halt and skip guards and the mission FSM. The manager today does this with a hand-written flow (`core/mission_fsm.hpp`,
`core/mission_ledger.hpp`, `core/coverage_task.hpp`, and the run loop in `ros/manipulation_manager_node.cpp`). This document evaluates the change.
It does not rewrite the manager: there is no simulator in this work, so a rewrite could not be checked against behaviour.

What exists: this document, a target tree in BT XML ([behavior_trees/coverage_mission.xml](behavior_trees/coverage_mission.xml)), and an opt-in,
ROS-free prototype package `src/manipulation/manipulation_bt_prototype` with gtests over fake executors.

Status update: the maintainer wants the entire expected behaviour of the stack expressed as a tree. That tree now exists as design plus ROS-free node
logic: [behavior_trees/mission.md](behavior_trees/mission.md) (diagram, node catalogue, guards, cancel contract, migration plan),
`manipulation_bt_prototype/trees/mission.xml`, node implementations behind abstract ports and 53 gtests over fake ports (stand and column loops, retry then
re-stand, re-order, split, skip, safety gate, teleop handover, calibration gate, cancel cleanup order, always-report, resume). It is **not wired to
ROS, not connected to the manager and not run in a simulator**; the recommendation below (adopt later, staged, each step checked by the sim passes) is unchanged,
the staged plan is now in the mission document. `coverage_mission.xml` stays as the earlier, smaller design sketch.

## 1. Availability (checked 2026-11-18 in `motion-planner:build`, Ubuntu 24.04, ROS Jazzy)

| Item | Result |
|---|---|
| `behaviortree_cpp` (BT.CPP 4) | `ros-jazzy-behaviortree-cpp`: 4.10.0 already installed in the image (it comes in with Nav2); apt candidate 4.11.0. Rosdep key `behaviortree_cpp` resolves to it. Licence field in the package: MIT. |
| `behaviortree_cpp_v3` | 3.8.8 also in the Jazzy apt index. Not wanted: v3 is the old API, nothing here needs it. |
| `behaviortree_ros2` (BehaviorTree.ROS2 `RosActionNode`) | Not available: no `ros-jazzy-behaviortree-ros2` apt package and `rosdep resolve behaviortree_ros2` fails ("no rosdep rule"). The upstream repository (BehaviorTree/BehaviorTree.ROS2, licence believed to be MIT like BT.CPP but not checked: no network access to its licence file was used here) would have to be vendored or built from source in the workspace, together with its `btcpp_ros2_interfaces` package. |

Consequence: the ROS-free BT.CPP part can be adopted today without leaving apt; the "BehaviorTree.ROS2 action nodes" part cannot, unless the repository
takes a source dependency. Nav2 ships its own BT action node base (`nav2_behavior_tree`, Apache-2.0) which is a possible alternative to copy the pattern from, not to depend on.

## 2. What the manager does today, as BT nodes

Sources read: `core/mission_fsm.hpp`, `core/mission_ledger.hpp`, `core/coverage_task.hpp` (`TaskStateMachine`, `TaskTracker`),
`runColumnWithRetry` and the `ExecuteCoverage` handler in `ros/manipulation_manager_node.cpp`, `ExecuteCoverage.action`.

### States and transitions

| Today | BT equivalent | Note |
|---|---|---|
| `MissionState` IDLE / ACTIVE / PAUSED / STOPPED / RECOVERY (`MissionFsm`) | Stays as data. The tree reads it in `ForEachColumn` (pause guard) and `RecoverArm` (enter/leave RECOVERY). | A BT is not a state machine over operator modes; the mission FSM is orthogonal state that guards the tree. |
| `pause` on `mission_command`, effective at a column boundary (`atBoundary()` in `runColumnWithRetry`) | Pause guard at the top of the column loop (`SegmentLoop` / `ForEachColumn` returns RUNNING while PAUSED). | Done in the prototype; a pause never cancels the segment in flight (tested). |
| Cancel (`handle->is_canceling()`, `stop()`, `abortCleanup()`) | `tree.haltTree()`; each running leaf's `onHalted()` does its cleanup (cancel trajectory, tool off, admittance off, force target 0). | Done in the prototype with a fake executor; the real cleanup lives in `abortCleanup()` and the tool actuation node. |
| `TaskState` IDLE, PLANNING_STANDS, NAVIGATING, RUNNING_COLUMNS, SUCCEEDED, FAILED, CANCELED (`TaskStateMachine`) | The active path in the tree. Feedback `state` string would be written by each leaf (as `setState` is today), or read from a tree status observer. | The state enum would disappear, the feedback strings stay. |
| `NavVerdict` retry / skip / finish | `RetryUntilSuccessful(num_attempts=nav_attempts)` around `NavigateToStand`; the failure branch skips the stand. | |
| `TaskTracker` done / partial / failed lists, stand outcomes | Stays (result assembly). Leaves and the loop node call it. | |
| `MissionLedger` PENDING / RUNNING / DONE / SKIPPED / FAILED | Stays. Used by the loop node exactly where `runColumnWithRetry` uses it. | Prototype uses the real header unchanged. |
| `ExecuteCoverage.resume` | The loop node skips `ledger.resumeSkips(i)`; a halted tree resumes the same way. | Tested in the prototype (halt, tick again, only unfinished segments rerun). |

### Failure reasons

| Reason / outcome | Where it arises today | BT mapping |
|---|---|---|
| `NAV_FAILED` | all `nav_attempts` of a stand failed (`runStands`) | `NavigateToStand` (action node) FAILURE after the retry decorator; the stand subtree's failure branch writes the reason and the stand's columns are skipped. |
| `DOCK_FAILED` | `column_reason_` set in `runColumn` when the dock step fails | `DockColumn` sets the blackboard reason, returns FAILURE. |
| `PLAN_FAILED` | default `column_reason_` of a failed column | default in the loop node; also written by the last fallback branch. |
| `CONTACT_OVERLOAD` | `column_peak_ > contact_overload_n_`, never retried | `ContactOverloaded` condition before the retry branch, writes the reason. |
| partial column (some poses done) | `ColumnOutcome::kPartial` -> ledger FAILED, rerun on resume | needs a third child status. BT has SUCCESS / FAILURE only, so partial must be a blackboard flag read by the loop node. This is an awkward point, not a blocker. |
| RECOVERY | `enterRecovery()` around `recoverArm` between retries | `RecoverArm` leaf in the retry branch toggles the FSM. |
| cancel | `stop()` | halt, see above; the ledger keeps the column RUNNING so a resume reruns it (same as today). |

### Tree

[behavior_trees/coverage_mission.xml](behavior_trees/coverage_mission.xml) holds the whole target tree: `CoverageMission` (always-report fallback around
load plan, plan stands, per-stand loop, report), `Stand` (stow, navigate with retry, prime the scene, per-column loop; a stand that cannot be reached is skipped with
`NAV_FAILED`), `Column` (controller switch to position, dock, switch to contact, press/stroke/retract with the contact-overload, arm recovery and plan-failed branches, switch back, safe state on any
non-success exit). Controller switching is a leaf that only appears at the column boundary. The prototype's tree
(`manipulation_bt_prototype/trees/prototype_mission.xml`) is the same shape reduced to what fake executors can exercise:

```xml
<Fallback>
  <Sequence>
    <NavigateToStand/>
    <SegmentLoop>
      <Sequence> <SwitchController mode="contact"/> <ExecuteSegment/> </Sequence>
    </SegmentLoop>
    <ReportStatus outcome="finished"/>
  </Sequence>
  <ForceFailure> <ReportStatus outcome="aborted"/> </ForceFailure>
</Fallback>
```

The literal `Fallback(work, ForceFailure(ReportStatus))` of the roadmap only reports on failure; a success branch needs its own report, which is why
the success path ends in `ReportStatus outcome="finished"` inside `work`. The always-report property is then: every path that returns a status (not halt, not exception) reports exactly once.

## 3. Prototype: what it proves

`src/manipulation/manipulation_bt_prototype` (header-only library plus one gtest, BT.CPP only, no ROS). It reuses `MissionFsm` and `MissionLedger` unchanged
by including the manager's headers (a prototype shortcut; linking the manager library would pull MoveIt and PCL). Opt-in: `find_package(behaviortree_cpp QUIET)`; without it the package configures to nothing and the workspace builds.

Tests (`test/bt_prototype_test.cpp`):

| Test | Proves |
|---|---|
| SuccessPath | all segments run in order, one report, ledger all DONE, no cancel |
| FailedSegmentIsSkipped... | a failing segment is SKIPPED with its typed reason (`DOCK_FAILED`, `CONTACT_OVERLOAD`), the next segments still run, the mission returns SUCCESS and reports once; resume rules (contact overload not rerun, dock failure rerun) hold |
| HaltMidSegment... | `haltTree()` calls the executor's cancel once, the segment stays RUNNING in the ledger, no report; ticking again reruns only segment 1 and 2 |
| FailureInsideTheWork... | a failure before any segment (`NAV_FAILED`) still ends in `ReportStatus outcome=aborted`, and the tree still returns FAILURE |
| ExceptionInANode... | documents a limit: an exception bypasses the fallback and nothing is reported. Nodes must catch their own, or the host must wrap `tickOnce()` |
| PauseAtTheSegmentBoundary | a pause requested mid-segment lets the segment finish, nothing starts for as long as the FSM is PAUSED, resume continues, no cancel |
| ControllerSwitches... | `SwitchController` events never fall between a segment's start and its end; the node refuses (FAILURE) while a segment is in flight |
| TargetTreeInDocs... | `docs/behavior_trees/coverage_mission.xml` parses and every node and port it uses is declared (stub leaves). It is not ticked. |

What it does not prove: anything about real timing, real action servers, cancel latency, the behaviour with the actual JTAC/MoveIt/stand_navigator,
or that the manager's flow is reproduced (the target tree is untested beyond parsing). One typing gap found while writing it: an executor failure
with reason `kNone` is recorded as SKIPPED with reason NONE; the real adapters must always supply a reason, or the loop node should default to `PLAN_FAILED` as the manager does (not done in the prototype).

## 4. Cost and benefit

Benefit
- The always-report, halt-cleanup and skip-with-reason rules become structure that can be read and tested in isolation (as the prototype tests do), instead of being spread through a 1700-line node file.
- Subtrees (`Stand`, `Column`) give the per-stand and per-column policy (retry counts, which failure is retried) as XML, changeable without recompiling; new applications (a different tool flow) are new trees over the same leaves.
- Groot-style tree logging is a ready path to the "BT log" item of the observability bundle (ROADMAP section 18).

Cost
- Extra dependency: BT.CPP is already in the image through Nav2 and is MIT; low. BehaviorTree.ROS2 is not packaged for Jazzy: a vendored or source-built second dependency, with its own interfaces package, to maintain.
- The manager's flow is a loop with shared mutable state (`column_peak_`, `column_reason_`, `arm_stuck_`, the tracker, the ledger, the tool actuation). A tree needs these on a blackboard or a context object; most of the node file would move into leaf classes, not disappear.
- Ticking: BT.CPP is tick-driven. The current flow blocks on actions (`waitFor`). A tree must either tick at a fixed rate (10 to 50 Hz is cheap for a tree of this size: a tick is a handful of virtual calls) with stateful action leaves, or use the BT.ROS2 nodes' own spinning. Cancel and pause latency become one tick period. This is a design change, not a measurement; no rate was measured here.
- BT status is three-valued. Partial column and typed reasons ride on the blackboard (see section 2); that is workable but it is a convention that has to be kept.
- Exceptions bypass fallbacks (tested): the always-report guarantee needs try/catch in leaves.
- Rewrite risk is the main cost: the current flow has 1259 tests across the stack and was exercised in simulation (multi-stand runs, pause, resume, jog recovery). The tree would have to repeat that without regressions, and the sim runs are what is missing today.

Composition with `core/`
- `MissionFsm` and `MissionLedger` are ROS-free and unchanged in the prototype: they compose as data used by the loop node. Nothing in `core/` has to change. `TaskStateMachine` (stand and nav bookkeeping) would be absorbed by the tree; `TaskTracker` stays for results.
- Hence the migration can be incremental: a BT host that calls the existing column function as one leaf (`RunColumn`) gives the structure benefits without moving the flow.

## 5. Recommendation: adopt later (not now)

Reasons
1. The hand-written flow works and has the guarantees the roadmap asks for that matter (typed skips, boundary pause, ledger resume, cancel cleanup), each unit tested. What the tree would add is readability and configurability, not a behaviour that is missing.
2. A rewrite cannot be verified without the simulator. The prototype shows the pattern is sound for the isolated rules; it cannot show equivalence with the manager.
3. BehaviorTree.ROS2 is not available as a Jazzy package, so the "action nodes, not hand-rolled clients" half of the roadmap item would add a vendored dependency now, for plain clients that already work.
4. The trigger to adopt: a second application flow that differs from coverage (for example a spray flow with a different retry policy or a pick flow), or the need for run-time editable policy. At that point do the incremental route: BT host with one `RunColumn` leaf wrapping today's code, then peel leaves out, each step checked by the existing sim pass scripts; take BehaviorTree.ROS2 only if its source dependency is acceptable by then (or write the three action leaves on BT.CPP's `StatefulActionNode`, as the prototype does).

Not "keep the FSM for good": the evaluation found no blocker. Not "adopt": no verification path today.

## 6. Not verified

- Nothing ran with ROS, the manager, a simulator or hardware. The prototype is tested only against fake executors.
- The target tree is checked for well-formedness and declared nodes only; it is not ticked and its branch logic (retry then overload ordering) has not been compared to `runColumnWithRetry` by execution.
- BehaviorTree.ROS2: availability was checked by apt/rosdep only; its API, licence file and Jazzy build were not checked.
- Tick-rate, cancel latency and CPU cost were not measured.
- Licences are the `package.xml` / Debian copyright fields of the installed packages, not a legal review.
