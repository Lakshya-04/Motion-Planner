# manipulation_bt_prototype

Opt-in evaluation prototype: BehaviorTree.CPP 4 nodes for the manager's mission flow (always-report fallback, segment loop with typed skips, pause at segment boundaries, halt cleanup) over fake segment executors. The manager does not use it.

## Tier

ROS-free core library (header-only `INTERFACE` library), an evaluation prototype.

## Inputs and outputs

- C++: `include/manipulation_bt_prototype/bt_prototype.hpp`, namespace `manipulation_bt`: `ISegmentExecutor` (the three calls the real manager makes through its JTAC adapter, reduced; you implement it), `SegmentPoll`, and the BT nodes `NavigateToStand`, `SegmentLoop`, `SwitchController`, `ExecuteSegment`, `ReportStatus`.
- Tree: `trees/prototype_mission.xml`; the test also loads `docs/behavior_trees/coverage_mission.xml`.
- It reuses the manager's `MissionFsm` and `MissionLedger` unchanged, by including `manipulation_manager`'s core headers from the sibling source directory.

## Dependencies

- In this repo: the headers of `manipulation_manager` (found by relative path, not declared in `package.xml`).
- External: `behaviortree_cpp` (version 4), `nlohmann_json`. The package builds only when `behaviortree_cpp` is found.
- No ROS runtime, no GPU, no simulator.

## Use it on its own

```bash
colcon build --packages-select manipulation_bt_prototype   # skipped with a status message if behaviortree_cpp is missing
```

Implement `ISegmentExecutor`, register the nodes with a `BT::BehaviorTreeFactory` as the test does, and load a tree.

## Tests

10 test cases in `test/bt_prototype_test.cpp` (counted from the source).

```bash
colcon test --packages-select manipulation_bt_prototype
```

## Status and limits

- A prototype for the evaluation in the BT doc, whose recommendation is to adopt later. Not wired into the manager, not run in any simulation pass.
- Fake executors only; nothing here has run on hardware.

## Related docs

- [behavior_tree_evaluation.md](../../../docs/behavior_tree_evaluation.md)
- [architecture.md](../../../docs/architecture.md)
