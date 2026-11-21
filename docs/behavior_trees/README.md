# Behaviour tree files

- [mission.md](mission.md): the full mission tree (diagram, node catalogue, cross-cutting guards, cancel and cleanup contract, what is stubbed, migration plan).
  The tree itself is `src/manipulation/manipulation_bt_prototype/trees/mission.xml`; its ROS-free nodes and fake ports are tested on the host, not wired and not run in a simulator.
- `coverage_mission.xml`: the earlier, smaller target tree for `ExecuteCoverage` written for BehaviorTree.CPP 4 (design only; the manager does not run it).
- The reduced prototype's tree is `src/manipulation/manipulation_bt_prototype/trees/prototype_mission.xml`.

The reasoning, the node map and the recommendation are in [../behavior_tree_evaluation.md](../behavior_tree_evaluation.md).
