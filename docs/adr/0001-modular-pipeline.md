# 0001: Every pipeline stage is a swappable module

- Status: accepted
- Date: 2026-10-30

## Context

The stack must serve any task, robot and surface: walls today, non-planar surfaces later,
learned components eventually. Planning backends differ in strengths (MoveIt for
constraints and ecosystem, cuRobo v2 for GPU trajectory optimization and depth-fused ESDF,
VAMP for microsecond CPU collision checks and fast free-space plans). Earlier stacks that
hard-wired their IK, collision and timing libraries turned every backend change into a
rewrite.

## Decision

Each stage (perception, scene, surface, coverage, sequencing, IK, collision, planning,
timing, execution, control, policy) is a port with interchangeable implementations,
selected by config, exchanging typed serializable contracts. Third-party libraries are
used pinned and unmodified behind adapters. No backend is dropped; MoveIt, cuRobo v2 and
VAMP are all supported.

## Consequences

- More interfaces and conversion code up front.
- Backends are compared on one shared benchmark, with data instead of preference.
- Learned policies and new surface types plug in without touching unrelated stages.
- Upstream updates (VAMP, cuRobo) are a pin bump, not a merge.
