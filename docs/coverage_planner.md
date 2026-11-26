# Coverage plan redundancy report

A coverage plan should not move for nothing: strips alternate direction (serpentine), no area is swept twice,
free-space travel between strips is one pitch, and the plan finishes at the bottom of the wall. The report in
`coverage_planner/core/plan_redundancy.hpp` measures this on an expanded plan (`expandCoveragePasses` output, or the
`final_lifted_and_expanded` object of `coverage_output.json`). It is ROS-free.

```bash
plan_redundancy_report coverage_output.json [--pitch 0.2] [--bottom-z 0] [--top-z 2] [--json-out report.json]
score_run --samples run.csv --wall wall.json --plan coverage_output.json   # adds "plan_redundancy" to the card
```

`plan_redundancy_report` prints a table, writes the JSON with `--json-out`, and exits 1 when it finds a jump, a
direction violation or a double sweep (2 = bad input).

| Field | Meaning |
|-------|---------|
| `strip_length_m`, `connector_length_m` | Cartesian passes summed; path between consecutive passes (connectors and jumps) |
| `jumps_over_2x_pitch`, `max_jump_m` | Connectors longer than 2x the strip pitch (pitch given, else the median connector) |
| `double_sweep_fraction` | Share of the touched cells (pitch/2 grid on the wall plane) crossed by two different strips. A tool's own there-and-back pass over one strip does not count. |
| `alternation_ok`, `direction_violations` | Single pass: consecutive strips must run in opposite directions. Double pass (`second_pass` tag present): every first pass must start at the same end |
| `final_end_z`, `ends_at_bottom`, `ends_at_top` | End of the last strip against the lowest and highest strip point (or `--bottom-z` / `--top-z`), tolerance pitch/2 |

The tests (`plan_redundancy_test`) lock the good behaviour: a single-pass serpentine of N strips (even and odd) has
no jumps, alternates, and for the first strip going up ends at the bottom for even N; a double pass has no jump above
the pitch, with and without strip connectors; the pre-#196 plan (full-length jump at every joint) is reported.

## Fix found by the report

With strip connectors on (`add_strip_connectors`) and a double-pass tool, the expander flipped the strip start but left
the planner's straight two-pose connector at the old end, so every joint cost two strip lengths. The expander now
re-aims such a connector from where the previous strip started to where the next one starts (connectors routed round
an obstacle, more than two poses, are kept).

## Strip and stand order

`checkTravelOrder` takes the stands in the executed order, each with its base position and its strips (columns) in
the executed order with their TCP start and end. Free-space travel is the TCP chord from one strip's end to the next
strip's start within a stand, plus the base distance between consecutive stands. It is compared with the best order:
per stand an exact search over strip order and direction (up to 8 strips, then nearest neighbour over all starts,
`exact=false`), and over the stand order (free start). The report gives executed, optimal and the excess in metres
and as a fraction.

Findings: the planner's column order (adjacent columns, alternating direction) has zero excess (tested for 1 to 3
stands); the stand order of `reachability` is already a nearest-next plus 2-opt tour, so no change was needed in the
planner. A shuffled order shows the excess in the tests, so a regression would be caught.

## Coverage feed over ROS

The manager can take its plan from the coverage planner over ROS instead of reading `coverage_output.json`. The default is unchanged: `coverage_source: json` reads the file named by `coverage_json_path` (or the goal's `coverage_json_path`).

| Parameter (`manipulation_manager_node`) | Default | Meaning |
|---|---|---|
| `coverage_source` | `json` | `json`: read the file. `ros`: use the latest `coverage_planner_msgs/CoveragePlan` on `coverage_plan_topic`. Any other value stops the node at start. |
| `coverage_plan_topic` | `/wall_coverage_service_node/coverage_plan` | Topic of the plan (`wall_coverage_service_node`'s `~/coverage_plan`). |
| `coverage_plan_timeout_s` | `10.0` | How long a task waits for a plan if none has arrived; then the task fails with "cannot load the coverage plan". |

Data flow with `coverage_source:=ros`:

1. `ros2 service call /wall_coverage_service_node/start std_srvs/srv/Trigger` makes `wall_coverage_service_node` call `GenerateCoveragePath`, lift and expand the result (`expandCoveragePasses`) and publish one `CoveragePlan` on `~/coverage_plan`. The publisher is now `reliable` + `transient_local` (depth 1), so the last plan is kept and a subscriber that starts later still gets it.
2. The manager subscribes with the same QoS at start and keeps the latest message. At the start of a task `RosCoverageSource` waits for it (spinning the node, up to `coverage_plan_timeout_s`), checks it (`success`, at least one pose, segment indices inside the path), and gives it to `buildCoverageIntents()`.
3. `buildCoverageIntents()` (`core/coverage_plan_intents.hpp`) is the code `CoverageOutputJsonSource` always had, moved out: columns by position along the wall, one stroke per segment span, resampling inside `coverage_z_min_m` to `coverage_z_max_m`, `z_offset_m`, `wall_direction`. Both sources call it, so the same plan gives the same `CoverageSegmentIntent` list (a gtest compares them).

Why the topic and not a service call: the planner side already has two entry points. `GenerateCoveragePath` takes an occupancy grid and returns the raw (not lifted) path, so using it from the manager would mean the manager rasterises the wall and repeats the lifting. `wall_coverage_service_node` already does all of that and already publishes the finished plan, so the only interface change is the QoS of that publisher (one line). It is the pattern ROS 2 gives for "latest value, available to late joiners" (the `transient_local` durability of the QoS guide; Nav2's `map_server` publishes `/map` the same way).

Notes and limits:

- With `ros`, a task uses the newest plan received. The goal's `coverage_json_path` is ignored. A new plan on the topic is picked up by the next task, not one that is running.
- The mission ledger identifies the plan by a text of its poses and segments (6 decimals) instead of the file bytes, so `resume` works the same way.
- The message frame is not checked: like the JSON, the poses are taken as given (`z_offset_m` converts base_link to base_footprint).
- Status: the conversion, validation and wait are host-tested (`coverage_plan_intents_test`). The ROS glue (`ros_coverage_source.cpp`, the node's parameters and subscription, the QoS change) was reviewed by hand only: it could not be compiled on the machine this was written on (Humble headers, Jazzy target), and no run on a simulator was done.

