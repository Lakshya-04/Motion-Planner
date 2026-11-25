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
