# Tools

A tool is the thing on the end of the arm: a wiper pressed on the wall, a spray gun held away from it. One
`ToolProfile` (`core_types/tool_profile.hpp`) describes it for every consumer: the coverage planner, the
manipulation manager and the force layer. Profiles live in YAML and are read by
`ros_adapters/tool_profile_yaml_loader.hpp`.

Shipped profiles: `src/perception/coverage_planner/config/tool_profiles.yaml` (installed to
`share/coverage_planner/config/`). Every field there is commented with its unit.

| Profile | Kind | Notes |
|---|---|---|
| `contact_wiper` | contact | swath 0.20 m, overlap 20 %, 15 N, two passes (second reversed), admittance control |
| `spray_gun` | non-contact | standoff 0.30 m, cone half-angle 20 deg, swath derived from the standoff, valve command, trajectory control only |

## Selecting a tool

`wall_coverage_demo_node` and `wall_coverage_service_node` take:

| Parameter | Default | Meaning |
|---|---|---|
| `tool_profiles_file` | the shipped file | any file in the same schema |
| `tool_name` | `contact_wiper` | key under `tools:` |

Both launch files take the same two arguments. Explicit parameters override single fields of the
profile and are only applied when set: `contact`, `standoff_m`, `tool_normal_policy`, `max_edge_tilt_deg`,
`tool_frame_yaw_deg`, `swath_m`, `overlap_pct`, `sweep_direction`, `enable_boundary_pass`,
`boundary_offset_m`, `obstacle_inflation`, `obstacle_inflation_radius_m`, `double_pass` (true forces two
passes). An explicit `swath_m` also switches off the spray-cone derivation. If no inflation radius is set
anywhere it falls back to half the swath.

```
ros2 launch coverage_planner wall_coverage_demo.launch.py tool_name:=spray_gun standoff_m:=0.4
```

## ToolProfile fields

Four facets. The coverage planner reads the ones marked (C); the rest are for the manager and the force layer.

- `geometry`: `mesh`, `mesh_origin_xyz/rpy`, `tcp_xyz/rpy` (flange to `link_tcp`), `contact_allowed_links`
  (links allowed to touch the wall; empty for a non-contact tool).
- `task`: `contact` (C), `standoff_m` (C, 0 for contact), `tool_normal_policy` (C, `constant` or
  `tilt_at_edges`), `max_edge_tilt_deg` (C), `tool_frame_yaw_deg` (C), `target_force_n`, `force_band_n`,
  `force_zero_threshold_n`, `standoff_tolerance_m`, `approach_standoff_m`, `approach_speed_mps`,
  `ee_speed_mps`, `reachability_tilt_deg_max` (0 for contact tools), `settle_timeout_s`, `overload_abort_n`,
  `tangential_torque_abort_nm`, `unexpected_contact_abort_n`, `tool_command` (`none`, `pump`, `valve`),
  `tool_off_lead_s`, `tool_on_at_speed_ratio`.
- `coverage`: `swath_m` (C), `spray_half_angle_deg` (C), `effective_width_fraction` (C), `overlap_pct` (C),
  `passes` (C, list of `standoff_delta_m` and `reverse_direction`), `sweep_direction` (C),
  `enable_boundary_pass` (C), `boundary_offset_m` (C), `obstacle_inflation` (C, `uniform` or `primary_axis`),
  `obstacle_inflation_radius_m` (C), `ensure_coverage_within_boundary`, `uniform_direction`.
- `control`: `controller` (`admittance_chain` or `jtc_only`), `admittance` (axes, mass, damping, stiffness,
  `wrench_reference_z_n`, trim and thresholds), `ft` (filter cutoff, deadband, clamp, tare).

The loader rejects: contact without `admittance_chain` (and the reverse); `wrench_reference_z_n` different
from `target_force_n`; a non-contact tool with `standoff_m <= 0` or with `contact_allowed_links`;
`reachability_tilt_deg_max > 0` on a contact tool; a spray cone on a contact tool, a half-angle outside
(0, 90) deg or a fraction outside (0, 1].

## Contact and non-contact tools in planning

| | Contact | Non-contact |
|---|---|---|
| Path height | on the wall plane | `standoff_m` in front of it |
| Swath | `coverage.swath_m` | `swath(standoff)` from the spray cone when `spray_half_angle_deg > 0`, else `swath_m` |
| Strip spacing | `swath * (1 - overlap/100)` | the same formula with the derived swath |
| Tool orientation | tool Z into the wall; `reachability_tilt_deg_max` must be 0 | tool Z along -normal; may tilt at wall edges (`tilt_at_edges`) |
| Segments | `requires_contact` true: the executor engages and holds force | no force engage; holds the standoff |
| Control | admittance chain on the normal axis | trajectory controller only; any force above `unexpected_contact_abort_n` is a collision |
| Obstacle inflation | usually `uniform` | usually `primary_axis` (the fan is long across the travel direction) |

## Spray swath model

`core_types/sprayer_effective_swath.hpp`. The spray is a cone with half-angle theta from the nozzle. On a flat
wall at standoff d:

```
geometric width   w_geo(d) = 2 d tan(theta)
effective swath   w(d)     = f * w_geo(d)          f = effective_width_fraction, in (0, 1]
line spacing                 = w(d) * (1 - overlap_pct / 100)
```

`f` stands for the Gaussian-like deposition across the cone: the edges get little paint, so only the central
part counts. Example (theta 20 deg, f 0.8): swath 0.175 m at 0.30 m, 0.233 m at 0.40 m; with 20 % overlap the
spacing is 0.140 m and 0.186 m. The coverage planner (`toolSwath`, `toolLineSpacing`) uses this for every
non-contact tool whose profile has `spray_half_angle_deg > 0`, so changing `standoff_m` changes the spacing.
Zero or negative standoff, a half-angle outside (0, 90) deg or a fraction outside (0, 1] throw.

### Deposition and thickness uniformity

`coverage_planner/core/spray_deposition.hpp` adds the deposit across the strip: a Gaussian centred on the path,
truncated at the cone edge `h = d tan(theta)` and normalised to integrate to 1 (`sigma = 0.25 * w_geo`, the edge at two
sigma, 13.5 % of the peak). A pass of length L with the valve open deposits `g(x) * L` at perpendicular distance x
(relative thickness, constant speed). `CoverageVerifier::addSprayStroke` accumulates this per cell for the engaged
(valve-open) stretches of a stroke and the report carries `thickness`: mean, coefficient of variation (CoV = standard
deviation / mean over the covered target cells), min, max. Samples with the valve closed (dock, transit, retract) add
nothing, to coverage or thickness. Pass the footprint to the scorer:

```
ros2 run coverage_planner score_run --no-contact --samples run.csv --wall run.csv.wall.json --swath 0.175 \
  --spray-standoff 0.30 --spray-half-angle 20 [--spray-sigma 0.25] [--max-thickness-cov 0.4]
```

A non-contact scorecard has no force limits (the force statistics are for contact tools); the JSON gains a `thickness`
section only when `--spray-standoff` is given, so a contact scorecard is unchanged.

Limits:

- The width model has no dependence on speed, pressure or wall angle; the thickness model is the stationary Gaussian
  above with no overspray beyond the cone, no flow transient at valve opening and no obstacle shadowing. Spacing is not
  optimised for thickness uniformity. Cells at a stroke's two ends get less than their share (a stroke deposits along its
  length only), so the CoV includes that fall-off at the boundary of the wall.
- `theta` and `f` are guesses until measured on the real nozzle (spray a test wall, measure the width of the
  usable band at two standoffs, fit both).
- Valve timing is coarse: the manager opens the valve once the tool has taken up the first stroke pose and closes it when
  the stroke segment returns. The manager does not yet use `tool_off_lead_s` and `tool_on_at_speed_ratio`; the
  opt-in executor below does. No hardware valve driver exists (only the `std_msgs/Bool` topic).

## Tool actuation executor

`manipulation_manager/core/tool_actuation.hpp` (ROS-free, `tool_actuation_test`) runs a tool window in trajectory
time: on at `on_s`, off at `end_s - tool_off_lead_s` (never before `on_s`); a pump also waits for
`tool_on_at_speed_ratio`. A pause drives the tool off (trajectory time stands still, the schedule resumes there;
`pause_keeps_state` leaves it as is). With an echo wired, an unconfirmed command is resent every `retry_s` and after
`response_timeout_s` the result is `TOOL_NOT_RESPONSIVE`; a tool whose echo departs from a confirmed command is
`TOOL_FAULT`. Cancel, fault and the end of the window all drive the tool off, resent until the echo confirms it. On and
off latency (command to echo) are measured and published, as profile data. Tool I/O is not safety-critical: the
executor reports and never stops the arm.

`tool_actuation_node` is the thin wrapper (not started by the manager): `~/window` (Float64MultiArray `[on_s, end_s]`),
`~/trajectory_time`, `~/paused`, `~/speed_ratio`, `~/cancel`, `~/fault`, echo topic (`echo_topic`, default
`/tool/valve_state`, empty for none) in; `valve_topic` (`/tool/valve`) and `~/status` out. The valve is closed at start
and on node shutdown. Not yet done: the manager does not publish the window or trajectory time, and nothing has run
against a real valve.

## Non-contact column in the manager

`manipulation_manager` parameter `contact` (default `true`, the contact flow unchanged). With `contact: false`
(`core/tool_mode.hpp`): the coverage plan already carries the standoff (the planner lifts the poses `standoff_m` in front
of the wall for a non-contact tool), so a column is dock (fsp, `dock_standoff_m` further back) -> move to the first stroke
pose with the valve closed -> valve ON -> Cartesian stroke along the column -> valve OFF -> retract. No admittance engage,
no force target, no press and settle, no `allow_target_contact` / penetration allowance, no `contact_y_overshoot_m`;
`spray_extra_standoff_m` (default 0) moves the stroke further from the wall, never closer. The valve is
`std_msgs/Bool` on `valve_topic` (default `/tool/valve`, reliable and transient local), closed at start and after the plan;
`record_run.py` records it as the `active` column. MuJoCo run: `run_spray_pass.sh` (`docs/sim_mujoco.md`).
- Obstacle inflation is a fixed margin, not derived from the swath unless left unset.

## How to add a tool

Most tools need no code.

1. Copy the closest entry in `tool_profiles.yaml` (or start a new file and pass `tool_profiles_file`).
2. Rename it, set the geometry (`tcp_xyz/rpy`, `contact_allowed_links`), the task and coverage values, and the
   control block. Keep the invariants above in mind: a contact tool uses `admittance_chain` and
   `wrench_reference_z_n == target_force_n`; a non-contact tool has `standoff_m > 0`.
3. A non-contact tool with a cone spray sets `spray_half_angle_deg` and `effective_width_fraction`. A flat
   fixed-width tool leaves `spray_half_angle_deg` at 0 and sets `swath_m`.
4. Run: `ros2 launch coverage_planner wall_coverage_demo.launch.py tool_name:=<name>`.
5. Add the tool mesh and the TCP offset to the robot description if the tool is not already there.

Code is needed only when the tool does something the profile cannot say: a swath law other than a cone (add
a function next to `sprayerEffectiveSwath` and have `toolSwath` in `tool_coverage_profile.cpp` choose it by a
new field), a new `tool_normal_policy`, a new `tool_command` (the executor must implement it), or a new
control scheme (`controller`). New fields go in `ToolProfile`, the YAML loader and its test.
