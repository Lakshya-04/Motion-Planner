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

- `geometry`: `mesh`, `mesh_origin_xyz/rpy`, `tcp_xyz/rpy` (flange to the contact frame `link_tcp`, +Z into the work), `tilt_deg` and `tilt_axis` (intended
  tilt, see "Contact frame, tilt and offset tools"), `contact_allowed_links`
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
  `obstacle_inflation_radius_m` (C), `ensure_coverage_within_boundary`, `uniform_direction`, and for contact
  tools (opt-in) `contact_area` (C, see below), `edge_zone_m` (C), `edge_spacing_scale` (C).
- `control`: `controller` (`admittance_chain` or `jtc_only`), `admittance` (axes, mass, damping, stiffness,
  `wrench_reference_z_n`, trim and thresholds), `ft` (filter cutoff, deadband, clamp, tare).

The loader rejects: contact without `admittance_chain` (and the reverse); `wrench_reference_z_n` different
from `target_force_n`; a non-contact tool with `standoff_m <= 0` or with `contact_allowed_links`;
`reachability_tilt_deg_max > 0` on a contact tool; a spray cone on a contact tool, a half-angle outside
(0, 90) deg or a fraction outside (0, 1]; `contact_area` or `edge_*` on a non-contact tool, a
`contact_area` with a factor, tilt, stiffness or curvature out of range, `edge_spacing_scale` outside (0, 1];
`geometry.tilt_deg` not finite or `|tilt_deg| >= 80`, a tilt without an in-surface `tilt_axis`, a tilt on a
contact tool, a tilt together with `contact_area.nominal_tilt_deg`, and `tilt_deg + max_edge_tilt_deg >= 85` with
`tilt_at_edges`.

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

## Contact frame, tilt and offset tools

`geometry.tcp_xyz` / `tcp_rpy` are the CONTACT frame (`tcp_link`) relative to the flange, not just "the tool
tip": its origin is the point that touches the work and its +Z points into the work at the nominal pose. An
offset or angled tool (a side-mounted pad, an angled sander) puts its offset and angle in these two fields,
so the controller, the force path and the coverage lift all see the same frame. The loader does not yet
compare them with the running URDF (`tcp_link` there is a separate source until the URDF generator reads the
profile).

`geometry.tilt_deg` and `geometry.tilt_axis` (both optional, default 0) are the intended angle between the tool
axis and the surface normal, and the in-surface axis the tool tilts about, given in the contact frame as
`[x, y, 0]` (any length; the loader normalises it). A negative tilt is stored as the positive tilt about the
opposite axis. The loader rejects non-finite values, `|tilt_deg| >= 80`, a tilt without an axis and an axis
with a z component. For now a tilt is refused on a contact tool (`task.contact: true`): the force path still
presses along the tool Z, so a tilted contact tool would press with `F cos` and shear with `F sin` until the JTAC
and conditioner frame checks exist. A tilt together with `contact_area.nominal_tilt_deg` is refused (both would
narrow or widen the same swath), and so is a fixed tilt plus `max_edge_tilt_deg` of 85 degrees or more. The two
shipped tools have no tilt, and with `tilt_deg == 0` nothing changes: the lift output is bit-identical.

What the tilt does:

- Lift (`wall_pose_lift`): the nominal tool Z (-normal) is rotated by `tilt_deg` about the axis (rotated with
  `tool_frame_yaw_deg`, because the axis belongs to the tool). The standoff is measured along the tilted axis:
  the nozzle sits at `strip point - axis * standoff`, so the axis meets the wall at the strip point (at tilt 0
  this is the plain offset along the normal, bit-identical). The lift clamps the axis to 85 degrees from
  into-wall as a defensive bound (the loader already refuses larger totals). The edge tilt of a non-contact tool (`tilt_at_edges`) composes
  afterwards: the intended tilt is applied first, the edge tilt rotates that axis toward the wall edge.
- Swath: a tilted flat pad's footprint is stretched by `1/cos(tilt)` along the tilt direction (perpendicular to
  the tilt axis, in the surface). The swath across the stroke becomes `w (1 + k sin^2 phi) + l |k sin phi cos phi|`
  with `k = 1/cos(tilt) - 1`, `phi` the angle from the stroke to the tilt direction, `w` the pad width and `l`
  the pad length. Tilt along the stroke leaves `w`, tilt across it gives `w / cos(tilt)`
  (`core_types/tool_footprint.hpp`). `toolSwath` applies it to the spray cone of a non-contact tool;
  the contact-area model keeps its own width model (a fixed tilt is refused on a contact tool) and a fixed
  `swath_m` is left as the author set it. The panel and bottom rows of a wall pattern take the swath from the
  tool as turned for those rows (horizontal sweep, yaw plus `panel_tool_turn_deg`), not from the tool's own sweep
  direction. For the spray cone this is a first-order approximation (the
  cone asymmetry and the longer slant range are ignored). `contact_area.nominal_tilt_deg` is the older, separate
  narrowing for a tilted wall; the loader refuses it together with `geometry.tilt_deg`.
- Force path: the wrench is already moved from the sensor to the tool frame by `force_core::moveWrench` with the
  conditioner's `tool_in_sensor` offset, so it is expressed at the contact frame provided `tcp_link` is the
  contact frame. A tilted tool's normal force is then along the tool Z, not the wall normal; the JTAC
  and conditioner frame checks (the conditioner's tool frame equals the controller's tip, the tilt in the
  force reference) are a later step and are not made yet.

Not done: maps per tool offset (cache key), the URDF generator reading `tilt_*`, the MuJoCo angled-tool
scenario, anything on hardware.

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

## Contact-area model (mop, wiper, sander)

`core_types/contact_area.hpp`, off by default. A compliant pad touches a wider strip when pressed harder, a
narrower one when tilted or on a convex surface. Add `coverage.contact_area` to a contact tool and the planner
uses the effective width at `task.target_force_n` as the swath (`toolSwath`), so spacing follows the force:

```
w(F)   = pad_width_m * clamp(1 + width_gain_per_n * (F - reference_force_n), min_width_factor, max_width_factor)
w_tilt = w(F) * cos(nominal_tilt_deg)
w_conv = min(w_tilt, 2 * sqrt(2 * d / kappa))     d = F / pad_stiffness_n_per_m, kappa = surface_curvature_per_m
spacing = w * (1 - overlap_pct / 100)
```

```yaml
coverage:
  swath_m: 0.20                  # kept as the fallback; ignored while pad_width_m > 0
  overlap_pct: 20.0
  contact_area:
    pad_width_m: 0.20            # width measured at reference_force_n; 0 = model off
    pad_length_m: 0.10           # along travel; only used for contactPressurePa
    reference_force_n: 10.0
    width_gain_per_n: 0.01       # +1 % width per N (fit it: press at 2-3 forces, measure the wiped strip)
    min_width_factor: 0.5
    max_width_factor: 1.5
    pad_stiffness_n_per_m: 0.0   # > 0 enables the convex-curvature cap
    nominal_tilt_deg: 0.0
    surface_curvature_per_m: 0.0
  edge_zone_m: 0.15              # strips this close to a boundary / obstacle edge are packed tighter
  edge_spacing_scale: 0.6        # spacing factor in that zone, in (0, 1]; 1.0 = uniform (default)
```

An explicit `swath_m` node parameter switches the model off (the user chose the width). Edge adaptation is
`contactStripOffsets` (`coverage_planner/core/contact_spacing.hpp`): strip centres over a range, spacing times
`edge_spacing_scale` where a strip starts within `edge_zone_m` of the range ends or of a listed obstacle edge.
Spacing never exceeds the uniform one. Limits: the model is static and quasi-flat (a fitted gain, not a contact
mechanics solver); the boustrophedon (columns and cells) and sweepline planners apply the same rule on their grid through
`CoveragePlannerParams::edge_zone_m` / `edge_spacing_scale` (`stripCellPositions`; node parameters of the same
names, default 0 / 1.0 = uniform, plans unchanged; `test_edge_spacing_planner`). Only the two ends of the stepover
axis (of a cell, for the cell decomposition) open a zone there. In the sweepline planner and in the boustrophedon column
sweeps the borders of every obstacle (the lowest and highest stepover position of each blocked region inside the free
span, `obstacleEdgeCells`) open the same zone; the cell decomposition already ends its cells at obstacle borders, so
its zones sit there without extra input. Both values are in the tool profile and in `ToolOverrides` / node parameters of
the same names (`edge_zone_m`, `edge_spacing_scale`); `wall_coverage_service_node` and `wall_coverage_demo_node` copy the
profile's values into the new `edge_zone_m` / `edge_spacing_scale` fields of `GenerateCoveragePath` (zone <= 0 or scale
outside (0, 1) = use the planner node's parameter, `resolveEdgeSpacing`). Shipped profiles are unchanged (uniform). Not
verified: the effect on a real wall's coverage or on a run, only that the generated strip positions are finer near the
obstacle borders (`test_edge_spacing_planner`).

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
and on node shutdown.

Feeding it from the manager (opt-in, `contact: false` only): `tool_actuation_feed:=true` (and `tool_actuation_node`,
default `tool_actuation`) wraps the execution port in `ToolFeedExecutionPort`
(`core/execution/tool_feed_execution_port.hpp`, `tool_feed_execution_port_test`). During a spray stroke the trajectory is
the tool window: the manager publishes `[0, last time_from_start]` to `<node>/window`, then the trajectory time to
`<node>/trajectory_time` at 50 Hz, the end time on success and `<node>/cancel` on failure or exception. The manager stops
opening the valve itself (it only sends "off" around the stroke as a backstop), so start `tool_actuation_node` with the
same `valve_topic`. Limits: the trajectory time is the elapsed wall time since the execution call, not the controller's
reported progress; the manager node wiring is built but not run against the node or a sim, and pauses are column-boundary
only so `~/paused` is not fed. No echo in the sim, nothing has run against a real valve.

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
3. A contact tool whose pad width changes with force adds `coverage.contact_area` (above); a rigid pad keeps
   `swath_m`.
   A non-contact tool with a cone spray sets `spray_half_angle_deg` and `effective_width_fraction`. A flat
   fixed-width tool leaves `spray_half_angle_deg` at 0 and sets `swath_m`.
4. Run: `ros2 launch coverage_planner wall_coverage_demo.launch.py tool_name:=<name>`.
5. Add the tool mesh and the TCP offset to the robot description if the tool is not already there.

Code is needed only when the tool does something the profile cannot say: a swath law other than a cone (add
a function next to `sprayerEffectiveSwath` and have `toolSwath` in `tool_coverage_profile.cpp` choose it by a
new field), a new `tool_normal_policy`, a new `tool_command` (the executor must implement it), or a new
control scheme (`controller`). New fields go in `ToolProfile`, the YAML loader and its test.
