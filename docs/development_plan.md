# Development plan

What is planned next, in the order it should land, with the design each theme follows. Written 2026-11-21 from a
read-only review of the code for each topic; file references are to the tree at that date. Status of every
capability stays in [ROADMAP.md](ROADMAP.md); this page is the sequence and the design. Each theme is one pull
request (or a short stack), with its tests, docs and changelog entry inside it.

## 0. Scoring must measure the tool, not just a force

Finding: the run scorecard counted coverage from the tool point and a force of at least 1 N. It never checked the tool's
orientation or its distance from the wall, so a tilted or off-wall tool could score as covered.

- Scorer: samples carry `tilt_deg`; `max_wall_gap_m`, `max_tilt_deg` and `max_bad_pose_fraction` refuse samples where
  the tool is off the plane or tilted, and the card reports tilt and gap statistics (`score_run --max-wall-gap
  --max-tilt --max-bad-pose`). The recorder logs the tool axis angle from TF.
- Then: the plane comes from a JSON file, not the constant `x = 0.98`, and the allowed tilt is the tool's intended tilt
  (zero for a flat pad), so ceilings and angled tools score correctly (sections 2 and 3).
- Measure a real pass, find the cause of any tilt or gap, fix it, and re-film the demos from the corrected runs.

## 1. Repository and release hygiene

CI on free GitHub-hosted runners only (build, colcon tests, readiness scan); simulators and GPU stay local. Issue and PR
templates. A README per package and `docs/modules.md` (module map and recipes). Run the release-checklist scan, settle the
maintainer-address and licence-header questions, tag `v0.1.0`, publish images. A support matrix of what is verified in
simulation, on GPU and on hardware (nothing on hardware yet).

## 2. Plane-generic pipeline and the ceiling

A ceiling is the same pipeline as a wall with a different plane. Already generic: `liftCoveragePathPose`, `SurfacePatch`
(floor and ceiling), the rasterizer, the scorecard core, the manager's `WallDirection`.
Still assuming a vertical wall, to fix: `UnitZ x n` in `reachability/src/core/station.cpp:17,43` (zero for a ceiling),
`global_stands.cpp:28`, `stand_cover.cpp:48`, `column_stands.cpp:286`; the manager's lateral grouping and `StanceWall`;
names `wall_x/y/z`; the recorder and scripts with `0.98`. Plan: a plane type through `chooseStation`, manager
generalisation with wall output unchanged, a MuJoCo ceiling and floor scenario, then the doc "how a ceiling differs": reach
with the lift raised and the arm overhead, gravity on the tool and the sign of the press, spray drip, base under the patch,
stroke direction, self-collision. Written last, from what the pass showed.

## 3. Offset and tilted tools, with force

`tcp_link` is defined as the contact frame with +Z into the wall at the nominal pose. Tool geometry gets a contact frame,
an intended tilt and tilt axis, owned by the tool profile and fed to the URDF generator (today `tcp_xyz` in the profiles and
`tool_profiles.yaml` are two unlinked sources, and the latter is read by nothing). The lift applies the tilt; the swath and
spray footprint are projected for a tilted pad; the wrench is already moved to the contact frame (`moveWrench`), so add
checks that the conditioner's tool frame equals JTAC's tip, tool payload, and tilted-offset tests. Maps are per tool (cache key
includes the tool offset). One MuJoCo scenario: a 30 degree angled tool pressing a wall.

## 4. Splits, IK branches and column feasibility

Today IK is solved per pose, so a column can be "whole" in the reachability map while no continuous joint path exists, and
a branch flip appears only as a jump truncation, a retry or a skipped column, uncounted. Plan: a ROS-free `ColumnIkChain`
(seed each pose from the previous solution along approach, contact, strokes and retract; several folded seeds), a branch
signature and a continuity ratio `max |dq_i| / jump_i` with per-joint-type thresholds, a whole-column check before commit
(`PoseFailure::kBranchBreak`), and escalation: reseed or change lift height, then re-stand, then re-order columns, split
only as the last resort. Report `split_count`, `branch_changes`, `max_continuity_ratio`, `lift_travel_m` and execution-time
splits in the column analysis, scorecard and verify result. The `ColumnReachabilityPort` has no adapter yet; wiring it enables
re-stand. Doc: "How splits are avoided". Closest published method: Descartes-style layered search over IK solutions.

## 5. Singularities and joint limits

Existing but unwired: `minSingularValue`, `jointLimitMargin`, `MarginConfig` (both thresholds default to 0, nothing sets them).
Plan: add condition number and manipulability; a post-IK `PathQualityPort` audit of every stroke (min sigma, min limit margin,
worst index, reason `near_singular` / `near_limit`) in the Cartesian planner and a manager gate; profile margins and thresholds
wired into the stand verifier and into the cuRobo guard (which checks velocity and acceleration only today); a damped
least-squares plus null-space-centering IK decorator with velocity-constrained lift bounds; an opt-in JTAC monitor that slows
and then faults. Report min sigma, margins, lift travel and gate counts per pass, with an A/B on the same wall.

## 6. Base placement at corners and edges

The base footprint is checked against one infinite plane only. Plan: a `free_space` input (polygons or a grid plus obstacles),
the footprint inflated and tested over all yaw samples for every candidate; arm and tool versus an adjacent wall prefilter;
a nav-feasibility connectivity check with coded reasons; then a pure-C++ property harness (randomised corner angle, wall
length, patch offset, reach and footprint; real coverage planners in the loop; metrics: coverage, stand count against the
exact-search minimum, splits, travel, clearance) and new sim scenarios (inside corner, outside corner, wall end, opening,
column, alcove, corridor). Verify-and-nudge refinement afterwards, behind an option.

## 7. Base docking and navigation

Two meanings of dock: the base stopping at a stand (`stand_navigator`, `WallDock`) and the arm's dock pose (manager).
Gaps: no check of the achieved stand pose against the reachability margin before the arm moves, no lateral control in
`WallDock`, no re-dock or alternate-stand policy, `base_check` tolerance 0.15 m, no odom noise in MuJoCo. Plan: noise and a
metrics harness first (N of at least 30, error percentiles), a stand-check gate with a margin-derived tolerance, a lateral
term, a re-dock and abort policy, then Nav2 `NavigateToPose` and an `opennav_docking` plane-dock plugin. The Gazebo two-stand
failure is the arm dock on stand 1 (seeded dock IK, start state at a limit), not navigation.

## 8. Control: gravity, teleoperation, impedance

JTAC commands position and velocity only. Order: gravity first (a `contact_free` publisher so the bias tracker runs,
per-body `gravcomp` in the MuJoCo generator, pose-sweep residual metrics); a `teleop_bridge` package (joystick to Cartesian
and joint jog through MoveIt Servo, an AUTO / TELEOP / HOLD mode manager arbitrated by the contact state machine);
effort actuators in MuJoCo; then a `joint_impedance_controller` plugin with torque gravity compensation. Real-hardware
impedance needs an effort command interface the arm driver may not offer.

Reference studied for the impedance work: [wengmister/impedance_control_mujoco](https://github.com/wengmister/impedance_control_mujoco)
(Python, MuJoCo, xArm 7 and Franka from MuJoCo Menagerie). It shows joint-space impedance (`tau = -Kp(q - q_des) - Kd(qd - qd_des) + bias`
with `mj_rne` as gravity and Coriolis compensation), task-space impedance (`tau = J^T F + J_r^T M`), a null-space torque projector,
a velocity admittance law, and bilateral teleoperation with an end-effector proxy for arms with different kinematics. Its notes record
two MuJoCo traps that apply to our effort-actuator step: `ctrlrange` must match `forcerange` (the default of +-1 saturates torque
commands) and `biastype="affine"` actuators carry a hidden PD term unless `biasprm` is zero. **The repository has no licence**, so no
code is copied into this Apache-2.0 repository: it is a design reference and a behavioural cross-check (our controller is written
independently from the published equations and compared against it on the same arm), and the author can be asked to add a licence if
direct reuse is wanted. It is a single-threaded Python playground without ros2_control, a hardware path or inertia shaping, so
it does not replace the planned controller plugin.

## 9. Cobot operation: whole-robot collision monitor and safety layer

Today there is no runtime monitor: collision checks (MoveIt, cuRobo, VAMP) run at plan time on a snapshot, `robot_self_filter`
only removes points, the MuJoCo `scanUnexpectedContacts` logs and never stops anything, and nothing ties the per-tool abort
thresholds to an e-stop path. Hardware safety stays the vendor controller's job. Plan: a `safety_monitor` package (ROS-free
core plus a thin node) with ports for the robot sphere model (one model shared with cuRobo and VAMP), obstacle sources (the
static ESDF, live clouds through the self filter in monitor mode, a people list, stubbed until a detector exists) and the
SRDF allowed-collision pairs. It checks the swept volume over the stopping horizon and publishes `/safety/speed_scale`
(speed-and-separation form, ISO/TS 15066) and a latched `/safety/stop_request` with a reason; JTAC stretches its time base and
the manager aborts the segment, both failing safe on stale data. Contact detection by external-torque residual or the F/T
residual, with a reaction ladder: stop, retract, compliant yield. The tool body is exempt on the wall. Metrics: intrusion-to-stop
latency (p50, p99), stopping distance against speed, false-stop rate over normal passes, missed detections on injected
obstacles. **This is not a certified safety function:** a deployment needs a safety-rated e-stop and protective stop, a safety
PLC or controller-level safety, safety-rated people sensing and a risk assessment (ISO 12100, 10218-2, TS 15066); the docs and
README say so. The monitor's job is earlier, gentler slowdowns so the hardware limits rarely trip. Order: core and unit tests,
publish `scanUnexpectedContacts` as an oracle topic, the node with the static source and heartbeat, JTAC and manager
consumers, live clouds, residual contact detection, the people source and full metrics.

## 10. Differential-IK (Cartesian tracking) controller

JTAC is not chainable and its weighted redundancy path (`enable_redundancy_weighting`) is undamped, position-only and breaks
near a singularity. Strokes are planned with single-seed IK per 1 cm step and truncated on a jump. Plan: a ROS-free
`diff_ik_core` (damped least squares with singular-value-adaptive damping, null-space joint centering, limit and manipulability
terms, lift handled as a cost, uniform task scaling at limits, so the path direction is kept), then a chainable
`cartesian_tracking_controller` plugin that tracks a pose path with feedback and feed-forward, so branch flips and jump
truncation disappear for strokes (the joint path is continuous by construction) and failures become measured quantities
(tracking error, sigma_min, margin, scale). Force-aware: motion along the contact tangents, admittance along the normal, in
Cartesian space (this retires JTAC's weighted path later). The planner keeps approach, retract and fallback. MoveIt Servo is one
optional source of twist references for teleop. Needs an offline linear stability analysis over a contact-stiffness sweep before
hardware, and checks that the target drivers honour velocity commands at 100 Hz. Reuse: `kinematics_interface`, the admittance
controller's FT handling; `mink` and `pink` only as cross-checks (LGPL for `pink`: do not link).

## 11. Run-time tool correction

Built on section 3. Present: `pivotCalibration` (TCP position, no node), payload identification in `ft_conditioner` (the
excitation plan is never executed), a bias tracker whose `contact_free` gate nothing publishes, the plane fit in
`wall_detector`. Missing: tilt calibration, an excitation executor, a calibration record, wear tracking. Plan: wire
`contact_free` and expose the capture parameters; a `ToolCalibration` block in the tool profile (TCP, tilt, payload, bias, method,
date, residuals, validity) that the loader checks for contact tools; `planeTouchCalibration` (a touch at 20 degrees or more of
orientation variation fixes the TCP along the normal); a calibration stage in the manager; an excitation executor with
forward kinematics and a collision proxy; tilt from a plane normal at several commanded tilts (tilt from force alone is
confounded by friction and needs validation on a real pad); plane-relative standoff and tilt servo online; wear flagged from
contact-depth drift, not compensated. Proof: inject a tip offset, a tilt, a mass error and a bias into the MuJoCo model and
require recovery (1.5 mm, 0.5 degrees, 1 %), with a seed sweep, published in the scorecard.

## Sequence (all themes)

1. Scoring (section 0), then the measurement and fix of the demo tilt.
2. Hygiene and release (1), in parallel; plane-generic and ceiling (2); tool model (3).
3. Splits and IK chain (4) with singularity audit (5), sharing the path-quality port; `diff_ik_core` (10) reuses the same measures.
4. Base placement (6) and docking (7).
5. Safety monitor core and oracle (9), tool correction (11), gravity and teleop (8), diff-IK controller (10).
6. Impedance control, only with an effort-capable arm.

## Working rule: adopt industry practice first

For every open design question the default is what industry and maintained open source already do: reuse or adapt it,
cite it in the [ROADMAP reuse map](ROADMAP.md) (section 17) with its licence, and write our own only where nothing fits
(JTAC was built that way, after studying a reference design). Decided on that basis:

- **Calibration fixture:** allowed and preferred. Industry TCP calibration uses a fixed pointer, plate or cone; the wall-touch
  method stays as the field option. Tilt is found from a measured plane at several commanded tilts, never from force alone.
- **Pad:** treat it as compliant by default (industry contact tools for sanding, polishing and wiping are compliant or
  force-controlled flanges) and keep rigid pads working through the plane-touch method.
- **Hand-eye and sensor extrinsics:** in scope as adoption, not development: use maintained packages (MoveIt Calibration,
  `easy_handeye2`, `robot_calibration`; licences to verify) and consume their result; do not write our own solver.
- **Safety:** follow ISO 10218 / ISO/TS 15066 structure (speed and separation, power and force limiting), implemented as a protective layer on top of the
  robot controller's certified safety, never replacing it.
- **Controllers, docking, navigation:** ros2_controllers, Nav2 and `opennav_docking`, MoveIt Servo, `kinematics_interface` first;
  our own code only for the contact-stroke behaviour no package provides.

## Decisions pending

- Is a free-space reposition inside a column acceptable (split as fallback), or must a column be one contact pass?
- Is `split_count` a hard gate or reported only? Are quality gates on by default or opt-in per profile?
- May one stand wrap an outside corner, or does each face get its own stand?
- Does the target arm expose torque control? If not, impedance stays simulation-only.
- Where does room geometry come from at run time (scene model, wall detector, supplied polygon)?
- Which arm and controller, and is its safety I/O reachable from ROS? Which people-detection source will exist?
- Do the target drivers honour velocity commands at 100 Hz? One controller (replace JTAC admittance) or two chained?
