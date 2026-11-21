---
type: Reference
title: safety_monitor
description: Package README for the ROS-free core of a whole-robot proximity monitor (sphere model, obstacle sources, speed-and-separation speed scale, latched protective stop). Not a certified safety function.
tags: [safety, collision-monitor, speed-and-separation, package-readme]
status: experimental
---

# safety_monitor

> **NOT A CERTIFIED SAFETY FUNCTION.** This is ordinary, non-safety-rated software meant to run on a
> general-purpose operating system. Software of that kind cannot reach an ISO 13849 performance level,
> whatever its tests say. A deployment around people needs a safety-rated e-stop and protective stop,
> a safety PLC or controller-level safety functions, safety-rated people sensing, and a risk assessment
> (ISO 12100, ISO 10218-2, ISO/TS 15066). **This layer only slows the robot earlier**, so that the
> hardware limits trip rarely; it never replaces them and nothing may rely on it to prevent harm.

ROS-free core (Eigen and yaml-cpp only, no clock, no I/O) of the whole-robot collision monitor planned in
`docs/development_plan.md` section 9. The ROS node, the controller and manager consumers are later steps.

## What's here

All in namespace `safety_monitor`, headers under `include/safety_monitor/`:

- **`sphere_model.hpp` -- `SphereModel`**: per-link sphere lists (centre in the link frame, radius);
  `worldSpheres()` turns caller-supplied per-link transforms (from the caller's own FK) into world
  spheres; `attachSphere()` / `detachSpheres()` add tool or payload spheres at run time under a tag; the
  allowed-collision table (`allowPair()`, parent-child excluded automatically, `selfPairs()` lists what
  is checked); `setContactExempt()` marks the tool body that may touch the work surface;
  `sphereVelocities()` builds sphere velocities from per-link twists.
- **`distance_source.hpp` -- `DistanceSource` port** (signed distance, unit gradient, obstacle velocity;
  `query(point, t_ahead)`) with `StaticPrimitiveSource` (sphere, oriented box, half-space; may be flagged
  as the work surface), `EsdfGrid` (dense signed-distance grid, trilinear lookup, independent of
  `scene_model`; an adapter from its ESDF is a later step; points outside the grid answer "no
  information" only when `setOutsideBehavior(OutsideGrid::kFree)` is chosen; the default answers a NaN
  distance there and the monitor stops; a NaN or infinite voxel also stops) and `MovingCapsuleSource` (a person proxy: capsules with a velocity, predicted at
  constant velocity).
- **`safety_monitor.hpp` -- `SafetyMonitor::evaluate()`** returns `Result{speed_scale, state, reason,
  worst_sphere, worst_link, min_distance}`.
- **`heartbeat_watchdog.hpp` -- `HeartbeatWatchdog`**: `beat(t)`, `expired(t)`; never beaten, bad time
  or a clock running backwards counts as expired.

## Reuse of the planner models

The monitor does not duplicate the robot and world models; it takes them through small seams, and the
package depends on neither `motion_planner` nor MoveIt.

- **Robot spheres**: `loadSphereModelFromCuroboYaml()` (`curobo_yaml.hpp`, yaml-cpp) builds the
  `SphereModel` from the collision-sphere table of a cuRobo robot yml, the same table the cuRobo
  planner and `docker/tools/generate_curobo_robot.py` use (`kinematics.collision_spheres`,
  `collision_link_names`, `self_collision_ignore` as allowed pairs; the URDF parent map is optional
  input). One sphere table can then serve cuRobo, VAMP-style planners and this monitor.
- **Distances**: `DistanceSource` is the port a future adapter implements over the planner backends
  (MoveIt distance queries, cuRobo or VAMP spheres) and over `scene_model`'s ESDF (`computeEsdf`); the
  built-in `EsdfGrid` is the stand-in until that adapter exists. `CollisionPort` in `motion_planner`
  returns only `collision_free`, no distance, so a `DistancePort` next to it is the planned extension.
  None of these adapters exist yet.

## The monitor

For each robot sphere and each source, over the stopping horizon (spheres advanced along their velocity
at `sweep_samples + 1` times, radius inflated by `sweep_radius_inflation_m`, a moving source advanced
too):

```
d_stop = v_human * (T_stop + T_reaction) + v_toward * T_reaction + margin
d_slow = d_stop + slow_band
scale  = clamp((d - d_stop) / (d_slow - d_stop), 0, 1)
```

The form is the ISO/TS 15066 separation distance: `v_human * (T_stop + T_reaction)` is the
distance the person covers while the system reacts and stops (S_h), `v_toward * T_reaction` the robot's
travel during the reaction time (S_r), the robot's stopping travel (S_s) is covered by the sweep over
`[0, T_stop]`, and `margin` stands for C + Z.
`v_human` is 0 for static sources and `max(human_speed, measured)` for a person; `margin` is intrusion
plus uncertainty for a person and a smaller value for static obstacles; the person slow band is wider.
Self-collision uses the same form with the pair closing speed. The result scale is the minimum; 0 is a
protective stop. Person sources are queried at `now - stamp` ahead, so their position is extrapolated
over the age of their data. `Params` and `HeartbeatWatchdog` are validated in the constructors
(finite, non-negative times, speeds, margins and bands; std::invalid_argument otherwise). Defaults are in `Params` and are starting values, not calibrated.

- **Latch**: a protective stop holds until the hazard is gone, has stayed gone for `min_clear_time_s`
  and `reset()` was called after it cleared (a reset during the hazard is discarded).
- **Fail-safe**: a missing, negative, NaN or future-dated (more than 1 ms ahead of now) stamp, input older than `stale_timeout_s`, a stale or
  unstamped source, a NaN in transforms, velocities or distances, or a size mismatch gives scale 0,
  `kProtectiveStop`, reason `kStaleData` or `kInvalidInput` (and latches).
- **Contact exemption**: `contact_exempt` links are skipped against the source flagged as the work
  surface only, never against persons (even a person source flagged as work surface) or other obstacles.

## Status and limits

- Math only: no node, no `/safety/speed_scale` or `/safety/stop_request` topics, no consumers in the
  trajectory controller or the manager.
- Unit-tested with synthetic poses (67 gtests); **not run in any simulator or on hardware**; thresholds
  and timings are not calibrated and no latency or false-stop metrics exist.
- Constant-velocity prediction only; no braking model of the real arm; one frame; sphere
  approximations of the robot are conservative only as far as the spheres cover the links.
- The caller owns forward kinematics, sphere velocities, time stamps and the people detection.

```bash
colcon test --packages-select safety_monitor   # ROS not required beyond ament_cmake
```
