# Learned and optimization-based recovery

Research notes on getting the robot from an unknown or off-nominal state back to a known
state in the stack (after a fault, e-stop, collision, lost contact, partial trajectory, or an
unplanned configuration near a wall), and where RL, imitation learning and MPC fit.
Written 2026-10-30 from a literature search.

**Evidence marks.** [V] = checked by web search/fetch while writing; [M] = cited from memory
(well-known, but not re-checked). No production report of a learned manipulator recovery
policy with uptime or recovery-rate numbers was found. Everything learned below is
research-grade; the deployed practice is scripted and planner-based recovery.

## 1. Research landscape

**Safe RL with an explicit recovery policy**
- *Recovery RL* (Thananjeyan et al., RA-L/ICRA 2021, [arXiv:2010.15920](https://arxiv.org/abs/2010.15920)) [V]:
  a task policy plus a recovery policy that takes over when a learned safety critic predicts a
  constraint violation. 2-20x better violation/success trade-off in sim, 3x on hardware,
  including contact-rich tasks. The critic is learned and has no guarantee.
- *RecoveryChaining* (MERL, [arXiv:2410.13979](https://arxiv.org/abs/2410.13979), RSS 2025 workshop) [V]:
  hierarchical RL learns local recovery policies that fire on detected failure **and learns
  which nominal controller to hand back to**, which is our "return to a known state" problem.
  Sim-to-real on a Mitsubishi Assista arm without real-world fine-tuning. Closest match.
- *Leave No Trace* (Eysenbach et al., ICLR 2018, [arXiv:1711.06782](https://arxiv.org/abs/1711.06782)) [M] and
  *EARL* ([arXiv:2112.09605](https://arxiv.org/abs/2112.09605)) [M]: learned reset policies; mostly a way to
  automate data collection, not a deployable safety mechanism.

**Recovery from demonstrations (imitation learning)**
- HG-DAgger ([arXiv:1810.02890](https://arxiv.org/abs/1810.02890)) [M], ThriftyDAgger ([arXiv:2109.08273](https://arxiv.org/abs/2109.08273)) [M]:
  human takeover segments are exactly the recovery data.
- *IntervenGen* (IROS 2024, [arXiv:2405.01472](https://arxiv.org/abs/2405.01472)) [V]: ~10 human
  interventions expanded synthetically, up to 39x more robust to pose error; limited to
  quasi-static rigid-body tasks.
- CCIL ([arXiv:2310.12972](https://arxiv.org/abs/2310.12972)) [M]; RACER (ICRA 2025,
  [arXiv:2409.14674](https://arxiv.org/abs/2409.14674)) [V]: recovery trajectories + language, VLM supervisor.

**Failure detection and reasoning (LLM/VLM)**
- REFLECT ([arXiv:2306.15724](https://arxiv.org/abs/2306.15724)) [V], AHA ([arXiv:2410.00371](https://arxiv.org/abs/2410.00371)) [V],
  Agia et al. ([arXiv:2410.04640](https://arxiv.org/abs/2410.04640)) [M].
- *FAIL-Detect* (RSS 2025, [arXiv:2503.08558](https://arxiv.org/abs/2503.08558)) [V]: runtime failure
  detection for imitation policies without failure data, using conformal prediction for
  false-positive guarantees.
- Seconds-scale, task-level ("what went wrong, which skill next"). Use for diagnosis and
  strategy choice, never for joint-level motion near a wall or for safety.

**Safety filters (the guarantee layer)**
- Hsu, Hu, Fisac, *The Safety Filter* (Annu. Rev. CRAS 2024, [arXiv:2309.05837](https://arxiv.org/abs/2309.05837)) [V]:
  unifies CBF, Hamilton-Jacobi reachability and predictive (MPC) filters. Best entry point.
- CBFs: Ames et al. ([arXiv:1903.11199](https://arxiv.org/abs/1903.11199)) [M].
- For diffusion policies: PACS (ICRA 2026, [arXiv:2511.06385](https://arxiv.org/abs/2511.06385)) [V],
  up to 68% higher success than reactive CBF filtering; LatentCBF
  ([arXiv:2511.18606](https://arxiv.org/abs/2511.18606)) [V]. Both research-only.

**Contact-rich recovery.** Nothing found specific to "lost contact while wiping a wall".
In practice contact loss is handled by the admittance controller and the task state machine:
re-approach along the surface normal, then re-establish the force set-point.

## 2. What industry does today

- **Retrace the interrupted path.** ABB RAPID `StopMove` / `StorePath` / `RestoPath` /
  `StartMove`; after an e-stop the controller slowly returns the robot to the path behind a
  confirmation dialog [V]. KUKA has an equivalent (unverified, third-party sources only).
  Logging the executed path and retracing it backwards is the standard recovery primitive.
- **Scripted "safe home" routines** with zone checks, bounded by safety-rated monitoring
  (ABB SafeMove, FANUC DCS, ISO 10218) [M].
- **ROS 2:** Nav2 `RecoveryNode` behavior trees (clear costmap, spin, wait, back up) [V];
  MoveIt 2 Hybrid Planning, where `COLLISION_AHEAD` / `LOCAL_PLANNER_STUCK` events trigger a
  global replan (its own notes admit the local planner can get stuck in local minima) [V].
- **Credible for production soon:** planning-based recovery (cuRobo / VAMP / MoveIt from the
  current state to a standoff), reactive MPC with ESDF costs, runtime monitors with
  statistical guarantees. **Research-only:** RL / imitation policies commanding the arm
  directly, VLM-driven motion.

## 3. MPC for recovery

- MPPI ([arXiv:1707.02342](https://arxiv.org/abs/1707.02342)) [M], STORM
  ([arXiv:2104.13542](https://arxiv.org/abs/2104.13542)) [M]; cuRobo has a
  `model_predictive_control` module and a reactive-control example [V]. The older cuRobo
  MPC docs warn MPC is local and can get stuck near joint limits or behind obstacles, and
  recommend a global planner [V]. Not verified: what optimizer the v2 MPC uses.
- ESDF-aware MPC: Chiu et al. ([arXiv:2202.12385](https://arxiv.org/abs/2202.12385)) [V], Pankert &
  Hutter, RA-L 2020 [V], Koptev et al. neural joint-space SDF, RA-L 2022 [V].
  Reactive geometric controllers: RMPflow, Geometric Fabrics [M].

| | Global planner + MPC | RL policy |
|---|---|---|
| New scene / ESDF | immediate, consumes the map | only within training distribution |
| Constraints | explicit (limits, SDF, torque) | penalties only, no guarantees |
| Failure mode | local minima / planning time | silent out-of-distribution behavior |
| Debugging, certification | inspectable | opaque |
| Best for | geometric recovery (our case) | hard-to-model contact, and choosing *which* recovery |

Our recovery is mostly **geometric** (known model, known map, target set of states): use a
global planner to escape local minima and MPC to track reactively. RL earns its place
where models are poor (re-establishing wall contact) or as a proposal / warm-start.

## 4. Imitation learning

- Diffusion Policy ([arXiv:2303.04137](https://arxiv.org/abs/2303.04137)) [M] and ACT
  ([arXiv:2304.13705](https://arxiv.org/abs/2304.13705)) [M]: roughly 50-200 demos per task in
  single-task real-robot results (order of magnitude from the papers, not a guarantee).
- Recovery states are off-distribution by definition, so nominal demos contain almost none
  and errors compound exactly where recovery is needed.
- Mitigations: DAgger-style intervention logging; synthetic corrections (IntervenGen, CCIL);
  **use our own planner/MPC as the expert**, generating demos from perturbed start states
  in simulation (Isaac Lab, [arXiv:2511.04831](https://arxiv.org/abs/2511.04831) [V]) — far cheaper
  than teleoperation.
- Always wrap: OOD/failure monitor (FAIL-Detect style) + safety filter on outputs + planner
  fallback on rejection.

## 5. Recommended architecture

Principle: **learned components propose; certified components dispose.**

```
Fault/State Monitor -> Recovery Manager (behavior tree, like Nav2 RecoveryNode)
   classify: e-stop / collision / contact loss / partial trajectory / unknown configuration
 -> Strategy selector   : rule table (v1) | learned classifier or VLM diagnosis (later)
 -> RecoveryProposer    : (a) retrace logged path backwards
                          (b) Cartesian retract along -wall normal to standoff
                          (c) global plan (cuRobo / VAMP / MoveIt) to nearest safe target set
                          (d) learned policy / warm-start (later)
 -> Verifier            : full trajectory vs fresh ESDF + self-collision + limits
 -> Executor            : reactive MPC tracking, speed-scaled
 -> Safety filter       : CBF/QP or distance-based velocity scaling at controller rate,
                          plus hardware-rated safety, independent of everything above
 -> Re-entry            : known-state handshake (standoff pose or start of next segment)
```

Stack-specific points:
1. Define "known state" as a **target set** (home, per-segment standoffs, next segment start)
   and let the planner pick the cheapest reachable member.
2. **Log every executed trajectory** with segment IDs so retrace is always available.
3. First move near a wall: drop the admittance controller to low stiffness / zero force,
   then retract along the estimated surface normal. This alone solves most cases.
4. **Refresh the map before planning** (re-fuse depth into the ESDF); the old scene is stale
   after a collision or new obstacle.
5. `RecoveryProposer` is a plugin interface (state + ESDF + target set -> candidate
   trajectory), so scripted, planner and learned proposers swap behind one verifier.

### Roadmap
- **v1 (weeks), deterministic:** BT recovery tree, rule-based fault classes; proposers in
  order retract -> retrace -> global plan to nearest standoff; full-trajectory verification;
  human-confirm gate for the "unknown configuration" class (like ABB's Regain dialog).
- **v2, reactive:** cuRobo MPC tracking, velocity-scaling / CBF filter, whole-body (base +
  lift) recovery.
- **v3, learning as acceleration and selection, not authority:** generate perturbed start
  states in Isaac Lab, label with planner + MPC solutions, train a strategy classifier and
  a distilled policy / warm-start (Diffusion Policy suits multimodal escapes), everything
  still behind the verifier. Log real interventions (HG-DAgger) and augment (IntervenGen).
- **v4, learned control where models are weak:** residual RL on the admittance controller
  for re-establishing contact (Johannink et al., [arXiv:1812.03201](https://arxiv.org/abs/1812.03201) [M]),
  a Recovery-RL-style safety critic as an extra trigger, a VLM layer that only selects
  strategies or flags a human.

### Evaluation
- Scenario bank stratified by fault class: e-stop mid-stroke, collision, contact loss,
  injected obstacle, random configuration within 5 cm of the wall.
- Metrics: recovery success rate, time to known state, minimum clearance, peak contact force,
  safety-filter intervention rate, human-intervention rate, monitor false-trigger rate.
- Hold out scene geometries; a learned proposer ships only if it does not reduce clearance
  or success relative to the v1 baseline; track sim-to-real gap per fault class.

## Priorities

1. Build v1 now (BT manager, retract, retrace, global plan to target set, verified).
2. Keep the safety filter and hardware safety independent of every proposer.
3. Add cuRobo MPC for reactive execution, always with a global-planner fallback.
4. Define the `RecoveryProposer` interface and log every recovery (state, ESDF snapshot,
   outcome) as future training data.
5. Use learning first for strategy selection and planner warm-starts; direct learned control
   only where contact models fail.
6. Add monitors with statistical guarantees before any learned policy runs on hardware;
   keep VLMs to diagnosis only.

## Additional references

- cuRobo v1: [arXiv:2310.17274](https://arxiv.org/abs/2310.17274) [V]; cuRoboV2: [arXiv:2603.05493](https://arxiv.org/abs/2603.05493) [V]
- VAMP: [arXiv:2309.14545](https://arxiv.org/abs/2309.14545) [M]
- MoveIt 2 Hybrid Planning: <https://docs.ros.org/en/iron/p/moveit_hybrid_planning> [V]
- Nav2 recovery BT: <https://docs.nav2.org/behavior_trees/trees/nav_to_pose_recovery.html> [V]
- ABB path recovery: <https://tech-community.robotics.abb.com/t/path-recovery-after-e-stop-was-triggered/11097> [V]
