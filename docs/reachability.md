# Reachability analysis module (RM4D)

Answers, for a given robot arm: **can the tool reach this pose** (forward query) and **where
should the base be so that it can** (inverse query), from one compact precomputed map.
Planned module; design below. Method: **RM4D**, Rudorfer, *"RM4D: A Combined Reachability and
Inverse Reachability Map for Common 6-/7-Axis Robot Arms by Dimensionality Reduction to 4D"*,
ICRA 2025, [arXiv:2410.06968](https://arxiv.org/abs/2410.06968) (project page
mrudorfer.github.io/rm4d). Per the paper's abstract, the map is far more compact and builds
about an order of magnitude faster than 6D maps with no loss of accuracy.

## Idea (from the paper)

A classic 6D reachability map discretizes position and orientation. For common 6- and 7-axis
arms the last wrist joint rotates fully and the base can rotate 360 degrees about its vertical
axis, which removes two dimensions: the map is indexed by four values (pz, theta, x*, y*)
around a canonical base position. One structure then serves both the forward query and the
inverse query, with no inversion cost. The paper reports accuracy holds even when those
assumptions are only partly satisfied.

## Why we need it

| Use | How |
|---|---|
| **Base stationing** | inverse query for the coverage poses of a wall region gives candidate base poses; pick stations that cover the most poses ([cleaning / stationing work in the roadmap]) |
| **Lift band design** | build the map for the **6-DOF arm only**; the lift shifts pz, so the map tells which wall rows each lift height covers. Choose the fewest bands, then index the lift between bands with it locked (see [design_principles.md](design_principles.md) section 4) |
| **Feasibility pre-filter** | reject unreachable coverage waypoints in microseconds before IK / planning (the manager's coverage feasibility analyzer) |
| **Recovery targets** | prefer known-state targets that are reachable from the current station ([learned_recovery.md](learned_recovery.md)) |
| **Arm / lift selection** | compare candidate arms and lift strokes on the same wall layout before buying hardware |

Assumptions to check for our robot: last wrist joint full rotation (UR family: yes), vertical
base axis (arm on a vertical lift: yes), base free to rotate (mobile base: yes). Contact work
adds an orientation constraint (tool normal to the wall); query with the constrained pose set.

## Port (ROS-free core)

```cpp
struct MapParams { /* workspace envelope, resolution, polar-angle bin size */ };
struct BaseCandidate { double x, y, theta; int score; };

class ReachabilityMap {            // core/, no ROS
 public:
  static ReachabilityMap build(const RobotModel&, const MapParams&, const Backend&);
  void save(const std::string&) const;  static ReachabilityMap load(const std::string&);
  bool  isReachable(const Pose& tool_in_arm_base) const;
  int   cellCount(const Pose& tool_in_arm_base) const;            // how many wrist configs
  std::vector<BaseCandidate> inverseQuery(const Pose& tool_in_world) const;
  void  mergeFrom(const std::vector<ReachabilityMap>& shards);    // parallel build
};
```

A **Backend** supplies batched FK, joint limits, self-collision and optional IK, so the build
reuses the planning stack's models instead of a separate one: Pinocchio (CPU, reference),
VAMP (SIMD CPU, [vamp_notes.md](vamp_notes.md)), cuRobo (GPU, [curobo_v2_notes.md](curobo_v2_notes.md)).
Input robot: the [description package](robot_description.md) profile (arm joints, limits, frames),
so one robot definition drives planning, reachability and filtering.

ROS shell: a node / CLI that builds and saves maps, and a service for forward / inverse queries
(`GetBaseCandidates`, `IsReachable`), plus RViz slice markers.

## File format

Own versioned format: header (format version, robot profile hash, `MapParams`, joint-limit
snapshot) + the 4D cell array. A map is invalid for a different robot or limits (hash check).
Maps are build artifacts: stored outside git, reproducible from the profile.

## Evaluation

- **Accuracy** against brute-force sampling of the 6D reachable set: precision / recall per
  workspace region and per resolution.
- **Build time** and memory versus resolution (parallel shards, OpenMP / GPU).
- **Query latency** for forward and inverse queries.
- **Assumption stress:** limited wrist range, restricted base rotation (e.g. docking), tool
  orientation constraints.
- **End-to-end:** share of coverage waypoints on a real wall that the map says are reachable
  versus what the planner actually reaches.

## Provenance

The method and its public code come from the paper above. An earlier production stack had its
own C++ implementation (map, builder with Pinocchio and VAMP backends, numerical IK, file
format, apps for building and querying, paper-accuracy tests); it informed the module list and
test ideas here, but its code is not used. Our implementation is written from the paper;
check the license of any public reference code before reusing it.
