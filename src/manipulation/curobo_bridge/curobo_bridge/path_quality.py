"""Path-quality scoring for free-space plans. ROS-free and torch/cuRobo-free (numpy only), so it runs on the host.

A free-space plan that is longer than it has to be executes just as slowly, so the optimal mode of
CuroboPlanner plans several candidates, adds the straight joint-space line as one more candidate, scores them
here and returns the best.

Weighted path length: the sum over the waypoints of the weighted Euclidean norm of the joint step. The default
weight of a joint is 1 / (its range), so a metre of lift (a prismatic joint with a range of about a metre) and a
radian of an arm joint (a range of about 2 pi) become comparable fractions of their travel; explicit weights
(one per joint) override that. Duration: dt * (waypoints - 1) of the time-parameterised result.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

OBJECTIVES = ('length', 'duration')


def joint_weights(lower: Sequence[float], upper: Sequence[float], floor: float = 1e-6) -> List[float]:
    """1 / range per joint (a locked joint with no range gets weight 1 / floor-free fallback of 1)."""
    out = []
    for lo, hi in zip(lower, upper):
        span = float(hi) - float(lo)
        out.append(1.0 / span if span > floor else 1.0)
    return out


DEFAULT_LIFT_COST_WEIGHT = 2.0


def apply_lift_cost_weight(weights: Sequence[float], joint_names: Sequence[str],
                           lift_joints: Sequence[str], lift_cost_weight: float = DEFAULT_LIFT_COST_WEIGHT) -> List[float]:
    """`weights` with the entries of `lift_joints` multiplied by `lift_cost_weight` (lift motion that many times as
    costly as arm motion). 1.0 (or a non-positive / non-finite value) returns the weights unchanged."""
    out = [float(w) for w in weights]
    if not (np.isfinite(lift_cost_weight) and lift_cost_weight > 0.0) or lift_cost_weight == 1.0:
        return out
    lift = set(lift_joints)
    return [w * float(lift_cost_weight) if name in lift else w for w, name in zip(out, joint_names)]


def resolve_joint_costs(joint_names: Sequence[str], joint_costs: Optional[Sequence[float]]) -> Optional[List[float]]:
    """`joint_costs` (one per joint, lift first: the profile's `joint_costs`) checked against the robot's joints;
    None or empty means none given. Raises ValueError on a wrong size or an entry that is not finite and above 0."""
    if joint_costs is None or len(joint_costs) == 0:
        return None
    if len(joint_costs) != len(joint_names):
        raise ValueError(f'joint_costs has {len(joint_costs)} entries, the robot has {len(joint_names)} joints '
                         f'{list(joint_names)}')
    out = [float(c) for c in joint_costs]
    if not all(np.isfinite(c) and c > 0.0 for c in out):
        raise ValueError(f'joint_costs entries must be finite and above 0, got {out}')
    return out


def apply_joint_costs(weights: Sequence[float], joint_costs: Sequence[float]) -> List[float]:
    """`weights` with joint j multiplied by `joint_costs[j]` (the per-joint movement cost). Same rule as the lift
    weight with the lift costing 2: apply_joint_costs(w, [2, 1, 1, ...])."""
    if len(weights) != len(joint_costs):
        raise ValueError(f'{len(joint_costs)} joint costs for {len(weights)} weights')
    return [float(w) * float(c) for w, c in zip(weights, joint_costs)]


def path_length(positions: Sequence[Sequence[float]], weights: Optional[Sequence[float]] = None) -> float:
    """Weighted joint-space path length; 0 for fewer than two waypoints."""
    q = np.asarray(positions, dtype=float)
    if q.ndim != 2 or len(q) < 2:
        return 0.0
    w = np.ones(q.shape[1]) if weights is None else np.asarray(weights, dtype=float)
    if w.shape != (q.shape[1],):
        raise ValueError(f'{w.shape[0]} weights for {q.shape[1]} joints')
    step = np.diff(q, axis=0) * w
    return float(np.sum(np.sqrt(np.sum(step * step, axis=1))))


def duration(positions: Sequence[Sequence[float]], dt: float) -> float:
    return float(dt) * max(0, len(positions) - 1)


def straight_duration(start: Sequence[float], goal: Sequence[float], velocity_limits: Sequence[float],
                      acceleration_limits: Sequence[float]) -> float:
    """Time of the fastest synchronised straight joint-space move under per-joint velocity and acceleration
    limits (trapezoidal, or triangular when the distance is too short to reach cruise speed)."""
    s_speed, s_acc = _path_rates(start, goal, velocity_limits, acceleration_limits)
    if s_speed is None:
        return 0.0
    return _trapezoid_time(s_speed, s_acc)


def _path_rates(start, goal, vlim, alim) -> Tuple[Optional[float], Optional[float]]:
    d = np.abs(np.asarray(goal, dtype=float) - np.asarray(start, dtype=float))
    moving = d > 1e-12
    if not moving.any():
        return None, None
    v = np.asarray(vlim, dtype=float)[moving]
    a = np.asarray(alim, dtype=float)[moving]
    if np.any(v <= 0) or np.any(a <= 0):
        raise ValueError('velocity and acceleration limits must be positive')
    # the path parameter s in [0, 1] moves at most vmax_s per second and accelerates at most amax_s
    return float(np.min(v / d[moving])), float(np.min(a / d[moving]))


def _trapezoid_time(v: float, a: float) -> float:
    if v * v / a >= 1.0:                      # triangular profile, never reaches cruise speed
        return 2.0 * np.sqrt(1.0 / a)
    return 1.0 / v + v / a


def straight_line(start: Sequence[float], goal: Sequence[float], dt: float,
                  velocity_limits: Optional[Sequence[float]] = None,
                  acceleration_limits: Optional[Sequence[float]] = None) -> List[List[float]]:
    """The straight joint-space interpolation start -> goal as waypoints `dt` apart (both ends included).
    With limits the path parameter follows a trapezoidal (time-optimal) profile, so the duration is the
    fastest the limits allow; without them it is uniform at a nominal 1 s. Same start and goal: two rows."""
    q0, q1 = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if dt <= 0:
        raise ValueError('dt must be positive')
    if velocity_limits is not None and acceleration_limits is not None:
        s_speed, s_acc = _path_rates(q0, q1, velocity_limits, acceleration_limits)
    else:
        s_speed = s_acc = None
    if s_speed is None and velocity_limits is not None:
        total = dt
        profile = lambda t: 1.0   # noqa: E731 - same start and goal
    elif s_speed is None:
        total = 1.0
        profile = lambda t: t / total   # noqa: E731
    else:
        total = _trapezoid_time(s_speed, s_acc)
        profile = lambda t: _trapezoid_s(t, s_speed, s_acc, total)   # noqa: E731
    n = max(1, int(np.ceil(total / dt - 1e-9)))
    scale = total / (n * dt)                  # stretch slightly so the last row lands on the goal exactly
    rows = []
    for k in range(n + 1):
        s = min(1.0, max(0.0, profile(k * dt * scale)))
        rows.append((q0 + (q1 - q0) * s).tolist())
    rows[-1] = q1.tolist()
    return rows


def _trapezoid_s(t: float, v: float, a: float, total: float) -> float:
    if v * v / a >= 1.0:
        half = total / 2.0
        return 0.5 * a * t * t if t <= half else 1.0 - 0.5 * a * (total - t) ** 2
    ta = v / a
    if t <= ta:
        return 0.5 * a * t * t
    if t >= total - ta:
        return 1.0 - 0.5 * a * (total - t) ** 2
    return 0.5 * v * ta + v * (t - ta)


def efficiency(length: float, baseline: float) -> float:
    """baseline / length: 1.0 is as short as the straight line, smaller is longer. A zero-length baseline
    (start == goal) gives 1.0 for a zero-length path and 0.0 otherwise; the value never exceeds 1."""
    if length <= 1e-12:
        return 1.0
    return float(min(1.0, baseline / length))


@dataclass
class Candidate:
    name: str                                  # e.g. 'straight', 'trajopt-0'
    positions: List[List[float]]
    dt: float
    valid: bool = True
    length: float = 0.0
    duration_s: float = 0.0
    info: dict = field(default_factory=dict)


def score(candidate: Candidate, weights: Optional[Sequence[float]] = None) -> Candidate:
    candidate.length = path_length(candidate.positions, weights)
    candidate.duration_s = duration(candidate.positions, candidate.dt)
    return candidate


def objective_value(candidate: Candidate, objective: str) -> float:
    if objective == 'length':
        return candidate.length
    if objective == 'duration':
        return candidate.duration_s
    raise ValueError(f'objective must be one of {OBJECTIVES}, got {objective!r}')


def select_best(candidates: Sequence[Candidate], objective: str = 'length',
                weights: Optional[Sequence[float]] = None, tol: float = 1e-9) -> Optional[Candidate]:
    """The valid candidate with the lowest objective (scored here); ties (within `tol`) go to the lower
    secondary objective (duration for 'length', length for 'duration'), then to the earlier candidate, so a
    straight line listed first wins every tie. None when no candidate is valid."""
    if objective not in OBJECTIVES:
        raise ValueError(f'objective must be one of {OBJECTIVES}, got {objective!r}')
    other = 'duration' if objective == 'length' else 'length'
    best, best_key = None, None
    for c in candidates:
        if not c.valid or len(c.positions) < 1:
            continue
        score(c, weights)
        key = (objective_value(c, objective), objective_value(c, other))
        if best is None or key[0] < best_key[0] - tol or (abs(key[0] - best_key[0]) <= tol and key[1] < best_key[1] - tol):
            best, best_key = c, key
    return best


def summary_line(n_candidates: int, best: Candidate, objective: str, baseline: Candidate) -> str:
    """'free-space plan: candidates N, best <objective>=X (baseline straight Y, efficiency Z)'."""
    x, y = objective_value(best, objective), objective_value(baseline, objective)
    z = efficiency(x, y)
    return (f'free-space plan: candidates {n_candidates}, best {objective}={x:.4f} '
            f'(baseline straight {y:.4f}, efficiency {z:.3f}), picked {best.name}')
