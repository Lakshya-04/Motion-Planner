"""Base drive modes: what motion a mode allows, read from the `base.modes` list of a robot profile.

ROS-free. A profile without `modes` gets one mode derived from `drive_type` the way the planner already
treats it (omni and 4ws translate freely, diff and skid turn in place and never go sideways).
"""
import dataclasses
from dataclasses import dataclass

MODE_NAMES = ('omni', 'diff_turn_in_place', 'skid', 'ackermann', 'crab', 'spin')

# can_strafe, can_spin_in_place per mode name; a profile may only restate these, not contradict them
_CAPS = {
    'omni': (True, True),
    'diff_turn_in_place': (False, True),
    'skid': (False, True),
    'ackermann': (False, False),
    'crab': (True, False),
    'spin': (False, True),
}
_KEYS = {'name', 'can_strafe', 'can_spin_in_place', 'min_turn_radius_m', 'max_linear_mps', 'max_angular_radps',
         'max_linear_acceleration_mps2', 'max_angular_acceleration_radps2', 'default',
         'max_linear_turning_mps'}


class ModeError(ValueError):
    pass


@dataclass(frozen=True)
class DriveMode:
    name: str
    can_strafe: bool
    can_spin_in_place: bool
    min_turn_radius: object            # metres at base_footprint, or None
    max_lin: float
    max_ang: float
    max_lin_acc: float
    max_ang_acc: float
    default: bool = False
    max_lin_turning: object = None     # speed cap on curved path pieces (Ackermann), None = max_lin

    @property
    def turning_speed(self):
        return self.max_lin_turning or self.max_lin

    @property
    def max_curvature(self):
        return 0.0 if not self.min_turn_radius else 1.0 / self.min_turn_radius


def derive_mode(drive, max_lin, max_ang, max_lin_acc, max_ang_acc):
    """The single mode a profile without `modes` stands for (same split as BaseProfile.holonomic)."""
    name = {'omni': 'omni', '4ws': 'omni', 'skid': 'skid'}.get(drive, 'diff_turn_in_place')
    strafe, spin = _CAPS[name]
    return DriveMode(name, strafe, spin, None, max_lin, max_ang, max_lin_acc, max_ang_acc, True)


def _positive(v, profile, mode, key):
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ModeError(f'profile {profile}: mode {mode}: {key} must be a number, got {v!r}') from None
    if not f > 0.0 or f != f or f == float('inf'):
        raise ModeError(f'profile {profile}: mode {mode}: {key} must be positive, got {v!r}')
    return f


def parse_modes(base, profile_name, limits):
    """List of `DriveMode` from a `base:` dict, () when it has no `modes`. `limits` = (max_lin, max_ang,
    max_lin_acc, max_ang_acc), used where a mode leaves a limit out."""
    raw = base.get('modes')
    if raw is None:
        return ()
    if not isinstance(raw, list) or not raw:
        raise ModeError(f'profile {profile_name}: base.modes must be a non-empty list')
    out, seen = [], set()
    for i, m in enumerate(raw):
        if not isinstance(m, dict):
            raise ModeError(f'profile {profile_name}: modes[{i}] must be a mapping')
        name = m.get('name')
        if name not in MODE_NAMES:
            raise ModeError(f'profile {profile_name}: mode {name!r}: unknown name, expected one of {MODE_NAMES}')
        if name in seen:
            raise ModeError(f'profile {profile_name}: mode {name}: listed twice')
        seen.add(name)
        extra = set(m) - _KEYS
        if extra:
            raise ModeError(f'profile {profile_name}: mode {name}: unknown keys {sorted(extra)}')
        strafe, spin = _CAPS[name]
        for key, want in (('can_strafe', strafe), ('can_spin_in_place', spin)):
            if key in m and bool(m[key]) != want:
                raise ModeError(f'profile {profile_name}: mode {name}: {key} must be {str(want).lower()}')
        radius = m.get('min_turn_radius_m')
        if name == 'ackermann':
            if radius is None:
                raise ModeError(f'profile {profile_name}: mode ackermann: min_turn_radius_m is required')
            radius = _positive(radius, profile_name, name, 'min_turn_radius_m')
        elif radius is not None:
            raise ModeError(f'profile {profile_name}: mode {name}: min_turn_radius_m must be null')
        vals = []
        for key, fallback in zip(('max_linear_mps', 'max_angular_radps', 'max_linear_acceleration_mps2',
                                  'max_angular_acceleration_radps2'), limits):
            vals.append(_positive(m[key], profile_name, name, key) if m.get(key) is not None else fallback)
        turning = m.get('max_linear_turning_mps')
        if turning is not None:
            if name != 'ackermann':
                raise ModeError(f'profile {profile_name}: mode {name}: max_linear_turning_mps only applies to ackermann')
            turning = _positive(turning, profile_name, name, 'max_linear_turning_mps')
        out.append(DriveMode(name, strafe, spin, radius, *vals, default=bool(m.get('default', False)),
                             max_lin_turning=turning))
    if sum(md.default for md in out) > 1:
        raise ModeError(f'profile {profile_name}: more than one mode has default: true')
    if not any(md.default for md in out):
        out[0] = dataclasses.replace(out[0], default=True)
    return tuple(out)


def default_mode(modes):
    return next((m for m in modes if m.default), modes[0])
