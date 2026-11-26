"""Per-joint movement costs from a robot profile: the same `joint_costs` key the C++ manager reads."""
import math
import os
import pathlib
from typing import List, Optional

import yaml


def profile_path(profile: str) -> pathlib.Path:
    """A profile name ("ur10e_telescopic") or a path to a .yaml file, like mobile_manipulator_profile::loadProfile."""
    p = pathlib.Path(profile)
    if p.suffix in ('.yaml', '.yml') or p.is_file():
        return p
    env = os.environ.get('MOBILE_MANIPULATOR_PROFILE_DIR')
    if env:
        return pathlib.Path(env) / f'{profile}.yaml'
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory('mobile_manipulator_description')) / 'config' / 'profiles' / f'{profile}.yaml'


def load_joint_costs(profile: str) -> Optional[List[float]]:
    """The profile's `joint_costs` (lift first, then the arm), or None when it has none.
    Raises ValueError on a wrong size or an entry that is not finite and above 0."""
    path = profile_path(profile)
    doc = yaml.safe_load(open(path))
    costs = doc.get('joint_costs')
    if costs is None:
        return None
    joints = doc.get('joints', {})
    expected = (1 if joints.get('lift') else 0) + len(joints.get('arm', []))
    if len(costs) != expected:
        raise ValueError(f'{path}: joint_costs has {len(costs)} entries, the profile has {expected} joints '
                         '(lift first, then the arm)')
    out = [float(c) for c in costs]
    if not all(math.isfinite(c) and c > 0.0 for c in out):
        raise ValueError(f'{path}: joint_costs entries must be finite and above 0, got {out}')
    return out
