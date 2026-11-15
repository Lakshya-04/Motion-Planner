"""rigid_pairs_srdf.py on a small URDF: platform (chassis, column, carriage, arm base) rigid because the
lift is fixed, then a serial arm. Run with `python3 -m pytest docker/tools/test_rigid_pairs_srdf.py`."""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__))
from rigid_pairs_srdf import pairs  # noqa: E402

URDF = """<robot name="r">
  <link name="chassis"/><link name="column"/><link name="carriage"/><link name="arm_base"/>
  <link name="shoulder"/><link name="upper_arm"/><link name="forearm"/><link name="wrist"/>
  <joint name="j1" type="fixed"><parent link="chassis"/><child link="column"/></joint>
  <joint name="j2" type="fixed"><parent link="column"/><child link="carriage"/></joint>
  <joint name="j3" type="fixed"><parent link="carriage"/><child link="arm_base"/></joint>
  <joint name="j4" type="revolute"><parent link="arm_base"/><child link="shoulder"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="j5" type="revolute"><parent link="shoulder"/><child link="upper_arm"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="j6" type="revolute"><parent link="upper_arm"/><child link="forearm"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="j7" type="revolute"><parent link="forearm"/><child link="wrist"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
</robot>"""


def _pairs(hops):
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(URDF)
    try:
        return {frozenset(p) for p in pairs(f.name, hops=hops)}
    finally:
        os.unlink(f.name)


def test_hops_two_does_not_hide_the_upper_arm_against_the_rigid_platform():
    allowed = _pairs(2)
    assert frozenset(('chassis', 'upper_arm')) not in allowed       # far along the chain, a real check
    assert frozenset(('column', 'upper_arm')) not in allowed
    assert frozenset(('carriage', 'upper_arm')) not in allowed


def test_hops_two_still_allows_what_is_truly_close_or_rigid():
    allowed = _pairs(2)
    assert frozenset(('chassis', 'column')) in allowed              # rigid group
    assert frozenset(('chassis', 'carriage')) in allowed
    assert frozenset(('arm_base', 'upper_arm')) in allowed          # two joints apart
    assert frozenset(('shoulder', 'forearm')) in allowed
    assert frozenset(('upper_arm', 'wrist')) in allowed
    assert frozenset(('chassis', 'wrist')) not in allowed
