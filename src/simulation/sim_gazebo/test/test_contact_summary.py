"""contact_summary: the tool wrench sum and its statistics (no ROS, no Gazebo)."""
import os
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from sim_gazebo.contact_summary import tool_wrench_world  # noqa: E402

TOOL = 'm::tool_tip_link::tool_tip_link_collision'


def _v(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def _contact(c1, c2, points):
    """points: [(position xyz, force on body 1 xyz)]; body 2 gets the opposite force."""
    return NS(collision1=NS(name=c1), collision2=NS(name=c2),
              positions=[_v(*p) for p, _ in points],
              wrenches=[NS(body_1_wrench=NS(force=_v(*f), torque=_v()),
                           body_2_wrench=NS(force=_v(*(-np.array(f))), torque=_v())) for _, f in points])


def test_sum_picks_the_tool_side_and_counts_points():
    msg = NS(contacts=[_contact('w::wall::collision', TOOL, [((1, 0, 0), (0, 0, 30)), ((1, 0.01, 0), (0, 0, 10))])])
    f, tau, st = tool_wrench_world(msg, np.zeros(3))
    assert f == pytest.approx([0, 0, -40])     # tool is body 2: the wall pushes it with -f of the wall's body 1
    assert st['contacts'] == 1 and st['points'] == 2 and st['dup_points'] == 0
    assert st['sum_point_norm'] == pytest.approx(40) and st['max_point_norm'] == pytest.approx(30)
    assert st['others'] == ['w::wall::collision']


def test_duplicate_points_are_counted_not_removed():
    p = ((1, 0, 0), (0, 0, 5))
    msg = NS(contacts=[_contact(TOOL, 'w::wall::collision', [p, p])])
    f, _, st = tool_wrench_world(msg, np.zeros(3))
    assert f == pytest.approx([0, 0, 10])      # unchanged behaviour: summed as reported
    assert st['dup_points'] == 1


def test_name_filters_select_pairs():
    msg = NS(contacts=[_contact(TOOL, 'w::wall_0::collision', [((1, 0, 0), (0, 0, 7))]),
                       _contact(TOOL, 'g::ground::collision', [((1, 0, 0.1), (0, 0, 900))]),
                       _contact('a::arm::c', 'b::other::c', [((0, 0, 0), (0, 0, 1e6))])])
    f_all, _, st_all = tool_wrench_world(msg, np.zeros(3))
    assert f_all[2] == pytest.approx(907) and st_all['contacts'] == 2
    f, _, st = tool_wrench_world(msg, np.zeros(3), include_other=('wall',))
    assert f[2] == pytest.approx(7) and st['filtered'] == 1
    f, _, st = tool_wrench_world(msg, np.zeros(3), exclude_other=('ground',))
    assert f[2] == pytest.approx(7) and st['others'] == ['w::wall_0::collision']


def test_torque_has_the_lever_arm_about_tcp():
    msg = NS(contacts=[_contact(TOOL, 'w::wall::c', [((0.1, 0, 0), (0, 0, 10))])])
    _, tau, _ = tool_wrench_world(msg, np.zeros(3))
    assert tau == pytest.approx(np.cross([0.1, 0, 0], [0, 0, 10]))
