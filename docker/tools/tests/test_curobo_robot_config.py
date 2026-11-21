"""Tests for the ignore-table helpers of generate_curobo_robot.py and the shipped generator scripts (no GPU,
no cuRobo: the builder import is lazy).

    python -m pytest docker/tools/tests/test_curobo_robot_config.py
"""
import pathlib
import re
import sys

import pytest

TOOLS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import generate_curobo_robot as g  # noqa: E402


def test_module_imports_without_curobo():
    assert callable(g.merge_ignore)


def test_merge_ignore_is_symmetric_sorted_and_keeps_the_base():
    base = {'a': ['b'], 'b': ['a']}
    out = g.merge_ignore(base, pairs=['lift_stage_2:arm_base'], groups=['x,y,z'])
    assert out['a'] == ['b'] and out['b'] == ['a']
    assert out['lift_stage_2'] == ['arm_base'] and out['arm_base'] == ['lift_stage_2']
    assert out['x'] == ['y', 'z'] and out['y'] == ['x', 'z'] and out['z'] == ['x', 'y']
    assert list(out) == sorted(out)
    assert base == {'a': ['b'], 'b': ['a']}      # the input is not modified


def test_merge_ignore_without_a_base_and_without_self_pairs():
    assert g.merge_ignore(None, pairs=['a:a']) == {}
    assert g.merge_ignore(None, groups=['a, b']) == {'a': ['b'], 'b': ['a']}


@pytest.mark.parametrize('pair', ['a', 'a:', ':b', 'a:b:c'])
def test_merge_ignore_rejects_a_malformed_pair(pair):
    with pytest.raises(ValueError, match='LINK_A:LINK_B'):
        g.merge_ignore(None, pairs=[pair])


def test_merge_ignore_rejects_a_group_of_one():
    with pytest.raises(ValueError, match='at least two'):
        g.merge_ignore(None, groups=['only'])


def test_unknown_links_reports_typos():
    table = g.merge_ignore(None, pairs=['lift_carriage_link:arm_base_link', 'lift_carrage_link:arm_base_link'])
    assert g.unknown_links(table, ['lift_carriage_link', 'arm_base_link']) == ['lift_carrage_link']


def test_the_7dof_script_defaults_to_the_hop_ignore_rule():
    """Without --ignore-hops the sampled matrix alone failed 52 of 158 dock-pose problems (docs/curobo_v2_notes.md);
    the 6-joint robot_assets.py stage already passes it, the 7-joint script must too."""
    text = (TOOLS / 'generate_curobo_7dof.sh').read_text()
    assert re.search(r'\$\{CUROBO_SPHERE_ARGS-+ignore-hops 2\}', text)
