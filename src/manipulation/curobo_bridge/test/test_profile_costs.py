import pytest

from curobo_bridge import profile_costs as pc

PROFILE = """
robot:
  xacro_args: {arm: generic_6dof}
joints:
  lift: lift_joint
  arm: [j1, j2]
home: {lift_joint: 0.0, j1: 0.0, j2: 0.0}
"""


def write(tmp_path, extra=''):
    path = tmp_path / 'p.yaml'
    path.write_text(PROFILE + extra)
    return str(path)


def test_no_joint_costs_is_none(tmp_path):
    assert pc.load_joint_costs(write(tmp_path)) is None


def test_reads_the_array_in_joints_order(tmp_path):
    assert pc.load_joint_costs(write(tmp_path, 'joint_costs: [3.0, 1, 0.5]\n')) == [3.0, 1.0, 0.5]


def test_name_resolves_through_the_profile_dir(tmp_path, monkeypatch):
    write(tmp_path, 'joint_costs: [2, 1, 1]\n')
    monkeypatch.setenv('MOBILE_MANIPULATOR_PROFILE_DIR', str(tmp_path))
    assert pc.load_joint_costs('p') == [2.0, 1.0, 1.0]


def test_wrong_size_and_bad_values_raise(tmp_path):
    with pytest.raises(ValueError):
        pc.load_joint_costs(write(tmp_path, 'joint_costs: [2, 1]\n'))
    with pytest.raises(ValueError):
        pc.load_joint_costs(write(tmp_path, 'joint_costs: [2, 0, 1]\n'))
