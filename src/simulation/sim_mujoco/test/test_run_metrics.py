"""run_metrics: lift range, chassis-to-wall clearance and tool reach fraction from manager.log lines (no ROS)."""
import importlib.util
import json
import math
import pathlib

import pytest

from sim_mujoco import run_metrics as rm

HERE = pathlib.Path(__file__).parent
PLANE = [-1.0, 0.0, 0.0, 0.98]   # the stock wall x = 0.98, normal toward the robot

LOG = """\
[manipulation_manager]: stand 0: arm base (0.180, 0.010) yaw 1.571 reaches 36 pose(s)
[manipulation_manager]: stand 1: arm base (0.180, 0.510) yaw 1.571 reaches 18 pose(s)
[manipulation_manager]: ARMSTATE stowed stand 0: q=[0.0000, 0.0, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.100, 0.010, 1.300)
[manipulation_manager]: ARMSTATE docked_wall_column_4 stand 0: q=[0.0455, 2.7983, -2.8, -1.3, -2.2, -0.36, -1.5708] tcp_in_odom=(0.940, 0.810, 1.010)
[manipulation_manager]: ARMSTATE docked_wall_column_5 stand 0: q=[0.1700, 2.7, -2.8, -1.3, -2.2, -0.36, -1.5708] tcp_in_odom=(0.940, 0.660, 1.100)
[manipulation_manager]: ARMSTATE docked_wall_column_0 stand 1: q=[0.0900, 2.7, -2.8, -1.3, -2.2, -0.36, -1.5708] tcp_in_odom=(0.940, 0.510, 1.050)
unrelated line with stand 3 and q=[1, 2]
"""


def test_parse_log():
    arm, stands = rm.parse_log(LOG.splitlines())
    assert len(arm) == 4 and arm[1]['tag'] == 'docked_wall_column_4' and arm[1]['stand'] == 0
    assert arm[1]['q'][:2] == [0.0455, 2.7983] and arm[1]['tcp'] == (0.94, 0.81, 1.01)
    assert stands[0] == dict(x=0.18, y=0.01, yaw=1.571, poses=36) and stands[1]['y'] == 0.51


def test_parse_log_ignores_malformed_and_keeps_last_stand_line():
    arm, stands = rm.parse_log(['ARMSTATE t stand 0: q=[] tcp_in_odom=(1, 2, 3)',
                                'ARMSTATE t stand 0: q=[a, b] tcp_in_odom=(1, 2, 3)',
                                'stand 0: arm base (0.1, 0.2) yaw 0.0 reaches 3 pose(s)',
                                'stand 0: arm base (0.3, 0.4) yaw 0.0 reaches 5 pose(s)'])
    assert arm == [] and stands[0]['x'] == 0.3 and stands[0]['poses'] == 5


def test_lift_stats_shows_a_lift_that_is_barely_used():
    arm, _ = rm.parse_log(LOG.splitlines())
    L = rm.lift_stats(arm, 1.5)
    assert (L['min'], L['max']) == (0.0, 0.17) and L['span'] == pytest.approx(0.17)
    assert L['span_fraction'] == pytest.approx(0.17 / 1.5)
    assert L['travel'] == pytest.approx(0.0455 + 0.1245 + 0.08)
    assert (L['docked_min'], L['docked_max']) == (0.0455, 0.17)
    assert rm.lift_stats([]) is None


def test_chassis_clearance_uses_the_rotated_footprint():
    _, stands = rm.parse_log(LOG.splitlines())
    c = rm.chassis_clearance(stands, PLANE)
    # yaw 1.571: the 0.60 m side faces the wall, so the chassis edge is 0.30 m from the arm base's x
    assert c[0]['arm_base'] == pytest.approx(0.80)
    assert c[0]['chassis'] == pytest.approx(0.80 - 0.30 * math.cos(1.571 - math.pi / 2) - 0.45 * abs(math.sin(1.571 - math.pi / 2)),
                                           abs=2e-3)
    # yaw 0: the 0.45 m end faces the wall
    c0 = rm.chassis_clearance({0: dict(x=0.18, y=0.0, yaw=0.0)}, PLANE)
    assert c0[0]['chassis'] == pytest.approx(0.80 - 0.45)
    # an arm mount offset of 0.2 m ahead moves the base back by 0.2
    c1 = rm.chassis_clearance({0: dict(x=0.18, y=0.0, yaw=0.0)}, PLANE, arm_mount_xy=(0.2, 0.0))
    assert c1[0]['chassis'] == pytest.approx(0.98 - (0.18 - 0.2 + 0.45))


def test_reach_fraction_is_distance_from_the_arm_base_over_the_reach():
    arm, stands = rm.parse_log(LOG.splitlines())
    r = rm.reach_fractions(arm, stands, arm_mount_z_m=1.17, reach_m=1.3)
    d = math.dist((0.18, 0.01, 1.17 + 0.0455), (0.94, 0.81, 1.01))
    assert d == pytest.approx(1.12, abs=0.01)
    assert r[0]['n'] == 2 and r[0]['max'] == pytest.approx(max(
        d, math.dist((0.18, 0.01, 1.34), (0.94, 0.66, 1.10))) / 1.3)
    assert r[1]['n'] == 1
    assert rm.reach_fractions(arm, stands, docked_only=False)[0]['n'] == 3   # the stowed sample too


def test_evaluate_flags_the_user_complaints():
    res = rm.evaluate(LOG.splitlines(), PLANE)
    text = ' | '.join(res['flags'])
    assert 'lift not used' in text and '11%' in text
    assert 'tool at' in text                              # 1.12 / 1.3 = 86 %
    out = '\n'.join(rm.format_table(res))
    assert 'span 0.170 m' in out and 'FLAG:' in out and out.count('\n') >= 4


def test_evaluate_a_good_run_has_no_flags_and_missing_data_is_said():
    lines = ['stand 0: arm base (0.0, 0.0) yaw 0.0 reaches 5 pose(s)',
             'ARMSTATE docked_a stand 0: q=[0.2, 0] tcp_in_odom=(0.9, 0.0, 1.5)',
             'ARMSTATE docked_b stand 0: q=[1.2, 0] tcp_in_odom=(0.9, 0.0, 2.3)']
    res = rm.evaluate(lines, [-1.0, 0.0, 0.0, 1.6], reach_m=1.3)
    assert res['flags'] == [] and res['lift']['span_fraction'] == pytest.approx(1.0 / 1.5)
    assert 'no ARMSTATE' in rm.evaluate(['nothing here'])['flags'][0]


def test_cli_prints_a_table_for_a_run_directory(tmp_path, capsys):
    spec = importlib.util.spec_from_file_location('run_metrics_cli', HERE.parent / 'scripts' / 'run_metrics.py')
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    (tmp_path / 'manager.log').write_text(LOG)
    (tmp_path / 'data').mkdir()
    (tmp_path / 'data' / 'wall_pass_coverage_scene.json').write_text(json.dumps({'walls': [{'plane_abcd': PLANE}]}))
    assert cli.main([str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert 'lift:' in out and '0.500 m' in out and 'FLAG: lift not used' in out
    assert cli.main([str(tmp_path / 'missing')]) == 2
