"""Tests of scenario_sync: schema, generator, checker (no ROS, no simulator)."""
import copy
import json
import math
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from scenario_sync import check_sync, scenario_gen, sources  # noqa: E402
from scenario_sync.schema import ScenarioError, load_scenario, validate  # noqa: E402

EX = os.path.join(ROOT, 'examples')
REPO = scenario_gen.find_repo()
needs_repo = pytest.mark.skipif(REPO is None, reason='no source tree with the simulator generators')

BASE_DOC = {
    'name': 't', 'world_frame': 'odom',
    'wall': {'point': [0.98, 0.0, 2.0], 'normal': [-1, 0, 0], 'u_range': [-1.5, 1.5], 'v_range': [0.0, 2.0]},
    'patch': {'mode': 'crop', 'u_range': [0.3, 1.2], 'v_range': [0.7, 1.0]},
    'base': {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'standoff_m': 0.98},
    'robot_profile': 'ur10e_telescopic', 'tool_profile': 'spray_gun', 'lift_range_m': [0.0, 1.5],
}


def doc(**over):
    d = copy.deepcopy(BASE_DOC)
    for k, v in over.items():
        d[k] = v
    return d


def scn_of(**over):
    return validate(doc(**over))


def status(checks, name_part, source_part=''):
    rows = [c for c in checks if name_part in c.name and source_part in c.source]
    assert rows, f'no check {name_part!r} / {source_part!r} in {[(c.source, c.name) for c in checks]}'
    return rows[0].status


# ------------------------------------------------------------------ schema
@pytest.mark.parametrize('name', ['gazebo_smoke_full', 'gazebo_patch_crop', 'mujoco_tall_wall'])
def test_examples_validate(name):
    s = load_scenario(os.path.join(EX, name + '.yaml'))
    assert s.name == name and s.world_frame == 'odom'
    assert s.standoff_m == pytest.approx(0.98)


def test_example_numbers():
    full = load_scenario(os.path.join(EX, 'gazebo_smoke_full.yaml'))
    crop = load_scenario(os.path.join(EX, 'gazebo_patch_crop.yaml'))
    tall = load_scenario(os.path.join(EX, 'mujoco_tall_wall.yaml'))
    assert full.wall_area == pytest.approx(6.0) and full.patch_fraction == pytest.approx(1.0)
    assert crop.patch_area == pytest.approx(0.27) and crop.patch_fraction == pytest.approx(0.045)
    assert crop.patch_z == pytest.approx((1.0, 1.3))
    assert tall.wall_z == pytest.approx((0.3, 2.3)) and tall.patch_z == pytest.approx((0.6, 2.0))


def test_all_errors_listed():
    with pytest.raises(ScenarioError) as e:
        validate({'name': 'bad name', 'wall': {'point': [0, 0], 'normal': [0, 0, 1], 'u_range': [1, 0]},
                  'patch': {'mode': 'half'}, 'colour': 1})
    msg = ' '.join(e.value.errors)
    for needle in ('unknown key', 'name', 'wall.point', 'horizontal', 'u_range', 'patch.mode', 'base', 'robot_profile',
                   'lift_range_m'):
        assert needle in msg, needle


def test_patch_must_lie_in_wall():
    with pytest.raises(ScenarioError, match='not inside wall.u_range'):
        scn_of(patch={'mode': 'crop', 'u_range': [1.0, 1.8], 'v_range': [0.7, 1.0]})
    with pytest.raises(ScenarioError, match='only used with mode'):
        scn_of(patch={'mode': 'full', 'u_range': [0, 1], 'v_range': [0, 1]})


def test_standoff_must_match_base_position():
    with pytest.raises(ScenarioError, match='disagrees with the base-to-wall distance'):
        scn_of(base={'x': 0.0, 'y': 0.0, 'standoff_m': 0.9})


def test_standoff_only_places_base_and_faces_wall():
    s = scn_of(base={'standoff_m': 0.9})
    assert (s.base_x, s.base_y) == pytest.approx((0.08, 0.0))   # wall plane x 0.98, 0.9 in front of it
    assert s.base_yaw == pytest.approx(0.0)
    s = scn_of(wall={'point': [0.0, 2.0, 2.0], 'normal': [0, -1, 0], 'u_range': [-1, 1], 'v_range': [0, 2]},
               patch={'mode': 'full'}, base={'standoff_m': 1.0})
    assert (s.base_x, s.base_y) == pytest.approx((0.0, 1.0)) and s.base_yaw == pytest.approx(math.pi / 2)


def test_advisor_json_sets_standoff_and_moves_base(tmp_path):
    adv = tmp_path / 'advice.json'
    adv.write_text(json.dumps({'recommended_standoff_m': 0.85, 'other': 1}))
    s = scn_of(base={'x': 0.0, 'y': 0.2, 'yaw': 0.0, 'standoff_advisor_json': str(adv)})
    assert s.standoff_m == pytest.approx(0.85) and s.base_x == pytest.approx(0.13) and s.base_y == pytest.approx(0.2)
    assert 'advisor' in s.standoff_source
    adv.write_text(json.dumps({'nope': 1}))
    with pytest.raises(ScenarioError, match='recommended_standoff_m'):
        scn_of(base={'x': 0.0, 'y': 0.0, 'standoff_advisor_json': str(adv)})
    with pytest.raises(ScenarioError, match='cannot read'):
        scn_of(base={'x': 0.0, 'y': 0.0, 'standoff_advisor_json': str(tmp_path / 'missing.json')})


# ------------------------------------------------------------------ generator
def test_gen_plan_scene_is_crop_or_full(tmp_path):
    crop = scn_of()
    m = scenario_gen.generate(crop, str(tmp_path / 'c'), sim_generators=False)
    plan = json.load(open(m['files']['plan_scene']))
    full = json.load(open(m['files']['full_scene']))
    assert plan['frame_id'] == 'odom'
    assert plan['walls'][0]['u_range_m'] == [0.3, 1.2] and full['walls'][0]['u_range_m'] == [-1.5, 1.5]
    assert plan['walls'][0]['plane_abcd'] == [-1.0, 0.0, 0.0, 0.98]
    assert m['patch_fraction_of_wall'] == pytest.approx(0.045)
    whole = scn_of(patch={'mode': 'full'})
    m = scenario_gen.generate(whole, str(tmp_path / 'f'), sim_generators=False)
    assert json.load(open(m['files']['plan_scene']))['walls'][0]['u_range_m'] == [-1.5, 1.5]


def test_gen_rviz_and_env(tmp_path):
    s = scn_of(base={'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'standoff_m': 0.98},
               consumers={'rviz': {'fixed_frame': 'map'}})
    m = scenario_gen.generate(s, str(tmp_path), sim_generators=False)
    r = yaml.safe_load(open(m['files']['rviz_args']))
    blk = r['scenario_sync']
    assert blk['fixed_frame'] == 'map' and blk['plan_frame'] == 'odom'
    assert (blk['base_x'], blk['base_y'], blk['base_yaw']) == (0.0, 0.0, 0.0)
    assert os.path.exists(blk['wall_scene_json'])
    assert r['mp_viz_scene']['ros__parameters']['wall_normal'] == [-1.0, 0.0, 0.0]
    env = open(m['files']['env_sh']).read()
    assert 'SCENARIO_PLAN_SCENE=' in env and 'MULTI_STAND_WALL_DY=0.3' in env and 'MULTI_STAND_WALL_WIDTH=0.9' in env
    assert 'scene_offset:=0.0 0.0 0.0 0.0' in env


def test_gen_moved_base_gives_base_frame_gazebo_scene_and_mujoco_offset(tmp_path):
    s = scn_of(wall={'point': [1.98, 1.0, 2.0], 'normal': [-1, 0, 0], 'u_range': [-1.5, 1.5], 'v_range': [0, 2]},
               patch={'mode': 'full'}, base={'x': 1.0, 'y': 1.0, 'yaw': 0.0, 'standoff_m': 0.98})
    m = scenario_gen.generate(s, str(tmp_path), sim_generators=False)
    gz = json.load(open(m['files']['gazebo_scene']))['walls'][0]
    assert gz['origin_xyz'][:2] == pytest.approx([0.98, 0.0])   # the wall in the base-start frame
    assert m['mujoco_scene_offset'] == pytest.approx([-1.0, -1.0, 0.0, 0.0])
    assert any('MULTI_STAND' in w or 'origin' in w for w in m['legacy_multi_stand_mismatch'])


@needs_repo
def test_generated_files_check_clean_for_every_example(tmp_path):
    for name in ('gazebo_smoke_full', 'gazebo_patch_crop', 'mujoco_tall_wall'):
        s = load_scenario(os.path.join(EX, name + '.yaml'))
        m = scenario_gen.generate(s, str(tmp_path / name))
        assert 'gazebo_sdf' in m['files'] and 'mujoco_mjcf' in m['files'], m['notes']
        checks = check_sync.run_checks(s, **check_sync.gen_dir_files(str(tmp_path / name)))
        assert check_sync.verdict(checks) == 0, [c.as_dict() for c in checks if c.status == check_sync.FAIL]
        assert not [c for c in checks if c.status == check_sync.SKIP]


@needs_repo
def test_generated_files_with_moved_rotated_base(tmp_path):
    s = scn_of(wall={'point': [1.0, 3.0, 2.0], 'normal': [0, -1, 0], 'u_range': [-1.0, 1.0], 'v_range': [0, 2]},
               patch={'mode': 'full'}, base={'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2, 'standoff_m': 1.0},
               consumers={'gazebo': {'sim_wall_offset_m': 0.05}})
    scenario_gen.generate(s, str(tmp_path))
    checks = check_sync.run_checks(s, **check_sync.gen_dir_files(str(tmp_path)))
    assert check_sync.verdict(checks) == 0, [c.as_dict() for c in checks if c.status == check_sync.FAIL]
    assert status(checks, 'plane distance', 'gazebo') == check_sync.PASS


# ------------------------------------------------------------------ checker: the real drift cases
def smoke_scene(frame='map', x=2.0, u=(-1.5, 1.5), v=(0.0, 2.0), z0=2.0, y0=0.0, obstacles=True):
    return {'frame_id': frame, 'walls': [{
        'id': 0, 'plane_abcd': [-1.0, 0.0, 0.0, x], 'origin_xyz': [x, y0, z0], 'basis_u': [0, 1, 0],
        'basis_v': [0, 0, -1], 'normal_outward': [-1, 0, 0], 'u_range_m': list(u), 'v_range_m': list(v)}],
        'obstacles': [{'id': 0, 'parent_wall': 0, 'protrusion_max_m': 0.05,
                       'uv_on_wall': {'u': [0.6, 0.9], 'v': [0.5, 0.8]}}] if obstacles else []}


def write(tmp_path, name, doc_):
    p = tmp_path / name
    p.write_text(json.dumps(doc_))
    return str(p)


OBSTACLE = [{'kind': 'box', 'u': [0.6, 0.9], 'v': [0.5, 0.8], 'depth': 0.05}]


def test_drift_x_2_vs_0_98_and_frame_map(tmp_path):
    s = scn_of(patch={'mode': 'full'}, obstacles=OBSTACLE)
    checks = check_sync.check_sim_scene(s, smoke_scene(), 'gazebo shipped smoke scene')
    assert status(checks, 'frame') == check_sync.FAIL
    assert status(checks, 'plane distance') == check_sync.FAIL
    detail = [c for c in checks if 'plane' in c.name][0].detail
    assert '2.000 m from the base' in detail and '-1.020' in detail
    assert status(checks, 'extents') == check_sync.PASS and status(checks, 'obstacle') == check_sync.PASS
    assert check_sync.verdict(checks) == 1


def test_drift_x_0_98_scene_passes_with_base_link_warning():
    s = scn_of(patch={'mode': 'full'}, obstacles=OBSTACLE)
    checks = check_sync.check_sim_scene(s, smoke_scene(frame='base_link', x=0.98), 'mujoco')
    assert status(checks, 'frame') == check_sync.WARN
    assert status(checks, 'plane distance') == check_sync.PASS
    assert check_sync.verdict(checks) == 0 and check_sync.verdict(checks, strict=True) == 1


def test_frame_names():
    s = scn_of()
    f = check_sync.frame_check
    assert f(s, 'x', 'frame', 'odom').status == check_sync.PASS
    assert f(s, 'x', 'frame', 'base_link').status == check_sync.WARN
    assert f(s, 'x', 'frame', 'base_footprint').status == check_sync.WARN
    assert f(s, 'x', 'frame', 'map').status == check_sync.FAIL
    assert f(s, 'x', 'frame', 'link_base').status == check_sync.FAIL
    assert f(s, 'x', 'frame', None).status == check_sync.SKIP
    moved = scn_of(base={'x': 0.1, 'y': 0.0, 'yaw': 0.0, 'standoff_m': 0.88})
    assert f(moved, 'x', 'frame', 'base_link').status == check_sync.FAIL   # not the same place any more


def test_drift_crop_vs_full_wall():
    full = scn_of(patch={'mode': 'full'})
    crop_scene = smoke_scene(frame='odom', x=0.98, u=(0.3, 1.2), v=(0.7, 1.0))
    # the plan was cropped, the scenario says the whole wall is planned
    checks = check_sync.check_plan_scene(full, crop_scene, 'plan')
    assert status(checks, 'extents vs work patch') == check_sync.FAIL
    share = [c for c in checks if c.name == 'share of the wall'][0]
    assert share.status == check_sync.WARN and 'covers 4.5 % of the wall' in share.detail
    # a crop declared in the scenario passes the extents but is still reported as 4.5 %
    crop = scn_of()
    checks = check_sync.check_plan_scene(crop, crop_scene, 'plan')
    assert status(checks, 'extents vs work patch') == check_sync.PASS
    assert [c for c in checks if c.name == 'share of the wall'][0].status == check_sync.WARN
    assert check_sync.verdict(checks) == 0


def test_drift_sim_wall_wider_than_scenario_wall():
    s = scn_of()
    # shift_wall_scene.py widens the sim wall with the patch: 3.6 m instead of 3.0 m
    checks = check_sync.check_sim_scene(s, smoke_scene(frame='odom', x=0.98, u=(-1.8, 1.8), obstacles=False), 'gazebo sim')
    ext = [c for c in checks if c.name.startswith('extents')][0]
    assert ext.status == check_sync.FAIL and 'max diff 0.300 m' in ext.detail


def test_patch_not_inside_wall_for_plan_scene():
    s = scn_of(patch={'mode': 'full'})
    checks = check_sync.check_plan_scene(s, smoke_scene(frame='odom', x=0.98, u=(-2.0, 2.0)), 'plan')
    assert status(checks, 'inside the wall') == check_sync.FAIL


def test_normal_flip_detected():
    s = scn_of(patch={'mode': 'full'})
    scene = smoke_scene(frame='odom', x=0.98)
    scene['walls'][0]['normal_outward'] = [1, 0, 0]
    assert status(check_sync.check_sim_scene(s, scene, 'x'), 'normal') == check_sync.FAIL


def plan_json(positions, frame='odom', area=None):
    recv = {'path': {'frame_id': frame, 'poses': []}}
    if area is not None:
        recv['coverage_area_m2'] = area
    return {'wall_scene_json': 'x', 'received_from_coverage_planner_node': recv,
            'final_lifted_and_expanded': {'path': {'frame_id': frame, 'poses': [
                {'position': {'x': p[0], 'y': p[1], 'z': p[2]}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}
                for p in positions]}}}


def lawnmower(x=0.9, y=(0.35, 1.15), z=(1.05, 1.25), n=5):
    pts = []
    for i in range(n):
        zz = z[0] + (z[1] - z[0]) * i / (n - 1)
        ys = y if i % 2 == 0 else y[::-1]
        pts += [(x, ys[0], zz), (x, ys[1], zz)]
    return pts


def test_plan_output_inside_patch_and_share_of_wall(tmp_path):
    s = scn_of()
    p = sources.read_plan_output(write(tmp_path, 'ok.json', plan_json(lawnmower(), area=0.25)))
    checks = check_sync.check_plan_output(s, p)
    assert check_sync.verdict(checks) == 0
    assert status(checks, 'bounding box') == check_sync.PASS
    share = [c for c in checks if c.name == 'share of the wall'][0]
    assert 'covers 4.2 % of the wall' in share.detail and share.status == check_sync.WARN


def test_plan_output_outside_patch_wrong_plane_wrong_frame(tmp_path):
    s = scn_of()
    p = sources.read_plan_output(write(tmp_path, 'bad.json', plan_json(lawnmower(x=1.95, y=(-1.0, 0.5)), frame='map')))
    checks = check_sync.check_plan_output(s, p)
    assert status(checks, 'bounding box') == check_sync.FAIL
    assert status(checks, 'distance of the poses') == check_sync.FAIL
    assert status(checks, 'frame') == check_sync.FAIL
    assert check_sync.verdict(checks) == 1


def test_plan_output_covers_too_little_of_patch(tmp_path):
    s = scn_of()
    p = sources.read_plan_output(write(tmp_path, 'small.json', plan_json(lawnmower(y=(0.35, 0.45), z=(1.1, 1.15)))))
    assert status(check_sync.check_plan_output(s, p), 'plan area vs the patch') == check_sync.FAIL


def test_plan_output_empty(tmp_path):
    s = scn_of()
    p = sources.read_plan_output(write(tmp_path, 'e.json', plan_json([])))
    assert check_sync.verdict(check_sync.check_plan_output(s, p)) == 1


def test_rviz_params(tmp_path):
    s = scn_of(consumers={'rviz': {'fixed_frame': 'odom'}})
    scene = write(tmp_path, 'wall_scene.json', smoke_scene(frame='odom', x=0.98, u=(0.3, 1.2), v=(0.7, 1.0), obstacles=False))
    good = {'scenario_sync': {'fixed_frame': 'odom', 'plan_frame': 'odom', 'wall_scene_json': scene,
                              'base_x': 0.0, 'base_y': 0.0, 'base_yaw': 0.0}}
    p = tmp_path / 'good.yaml'
    p.write_text(yaml.safe_dump(good))
    assert check_sync.verdict(check_sync.run_checks(s, rviz_args=str(p))) == 0
    bad = copy.deepcopy(good)
    bad['scenario_sync'].update(fixed_frame='map', plan_frame='link_base', base_x=0.6, base_yaw=1.57)
    p = tmp_path / 'bad.yaml'
    p.write_text(yaml.safe_dump(bad))
    checks = check_sync.run_checks(s, rviz_args=str(p))
    assert status(checks, 'fixed frame') == check_sync.FAIL and status(checks, 'plan frame') == check_sync.FAIL
    assert status(checks, 'base start pose') == check_sync.FAIL


@needs_repo
def test_sdf_wall_at_aligned_standoff_but_centred(tmp_path):
    """The Gazebo launch's own generator re-centres the wall on y = 0: a wall scenario at y 0.75 is not reproduced."""
    s = load_scenario(os.path.join(EX, 'mujoco_tall_wall.yaml'))
    wg = scenario_gen.load_module(os.path.join(REPO, 'src/simulation/sim_gazebo/sim_gazebo/wall_world.py'), '_t_gz')
    scene = scenario_gen.wall_scene(s, frame_id='odom')
    p = write(tmp_path, 'scene.json', scene)
    out = str(tmp_path / 'wall.sdf')
    wg.generate_world(p, out, wall_id=0, standoff=0.98)   # align=True like sim.launch.py
    checks = check_sync.check_sdf(s, out)
    assert status(checks, 'plane distance') == check_sync.PASS
    ext = [c for c in checks if c.name.startswith('extents')][0]
    assert ext.status == check_sync.FAIL and 'max diff 0.750 m' in ext.detail


# ------------------------------------------------------------------ CLI
def test_cli_exit_codes(tmp_path, capsys):
    ex = os.path.join(EX, 'gazebo_smoke_full.yaml')
    good = write(tmp_path, 'good.json', smoke_scene(frame='odom', x=0.98))
    bad = write(tmp_path, 'bad.json', smoke_scene())
    assert check_sync.main([ex, '--sim-scene', f'good={good}']) == 0
    out = capsys.readouterr().out
    assert 'in sync' in out and 'PASS' in out
    assert check_sync.main([ex, '--sim-scene', f'bad={bad}']) == 1
    assert 'MISMATCH' in capsys.readouterr().out
    assert check_sync.main([str(tmp_path / 'nope.yaml')]) == 2
    broken = tmp_path / 'broken.yaml'
    broken.write_text('name: x\n')
    assert check_sync.main([str(broken)]) == 2
    crop = os.path.join(EX, 'gazebo_patch_crop.yaml')
    scene = write(tmp_path, 'crop.json', smoke_scene(frame='odom', x=0.98, u=(0.3, 1.2), v=(0.7, 1.0), obstacles=False))
    assert check_sync.main([crop, '--plan-scene', scene]) == 0
    assert check_sync.main([crop, '--plan-scene', scene, '--strict']) == 1   # "covers 4.5 % of the wall" is a WARN
    assert check_sync.main([crop, '--plan-scene', str(tmp_path / 'missing.json')]) == 2


def test_cli_formats(tmp_path, capsys):
    ex = os.path.join(EX, 'gazebo_patch_crop.yaml')
    assert check_sync.main([ex, '--format', 'md']) == 0
    assert capsys.readouterr().out.startswith('| Source |')
    assert check_sync.main([ex, '--format', 'json']) == 0
    out = capsys.readouterr().out
    assert json.loads(out[:out.rindex(']') + 1])[0]['source'] == 'scenario'


def test_scenario_gen_cli(tmp_path, capsys):
    assert scenario_gen.main([os.path.join(EX, 'gazebo_patch_crop.yaml'), str(tmp_path), '--no-sim-generators']) == 0
    assert 'rviz_args' in capsys.readouterr().out
    assert scenario_gen.main([str(tmp_path / 'nope.yaml'), str(tmp_path)]) == 2
