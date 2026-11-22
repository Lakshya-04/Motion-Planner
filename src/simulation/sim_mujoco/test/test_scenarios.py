"""Obstacle scenarios: the scenario generator, the window hole in the sim wall, the recorder's obstacle list and
score_run's obstacle-contact count (the last one runs the installed score_run binary and is skipped without it)."""
import importlib.util
import json
import pathlib
import shutil
import subprocess

import pytest

from sim_mujoco.wall_world import load_scene

HERE = pathlib.Path(__file__).parent
DATA = HERE / 'data'
spec = importlib.util.spec_from_file_location('make_scenario_scene', HERE.parent / 'scripts' / 'make_scenario_scene.py')
mss = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mss)


def pcd_points(path):
    lines = path.read_text().splitlines()
    return [tuple(map(float, ln.split())) for ln in lines[lines.index('DATA ascii') + 1:] if ln.strip()]


def test_plain_is_the_stock_wall(tmp_path):
    info = mss.build(str(DATA), str(tmp_path), 'plain')
    assert info['OBSTACLES'] == 0 and info['MODE'] == 'single' and info['WALL_X'] == 0.98
    cov = json.loads((tmp_path / 'wall_pass_coverage_scene.json').read_text())
    assert cov['obstacles'] == []
    assert cov['walls'][0]['origin_xyz'] == [0.98, 0.15, 1.15]


def test_every_scenario_builds_and_both_scenes_carry_the_same_obstacles(tmp_path):
    for name in mss.SCENARIOS:
        out = tmp_path / name
        info = mss.build(str(DATA), str(out), name)
        sim = json.loads((out / 'smoke_wall_scene.json').read_text())
        cov = json.loads((out / 'wall_pass_coverage_scene.json').read_text())
        assert sim['obstacles'] == cov['obstacles'] and len(cov['obstacles']) == info['OBSTACLES']
        assert sim['walls'][0]['origin_xyz'] == cov['walls'][0]['origin_xyz']
        assert info['WALL_Y1'] - info['WALL_Y0'] == pytest.approx(info['WIDTH'])


def test_protrusion_is_a_box_in_the_sim_a_blocked_region_for_the_planner_and_points_in_the_cloud(tmp_path):
    mss.build(str(DATA), str(tmp_path), 'protrusion')
    cov = json.loads((tmp_path / 'wall_pass_coverage_scene.json').read_text())
    ob = cov['obstacles'][0]
    # planner: uv rectangle 0.07 x 0.07; the wall's origin is the patch centre (y 0.15, z 1.15), v = -z
    u0, u1 = ob['uv_on_wall']['u']
    v0, v1 = ob['uv_on_wall']['v']
    assert (u1 - u0, v1 - v0) == pytest.approx((0.07, 0.07))
    # sim: the box stands 35 mm in front of the wall plane x = 0.98 (toward the robot)
    prims, _ = load_scene(tmp_path / 'smoke_wall_scene.json')
    box = [p for p in prims if p.name.startswith('obstacle_')][0]
    assert box.pos[0] + box.size[2] == pytest.approx(0.98) and 2 * box.size[2] == pytest.approx(0.035)  # size is (u, v, n) half extents
    assert box.pos[1] == pytest.approx(0.15 + (u0 + u1) / 2) and box.pos[2] == pytest.approx(1.15 - (v0 + v1) / 2)
    # cloud: points on the box's face, label 1 (the planner reads label 0 only)
    face = [p for p in pcd_points(tmp_path / 'walls.pcd') if abs(p[0] - 0.945) < 1e-6]
    assert face and all(p[3] == 1 for p in face)


def test_window_is_a_hole_in_the_sim_wall_and_the_cloud(tmp_path):
    mss.build(str(DATA), str(tmp_path), 'window')
    prims, _ = load_scene(tmp_path / 'smoke_wall_scene.json')
    assert [p.name for p in prims] == ['wall_0', 'wall_0_1', 'wall_0_2', 'wall_0_3']
    # the pieces tile the wall minus the hole: area of the full wall (1.0 x 0.7) minus 0.10 x 0.08
    area = sum(4 * p.size[0] * p.size[1] for p in prims)
    assert area == pytest.approx(1.0 * 0.7 - 0.10 * 0.08)
    # nothing at the window's centre (y 0.15, z 1.15), wall material right next to it
    def covered(y, z):
        return any(abs(p.pos[1] - y) < p.size[0] and abs(p.pos[2] - z) < p.size[1] for p in prims)
    assert not covered(0.15, 1.15) and covered(0.15, 1.25) and covered(0.30, 1.15)
    pts = pcd_points(tmp_path / 'walls.pcd')
    assert not [p for p in pts if abs(p[1] - 0.15) < 0.04 and abs(p[2] - 1.15) < 0.03]
    assert [p for p in pts if abs(p[1] - 0.30) < 0.001 and abs(p[2] - 1.15) < 0.001]


def test_rib_is_full_height_and_the_scene_has_two_stand_width(tmp_path):
    info = mss.build(str(DATA), str(tmp_path), 'two_panels')
    assert info['MODE'] == 'multi' and info['WIDTH'] == 0.9
    ob = mss.obstacles_from_scene(str(tmp_path / 'wall_pass_coverage_scene.json'))[0]
    assert ob['u'] == pytest.approx([0.73, 0.77]) and ob['v'][0] < 1.0 and ob['v'][1] > 1.3
    assert ob['height_m'] == 0.03


def test_recorder_obstacles_are_in_the_scoring_walls_y_z(tmp_path):
    mss.build(str(DATA), str(tmp_path), 'protrusion')
    ob = mss.obstacles_from_scene(str(tmp_path / 'wall_pass_coverage_scene.json'))[0]
    # box centre y 0.08, z 1.19 (dy -0.07, dz +0.04 from the patch centre), 0.07 square, 35 mm high
    assert ob['u'] == pytest.approx([0.045, 0.115]) and ob['v'] == pytest.approx([1.155, 1.225])
    assert ob['height_m'] == 0.035


def test_unknown_scenario_is_rejected(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr('sys.argv', ['make_scenario_scene.py', str(DATA), str(tmp_path), 'nope'])
    with pytest.raises(SystemExit):
        mss.main()


def find_score_run():
    path = shutil.which('score_run')
    if path:
        return [path]
    ros2 = shutil.which('ros2')
    if ros2:
        r = subprocess.run([ros2, 'pkg', 'prefix', 'coverage_planner'], capture_output=True, text=True)
        exe = pathlib.Path(r.stdout.strip()) / 'lib' / 'coverage_planner' / 'score_run'
        if r.returncode == 0 and exe.exists():
            return [str(exe)]
    return None


@pytest.mark.skipif(find_score_run() is None, reason='coverage_planner score_run is not installed')
def test_score_run_counts_obstacle_contacts(tmp_path):
    mss.build(str(DATA), str(tmp_path), 'protrusion')
    ob = mss.obstacles_from_scene(str(tmp_path / 'wall_pass_coverage_scene.json'))
    wall = {'origin': [0.98, 0.0, 0.0], 'u': [0, 1, 0], 'v': [0, 0, 1], 'u_range': [-0.1, 0.4], 'v_range': [1.0, 1.3],
            'obstacles': ob}
    (tmp_path / 'wall.json').write_text(json.dumps(wall))
    # a vertical stroke at y = 0.3 (clear of the box, y 0.045..0.115), then one at y = 0.08 straight through it
    rows = ['t,x,y,z,force,active']
    t = 0.0
    for y in (0.3, 0.08):
        for i in range(31):
            rows.append(f'{t:.2f},0.98,{y},{1.0 + 0.01 * i},10.0,0')
            t += 0.1
        rows.append(f'{t:.2f},0.9,{y},1.3,0.0,0')
        t += 0.1
    (tmp_path / 'run.csv').write_text('\n'.join(rows) + '\n')

    def score(*extra):
        r = subprocess.run(find_score_run() + ['--samples', str(tmp_path / 'run.csv'), '--wall', str(tmp_path / 'wall.json'),
                                               '--swath', '0.14', '--min-coverage', '0'] + list(extra),
                           capture_output=True, text=True)
        return r.returncode, json.loads(r.stdout)
    rc, card = score()
    assert card['obstacle']['contacts'] == 1 and card['obstacle']['contact_samples'] > 5 and rc == 0
    rc, card = score('--max-obstacle-contacts', '0')
    assert rc == 1 and any('obstacle contacts' in f for f in card['failures'])
    # the same strokes with the tool kept 6 cm off the wall at the box are not a contact
    rows = [r if i == 0 or float(r.split(',')[2]) != 0.08 else r.replace(',0.98,', ',0.92,') for i, r in enumerate(rows)]
    (tmp_path / 'run.csv').write_text('\n'.join(rows) + '\n')
    rc, card = score('--max-obstacle-contacts', '0')
    assert card['obstacle']['contacts'] == 0 and rc == 0
    # grown keep-out zone leaves less wall to cover
    _, a = score()
    _, b = score('--keepout-margin', '0.07')
    assert b['coverage']['target_cells'] < a['coverage']['target_cells']


# ---- tall wall: the patch is high enough that the lift must telescope
def test_two_panels_tall_has_a_tall_wall_a_wide_patch_and_a_matching_curtain(tmp_path):
    info = mss.build(str(DATA), str(tmp_path), 'two_panels_tall')
    assert (info['PATCH_Z_LO'], info['PATCH_Z_HI']) == (0.6, 2.0) and (info['WALL_Z_LO'], info['WALL_Z_HI']) == (0.3, 2.3)
    sim = json.loads((tmp_path / 'smoke_wall_scene.json').read_text())['walls'][0]
    cov = json.loads((tmp_path / 'wall_pass_coverage_scene.json').read_text())['walls'][0]
    assert sim['origin_xyz'][2] == cov['origin_xyz'][2] == pytest.approx(1.3)
    # basis_v = -z: z = origin z - v
    assert [1.3 - v for v in sim['v_range_m']] == pytest.approx([2.3, 0.3])
    assert [1.3 - v for v in cov['v_range_m']] == pytest.approx([2.0, 0.6])
    prims, _ = load_scene(tmp_path / 'smoke_wall_scene.json')
    wall = [p for p in prims if not p.name.startswith('obstacle_')][0]
    assert wall.pos[2] == pytest.approx(1.3) and wall.size[1] == pytest.approx(1.0)   # half height of the 2 m wall
    pts = pcd_points(tmp_path / 'walls.pcd')
    zs = [p[2] for p in pts]
    assert min(zs) == pytest.approx(0.55) and max(zs) == pytest.approx(2.05)
    sim_pts = pcd_points(tmp_path / 'walls_sim.pcd')
    assert max(p[2] for p in sim_pts) >= 2.6 - 0.02 and min(p[2] for p in sim_pts) <= 0.02   # curtain covers the wall


def test_stock_scenarios_keep_the_stock_patch_and_z_override_widens_the_wall(tmp_path):
    info = mss.build(str(DATA), str(tmp_path / 'a'), 'two_panels_plain')
    assert (info['PATCH_Z_LO'], info['PATCH_Z_HI'], info['WALL_Z_LO']) == (1.0, 1.3, 'default')
    cov = json.loads((tmp_path / 'a' / 'wall_pass_coverage_scene.json').read_text())['walls'][0]
    assert cov['origin_xyz'][2] == 1.15 and cov['v_range_m'] == [-0.15, 0.15]
    info = mss.build(str(DATA), str(tmp_path / 'b'), 'two_panels_plain', patch_z=(0.8, 1.9))
    assert (info['WALL_Z_LO'], info['WALL_Z_HI']) == (0.5, 2.2)
    with pytest.raises(ValueError):
        mss.build(str(DATA), str(tmp_path / 'c'), 'two_panels_plain', patch_z=(1.9, 0.8))
    with pytest.raises(ValueError):
        mss.patch_and_wall_z(dict(patch_z=(0.6, 2.0), wall_z=(0.8, 2.3)))
