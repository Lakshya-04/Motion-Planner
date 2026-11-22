"""Synthetic cloud -> wall fit (python mirror of the detector core) -> comparison with the ground truth scene.
Host only: numpy + pytest, no ROS."""
import json
import pathlib
import sys

import numpy as np
import pytest

SCRIPTS = pathlib.Path(__file__).resolve().parents[2] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import compare_wall_scene as cws  # noqa: E402
import wall_fit_ref as ref  # noqa: E402

# the sim's wall: plane x = 0.98 facing -x, patch y 0.3..1.2, z 1.0..1.3 (the baked coverage scene's convention)
Y0, Y1, Z0, Z1, X = 0.3, 1.2, 1.0, 1.3, 0.98


def truth_scene(obstacles=()):
    return dict(frame_id='odom', units='meters', obstacles=list(obstacles), walls=[dict(
        id=0, plane_abcd=[-1.0, 0.0, 0.0, X], origin_xyz=[X, (Y0 + Y1) / 2, (Z0 + Z1) / 2], basis_u=[0, 1, 0],
        basis_v=[0, 0, -1], normal_outward=[-1, 0, 0], u_range_m=[-(Y1 - Y0) / 2, (Y1 - Y0) / 2],
        v_range_m=[-(Z1 - Z0) / 2, (Z1 - Z0) / 2])])


def truth_box(cy, cz, w, h, depth):
    return dict(id=0, parent_wall=0, kind='box', box_center_xyz=[X - depth / 2, cy, cz], box_extents_uvn=[w, h, depth])


def wall_cloud(noise=0.0, step=0.01, y=(Y0, Y1), z=(Z0, Z1), tilt_deg=0.0, boxes=(), seed=3):
    rng = np.random.default_rng(seed)
    yy, zz = np.meshgrid(np.arange(y[0], y[1] + 1e-9, step), np.arange(z[0], z[1] + 1e-9, step))
    pts = np.c_[np.full(yy.size, X), yy.ravel(), zz.ravel()]
    for cy, cz, w, h, depth in boxes:
        sy, sz = np.meshgrid(np.arange(cy - w / 2, cy + w / 2, step / 2), np.arange(cz - h / 2, cz + h / 2, step / 2))
        pts = np.r_[pts, np.c_[np.full(sy.size, X - depth), sy.ravel(), sz.ravel()]]
    t = np.radians(tilt_deg)
    rot = np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])   # about y
    pts = (pts - [X, 0, 1.15]) @ rot.T + [X, 0, 1.15]
    return pts + rng.normal(0.0, noise, pts.shape) if noise else pts


def detect(cloud, **kw):
    det = ref.detect_wall(cloud, viewpoint=(0, 0, 0), **kw)
    assert det is not None
    return det, ref.wall_scene_json(det)


def test_clean_cloud_recovers_the_wall():
    det, scene = detect(wall_cloud())
    r = cws.compare(scene, truth_scene())
    assert r['angle_deg'] < 0.05 and r['offset_mm'] < 0.5
    assert r['extent']['max_edge_error_mm'] < 1.0 and r['extent']['iou'] > 0.99
    assert det['normal'][0] < 0                        # toward the sensor, like the baked scene


@pytest.mark.parametrize('noise', [0.001, 0.003, 0.005])
def test_noisy_cloud_stays_within_tolerance(noise):
    _, scene = detect(wall_cloud(noise=noise))
    r = cws.compare(scene, truth_scene())
    assert r['angle_deg'] < 0.5 and r['offset_mm'] < 3.0
    # noise points beyond the inlier threshold shave the rim: the extent is good to a few cells
    assert r['extent']['max_edge_error_mm'] < 40.0
    assert r['extent']['iou'] > 0.9


def test_tilted_wall_angle_is_measured():
    _, scene = detect(wall_cloud(tilt_deg=3.0, noise=0.001), inlier_threshold=0.02)
    r = cws.compare(scene, truth_scene())
    assert 2.7 < r['angle_deg'] < 3.3


def test_outliers_and_a_floor_do_not_move_the_wall():
    rng = np.random.default_rng(5)
    floor = np.c_[rng.uniform(0.2, 0.9, 400), rng.uniform(0.3, 1.2, 400), np.zeros(400)]
    spikes = rng.uniform([0.2, 0.0, 0.0], [0.95, 1.5, 2.0], (300, 3))
    cloud = np.r_[wall_cloud(noise=0.002), floor, spikes]
    _, scene = detect(cloud)
    r = cws.compare(scene, truth_scene())
    assert r['angle_deg'] < 0.5 and r['offset_mm'] < 3.0


def test_obstacle_recall_zero_for_the_wall_detector_core_one_for_the_reference_stage():
    box = (0.75, 1.15, 0.07, 0.07, 0.035)
    det, scene = detect(wall_cloud(boxes=[box]))
    truth = truth_scene([truth_box(*box[:2], box[2], box[3], box[4])])
    r0 = cws.compare(scene, truth)                       # what wall_detector writes: obstacles []
    assert r0['obstacles']['recall'] == 0.0 and r0['obstacles']['truth'] == 1
    assert r0['angle_deg'] < 0.1                         # the box did not pull the plane
    scene['obstacles'] = ref.detect_obstacles(det)
    r1 = cws.compare(scene, truth)
    assert r1['obstacles']['recall'] == 1.0 and r1['obstacles']['false_positives'] == 0


def test_recall_is_none_without_true_obstacles():
    _, scene = detect(wall_cloud())
    assert cws.compare(scene, truth_scene())['obstacles']['recall'] is None


def test_extent_errors_show_a_wall_that_is_too_short():
    _, scene = detect(wall_cloud(y=(Y0, Y1 - 0.3)))
    r = cws.compare(scene, truth_scene())
    assert 280.0 < r['extent']['max_edge_error_mm'] < 320.0 and r['extent']['iou'] < 0.75


def test_different_in_plane_axes_are_compared_in_the_truth_basis():
    # the detector's own basis (u = up x n = -y, v = z) differs from the scene file's (u = y, v = -z)
    det, scene = detect(wall_cloud())
    assert abs(scene['walls'][0]['basis_u'][1] + 1.0) < 1e-6
    assert cws.compare(scene, truth_scene())['extent']['max_edge_error_mm'] < 1.0


def test_cli_exit_code_and_json(tmp_path, capsys):
    _, scene = detect(wall_cloud(noise=0.002))
    (tmp_path / 'd.json').write_text(json.dumps(scene))
    (tmp_path / 't.json').write_text(json.dumps(truth_scene()))
    out = tmp_path / 'r.json'
    assert cws.main([str(tmp_path / 'd.json'), str(tmp_path / 't.json'), '--json', str(out)]) == 0
    assert json.loads(out.read_text())['ok'] is True
    assert 'perception check: OK' in capsys.readouterr().out
    assert cws.main([str(tmp_path / 'd.json'), str(tmp_path / 't.json'), '--max-angle-deg', '0.0001']) == 1


def test_no_wall_in_a_sparse_cloud():
    assert ref.detect_wall(np.random.default_rng(1).uniform(0, 1, (150, 3))) is None


SIM = SCRIPTS.parents[2] / 'simulation' / 'sim_mujoco'


@pytest.mark.skipif(not (SIM / 'scripts' / 'make_scenario_scene.py').exists(), reason='sim_mujoco not in the tree')
def test_the_two_stand_pass_pipeline_on_the_sim_cloud(tmp_path):
    """What MULTI_STAND_PERCEPTION=1 does, without ROS: walls_sim.pcd (the sim wall's cloud incl. its curtain) ->
    detector -> crop to the contact patch (crop_wall_scene.py) -> compare with the baked planner scene."""
    import subprocess
    sys.path.insert(0, str(SIM / 'scripts'))
    import make_scenario_scene as mss
    info = mss.build(str(SIM / 'test' / 'data'), str(tmp_path), 'two_panels_plain')
    lines = (tmp_path / 'walls_sim.pcd').read_text().splitlines()
    cloud = np.array([[float(t) for t in ln.split()[:3]] for ln in lines[lines.index('DATA ascii') + 1:] if ln.strip()])
    det = ref.detect_wall(cloud, viewpoint=(0, 0, 0))
    assert det is not None
    (tmp_path / 'wall_scene.json').write_text(json.dumps(ref.wall_scene_json(det)))
    subprocess.run([sys.executable, str(SCRIPTS.parents[2] / 'simulation' / 'sim_mujoco' / 'scripts' / 'crop_wall_scene.py'),
                    str(tmp_path / 'wall_scene.json'), str(tmp_path / 'crop.json'), '--y', str(info['WALL_Y0']),
                    str(info['WALL_Y1']), '--z', str(info['PATCH_Z_LO']), str(info['PATCH_Z_HI'])], check=True,
                   capture_output=True)
    crop = json.loads((tmp_path / 'crop.json').read_text())
    truth = json.loads((tmp_path / 'wall_pass_coverage_scene.json').read_text())
    r = cws.compare(crop, truth)
    # a noise-free cloud: the fit is exact, the crop reproduces the planner's patch to the cell size
    assert r['angle_deg'] < 0.01 and r['offset_mm'] < 0.1
    assert r['extent']['max_edge_error_mm'] < 60.0, r['extent']
