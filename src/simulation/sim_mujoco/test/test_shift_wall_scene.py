"""shift_wall_scene.py: the shifted and widened wall files the stand passes use (pure Python)."""
import importlib.util
import json
import pathlib

HERE = pathlib.Path(__file__).parent
DATA = HERE / 'data'
spec = importlib.util.spec_from_file_location('shift_wall_scene', HERE.parent / 'scripts' / 'shift_wall_scene.py')
sws = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sws)


def pcd_points(path):
    lines = path.read_text().splitlines()
    return [tuple(map(float, ln.split()[:3])) for ln in lines[lines.index('DATA ascii') + 1:] if ln.strip()]


def run(tmp_path, dx, dy, width=None):
    argv = [str(DATA), str(tmp_path), str(dx), str(dy)] + ([] if width is None else [str(width)])
    import sys
    old, sys.argv = sys.argv, ['shift_wall_scene.py'] + argv
    try:
        sws.main()
    finally:
        sys.argv = old


def test_default_width_only_shifts(tmp_path):
    run(tmp_path, 0.0, 0.9)
    wall = json.loads((tmp_path / 'wall_pass_coverage_scene.json').read_text())['walls'][0]
    assert wall['origin_xyz'] == [0.98, 1.05, 1.15]
    assert wall['u_range_m'] == [-0.15, 0.15]
    ys = [p[1] for p in pcd_points(tmp_path / 'walls.pcd')]
    assert min(ys) == 0.85 and max(ys) == 1.25


def test_wider_patch_starts_at_dy_and_grows_both_sides(tmp_path):
    run(tmp_path, 0.1, 0.3, 0.9)
    cov = json.loads((tmp_path / 'wall_pass_coverage_scene.json').read_text())['walls'][0]
    # patch y 0.3 .. 1.2: centre 0.75, half width 0.45
    assert abs(cov['origin_xyz'][1] - 0.75) < 1e-9
    assert cov['u_range_m'] == [-0.45, 0.45]
    assert cov['plane_abcd'][3] == 0.98 + 0.1
    sim = json.loads((tmp_path / 'smoke_wall_scene.json').read_text())['walls'][0]
    assert abs(sim['origin_xyz'][1] - 0.75) < 1e-9
    assert sim['u_range_m'] == [-0.7, 0.7]
    pts = pcd_points(tmp_path / 'walls.pcd')
    ys = [p[1] for p in pts]
    assert abs(min(ys) - 0.25) < 1e-9 and abs(max(ys) - 1.25) < 1e-9
    assert {p[0] for p in pts} == {1.08}
    header = (tmp_path / 'walls.pcd').read_text().splitlines()
    assert f'POINTS {len(pts)}' in header and f'WIDTH {len(pts)}' in header
