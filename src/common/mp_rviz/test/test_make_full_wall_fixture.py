"""Host tests of make_full_wall_fixture.py output."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, '..', 'scripts', 'make_full_wall_fixture.py')
KEYS = ['observed_area_m2', 'observed_fraction', 'plane_rms_mm', 'spread_p95_mm', 'spread_p99_mm', 'points',
        'merged_layers', 'layer_offset_spread_mm', 'plane_abcd', 'origin_xyz', 'basis_u', 'basis_v', 'u_range_m',
        'v_range_m']


def test_fixture_complete(tmp_path):
    subprocess.run([sys.executable, SCRIPT, str(tmp_path), '--step', '0.1'], check=True, capture_output=True)
    scene = json.load(open(tmp_path / 'wall_scene.json'))
    w = scene['walls'][0]
    assert all(k in w for k in KEYS)
    assert w['u_range_m'] == [-1.5, 1.5] and w['v_range_m'] == [0.0, 2.0]
    lines = (tmp_path / 'walls.pcd').read_text().splitlines()
    hdr = {l.split()[0]: l.split()[1:] for l in lines[:11] if not l.startswith('#')}
    assert hdr['FIELDS'] == ['x', 'y', 'z', 'label'] and hdr['DATA'] == ['ascii']
    body = lines[11:]
    assert len(body) == int(hdr['POINTS'][0]) == w['points'] == 30 * 20
    xs = [list(map(float, l.split())) for l in body]
    assert all(abs(r[0] - 2.0) < 1e-6 and -1.5 <= r[1] <= 1.5 and 0 <= r[2] <= 2.0 and r[3] == 0 for r in xs)
