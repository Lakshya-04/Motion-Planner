import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sim_overlay import gz_commands as gc  # noqa: E402
from sim_overlay.overlay import OverlayState  # noqa: E402

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'gz_overlay.py')


def prims():
    s = OverlayState()
    s.update('/mp_viz/plan_columns', [{'type': 'LINE_STRIP', 'ns': 'plan_columns', 'id': 0, 'color': (1, 0, 0, 1),
                                       'points': [(0, 0, 0), (0, 0, 1)], 'scale': (0.02, 0, 0)}])
    s.update('/mp_viz/stands', [{'type': 'TEXT_VIEW_FACING', 'ns': 'stand_text', 'id': 1, 'text': 'S1',
                                 'pose': (1, 2, 0.4, 0, 0, 0, 1), 'scale': (0, 0, 0.15), 'color': (1, 1, 1, 1)},
                                {'type': 'ARROW', 'ns': 'stand_arrow', 'id': 1, 'scale': (0.5, 0.06, 0.06),
                                 'color': (0, 1, 0, 1)}])
    return s.primitives()


def test_command_shape():
    cmds, ns = gc.plan_commands(prims())
    assert ns == ['plan_columns', 'stand_arrow', 'stand_text']
    c = cmds[0]
    assert c[:8] == ['gz', 'service', '-s', '/marker', '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Boolean']
    assert c[-1] == 'ns: "mp_overlay/plan_columns", action: DELETE_ALL'
    adds = [x[-1] for x in cmds[3:]]
    assert any('type: LINE_STRIP' in a and 'point: {x: 0.0000, y: 0.0000, z: 1.0000}' in a for a in adds)
    assert any('type: TEXT' in a and 'text: "S1"' in a for a in adds)
    assert all('ADD_MODIFY' in a for a in adds)


def test_previous_namespaces_are_cleared():
    cmds, _ = gc.plan_commands([], previous_ns=['old'])
    assert len(cmds) == 1 and 'mp_overlay/old' in cmds[0][-1]


def test_dry_run_cli(tmp_path):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from sim_overlay.mujoco_overlay import save_overlay
    f = tmp_path / 'o.json'
    save_overlay(str(f), prims())
    out = subprocess.run([sys.executable, SCRIPT, '--from-json', str(f), '--dry-run'], capture_output=True,
                         text=True, check=True).stdout.splitlines()
    assert out and all(l.startswith('gz service -s /marker') for l in out)
    assert any('DELETE_ALL' in l for l in out) and any('TEXT' in l for l in out)
    out = subprocess.run([sys.executable, SCRIPT, '--from-json', str(f), '--dry-run', '--clear'],
                         capture_output=True, text=True, check=True).stdout.splitlines()
    assert len(out) == 3 and all('DELETE_ALL' in l for l in out)
