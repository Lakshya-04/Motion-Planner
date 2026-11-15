"""contact_test.py helpers (cleanup, release timing) and run_contact.sh process handling, without a sim."""
import os
import pathlib
import subprocess
import sys
import textwrap
import time

import pytest

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / 'scripts'


def _contact_test():
    pytest.importorskip('rclpy')
    pytest.importorskip('control_msgs')
    sys.path.insert(0, str(SCRIPTS))
    import contact_test
    return contact_test


class FakeNode:
    def __init__(self, fail_axes=False):
        self.calls, self.fail_axes = [], fail_axes

    def set_axes(self, axes):
        self.calls.append(('axes', list(axes)))
        if self.fail_axes:
            raise RuntimeError('service call timed out')

    def set_target(self, fz):
        self.calls.append(('target', fz))


def test_abort_disengages_and_clears_the_target():
    ct = _contact_test()
    node = FakeNode()

    def body():
        raise RuntimeError('service call timed out')

    with pytest.raises(RuntimeError):
        ct.guarded(node, body)
    assert node.calls == [('axes', [False] * 6), ('target', 0.0)]


def test_cleanup_clears_the_target_even_when_the_axes_call_fails():
    ct = _contact_test()
    node = FakeNode(fail_axes=True)
    ct.release(node)
    assert ('target', 0.0) in node.calls


def test_release_time_ignores_a_bounce_and_needs_a_quiet_window():
    ct = _contact_test()
    # force vanishes at t=2.0 and stays gone for 1.5 s: released 2.0 s after the start
    steady = [(0.1 * i, 8.0 if 0.1 * i < 2.0 else 0.0) for i in range(36)]
    assert ct.release_time(steady, 0.0) == pytest.approx(1.9)
    # a single zero sample at t=1.0 and force again afterwards: not released
    bounce = [(0.1 * i, 0.0 if i == 10 else 8.0) for i in range(36)]
    assert ct.release_time(bounce, 0.0) is None               # force came back, no quiet window after it
    assert ct.release_time([], 0.0) is None


def test_run_contact_stops_the_whole_launch_tree(tmp_path):
    """The launch ignores SIGINT and has a child: both must be gone when the script returns."""
    marker = f'sleep {300 + os.getpid() % 100}.5'
    (tmp_path / 'share/sim_mujoco').mkdir(parents=True)
    (tmp_path / 'lib/sim_mujoco').mkdir(parents=True)
    (tmp_path / 'lib/sim_mujoco/contact_test.py').write_text('import sys\nsys.exit(0)\n')
    fake = tmp_path / 'bin'
    fake.mkdir()
    ros2 = fake / 'ros2'
    ros2.write_text(textwrap.dedent(f'''\
        #!/bin/bash
        if [ "$1" = pkg ]; then echo {tmp_path}; exit 0; fi
        trap '' INT
        {marker} &
        wait
        '''))
    ros2.chmod(0o755)
    env = dict(os.environ, PATH=f'{fake}:{os.environ["PATH"]}', CONTACT_STOP_WAIT='1',
               CONTACT_LOG=str(tmp_path / 'launch.log'))
    try:
        r = subprocess.run(['bash', str(SCRIPTS / 'run_contact.sh')], env=env, timeout=60,
                           capture_output=True, text=True)
        assert r.returncode == 0, r.stdout + r.stderr
        time.sleep(0.3)
        left = subprocess.run(['pgrep', '-f', marker], capture_output=True, text=True).stdout.split()
        assert left == [], f'orphaned launch children: {left}'
    finally:
        subprocess.run(['pkill', '-f', marker])
