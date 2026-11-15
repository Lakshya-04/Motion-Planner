"""contact_test.py helpers (cleanup, release timing) and run_contact.sh process handling, without a sim."""
import os
import pathlib
import subprocess
import sys
import textwrap
import time

import pytest

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / 'scripts'


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
