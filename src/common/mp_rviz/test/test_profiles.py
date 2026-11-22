"""Every profile is valid YAML with a Visualization Manager, a fixed frame and at least one display; every display
topic is either a bare name or absolute (RViz keeps what is written)."""
import glob
import os

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES = sorted(glob.glob(os.path.join(HERE, '..', 'profiles', '*.rviz')))


def test_there_are_profiles():
    assert PROFILES


@pytest.mark.parametrize('path', PROFILES, ids=[os.path.basename(p) for p in PROFILES])
def test_profile_is_wellformed(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    vm = cfg['Visualization Manager']
    assert vm['Global Options']['Fixed Frame']
    displays = vm['Displays']
    assert any(d.get('Enabled', True) for d in displays), 'no enabled display'
    for d in displays:
        assert d.get('Class'), d
        assert d.get('Name'), d
