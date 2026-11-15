"""The Isaac Sim backend is a stub; pin that so it is never taken for a working one."""
import importlib.util
import pathlib

from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown

PKG = pathlib.Path(__file__).resolve().parents[1]


def _launch():
    spec = importlib.util.spec_from_file_location('isaac_launch', PKG / 'launch' / 'isaac.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_shuts_down_with_a_clear_message():
    module = _launch()
    entities = module.generate_launch_description().entities
    assert any(isinstance(e, LogInfo) for e in entities)
    shutdowns = [e for e in entities if isinstance(e, EmitEvent) and isinstance(e.event, Shutdown)]
    assert len(shutdowns) == 1
    assert 'not implemented' in module.NOT_IMPLEMENTED
    # it must not send users to backends that may not exist on their checkout
    assert 'hardware:=gazebo' not in module.NOT_IMPLEMENTED
    assert 'docs/simulation.md' in module.NOT_IMPLEMENTED


def test_readme_says_it_is_a_stub():
    text = (PKG / 'README.md').read_text()
    assert 'stub' in text.lower() and 'Nothing here simulates' in text

