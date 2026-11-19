"""Tests for the URDF helper scripts (no docker, no GPU, no ROS: ament_index_python is stubbed).

    python -m pytest docker/tools/tests/test_helper_scripts.py
"""
import pathlib
import subprocess
import sys
import types

import pytest

TOOLS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))


@pytest.fixture()
def resolve_module(tmp_path, monkeypatch):
    share = tmp_path / 'a' / 'b' / 'share'
    (share / 'meshes').mkdir(parents=True)
    (share / 'meshes' / 'a.stl').write_text('solid')
    (tmp_path / 'a' / 'secret.stl').write_text('solid')   # what ../../secret.stl names under the share dir
    stub = types.ModuleType('ament_index_python.packages')
    stub.get_package_share_directory = lambda pkg: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python', types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', stub)
    sys.modules.pop('resolve_package_uris', None)
    import resolve_package_uris
    return resolve_package_uris


def _urdf(path, uri):
    path.write_text(f'<robot name="r"><link name="l"><collision><geometry><mesh filename="{uri}"/>'
                    '</geometry></collision></link></robot>')
    return path


def test_resolve_copies_a_package_mesh(resolve_module, tmp_path):
    out = tmp_path / 'out'
    assert resolve_module.resolve(_urdf(tmp_path / 'r.urdf', 'package://pkg/meshes/a.stl'), out) == 1
    assert (out / 'pkg' / 'meshes' / 'a.stl').is_file()


def test_resolve_rejects_a_path_that_leaves_the_output_directory(resolve_module, tmp_path):
    out = tmp_path / 'out'
    with pytest.raises(ValueError, match='leaves the output directory'):
        resolve_module.resolve(_urdf(tmp_path / 'r.urdf', 'package://pkg/../../secret.stl'), out)
    assert not (tmp_path / 'secret.stl').exists()           # nothing was copied outside out_dir


def test_primitives_to_meshes_skips_a_collision_without_geometry(tmp_path, capsys):
    import primitives_to_meshes as p2m
    src = tmp_path / 'r.urdf'
    src.write_text('<robot name="r"><link name="l"><collision/><collision><geometry>'
                   '<box size="1 1 1"/></geometry></collision></link></robot>')
    assert p2m.convert(str(src), tmp_path / 'out') == 1
    assert 'without <geometry>' in capsys.readouterr().err


def test_generate_vamp_robot_refuses_a_urdf_without_spheres(tmp_path):
    out = tmp_path / 'out'
    out.mkdir()
    (out / 'spherized.urdf').write_text('<robot name="r"><link name="l"/></robot>')
    cricket = tmp_path / 'cricket' / 'resources' / 'templates'
    cricket.mkdir(parents=True)
    urdf = tmp_path / 'r.urdf'
    urdf.write_text('<robot name="r"/>')
    env = {'PATH': '/usr/bin:/bin', 'SKIP_FOAM': '1', 'CRICKET_SRC': str(tmp_path / 'cricket')}
    r = subprocess.run(['bash', str(TOOLS / 'generate_vamp_robot.sh'), 'bot', str(urdf), '-', 'ee', str(out)],
                       env=env, capture_output=True, text=True)
    assert r.returncode == 1 and 'no <sphere> collision' in r.stderr
