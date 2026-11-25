"""Static checks that every build stage of docker/Dockerfile gets ccache + mold by default (no docker needed).

    python -m pytest docker/tools/tests/test_dockerfile_ccache.py
"""
import pathlib
import re

import pytest
import yaml

DOCKER = pathlib.Path(__file__).resolve().parents[2]
TEXT = (DOCKER / 'Dockerfile').read_text().replace('\\\n', ' ')
COMPILES = re.compile(r'colcon |cmake |mp-build|pip install')


def stages():
    out, cur = {}, None
    for line in TEXT.splitlines():
        m = re.match(r'FROM\s+\S+\s+AS\s+(\S+)', line)
        if m:
            cur = m.group(1)
            out[cur] = {'parent': line.split()[1], 'body': []}
        elif cur:
            out[cur]['body'].append(line)
    return out


def ancestors(name, st):
    chain = [name]
    while st[chain[-1]]['parent'] in st:
        chain.append(st[chain[-1]]['parent'])
    return chain


def env_text(name, st):
    return '\n'.join(l for a in ancestors(name, st) for l in st[a]['body'] if l.startswith('ENV'))


def test_base_installs_ccache_and_mold():
    base = '\n'.join(stages()['base']['body'])
    assert re.search(r'apt-get install[^&]*\bccache\b', base) and re.search(r'apt-get install[^&]*\bmold\b', base)


@pytest.mark.parametrize('name', sorted(stages()))
def test_stage_inherits_cache_env_when_it_compiles(name):
    st = stages()
    body = '\n'.join(st[name]['body'])
    if not COMPILES.search(body) and name not in ('dev', 'sim-gazebo'):
        pytest.skip('stage compiles nothing')
    env = env_text(name, st)
    for var in ('CCACHE_DIR=/ccache', 'CCACHE_MAXSIZE', 'CCACHE_COMPILERCHECK=content', 'CCACHE_BASEDIR=/ws',
                'CMAKE_C_COMPILER_LAUNCHER=ccache', 'CMAKE_CXX_COMPILER_LAUNCHER=ccache',
                'CMAKE_CUDA_COMPILER_LAUNCHER=ccache', 'COLCON_DEFAULTS_FILE=/etc/colcon/defaults.yaml'):
        assert var in env, f'{name}: {var} missing'


def test_cache_dir_is_world_writable():
    assert 'chmod 1777 /ccache' in TEXT


def test_defaults_file_is_installed_and_has_launchers_and_mold():
    assert 'docker/colcon-defaults.yaml /etc/colcon/defaults.yaml' in TEXT
    args = yaml.safe_load((DOCKER / 'colcon-defaults.yaml').read_text())['build']['cmake-args']
    for a in ('-DCMAKE_C_COMPILER_LAUNCHER=ccache', '-DCMAKE_CXX_COMPILER_LAUNCHER=ccache',
              '-DCMAKE_CUDA_COMPILER_LAUNCHER=ccache', '-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold'):
        assert a in args
    assert not any('BUILD_TYPE' in a for a in args)


def test_compose_mounts_ccache_for_every_service():
    c = yaml.safe_load((DOCKER / 'compose.yaml').read_text())
    assert 'ccache' in c['volumes']
    for svc in c['services'].values():
        assert any(str(v).endswith(':/ccache') for v in svc['volumes']), svc
