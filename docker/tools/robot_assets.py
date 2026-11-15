#!/usr/bin/env python3
"""Driver behind docker/tools/generate_robot_assets.sh: every per-robot artefact from one profile.

    docker/tools/generate_robot_assets.sh <profile.yaml> [--out DIR] [--only a,b] [--skip a,b]

Stages, in order, and the image each one runs in (see docs/robot_assets.md):

  urdf     xacro -> robot.urdf (full) and arm_only/ (lift pinned, package:// meshes resolved)   build
  srdf     xacro -> robot.srdf (groups + structural allowed-collision matrix), joint_limits      build
  spheres  FOAM collision spheres of the arm-only model                                          foam
  vamp     --hops 2 rigid-group SRDF, then Cricket -> <Name>.hh                                  cricket
  curobo   cuRobo robot config (spheres + collision matrix), needs a GPU                         gpu
  reach    urdf_to_chain.py chain YAML + the RM4D map                                            build
  check    self-collision sanity on the spheres: home free, folded collides                      build
  manifest versions, hashes, sphere counts, check results -> manifest.json                       host

Existing scripts are reused unchanged (generate_vamp_robot.sh, rigid_pairs_srdf.py,
generate_curobo_robot.py, sphere_self_collision.py, urdf_to_chain.py, generate_acm's matrices); this file
is glue. Stages read the files earlier stages left in the output directory, so any subset can be re-run.
The same file runs inside the images with --inner <stage>.
"""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import re
import shlex
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[2]
TOOLS = REPO / 'docker' / 'tools'
DESC = REPO / 'src' / 'description' / 'mobile_manipulator_description'
STAGES = ['urdf', 'srdf', 'spheres', 'vamp', 'curobo', 'reach', 'check', 'manifest']
IMAGES = {
    'build': os.environ.get('BUILD_IMG', 'motion-planner:build'),
    'gpu': os.environ.get('GPU_IMG', 'motion-planner:gpu'),
    'full': os.environ.get('FULL_IMG', 'motion-planner:full'),
    'foam': os.environ.get('FOAM_IMG', 'motion-planner-foam:116928f71aaa'),
    'cricket': os.environ.get('CRICKET_IMG', 'motion-planner-cricket:a2e664f97b6a'),
}
FOLDED = [0.0, 0.0, 3.1, 0.0, 0.0, 0.0]  # elbow folded back on itself: must collide
SOURCE_ENV = ('source /opt/ros/*/setup.bash >/dev/null; '
              'mkdir -p /tmp/pfx/share/ament_index/resource_index/packages; '
              'ln -sfn /repo/src/description/mobile_manipulator_description /tmp/pfx/share/mobile_manipulator_description; '
              'touch /tmp/pfx/share/ament_index/resource_index/packages/mobile_manipulator_description; '
              'export AMENT_PREFIX_PATH=/tmp/pfx:$AMENT_PREFIX_PATH; ')


# ----------------------------------------------------------------------------- helpers
def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def load_module(path, name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def profile_args(profile):
    return load_module(DESC / 'scripts' / 'profile_xacro_args.py', 'profile_xacro_args').xacro_args(profile)


def robot_name(profile_path):
    """C++ identifier for the Cricket header, from the profile file name."""
    return ''.join(p.capitalize() for p in re.split(r'[^A-Za-z0-9]+', pathlib.Path(profile_path).stem) if p)


def arm_only_args(profile):
    """Xacro args for the arm-only model: the lift pinned (6 DOF).

    A moving lift is pinned at its home height. A fixed lift (or a pinned `lift_fixed_height` in the
    profile's xacro_args) keeps the height the full robot.urdf renders at, 0.4 m by default."""
    args = profile_args(profile)
    kind = (profile.get('lift') or {}).get('type', 'telescopic')
    if kind != 'none':
        args['lift'] = 'fixed' if kind in ('telescopic', 'fixed') else kind
        if kind == 'fixed':
            args.setdefault('lift_fixed_height', '0.4')
        elif 'lift_fixed_height' not in args:
            args['lift_fixed_height'] = str((profile.get('home') or {}).get('lift_joint', 0.0))
    return args


def run(cmd, **kw):
    print('+', ' '.join(shlex.quote(str(c)) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kw)


def in_image(image, out, inner_args, gpu=False):
    cmd = ['docker', 'run', '--rm', '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp',
           '-e', 'PYTHONDONTWRITEBYTECODE=1', '-v', f'{REPO}:/repo:ro', '-v', f'{out}:{out}', '-w', str(out)]
    if gpu:
        cmd += ['--gpus', 'all']
    cmd += [image, 'bash', '-c', SOURCE_ENV + 'exec "$@"', '--', 'python3', '/repo/docker/tools/robot_assets.py'] + inner_args
    run(cmd)


# ----------------------------------------------------------------------------- inner stages
def inner_urdf(profile, out):
    import xacro
    pkg = DESC / 'urdf' / 'mobile_manipulator.urdf.xacro'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'arm_only').mkdir(exist_ok=True)
    for path, args in ((out / 'robot.urdf', profile_args(profile)),
                       (out / 'arm_only' / 'robot.urdf', arm_only_args(profile))):
        path.write_text(xacro.process_file(str(pkg), mappings=args).toxml())
        print('wrote', path)
    resolver = load_module(TOOLS / 'resolve_package_uris.py', 'resolve_package_uris')
    n = resolver.resolve(str(out / 'arm_only' / 'robot.urdf'), str(out / 'arm_only' / 'resolved'))
    print(f'resolved {n} package:// meshes into arm_only/resolved')


def inner_srdf(profile, out):
    import xacro
    import yaml
    args = profile_args(profile)
    mappings = {'arm': args['arm'], 'lift': args.get('lift', 'telescopic')}
    if args.get('lift_fixed_height'):
        mappings['lift_fixed_height'] = args['lift_fixed_height']
    if args.get('lift_stages'):
        mappings['lift_stages'] = args['lift_stages']
    srdf = xacro.process_file(str(DESC / 'srdf' / 'mobile_manipulator.srdf.xacro'), mappings=mappings).toxml()
    (out / 'robot.srdf').write_text(srdf)
    lift, joints = profile.get('lift') or {}, profile['joints']
    limits = {}
    if joints.get('lift') and lift.get('type') in ('telescopic', 'scissor'):
        limits[joints['lift']] = {
            'has_velocity_limits': True, 'max_velocity': float(lift['max_velocity_mps']),
            'has_acceleration_limits': 'max_acceleration_mps2' in lift,
            'max_acceleration': float(lift.get('max_acceleration_mps2', 0.0))}
    (out / 'lift_limits.yaml').write_text(
        '# Generated from the profile lift: section; merge into the MoveIt joint_limits.\n'
        + yaml.safe_dump({'joint_limits': limits}, sort_keys=False))
    print('wrote robot.srdf, lift_limits.yaml')


def inner_reach(profile, out):
    (out / 'reach').mkdir(exist_ok=True)
    chain = out / 'reach' / 'chain.yaml'
    run(['python3', REPO / 'src' / 'manipulation' / 'reachability' / 'scripts' / 'urdf_to_chain.py',
         out / 'robot.urdf', '--base', profile['frames']['arm_base'], '--tip', profile['frames']['tcp'], '-o', chain])
    name = robot_name(profile['_path'])
    tool = next((p for p in (pathlib.Path('/ws/install/reachability/lib/reachability/rm4d_tool'),)
                 if p.exists()), None)
    # lift-independent 6-DOF map; the lift shifts pz, `rm4d_tool bands` turns it into lift bands
    cmd = ['rm4d_tool', 'build', 'chain.yaml', f'{name}.rm4d', '--samples', '3000000', '--resolution', '0.05',
           '--extent', '2.0', '--zmin', '-1.8', '--zmax', '2.0']
    (out / 'reach' / 'build_map.sh').write_text('#!/usr/bin/env bash\n# map build used for the manifest\ncd "$(dirname "$0")"\n'
                                                + ' '.join(cmd) + '\n')
    if tool is None:
        print('rm4d_tool not found in this image: chain.yaml written, map not built')
        return
    cmd[0] = str(tool)
    run(cmd, cwd=out / 'reach')


def inner_check(profile, out):
    import numpy as np
    model_mod = load_module(TOOLS / 'sphere_self_collision.py', 'sphere_self_collision')
    spherized, srdf = out / 'vamp' / 'spherized.urdf', out / 'vamp' / 'robot.srdf'
    if not spherized.exists():
        raise SystemExit('check needs the spheres stage (vamp/spherized.urdf)')
    if not srdf.exists():
        raise SystemExit('check needs the vamp stage (vamp/robot.srdf, the ignore list for these spheres)')
    model = model_mod.Model(str(spherized), str(srdf))
    arm = profile['joints']['arm']
    if sorted(model.dof) != sorted(arm):
        raise SystemExit(f'revolute joints {model.dof} do not match the profile arm {arm}')
    home = [float(profile['home'][j]) for j in model.dof]
    folded = FOLDED if len(model.dof) == 6 else [0.0] * len(model.dof)
    rng = np.random.default_rng(0)
    free = sum(not model.colliding_pairs(rng.uniform(-np.pi, np.pi, len(model.dof))) for _ in range(500))
    home_pairs, folded_pairs = model.colliding_pairs(home), model.colliding_pairs(folded)
    result = {'joint_order': model.dof, 'home': home, 'home_pairs': [list(p) for p in home_pairs],
              'home_free': not home_pairs, 'folded': folded, 'folded_collides': bool(folded_pairs),
              'folded_pairs': [list(p) for p in folded_pairs][:8], 'random_free_of_500': free,
              'ok': (not home_pairs) and bool(folded_pairs)}
    (out / 'check.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k not in ('home_pairs', 'folded_pairs')}))
    if not result['ok']:
        raise SystemExit('self-collision check failed: home must be free and the folded pose must collide (see check.json)')


# ----------------------------------------------------------------------------- host stages
def host_spheres(profile, out, env):
    # new spheres invalidate the ignore list of an earlier run; the vamp stage writes it again
    (out / 'vamp' / 'robot.srdf').unlink(missing_ok=True)
    run(['bash', TOOLS / 'generate_vamp_robot.sh', robot_name(profile['_path']),
         out / 'arm_only' / 'resolved' / 'robot.urdf', '-', profile['frames']['tcp'], out / 'vamp'],
        env={**env, 'FOAM_ONLY': '1'})


def host_vamp(profile, out, env, hops):
    vamp = out / 'vamp'
    rigid = subprocess.run([sys.executable, str(TOOLS / 'rigid_pairs_srdf.py'), '--hops', str(hops),
                            str(vamp / 'spherized.urdf')], check=True, capture_output=True, text=True).stdout
    (vamp / 'rigid.srdf').write_text(rigid)
    run(['bash', TOOLS / 'generate_vamp_robot.sh', robot_name(profile['_path']),
         out / 'arm_only' / 'resolved' / 'robot.urdf', vamp / 'rigid.srdf', profile['frames']['tcp'], vamp],
        env={**env, 'SKIP_FOAM': '1'})


def host_curobo(profile, out, env, batch_size):
    cdir = out / 'curobo'
    cdir.mkdir(exist_ok=True)
    run([sys.executable, TOOLS / 'primitives_to_meshes.py', out / 'arm_only' / 'resolved' / 'robot.urdf', cdir / 'meshed'])
    # copy the package meshes the resolver wrote next to the URDF, as the VAMP script does
    src = out / 'arm_only' / 'resolved'
    for d in src.iterdir():
        if d.is_dir():
            run(['cp', '-r', d, cdir / 'meshed'])
    name = robot_name(profile['_path'])
    cmd = ['docker', 'run', '--rm', '--gpus', 'all', '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp',
           '-v', f'{REPO}:/repo:ro', '-v', f'{out}:{out}', '-w', str(out), IMAGES['gpu'],
           '/opt/venv/bin/python', '/repo/docker/tools/generate_curobo_robot.py', '--urdf', cdir / 'meshed' / 'robot.urdf',
           '--tool-frame', profile['frames']['tcp'], '--batch-size', str(batch_size), '--out', cdir / f'{name}.yml']
    run(cmd)


def image_info(key):
    r = subprocess.run(['docker', 'image', 'inspect', IMAGES[key], '--format', '{{.Id}}'], capture_output=True, text=True)
    return {'image': IMAGES[key], 'id': r.stdout.strip() or None}


def host_manifest(profile, out, status):
    import xml.etree.ElementTree as ET
    import yaml
    files = {}
    for p in sorted(out.rglob('*')):
        rel = p.relative_to(out)
        if p.is_file() and 'meshed' not in rel.parts and 'resolved' not in rel.parts \
                and 'templates' not in rel.parts and rel.name not in ('manifest.json', 'status.json') \
                and rel.suffix not in ('.stl', '.dae') and rel.name != 'sphere_database.json':
            files[str(rel)] = {'sha256': sha256(p), 'bytes': p.stat().st_size}
    counts = {}
    sph = out / 'vamp' / 'spherized.urdf'
    if sph.exists():
        per = {}
        for link in ET.parse(sph).getroot().iter('link'):
            n = len(link.findall('collision/geometry/sphere'))
            if n:
                per[link.get('name')] = n
        counts['foam'] = {'spheres': sum(per.values()), 'links': len(per), 'per_link': per}
    for yml in (out / 'curobo').glob('*.yml') if (out / 'curobo').exists() else []:
        data = yaml.safe_load(yml.read_text())

        def find(node):
            if isinstance(node, dict):
                if 'collision_spheres' in node:
                    return node['collision_spheres']
                for v in node.values():
                    r = find(v)
                    if r is not None:
                        return r
            return None
        spheres = find(data)
        if isinstance(spheres, dict):
            per = {k: len(v) for k, v in spheres.items() if isinstance(v, list)}
            counts['curobo'] = {'spheres': sum(per.values()), 'links': len(per), 'per_link': per}
    headers = sorted((out / 'vamp').glob('*.hh')) if (out / 'vamp').exists() else []
    cricket_src = pathlib.Path(os.environ.get('CRICKET_SRC', pathlib.Path.home() / '.cache/motion-planner-tools/cricket'))
    tpl = cricket_src / 'resources' / 'templates' / 'fk_template.hh'
    refs = dict(re.findall(r'^(FOAM_REF|CRICKET_REF)=(\w+)', (TOOLS / 'build_robot_tools.sh').read_text(), re.M))
    check = json.loads((out / 'check.json').read_text()) if (out / 'check.json').exists() else None
    git = subprocess.run(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    manifest = {
        'generated_utc': datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'profile': {'name': robot_name(profile['_path']), 'path': str(profile['_path']), 'sha256': sha256(profile['_path']),
                    'lift': (profile.get('lift') or {}).get('type'), 'xacro_args': profile_args(profile)},
        'versions': {'repo_commit': git, 'foam_ref': refs.get('FOAM_REF'), 'cricket_ref': refs.get('CRICKET_REF'),
                     'cricket_template_sha256': sha256(tpl) if tpl.exists() else None,
                     'images': {k: image_info(k) for k in ('build', 'foam', 'cricket', 'gpu')}},
        'sphere_counts': counts,
        'vamp_headers': [h.name for h in headers],
        'self_collision': check,
        'stages': status,
        'files': files,
    }
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('wrote', out / 'manifest.json')
    return manifest


# ----------------------------------------------------------------------------- main
def select(only, skip):
    for name in list(only or []) + list(skip or []):
        if name not in STAGES:
            raise SystemExit(f'unknown stage {name!r}; stages: {", ".join(STAGES)}')
    return [s for s in STAGES if (not only or s in only) and s not in (skip or [])]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('profile', nargs='?')
    ap.add_argument('--out', help='output directory (default build/robot_assets/<profile name>)')
    ap.add_argument('--only', type=lambda s: s.split(','), help='comma-separated stages to run')
    ap.add_argument('--skip', type=lambda s: s.split(','), help='comma-separated stages to skip')
    ap.add_argument('--hops', type=int, default=2, help='rigid_pairs_srdf.py --hops for the VAMP SRDF')
    ap.add_argument('--curobo-batch-size', type=int, default=1000)
    ap.add_argument('--inner', choices=['urdf', 'srdf', 'reach', 'check'], help=argparse.SUPPRESS)
    ap.add_argument('--list', action='store_true', help='list the stages and exit')
    args = ap.parse_args()
    if args.list:
        print('\n'.join(STAGES))
        return 0
    if not args.profile:
        ap.error('profile.yaml is required')
    import yaml
    path = pathlib.Path(args.profile).resolve()
    profile = yaml.safe_load(path.read_text())
    profile['_path'] = path
    out = pathlib.Path(args.out or REPO / 'build' / 'robot_assets' / robot_name(path)).resolve()

    if args.inner:
        {'urdf': inner_urdf, 'srdf': inner_srdf, 'reach': inner_reach, 'check': inner_check}[args.inner](profile, out)
        return 0

    stages = select(args.only, args.skip)
    out.mkdir(parents=True, exist_ok=True)
    inner_profile = out / path.name               # inside the images only OUT and /repo are mounted
    inner_profile.write_text(path.read_text())
    status_file = out / 'status.json'
    status = json.loads(status_file.read_text()) if status_file.exists() else {}
    env = dict(os.environ)
    inner = lambda stage: [str(inner_profile), '--inner', stage, '--out', str(out)]
    failed = False
    for stage in stages:
        t0 = time.time()
        print(f'\n=== {stage} ===', flush=True)
        try:
            if stage in ('urdf', 'srdf', 'reach', 'check'):
                in_image(IMAGES['build'], out, inner(stage))
            elif stage == 'spheres':
                host_spheres(profile, out, env)
            elif stage == 'vamp':
                host_vamp(profile, out, env, args.hops)
            elif stage == 'curobo':
                host_curobo(profile, out, env, args.curobo_batch_size)
            elif stage == 'manifest':
                status_file.write_text(json.dumps(status, indent=2))
                host_manifest(profile, out, status)
            status[stage] = {'status': 'ok', 'seconds': round(time.time() - t0, 1)}
        except (subprocess.CalledProcessError, SystemExit) as e:
            status[stage] = {'status': 'failed', 'error': str(e)[:200], 'seconds': round(time.time() - t0, 1)}
            print(f'stage {stage} failed: {e}', file=sys.stderr)
            failed = True
            break
        except Exception as e:  # noqa: BLE001  e.g. docker missing: record which stage died, then show the traceback
            status[stage] = {'status': 'failed', 'error': f'{type(e).__name__}: {e}'[:200],
                             'seconds': round(time.time() - t0, 1)}
            status_file.write_text(json.dumps(status, indent=2))
            raise
        status_file.write_text(json.dumps(status, indent=2))
    if 'manifest' not in stages or failed:
        status_file.write_text(json.dumps(status, indent=2))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
