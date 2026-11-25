#!/usr/bin/env python3
"""audit_repo: run the repository's own scene generators on the host and check_sync what they produce against the
example scenarios. This is how docs/scenario_sync.md's findings were made; rerun it after changing a generator.

    python3 -m scenario_sync.audit_repo [--repo ROOT] [--work DIR] [--format md|text]

No ROS, no simulator: only shift_wall_scene.py, make_scenario_scene.py and the two wall_world.py modules (numpy), and
the RViz profiles of branch feat/rviz-profiles when that branch exists (git show). The coverage plan itself is not
run (it needs the C++ planner), so the plan is represented by the wall_scene.json it is given.
"""
import argparse
import contextlib
import json
import os
import re
import subprocess
import sys
import tempfile

from . import sources
from .check_sync import FAIL, WARN, Check, check_plan_scene, check_sdf, check_mjcf, check_sim_scene, frame_check, render
from .scenario_gen import find_repo, gazebo_sdf, load_module, mujoco_world
from .schema import load_scenario


@contextlib.contextmanager
def _path(*dirs):
    sys.path[:0] = list(dirs)
    try:
        yield
    finally:
        for d in dirs:
            sys.path.remove(d)


def _table(title, checks, notes=()):
    return {'title': title, 'checks': checks, 'notes': list(notes)}


def rviz_profile_frames(repo):
    """{profile: fixed frame} of mp_rviz profiles on branch feat/rviz-profiles (empty if the branch is missing)."""
    try:
        names = subprocess.run(['git', '-C', repo, 'ls-tree', '--name-only', 'feat/rviz-profiles',
                                'src/common/mp_rviz/profiles/'], capture_output=True, text=True, check=True).stdout.split()
    except (OSError, subprocess.CalledProcessError):
        return {}
    out = {}
    for n in names:
        txt = subprocess.run(['git', '-C', repo, 'show', f'feat/rviz-profiles:{n}'], capture_output=True, text=True).stdout
        m = re.search(r'^\s+Fixed Frame: (\S+)', txt, re.M)
        if m:
            out[os.path.basename(n)[:-5]] = m.group(1)
    return out


def audit(repo, work):
    ex = os.path.join(repo, 'src', 'common', 'scenario_sync', 'examples')
    full = load_scenario(os.path.join(ex, 'gazebo_smoke_full.yaml'))
    crop = load_scenario(os.path.join(ex, 'gazebo_patch_crop.yaml'))
    tall = load_scenario(os.path.join(ex, 'mujoco_tall_wall.yaml'))
    gz = os.path.join(repo, 'src', 'simulation', 'sim_gazebo')
    mj = os.path.join(repo, 'src', 'simulation', 'sim_mujoco')
    gz_data, mj_data = os.path.join(gz, 'test', 'data'), os.path.join(mj, 'test', 'data')
    tables = []
    jl = sources.read_json

    # 1. the Gazebo smoke scene as shipped
    tables.append(_table('Gazebo smoke_wall_scene.json as shipped vs gazebo_smoke_full (the whole wall)',
                         check_sim_scene(full, jl(os.path.join(gz_data, 'smoke_wall_scene.json')), 'gazebo smoke_wall_scene.json'),
                         ['The file is what `sim.launch.py wall_scene:=` takes; the launch moves the wall to `standoff` anyway, '
                          'so its x = 2.0 only matters to a consumer that reads the file as it is.']))

    # 2. the Gazebo multi-stand pass: what its runner builds
    shift_gz = load_module(os.path.join(gz, 'scripts', 'shift_wall_scene.py'), '_audit_gz_shift')
    d = os.path.join(work, 'gz_multi_stand')
    os.makedirs(d, exist_ok=True)
    shift_gz.shift_scene(os.path.join(gz_data, 'wall_pass_coverage_scene.json'), os.path.join(d, 'plan.json'), 0.0, 0.3, 0.9)
    shift_gz.shift_scene(os.path.join(gz_data, 'smoke_wall_scene.json'), os.path.join(d, 'sim.json'), 0.0, 0.3, 0.9)
    sim = jl(os.path.join(d, 'sim.json'))
    sim['obstacles'] = []   # run_multi_stand_pass.sh drops the obstacle
    json.dump(sim, open(os.path.join(d, 'sim.json'), 'w'))
    sdf = os.path.join(d, 'wall.sdf')
    gazebo_sdf_aligned(gz, os.path.join(d, 'sim.json'), sdf, 0.915)
    checks = check_plan_scene(crop, jl(os.path.join(d, 'plan.json')), 'gazebo plan scene (shift 0 0.3 0.9)')
    checks += check_sdf(crop, sdf, 'gazebo wall.sdf (launch, standoff 0.915)')
    built = sources.wall_obs_from_sdf(crop, sdf)
    plan_obs = sources.wall_obs_from_scene(crop, jl(os.path.join(d, 'plan.json')), 'plan')
    tables.append(_table('Gazebo multi-stand pass as its runner builds it vs gazebo_patch_crop', checks, [
        f'As built, the Gazebo wall is {built.u[1] - built.u[0]:.2f} x {built.v[1] - built.v[0]:.2f} m '
        f'({built.area:.2f} m^2); the planned patch is {plan_obs.u[1] - plan_obs.u[0]:.2f} x '
        f'{plan_obs.v[1] - plan_obs.v[0]:.2f} m = {100 * plan_obs.area / built.area:.1f} % of it.']))

    # 3. the MuJoCo stock smoke scene
    tables.append(_table('MuJoCo stock smoke_wall_scene.json vs gazebo_smoke_full (the same wall in the other simulator)',
                         check_sim_scene(full, jl(os.path.join(mj_data, 'smoke_wall_scene.json')), 'mujoco smoke_wall_scene.json')))

    # 4. the MuJoCo tall wall (PR #190)
    with _path(os.path.join(mj, 'scripts'), mj):
        msc = load_module(os.path.join(mj, 'scripts', 'make_scenario_scene.py'), '_audit_make_scenario')
        td = os.path.join(work, 'mj_tall')
        info = msc.build(mj_data, td, 'two_panels_tall')
    mjcf = os.path.join(td, 'wall_world.xml')
    mujoco_world(repo, os.path.join(td, 'smoke_wall_scene.json'), mjcf, (0.0, 0.0, 0.0, 0.0))
    checks = check_plan_scene(tall, jl(os.path.join(td, 'wall_pass_coverage_scene.json')), 'mujoco plan scene (two_panels_tall)')
    checks += check_sim_scene(tall, jl(os.path.join(td, 'smoke_wall_scene.json')), 'mujoco sim scene (two_panels_tall)')
    checks += check_mjcf(tall, mjcf, 'mujoco MJCF (two_panels_tall)')
    tables.append(_table('MuJoCo two_panels_tall (make_scenario_scene.py) vs mujoco_tall_wall', checks,
                         [f'make_scenario_scene.py reports {json.dumps(info)}']))

    # 5. the same MuJoCo wall seen by the Gazebo scenario
    tables.append(_table('MuJoCo two_panels_tall sim scene vs gazebo_patch_crop (does one scenario fit both simulators today?)',
                         check_sim_scene(crop, jl(os.path.join(td, 'smoke_wall_scene.json')), 'mujoco sim scene (two_panels_tall)')))

    # 6. frames of the views
    frames = [('wall_coverage_demo.launch.py default Fixed Frame', 'link_base'),
              ('wall_scene.json written by the MuJoCo/Gazebo scenario builders', 'base_link'),
              ('Gazebo smoke_wall_scene.json', jl(os.path.join(gz_data, 'smoke_wall_scene.json')).get('frame_id')),
              ('recording frame, multi-stand pass (docs/how_to_read_a_run.md)', 'odom'),
              ('recording frame, other passes (docs/how_to_read_a_run.md)', 'base_footprint')]
    groups = {}
    for k, v in sorted(rviz_profile_frames(repo).items()):
        groups.setdefault(v, []).append(k)
    for fr, names in sorted(groups.items()):
        frames.append((f'mp_rviz profiles ({len(names)}): {", ".join(names)}', fr))
    checks = [frame_check(crop, 'frames', name, fr) for name, fr in frames]
    tables.append(_table('Frame names of the views vs the world frame odom', checks))

    # 7. base-to-wall distance by entry point
    tables.append(_table('Base-to-wall distance by entry point', [
        Check('sim_gazebo/sim.launch.py', 'default standoff', WARN, '0.900 m (also wall_world_gen.py --standoff)'),
        Check('sim_gazebo run_multi_stand_pass / run_manager_pass / run_spray_pass / run_depth_wall', 'standoff',
              WARN, '0.915 m to the sim wall; the planned plane is 0.980 m (tool tip 6.5 cm short, documented)'),
        Check('sim_mujoco run_*pass', 'plane', WARN, '0.980 m (+ MULTI_STAND_WALL_DX)'),
        Check('sim_gazebo smoke_wall_scene.json', 'plane', FAIL, 'x = 2.000 m in frame map (not used as is by any pass)')]))
    return tables


def gazebo_sdf_aligned(gz, scene, out, standoff):
    """The Gazebo launch's own call: generate_world(scene, wall_id 0, standoff), aligned and centred on the robot."""
    wg = load_module(os.path.join(gz, 'sim_gazebo', 'wall_world.py'), '_audit_gz_wall_world')
    return wg.generate_world(scene, out, wall_id=0, standoff=standoff)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo')
    ap.add_argument('--work', help='keep the generated files here (default: a temporary directory)')
    ap.add_argument('--format', choices=('md', 'text'), default='md')
    a = ap.parse_args(argv)
    repo = a.repo or find_repo()
    if not repo:
        print('audit_repo: no source tree found (use --repo)', file=sys.stderr)
        return 2
    work = a.work or tempfile.mkdtemp(prefix='scenario_sync_audit_')
    os.makedirs(work, exist_ok=True)
    for t in audit(repo, work):
        print(f'\n### {t["title"]}\n' if a.format == 'md' else f'\n== {t["title"]}')
        print(render(t['checks'], a.format))
        for n in t['notes']:
            print(f'\n{n}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
