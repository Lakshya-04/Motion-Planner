#!/usr/bin/env python3
"""Headless Isaac Sim runner: robot USD, ground plane, ROS 2 bridge graph. Run with the container's ./python.sh.

    ./python.sh run_isaac.py --usd robot.usd --profile ur10e_telescopic --profile-dir /sim_isaac/profiles --seconds 60

Publishes /clock, /joint_states (and the transform tree), takes JointState position commands on
/isaac/joint_command. Joint list and order come from the profile (isaac_common.joint_order).
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import isaac_common as ic  # noqa: E402

ROBOT_PRIM = '/World/robot'


def log(msg):
    print(f'[run_isaac] {msg}', flush=True)


def find_articulation_root(stage, under):
    from pxr import Usd, UsdPhysics
    for prim in Usd.PrimRange(stage.GetPrimAtPath(under)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError(f'no articulation root under {under}')


def build_graph(spec, art_root, robot_prim, og, set_targets, stage):
    keys = og.Controller.Keys
    og.Controller.edit({'graph_path': '/ROS2Graph', 'evaluator_name': 'execution'}, {
        keys.CREATE_NODES: spec['nodes'], keys.CONNECT: spec['connect'], keys.SET_VALUES: spec['values']})
    for attr, prim in spec['targets']:
        node, name = attr.split('.')
        prim_path = art_root if prim == ROBOT_PRIM else prim
        set_targets(prim=stage.GetPrimAtPath(f'/ROS2Graph/{node}'), attribute=name, target_prim_paths=[prim_path])


def main():
    args = ic.parse_args()
    profile = ic.load_profile(ic.find_profile(args.profile, args.profile_dir))
    joints = ic.joint_order(profile)
    home = ic.home_positions(profile, joints)

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.ros2.bridge')
    app.update()
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.prims import set_targets
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction

    world = World(physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / args.physics_hz, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=os.path.abspath(args.usd), prim_path=ROBOT_PRIM)
    stage = omni.usd.get_context().get_stage()
    art_root = find_articulation_root(stage, ROBOT_PRIM)
    log(f'articulation root {art_root}')

    spec = ic.graph_spec(args.command_topic, args.state_topic, ROBOT_PRIM, tf=not args.no_tf)
    build_graph(spec, art_root, ROBOT_PRIM, og, set_targets, stage)
    world.reset()

    art = SingleArticulation(prim_path=art_root)
    art.initialize()
    dofs = list(art.dof_names)
    log(f'dof order in Isaac: {dofs}')
    missing = [j for j in joints if j not in dofs]
    if missing:
        raise RuntimeError(f'profile joints not in the articulation: {missing}')
    idx = [dofs.index(j) for j in joints]
    art.set_joint_positions(home, joint_indices=idx)
    art.apply_action(ArticulationAction(joint_positions=home, joint_indices=idx))
    log(f'start pose set for {len(joints)} joints, physics {args.physics_hz:g} Hz')

    t_wall = time.monotonic()
    step = 0
    period = 1.0 / args.physics_hz
    while app.is_running():
        world.step(render=True)
        step += 1
        if args.steps is not None and step >= args.steps:
            break
        if args.seconds is not None and time.monotonic() - t_wall >= args.seconds:
            break
        if not args.no_realtime:
            ahead = step * period - (time.monotonic() - t_wall)
            if ahead > 0:
                time.sleep(ahead)
    q = art.get_joint_positions()
    log(f'done after {step} steps, final positions ' +
        ', '.join(f'{d}={q[i]:.4f}' for i, d in enumerate(dofs)))
    app.close()


if __name__ == '__main__':
    try:
        main()
    except SystemExit:
        raise
    except BaseException:   # Kit swallows tracebacks on shutdown, print it and exit non-zero ourselves
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
