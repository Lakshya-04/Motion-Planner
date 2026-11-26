#!/usr/bin/env python3
"""URDF to USD with Isaac's URDF importer. Run with the Isaac container's python (./python.sh).

    ./python.sh urdf_to_usd.py --urdf robot.urdf --usd robot.usd [--profile ur10e_telescopic]

The URDF must have package:// URIs resolved already (docker/tools/resolve_package_uris.py). Position drives go
on the profile's joints; <mimic> joints get a PhysX mimic joint and no drive of their own.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import isaac_common as ic  # noqa: E402


def fix_mimic(prim, limits, Sdf, UsdPhysics):
    """The importer's mimic follows the lift, but it writes bogus limits and a soft constraint that sags.

    Sets the URDF limits back and stiffens the constraint (its spring is what lets the stages hang under gravity).
    """
    lower, upper = limits
    UsdPhysics.PrismaticJoint(prim).GetLowerLimitAttr().Set(lower)
    UsdPhysics.PrismaticJoint(prim).GetUpperLimitAttr().Set(upper)
    for s in prim.GetAppliedSchemas():
        if s.startswith('PhysxMimicJointAPI:'):
            inst = s.split(':', 1)[1]
            for attr, val in (('naturalFrequency', ic.MIMIC_FREQUENCY_HZ), ('dampingRatio', ic.MIMIC_DAMPING_RATIO)):
                prim.CreateAttribute(f'physxMimicJoint:{inst}:{attr}', Sdf.ValueTypeNames.Float).Set(val)


def main():
    sys.stdout.reconfigure(line_buffering=True)   # Kit exits without flushing
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--usd', required=True)
    ap.add_argument('--profile', default=ic.DEFAULT_PROFILE)
    ap.add_argument('--profile-dir', action='append', default=[])
    args = ap.parse_args()

    profile = ic.load_profile(ic.find_profile(args.profile, args.profile_dir))
    joints = ic.joint_order(profile)
    mimics = ic.urdf_mimics(args.urdf)
    limits = ic.urdf_prismatic_limits(args.urdf)
    ic.check_mimics(profile, mimics)

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.asset.importer.urdf')
    app.update()
    import math

    import omni.kit.commands
    import omni.usd
    from isaacsim.asset.importer.urdf import _urdf
    from pxr import Sdf, Usd, UsdPhysics

    ok, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
    cfg.fix_base = True
    cfg.merge_fixed_joints = False      # keep tcp_link and the other frames as prims
    cfg.self_collision = False
    cfg.convex_decomp = False
    cfg.import_inertia_tensor = True
    cfg.parse_mimic = True
    cfg.make_default_prim = True
    cfg.create_physics_scene = False    # the runner owns the physics scene
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    cfg.default_drive_strength = ic.STIFFNESS
    cfg.default_position_drive_damping = ic.DAMPING
    ok, usd_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=os.path.abspath(args.urdf),
                                             import_config=cfg, dest_path=os.path.abspath(args.usd))
    print('IMPORT', ok, usd_path, flush=True)

    stage = Usd.Stage.Open(os.path.abspath(args.usd))
    report = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        name = prim.GetName()
        kind = 'linear' if prim.IsA(UsdPhysics.PrismaticJoint) else 'angular'
        drive = UsdPhysics.DriveAPI.Get(prim, kind)
        has_mimic = any('PhysxMimicJoint' in str(s) for s in prim.GetAppliedSchemas())
        if name in mimics:
            fix_mimic(prim, limits[name], Sdf, UsdPhysics)
            has_mimic = True
        elif name in joints and kind == 'linear' and drive:
            k, d = ic.drive_gains(name, profile)
            drive.GetStiffnessAttr().Set(k)
            drive.GetDampingAttr().Set(d)
        report[name] = (kind, bool(drive), has_mimic)
    stage.GetRootLayer().Save()
    for n, v in report.items():
        print('JOINT', n, v)
    missing = [j for j in joints if j not in report]
    if missing:
        print('MISSING', missing)
        app.close()
        sys.exit(1)
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
