#!/usr/bin/env python3
"""URDF -> cuRobo v2 robot config (collision spheres + collision matrix), via cuRobo's RobotBuilder.

Run inside the gpu image (needs CUDA):

  docker run --rm --gpus all -v "$PWD/robots:/robots" motion-planner:gpu \
    /opt/venv/bin/python docker/tools/generate_curobo_robot.py \
      --urdf /robots/meshed/robot.urdf --tool-frame tcp_link --out /robots/mm_arm.yml

The URDF's collision geometry must be meshes (docker/tools/primitives_to_meshes.py converts
primitives, as for the VAMP pipeline). cuRobo stores absolute paths in the config, so the URDF and
its meshes must live where the planner will read them: pass the final location (by default the
directory the URDF is in) and mount the same path when planning.
"""
import argparse
import pathlib
import sys
import xml.etree.ElementTree as ET



def urdf_spheres(path):
    """link -> [{center, radius}] from the collision <sphere> elements of a spherized URDF (FOAM output,
    or any URDF written with spheres). Only the origin's xyz is used: a sphere has no orientation."""
    out = {}
    for link in ET.parse(path).getroot().iter('link'):
        for col in link.findall('collision'):
            sph = col.find('geometry/sphere')
            if sph is None:
                continue
            origin = col.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0.0] * 3
            out.setdefault(link.get('name'), []).append({'center': xyz, 'radius': float(sph.get('radius'))})
    return out


def ignore_within_hops(urdf, hops):
    """link -> links up to `hops` joints away (rigid groups count as one body), as cuRobo's
    self_collision_ignore. Same rule as rigid_pairs_srdf.py --hops."""
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from rigid_pairs_srdf import pairs
    out = {}
    for a, b in pairs(str(urdf), (), hops):
        out.setdefault(a, []).append(b)
        out.setdefault(b, []).append(a)
    return out


def merge_ignore(base, pairs=(), groups=()):
    """Self-collision ignore table (link -> sorted links) = `base` plus every `A:B` pair of `pairs` and every
    pair inside each `A,B,C` group of `groups`. Symmetric, no self-pairs, deterministic order. A malformed
    pair or group is an error (a typo must not silently ignore nothing). Pure: no cuRobo needed."""
    table = {k: set(v) for k, v in (base or {}).items()}

    def add(a, b):
        if a == b:
            return
        table.setdefault(a, set()).add(b)
        table.setdefault(b, set()).add(a)

    for pair in pairs:
        names = [n.strip() for n in pair.split(':')]
        if len(names) != 2 or not all(names):
            raise ValueError(f'ignore pair {pair!r} is not LINK_A:LINK_B')
        add(*names)
    for group in groups:
        names = [n.strip() for n in group.split(',') if n.strip()]
        if len(names) < 2:
            raise ValueError(f'ignore group {group!r} needs at least two comma separated links')
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                add(a, b)
    return {k: sorted(v) for k, v in sorted(table.items())}


def unknown_links(table, links):
    """Names in an ignore table that are not in `links` (sorted), so a typo is reported before the GPU run."""
    known = set(links)
    return sorted({n for k, v in table.items() for n in (k, *v)} - known)


def main():
    from curobo.robot_builder import RobotBuilder
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--asset-root', help='directory mesh paths are relative to (default: the URDF directory)')
    ap.add_argument('--tool-frame', action='append', required=True, help='repeatable')
    ap.add_argument('--out', required=True)
    ap.add_argument('--sphere-density', type=float, default=1.0)
    ap.add_argument('--batch-size', type=int, default=1000,
                    help='collision-matrix sampling batch; cuRobo defaults to 10000, which needs >7 GB of GPU memory')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--spheres-urdf', help='use the collision spheres of this spherized URDF (e.g. FOAM output) '
                    'instead of fitting; the link names must match --urdf')
    ap.add_argument('--ignore-hops', type=int, default=0,
                    help='also ignore self-collision between links up to N joints apart (use 2 with FOAM '
                         'spheres on a serial arm: their adjacent-link overlap is large); needs --spheres-urdf '
                         'or applies on top of the computed matrix')
    ap.add_argument('--ignore-pair', action='append', default=[], metavar='A:B',
                    help='also ignore self-collision between these two links (repeatable). For a pair the sampled '
                         'matrix keeps although the real parts never touch, e.g. a lift stage and the arm base at a '
                         'low lift; a wrongly ignored pair hides a real self-collision, so list only rigidly mounted '
                         'or physically separated pairs')
    ap.add_argument('--ignore-group', action='append', default=[], metavar='A,B,C',
                    help='also ignore self-collision between every two links of this comma separated list '
                         '(repeatable), e.g. all links of the lift column')
    ap.add_argument('--sphere-buffer', type=float, default=0.0,
                    help='metres added to every collision sphere radius by cuRobo (collision_sphere_buffer)')
    args = ap.parse_args()

    import numpy as np
    import torch
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    urdf = pathlib.Path(args.urdf).resolve()
    root = pathlib.Path(args.asset_root).resolve() if args.asset_root else urdf.parent
    builder = RobotBuilder(urdf_path=str(urdf), asset_path=str(root), tool_frames=args.tool_frame)
    # cuRobo fits to the *visual* meshes by default; our pipelines carry collision meshes only (the
    # resolver drops visuals), so fit to those.
    if args.spheres_urdf:
        table = urdf_spheres(args.spheres_urdf)
        # RobotBuilder has no public setter for an existing sphere table; fit_collision_spheres is the
        # only writer of this attribute, and everything downstream (matrix, build, save) reads it.
        builder._collision_spheres = table
    else:
        builder.fit_collision_spheres(sphere_density=args.sphere_density, use_collision_mesh=True)
    if builder.num_spheres == 0:
        raise SystemExit('no collision spheres: the URDF has no collision meshes cuRobo could load '
                         '(or --spheres-urdf has no <sphere> collisions)')
    custom = ignore_within_hops(urdf, args.ignore_hops) if args.ignore_hops else None
    if args.ignore_pair or args.ignore_group:
        # Only the names the user typed are checked: the hop table names links without spheres (frames), which is fine.
        bad = unknown_links(merge_ignore(None, args.ignore_pair, args.ignore_group), builder.collision_link_names)
        custom = merge_ignore(custom, args.ignore_pair, args.ignore_group)
        if bad:
            raise SystemExit(f'--ignore-pair / --ignore-group name links without collision spheres: {bad}')
    builder.compute_collision_matrix(batch_size=args.batch_size, custom_ignore=custom)
    builder.save(builder.build(), args.out)
    if args.sphere_buffer:
        import yaml
        doc = yaml.safe_load(pathlib.Path(args.out).read_text())
        doc['kinematics']['collision_sphere_buffer'] = args.sphere_buffer
        pathlib.Path(args.out).write_text(yaml.safe_dump(doc, sort_keys=False))
    print(f'wrote {args.out}: {builder.num_spheres} spheres on {len(builder.collision_link_names)} links')


if __name__ == '__main__':
    main()
