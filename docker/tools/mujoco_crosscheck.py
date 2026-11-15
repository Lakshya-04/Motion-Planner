#!/usr/bin/env python3
"""MuJoCo / MJX cross-check of our arm models: forward kinematics and self-collision verdicts.

For N seeded random configurations inside the URDF joint limits:

  (a) tool-frame FK from MuJoCo vs a from-the-spec URDF FK, the analytic UR10e DH chain (ur10e only)
      and cuRobo's FK (from the .npz written by curobo_crosscheck_dump.py);
  (b) self-collision: MuJoCo mesh contacts (ground truth) vs the FOAM sphere set, the cuRobo sphere set
      (same spheres, numpy check) and cuRobo's own native verdict (from the .npz). MuJoCo, FOAM and
      cuRobo-numpy share one allowed-pair list (see `mujoco_model.allowed_pairs`, run for hops 1 and 2);
      cuRobo native uses its own generated `self_collision_ignore`;
  (c) the same FK and contact verdicts from a jax-vmapped `mjx.forward` batch, with timing.

    mujoco_crosscheck.py --robot ur10e --urdf robot.urdf --foam-urdf spherized.urdf \
        --curobo-cfg curobo.yml --curobo-dump curobo_dump.npz --n 2000 --seed 0 --json out.json

Run it with `JAX_PLATFORMS=cpu` so it never touches the GPU. Only --urdf is required; every other
input is optional and its rows are skipped when absent.
"""
import argparse
import json
import sys
import time

import numpy as np

import mujoco_model as mm


def rot_from_quat_wxyz(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def error_stats(pos_m, rot_rad):
    pos_mm = np.asarray(pos_m) * 1000
    rot_deg = np.degrees(rot_rad)
    return {'n': len(pos_mm), 'pos_mm_mean': float(pos_mm.mean()), 'pos_mm_max': float(pos_mm.max()),
            'rot_deg_mean': float(rot_deg.mean()), 'rot_deg_max': float(rot_deg.max())}


def check_dump(dump, joints, q):
    """The cuRobo dump must be for these joints and these exact configurations (row by row)."""
    if list(dump['joint_names']) != joints:
        sys.exit(f"cuRobo joint order {list(dump['joint_names'])} differs from the URDF's {joints}")
    if len(dump['position']) < len(q):
        sys.exit('cuRobo dump has fewer rows than --n')
    if 'q' not in dump:
        sys.exit('cuRobo dump has no `q`: rerun curobo_crosscheck_dump.py (it now saves the configurations)')
    if not np.allclose(np.asarray(dump['q'])[:len(q)], q, atol=1e-5):
        sys.exit('cuRobo dump was made from other configurations than this run samples (check --n and --seed)')


def run(args):
    import mujoco
    kin = mm.UrdfKinematics(args.urdf)
    joints = kin.dof
    model = mm.load_mujoco(args.urdf)
    data = mujoco.MjData(model)
    adr = mm.mujoco_qpos_addresses(model, joints)
    q = mm.sample_configs(kin, args.n, args.seed, joints)
    result = {'robot': args.robot, 'n': args.n, 'seed': args.seed, 'mujoco': mujoco.__version__,
              'joints': joints, 'ngeom': int(model.ngeom)}

    # joint limits survive the import
    ranges = mm.mujoco_joint_ranges(model, joints)
    result['joint_limit_max_abs_diff'] = max(abs(ranges[j][i] - kin.limits[j][i]) for j in joints for i in (0, 1))

    # ---------------- MuJoCo state per configuration
    mj_tool, mj_pairs = [], {h: [] for h in args.hops}
    allowed = {h: mm.allowed_pairs(kin, h) for h in args.hops}
    t0 = time.perf_counter()
    for qi in q:
        data.qpos[adr] = qi
        mujoco.mj_forward(model, data)
        mj_tool.append(mm.mujoco_pose(model, data, args.tool_frame))
        for h in args.hops:
            mj_pairs[h].append(mm.mujoco_colliding_pairs(model, data, allowed[h]))
    result['mujoco_cpu_s'] = time.perf_counter() - t0
    truth = {h: np.array([len(p) > 0 for p in mj_pairs[h]]) for h in args.hops}

    # ---------------- (a) FK
    fk = {}
    qd = [dict(zip(joints, qi)) for qi in q]
    urdf_tool = [kin.link_pose(args.tool_frame, d) for d in qd]
    fk['mujoco_vs_urdf_fk'] = error_stats(*zip(*[mm.pose_error(a, b) for a, b in zip(mj_tool, urdf_tool)]))
    if args.robot.startswith('ur10e'):
        q0 = {j: 0.0 for j in joints}
        flange_to_tool = np.linalg.inv(kin.link_pose('arm_flange_link', q0)) @ kin.link_pose(args.tool_frame, q0)
        base0 = kin.link_pose('arm_base_link', q0)
        an = [base0 @ mm.ur10e_flange_pose(qi) @ flange_to_tool for qi in q]
        fk['mujoco_vs_analytic_ur10e_dh'] = error_stats(*zip(*[mm.pose_error(a, b) for a, b in zip(mj_tool, an)]))
        fk['urdf_fk_vs_analytic_ur10e_dh'] = error_stats(*zip(*[mm.pose_error(a, b) for a, b in zip(urdf_tool, an)]))
    dump = None
    if args.curobo_dump:
        dump = np.load(args.curobo_dump)
        check_dump(dump, joints, q)
        cu = []
        for i in range(len(q)):
            t = np.eye(4)
            t[:3, :3] = rot_from_quat_wxyz(dump['quaternion'][i].astype(float))
            t[:3, 3] = dump['position'][i]
            cu.append(t)
        fk['mujoco_vs_curobo_fk_float32'] = error_stats(*zip(*[mm.pose_error(a, b) for a, b in zip(mj_tool, cu)]))
    result['fk'] = fk

    # ---------------- (b) collision verdicts
    sets = {}
    if args.foam_urdf:
        sets['foam_spheres'] = mm.load_foam_spheres(args.foam_urdf)
    if args.curobo_cfg:
        sets['curobo_spheres'] = mm.load_curobo_spheres(args.curobo_cfg)
    verdicts = {h: {} for h in args.hops}
    for name, spheres in sets.items():
        worlds = [mm.sphere_world(kin, spheres, d) for d in qd]
        for h in args.hops:
            verdicts[h][name] = np.array([len(mm.overlapping_pairs(w, allowed[h])) > 0 for w in worlds])
    if dump is not None:
        for h in args.hops:
            verdicts[h]['curobo_native'] = dump['self_collision'].astype(bool)[:len(q)]
    collision = {}
    for h in args.hops:
        rows = {name: mm.verdict_rates(truth[h], v) for name, v in verdicts[h].items()}
        collision[f'hops{h}'] = {'truth_collision_rate': float(truth[h].mean()), 'n': len(q), 'models': rows}
        if args.hops and h == args.hops[-1]:
            counts = {}
            for ps in mj_pairs[h]:
                for p in ps:
                    counts[' / '.join(p)] = counts.get(' / '.join(p), 0) + 1
            collision[f'hops{h}']['mujoco_pair_counts'] = dict(sorted(counts.items(), key=lambda kv: -kv[1])[:8])
    result['collision'] = collision

    # ---------------- (c) MJX batch
    if args.mjx_batch:
        h = args.hops[-1]
        result['mjx'] = run_mjx(args.urdf, kin, joints, q[:args.mjx_batch], args.tool_frame, h, args.mjx_vertices,
                                truth[h][:args.mjx_batch], args.mjx_collision)
        result['mjx']['hops'] = h
    return result


def run_mjx(urdf, kin, joints, q, tool_frame, hops, vertices, full_truth, collision):
    """Vectorised variant: jax.vmap of `mjx.forward` (and one `mjx.step`) over a batch of configurations.

    FK-only (default): the full model with contacts disabled, compared with CPU MuJoCo's tool pose.
    With `collision`: the full collision meshes need ~1.8 GB of contact buffers per configuration in MJX,
    so the batch runs on a hull-thinned copy; CPU MuJoCo on that same copy is the reference for MJX and
    its agreement with the full-mesh verdicts is reported as the simplification cost. The collision
    variant compiles for a very long time on the CPU backend (see docs/mujoco_crosscheck.md).
    """
    import tempfile

    import jax
    import jax.numpy as jnp
    import mujoco
    from mujoco import mjx
    out = {'batch': len(q), 'jax_backend': jax.default_backend(), 'collision': collision}
    if collision:
        out['hull_vertices'] = vertices
        with tempfile.TemporaryDirectory() as tmp:
            model = mm.load_mujoco(mm.simplify_meshes(urdf, tmp, vertices))
    else:
        model = mm.load_mujoco(urdf)
        model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    adr = mm.mujoco_qpos_addresses(model, joints)
    allowed = mm.allowed_pairs(kin, hops)
    data = mujoco.MjData(model)
    cpu_tool, cpu_hit = [], []
    for qi in q:
        data.qpos[adr] = qi
        mujoco.mj_forward(model, data)
        cpu_tool.append(mm.mujoco_pose(model, data, tool_frame))
        if collision:
            cpu_hit.append(len(mm.mujoco_colliding_pairs(model, data, allowed)) > 0)
    mx = mjx.put_model(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, tool_frame)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) for g in range(model.ngeom)]
    counts = jnp.asarray(np.array([[frozenset((a, b)) not in allowed for b in names] for a in names]))
    base = mjx.make_data(model)
    adr_j = jnp.asarray(adr)

    def one(qi):
        d = mjx.forward(mx, base.replace(qpos=base.qpos.at[adr_j].set(qi)))
        if not collision:
            return d.xpos[body], d.xmat[body].reshape(3, 3), jnp.bool_(False)
        # private MJX API (`_impl`), unverified and tied to the MuJoCo version in the results table (4.0.1)
        c = d._impl.contact
        g = jnp.clip(c.geom, 0, model.ngeom - 1)
        return d.xpos[body], d.xmat[body].reshape(3, 3), jnp.any((c.dist < 0) & counts[g[:, 0], g[:, 1]])

    batched = jax.jit(jax.vmap(one))
    qj = jnp.asarray(q)
    t0 = time.perf_counter()
    jax.block_until_ready(batched(qj))
    out['first_call_s_incl_compile'] = time.perf_counter() - t0
    t0 = time.perf_counter()
    pos, rot, hit = (np.asarray(x) for x in jax.block_until_ready(batched(qj)))
    out['second_call_s'] = time.perf_counter() - t0
    errs = []
    for i in range(len(q)):
        t = np.eye(4)
        t[:3, :3], t[:3, 3] = rot[i], pos[i]
        errs.append(mm.pose_error(t, cpu_tool[i]))
    out['fk_vs_cpu_mujoco'] = error_stats(*zip(*errs))
    if collision:
        cpu_hit = np.array(cpu_hit)
        out['hull_cpu_vs_full_mesh_cpu'] = mm.verdict_rates(full_truth, cpu_hit)
        out['collision_vs_hull_cpu'] = mm.verdict_rates(cpu_hit, hit)
        out['collision_vs_full_mesh_cpu'] = mm.verdict_rates(full_truth, hit)
    step = jax.jit(jax.vmap(lambda qi: mjx.step(mx, base.replace(qpos=base.qpos.at[adr_j].set(qi))).qpos[adr_j]))
    t0 = time.perf_counter()
    stepped = np.asarray(jax.block_until_ready(step(qj)))
    out['step_first_call_s_incl_compile'] = time.perf_counter() - t0
    out['step_finite'] = bool(np.all(np.isfinite(stepped)))
    return out


def markdown(r):
    lines = [f"### {r['robot']} (n={r['n']}, seed={r['seed']}, mujoco {r['mujoco']})", '',
             f"joint limit max |mujoco - urdf|: {r['joint_limit_max_abs_diff']:.2e}", '',
             '| FK comparison (tool frame) | n | pos mean / max mm | rot mean / max deg |', '|---|---|---|---|']
    for k, v in r['fk'].items():
        lines.append(f"| {k} | {v['n']} | {v['pos_mm_mean']:.2e} / {v['pos_mm_max']:.2e} | "
                     f"{v['rot_deg_mean']:.2e} / {v['rot_deg_max']:.2e} |")
    for k, c in r['collision'].items():
        lines += ['', f"Self-collision vs MuJoCo ({k}, MuJoCo collision rate {c['truth_collision_rate']:.3f}, n={c['n']})", '',
                  '| model | agree | false-free | false-blocked | false-free / collisions | false-blocked / free | model collision rate |',
                  '|---|---|---|---|---|---|---|']
        for name, v in c['models'].items():
            lines.append(f"| {name} | {v['agree']:.3f} | {v['false_free']:.3f} | {v['false_blocked']:.3f} | "
                         f"{v['false_free_given_collision']:.3f} | {v['false_blocked_given_free']:.3f} | {v['other_collision_rate']:.3f} |")
    if 'mjx' in r:
        m = r['mjx']
        lines += ['', f"MJX ({m['jax_backend']}, batch {m['batch']}, {'contacts on a hull-thinned model' if m['collision'] else 'contacts disabled'}): "
                      f"compile+first call {m['first_call_s_incl_compile']:.1f} s, second call {m['second_call_s']:.4f} s; "
                      f"mjx.step first call {m['step_first_call_s_incl_compile']:.1f} s, finite: {m['step_finite']}",
                  f"- tool FK vs CPU MuJoCo on the same model: max {m['fk_vs_cpu_mujoco']['pos_mm_max']:.2e} mm, "
                  f"{m['fk_vs_cpu_mujoco']['rot_deg_max']:.2e} deg (float32)"]
        if m['collision']:
            lines += [f"- contact verdict vs CPU MuJoCo on the same model: agree {m['collision_vs_hull_cpu']['agree']:.3f}",
                      f"- thinned-hull CPU vs full-mesh CPU: agree {m['hull_cpu_vs_full_mesh_cpu']['agree']:.3f}, "
                      f"false-free {m['hull_cpu_vs_full_mesh_cpu']['false_free']:.3f}, false-blocked {m['hull_cpu_vs_full_mesh_cpu']['false_blocked']:.3f}"]
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default='ur10e', help='label; names starting with ur10e also get the analytic DH FK')
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--foam-urdf')
    ap.add_argument('--curobo-cfg')
    ap.add_argument('--curobo-dump')
    ap.add_argument('--tool-frame', default='tcp_link')
    ap.add_argument('--n', type=int, default=2000)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--hops', type=int, nargs='+', default=[1, 2])
    ap.add_argument('--mjx-batch', type=int, default=0, help='configurations for the jax/MJX variant (0 = skip)')
    ap.add_argument('--mjx-vertices', type=int, default=32, help='hull vertices per collision mesh for the MJX batch')
    ap.add_argument('--mjx-collision', action='store_true',
                    help='also compare MJX contact verdicts (hull-thinned model; very slow to compile on CPU)')
    ap.add_argument('--json')
    args = ap.parse_args()
    r = run(args)
    print(markdown(r))
    if args.json:
        json.dump(r, open(args.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
