#!/usr/bin/env python3
"""Evaluate cuRobo v2's standalone Mapper (TSDF + ESDF) on synthetic depth frames.

Self-contained: no ROS, no simulator, no dataset. A known scene (a wall plane at x = 1 m and a
box) is ray cast with numpy into depth frames of a pinhole camera along a few poses, the frames go
into `curobo.perception.Mapper`, and the resulting ESDF is compared cell by cell with the analytic
signed distance. Each configuration runs in its own subprocess so peak VRAM is per configuration
and an out-of-memory failure is reported instead of killing the sweep.

Run inside the GPU image (cuRobo lives in /opt/venv):

    docker run --rm --gpus all -v "$PWD":/ws:ro motion-planner:gpu \
        /opt/venv/bin/python /ws/docker/tools/eval_curobo_mapper.py            # full sweep
    ... eval_curobo_mapper.py --extent 2 --voxel 0.04 --frames 50 --res 320x240   # one config
    ... eval_curobo_mapper.py --bridge                                         # ESDF -> set_esdf

VRAM is the device-wide used memory (torch.cuda.mem_get_info, the same driver counter as
nvidia-smi) polled every 20 ms, minus the value before the run, plus torch's own
max_memory_allocated. Warp (cuRobo's kernels) allocates outside torch, so torch alone under-reports.
"""
import argparse
import json
import math
import subprocess
import sys
import threading
import time

import numpy as np

WALL_X = 1.0                                  # wall plane x = 1 m, free space at x < 1
BOX_C = np.array([0.6, 0.25, -0.1])           # box centre
BOX_H = np.array([0.15, 0.15, 0.15])          # box half extents
FX = 525.0                                    # focal length (px) at 640x480; scaled for other sizes
MAX_DEPTH = 4.0


# --------------------------------------------------------------------------- scene and camera
def sdf_scene(p):
    """Analytic signed distance (m, positive outside) of the wall half space and the box. (N, 3)."""
    d_wall = WALL_X - p[:, 0]                 # positive in front of the wall, negative behind it
    q = np.abs(p - BOX_C) - BOX_H
    d_box = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    return np.minimum(d_wall, d_box), d_wall, d_box


def camera_pose(i, n, center):
    """Camera-to-world 4x4, OpenCV axes (x right, y down, z forward), on a small arc looking at the wall."""
    t = 2 * math.pi * i / max(n, 1)
    pos = np.array([center[0] - 1.3 + 0.2 * math.cos(t), 0.35 * math.sin(t), 0.1 * math.cos(2 * t)])
    target = np.array([WALL_X, 0.15 * math.sin(t + 1.0), -0.1])
    fwd = target - pos
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = right, down, fwd, pos
    return m


def render_depth(pose, h, w, fx, noise=0.0, rng=None):
    """z-depth image (h, w) in metres, 0 where nothing is hit within MAX_DEPTH."""
    u, v = np.meshgrid(np.arange(w) + 0.0, np.arange(h) + 0.0)
    rays = np.stack([(u - w / 2) / fx, (v - h / 2) / fx, np.ones_like(u)], -1).reshape(-1, 3)
    d = rays @ pose[:3, :3].T                 # world directions, z-component in camera = 1 => t is z-depth
    o = pose[:3, 3]
    with np.errstate(divide='ignore', invalid='ignore'):
        t_wall = np.where(d[:, 0] > 1e-9, (WALL_X - o[0]) / d[:, 0], np.inf)
        lo, hi = (BOX_C - BOX_H - o) / d, (BOX_C + BOX_H - o) / d
    tmin = np.minimum(lo, hi)
    tmax = np.maximum(lo, hi)
    tmin = np.where(np.isnan(tmin), -np.inf, tmin).max(axis=1)
    tmax = np.where(np.isnan(tmax), np.inf, tmax).min(axis=1)
    t_box = np.where((tmax >= tmin) & (tmax > 0), tmin, np.inf)
    t = np.minimum(t_wall, t_box)
    t[(t > MAX_DEPTH) | (t <= 0)] = 0.0
    depth = t.reshape(h, w).astype(np.float32)
    if noise > 0:
        depth = depth + (rng.standard_normal(depth.shape) * noise * (depth > 0)).astype(np.float32)
    return depth


# --------------------------------------------------------------------------- one configuration
class VramPeak(threading.Thread):
    """Polls device-wide used memory (MiB) from the driver."""

    def __init__(self, torch):
        super().__init__(daemon=True)
        self.torch, self.stop_flag = torch, False
        self.base = self.used()
        self.peak = self.base

    def used(self):
        free, total = self.torch.cuda.mem_get_info()
        return (total - free) / 2**20

    def run(self):
        while not self.stop_flag:
            self.peak = max(self.peak, self.used())
            time.sleep(0.02)


def eval_single(a):
    import torch
    from curobo.perception import FilterDepth, Mapper, MapperCfg
    from curobo.types import CameraObservation, Pose

    w, h = (int(s) for s in a.res.split('x'))
    ex = (a.extent, a.extent, 2.0)
    center = (0.5 if a.extent == 2 else 1.0, 0.0, 0.0)       # the wall at x = 1 lies inside both
    out = dict(extent=ex, voxel=a.voxel, seeding=a.seeding, trunc_vox=a.trunc, res=f'{w}x{h}', frames=a.frames, noise=a.noise, ok=False)
    torch.cuda.init()
    mon = VramPeak(torch)
    mon.start()
    try:
        cfg = MapperCfg(
            extent_meters_xyz=ex, voxel_size=a.voxel, esdf_voxel_size=a.voxel,
            extent_esdf_meters_xyz=ex, grid_center=torch.tensor(center),
            truncation_distance=a.voxel * a.trunc, depth_minimum_distance=0.1,
            depth_maximum_distance=MAX_DEPTH, minimum_tsdf_weight=1.0,
            image_height=h, image_width=w, block_size=8, seeding_method=a.seeding)
        out['max_blocks'] = cfg.max_blocks
        mapper = Mapper(cfg)
        out['mapper_mb_reported'] = round(mapper.memory_usage_mb(), 1)
        fx = FX * w / 640.0
        K = torch.tensor([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1.0]], device='cuda')
        filt = FilterDepth(image_shape=(h, w), depth_minimum_distance=0.1,
                           depth_maximum_distance=MAX_DEPTH) if a.filter else None
        rng = np.random.default_rng(0)

        def frame(i):
            pose = camera_pose(i, a.frames, center)
            depth = torch.from_numpy(render_depth(pose, h, w, fx, a.noise, rng)).cuda()
            if filt is not None:
                depth = filt(depth.unsqueeze(0))[0][0]
            pt = torch.tensor(pose, dtype=torch.float32, device='cuda')
            return CameraObservation(
                depth_image=depth.unsqueeze(0), rgb_image=torch.zeros(1, h, w, 3, dtype=torch.uint8, device='cuda'),
                pose=Pose.from_matrix(pt), intrinsics=K.unsqueeze(0))

        # the first frame compiles the Warp kernels; it is timed apart and not counted
        t0 = time.perf_counter()
        mapper.integrate(frame(0))
        torch.cuda.synchronize()
        out['first_integrate_s_incl_compile'] = round(time.perf_counter() - t0, 2)
        mapper.reset()
        obs = [frame(i) for i in range(a.frames)]               # rendering stays out of the timing
        times = []
        for o in obs:
            s, e = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
            s.record()
            mapper.integrate(o)
            e.record()
            e.synchronize()
            times.append(s.elapsed_time(e))
        out['integrate_ms_mean'] = round(float(np.mean(times)), 2)
        out['integrate_ms_p95'] = round(float(np.percentile(times, 95)), 2)
        t0 = time.perf_counter()
        grid = mapper.compute_esdf()
        torch.cuda.synchronize()
        out['esdf_first_s_incl_compile'] = round(time.perf_counter() - t0, 2)
        et = []
        for _ in range(5):
            s, e = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
            s.record()
            grid = mapper.compute_esdf()
            e.record()
            e.synchronize()
            et.append(s.elapsed_time(e))
        out['esdf_ms_median'] = round(float(np.median(et)), 2)
        st = mapper.get_stats()
        out['stats'] = {k: st[k] for k in ('num_allocated', 'active_blocks', 'allocation_failures',
                                          'tsdf_memory_mb', 'esdf_memory_mb', 'total_memory_mb')}
        out['esdf_shape'] = list(grid.feature_tensor.shape)
        out['esdf_dtype'] = str(grid.feature_tensor.dtype)
        out['esdf_voxel'] = round(float(grid.voxel_size), 5)
        out['esdf_pose'] = [round(float(x), 4) for x in grid.pose]
        out['accuracy'] = accuracy(grid, a)
        out['ok'] = True
        out['torch_max_allocated_mb'] = round(torch.cuda.max_memory_allocated() / 2**20, 1)
    except Exception as ex_:                       # noqa: BLE001 - the point is to report it verbatim
        out['error'] = f'{type(ex_).__name__}: {ex_}'[:600]
    mon.stop_flag = True
    mon.join()
    out['vram_peak_mb_above_baseline'] = round(mon.peak - mon.base, 1)
    out['vram_baseline_mb'] = round(mon.base, 1)
    out['vram_peak_total_mb'] = round(mon.peak, 1)
    return out


def cell_centers(shape, vs, origin):
    """Voxel-centre world coordinates of an (nx, ny, nz) grid, X slowest: origin + (i + 0.5 - n/2) vs."""
    ax = [origin[k] + (np.arange(shape[k]) + 0.5 - shape[k] / 2) * vs for k in range(3)]
    return np.stack(np.meshgrid(*ax, indexing='ij'), -1)


def accuracy(grid, a):
    esdf = grid.feature_tensor.float().cpu().numpy()
    nx, ny, nz = esdf.shape
    pts = cell_centers(esdf.shape, float(grid.voxel_size), [float(x) for x in grid.pose[:3]])
    d, d_wall, d_box = sdf_scene(pts.reshape(-1, 3))
    d, esdf_f, p = d, esdf.reshape(-1), pts.reshape(-1, 3)
    # cells the cameras can judge: central window of the wall, outside the box's occluded shadow
    shadow = (p[:, 0] > BOX_C[0] + BOX_H[0]) & (np.abs(p[:, 1] - BOX_C[1]) < 0.35) & (np.abs(p[:, 2] - BOX_C[2]) < 0.35)
    win = (np.abs(p[:, 1]) <= 0.5) & (np.abs(p[:, 2]) <= 0.4) & ~shadow
    unobs = esdf_f >= 9999.0
    res = {'unobserved_fraction_in_window': round(float(unobs[win].mean()), 3)}
    bands = {'near_outside(0,0.25]': (d > 0) & (d <= 0.25), 'far_outside(0.25,1]': (d > 0.25) & (d <= 1.0),
             'inside[-0.25,0)': (d < 0) & (d >= -0.25)}
    for name, m in bands.items():
        m = m & win & ~unobs
        if m.sum() == 0:
            res[name] = None
            continue
        e = esdf_f[m] - d[m]
        res[name] = dict(n=int(m.sum()), mean_signed=round(float(e.mean()), 4), mean_abs=round(float(np.abs(e).mean()), 4),
                         max_abs=round(float(np.abs(e).max()), 4), sign_agree=round(float((np.sign(esdf_f[m]) == np.sign(d[m])).mean()), 3))
    # wall surface itself: the ESDF at the cells straddling x = 1 should change sign there
    col = (np.abs(p[:, 1]) <= 0.1) & (np.abs(p[:, 2] + 0.3) <= 0.1) & ~unobs
    if col.any():
        x = p[col, 0]
        v = esdf_f[col]
        # zero crossing along x by linear interpolation of the column mean
        xs = np.unique(x)
        vs_ = np.array([v[x == xi].mean() for xi in xs])
        idx = np.where((vs_[:-1] > 0) & (vs_[1:] <= 0))[0]
        if len(idx):
            i = idx[0]
            res['wall_zero_crossing_x'] = round(float(xs[i] + (xs[i + 1] - xs[i]) * vs_[i] / (vs_[i] - vs_[i + 1])), 4)
    return res


# --------------------------------------------------------------------------- bridge check
def eval_bridge(a):
    """Mapper ESDF -> CuroboPlanner.set_esdf, with a known obstacle (the wall) next to the arm."""
    import torch
    from curobo.perception import Mapper, MapperCfg
    from curobo.types import CameraObservation, Pose
    sys.path.insert(0, a.bridge_src)
    from curobo_bridge.planner import CuroboPlanner, EsdfSpec

    out = {}
    w, h, vs = 320, 240, 0.05
    ex, center = (2.0, 2.0, 2.0), (0.5, 0.0, 0.0)
    cfg = MapperCfg(extent_meters_xyz=ex, voxel_size=vs, esdf_voxel_size=vs, extent_esdf_meters_xyz=ex,
                    grid_center=torch.tensor(center), truncation_distance=vs * 6, depth_maximum_distance=MAX_DEPTH,
                    minimum_tsdf_weight=1.0, image_height=h, image_width=w)
    m = Mapper(cfg)
    fx = FX * w / 640.0
    K = torch.tensor([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1.0]], device='cuda').unsqueeze(0)
    for i in range(20):
        pose = camera_pose(i, 20, center)
        dep = torch.from_numpy(render_depth(pose, h, w, fx)).cuda().unsqueeze(0)
        m.integrate(CameraObservation(depth_image=dep, rgb_image=torch.zeros(1, h, w, 3, dtype=torch.uint8, device='cuda'),
                                      pose=Pose.from_matrix(torch.tensor(pose, dtype=torch.float32, device='cuda')), intrinsics=K))
    grid = m.compute_esdf()
    field = grid.feature_tensor.float().cpu().numpy()
    out['mapper_esdf'] = dict(shape=list(field.shape), dtype=str(grid.feature_tensor.dtype), voxel=float(grid.voxel_size),
                              pose=[float(x) for x in grid.pose], min=float(field.min()), max=float(field.max()),
                              contiguous=bool(grid.feature_tensor.is_contiguous()), device=str(grid.feature_tensor.device))
    spec = EsdfSpec(dims=ex, voxel_size=vs, center=tuple(float(x) for x in grid.pose[:3]))
    out['spec_shape'] = list(spec.shape())
    out['layout_matches'] = tuple(field.shape) == spec.shape()
    # unobserved cells hold +10000 (cuRobo's "far"); our planner's grids clamp at +-max_distance
    out['unobserved_fraction'] = round(float((field >= 9999).mean()), 3)
    field = np.minimum(field, 1.0)                 # the clamp the bridge/scene_model fields use (max_distance 1 m)
    p = CuroboPlanner(a.robot, esdf=spec)
    out['set_esdf_obstacles'] = p.set_esdf(field, center=spec.center)
    out['set_esdf_ok'] = True
    # Functional check: random joint states of the Franka (base at the origin, reach ~0.85 m, so the
    # box at (0.6, 0.25, -0.1) is reachable). Validity against the Mapper's ESDF vs against the analytic one.
    lo, hi = p._joint_limits()
    rng = np.random.default_rng(1)
    qs = rng.uniform(lo, hi, size=(a.n_states, len(lo))).tolist()
    pts = cell_centers(spec.shape(), vs, spec.center).reshape(-1, 3)
    truth = np.minimum(sdf_scene(pts)[0], 1.0).reshape(spec.shape()).astype(np.float32)
    v_map = np.array(p.check_states(qs))
    p.set_esdf(truth, center=spec.center)
    v_true = np.array(p.check_states(qs))
    p.clear_world()
    v_free = np.array(p.check_states(qs))
    out['check_states'] = dict(n=len(qs), valid_free_world=int(v_free.sum()), valid_with_analytic_esdf=int(v_true.sum()),
                               valid_with_mapper_esdf=int(v_map.sum()), disagree=int((v_map != v_true).sum()),
                               mapper_valid_but_analytic_invalid=int((v_map & ~v_true).sum()),
                               mapper_invalid_but_analytic_valid=int((~v_map & v_true).sum()))
    return out


# --------------------------------------------------------------------------- sweep
def sweep(a):
    grid = [(ext, vox, res, seed) for seed in ('gather', 'scatter') for ext in (2, 4) for vox in (0.08, 0.04, 0.02)
            for res in ('320x240', '640x480')]
    if a.stress:                                   # beyond the requested set: find where 4 GB ends
        grid += [(4, 0.01, '640x480', 'gather'), (6, 0.02, '640x480', 'gather'), (8, 0.02, '640x480', 'gather'),
                 (8, 0.01, '640x480', 'gather')]
    rows = []
    for ext, vox, res, seed in grid:
        cmd = [sys.executable, __file__, '--extent', str(ext), '--voxel', str(vox), '--res', res, '--frames', str(a.frames),
               '--noise', str(a.noise), '--trunc', str(a.trunc), '--seeding', seed, '--single']
        if a.filter:
            cmd.append('--filter')
        r = subprocess.run(cmd, capture_output=True, text=True)
        line = [ln for ln in r.stdout.splitlines() if ln.startswith('RESULT ')]
        if line:
            row = json.loads(line[-1][7:])
        else:
            row = dict(extent=[ext, ext, 2.0], voxel=vox, res=res, seeding=seed, ok=False,
                       error=f'subprocess exit {r.returncode}: ' + ' | '.join(r.stderr.strip().splitlines()[-3:])[:400])
        rows.append(row)
        print(f'{ext}x{ext}x2 vox {vox} {res} {seed}:', 'ok' if row['ok'] else row.get('error'), flush=True)
    print('RESULTS ' + json.dumps(rows))
    print_table(rows)


def print_table(rows):
    print('\n| extent m | voxel m | image | seeding | VRAM peak MiB (device total / above start) | integrate ms | ESDF ms | ESDF grid | '
          'near err mean/max m | far err mean/max m | result |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        ext = 'x'.join(f'{e:g}' for e in r['extent'])
        head = f"| {ext} | {r['voxel']} | {r['res']} | {r.get('seeding', '')} |"
        vram = f"{r.get('vram_peak_total_mb', '-')} / {r.get('vram_peak_mb_above_baseline', '-')}"
        if not r['ok']:
            print(f"{head} {vram} | - | - | - | - | - | FAILED: {r.get('error', '')[:160]} |")
            continue
        acc = r['accuracy']
        n, f = acc.get('near_outside(0,0.25]'), acc.get('far_outside(0.25,1]')
        fmt = lambda b: f"{b['mean_abs']:.3f}/{b['max_abs']:.3f}" if b else '-'
        print(f"{head} {vram} | {r['integrate_ms_mean']} | {r['esdf_ms_median']} | {'x'.join(map(str, r['esdf_shape']))} | "
              f"{fmt(n)} | {fmt(f)} | ok |")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--extent', type=float, help='XY extent in m (Z is 2 m); with --voxel runs one config')
    ap.add_argument('--voxel', type=float)
    ap.add_argument('--res', default='640x480')
    ap.add_argument('--frames', type=int, default=50)
    ap.add_argument('--noise', type=float, default=0.0, help='gaussian depth noise sigma (m)')
    ap.add_argument('--trunc', type=float, default=4.0, help='truncation distance in voxels')
    ap.add_argument('--seeding', default='gather', choices=('gather', 'scatter'), help='ESDF seeding (cuRobo default: gather)')
    ap.add_argument('--stress', action='store_true', help='sweep: also try larger extents / finer voxels to find the 4 GB limit')
    ap.add_argument('--filter', action='store_true', help='run FilterDepth before integrating')
    ap.add_argument('--single', action='store_true', help=argparse.SUPPRESS)
    ap.add_argument('--bridge', action='store_true', help='feed the Mapper ESDF to CuroboPlanner.set_esdf')
    ap.add_argument('--bridge-src', default='/ws/src/manipulation/curobo_bridge')
    ap.add_argument('--robot', default='franka.yml')
    ap.add_argument('--n-states', type=int, default=400)
    a = ap.parse_args()
    if a.bridge:
        print('BRIDGE ' + json.dumps(eval_bridge(a)))
    elif a.extent and a.voxel:
        r = eval_single(a)
        print('RESULT ' + json.dumps(r))
        if not a.single:
            print_table([r])
            print(json.dumps(r, indent=1))
    else:
        sweep(a)


if __name__ == '__main__':
    main()
