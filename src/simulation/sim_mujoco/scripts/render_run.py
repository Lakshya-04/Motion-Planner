#!/usr/bin/env python3
"""Photos and videos of a MuJoCo sim run, rendered headless (mujoco.Renderer, MUJOCO_GL=egl or osmesa).

The physics runs inside the ros2_control plugin, which has no camera, so a run is filmed in two steps:

  record  (inside the ROS environment, needs only rclpy)
      render_run.py record --out run.npz            # until SIGINT/SIGTERM or --timeout
    stores /joint_states and the ground-truth contact force (|Fz|) with their sim-clock stamps.
  render  (anywhere with the `mujoco` python package, numpy, Pillow; ffmpeg for video)
      MUJOCO_GL=egl render_run.py video  --mjcf robot.xml --run run.npz --out run.mp4 [--gif run.gif]
      render_run.py encode --frames run.mp4.frames --out run.mp4 --gif run.gif   # ffmpeg, if `video` did not find it
      MUJOCO_GL=egl render_run.py stills --mjcf robot.xml --run run.npz --out-dir media/

The MJCF is the one the launch file generated (pass `out_dir:=DIR` to sim_mujoco.launch.py to keep it; when
--mjcf is a directory its first *.xml is taken). Joint positions are replayed by name; the mimic lift stages
(an MJCF <equality><joint> constraint) are filled in from the constraint polynomial. The tool point is the
body `tcp_link`. A render-only stem is added between the tool mount and the pad disc (the model has a gap). The `motion-planner:sim` image renders with EGL (Mesa software rasteriser), no GPU needed.

Stills: overview, arm_at_wall (the highest-force sample in the middle of the pressing), tool_closeup. The video
has a main view, a close-up inset on the tool and a force trace.
"""
import argparse
import math
import os
import shutil
import signal
import subprocess
import threading
import time


# ----------------------------------------------------------------------------------------------- recording
def record(args):
    import numpy as np
    import rclpy
    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node('render_run_recorder')
    names, rows, force, base = [], [], [], []
    lock = threading.Lock()
    last = [-1.0]

    def stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_js(m):
        t = stamp(m)
        with lock:
            if t - last[0] < 1.0 / args.rate:
                return
            last[0] = t
            if not names:
                names.extend(m.name)
            d = dict(zip(m.name, m.position))
            rows.append([t] + [d.get(n, np.nan) for n in names])

    def on_ft(m):
        with lock:
            force.append((stamp(m), abs(m.wrench.force.z)))

    def on_base(m):
        p, o = m.pose.position, m.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        with lock:
            if not base or stamp(m) - base[-1][0] >= 1.0 / args.rate:
                base.append((stamp(m), p.x, p.y, yaw))

    node.create_subscription(JointState, '/joint_states', on_js, 10)
    node.create_subscription(PoseStamped, '/sim_mujoco/base_pose', on_base, qos_profile_sensor_data)
    node.create_subscription(WrenchStamped, '/sim_mujoco/ft_ground_truth', on_ft, qos_profile_sensor_data)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.monotonic()
    while not stop.is_set() and time.monotonic() - t0 < args.timeout:
        time.sleep(0.1)
    with lock:
        a = np.array(rows) if rows else np.zeros((0, 1))
        f = np.array(force) if force else np.zeros((0, 2))
        bs = np.array(base) if base else np.zeros((0, 4))
        np.savez_compressed(args.out, t=a[:, 0], q=a[:, 1:], names=np.array(names), ft=f, base=bs)
    print(f'recorded {len(a)} joint-state samples, {len(f)} force samples, {len(bs)} base poses to {args.out}', flush=True)
    rclpy.shutdown()


# ------------------------------------------------------------------------------------------------ rendering
class Replay:
    def __init__(self, mjcf, run):
        import mujoco
        import numpy as np
        self.mj, self.np = mujoco, np
        if os.path.isdir(mjcf):
            mjcf = os.path.join(mjcf, sorted(f for f in os.listdir(mjcf) if f.endswith('.xml'))[0])
        spec = mujoco.MjSpec.from_file(mjcf)
        self.add_backdrop(spec)
        self.model = m = spec.compile()
        self.data = mujoco.MjData(m)
        r = np.load(run, allow_pickle=False)
        self.t, self.q, self.ft = r['t'], r['q'], r['ft']
        self.base = r['base'] if 'base' in r.files and len(r['base']) else None   # t, x, y, yaw of the driven base
        self.t0 = float(self.t[0])
        self.names = [str(n) for n in r['names']]
        self.adr = {}
        for i, n in enumerate(self.names):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            if j >= 0:
                self.adr[i] = int(m.jnt_qposadr[j])
        self.base_adr = [int(m.jnt_qposadr[j]) if j >= 0 else -1 for j in (
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ('base_x_joint', 'base_y_joint', 'base_yaw_joint'))]
        if self.base is not None:
            self.base = self.base.copy()
            self.base[:, 3] = np.unwrap(self.base[:, 3])
        self.mimics = []   # (qpos adr of follower, qpos adr of leader, poly c0..c4): q1 = poly(q2)
        for e in range(m.neq):
            if m.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT:
                j1, j2 = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
                self.mimics.append((int(m.jnt_qposadr[j1]), int(m.jnt_qposadr[j2]), m.eq_data[e][:5].copy()))
        self.tcp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'tcp_link')
        self.force = np.interp(self.t, self.ft[:, 0], self.ft[:, 1]) if len(self.ft) else np.zeros_like(self.t)
        mujoco.mj_resetData(m, self.data)
        self.base_qpos = self.data.qpos.copy()

    def add_backdrop(self, spec):
        """Sky gradient, a checkered floor and brighter lighting (the generated MJCF has none: black background)."""
        mj = self.mj
        spec.add_texture(name='rr_sky', type=mj.mjtTexture.mjTEXTURE_SKYBOX, builtin=mj.mjtBuiltin.mjBUILTIN_GRADIENT,
                         rgb1=[0.55, 0.72, 0.9], rgb2=[0.95, 0.97, 1.0], width=512, height=512)
        spec.add_texture(name='rr_floor', type=mj.mjtTexture.mjTEXTURE_2D, builtin=mj.mjtBuiltin.mjBUILTIN_CHECKER,
                         rgb1=[0.62, 0.64, 0.66], rgb2=[0.5, 0.52, 0.55], width=256, height=256, mark=mj.mjtMark.mjMARK_NONE)
        mat = spec.add_material(name='rr_floor')
        mat.textures[mj.mjtTextureRole.mjTEXROLE_RGB] = 'rr_floor'
        mat.texrepeat = [10, 10]
        mat.texuniform = True
        g = spec.worldbody.add_geom(name='rr_floor', type=mj.mjtGeom.mjGEOM_PLANE, size=[6, 6, 0.1], material='rr_floor')
        g.contype = g.conaffinity = 0
        # render-only stem between the tool mount cylinder and the pad disc (the model has a gap there; no collision)
        mount = spec.body('tool_mount_link')
        if mount is not None:
            st = mount.add_geom(name='rr_stem', type=mj.mjtGeom.mjGEOM_CYLINDER, size=[0.012, 0.03, 0.0], pos=[0, 0, 0.13],
                                rgba=[0.45, 0.45, 0.48, 1])
            st.contype = st.conaffinity = 0
        spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
        spec.visual.headlight.diffuse = [0.7, 0.7, 0.7]

    def pose(self, ts):
        np = self.np
        self.data.qpos[:] = self.base_qpos
        for i, a in self.adr.items():
            v = np.interp(ts, self.t, self.q[:, i])
            if not np.isnan(v):
                self.data.qpos[a] = v
        if self.base is not None:
            for k, adr in enumerate(self.base_adr):
                if adr >= 0:
                    self.data.qpos[adr] = np.interp(ts, self.base[:, 0], self.base[:, 1 + k])
        for a1, a2, c in self.mimics:
            x = self.data.qpos[a2]
            self.data.qpos[a1] = sum(c[k] * x ** k for k in range(5))
        self.mj.mj_forward(self.model, self.data)

    def tcp_pos(self):
        return self.data.xpos[self.tcp].copy()

    def camera(self, lookat, distance, azimuth, elevation):
        cam = self.mj.MjvCamera()
        cam.type = self.mj.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
        return cam


def make_renderer(rp, w, h):
    g = rp.model.vis.global_
    g.offwidth, g.offheight = max(g.offwidth, w), max(g.offheight, h)
    return rp.mj.Renderer(rp.model, h, w)


def shot(rp, renderer, cam, extra=None):
    renderer.update_scene(rp.data, camera=cam)
    if extra is not None:
        extra(renderer.scene)
    return renderer.render()


WALL_X = 0.98   # the sim wall face in the odom/base frame (smoke_wall_scene.json)


def valve_intervals(log):
    """(open, close) sim-clock stamps from the manager log's `valve ON` / `valve OFF` lines (a spray pass)."""
    import re
    out, t_on = [], None
    for line in open(log):
        m = re.search(r'\[(\d+\.\d+)\] \[manipulation_manager\]: valve (ON|OFF)', line)
        if not m:
            continue
        if m.group(2) == 'ON':
            t_on = float(m.group(1))
        elif t_on is not None:
            out.append((t_on, float(m.group(1))))
            t_on = None
    return out


def spray_geoms(rp, intervals, t, standoff, half_angle_deg):
    """Scene callback: while the valve is open a translucent cone from the tool to the wall, and the wall spots sprayed so far."""
    mj, np = rp.mj, rp.np
    ts = np.arange(rp.t0, t, 0.3)
    open_ = [any(a <= x <= b for a, b in intervals) for x in ts]
    spots = []
    for x, o in zip(ts, open_):
        if o:
            rp.pose(x)
            spots.append(rp.tcp_pos())
    rp.pose(t)
    tip = rp.tcp_pos()
    on = any(a <= t <= b for a, b in intervals)
    rad = standoff * np.tan(np.radians(half_angle_deg))

    def add(scene):
        def new(kind):
            if scene.ngeom >= scene.maxgeom:
                return None
            g = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(g, kind, np.zeros(3), np.zeros(3), np.zeros(9), np.zeros(4, np.float32))
            scene.ngeom += 1
            return g
        for sp in spots:
            g = new(mj.mjtGeom.mjGEOM_BOX)
            if g is None:
                break
            g.size[:] = [0.002, rad * 0.8, 0.045]
            g.pos[:] = [WALL_X - 0.004, sp[1], sp[2]]
            g.mat[:] = np.eye(3)
            g.rgba[:] = [1.0, 0.4, 0.0, 0.12]
        if on:
            g = new(mj.mjtGeom.mjGEOM_CYLINDER)
            if g is not None:
                end = np.array([WALL_X - 0.004, tip[1], tip[2]])
                mj.mjv_connector(g, mj.mjtGeom.mjGEOM_CAPSULE, rad * 0.3, tip, end)
                g.rgba[:] = [1.0, 0.6, 0.1, 0.4]
    return add


def stroke_time(rp):
    """Time of the highest-force sample in the central 60 % of the samples that are pressing."""
    np = rp.np
    idx = np.where(rp.force > 2.0)[0]
    if not len(idx):
        return float(rp.t[len(rp.t) // 2])
    lo, hi = idx[int(len(idx) * 0.2)], idx[int(len(idx) * 0.8)]
    return float(rp.t[lo + int(np.argmax(rp.force[lo:hi + 1]))])


def stills(args):
    from PIL import Image
    rp = Replay(args.mjcf, args.run)
    np = rp.np
    os.makedirs(args.out_dir, exist_ok=True)
    r = make_renderer(rp, args.width, args.height)

    def save(name, cam):
        Image.fromarray(shot(rp, r, cam)).save(os.path.join(args.out_dir, name), quality=88, optimize=True)

    rp.pose(rp.t0)
    save('overview.jpg', rp.camera(np.array([0.5, 0.1, 1.1]), 4.2, 40, -12))
    rp.pose(stroke_time(rp))
    p = rp.tcp_pos()
    save('arm_at_wall.jpg', rp.camera(p + [-0.1, 0.0, -0.05], 1.3, 55, -10))
    save('tool_closeup.jpg', rp.camera(p, 0.5, 80, -8))
    print('stills written to', args.out_dir)


def video(args):
    from PIL import Image, ImageDraw, ImageFont
    rp = Replay(args.mjcf, args.run)
    np = rp.np
    W, H = args.width, args.height
    IW, IH = W // 3, H // 3
    r_main, r_in = make_renderer(rp, W, H), make_renderer(rp, IW, IH)
    dur = float(rp.t[-1] - rp.t0)
    n = int(min(args.max_seconds, dur / args.max_speed) * args.fps)
    speed = dur / (n / args.fps)
    ts = rp.t0 + np.linspace(0.0, dur, n)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', max(14, H // 32))
    except OSError:
        font = ImageFont.load_default()
    fmax = max(15.0, float(rp.force.max()) * 1.1)
    valve = valve_intervals(args.valve_log) if args.valve_log else None
    frames = args.out + '.frames'
    os.makedirs(frames, exist_ok=True)
    for k, t in enumerate(ts):
        rp.pose(t)
        p = rp.tcp_pos()
        extra = spray_geoms(rp, valve, t, args.standoff, args.half_angle) if valve is not None else None
        img = Image.fromarray(shot(rp, r_main, rp.camera(np.array([0.5, 0.1, 1.1]), 3.4, 40, -12), extra))
        ins = Image.fromarray(shot(rp, r_in, rp.camera(p, 0.5, 80, -8), extra))
        img.paste(ins, (W - IW - 12, 12))
        d = ImageDraw.Draw(img)
        d.rectangle([W - IW - 13, 11, W - 11, IH + 13], outline=(255, 255, 255))
        f = float(np.interp(t, rp.t, rp.force))
        d.rectangle([12, H - 150, 560, H - 12], fill=(0, 0, 0))
        label = f't = {t - rp.t0:6.1f} s   x{speed:.0f}   contact force {f:5.1f} N'
        if valve is not None:
            label = f't = {t - rp.t0:6.1f} s  x{speed:.0f}  valve {"OPEN" if any(a <= t <= b for a, b in valve) else "closed"}  force {f:4.1f} N'
        d.text((22, H - 146), label, fill=(255, 255, 255), font=font)
        gx0, gy0, gx1, gy1 = 22, H - 108, 548, H - 20
        d.rectangle([gx0, gy0, gx1, gy1], outline=(110, 110, 110))
        sel = (rp.t >= rp.t0) & (rp.t <= t)
        step = max(1, int(sel.sum()) // 600)
        xs = gx0 + (rp.t[sel] - rp.t0) / dur * (gx1 - gx0)
        ys = gy1 - np.clip(rp.force[sel] / fmax, 0, 1) * (gy1 - gy0)
        pts = list(zip(xs[::step], ys[::step]))
        if len(pts) > 1:
            d.line(pts, fill=(90, 200, 255), width=2)
        img.save(os.path.join(frames, f'{k:05d}.jpg'), quality=92)
    print(f'{n} frames in {frames}, {dur:.0f} s of sim time at x{speed:.0f}; the fps for `encode` is {args.fps}')
    if shutil.which('ffmpeg'):
        encode(argparse.Namespace(frames=frames, out=args.out, gif=args.gif, fps=args.fps, crf=args.crf, gif_fps=args.gif_fps,
                                  gif_width=args.gif_width, gif_speedup=args.gif_speedup))


def encode(args):
    """JPEG frames -> mp4 (+ small gif); needs ffmpeg, which the sim image does not have: run it on the host."""
    subprocess.check_call(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(args.fps), '-i',
                           os.path.join(args.frames, '%05d.jpg'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', str(args.crf),
                           '-preset', 'slow', '-movflags', '+faststart', args.out])
    if args.gif:
        pal = args.out + '.pal.png'
        vf = f'setpts=PTS/{args.gif_speedup},fps={args.gif_fps},scale={args.gif_width}:-1:flags=lanczos'
        subprocess.check_call(['ffmpeg', '-y', '-loglevel', 'error', '-i', args.out, '-vf', vf + ',palettegen=max_colors=96', pal])
        subprocess.check_call(['ffmpeg', '-y', '-loglevel', 'error', '-i', args.out, '-i', pal, '-lavfi',
                               vf + '[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=4', args.gif])
        os.remove(pal)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('record')
    p.add_argument('--out', required=True)
    p.add_argument('--rate', type=float, default=20.0, help='joint-state samples per second of sim time')
    p.add_argument('--timeout', type=float, default=3600.0)
    p = sub.add_parser('encode', help='JPEG frames from `video` -> mp4 and gif (needs ffmpeg)')
    p.add_argument('--frames', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--gif')
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--crf', type=int, default=26)
    p.add_argument('--gif-fps', type=int, default=10)
    p.add_argument('--gif-width', type=int, default=480)
    p.add_argument('--gif-speedup', type=float, default=3.0)
    for name in ('stills', 'video'):
        p = sub.add_parser(name)
        p.add_argument('--mjcf', required=True, help='generated MJCF (or the directory holding it)')
        p.add_argument('--run', required=True, help='npz from `record`')
        p.add_argument('--width', type=int, default=1280)
        p.add_argument('--height', type=int, default=720)
        if name == 'stills':
            p.add_argument('--out-dir', required=True)
        else:
            p.add_argument('--out', required=True)
            p.add_argument('--gif')
            p.add_argument('--fps', type=int, default=30)
            p.add_argument('--max-seconds', type=float, default=75.0, help='longest video')
            p.add_argument('--max-speed', type=float, default=60.0, help='highest speed-up of the sim time')
            p.add_argument('--crf', type=int, default=26)
            p.add_argument('--gif-fps', type=int, default=10)
            p.add_argument('--gif-speedup', type=float, default=3.0, help='extra speed-up of the gif over the video')
            p.add_argument('--gif-width', type=int, default=480)
            p.add_argument('--valve-log', help='manager log of a spray pass: draws the spray cone and the sprayed spots')
            p.add_argument('--standoff', type=float, default=0.30, help='spray stand-off, m (with --valve-log)')
            p.add_argument('--half-angle', type=float, default=20.0, help='spray cone half-angle, deg (with --valve-log)')
    args = ap.parse_args()
    {'record': record, 'stills': stills, 'video': video, 'encode': encode}[args.cmd](args)


if __name__ == '__main__':
    main()
