"""Live "done / not done" coverage map of a wall. Pure python + numpy: no ROS, no simulator, no matplotlib.

A 2D grid on the wall plane (default 2 cm cells) from a wall description, in the plane frame (u, v):

    NOT_DONE (0)  not swept while in contact
    DONE     (1)  swept by the tool swath while in contact: force >= threshold AND |distance to the plane| <= tolerance
    FAILED   (2)  not swept, inside a column the manager reported failed or partial

Wall descriptions accepted by ``load_wall`` (same files the planner and the recorders write):
    wall_scene.json   {"walls": [{"origin_xyz", "basis_u", "basis_v", "normal_outward", "u_range_m", "v_range_m"}], "frame_id"}
    run.csv.wall.json {"origin", "u", "v", "u_range", "v_range"}   (written by the sim record_run.py scripts)

The swath is modelled as a capsule (round-nosed stroke) of width ``swath_m`` around the segment between two consecutive
in-contact tool points; the true swath of a flat tool has square ends, so cells within half a swath beyond a stroke end are
credited. A DONE cell never reverts; a later in-contact sweep turns FAILED into DONE.

Output for the viewers is a list of marker dicts (the dict format of ``overlay.py``): one TRIANGLE_LIST per status (merged
rectangles, so a clean wall is a handful of quads, not thousands of cells), the wall outline and a percent label. The RViz node
publishes them as visualization_msgs/MarkerArray, and ``OverlayState`` turns the same dicts into primitives for Gazebo and MuJoCo.
"""
import csv
import json
import math

import numpy as np

NOT_DONE, DONE, FAILED = 0, 1, 2
STATUS_NAMES = {NOT_DONE: 'not_done', DONE: 'done', FAILED: 'failed'}
STATUS_RGBA = {NOT_DONE: (0.85, 0.15, 0.15, 0.55), DONE: (0.10, 0.75, 0.20, 0.80), FAILED: (1.0, 0.55, 0.0, 0.80)}
DEFAULT_CELL_M = 0.02
DEFAULT_SWATH_M = 0.14        # = MULTI_STAND_SWATH of the sim scripts
DEFAULT_FORCE_N = 1.0
DEFAULT_PLANE_TOL_M = 0.03
NS_PREFIX = 'coverage_paint'


def _unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError('zero-length wall axis')
    return v / n


class Wall:
    """Plane origin + orthonormal (u, v) + outward normal n and the u/v extents."""

    def __init__(self, origin, u, v, u_range, v_range, normal=None, frame_id='', wall_id=0):
        self.origin = np.asarray(origin, float)
        self.u = _unit(u)
        v = np.asarray(v, float)
        v = _unit(v - (v @ self.u) * self.u)      # v as given (orthogonalised): the u/v ranges are measured along it
        self.v = v
        self.n = np.cross(self.u, v)
        if normal is not None and float(self.n @ np.asarray(normal, float)) < 0:
            self.n = -self.n                      # outward side of the wall: where the quads are lifted to
        self.u_range = (float(min(u_range)), float(max(u_range)))
        self.v_range = (float(min(v_range)), float(max(v_range)))
        if self.u_range[1] <= self.u_range[0] or self.v_range[1] <= self.v_range[0]:
            raise ValueError('wall needs a non-empty u and v range')
        self.frame_id, self.wall_id = frame_id, wall_id

    @property
    def area_m2(self):
        return (self.u_range[1] - self.u_range[0]) * (self.v_range[1] - self.v_range[0])

    def to_uvd(self, p):
        """World point -> (u, v, signed distance along the normal)."""
        d = np.asarray(p, float) - self.origin
        return float(d @ self.u), float(d @ self.v), float(d @ self.n)

    def to_world(self, u, v, d=0.0):
        return (self.origin + self.u * u + self.v * v + self.n * d).tolist()

    def corners(self, d=0.0):
        (u0, u1), (v0, v1) = self.u_range, self.v_range
        return [self.to_world(u, v, d) for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def wall_from_dict(doc, wall_id=0):
    """Wall from a wall_scene.json dict (walls list) or a recorder .wall.json dict."""
    if 'walls' in doc:
        for i, w in enumerate(doc['walls']):
            if int(w.get('id', i)) == int(wall_id):
                return Wall(w['origin_xyz'], w['basis_u'], w.get('basis_v', (0, 0, 1)), w['u_range_m'], w['v_range_m'],
                            normal=w.get('normal_outward'), frame_id=doc.get('frame_id', ''), wall_id=wall_id)
        raise KeyError('wall %s not in scene' % wall_id)
    return Wall(doc['origin'], doc['u'], doc['v'], doc['u_range'], doc['v_range'], frame_id=doc.get('frame_id', ''),
                wall_id=wall_id)


def load_wall(path, wall_id=0):
    with open(path) as f:
        return wall_from_dict(json.load(f), wall_id)


class CoverageGrid:
    def __init__(self, wall, cell_m=DEFAULT_CELL_M, swath_m=DEFAULT_SWATH_M, force_threshold_n=DEFAULT_FORCE_N,
                 plane_tol_m=DEFAULT_PLANE_TOL_M, max_gap_s=1.0):
        if cell_m <= 0 or swath_m <= 0:
            raise ValueError('cell_m and swath_m must be positive')
        self.wall, self.cell, self.swath = wall, float(cell_m), float(swath_m)
        self.force_threshold, self.plane_tol, self.max_gap = float(force_threshold_n), float(plane_tol_m), float(max_gap_s)
        self.nu = max(1, int(math.ceil((wall.u_range[1] - wall.u_range[0]) / self.cell - 1e-9)))
        self.nv = max(1, int(math.ceil((wall.v_range[1] - wall.v_range[0]) / self.cell - 1e-9)))
        self.status = np.zeros((self.nv, self.nu), np.uint8)
        self._prev = None          # (u, v, t) of the previous in-contact sample
        self.columns = {}          # column index -> [(u, v), ...] polyline on the wall
        self.samples = self.contact_samples = 0
        self.dirty = True

    # ---- painting --------------------------------------------------------------------------------------------
    def _paint(self, a, b, value, only_not_done=False):
        """Set cells within swath/2 of segment a-b (u, v tuples) to ``value`` (clipped to the wall)."""
        r = self.swath / 2.0
        u0, v0 = self.wall.u_range[0], self.wall.v_range[0]
        i0 = max(0, int(math.floor((min(a[0], b[0]) - r - u0) / self.cell)))
        i1 = min(self.nu - 1, int(math.floor((max(a[0], b[0]) + r - u0) / self.cell)))
        j0 = max(0, int(math.floor((min(a[1], b[1]) - r - v0) / self.cell)))
        j1 = min(self.nv - 1, int(math.floor((max(a[1], b[1]) + r - v0) / self.cell)))
        if i1 < i0 or j1 < j0:
            return 0
        us = u0 + (np.arange(i0, i1 + 1) + 0.5) * self.cell
        vs = v0 + (np.arange(j0, j1 + 1) + 0.5) * self.cell
        U, V = np.meshgrid(us, vs)
        ab = np.array([b[0] - a[0], b[1] - a[1]])
        L2 = float(ab @ ab)
        t = np.clip(((U - a[0]) * ab[0] + (V - a[1]) * ab[1]) / L2, 0.0, 1.0) if L2 > 1e-12 else np.zeros_like(U)
        dist2 = (U - (a[0] + t * ab[0])) ** 2 + (V - (a[1] + t * ab[1])) ** 2
        hit = dist2 <= r * r
        view = self.status[j0:j1 + 1, i0:i1 + 1]
        if only_not_done:
            hit &= view == NOT_DONE
        changed = int(np.count_nonzero(hit & (view != value)))
        view[hit] = value
        if changed:
            self.dirty = True
        return changed

    def in_contact(self, p, force):
        """Contact test: force above the threshold (ignored when threshold <= 0) and near the plane."""
        _, _, d = self.wall.to_uvd(p)
        return abs(d) <= self.plane_tol and (self.force_threshold <= 0 or force >= self.force_threshold)

    def add_sample(self, p, force, t=0.0):
        """One tool point (world frame of the wall) and the contact force magnitude [N]. Returns cells newly DONE."""
        self.samples += 1
        if not (math.isfinite(force) and all(math.isfinite(c) for c in p)) or not self.in_contact(p, force):
            self._prev = None
            return 0
        self.contact_samples += 1
        u, v, _ = self.wall.to_uvd(p)
        prev, self._prev = self._prev, (u, v, t)
        before = self.counts()[DONE]
        if prev is not None and 0 <= t - prev[2] <= self.max_gap and math.hypot(u - prev[0], v - prev[1]) < 10 * self.swath:
            self._paint((prev[0], prev[1]), (u, v), DONE)
        else:
            self._paint((u, v), (u, v), DONE)
        return self.counts()[DONE] - before

    def break_stroke(self):
        self._prev = None

    # ---- manager outcome -------------------------------------------------------------------------------------
    def set_columns(self, columns):
        """columns: {index: [(x, y, z), ...]} planned column polylines in the wall frame (e.g. ``load_plan_columns``)."""
        self.columns = {int(k): [self.wall.to_uvd(p)[:2] for p in pts] for k, pts in columns.items()}

    def mark_column(self, index, status=FAILED):
        """Every cell of the column's swath that is NOT_DONE becomes ``status`` (FAILED for failed or partial columns)."""
        pts = self.columns.get(int(index))
        if not pts:
            return 0
        n = 0
        segs = list(zip(pts[:-1], pts[1:])) or [(pts[0], pts[0])]
        for a, b in segs:
            n += self._paint(a, b, status, only_not_done=True)
        return n

    def mark_outcome(self, done=(), partial=(), failed=()):
        """ExecuteCoverage result lists. Done columns are not painted (only force-and-distance contact marks DONE)."""
        return sum(self.mark_column(i, FAILED) for i in list(partial) + list(failed))

    # ---- numbers ---------------------------------------------------------------------------------------------
    def counts(self):
        c = np.bincount(self.status.ravel(), minlength=3)
        return {NOT_DONE: int(c[0]), DONE: int(c[1]), FAILED: int(c[2])}

    def percent_done(self):
        return 100.0 * self.counts()[DONE] / float(self.status.size)

    def summary(self):
        c = self.counts()
        a = self.cell * self.cell
        return {'percent_done': round(self.percent_done(), 2), 'cells': int(self.status.size),
                'done_m2': round(c[DONE] * a, 4), 'failed_m2': round(c[FAILED] * a, 4),
                'not_done_m2': round(c[NOT_DONE] * a, 4), 'wall_m2': round(self.wall.area_m2, 4),
                'samples': self.samples, 'contact_samples': self.contact_samples}

    # ---- geometry for the viewers ----------------------------------------------------------------------------
    def rects(self, status):
        """Merged cell rectangles of one status as (i0, i1, j0, j1) inclusive cell index ranges (greedy: horizontal
        runs, then rows with identical runs stacked)."""
        mask = self.status == status
        open_runs, out = {}, []
        for j in range(self.nv):
            row = mask[j]
            runs = set()
            if row.any():
                edges = np.diff(np.concatenate(([0], row.view(np.int8), [0])))
                runs = set(zip(np.flatnonzero(edges == 1).tolist(), (np.flatnonzero(edges == -1) - 1).tolist()))
            nxt = {}
            for r in runs:
                nxt[r] = open_runs.pop(r) if r in open_runs else j
            out += [(r[0], r[1], j0, j - 1) for r, j0 in open_runs.items()]
            open_runs = nxt
        out += [(r[0], r[1], j0, self.nv - 1) for r, j0 in open_runs.items()]
        return out

    def rect_corners(self, rect, lift_m=0.003):
        i0, i1, j0, j1 = rect
        ua, ub = self.wall.u_range[0] + i0 * self.cell, min(self.wall.u_range[1], self.wall.u_range[0] + (i1 + 1) * self.cell)
        va, vb = self.wall.v_range[0] + j0 * self.cell, min(self.wall.v_range[1], self.wall.v_range[0] + (j1 + 1) * self.cell)
        w = self.wall
        return [w.to_world(ua, va, lift_m), w.to_world(ub, va, lift_m), w.to_world(ub, vb, lift_m), w.to_world(ua, vb, lift_m)]

    def marker_dicts(self, lift_m=0.003, label=True, outline=True, ns_prefix=NS_PREFIX):
        """Marker dicts (overlay.py format, in the wall frame) of the current map."""
        out = []
        ident = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        for st in (NOT_DONE, DONE, FAILED):
            pts = []
            for r in self.rects(st):
                c = self.rect_corners(r, lift_m)
                pts += [tuple(c[0]), tuple(c[1]), tuple(c[2]), tuple(c[0]), tuple(c[2]), tuple(c[3])]
            out.append({'type': 'TRIANGLE_LIST', 'ns': '%s_%s' % (ns_prefix, STATUS_NAMES[st]), 'id': 0,
                        'action': 'ADD' if pts else 'DELETE', 'frame': self.wall.frame_id, 'pose': ident,
                        'scale': (1.0, 1.0, 1.0), 'color': STATUS_RGBA[st], 'points': pts, 'text': ''})
        if outline:
            c = self.wall.corners(lift_m * 2)
            out.append({'type': 'LINE_STRIP', 'ns': ns_prefix + '_outline', 'id': 0, 'action': 'ADD',
                        'frame': self.wall.frame_id, 'pose': ident, 'scale': (0.01, 0.0, 0.0), 'color': (1, 1, 1, 1),
                        'points': [tuple(p) for p in c + [c[0]]], 'text': ''})
        if label:
            top = self.wall.to_world((self.wall.u_range[0] + self.wall.u_range[1]) / 2, self.wall.v_range[1] + 0.12, 0.03)
            out.append({'type': 'TEXT_VIEW_FACING', 'ns': ns_prefix + '_text', 'id': 0, 'action': 'ADD',
                        'frame': self.wall.frame_id, 'pose': tuple(top) + ident[3:], 'scale': (0.0, 0.0, 0.1),
                        'color': (1, 1, 1, 1), 'points': [], 'text': 'covered %.1f %%' % self.percent_done()})
        return out


class FeedbackTracker:
    """Infers per-column outcomes from ExecuteCoverage feedback, which carries only counts: ``column`` (current, -1
    between columns), ``columns_done`` and ``columns_failed`` (failed or partial). When ``column`` moves off column c,
    c counts as failed if the failed count rose since c began, else as done if the done count rose."""

    def __init__(self):
        self.cur = -1
        self.start = (0, 0)
        self.done, self.failed = [], []

    def update(self, column, n_done, n_failed):
        events = []
        if self.cur >= 0 and column != self.cur:
            if n_failed > self.start[1]:
                self.failed.append(self.cur)
                events.append((self.cur, 'failed'))
            elif n_done > self.start[0]:
                self.done.append(self.cur)
                events.append((self.cur, 'done'))
        if column >= 0 and column != self.cur:
            self.start = (n_done, n_failed)
        self.cur = column
        return events


# ---- plan columns, replay, files ---------------------------------------------------------------------------------
def load_plan_columns(path):
    """Column polylines from a coverage_output.json: k-th non-connector segment of ``final_lifted_and_expanded`` (else the
    top level) = column k (ASSUMPTION: the manager's column index is the order of the file's strokes). {k: [(x, y, z)]}."""
    with open(path) as f:
        d = json.load(f)
    r = d.get('final_lifted_and_expanded') or d
    poses = (r.get('path') or {}).get('poses') or []
    pts = [(q['position']['x'], q['position']['y'], q['position']['z']) for q in poses]
    cols, k = {}, 0
    for sg in r.get('segments') or []:
        text = ('%s %s' % (sg.get('label', ''), sg.get('mode', ''))).lower()
        if 'connector' in text or 'transit' in text:
            continue
        a = int(sg.get('render_start_index', sg.get('start_index', 0)))
        b = int(sg.get('render_end_index', sg.get('end_index', 0)))
        if 0 <= a <= b < len(pts) and b > a:
            cols[k] = pts[a:b + 1]
        k += 1
    return cols


def read_run_csv(path):
    """run.csv (t,x,y,z,force[,active]) -> list of (t, (x, y, z), force, active or None)."""
    rows = []
    with open(path, newline='') as f:
        for rec in csv.DictReader(f):
            try:
                act = rec.get('active')
                rows.append((float(rec['t']), (float(rec['x']), float(rec['y']), float(rec['z'])), float(rec['force']),
                             None if act in (None, '') else int(float(act))))
            except (KeyError, ValueError):
                continue
    return rows


def replay(grid, rows, require_active=False):
    """Paint a recorded run into ``grid``. require_active: only samples with active == 1 count (spray valve open)."""
    for t, p, f, act in rows:
        if require_active and act == 0:
            grid.break_stroke()
            continue
        grid.add_sample(p, f, t)
    return grid


def render_png(grid, path, title='', dpi=130):
    """Final done / not-done map as a PNG (matplotlib, Agg backend). u horizontal, v vertical."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    cols = [STATUS_RGBA[s][:3] for s in (NOT_DONE, DONE, FAILED)]
    (u0, u1), (v0, v1) = grid.wall.u_range, grid.wall.v_range
    fig, ax = plt.subplots(figsize=(7.5, 7.5 * max(0.3, min(1.2, (v1 - v0) / (u1 - u0))) + 1.2))
    ax.imshow(grid.status, origin='lower', extent=(u0, u0 + grid.nu * grid.cell, v0, v0 + grid.nv * grid.cell),
              cmap=ListedColormap(cols), vmin=0, vmax=2, interpolation='nearest', aspect='equal')
    s = grid.summary()
    ax.set_xlabel('u along the wall [m]')
    ax.set_ylabel('v up the wall [m]')
    ax.set_title((title + '\n' if title else '') + 'covered %.1f %%  (%.2f of %.2f m2; failed %.2f m2)' % (
        s['percent_done'], s['done_m2'], s['wall_m2'], s['failed_m2']), fontsize=10)
    ax.legend(handles=[Patch(color=cols[1], label='done (contact swept)'), Patch(color=cols[0], label='not done'),
                       Patch(color=cols[2], label='failed / partial column')], loc='upper center',
              bbox_to_anchor=(0.5, -0.12), ncol=3, frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path
