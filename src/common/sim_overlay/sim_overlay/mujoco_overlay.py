"""MuJoCo adapter: draw sim-neutral primitives into ``viewer.user_scn`` (mujoco passive viewer), and the JSON feed.

``draw(user_scn, primitives)`` rebuilds the user geoms every call (sets ``ngeom`` to 0 first) and never exceeds
``user_scn.maxgeom``; it returns the number of primitives dropped for lack of room.
"""
import json
import os
import tempfile

from .overlay import arrow_polyline, box_polyline

LABEL_HEIGHT_Z = 0.0


def save_overlay(path, primitives):
    """Atomic write (temp file in the same directory + rename) so a reader never sees a half file."""
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=d, prefix='.overlay_', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump({'version': 1, 'primitives': primitives}, f)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def load_overlay(path):
    """Primitives from the JSON feed; [] when the file is missing or unreadable (a viewer must not crash)."""
    try:
        with open(path) as f:
            data = json.load(f)
        return list(data.get('primitives', []))
    except (OSError, ValueError, AttributeError):
        return []


def _segments(prim):
    k = prim['kind']
    if k == 'polyline':
        pts = prim['points']
        if prim.get('list'):
            return [(pts[i], pts[i + 1]) for i in range(0, len(pts) - 1, 2)]
        return list(zip(pts[:-1], pts[1:]))
    if k == 'arrow':
        pts = arrow_polyline(prim['start'], prim['end'])
        return list(zip(pts[:-1], pts[1:]))
    if k == 'box':
        pts = box_polyline(prim['center'], prim['size'], prim['quat'])
        return list(zip(pts[:-1], pts[1:]))
    return []


def draw(user_scn, primitives, mujoco=None, min_width=0.004):
    if mujoco is None:
        import mujoco
    import numpy as np
    user_scn.ngeom = 0
    cap = int(user_scn.maxgeom)
    dropped = 0

    def room(n=1):
        return user_scn.ngeom + n <= cap

    def next_geom():
        g = user_scn.geoms[user_scn.ngeom]
        user_scn.ngeom += 1
        return g

    ident = np.eye(3).flatten()
    for prim in primitives:
        rgba = np.array(prim.get('color', (1, 1, 1, 1)), dtype=np.float32)
        k = prim['kind']
        if k in ('polyline', 'arrow', 'box'):
            segs = _segments(prim)
            if not segs or not room(len(segs)):
                dropped += 1
                continue
            w = max(float(prim.get('width', 0.02)), min_width) / 2.0
            for a, b in segs:
                g = next_geom()
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), ident, rgba)
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, np.array(a, float), np.array(b, float))
        elif k == 'tiles':
            quads = prim['quads']
            if not room(len(quads)):
                dropped += 1
                continue
            for q in quads:
                a, b, d = (np.array(q[i], float) for i in (0, 1, 3))
                e1, e2 = b - a, d - a
                l1, l2 = float(np.linalg.norm(e1)), float(np.linalg.norm(e2))
                if l1 < 1e-9 or l2 < 1e-9:
                    continue
                x, y = e1 / l1, e2 / l2
                z = np.cross(x, y)
                z /= (np.linalg.norm(z) or 1.0)
                g = next_geom()
                # thin box: columns of the rotation matrix are the box axes
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.array([l1 / 2, l2 / 2, 0.0015]),
                                    a + e1 / 2 + e2 / 2,
                                    np.column_stack([x, np.cross(z, x), z]).flatten(), rgba)
        elif k in ('sphere', 'label'):
            if not room():
                dropped += 1
                continue
            g = next_geom()
            pos = prim['center'] if k == 'sphere' else prim['position']
            rad = prim['radius'] if k == 'sphere' else max(0.01, prim.get('height', 0.1) / 4.0)
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([rad, 0, 0]), np.array(pos, float),
                                ident, rgba)
            if k == 'label':
                g.label = prim.get('text', '')[:99]
        else:
            dropped += 1
    return dropped
