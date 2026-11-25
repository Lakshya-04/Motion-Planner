"""Optional overlay for the passive MuJoCo viewer (scripts/viewer.py).

Gated on the env variable ``SIM_MUJOCO_OVERLAY_JSON`` (unset or empty = no change to the viewer). The file is the
sim_overlay JSON feed written by ``sim_overlay/scripts/overlay_dump.py`` (``{"version": 1, "primitives": [...]}``);
``OverlayHook.update(user_scn)`` redraws ``viewer.user_scn`` only when the file's mtime changed.
"""
import os

ENV_VAR = 'SIM_MUJOCO_OVERLAY_JSON'


class OverlayHook:
    def __init__(self, path, mujoco, draw=None, load=None):
        if draw is None or load is None:
            from sim_overlay.mujoco_overlay import draw as _draw, load_overlay as _load
            draw, load = draw or _draw, load or _load
        self.path, self.mujoco, self._draw, self._load = path, mujoco, draw, load
        self.last_mtime = None

    @classmethod
    def from_env(cls, mujoco, environ=None):
        """The hook, or None when the env variable is unset/empty or sim_overlay is not importable."""
        path = (os.environ if environ is None else environ).get(ENV_VAR, '')
        if not path:
            return None
        try:
            return cls(path, mujoco)
        except ImportError as e:
            print(f'viewer: {ENV_VAR} set but sim_overlay is not importable ({e}); overlay off')
            return None

    def update(self, user_scn):
        """Redraw when the file changed; returns True if it redrew. Never raises (a viewer must not die on a bad feed)."""
        try:
            mt = os.stat(self.path).st_mtime_ns
        except OSError:
            mt = None
        if mt == self.last_mtime:
            return False
        self.last_mtime = mt
        try:
            self._draw(user_scn, self._load(self.path), self.mujoco)
        except Exception as e:
            print(f'viewer: overlay draw failed: {e}')
            return False
        return True
