"""Error of an estimated pose track against the truth (numpy only, no ROS)."""
import math

import numpy as np


def wrap(a):
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def align_first(truth, est):
    """Move the estimate by the rigid 2D transform that puts its first pose on the first truth pose.

    Both are (n, 3) arrays of x, y, yaw at matching times. A filter that starts at the odom origin and a
    truth that starts a few millimetres off would otherwise show a constant offset.
    """
    dyaw = float(wrap(truth[0, 2] - est[0, 2]))
    c, s = math.cos(dyaw), math.sin(dyaw)
    rel = est[:, :2] - est[0, :2]
    out = np.empty_like(est)
    out[:, 0] = truth[0, 0] + c * rel[:, 0] - s * rel[:, 1]
    out[:, 1] = truth[0, 1] + s * rel[:, 0] + c * rel[:, 1]
    out[:, 2] = wrap(est[:, 2] + dyaw)
    return out


def resample(t_ref, t, values):
    """Nearest-sample values of a track (t ascending, values (n, k)) at the times t_ref."""
    i = np.clip(np.searchsorted(t, t_ref), 1, len(t) - 1)
    pick = np.where(np.abs(t[i - 1] - t_ref) <= np.abs(t[i] - t_ref), i - 1, i)
    return values[pick]


def score(truth_t, truth, est_t, est):
    """Position and heading error of est against truth, after the first-sample alignment.

    Returns a dict: path_m (truth path length), final_m, rms_m, max_m, final_yaw_deg, rms_yaw_deg.
    """
    truth, est = np.asarray(truth, float), np.asarray(est, float)
    sampled = align_first(truth, resample(truth_t, np.asarray(est_t, float), est))
    err = np.hypot(*(sampled[:, :2] - truth[:, :2]).T)
    yaw_err = wrap(sampled[:, 2] - truth[:, 2])
    path = float(np.sum(np.hypot(*np.diff(truth[:, :2], axis=0).T)))
    return {'path_m': path, 'final_m': float(err[-1]), 'rms_m': float(np.sqrt(np.mean(err ** 2))),
            'max_m': float(err.max()), 'final_yaw_deg': float(np.degrees(yaw_err[-1])),
            'rms_yaw_deg': float(np.degrees(np.sqrt(np.mean(yaw_err ** 2))))}
