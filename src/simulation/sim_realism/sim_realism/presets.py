"""Parameter sets for the odometry and depth models. Numbers and reasons: docs/sim_realism.md."""
from dataclasses import dataclass, fields, replace

PRESET_NAMES = ('off', 'good', 'typical', 'poor')


@dataclass(frozen=True)
class OdomParams:
    scale_lin_sigma: float = 0.0      # per-run std of the linear speed scale error (fraction)
    scale_yaw_sigma: float = 0.0      # same for the yaw rate
    yaw_bias_init_sigma: float = 0.0  # per-run std of the initial yaw-rate bias, rad/s
    yaw_bias_walk: float = 0.0        # random walk of the bias, rad/s per sqrt(s)
    trans_noise: float = 0.0          # white noise on a body-frame increment, m per sqrt(m travelled)
    yaw_noise: float = 0.0            # white noise on the heading increment, rad per sqrt(m travelled)
    yaw_noise_rot: float = 0.0        # same, rad per sqrt(rad turned)
    slip_rate_per_m: float = 0.0      # slip events per metre driven
    slip_duration_s: float = 1.0
    slip_scale_sigma: float = 0.0     # std of the extra scale error during a slip
    latency_periods: int = 0          # whole truth periods the output is late
    dropout: float = 0.0              # probability that an output message is lost

    def is_identity(self):
        return all(getattr(self, f.name) == 0 for f in fields(self) if f.name != 'slip_duration_s')


@dataclass(frozen=True)
class DepthParams:
    sigma_a: float = 0.0              # range noise std: sigma = a + b r^2, m
    sigma_b: float = 0.0              # 1/m
    dropout: float = 0.0              # base probability that a valid pixel returns nothing
    edge_dropout: float = 0.0         # extra, on depth discontinuities
    grazing_dropout: float = 0.0      # extra, at 85 deg incidence (ramps up from 60 deg)
    flying_pixel_prob: float = 0.0    # at a depth edge: range replaced by one between the two surfaces
    outlier_prob: float = 0.0         # per pixel: range replaced by a random one
    quantum_m: float = 0.0            # range step, 0 = none
    min_range_m: float = 0.0
    max_range_m: float = 0.0          # 0 = no cut
    edge_abs_m: float = 0.05          # discontinuity: neighbour range differs by more than abs + rel * r
    edge_rel: float = 0.02

    def is_identity(self):
        skip = ('edge_abs_m', 'edge_rel')
        return all(getattr(self, f.name) == 0 for f in fields(self) if f.name not in skip)


ODOM_PRESETS = {
    'off': OdomParams(),
    'good': OdomParams(scale_lin_sigma=0.005, scale_yaw_sigma=0.01, yaw_bias_init_sigma=0.00017,
                       yaw_bias_walk=0.00005, trans_noise=0.001, yaw_noise=0.002, yaw_noise_rot=0.004),
    'typical': OdomParams(scale_lin_sigma=0.015, scale_yaw_sigma=0.02, yaw_bias_init_sigma=0.0005,
                          yaw_bias_walk=0.00017, trans_noise=0.003, yaw_noise=0.004, yaw_noise_rot=0.01,
                          slip_rate_per_m=0.05, slip_duration_s=1.0, slip_scale_sigma=0.15,
                          latency_periods=1, dropout=0.005),
    'poor': OdomParams(scale_lin_sigma=0.03, scale_yaw_sigma=0.04, yaw_bias_init_sigma=0.0017,
                       yaw_bias_walk=0.0005, trans_noise=0.01, yaw_noise=0.01, yaw_noise_rot=0.02,
                       slip_rate_per_m=0.2, slip_duration_s=1.5, slip_scale_sigma=0.3,
                       latency_periods=2, dropout=0.03),
}

DEPTH_PRESETS = {
    'off': DepthParams(),
    'good': DepthParams(sigma_a=0.0005, sigma_b=0.0008, dropout=0.002, edge_dropout=0.3, grazing_dropout=0.3,
                        flying_pixel_prob=0.1, outlier_prob=0.0001, quantum_m=0.001, min_range_m=0.15,
                        max_range_m=6.0),
    'typical': DepthParams(sigma_a=0.001, sigma_b=0.002, dropout=0.01, edge_dropout=0.5, grazing_dropout=0.5,
                           flying_pixel_prob=0.3, outlier_prob=0.0005, quantum_m=0.001, min_range_m=0.2,
                           max_range_m=5.0),
    'poor': DepthParams(sigma_a=0.002, sigma_b=0.005, dropout=0.05, edge_dropout=0.8, grazing_dropout=0.8,
                        flying_pixel_prob=0.6, outlier_prob=0.002, quantum_m=0.004, min_range_m=0.3,
                        max_range_m=3.5),
}


def resolve(kind, preset, overrides=None):
    """Preset plus explicit overrides (a dict of field -> value; None and NaN are skipped)."""
    table, cls = (ODOM_PRESETS, OdomParams) if kind == 'odom' else (DEPTH_PRESETS, DepthParams)
    if preset not in table:
        raise ValueError(f'unknown preset {preset!r}, expected one of {PRESET_NAMES}')
    names = {f.name for f in fields(cls)}
    clean = {}
    for key, value in (overrides or {}).items():
        if key not in names:
            raise ValueError(f'unknown {kind} parameter {key!r}')
        if value is None or (isinstance(value, float) and value != value):
            continue
        clean[key] = int(value) if key == 'latency_periods' else float(value)
    return replace(table[preset], **clean)
