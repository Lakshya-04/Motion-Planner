"""Ray fan of the simulated 2D lidar (the C++ plugin builds the same fan from the MJCF numeric)."""
import math

# angles in rad, ranges in m; the default is a 270 degree scanner at 0.5 degree steps
LIDAR_DEFAULTS = dict(angle_min=-0.75 * math.pi, angle_max=0.75 * math.pi, angle_increment=math.radians(0.5),
                      range_min=0.05, range_max=10.0, rate_hz=10.0, noise_std=0.0, dropout=0.0, seed=12345)


def ray_count(angle_min, angle_max, angle_increment):
    """Rays in the fan; a thousandth of a step absorbs rounding in the angles."""
    if not angle_increment > 0.0 or angle_max < angle_min:
        raise ValueError('lidar needs angle_increment > 0 and angle_max >= angle_min')
    return int(math.floor((angle_max - angle_min) / angle_increment + 1e-3)) + 1


def ray_angles(angle_min, angle_max, angle_increment):
    return [angle_min + i * angle_increment for i in range(ray_count(angle_min, angle_max, angle_increment))]


def range_to_wall(angle, wall_x):
    """Range along a ray at `angle` (rad, 0 = +x) to the plane x = wall_x; inf when it does not point at it."""
    c = math.cos(angle)
    return wall_x / c if c > 1e-9 and wall_x > 0.0 else math.inf


def lidar_numeric(options):
    """The 9 values of the MJCF numeric `sim_mujoco_lidar`, after checking them."""
    d = dict(LIDAR_DEFAULTS, **{k: v for k, v in options.items() if k in LIDAR_DEFAULTS})
    if not d['range_max'] > d['range_min'] >= 0.0:
        raise ValueError('lidar needs range_max > range_min >= 0')
    if not d['rate_hz'] > 0.0:
        raise ValueError('lidar rate_hz must be positive')
    if d['noise_std'] < 0.0 or not 0.0 <= d['dropout'] <= 1.0:
        raise ValueError('lidar noise_std must be >= 0 and dropout within 0..1')
    if ray_count(d['angle_min'], d['angle_max'], d['angle_increment']) > 100000:
        raise ValueError('lidar has more than 100000 rays')
    return [d['angle_min'], d['angle_max'], d['angle_increment'], d['range_min'], d['range_max'], d['rate_hz'],
            d['noise_std'], d['dropout'], float(int(d['seed']))]
