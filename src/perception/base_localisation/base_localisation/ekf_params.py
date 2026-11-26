"""Parameters of robot_localization's ekf_node for one drive class and a set of sources (no ROS)."""
import os

import yaml

PLATFORMS = ('diff', 'omni', 'ackermann')
NODE = 'ekf_filter_node'
STATE = 15      # x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az


def config_path(platform):
    if platform not in PLATFORMS:
        raise ValueError(f'unknown platform {platform!r}, expected one of {PLATFORMS}')
    name = f'ekf_{platform}.yaml'
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('base_localisation'), 'config', name)
        if os.path.exists(path):
            return path
    except Exception:  # noqa: BLE001  not installed: use the source tree
        pass
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', name)
    if os.path.exists(path):
        return path
    raise FileNotFoundError(name)


def build(platform, imu=True, scan_match=False, odom_topic=None, imu_topic=None, scan_odom_topic=None,
          publish_tf=True, use_sim_time=False, path=None):
    """The ekf_filter_node parameter dict. Sources that are off are dropped, so the filter never waits for them."""
    with open(path or config_path(platform)) as f:
        params = dict(yaml.safe_load(f)[NODE]['ros__parameters'])
    diag = params.pop('process_noise_diag')
    if len(diag) != STATE:
        raise ValueError(f'process_noise_diag needs {STATE} values, got {len(diag)}')
    cov = [0.0] * (STATE * STATE)
    for i, v in enumerate(diag):
        cov[i * STATE + i] = float(v)
    params['process_noise_covariance'] = cov
    for key, topic, on in (('imu0', imu_topic, imu), ('odom1', scan_odom_topic, scan_match)):
        if on and topic:
            params[key] = topic
        elif not on:
            for k in [k for k in params if k.startswith(key)]:
                del params[k]
    if odom_topic:
        params['odom0'] = odom_topic
    params['publish_tf'] = bool(publish_tf)
    params['use_sim_time'] = bool(use_sim_time)
    return params
