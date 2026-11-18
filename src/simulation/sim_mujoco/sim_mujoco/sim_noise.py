"""Opt-in sensor non-idealities of the MuJoCo plugin: launch argument -> environment variables."""

NOISE_KEYS = {'ft_noise_density': 'SIM_MUJOCO_FT_NOISE_DENSITY', 'ft_bias_walk': 'SIM_MUJOCO_FT_BIAS_WALK',
              'ft_lowpass_hz': 'SIM_MUJOCO_FT_LOWPASS_HZ', 'ft_delay_ms': 'SIM_MUJOCO_FT_DELAY_MS',
              'ft_quantum': 'SIM_MUJOCO_FT_QUANTUM', 'joint_pos_noise': 'SIM_MUJOCO_JOINT_POS_NOISE',
              'joint_vel_noise': 'SIM_MUJOCO_JOINT_VEL_NOISE', 'control_delay_ms': 'SIM_MUJOCO_CONTROL_DELAY_MS',
              'seed': 'SIM_MUJOCO_NOISE_SEED'}


def parse_sim_noise(text):
    """'ft_noise_density=0.05 ft_delay_ms=5' -> {environment variable: value} for the plugin; '' = all off."""
    out = {}
    for item in text.replace(",", " ").split():
        key, sep, value = item.partition('=')
        if not sep or key not in NOISE_KEYS:
            raise ValueError(f'sim_noise: unknown or malformed "{item}"; keys: {", ".join(NOISE_KEYS)}')
        float(value)
        out[NOISE_KEYS[key]] = value
    return out
