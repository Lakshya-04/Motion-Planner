"""sim_noise launch argument parsing (no ROS needed)."""
import pytest

from sim_mujoco.sim_noise import parse_sim_noise


def test_empty_means_everything_off():
    assert parse_sim_noise('') == {}


def test_keys_map_to_the_plugin_environment():
    env = parse_sim_noise('ft_noise_density=0.05 ft_delay_ms=5 joint_pos_noise=1e-4')
    assert env == {'SIM_MUJOCO_FT_NOISE_DENSITY': '0.05', 'SIM_MUJOCO_FT_DELAY_MS': '5',
                   'SIM_MUJOCO_JOINT_POS_NOISE': '1e-4'}


@pytest.mark.parametrize('bad', ['nonsense=1', 'ft_delay_ms', 'ft_delay_ms=abc'])
def test_bad_input_is_rejected(bad):
    with pytest.raises(ValueError):
        parse_sim_noise(bad)
