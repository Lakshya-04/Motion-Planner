import numpy as np
import pytest

from base_localisation.sim_imu import GRAVITY, ImuModel


def run(model, vx, vy, wz, n=50, dt=0.01):
    out = None
    for i in range(n):
        out = model.step(i * dt, vx, vy, wz)
    return out


def test_ideal_imu_reads_the_yaw_rate_and_gravity():
    gyro, accel = run(ImuModel(), 0.0, 0.0, 0.4)
    assert np.allclose(gyro, [0, 0, 0.4]) and accel[2] == pytest.approx(GRAVITY)
    assert abs(accel[0]) < 1e-9 and abs(accel[1]) < 1e-9


def test_steady_turn_gives_centripetal_acceleration():
    # v = 0.3 m/s forward, w = 0.5 rad/s: a_y = w v = 0.15 m/s^2
    _, accel = run(ImuModel(), 0.3, 0.0, 0.5)
    assert accel[1] == pytest.approx(0.15, abs=1e-6)


def test_speeding_up_gives_forward_acceleration():
    m = ImuModel(alpha=1.0)
    m.step(0.0, 0.0, 0.0, 0.0)
    _, accel = m.step(0.1, 0.05, 0.0, 0.0)
    assert accel[0] == pytest.approx(0.5)


def test_noise_and_bias_default_off_and_seeded_when_on():
    gyro, _ = run(ImuModel(), 0.1, 0, 0.1)
    assert np.allclose(gyro, [0, 0, 0.1])
    kw = dict(gyro_noise=0.01, gyro_bias=0.01, accel_noise=0.05)
    n1 = run(ImuModel(seed=3, **kw), 0.1, 0, 0.1)
    n2 = run(ImuModel(seed=3, **kw), 0.1, 0, 0.1)
    n3 = run(ImuModel(seed=4, **kw), 0.1, 0, 0.1)
    assert np.array_equal(n1[0], n2[0]) and not np.array_equal(n1[0], n3[0])
    assert n1[0][2] != 0.1


def test_bias_is_constant_over_a_run():
    m = ImuModel(gyro_bias=0.05, seed=1)
    g1, _ = m.step(0.0, 0, 0, 0.0)
    g2, _ = m.step(0.01, 0, 0, 0.0)
    assert np.allclose(g1, g2) and g1[2] != 0.0


def test_covariance_has_a_floor_and_follows_the_noise():
    assert ImuModel().covariances() == (1e-6, 1e-6)
    g, a = ImuModel(gyro_noise=0.01, accel_noise=0.1).covariances()
    assert g == pytest.approx(1e-4) and a == pytest.approx(1e-2)
