import math

import numpy as np
import pytest

from sim_realism.odom_model import MessageChannel, OdomModel, wrap
from sim_realism.presets import ODOM_PRESETS, OdomParams, resolve

DT = 0.02


def drive(model, speed=0.3, seconds=33.3, yaw_rate=0.0, heading=0.0):
    """Truth: constant body speed (and yaw rate) from the origin. Returns the last reported sample."""
    x = y = 0.0
    yaw = heading
    out = model.step(x, y, yaw, 0.0, (speed, 0.0, yaw_rate))
    for i in range(1, int(round(seconds / DT)) + 1):
        x += speed * math.cos(yaw) * DT
        y += speed * math.sin(yaw) * DT
        yaw = wrap(yaw + yaw_rate * DT)
        out = model.step(x, y, yaw, i * DT, (speed, 0.0, yaw_rate))
    return out, (x, y, yaw)


def many(params, runs, **kw):
    res = []
    for seed in range(runs):
        m = OdomModel(params, seed)
        out, truth = drive(m, **kw)
        res.append((out, truth, m))
    return res


def test_linear_scale_error_is_zero_mean_with_the_configured_sigma():
    runs = many(OdomParams(scale_lin_sigma=0.02), 300, seconds=33.3)
    ratio = np.array([o[0] / t[0] - 1.0 for o, t, _ in runs])
    assert abs(ratio.mean()) < 0.005
    assert ratio.std() == pytest.approx(0.02, rel=0.15)


def test_scale_error_grows_linearly_with_distance():
    p = OdomParams(scale_lin_sigma=0.02)
    short = OdomModel(p, 5)
    (xs, *_), _ = drive(short, seconds=16.6)
    long = OdomModel(p, 5)
    (xl, *_), _ = drive(long, seconds=33.2)
    err_s, err_l = xs - 0.3 * 16.6, xl - 0.3 * 33.2
    assert err_l / err_s == pytest.approx(2.0, rel=0.02)


def test_initial_yaw_bias_gives_heading_drift_of_bias_times_time():
    p = OdomParams(yaw_bias_init_sigma=0.001)
    runs = many(p, 300, seconds=30.0)
    err = np.array([wrap(o[2] - t[2]) for o, t, _ in runs])
    assert abs(err.mean()) < 0.006
    assert err.std() == pytest.approx(0.001 * 30.0, rel=0.15)


def test_bias_random_walk_variance_matches_the_walk():
    p = OdomParams(yaw_bias_walk=0.0005)
    runs = many(p, 400, seconds=30.0)
    err = np.array([wrap(o[2] - t[2]) for o, t, _ in runs])
    expected = 0.0005 * math.sqrt(30.0 ** 3 / 3.0)
    assert abs(err.mean()) < 3 * expected / math.sqrt(400)
    assert err.std() == pytest.approx(expected, rel=0.15)


def test_white_noise_std_follows_sqrt_of_distance():
    p = OdomParams(trans_noise=0.01)

    def err_std(seconds):
        errs = []
        for seed in range(300):
            (x, y, _, *_), (tx, ty, _) = drive(OdomModel(p, seed), seconds=seconds)
            errs.append(x - tx)
        return np.std(errs), np.mean(errs)

    s10, m10 = err_std(33.3)    # 10 m
    s40, _ = err_std(133.3)     # 40 m
    assert abs(m10) < 0.003
    assert s10 == pytest.approx(0.01 * math.sqrt(10.0), rel=0.15)
    assert s40 / s10 == pytest.approx(2.0, rel=0.2)


def test_yaw_scale_error_on_a_turn():
    p = OdomParams(scale_yaw_sigma=0.05)
    m = OdomModel(p, 3)
    (_, _, yaw, *_), (_, _, tyaw) = drive(m, speed=0.0, seconds=math.pi / 2 / 0.5, yaw_rate=0.5)
    assert yaw == pytest.approx(tyaw * (1.0 + m.yaw_err), abs=2e-3)


def test_slip_adds_error_and_counts_events():
    base = OdomParams(scale_lin_sigma=0.0)
    slip = OdomParams(slip_rate_per_m=0.5, slip_duration_s=1.0, slip_scale_sigma=0.3)
    errs = []
    for seed in range(50):
        m = OdomModel(slip, seed)
        (x, *_), (tx, *_) = drive(m, seconds=66.6)
        errs.append(abs(x - tx))
        assert m.slips > 0
    assert np.mean(errs) > 0.05
    (x, *_), (tx, *_) = drive(OdomModel(base, 0), seconds=66.6)
    assert x == pytest.approx(tx, abs=1e-6)


def test_same_seed_same_run_other_seed_other_run():
    p = ODOM_PRESETS['typical']
    a = drive(OdomModel(p, 11))[0][:3]
    b = drive(OdomModel(p, 11))[0][:3]
    c = drive(OdomModel(p, 12))[0][:3]
    assert a == b
    assert a != c


def test_off_is_exact_pass_through():
    m = OdomModel(ODOM_PRESETS['off'], 0)
    for i, (x, y, yaw) in enumerate([(0.0, 0.0, 0.1), (0.123456789, -0.5, 0.2), (1e-9, 3.3, -3.1)]):
        out = m.step(x, y, yaw, i * DT, (0.1, 0.0, 0.2))
        assert out[:3] == (x, y, yaw) and out[3] == (0.1, 0.0, 0.2)


def test_first_sample_starts_at_the_truth_pose():
    m = OdomModel(ODOM_PRESETS['poor'], 1)
    out = m.step(2.0, -1.0, 0.7, 0.0)
    assert out[:3] == (2.0, -1.0, 0.7)


def test_covariance_filled_and_growing():
    m = OdomModel(ODOM_PRESETS['typical'], 2)
    (_, _, _, _, c0, _), _ = drive(m, seconds=1.0)
    (_, _, _, _, c1, tc), _ = drive(OdomModel(ODOM_PRESETS['typical'], 2), seconds=33.3)
    assert c1[0] > c0[0] > 0 and c1[35] > c0[35] > 0 and c1[7] == c1[0]
    assert tc[0] > 0 and tc[35] > 0
    assert c1[14] == pytest.approx(1e-9)


def test_reported_covariance_covers_the_actual_error_scale():
    p = ODOM_PRESETS['typical']
    ratios = []
    for seed in range(100):
        m = OdomModel(p, seed)
        (x, y, yaw, _, cov, _), (tx, ty, tyaw) = drive(m, seconds=33.3)
        ratios.append((wrap(yaw - tyaw) ** 2) / cov[35])
    assert 0.3 < np.mean(ratios) < 3.0     # variance model within a factor of a few


def test_drift_figure():
    m = OdomModel(OdomParams(scale_lin_sigma=0.02), 4)
    _, (tx, ty, _) = drive(m, seconds=33.3)
    pos, yaw, path = m.drift()
    assert path == pytest.approx(10.0, rel=0.01)
    assert pos == pytest.approx(abs(10.0 * m.lin_err), rel=0.01)


def test_latency_is_exactly_n_periods():
    for n in (0, 1, 3):
        ch = MessageChannel(n)
        got = [ch.push(i) for i in range(10)]
        assert got[:n] == [None] * n
        assert got[n:] == list(range(10 - n))


def test_dropout_rate():
    ch = MessageChannel(0, 0.1, seed=1)
    out = [ch.push(i) for i in range(20000)]
    assert sum(o is None for o in out) / 20000 == pytest.approx(0.1, abs=0.01)


def test_channel_is_deterministic_per_seed():
    a = [MessageChannel(1, 0.2, seed=5).push(i) for i in range(5)]
    ch1, ch2 = MessageChannel(1, 0.2, seed=5), MessageChannel(1, 0.2, seed=5)
    assert [ch1.push(i) for i in range(200)] == [ch2.push(i) for i in range(200)]
    assert a[0] is None


def test_presets_resolve_and_are_ordered():
    assert resolve('odom', 'off').is_identity()
    s = [ODOM_PRESETS[k].scale_lin_sigma for k in ('good', 'typical', 'poor')]
    assert s == sorted(s)
    assert resolve('odom', 'typical', {'scale_lin_sigma': 0.1, 'dropout': float('nan'), 'latency_periods': 4}).scale_lin_sigma == 0.1
    assert resolve('odom', 'typical', {'latency_periods': 4}).latency_periods == 4
    assert resolve('odom', 'typical', {'dropout': float('nan')}).dropout == ODOM_PRESETS['typical'].dropout
    with pytest.raises(ValueError):
        resolve('odom', 'bogus')
    with pytest.raises(ValueError):
        resolve('odom', 'good', {'nope': 1.0})


def test_typical_preset_error_budget():
    runs = many(ODOM_PRESETS['typical'], 200, seconds=33.3)
    dist = np.array([abs(o[0] - t[0]) / 10.0 for o, t, _ in runs])
    heading = np.array([abs(math.degrees(wrap(o[2] - t[2]))) for o, t, _ in runs])
    assert 0.005 < np.median(dist) < 0.02            # 1 to 2 percent distance error
    assert 0.5 < np.median(heading) < 4.0            # a few degrees per 10 m
