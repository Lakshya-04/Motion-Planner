import numpy as np
import pytest

from sim_realism.depth_model import apply_depth_model
from sim_realism.presets import DEPTH_PRESETS, DepthParams, resolve

H, W, F = 60, 80, 70.0


def rays():
    v, u = np.mgrid[0:H, 0:W].astype(float)
    d = np.stack([np.ones((H, W)), -(u - W / 2) / F, -(v - H / 2) / F], axis=-1)   # x forward
    return d / np.linalg.norm(d, axis=-1, keepdims=True), d


def plane(depth_x):
    """Frontal plane x = depth_x seen by a pinhole at the origin."""
    _, d = rays()
    return (d * depth_x).astype(np.float32)


def tilted_plane(depth_x, tilt_deg):
    """Plane through (depth_x, 0, 0) whose normal is tilted from the view axis about z."""
    r, _ = rays()
    t = np.radians(tilt_deg)
    n = np.array([np.cos(t), np.sin(t), 0.0])
    s = depth_x * n[0] / (r @ n)
    return (r * s[..., None]).astype(np.float32)


def step_scene(near=2.0, far=3.0):
    """Left half at `near`, right half at `far` along x."""
    a, b = plane(near), plane(far)
    out = b.copy()
    out[:, : W // 2] = a[:, : W // 2]
    return out


def rng_of(xyz):
    return np.linalg.norm(xyz, axis=-1)


def test_noise_is_zero_mean_with_sigma_a_plus_b_r_squared():
    p = DepthParams(sigma_a=0.001, sigma_b=0.002)
    xyz = plane(2.0)
    r0 = rng_of(xyz)
    rng = np.random.default_rng(1)
    diffs = np.stack([rng_of(apply_depth_model(xyz, p, rng)[0]) - r0 for _ in range(40)])
    assert abs(diffs.mean()) < 5e-4
    expected = p.sigma_a + p.sigma_b * r0 ** 2
    assert (diffs.std(axis=0) / expected).mean() == pytest.approx(1.0, rel=0.05)


def test_noise_grows_with_depth():
    p = DepthParams(sigma_a=0.001, sigma_b=0.002)
    rng = np.random.default_rng(2)
    stds = []
    for d in (1.0, 3.0):
        xyz = plane(d)
        r0 = rng_of(xyz)
        diffs = np.stack([rng_of(apply_depth_model(xyz, p, rng)[0]) - r0 for _ in range(30)])
        stds.append(diffs.std())
    assert stds[1] / stds[0] == pytest.approx((0.001 + 0.002 * 9) / (0.001 + 0.002), rel=0.25)


def test_noise_moves_points_along_their_ray():
    p = DepthParams(sigma_a=0.01)
    xyz = plane(2.0)
    new, _ = apply_depth_model(xyz, p, np.random.default_rng(3))
    d0 = xyz / rng_of(xyz)[..., None]
    d1 = new / rng_of(new)[..., None]
    assert np.allclose(d0, d1, atol=1e-5)


def test_dropout_rate_on_a_frontal_plane():
    p = DepthParams(dropout=0.1)
    xyz = plane(2.0)
    rng = np.random.default_rng(4)
    frac = np.mean([np.isnan(apply_depth_model(xyz, p, rng)[0][..., 0]).mean() for _ in range(20)])
    assert frac == pytest.approx(0.1, abs=0.01)


def test_input_nan_stays_nan_and_organisation_is_kept():
    xyz = plane(2.0)
    xyz[10:20, 5:15] = np.nan
    new, _ = apply_depth_model(xyz, DEPTH_PRESETS['typical'], np.random.default_rng(5))
    assert new.shape == xyz.shape and new.dtype == xyz.dtype
    assert np.isnan(new[10:20, 5:15]).all()


def test_dropout_is_higher_on_depth_edges():
    p = DepthParams(edge_dropout=0.5)
    xyz = step_scene()
    rng = np.random.default_rng(6)
    gone = np.mean([np.isnan(apply_depth_model(xyz, p, rng)[0][..., 0]) for _ in range(20)], axis=0)
    edge_cols = [W // 2 - 1, W // 2]
    assert gone[:, edge_cols].mean() == pytest.approx(0.5, abs=0.05)
    assert gone[:, 5:W // 2 - 5].mean() < 0.01


def test_dropout_is_higher_at_grazing_incidence():
    p = DepthParams(grazing_dropout=1.0)
    rng = np.random.default_rng(7)
    frontal = np.isnan(apply_depth_model(plane(2.0), p, rng)[0][..., 0])[2:-2, 2:-2].mean()
    grazing = np.isnan(apply_depth_model(tilted_plane(2.0, 80.0), p, rng)[0][..., 0])[2:-2, 2:-2].mean()
    assert frontal < 0.01
    r, _ = rays()
    t = np.radians(80.0)
    angle = np.degrees(np.arccos(np.abs(r @ np.array([np.cos(t), np.sin(t), 0.0]))))
    expected = np.clip((angle - 60.0) / 25.0, 0, 1)[2:-2, 2:-2].mean()
    assert expected > 0.3
    assert grazing == pytest.approx(expected, abs=0.05)


def test_flying_pixels_lie_between_the_surfaces_on_the_same_ray():
    p = DepthParams(flying_pixel_prob=1.0)
    xyz = step_scene(2.0, 3.0)
    r0 = rng_of(xyz)
    new, stats = apply_depth_model(xyz, p, np.random.default_rng(8))
    r1 = rng_of(new)
    moved = np.abs(r1 - r0) > 1e-4
    assert stats['flying'] > 0 and moved.sum() > 0
    assert moved[:, [W // 2 - 1, W // 2]].all()
    assert not moved[:, 5:W // 2 - 5].any()
    d0, d1 = xyz / r0[..., None], new / r1[..., None]
    assert np.allclose(d0[moved], d1[moved], atol=1e-5)
    col = W // 2 - 1
    assert (r1[:, col] > r0[:, col] - 1e-4).all() and (r1[:, col] < r0[:, col + 1] + 1e-4).all()


def test_outliers_hit_the_configured_fraction_of_pixels():
    p = DepthParams(outlier_prob=0.01, max_range_m=5.0)
    xyz = plane(2.0)
    r0 = rng_of(xyz)
    rng = np.random.default_rng(9)
    frac = np.mean([(np.abs(rng_of(apply_depth_model(xyz, p, rng)[0]) - r0) > 1e-4).mean() for _ in range(30)])
    assert frac == pytest.approx(0.01, abs=0.003)


def test_quantisation_snaps_to_the_step():
    p = DepthParams(quantum_m=0.01)
    new, _ = apply_depth_model(plane(2.0), p, np.random.default_rng(10))
    r = rng_of(new)
    assert np.allclose(r / 0.01, np.round(r / 0.01), atol=1e-3)
    assert np.abs(r - rng_of(plane(2.0))).max() <= 0.005 + 1e-4


def test_range_cuts():
    xyz = np.concatenate([plane(1.0)[:, :40], plane(4.0)[:, 40:]], axis=1)
    new, _ = apply_depth_model(xyz, DepthParams(min_range_m=1.5, max_range_m=3.0), np.random.default_rng(0))
    assert np.isnan(new).all()
    new, _ = apply_depth_model(xyz, DepthParams(max_range_m=3.0), np.random.default_rng(0))
    assert np.isnan(new[:, 40:]).all() and np.isfinite(new[:, :40]).all()


def test_same_seed_same_cloud():
    xyz = step_scene()
    p = DEPTH_PRESETS['typical']
    a = apply_depth_model(xyz, p, np.random.default_rng(3))[0]
    b = apply_depth_model(xyz, p, np.random.default_rng(3))[0]
    c = apply_depth_model(xyz, p, np.random.default_rng(4))[0]
    assert np.array_equal(a, b, equal_nan=True)
    assert not np.array_equal(a, c, equal_nan=True)


def test_off_is_bit_exact():
    xyz = step_scene()
    xyz[3, 3] = np.nan
    new, stats = apply_depth_model(xyz, DEPTH_PRESETS['off'], np.random.default_rng(0))
    assert new.tobytes() == xyz.tobytes() and stats == {}


def test_unorganised_rows_work_without_edge_terms():
    pts = plane(2.0).reshape(1, -1, 3)
    new, _ = apply_depth_model(pts, DEPTH_PRESETS['typical'], np.random.default_rng(0))
    assert new.shape == pts.shape


def test_presets_get_noisier():
    sig = [DEPTH_PRESETS[k].sigma_a + DEPTH_PRESETS[k].sigma_b * 4 for k in ('good', 'typical', 'poor')]
    assert sig == sorted(sig)
    assert resolve('depth', 'typical', {'sigma_a': 0.01}).sigma_a == 0.01
    with pytest.raises(ValueError):
        resolve('depth', 'typical', {'nope': 1.0})
