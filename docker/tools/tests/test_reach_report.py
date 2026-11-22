"""Host tests for docker/tools/reach_report.py on an analytic sphere-shell map (no big files)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import reach_report as rr  # noqa: E402

REACH, D, Z0, STROKE = 1.3, 0.72, 1.17, 1.5
HALF = math.sqrt(REACH ** 2 - D ** 2)   # 1.0825 m


@pytest.fixture(scope="module")
def shell():
    return rr.synthetic_shell_map(reach=REACH, z_min=-1.6, z_max=1.6)


def report(m, **kw):
    args = dict(patch_z=(0.6, 2.0), patch_y=(0.0, 0.0), clearance=0.42, arm_offset=0.3,
                stroke=STROKE, arm_base_z0=Z0, reach=REACH)
    args.update(kw)
    return rr.build_report(m, **args)


def test_vertical_half_span_matches_analytic(shell):
    rep = report(shell)
    e = rep["fractions"]["1.0"]["extent"]["lift_min"]
    assert e["z_low"] == pytest.approx(Z0 - HALF, abs=0.06)
    assert e["z_high"] == pytest.approx(Z0 + HALF, abs=0.06)
    # fractions shrink the span: sqrt((f R)^2 - D^2)
    e7 = rep["fractions"]["0.7"]["extent"]["lift_min"]
    half7 = math.sqrt((0.7 * REACH) ** 2 - D ** 2)
    assert e7["z_high"] - e7["z_low"] == pytest.approx(2 * half7, abs=0.12)


def test_lift_max_shifts_span_by_stroke(shell):
    ext = report(shell, patch_z=(0.0, 3.0))["fractions"]["1.0"]["extent"]
    assert ext["lift_max"]["z_low"] - ext["lift_min"]["z_low"] == pytest.approx(STROKE, abs=0.06)


def test_fraction_monotone(shell):
    rep = report(shell, patch_y=(-0.4, 0.4))
    u = [rep["fractions"][k]["union_fraction"] for k in ("0.7", "0.85", "1.0")]
    assert u[0] <= u[1] <= u[2]
    for q in rep["fractions"]["0.7"]["per_lift_fraction"]:
        assert (rep["fractions"]["0.7"]["per_lift_fraction"][q]
                <= rep["fractions"]["1.0"]["per_lift_fraction"][q] + 1e-12)


def test_min_lift_count_and_cover(shell):
    rep = report(shell, patch_z=(0.2, 2.8))      # 2.6 m tall: needs more than one 2.16 m span
    d = rep["fractions"]["1.0"]
    assert d["never_reachable_cells"] == 0
    assert d["min_lifts"] == 2
    # the chosen lifts really cover every cell
    grids = d["_grids"]
    lifts = list(rep["_lifts"])
    idx = [min(range(len(lifts)), key=lambda i: abs(lifts[i] - q)) for q in d["lift_heights"]]
    assert grids[idx].any(axis=0).all()
    # a patch inside one span needs a single lift
    assert report(shell, patch_z=(0.6, 1.6))["fractions"]["1.0"]["min_lifts"] == 1


def test_never_reachable_flag(shell):
    # wall far beyond reach: nothing reachable, everything flagged
    rep = report(shell, clearance=1.5, arm_offset=0.3)
    d = rep["fractions"]["1.0"]
    assert d["never_reachable_cells"] == len(d["z"])
    assert d["lift_heights"] == [] and d["min_lifts"] == 0
    # patch above the highest reachable height flags exactly the unreachable rows
    rep = report(shell, patch_z=(3.0, 4.5))
    d = rep["fractions"]["1.0"]
    top = Z0 + STROKE + HALF
    assert d["never_reachable_z_runs"][-1][1] == pytest.approx(4.5, abs=0.05)
    assert d["never_reachable_z_runs"][0][0] >= top - 0.06


def test_lateral_offset_narrows_span(shell):
    centre = report(shell, patch_y=(0.0, 0.0))["fractions"]["1.0"]["extent"]["lift_min"]
    off = report(shell, patch_y=(0.6, 0.6), arm_y=0.0)["fractions"]["1.0"]["extent"]["lift_min"]
    assert (off["z_high"] - off["z_low"]) < (centre["z_high"] - centre["z_low"])


def test_map_roundtrip_and_cli(tmp_path, shell, capsys):
    path = str(tmp_path / "shell.rm4d")
    rr.save_map(path, shell, "fp", arm_dof=6, redundancy="none")
    m = rr.load_map(path)
    assert m.shape == shell.shape and (m.cells == shell.cells).all()
    assert m.describe() == "6 joints, redundancy: none"
    assert m.estimate_reach() == pytest.approx(REACH, abs=0.06)
    png, js = str(tmp_path / "r.png"), str(tmp_path / "r.json")
    rc = rr.main(["--map", path, "--stroke", "1.5", "--arm-base-z0", "1.17", "--patch-z", "0.6", "2.0",
                  "--patch-y", "-0.3", "0.3", "--clearance", "0.42", "--arm-offset", "0.3",
                  "--reach", "1.3", "--png", png, "--json", js])
    out = capsys.readouterr().out
    assert rc == 0 and "REACH REPORT" in out and "Heat strip" in out
    assert os.path.getsize(png) > 1000 and os.path.getsize(js) > 100


def test_bad_file(tmp_path):
    p = tmp_path / "x.rm4d"
    p.write_bytes(b"not a map")
    with pytest.raises(ValueError):
        rr.load_map(str(p))


def test_narrow_z_window_warns_and_clips():
    narrow = rr.synthetic_shell_map(reach=REACH)   # default window -1.0..1.8 is below -reach
    rep = report(narrow)
    assert rep["warnings"]
    assert rep["fractions"]["1.0"]["extent"]["lift_min"]["z_low"] > Z0 - HALF + 0.05
