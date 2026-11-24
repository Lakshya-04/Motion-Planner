"""Host tests for docker/tools/run_report.py on a small synthetic run directory (no big fixtures)."""
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import run_report as rr  # noqa: E402

MANAGER_LOG = """\
[manager]: stand 0: arm base (0.060, 0.010) yaw 1.571 reaches 10 pose(s)
[manager]: ARMSTATE docked_wall_column_0 stand 0: q=[0.20, 0.0, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.700, 0.300, 1.100)
[manager]: segment_exec id=0 label=wall_column_0_approach outcome=completed mode=fsp planner=ompl frame=b wp=[0,0] attempted=1 completed=1 ratio=1 path_fraction=1 reason=none stage=none planning_ms=5 execution_ms=1000 fallback=false mode_switch=true traj_pts=9 traj_s=10
[manager]: segment_exec id=0 label=wall_column_0_contact outcome=completed mode=cartesian planner=ompl frame=b wp=[0,0] attempted=1 completed=1 ratio=1 path_fraction=1 reason=none stage=none planning_ms=5 execution_ms=1000 traj_pts=9 traj_s=20
[manager]: segment_exec id=0 label=wall_column_1_approach outcome=failed_runtime mode=fsp planner=ompl frame=b wp=[0,0] reason=planning_failed traj_s=0
columns_done [0] partial [] failed [1]
"""


def make_run(tmp_path, spike=True, with_npz=True):
    d = tmp_path / "run"
    d.mkdir()
    (d / "manager.log").write_text(MANAGER_LOG)
    wall = dict(origin=[0.98, 0, 0], u=[0, 1, 0], v=[0, 0, 1], u_range=[0.0, 0.4], v_range=[0.0, 1.0])
    (d / "run.csv.wall.json").write_text(json.dumps(wall))
    rows = ["t,x,y,z,force,active,tilt_deg"]
    t = 0.0
    def add(y, z, f):
        nonlocal t
        rows.append("%.3f,0.98,%.3f,%.3f,%.3f,1,0" % (t, y, z, f))
        t += 0.1
    for _ in range(20):                       # free space, 2 s
        add(0.1, 0.5, 0.0)
    for k in range(40):                       # stroke at u = 0.1, 10 N, ends with a spike
        add(0.1, 0.1 + 0.02 * k, 10.0)
    if spike:
        add(0.1, 0.9, 80.0)
    for _ in range(40):                       # a 4 s gap
        add(0.1 + 0.005 * _, 0.9, 0.0)
    for k in range(40):                       # stroke at u = 0.3, 14 N (outside the 1.5 N band)
        add(0.3, 0.9 - 0.02 * k, 14.0)
    for _ in range(10):
        add(0.3, 0.1, 0.0)
    (d / "run.csv").write_text("\n".join(rows) + "\n")
    path = [dict(position=dict(x=0.98, y=0.1 * i, z=z)) for i, z in enumerate([0.0, 1.0] * 3)]
    segs = [dict(label="strip_v_%d" % k, mode="cartesian", start_index=2 * k, end_index=2 * k + 1, tags=[]) for k in range(3)]
    (d / "coverage_output.json").write_text(json.dumps(
        dict(final_lifted_and_expanded=dict(path=dict(poses=path), segments=segs))))
    if with_npz:
        tt = np.arange(0, t, 0.1)
        q = np.zeros((len(tt), 3))
        q[:, 0] = np.where(tt < 6, 0.0, 1.0) * np.minimum(1, (tt - 6).clip(0) / 4) * 1.0
        q[:, 1] = np.sin(tt) * 0.1
        np.savez(d / "run_film.npz", t=tt, q=q)
    return d


def test_report_end_to_end(tmp_path):
    d = make_run(tmp_path)
    rep = rr.build_report(str(d))
    assert rep["columns"] == dict(done=[0], partial=[], failed=[1], source="log")
    assert rep["force"]["peak_n"] == pytest.approx(80.0)
    assert rep["force"]["source"] == "run.csv"
    assert len(rep["per_column"]) == 2
    assert rep["per_column"][0]["u_m"] == pytest.approx(0.1, abs=0.01)
    assert rep["per_column"][1]["peak_n"] == pytest.approx(14.0)
    t = rep["time"]
    assert t["free_space_s"] == pytest.approx(10.0)
    assert t["contact_s"] == pytest.approx(8.1, abs=0.3)
    assert t["idle_s"] == 0.0        # the synthetic log claims more motion than the 15 s recording
    assert rep["path_efficiency"]["gaps"]
    assert rep["redundancy"]["jumps_over_2x_pitch"] == 0
    joined = " | ".join(rep["flags"])
    assert "peak force" in joined and "failed columns: [1]" in joined and "retract" in joined
    assert rep["verdict"].startswith("FLAGGED: 1/2 columns done")


def test_outputs_written_and_cli(tmp_path, capsys):
    d = make_run(tmp_path)
    out = tmp_path / "out"
    assert rr.main([str(d), "--out", str(out)]) == 0
    assert "VERDICT: FLAGGED" in capsys.readouterr().out
    for name in ("report.md", "report.json", "report.png"):
        assert (out / name).stat().st_size > 0
    data = json.loads((out / "report.json").read_text())
    assert data["flags"] and "_grid" not in data
    assert "## Per-column peak force" in (out / "report.md").read_text()


def test_clean_run_and_score_card(tmp_path):
    d = make_run(tmp_path, spike=False, with_npz=False)
    (d / "manager.log").write_text(MANAGER_LOG.replace("columns_done [0] partial [] failed [1]", "columns_done [0, 1] partial [] failed []"))
    (d / "card.json").write_text(json.dumps(dict(force=dict(peak_n=12.0, rms_error_n=2.0, in_band_fraction=0.5))))
    rep = rr.build_report(str(d))
    assert rep["force"]["source"] == "score card" and rep["force"]["peak_n"] == 12.0
    assert rep["columns"]["failed"] == []
    assert not any("peak" in f or "failed" in f or "retract" in f for f in rep["flags"])
    assert rep["path_efficiency"] is None and any("run_film" in n for n in rep["notes"])


def test_partial_directory_does_not_fail(tmp_path):
    d = tmp_path / "empty"
    d.mkdir()
    rep = rr.build_report(str(d))
    assert rep["columns"] is None and rep["notes"]
    assert rr.main([str(d), "--out", str(tmp_path / "o")]) == 0
    assert rr.main([str(tmp_path / "nope")]) == 2


def test_plan_redundancy_mirror_finds_jump_and_double_sweep():
    pts = [(0, 0, 0), (0, 0, 1), (0.1, 0, 1), (0.1, 0, 0), (0.9, 0, 0), (0.9, 0, 1), (0.1, 0.0, 0.5), (0.1, 0.0, 0.6)]
    doc = dict(final_lifted_and_expanded=dict(
        path=dict(poses=[dict(position=dict(x=x, y=y, z=z)) for x, y, z in pts]),
        segments=[dict(label="a", mode="cartesian", start_index=0, end_index=1),
                  dict(label="b", mode="cartesian", start_index=2, end_index=3),
                  dict(label="c", mode="cartesian", start_index=4, end_index=5),
                  dict(label="d", mode="cartesian", start_index=6, end_index=7)]))
    r = rr.plan_redundancy(doc, "/nonexistent", pitch=0.1)
    assert r["jumps_over_2x_pitch"] >= 1
    assert r["double_sweep_cells"] >= 1
