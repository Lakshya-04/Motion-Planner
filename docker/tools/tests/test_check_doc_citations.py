import os
EXT = ".m" + "d"   # built at runtime so the repo link check does not read it as a reference
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import check_doc_citations as chk  # noqa: E402

ROOT = chk.ROOT


def repo(tmp_path, doc):
    (tmp_path / "docs").mkdir()
    (tmp_path / "pkg" / "config").mkdir(parents=True)
    (tmp_path / "pkg" / "config" / "a.yaml").write_text("speed_mps: 1.0\nnest:\n  depth_m: 2\n")
    (tmp_path / "pkg" / "other.py").write_text("x = declare_parameter('only_here', 1)\n")
    (tmp_path / "docs" / ("d" + EXT)).write_text(doc)
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    subprocess.run(["git", "add", "-A"], cwd=tmp_path, check=True)
    files = chk.tracked_files(tmp_path)
    return tmp_path / "docs" / ("d" + EXT), files


def run(tmp_path, doc, strict=False):
    d, files = repo(tmp_path, doc)
    return chk.check_doc(d, tmp_path, files, strict)


HEAD = "| a | b |\n|---|---|\n"


def test_good_row(tmp_path):
    e, w, np_, nq = run(tmp_path, HEAD + "| [`a.yaml`](../pkg/config/a.yaml): `speed_mps`, `nest.depth_m` | x |\n")
    assert e == [] and w == [] and np_ >= 1 and nq == 2


def test_missing_path(tmp_path):
    e, *_ = run(tmp_path, HEAD + "| `pkg/config/gone.yaml`: `speed_mps` | x |\n")
    assert any("path not found" in m for m in e)


def test_unknown_param_is_error(tmp_path):
    e, *_ = run(tmp_path, HEAD + "| `a.yaml`: `no_such_param` | x |\n")
    assert any("appears nowhere" in m for m in e)


def test_param_in_other_file_warns_or_fails_strict(tmp_path):
    row = HEAD + "| `a.yaml`: `only_here` | x |\n"
    e, w, *_ = run(tmp_path / "1", row) if (tmp_path / "1").mkdir() is None else (0, 0)
    assert e == [] and len(w) == 1
    e, w, *_ = run(tmp_path / "2", row, strict=True) if (tmp_path / "2").mkdir() is None else (0, 0)
    assert len(e) == 1


def test_link_outside_table_checked(tmp_path):
    e, *_ = run(tmp_path, "See [x](../pkg/nope" + EXT + ").\n")
    assert any("path not found" in m for m in e)


def test_real_docs_clean():
    assert chk.main([]) == 0
