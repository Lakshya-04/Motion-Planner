import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import check_public_readiness as chk  # noqa: E402


def make_repo(tmp_path, files):
    subprocess.check_call(["git", "init", "-q", str(tmp_path)])
    for rel, text in files.items():
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    subprocess.check_call(["git", "-C", str(tmp_path), "add", "-A"])
    return str(tmp_path)


def msgs(problems):
    return [m for _, _, m in problems]


def test_clean_repo_passes(tmp_path):
    root = make_repo(tmp_path, {
        "docs/a.md": "see b.md and https://example.com/missing.md\n",
        "docs/b.md": "hello\n",
        "src/x.cpp": "// see docs/a.md and CLAUDE.md\n",
    })
    assert chk.scan(root) == []


def test_dangling_md_reference_fails(tmp_path):
    root = make_repo(tmp_path, {"src/x.cpp": "// see design_notes.md section 3\n", "docs/a.md": "x\n"})
    problems = chk.scan(root)
    assert len(problems) == 1 and "design_notes.md" in problems[0][2]


def test_home_path_fails_but_placeholder_ok(tmp_path):
    root = make_repo(tmp_path, {"a.sh": "cd /home/alice/work\ncd /home/user/ws\n"})
    problems = chk.scan(root)
    assert msgs(problems) == ["absolute home path: /home/alice"]


def test_extra_terms_from_env_file(tmp_path, monkeypatch):
    root = make_repo(tmp_path, {"a.txt": "Contains SecretName here\n", "b.txt": "fine\n"})
    terms = tmp_path.parent / (tmp_path.name + "_terms.txt")
    terms.write_text("# comment\nsecretname\n")
    monkeypatch.setenv("PUBLIC_CHECK_EXTRA_TERMS", str(terms))
    problems = chk.scan(root)
    assert [(r, n) for r, n, _ in problems] == [("a.txt", 1)]


def test_exclude_globs(tmp_path):
    root = make_repo(tmp_path, {"docs/a.md": "see missing.md\n", "src/y.py": "# see gone.md\n"})
    assert len(chk.scan(root)) == 2
    assert chk.scan(root, ["docs/*", "src/*"]) == []
    assert chk.main(["--root", root, "--exclude", "docs/*", "--exclude", "src/*"]) == 0
    assert chk.main(["--root", root]) == 1
