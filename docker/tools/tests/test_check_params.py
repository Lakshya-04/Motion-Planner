import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import check_params as chk  # noqa: E402

CPP = '''
// declare_parameter<int>("commented_out", 1);
void f() {
  a_ = this->declare_parameter<double>("speed_mps", 0.1);
  b_ = declare_parameter<std::vector<int64_t>>(
    "joint_list", std::vector<int64_t>{0});
  c_ = node->declare_parameter("plain", std::string("x"));
  d_ = declare_parameter<bool>(prefix + ".dyn", false);
  declare_parameters<double>("ns", {{"k", 1.0}});
}
'''


def make(tmp_path, cpp=CPP, yaml_text=None, docs="", launch=None):
    tmp_path.mkdir(parents=True, exist_ok=True)
    pkg = tmp_path / "src" / "pkg"
    (pkg / "src").mkdir(parents=True)
    (pkg / "src" / "node.cpp").write_text(cpp)
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs" / ("page" + ".md")).write_text(docs)
    if yaml_text is not None:
        (pkg / "config").mkdir()
        (pkg / "config" / "p.yaml").write_text(yaml_text)
    if launch is not None:
        (pkg / "launch").mkdir()
        (pkg / "launch" / "x.launch.py").write_text(launch)
    return str(tmp_path), {"pkg": "src/pkg"}


def scan(tmp_path, rules=(), **kw):
    root, pk = make(tmp_path, **kw)
    return chk.scan(root, pk, list(rules))


def names(problems):
    return " | ".join(m for _, m in problems)


def test_declared_names_parsed_comments_and_dynamic_skipped(tmp_path):
    root, pk = make(tmp_path)
    got, dyn = chk.declared(os.path.join(root, pk["pkg"]))
    assert set(got) == {"speed_mps", "joint_list", "plain"}
    assert len(dyn) == 2  # prefix + ".dyn" and the declare_parameters call


def test_undocumented_parameters_fail_and_word_match_is_exact(tmp_path):
    problems, notes = scan(tmp_path, docs="`speed_mps` and joint_list_extra only")
    text = names(problems)
    assert "'speed_mps'" not in text            # in docs
    assert "'joint_list'" in text               # only a longer word is in docs
    assert "'plain'" in text
    assert len(notes) == 2


def test_documented_in_docs_yaml_or_launch_passes(tmp_path):
    problems, _ = scan(tmp_path, docs="speed_mps", yaml_text="pkg_node:\n  ros__parameters:\n    joint_list: [0]\n    plain: x\n")
    assert problems == []
    problems, _ = scan(tmp_path / "second", docs="speed_mps joint_list", launch='{"plain": 1}')
    assert problems == []


def test_yaml_key_not_declared_fails_nested_and_dotted(tmp_path):
    y = "n:\n  ros__parameters:\n    speed_mps: 1.0\n    joint_list: [1]\n    plain: x\n    typo_name: 3\n    grp:\n      inner: 1\n"
    problems, _ = scan(tmp_path, yaml_text=y)
    text = names(problems)
    assert "'typo_name'" in text and "'grp.inner'" in text
    assert "'speed_mps'" not in text


def test_yaml_without_ros_parameters_is_ignored(tmp_path):
    problems, _ = scan(tmp_path, docs="speed_mps joint_list plain", yaml_text="planning:\n  ik: moveit_ik\n")
    assert problems == []


def test_allowlist_rules(tmp_path):
    rules = [("undocumented", "pkg", "plain"), ("undeclared", "*", "grp.*")]
    y = "n:\n  ros__parameters:\n    grp:\n      inner: 1\n"
    problems, _ = scan(tmp_path, rules=rules, docs="speed_mps joint_list", yaml_text=y)
    assert problems == []


def test_allowlist_file_format(tmp_path):
    f = tmp_path / "allow.txt"
    f.write_text("# c\nundocumented * foo_*   # why\n\n")
    assert chk.load_allowlist(str(f)) == [("undocumented", "*", "foo_*")]
    f.write_text("bogus x y\n")
    try:
        chk.load_allowlist(str(f))
        assert False
    except SystemExit:
        pass


def test_real_repo_is_clean():
    if not os.path.isdir(os.path.join(chk.ROOT, "docs")):  # run_all_checks.sh mounts only src and docker
        import pytest
        pytest.skip("docs/ not mounted")
    problems, _ = chk.scan()
    assert problems == [], names(problems)
