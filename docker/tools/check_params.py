#!/usr/bin/env python3
"""Keep the parameters of the manager, reachability, stand_navigator, coverage_planner and wall_detector nodes and their documentation in step.

Two checks, per package:

 1. Every parameter the package's C++ or Python declares (`declare_parameter<T>("name", ...)`, `self.declare_parameter('name', ...)`) must show up as a
    word in the package's config yaml, launch files or README, or in docs/ (any *.md). A parameter
    nobody can find a description of is a parameter nobody can use.
 2. Every key under `ros__parameters` in the package's config yaml must be declared by the package's C++.
    A key the node never declares is silently ignored by ROS 2, which is how a typo or a removed
    parameter stays in a config for months.

Names that are not string literals (`declare_parameter(prefix + ".x", ...)`, `declare_parameters(...)`)
cannot be resolved statically. They are listed as notes, never as failures; a literal prefix, when the
expression starts with one, is kept in the note.

Intentional exceptions go in docker/tools/check_params_allowlist.txt (see that file for the format).

    python3 docker/tools/check_params.py            # exit 1 on any problem
    python3 docker/tools/check_params.py --notes    # also print the dynamic names
"""
import argparse
import fnmatch
import os
import re
import sys

try:
    import yaml
except ImportError:  # CI installs pyyaml; fall back to a clear message rather than a traceback
    sys.exit("check_params.py needs pyyaml (pip install pyyaml)")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PACKAGES = {
    "manipulation_manager": "src/manipulation/manipulation_manager",
    "reachability": "src/manipulation/reachability",
    "stand_navigator": "src/manipulation/stand_navigator",
    "coverage_planner": "src/perception/coverage_planner",
    "wall_detector": "src/perception/wall_detector",
}
CODE_DIRS = ("src", "include", "apps")
CODE_EXT = (".cpp", ".hpp", ".h", ".cc")
PY_DIRS = ("scripts", "src")  # python nodes: scripts/, src/ and the package's own module dir (<pkg>/<pkg>/)
PY_EXT = (".py",)
ALLOWLIST = os.path.join(os.path.dirname(__file__), "check_params_allowlist.txt")

# declare_parameter / declare_parameter<T> / declare_parameter<std::vector<double>>(  first-argument...
DECL = re.compile(r"\bdeclare_parameters?\s*(?:<(?:[^<>]|<[^<>]*>)*>)?\s*\(\s*([^,)]*)", re.S)
LITERAL = re.compile(r'^\s*["\']([^"\']+)["\']\s*$')


def read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def walk(base, exts):
    for d, _, files in os.walk(base):
        for n in sorted(files):
            if n.endswith(exts):
                yield os.path.join(d, n)


def strip_comments(text, python=False):
    if python:
        return re.sub(r"#[^\n]*", "", text)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def declared(pkg_dir):
    """-> (set of literal names, list of (file, expr) for names that are not plain literals)."""
    names, dynamic = {}, []
    sources = [(sub, CODE_EXT) for sub in CODE_DIRS] + [(sub, PY_EXT) for sub in PY_DIRS + (os.path.basename(pkg_dir),)]
    for sub, exts in sources:
        for path in walk(os.path.join(pkg_dir, sub), exts):
            text = strip_comments(read(path), path.endswith(PY_EXT))
            for m in DECL.finditer(text):
                arg = m.group(1).strip()
                lit = LITERAL.match(arg)
                rel = os.path.relpath(path, ROOT)
                if lit and "declare_parameters" not in m.group(0).split("(")[0]:
                    names.setdefault(lit.group(1), rel)
                else:
                    dynamic.append((rel, " ".join(arg.split())))
    return names, dynamic


def yaml_keys(path):
    """Flattened (dotted) ros__parameters keys of one yaml, with the node section each came from."""
    try:
        doc = yaml.safe_load(read(path)) or {}
    except yaml.YAMLError as e:
        return None, str(e)
    out = []

    def flat(prefix, d):
        for k, v in d.items():
            key = f"{prefix}.{k}" if prefix else str(k)
            if isinstance(v, dict) and v:
                flat(key, v)
            else:
                out.append(key)

    def find(d):
        if not isinstance(d, dict):
            return
        for k, v in d.items():
            if k == "ros__parameters" and isinstance(v, dict):
                flat("", v)
            else:
                find(v)

    find(doc)
    return out, None


def load_allowlist(path=ALLOWLIST):
    """Lines: `undocumented <pkg> <glob>` or `undeclared <pkg> <glob>`; `#` starts a comment."""
    rules = []
    if os.path.exists(path):
        for n, line in enumerate(read(path).splitlines(), 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 3 or parts[0] not in ("undocumented", "undeclared"):
                raise SystemExit(f"{path}:{n}: expected 'undocumented|undeclared <package> <name-glob>'")
            rules.append(tuple(parts))
    return rules


def allowed(rules, kind, pkg, name):
    return any(k == kind and p in (pkg, "*") and fnmatch.fnmatch(name, g) for k, p, g in rules)


def doc_corpus(root, pkg_dir):
    """Text a parameter may be described in: docs/, the package README, config yaml and launch files."""
    chunks = []
    for path in walk(os.path.join(root, "docs"), (".md",)):
        chunks.append(read(path))
    for sub, exts in (("config", (".yaml", ".yml")), ("launch", (".py", ".yaml", ".xml"))):
        for path in walk(os.path.join(pkg_dir, sub), exts):
            chunks.append(read(path))
    readme = os.path.join(pkg_dir, "README.md")
    if os.path.exists(readme):
        chunks.append(read(readme))
    return "\n".join(chunks)


def mentioned(name, corpus):
    return re.search(r"(?<![A-Za-z0-9_])" + re.escape(name) + r"(?![A-Za-z0-9_])", corpus) is not None


def scan(root=ROOT, packages=None, rules=None):
    """-> (problems, notes); problems are (package, message)."""
    packages = packages or PACKAGES
    rules = load_allowlist() if rules is None else rules
    problems, notes = [], []
    for pkg, rel in packages.items():
        pkg_dir = os.path.join(root, rel)
        names, dynamic = declared(pkg_dir)
        corpus = doc_corpus(root, pkg_dir)
        for name, src in sorted(names.items()):
            if not mentioned(name, corpus) and not allowed(rules, "undocumented", pkg, name):
                problems.append((pkg, f"parameter '{name}' ({src}) is not in any config yaml, launch file, "
                                      f"package README or docs/ page"))
        for path in walk(os.path.join(pkg_dir, "config"), (".yaml", ".yml")):
            keys, err = yaml_keys(path)
            r = os.path.relpath(path, root)
            if keys is None:
                problems.append((pkg, f"{r}: unparsable yaml ({err})"))
                continue
            for key in keys:
                if key not in names and not allowed(rules, "undeclared", pkg, key):
                    problems.append((pkg, f"{r}: key '{key}' under ros__parameters is not declared by the node"))
        for rel_src, expr in dynamic:
            notes.append(f"{pkg}: {rel_src}: non-literal parameter name `{expr}` not checked")
    return problems, notes


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--notes", action="store_true", help="print the names that could not be resolved")
    args = ap.parse_args(argv)
    problems, notes = scan()
    if args.notes:
        for n in notes:
            print("note:", n)
    for pkg, msg in problems:
        print(f"check_params: [{pkg}] {msg}")
    if problems:
        print(f"check_params: {len(problems)} problem(s); fix the docs/config or add a commented "
              f"exception to docker/tools/check_params_allowlist.txt")
        return 1
    print(f"check_params: ok ({len(PACKAGES)} packages, {len(notes)} dynamic names not checked)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
