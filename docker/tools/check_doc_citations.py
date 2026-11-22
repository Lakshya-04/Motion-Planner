#!/usr/bin/env python3
"""Check that the code citations in a doc table row are real.

For every Markdown table row of the given docs it extracts
  * file paths: link targets and backticked tokens that look like a path,
    resolved against the doc, then the repo root, then by unique file name;
  * parameter names: backticked identifiers (a dotted name is checked by its
    last segment).
A path must exist (links and paths outside tables are checked too). A parameter name must appear as a whole word in one of the
files the row cites, or else somewhere in tracked non-doc files; a name found
only outside the row's files is reported as a warning (use --strict to fail).
Stdlib only. Usage: check_doc_citations.py [--strict] [doc.md ...]
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DOCS = ["docs/hardware_readiness.md", "docs/manager_parameters.md"]
EXTS = ("yaml", "yml", "py", "cpp", "hpp", "h", "md", "launch.py", "txt", "xml",
        "urdf", "xacro", "srdf", "json", "sh", "cmake")
TICK = re.compile(r"`([^`\n]+)`")
LINK = re.compile(r"\]\(([^)#\s]+)(?:#[^)]*)?\)")
IDENT = re.compile(r"^[a-z_][a-z0-9_]*(\.[a-z_][a-z0-9_]*)*$")
# Backticked words in prose that are not parameter names.
NOT_PARAM = {"true", "false", "device", "mock", "none", "null", "kunsupported"}


def tracked_files(root):
    out = subprocess.run(["git", "ls-files"], cwd=root, capture_output=True,
                         text=True, check=True).stdout.split("\n")
    return [p for p in out if p]


def is_path(tok):
    return (re.search(r"\.(" + "|".join(EXTS) + r")$", tok) is not None
            and " " not in tok and ":" not in tok and not tok.startswith("~"))


def resolve(tok, doc, files, root):
    """Return a repo-relative path or None."""
    for base in (doc.parent, root):
        p = (base / tok).resolve()
        if p.is_file():
            return str(p.relative_to(root))
    tok = tok.lstrip("./") if tok.startswith("./") else tok
    hits = [f for f in files if f.endswith("/" + tok) or f == tok]
    if len(hits) >= 1 and ("/" in tok or len(hits) == 1):
        return hits[0]
    return None


def read(root, rel, cache):
    if rel not in cache:
        try:
            cache[rel] = (root / rel).read_text(errors="replace")
        except OSError:
            cache[rel] = ""
    return cache[rel]


def has_word(text, word):
    return re.search(r"(?<![A-Za-z0-9_])" + re.escape(word) + r"(?![A-Za-z0-9_])",
                     text) is not None


def check_doc(doc, root, files, strict=False):
    """Return (errors, warnings, n_paths, n_params) for one doc."""
    errors, warnings = [], []
    cache = {}
    code_files = [f for f in files if not f.endswith(".md")
                  and not f.endswith((".png", ".jpg", ".stl", ".dae", ".pdf"))]
    corpus = None
    pkg_names = {part for f in files for part in f.split("/")[:-1]}
    n_paths = n_params = 0
    rel_doc = doc.relative_to(root) if doc.is_absolute() else doc
    for no, line in enumerate(doc.read_text().splitlines(), 1):
        in_table = line.startswith("|")
        if in_table and re.match(r"^\|[-| :]+\|?$", line):
            continue
        cited, params = [], []
        toks = [(t, "tick") for t in TICK.findall(line)]
        toks += [(t, "link") for t in LINK.findall(line)]
        for tok, kind in toks:
            if kind == "link" and tok.startswith(("http:", "https:", "mailto:")):
                continue
            if is_path(tok) or (kind == "link" and tok.endswith("/")) or \
                    (kind == "tick" and tok.endswith("/") and "/" in tok[:-1]):
                n_paths += 1
                if tok.endswith("/"):
                    t = tok.rstrip("/")
                    ok = any((b / tok).is_dir() for b in (doc.parent, root)) or \
                        any(f.startswith(t + "/") or ("/" + t + "/") in f for f in files)
                    if not ok:
                        errors.append(f"{rel_doc}:{no}: directory not found: {tok}")
                    continue
                r = resolve(tok, doc, files, root)
                if r is None and kind == "tick":
                    # a link label: the link target in the same row resolves it
                    for t2 in LINK.findall(line):
                        if t2.endswith("/" + tok) or t2 == tok:
                            r = resolve(t2, doc, files, root)
                if r is None:
                    errors.append(f"{rel_doc}:{no}: path not found: {tok}")
                else:
                    cited.append(r)
            elif in_table and kind == "tick" and IDENT.match(tok) and tok not in NOT_PARAM:
                params.append(tok)
        for tok in params:
            n_params += 1
            segs = tok.split(".")
            name = segs[-1]
            if name in pkg_names:
                continue  # a package or directory name, not a parameter
            if any(all(has_word(read(root, c, cache), g) for g in segs) for c in cited):
                continue
            if corpus is None:
                corpus = "\n".join(read(root, f, cache) for f in code_files)
            if has_word(corpus, name):
                if not any(not c.endswith(".md") for c in cited):
                    continue
                msg = f"{rel_doc}:{no}: `{tok}` not in the row's files, only elsewhere in the repo"
                (errors if strict else warnings).append(msg)
            else:
                errors.append(f"{rel_doc}:{no}: `{tok}` appears nowhere in the repo")
    return errors, warnings, n_paths, n_params


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    strict = "--strict" in argv
    docs = [a for a in argv if not a.startswith("--")] or DEFAULT_DOCS
    files = tracked_files(ROOT)
    bad = 0
    for d in docs:
        p = Path(d)
        p = p if p.is_absolute() else ROOT / p
        e, w, np_, nq = check_doc(p, ROOT, files, strict)
        for m in w:
            print("warning:", m)
        for m in e:
            print("error:", m)
        print(f"{d}: {np_} paths, {nq} parameter names, {len(e)} errors, {len(w)} warnings")
        bad += len(e)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
