#!/usr/bin/env python3
"""Public-readiness scan of the tracked files in this repository (stdlib only).

Fails (exit 1) on:
  1. a ``*.md`` reference (``.gitignore`` patterns are not references) that does not resolve to a tracked file. A reference
     resolves when it names a tracked file relative to the repository root, to
     the referencing file's directory, or (bare names only) to ``docs/``.
     URLs and the names in ALLOWED_MD are ignored.
  2. an absolute ``/home/<user>`` path (placeholders in ALLOWED_HOME_USERS are
     ignored).
  3. any extra forbidden word listed in the optional, untracked file named by
     the environment variable PUBLIC_CHECK_EXTRA_TERMS (one term per line, '#'
     starts a comment, matched case-insensitively). Private names therefore
     never need to be written in the repository itself.

Usage:
  docker/tools/check_public_readiness.py [--root DIR] [--exclude GLOB ...]

``--exclude`` globs (fnmatch, matched against the repo-relative path; ``*``
also crosses ``/``) skip files, e.g. ones owned by work in flight.
"""
import argparse
import fnmatch
import os
import re
import subprocess
import sys

# Markdown names that are legitimately not tracked files (generic mentions,
# files that exist only in a user's checkout).
ALLOWED_MD = {"CLAUDE.md", "SKILL.md", "MEMORY.md"}
ALLOWED_HOME_USERS = {"user", "username", "ubuntu", "ros", "me", "you", "name", "your_name"}

MD_RE = re.compile(r"(?<![A-Za-z0-9_.~/-])((?:[A-Za-z0-9_.-]+/)*[A-Za-z0-9_.-]+\.md)\b")
URL_RE = re.compile(r"https?://\S+")
HOME_RE = re.compile(r"/home/([A-Za-z0-9_.-]+)")


def tracked_files(root):
    out = subprocess.check_output(["git", "-C", root, "ls-files", "-z"])
    return [p for p in out.decode().split("\0") if p]


def read_text(path):
    try:
        with open(path, "rb") as fh:
            data = fh.read()
    except OSError:
        return None
    if b"\0" in data:
        return None
    return data.decode("utf-8", errors="replace")


def md_resolves(ref, rel_file, tracked):
    base = os.path.dirname(rel_file)
    cands = [os.path.normpath(os.path.join(base, ref)), os.path.normpath(ref)]
    if "/" not in ref:
        cands.append(os.path.join("docs", ref))
    return any(c in tracked for c in cands)


def load_terms():
    path = os.environ.get("PUBLIC_CHECK_EXTRA_TERMS")
    if not path:
        return []
    with open(path, encoding="utf-8") as fh:
        lines = [ln.split("#", 1)[0].strip() for ln in fh]
    return [t.lower() for t in lines if t]


# The checker's own tests contain deliberately dangling references and placeholder paths.
SELF_EXCLUDES = ("docker/tools/tests/test_check_public_readiness.py",)


def scan(root, excludes=()):
    excludes = tuple(excludes) + SELF_EXCLUDES
    files = tracked_files(root)
    tracked = set(files)
    terms = load_terms()
    problems = []
    for rel in files:
        if any(fnmatch.fnmatch(rel, g) for g in excludes):
            continue
        text = read_text(os.path.join(root, rel))
        if text is None:
            continue
        for n, line in enumerate(text.splitlines(), 1):
            plain = URL_RE.sub(" ", line)
            for m in ([] if os.path.basename(rel) == ".gitignore" else MD_RE.finditer(plain)):
                ref = m.group(1)
                if os.path.basename(ref) in ALLOWED_MD:
                    continue
                if not md_resolves(ref, rel, tracked):
                    problems.append((rel, n, "dangling md reference: " + ref))
            for m in HOME_RE.finditer(plain):
                if m.group(1) not in ALLOWED_HOME_USERS:
                    problems.append((rel, n, "absolute home path: " + m.group(0)))
            low = line.lower()
            for t in terms:
                if t in low:
                    problems.append((rel, n, "forbidden term (from PUBLIC_CHECK_EXTRA_TERMS)"))
    return problems


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", default=None, help="repository root (default: toplevel of the cwd checkout)")
    ap.add_argument("--exclude", action="append", default=[], metavar="GLOB")
    args = ap.parse_args(argv)
    root = args.root
    if root is None:
        root = subprocess.check_output(["git", "rev-parse", "--show-toplevel"]).decode().strip()
    problems = scan(root, args.exclude)
    for rel, n, msg in problems:
        print("%s:%d: %s" % (rel, n, msg))
    print("public-readiness: %d problem(s)" % len(problems))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
