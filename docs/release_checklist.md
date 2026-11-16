# Release checklist

What to do before tagging a version or making the repository public. Two decisions belong to the
maintainer and are not changed by anything here: the email address in the `@author` tags and the
`package.xml` maintainer entries, and the repository visibility itself.

## Public scan

1. **Tracked-file check.** `python3 docker/tools/check_public_readiness.py` (stdlib only): fails on a
   Markdown reference that does not resolve to a tracked file and on an absolute `/home/<user>` path. Extra
   forbidden words (earlier project or company names) go in an untracked file named by
   `PUBLIC_CHECK_EXTRA_TERMS`, so private names never enter the repository. Its own tests:
   `python -m pytest docker/tools/tests/test_check_public_readiness.py`.
2. **Names.** Search tracked files and the commit history for names of earlier projects, companies or
   people that should not appear (`git log -p -S<term>` for history; `git grep -i <term>` for the tree).
   Commit messages and pull request text are part of the public record.
3. **Paths.** No machine-specific absolute paths, no hostnames, no private URLs in code, launch files,
   docs or test data (the checker covers `/home/<user>`; look for `/mnt`, `/media`, `C:\`, internal domains).
4. **Secrets.** No tokens, keys, passwords or credential files in the tree or history (`git grep -nIi -E
   'token|secret|passw|api[_-]?key|BEGIN .*PRIVATE'` and a history scan such as `gitleaks detect` or
   `trufflehog git file://.`, if you have one installed). Rotate anything found; removing it from the
   tree does not remove it from history.
5. **Generated and local files.** `build/`, `install/`, `log/`, reachability maps, robot assets under `robots/`,
   editor and agent configuration directories are untracked (`git ls-files` shows nothing of them; `.gitignore`).
6. **Large files and data.** No binaries, meshes or datasets with an unclear licence; third-party material is
   listed in [THIRD_PARTY.md](../THIRD_PARTY.md) with its licence and is not vendored.

## Licence

- [LICENSE](../LICENSE) is Apache-2.0 and every `package.xml` says `Apache-2.0`: check they still agree
  (`grep -L Apache-2.0 src/*/*/package.xml` lists exceptions).
- Source files carry **no per-file licence header** today (the Apache license text asks for one in a
  boilerplate notice but does not require it for the grant to apply). Decide whether to add a short header;
  if so do it in one mechanical, reviewed commit.
- [THIRD_PARTY.md](../THIRD_PARTY.md): every dependency and downloaded artefact has a row; licences marked
  unverified are checked or removed. Re-check the GPL-licensed component noted there (build recipes only,
  nothing shipped) if the Docker images are ever published.

## Versions and notes

1. Package versions are in each `package.xml` (all `0.1.0` now); bump them together. Nothing in the
   workspace depends on an exact version.
2. Move the `Unreleased` section of [CHANGELOG.md](../CHANGELOG.md) under the new version and date, and open a
   fresh `Unreleased`.
3. [ROADMAP.md](ROADMAP.md): update "last verified" and check that the status lines still match the code.
4. Tag the release on `main` after the checks below pass.

## Checks

Run `docker/run_all_checks.sh` (the command set in [docker.md](docker.md#run-all-checks)) from a clean checkout, and the Markdown
link check for what changed. State in the release notes which of those were run and which were not
(for example GPU, MuJoCo and Gazebo scripts) and that nothing has been verified on hardware.

## Maintainer decisions (left open)

- Keep or drop the maintainer email in `@author` tags and `package.xml` entries.
- Make the repository public (or not), and when.
