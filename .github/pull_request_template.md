## What and why

<!-- One paragraph. Link the issue or ROADMAP item. -->

## How it was checked

<!-- Commands you ran and what they printed. Simulator or GPU runs: say which, with the numbers, failures included. -->

- [ ] `docker/run_all_checks.sh --skip-sims` passes (or the failure is explained)
- [ ] New behaviour has a test; new packages have a README
- [ ] Docs, `CHANGELOG.md` and `docs/ROADMAP.md` updated where they changed
- [ ] No machine paths, secrets or private names (`python3 docker/tools/check_public_readiness.py`)
