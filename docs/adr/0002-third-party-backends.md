# 0002: Third-party backends are consumed upstream, unpatched

- Status: accepted
- Date: 2026-10-30

## Decision

- VAMP: upstream `KavrakiLab/vamp` (Apache-2.0) fetched at a pinned commit by a vendor
  package; robot headers generated with upstream FOAM and Cricket (MIT).
- cuRobo v2: upstream `NVlabs/curobo` (Apache-2.0), driven from a Python bridge node.
- Other teams' wrappers may inform design, but their code is not copied in; our layers
  are written from the upstream sources.

## Consequences

Updates are pin bumps; no fork maintenance; licensing stays clean for a public repo.
