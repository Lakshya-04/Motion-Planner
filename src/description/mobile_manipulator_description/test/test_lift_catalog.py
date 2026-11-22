"""docs/lift_catalog.md and the profiles agree: every listed profile exists with the listed stroke and stages.

The table is the source of the expected values; the profile yaml is the source of the model. Plausibility
bounds catch a unit slip (mm for m). Rendering each profile is covered by test_lifts.py (all profiles).
"""
import pathlib
import re

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
CATALOG = PKG.parents[2] / 'docs' / 'lift_catalog.md'
PROFILES = PKG / 'config' / 'profiles'
STROKE_RANGE_M = (0.2, 2.0)
MAX_STAGES = 3


def _rows():
    rows = []
    for line in CATALOG.read_text().splitlines():
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        if len(cells) < 8 or not line.startswith('| ') or cells[0] in ('Robot', '---'):
            continue
        profile = re.match(r'`([a-z0-9_]+)`', cells[1])
        if not profile:
            continue
        stroke = re.match(r'([0-9.]+) m', cells[2])
        stages = re.match(r'(\d+)\b', cells[3])  # the cell may say "not published (2 assumed)": no leading digit
        rows.append((profile.group(1), float(stroke.group(1)), int(stages.group(1)) if stages else None,
                     cells[-1], cells[-2]))
    return rows


ROWS = _rows()


def test_catalog_lists_every_candidate_that_has_a_source():
    assert len(ROWS) >= 7
    names = [r[0] for r in ROWS]
    assert len(set(names)) == len(names)
    assert {'fr3_tiago_style_torso', 'generic_6dof_fetch_style_torso'} <= set(names)  # refined, not duplicated


@pytest.mark.parametrize('profile,stroke,stages,status,source', ROWS, ids=[r[0] for r in ROWS])
def test_catalog_row_matches_its_profile(profile, stroke, stages, status, source):
    path = PROFILES / f'{profile}.yaml'
    assert path.exists(), f'{profile} is in the catalog but has no profile'
    lift = yaml.safe_load(path.read_text())['lift']
    assert lift['type'] == 'telescopic'
    assert lift['stroke_m'] == pytest.approx(stroke)
    if stages is not None:
        assert lift['stages'] == stages
    assert STROKE_RANGE_M[0] <= lift['stroke_m'] <= STROKE_RANGE_M[1]
    assert 0 <= lift['stages'] <= MAX_STAGES
    assert 0 < lift['max_velocity_mps'] <= 0.5
    assert lift['mount_height_m'] > 0
    assert re.search(r'\]\(https://[^)]+\)', source), 'every row cites a source URL'
    assert status, 'every row has a status'


def test_every_catalog_profile_is_unique_in_the_profile_directory():
    """A catalog entry must not duplicate another profile (same arm, base and stroke)."""
    seen = {}
    for profile, *_ in ROWS:
        data = yaml.safe_load((PROFILES / f'{profile}.yaml').read_text())
        key = (data['robot']['xacro_args']['arm'], data['robot']['xacro_args']['base'], data['lift']['stroke_m'])
        assert key not in seen, f'{profile} duplicates {seen[key]}'
        seen[key] = profile
