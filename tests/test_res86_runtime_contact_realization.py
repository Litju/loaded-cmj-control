"""RES-86 runtime effective-contact realization verification tests.

MISSION: RES86_TOUCHDOWN_CONFIGURATION_AND_ABSORPTION_RESOLUTION_001
LINEAR ISSUE: RES-86

The landing gates are stated against the declared nominal contact realization.
These tests bind the *engine-mixed* ``mjContact`` values the solver actually
consumes, including the MuJoCo friction-clamp realization of the declared 0.0
torsional/rolling placeholder.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
for _path in (str(TASK_ROOT), str(TASK_ROOT / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3.contact_realization import (  # noqa: E402
    DECLARED_PLANTAR_CONDIM,
    DECLARED_SLIDING_FRICTION,
    DECLARED_SOLIMP,
    DECLARED_SOLREF,
    ENGINE_MINIMUM_FRICTION,
    contact_realization_fingerprint,
    effective_floor_contacts,
    verify_effective_contact_realization,
)
from loaded_cmj.v3.launch_runtime import settle_standing_stance  # noqa: E402
from loaded_cmj.v3.plant import V3Plant  # noqa: E402


@pytest.fixture(scope="module")
def loaded_stance():
    plant = V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    return plant, data


def test_effective_plantar_contact_realization_matches_declared(loaded_stance):
    plant, data = loaded_stance
    report = verify_effective_contact_realization(plant, data)
    assert report["status"] == "PASS", report["failures"]
    assert report["plantar_rows"]
    for row in report["plantar_rows"]:
        assert row["dim"] == DECLARED_PLANTAR_CONDIM
        assert row["solref"] == list(DECLARED_SOLREF)
        assert row["solimp"] == list(DECLARED_SOLIMP)
        assert row["friction"][0] == DECLARED_SLIDING_FRICTION
        assert row["friction"][1] == DECLARED_SLIDING_FRICTION
        # the declared 0.0 torsional/rolling placeholder is realized at the
        # engine's minimum friction clamp, never as a literal zero
        assert row["friction"][2] == ENGINE_MINIMUM_FRICTION
        assert row["friction"][3] == ENGINE_MINIMUM_FRICTION
        assert row["friction"][4] == ENGINE_MINIMUM_FRICTION
        assert row["includemargin_m"] == 0.0


def test_effective_realization_is_deterministic(loaded_stance):
    plant, data = loaded_stance
    first = contact_realization_fingerprint(plant, data)
    second = contact_realization_fingerprint(plant, data)
    assert first == second
    assert len(first) == 64


def test_condim_mixing_resolves_floor_three_to_plantar_four(loaded_stance):
    plant, data = loaded_stance
    model = plant.model
    floor_gid = int(plant.idx.geom["floor"])
    assert int(model.geom_condim[floor_gid]) == 3
    rows = effective_floor_contacts(plant, data)
    plantar = [row for row in rows if row.is_plantar_floor]
    assert plantar
    for row in plantar:
        assert row.dim == max(int(model.geom_condim[floor_gid]),
                              int(model.geom_condim[int(plant.idx.geom[row.other_geom])]))
        assert [row.geom1_priority, row.geom2_priority] == [0, 0]
        assert [row.geom1_solmix, row.geom2_solmix] == [1.0, 1.0]


def test_verification_fails_closed_without_plantar_contact():
    plant = V3Plant()
    data = plant.make_data()
    report = verify_effective_contact_realization(plant, data)
    assert report["status"] == "FAIL"
    assert "NO_ACTIVE_PLANTAR_CONTACT" in report["failures"]
