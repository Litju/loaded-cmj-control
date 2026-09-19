"""RES-86 declared contact/timestep sensitivity audit tests.

MISSION: RES86_TOE_PHASE_FEASIBILITY_AND_RESOLUTION_001
LINEAR ISSUE: RES-86

These tests bind the audit-only model-copy construction (the sealed Plant XML is
never mutated) and the causal discrimination result: under the declared solref
stiffening the same uncontrolled collapse stays inside the 10 mm penetration
ceiling that the nominal provisional contact violates.
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

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3.plant import model_xml  # noqa: E402
from tools.res86.landing_contact_sensitivity import (  # noqa: E402
    NOMINAL,
    SENSITIVITY_LATTICE,
    build_realization,
)
from tools.res86.landing_feasibility_oracle import (  # noqa: E402
    LandingFeasibilityOracle,
    capture_canonical_branch,
)


@pytest.fixture(scope="session")
def branch():
    return capture_canonical_branch()


@pytest.fixture(scope="session")
def nominal_plant():
    return build_realization(NOMINAL)


@pytest.fixture(scope="session")
def stiff_plant():
    realization = [r for r in SENSITIVITY_LATTICE
                   if r.label == "solref_tau_0.01"][0]
    return build_realization(realization)


def test_sealed_xml_is_never_mutated():
    xml_before = model_xml()
    build_realization([r for r in SENSITIVITY_LATTICE
                       if r.label == "solref_tau_0.01_solimp_stiff"][0])
    assert model_xml() == xml_before


def test_declared_lattice_matches_res84_authority():
    labels = {r.label for r in SENSITIVITY_LATTICE}
    for expected in ("mu_0.5", "mu_1.5", "condim_3", "condim_6",
                     "dt_0.001", "dt_0.004", "solref_tau_0.01"):
        assert expected in labels
    support_names = set(C.V3_PLANTAR_SUPPORT_GEOMS)
    for realization in SENSITIVITY_LATTICE:
        plant = build_realization(realization)
        assert abs(float(plant.model.opt.timestep) - realization.dt_s) < 1e-12
        for name in support_names:
            gid = plant.idx.geom[name]
            assert int(plant.model.geom_condim[gid]) == realization.plantar_condim
            assert abs(float(plant.model.geom_friction[gid][0])
                       - realization.mu_slide) < 1e-12


def test_nominal_free_continuation_violates_penetration_ceiling(branch, nominal_plant):
    oracle = LandingFeasibilityOracle(branch, plant=nominal_plant, native_dt_s=0.002)
    steps = 35
    result = oracle.evaluate_sequence(np.zeros((steps, 9)), steps, collect_samples=False)
    assert result.max_penetration_m > 0.010
    assert "MAX_PENETRATION" in result.hard_gate_failures


def test_declared_solref_stiffening_keeps_free_continuation_inside_ceiling(
        branch, stiff_plant):
    oracle = LandingFeasibilityOracle(branch, plant=stiff_plant, native_dt_s=0.002)
    steps = 35
    result = oracle.evaluate_sequence(np.zeros((steps, 9)), steps, collect_samples=False)
    assert result.max_penetration_m <= 0.010
    assert "MAX_PENETRATION" not in result.hard_gate_failures
    assert result.prohibited_any is False


def test_contact_stiffening_materially_reduces_impact_penetration(branch, nominal_plant,
                                                                  stiff_plant):
    steps = 35
    sequence = np.zeros((steps, 9))
    nominal = LandingFeasibilityOracle(branch, plant=nominal_plant, native_dt_s=0.002)
    stiff = LandingFeasibilityOracle(branch, plant=stiff_plant, native_dt_s=0.002)
    nominal_result = nominal.evaluate_sequence(sequence, steps, collect_samples=False)
    stiff_result = stiff.evaluate_sequence(sequence, steps, collect_samples=False)
    assert stiff_result.max_penetration_m < 0.75 * nominal_result.max_penetration_m
    assert stiff_result.peak_total_fz_n > nominal_result.peak_total_fz_n
