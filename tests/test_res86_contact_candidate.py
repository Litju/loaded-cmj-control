"""RES-86 contact-candidate realization tests (solution verification).

MISSION: RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001
LINEAR ISSUE: RES-86

The RES-86 contact candidate is the smallest declared stiffening of the audit
lattice: the plantar support-geom ``solref`` time constant 0.02 -> 0.01 s.  The
tests bind the *realized* (engine-mixed) contact row, the frozen solver
semantics, the timestep resolution of the realized time constant, and the
committed reference-witness reproduction.  A label is never the claim: a
candidate that claims its geom-level 0.01 s declaration as the runtime value
must fail the realization verification.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
for _path in (str(TASK_ROOT), str(TASK_ROOT / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3.contact_realization import (  # noqa: E402
    DECLARED_SOLIMP,
    MUJOCO_COMPILED_DEFAULT_SOLREF,
    RES86_CANDIDATE_DECLARED_SOLREF,
    RES86_CANDIDATE_REALIZED_SOLREF,
    RES86_CONTACT_CANDIDATE_LABEL,
    RES86_NOMINAL_DECLARED_SOLREF,
    RES86_NOMINAL_REALIZED_SOLREF,
    TIMECONST_DT_RESOLUTION_FACTOR,
    build_contact_realization_plant,
    effective_floor_contacts,
    engine_solver_semantics,
    mixed_solref,
    timeconst_resolution,
    verify_effective_contact_realization,
)
from loaded_cmj.v3.landing_runtime import settle_standing_stance  # noqa: E402
from loaded_cmj.v3.plant import V3Plant  # noqa: E402

AUDIT = TASK_ROOT / "audit" / "EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001"
REFERENCE_WITNESSES = AUDIT / "contact_candidate" / "REFERENCE_WITNESSES.json"
QUALIFICATION = AUDIT / "contact_candidate" / "CONTACT_CANDIDATE_QUALIFICATION.json"


def _loaded(plant):
    data = plant.make_data()
    settle_standing_stance(plant, data)
    return plant, data


def test_candidate_identity_and_realized_mixing():
    assert RES86_CONTACT_CANDIDATE_LABEL == "RES86_CONTACT_CANDIDATE_SOLREF_TAU_0_01"
    assert RES86_CANDIDATE_DECLARED_SOLREF == (0.01, 1.0)
    assert RES86_NOMINAL_DECLARED_SOLREF == (0.02, 1.0)
    # the geom-level declaration is not the runtime realization
    assert mixed_solref(RES86_CANDIDATE_DECLARED_SOLREF) == RES86_CANDIDATE_REALIZED_SOLREF
    assert mixed_solref((0.004, 1.0)) == (0.012, 1.0)
    assert RES86_CANDIDATE_REALIZED_SOLREF == (0.015, 1.0)

    nominal = build_contact_realization_plant(RES86_NOMINAL_DECLARED_SOLREF, dt_s=0.002)
    candidate = build_contact_realization_plant(RES86_CANDIDATE_DECLARED_SOLREF, dt_s=0.002)
    _plant, nominal_data = _loaded(nominal)
    _plant2, candidate_data = _loaded(candidate)
    nominal_rows = [row for row in effective_floor_contacts(nominal, nominal_data)
                    if row.is_plantar_floor]
    candidate_rows = [row for row in effective_floor_contacts(candidate, candidate_data)
                      if row.is_plantar_floor]
    assert nominal_rows and candidate_rows
    for row in nominal_rows:
        assert row.solref == RES86_NOMINAL_REALIZED_SOLREF
    for row in candidate_rows:
        assert row.solref == RES86_CANDIDATE_REALIZED_SOLREF
        # a single declared contact-compliance change: nothing else moves
        assert row.dim == 4
        assert tuple(row.solimp) == DECLARED_SOLIMP
        assert row.friction[:2] == (0.9, 0.9)
        assert row.includemargin_m == 0.0
    # the declared candidate is strictly softer than the realized nominal value
    assert RES86_CANDIDATE_REALIZED_SOLREF[0] < RES86_NOMINAL_REALIZED_SOLREF[0]


def test_label_is_not_the_claim_and_verification_fails_closed():
    candidate = build_contact_realization_plant(RES86_CANDIDATE_DECLARED_SOLREF, dt_s=0.002)
    _plant, data = _loaded(candidate)
    mislabeled = verify_effective_contact_realization(
        candidate, data, expected_solref=RES86_CANDIDATE_DECLARED_SOLREF)
    assert mislabeled["status"] == "FAIL"
    assert any(failure.startswith("SOLREF:") for failure in mislabeled["failures"])
    realized = verify_effective_contact_realization(
        candidate, data, expected_solref=RES86_CANDIDATE_REALIZED_SOLREF)
    assert realized["status"] == "PASS", realized["failures"]
    assert realized["declared"]["solref"] == list(RES86_CANDIDATE_REALIZED_SOLREF)


def test_frozen_solver_semantics_and_refsafe_are_untouched():
    nominal = V3Plant()
    candidate = build_contact_realization_plant(RES86_CANDIDATE_DECLARED_SOLREF, dt_s=0.002)
    base = engine_solver_semantics(nominal.model)
    declared = engine_solver_semantics(candidate.model)
    for name in ("integrator", "cone", "solver", "iterations", "tolerance",
                 "ls_iterations", "disableflags", "refsafe_enabled",
                 "contact_disabled"):
        assert declared[name] == base[name], name
    assert declared["refsafe_enabled"] is True
    assert declared["contact_disabled"] is False
    assert declared["disableflags"] == 0
    assert base["refsafe_enabled"] is True


def test_timeconst_is_resolved_at_every_declared_dt():
    for dt, expected_ratio in ((0.001, 15.0), (0.002, 7.5), (0.004, 3.75)):
        report = timeconst_resolution(RES86_CANDIDATE_DECLARED_SOLREF, dt)
        assert report["realized_solref"][0] == 0.015
        assert report["timeconst_over_dt"] == pytest.approx(expected_ratio)
        assert report["resolved"] is True
        assert expected_ratio >= TIMECONST_DT_RESOLUTION_FACTOR
    # negative control: a declared pair whose mixed time constant is under two
    # steps at dt = 0.004 must be reported unresolved
    unresolved = timeconst_resolution((0.0001, 1.0), 0.004,
                                      other_solref=(0.0001, 1.0))
    assert unresolved["timeconst_over_dt"] < TIMECONST_DT_RESOLUTION_FACTOR
    assert unresolved["resolved"] is False
    # the floor's compiled default lifts even a tiny declared value: the
    # realized value is what the resolution claim must use
    lifted = timeconst_resolution((0.0001, 1.0), 0.004,
                                  other_solref=MUJOCO_COMPILED_DEFAULT_SOLREF)
    assert lifted["realized_solref"][0] > 0.0001
    assert lifted["resolved"] is True


def test_prior_reference_witnesses_reproduce_bit_exactly():
    from tools.res86.landing_feasibility_oracle import (
        LandingFeasibilityOracle,
        capture_canonical_branch,
    )

    document = json.loads(REFERENCE_WITNESSES.read_text())
    branch = capture_canonical_branch()
    outcomes = {}
    for name, declared, expect_admissible in (
            ("nominal_witness", RES86_NOMINAL_DECLARED_SOLREF, False),
            ("candidate_witness", RES86_CANDIDATE_DECLARED_SOLREF, True)):
        witness = document["witnesses"][name]
        desired = np.asarray(witness["desired_nm"], dtype=np.float64)
        plant = build_contact_realization_plant(declared, dt_s=0.002)
        oracle = LandingFeasibilityOracle(branch, plant=plant, native_dt_s=0.002)
        first = oracle.evaluate_sequence(desired, desired.shape[0], collect_samples=False)
        second = oracle.evaluate_sequence(desired, desired.shape[0], collect_samples=False)
        assert first.terminal_state_sha256 == second.terminal_state_sha256
        recorded = witness["recorded_outcome"]
        assert first.max_penetration_m == pytest.approx(
            recorded["max_penetration_m"], rel=1.0e-12, abs=1.0e-15)
        assert first.terminal_com_vz_m_s == pytest.approx(
            recorded["terminal_com_vz_m_s"], rel=1.0e-12, abs=1.0e-15)
        assert first.admissible is expect_admissible
        outcomes[name] = first
    assert list(outcomes["nominal_witness"].hard_gate_failures) == ["MAX_PENETRATION"]
    assert outcomes["nominal_witness"].max_penetration_m > 0.010
    assert list(outcomes["candidate_witness"].hard_gate_failures) == []
    assert outcomes["candidate_witness"].max_penetration_m <= 0.010


def test_qualification_artifact_is_pass_and_declares_the_boundary():
    report = json.loads(QUALIFICATION.read_text())
    assert report["status"] == "PASS"
    for name in report["required_criteria"]:
        assert report["criteria"][name] is True, name
    assert report["candidate"]["label"] == RES86_CONTACT_CANDIDATE_LABEL
    assert report["candidate"]["expected_realized_solref"] == [0.015, 1.0]
    assert set(report["dt_grid_s"]) == {0.001, 0.002, 0.004}
    boundary = report["boundary"]
    assert "not an E10 witness" in boundary["not_e10"]
    assert "never a global infeasibility proof" in boundary["nominal_10mm_is_not_infeasibility"]
    assert report["criteria"]["V6_nominal_boundary_failure"] == ["MAX_PENETRATION"]
