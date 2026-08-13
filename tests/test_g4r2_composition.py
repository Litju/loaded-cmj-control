"""Contract tests for target-side E3->E4 physical-row composition."""

from __future__ import annotations

import hashlib
import inspect
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from loaded_cmj.oracle import composition
from loaded_cmj.oracle.composition import (
    E3E4PhysicalConstraintComposer,
)
from loaded_cmj.oracle.derivatives import (
    DIRECT_STATE_OWNER,
    TRANSITION_WRAPPED_OWNER,
    DerivativeDomainError,
    differentiate_state_owner_output,
    evaluate_state_owner,
)
from loaded_cmj.oracle.transcription import ML241_STATE_STEPS

sys.path.insert(0, str(Path(__file__).parent))
from test_ml242_transcription import (  # noqa: E402
    STATE_DIMENSION,
    _r2_problem,
    qualified_fixtures,
)


@pytest.fixture(scope="module")
def r2_problem():
    fixtures = qualified_fixtures.__wrapped__()
    problem, actions = _r2_problem(fixtures, action_layout="FULL_LAYOUT")
    z = problem.pack_decision(
        [np.zeros(STATE_DIMENSION, dtype=np.float64) for _ in range(problem.horizon)],
        actions,
        np.zeros(len(problem.layout.slack_specs), dtype=np.float64),
    )
    return problem, z


def test_exactly_43_rows_and_canonical_order(r2_problem):
    problem, _ = r2_problem
    rows = E3E4PhysicalConstraintComposer(problem).physical_constraint_owner_receipt()

    assert len(rows) == 43
    assert [(row.constraint_id, row.knot_index) for row in rows[:2]] == [
        ("E3_E4_HORIZONTAL", 40),
        ("E3_E4_REVERSAL", 40),
    ]
    assert [(row.constraint_id, row.knot_index) for row in rows[2:]] == [
        ("SUPPORT_MARGIN", knot) for knot in range(41)
    ]
    assert [row.row_index for row in rows] == list(range(5280, 5323))
    assert all(row.support == (("state", row.knot_index),) for row in rows)


def test_live_catalog_metadata_and_hard_elastic_disposition(r2_problem):
    problem, _ = r2_problem
    rows = E3E4PhysicalConstraintComposer(problem).physical_constraint_owner_receipt()

    for row in rows:
        spec = problem.catalog.get(row.constraint_id)
        assert row.semantic_name == spec.name
        assert row.units == spec.units
        assert row.sense == spec.sense
        assert row.differentiability == spec.differentiability
    assert rows[0].hard_or_elastic == "ELASTIC"
    assert rows[1].hard_or_elastic == "HARD_NONELASTIC"
    assert rows[1].slack_decision_index is None


def test_knot_zero_has_no_decision_state_derivative(r2_problem):
    problem, z = r2_problem
    composer = E3E4PhysicalConstraintComposer(problem)

    result = composer.physical_constraint_jacobian_values(
        z,
        physical_row_indices=(2,),
        state_columns=(0,),
    )[0]

    assert result.row.knot_index == 0
    assert result.row.decision_state_columns == ()
    assert result.owner_sensitivity is None
    assert result.state_columns == ()
    assert result.state_values.shape == (1, 0)


def test_support_knots_one_through_40_are_state_only(r2_problem):
    problem, _ = r2_problem
    rows = E3E4PhysicalConstraintComposer(problem).physical_constraint_owner_receipt()[2:]

    assert [row.knot_index for row in rows] == list(range(41))
    assert rows[0].decision_state_columns == ()
    assert all(
        row.decision_state_columns == tuple(range(STATE_DIMENSION))
        for row in rows[1:]
    )
    assert all(row.decision_action_columns == () for row in rows)


def test_values_use_canonical_current_state_owners(r2_problem):
    problem, z = r2_problem
    composer = E3E4PhysicalConstraintComposer(problem)

    values = composer.physical_constraint_values(z)

    assert values.shape == (43,)
    assert np.isfinite(values).all()
    assert values[0] == pytest.approx(values[2 + 40])
    for owner_index, owner_id in ((0, "E3_E4_HORIZONTAL"), (1, "E3_E4_REVERSAL")):
        expected = evaluate_state_owner(
            plant=problem.plant,
            snapshot=problem.reference_snapshots[40],
            owner_id=owner_id,
        ).value[0]
        assert values[owner_index] == pytest.approx(expected)


def test_support_activity_is_taken_from_plant_contact_owner(r2_problem, monkeypatch):
    problem, z = r2_problem
    active_sets: list[tuple[bool, bool]] = []

    def contact_summary(plant, data):
        del plant, data
        return {"active_by_foot": np.asarray((True, False), dtype=bool)}

    def support_margin(plant, data, com, active):
        del plant, data, com
        active_sets.append(tuple(bool(value) for value in active))
        return 0.25

    monkeypatch.setattr(composition, "adapt_contact_wrench_summary", contact_summary)
    monkeypatch.setattr(composition, "adapt_support_margin", support_margin)
    values = E3E4PhysicalConstraintComposer(problem).physical_constraint_values(z)

    assert values.shape == (43,)
    assert set(active_sets) == {(True, False)}


def test_real_direct_owner_routing_matches_independent_ml241_results(r2_problem):
    problem, z = r2_problem
    calls: list[dict[str, object]] = []

    def real_owner(**kwargs):
        calls.append(kwargs)
        return differentiate_state_owner_output(**kwargs)

    results = E3E4PhysicalConstraintComposer(
        problem,
        derivative_owner=real_owner,
    ).physical_constraint_jacobian_values(
        z,
        physical_row_indices=(0, 1, 3),
        state_columns=(32,),
    )

    assert [result.row.constraint_id for result in results] == [
        "E3_E4_HORIZONTAL",
        "E3_E4_REVERSAL",
        "SUPPORT_MARGIN",
    ]
    assert [call["owner_id"] for call in calls] == [
        "E3_E4_HORIZONTAL",
        "E3_E4_REVERSAL",
        "SUPPORT_MARGIN",
    ]
    assert all("raw_action" not in call for call in calls)
    assert all("action_columns" not in call for call in calls)
    assert all(result.owner_sensitivity.evaluation_mode == DIRECT_STATE_OWNER for result in results)

    for result in results:
        direct = differentiate_state_owner_output(
            plant=problem.plant,
            base_snapshot=problem.reference_snapshots[result.row.knot_index],
            owner_id=result.row.constraint_id,
            state_steps=ML241_STATE_STEPS,
            state_columns=(32,),
        )
        np.testing.assert_allclose(result.state_values, direct.state_jacobian[:, (32,)])


def test_terminal_derivatives_need_no_action(r2_problem):
    problem, z = r2_problem
    results = E3E4PhysicalConstraintComposer(problem).physical_constraint_jacobian_values(
        z,
        physical_row_indices=(0, 1),
        state_columns=(32,),
    )

    assert all(result.row.knot_index == 40 for result in results)
    assert all(result.row.decision_action_columns == () for result in results)
    assert all(result.owner_sensitivity.requested_action_columns == () for result in results)


def test_real_g4r1b_smooth_branch_is_accepted(r2_problem):
    problem, z = r2_problem
    result = E3E4PhysicalConstraintComposer(problem).physical_constraint_jacobian_values(
        z,
        physical_row_indices=(3,),
        state_columns=(0,),
    )[0]

    assert result.owner_sensitivity.base_branch_certificate is not None
    assert result.owner_sensitivity.base_branch_certificate.finite
    assert result.owner_sensitivity.state_columns[0].active_set_preserved


def test_real_g4r1b_kink_propagates_through_composer(r2_problem, monkeypatch):
    problem, z = r2_problem
    base_x = float(problem.reference_snapshots[1].qpos[0])
    smooth = np.asarray(
        [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
        dtype=np.float64,
    )
    topology_switch = np.asarray(
        [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0], [0.0, 0.0]],
        dtype=np.float64,
    )

    def deterministic_support_polygon(data, latched):
        del latched
        if float(data.qpos[0]) > base_x + 5.0e-5:
            return smooth.copy()
        return topology_switch.copy()

    monkeypatch.setattr(problem.plant, "support_polygon", deterministic_support_polygon)
    with pytest.raises(DerivativeDomainError, match="SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID"):
        E3E4PhysicalConstraintComposer(problem).physical_constraint_jacobian_values(
            z,
            physical_row_indices=(3,),
            state_columns=(0,),
        )


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("owner_id", "SUPPORT_MARGIN"),
        ("evaluation_mode", TRANSITION_WRAPPED_OWNER),
        ("base_snapshot_digest", "wrong-base"),
        ("action_columns", None),
        ("state_columns", None),
    ),
)
def test_provenance_negative_controls(r2_problem, field, value):
    problem, z = r2_problem
    base = differentiate_state_owner_output(
        plant=problem.plant,
        base_snapshot=problem.reference_snapshots[1],
        owner_id="SUPPORT_MARGIN",
        state_steps=ML241_STATE_STEPS,
        state_columns=(32,),
    )
    if field == "owner_id":
        value = "E3_E4_REVERSAL"
    if field == "action_columns":
        value = base.state_columns
    if field == "state_columns":
        wrong = differentiate_state_owner_output(
            plant=problem.plant,
            base_snapshot=problem.reference_snapshots[1],
            owner_id="SUPPORT_MARGIN",
            state_steps=ML241_STATE_STEPS,
            state_columns=(31,),
        )
        value = wrong.state_columns
    tampered = replace(base, **{field: value})

    def tampered_owner(**kwargs):
        del kwargs
        return tampered

    with pytest.raises(ValueError):
        E3E4PhysicalConstraintComposer(
            problem,
            derivative_owner=tampered_owner,
        ).physical_constraint_jacobian_values(
            z,
            physical_row_indices=(3,),
            state_columns=(32,),
        )


def test_invalid_support_branch_certificate_is_rejected(r2_problem):
    problem, z = r2_problem
    base = differentiate_state_owner_output(
        plant=problem.plant,
        base_snapshot=problem.reference_snapshots[1],
        owner_id="SUPPORT_MARGIN",
        state_steps=ML241_STATE_STEPS,
        state_columns=(32,),
    )
    tampered_certificate = replace(base.base_branch_certificate, finite=False)
    tampered = replace(base, base_branch_certificate=tampered_certificate)

    def tampered_owner(**kwargs):
        del kwargs
        return tampered

    with pytest.raises(ValueError, match="branch certificate"):
        E3E4PhysicalConstraintComposer(
            problem,
            derivative_owner=tampered_owner,
        ).physical_constraint_jacobian_values(
            z,
            physical_row_indices=(3,),
            state_columns=(32,),
        )


def test_composition_uses_public_knot_reconstruction_only():
    source = inspect.getsource(composition)
    assert "reconstruct_knot_state" in source
    assert "_reconstruct_state" not in source


def test_composition_has_no_transition_or_task_local_derivative_logic():
    source = inspect.getsource(composition)
    assert "differentiate_owner_output" not in source
    assert "raw_action" not in source
    assert "u39" not in source
    assert "step_5ms(" not in source
    assert "mj_step(" not in source
    assert "Phi_5ms" not in source
    assert "np.diff(" not in source
    assert "central_difference" not in source
    assert "(True, True)" not in source
    assert "ipopt" not in source.lower()
    assert "rho" not in source.lower()


def test_historical_evidence_is_unchanged():
    path = Path(
        "/home/litju/Projects/loaded-cmj-control-evidence/"
        "F4-ORACLE-FEASIBILITY/ML212-E3-E4-SAME-PLANT/"
        "20260811T230148Z/ml212_runner.py"
    )
    assert hashlib.sha256(path.read_bytes()).hexdigest() == (
        "254c0389c62fcb2d244204eb26ff02751a9054d0690a32fcca30ed8541f695ba"
    )
