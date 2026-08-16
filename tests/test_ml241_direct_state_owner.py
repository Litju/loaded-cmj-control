"""ML-241 direct current-state owner contract tests."""

from __future__ import annotations

from dataclasses import replace
import inspect
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

from loaded_cmj.oracle import derivatives
from loaded_cmj.oracle.derivatives import (
    CENTRAL_DIFFERENCE_SCHEME,
    DerivativeDomainError,
    differentiate_owner_output,
    differentiate_state_owner_output,
    evaluate_state_owner,
)
from loaded_cmj.oracle.transcription import (
    DirectMultipleShootingProblem,
    ML241_STATE_STEPS,
    advance_snapshot_exact,
)
from loaded_cmj.simulation import drive
from loaded_cmj.simulation.tangent import TANGENT_DIMENSION, boxplus

sys.path.insert(0, str(Path(__file__).parent))
from test_ml238_macro_state import fixtures as ml238_fixtures


def _fixtures():
    return ml238_fixtures.__wrapped__()


def _independent_direct_value(fixture, snapshot, owner_id: str) -> np.ndarray:
    """Independent test oracle; deliberately does not call ML-241 APIs."""

    data = fixture.plant.make_data()
    drive_state = drive.DriveState()
    snapshot.restore(plant=fixture.plant, data=data, drive_state=drive_state)
    mujoco.mj_forward(fixture.plant.model, data)
    if owner_id == "E3_E4_REVERSAL":
        return np.asarray([fixture.plant.center_of_mass_velocity(data)[2]], dtype=np.float64)
    if owner_id not in {"SUPPORT_MARGIN", "E3_E4_HORIZONTAL"}:
        raise AssertionError(owner_id)
    summary = fixture.plant.contact_wrench_summary(data)
    active = tuple(bool(value) for value in summary["active_by_foot"])
    com = fixture.plant.center_of_mass(data)
    return np.asarray(
        [fixture.plant.support_margin(data, com, active)],
        dtype=np.float64,
    )


def _independent_direct_fd(fixture, owner_id: str, index: int) -> np.ndarray:
    block = (
        "qvel_translation"
        if 30 <= index < 33
        else "qvel_rotation_joint"
        if 33 <= index < 51
        else "configuration_translation"
    )
    h = float(ML241_STATE_STEPS[block])
    delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
    delta[index] = h
    plus = _independent_direct_value(
        fixture,
        boxplus(fixture.snapshot, delta, model=fixture.plant.model),
        owner_id,
    )
    minus = _independent_direct_value(
        fixture,
        boxplus(fixture.snapshot, -delta, model=fixture.plant.model),
        owner_id,
    )
    return (plus - minus) / (2.0 * h)


def test_entry_authority_exposes_old_transition_map_and_independent_direct_counterexample():
    """The pre-change API is proven wrong for the new state-only contract."""

    fixture = _fixtures()["S3"]
    old = differentiate_owner_output(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=fixture.raw_action,
        owner_id="E3_E4_REVERSAL",
        state_steps=ML241_STATE_STEPS,
        action_steps=1.0e-4,
        state_columns=(32,),
        action_columns=(),
    )
    direct_fd = _independent_direct_fd(fixture, "E3_E4_REVERSAL", 32)
    assert old.state_columns[0].valid
    assert np.isfinite(old.state_jacobian[:, 32]).all()
    assert not np.allclose(old.state_jacobian[:, 32], direct_fd, rtol=0.0, atol=1.0e-6)


def test_direct_owner_value_matches_independent_current_state_oracle():
    fixture = _fixtures()["S3"]
    for owner_id in ("SUPPORT_MARGIN", "E3_E4_HORIZONTAL", "E3_E4_REVERSAL"):
        result = evaluate_state_owner(
            plant=fixture.plant,
            snapshot=fixture.snapshot,
            owner_id=owner_id,
        )
        np.testing.assert_array_equal(
            result.value,
            _independent_direct_value(fixture, fixture.snapshot, owner_id),
        )
        assert result.evaluation_mode == "DIRECT_STATE_OWNER"


def test_direct_owner_value_has_bitwise_parity_with_canonical_owner_callback():
    fixture = _fixtures()["S3"]
    data = fixture.plant.make_data()
    drive_state = drive.DriveState()
    fixture.snapshot.restore(plant=fixture.plant, data=data, drive_state=drive_state)
    mujoco.mj_forward(fixture.plant.model, data)
    zeros = np.zeros(15, dtype=np.float64)
    for owner_id in ("SUPPORT_MARGIN", "E3_E4_HORIZONTAL", "E3_E4_REVERSAL"):
        canonical = derivatives._owner_output_values(
            plant=fixture.plant,
            data=data,
            raw_action=zeros,
            accepted_action=zeros,
            drive_result={},
            power={},
            owner_ids=(owner_id,),
        )[owner_id]
        direct = evaluate_state_owner(
            plant=fixture.plant,
            snapshot=fixture.snapshot,
            owner_id=owner_id,
        )
        np.testing.assert_array_equal(direct.value, canonical)


def test_direct_owner_derivative_matches_independent_direct_fd_and_has_no_action_columns():
    fixture = _fixtures()["S3"]
    for owner_id in ("SUPPORT_MARGIN", "E3_E4_HORIZONTAL", "E3_E4_REVERSAL"):
        result = differentiate_state_owner_output(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            owner_id=owner_id,
            state_steps=ML241_STATE_STEPS,
            state_columns=(32,),
        )
        np.testing.assert_allclose(
            result.state_jacobian[:, 32],
            _independent_direct_fd(fixture, owner_id, 32),
            rtol=0.0,
            atol=1.0e-12,
        )
        assert result.action_columns == ()
        assert not result.action_validity.any()
        assert np.isnan(result.action_jacobian).all()
        assert np.isnan(result.state_jacobian[:, 0]).all()
        assert not result.state_validity[0]
        assert tuple(item.index for item in result.state_columns) == (32,)
        assert result.evaluation_mode == "DIRECT_STATE_OWNER"
        assert result.requested_action_columns == ()
        assert result.requested_state_columns == (32,)
        assert result.scheme == CENTRAL_DIFFERENCE_SCHEME


@pytest.mark.parametrize("owner_id", ("SUPPORT_MARGIN", "E3_E4_HORIZONTAL", "E3_E4_REVERSAL"))
def test_direct_owner_has_no_dependency_on_previous_accepted_action_state(owner_id):
    fixture = _fixtures()["S3"]
    result = differentiate_state_owner_output(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        owner_id=owner_id,
        state_steps=ML241_STATE_STEPS,
        state_columns=(96,),
    )
    assert result.state_columns[0].valid
    np.testing.assert_array_equal(result.state_jacobian[:, 96], np.zeros(1))


def test_direct_owner_api_has_no_raw_action_or_transition_arguments():
    signature = inspect.signature(differentiate_state_owner_output)
    assert "raw_action" not in signature.parameters
    assert "action_steps" not in signature.parameters
    assert "action_columns" not in signature.parameters
    signature = inspect.signature(evaluate_state_owner)
    assert "raw_action" not in signature.parameters


def test_direct_owner_never_advances_or_projects_action(monkeypatch):
    fixture = _fixtures()["S3"]

    def forbidden(*args, **kwargs):
        del args, kwargs
        raise AssertionError("direct state owner called transition machinery")

    monkeypatch.setattr(derivatives, "step_5ms", forbidden)
    monkeypatch.setattr(derivatives, "project_accepted_action", forbidden)
    before = fixture.snapshot.time
    result = evaluate_state_owner(
        plant=fixture.plant,
        snapshot=fixture.snapshot,
        owner_id="E3_E4_REVERSAL",
    )
    assert result.time == before
    source = inspect.getsource(derivatives.evaluate_state_owner)
    assert "step_5ms(" not in source
    assert "mj_step(" not in source
    assert "project_accepted_action" not in source


def test_direct_samples_are_restore_order_invariant():
    fixture = _fixtures()["S3"]
    delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
    delta[32] = ML241_STATE_STEPS["qvel_translation"]
    snapshots = {
        "BASE": fixture.snapshot,
        "PLUS": boxplus(fixture.snapshot, delta, model=fixture.plant.model),
        "MINUS": boxplus(fixture.snapshot, -delta, model=fixture.plant.model),
    }
    outputs = {}
    for order in (("BASE", "PLUS", "MINUS"), ("MINUS", "BASE", "PLUS")):
        for label in order:
            result = evaluate_state_owner(
                plant=fixture.plant,
                snapshot=snapshots[label],
                owner_id="SUPPORT_MARGIN",
            )
            outputs.setdefault(label, []).append(result)
    for label, values in outputs.items():
        assert values[0].value.tobytes() == values[1].value.tobytes()
        assert values[0].owner_output_digest == values[1].owner_output_digest
        assert values[0].snapshot_digest == values[1].snapshot_digest


def test_direct_qacc_columns_are_qualified_without_forced_array_overwrite():
    fixture = _fixtures()["S3"]
    qacc = tuple(range(111, 132))
    for owner_id in ("SUPPORT_MARGIN", "E3_E4_HORIZONTAL", "E3_E4_REVERSAL"):
        result = differentiate_state_owner_output(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            owner_id=owner_id,
            state_steps=ML241_STATE_STEPS,
            state_columns=qacc,
        )
        assert all(item.valid for item in result.state_columns)
        assert result.qacc_derivative_disposition == "NUMERICALLY_NULL_WITH_BOUNDED_ERROR"
        assert result.qacc_zero_columns == qacc
        assert result.qacc_absolute_error_bound == (
            ("qacc_translation", 0.0),
            ("qacc_rotation_joint", 0.0),
        )
        assert all(
            np.array_equal(result.state_jacobian[:, index], np.zeros(1))
            for index in qacc
        )


def test_direct_and_transition_provenance_cannot_be_confused():
    fixture = _fixtures()["S3"]
    direct = differentiate_state_owner_output(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        owner_id="E3_E4_REVERSAL",
        state_steps=ML241_STATE_STEPS,
        state_columns=(32,),
    )
    wrapped = differentiate_owner_output(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=fixture.raw_action,
        owner_id="E3_E4_REVERSAL",
        state_steps=ML241_STATE_STEPS,
        action_steps=1.0e-4,
        state_columns=(32,),
        action_columns=(),
    )
    assert direct.evaluation_mode == "DIRECT_STATE_OWNER"
    assert wrapped.evaluation_mode == "TRANSITION_WRAPPED_OWNER"
    assert direct.evaluation_mode != wrapped.evaluation_mode
    assert direct.base_next_snapshot_digest == ""
    assert wrapped.base_next_snapshot_digest


def test_direct_support_owner_uses_real_branch_certificate():
    fixture = _fixtures()["S3"]
    result = differentiate_state_owner_output(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        owner_id="SUPPORT_MARGIN",
        state_steps=ML241_STATE_STEPS,
        state_columns=(0,),
    )
    assert result.base_branch_certificate is not None
    assert result.state_columns[0].active_set_preserved
    assert result.state_columns[0].reason == "SMOOTH_FIXED_ACTIVE_SET"


def test_direct_real_plant_branch_switches_are_rejected():
    fixture = _fixtures()["S3"]
    with pytest.raises(DerivativeDomainError, match="SUPPORT_HULL_KINK_INVALID"):
        differentiate_state_owner_output(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            owner_id="SUPPORT_MARGIN",
            state_steps=ML241_STATE_STEPS,
            state_columns=(10,),
        )
    with pytest.raises(DerivativeDomainError, match="PHYSICAL_CONTACT_SWITCH_INVALID"):
        differentiate_state_owner_output(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            owner_id="E3_E4_REVERSAL",
            state_steps=ML241_STATE_STEPS,
            state_columns=(2,),
        )


def test_direct_owner_rejects_non_state_owner_ids():
    fixture = _fixtures()["S3"]
    with pytest.raises(DerivativeDomainError):
        evaluate_state_owner(
            plant=fixture.plant,
            snapshot=fixture.snapshot,
            owner_id="COM_VELOCITY",
        )


def test_public_knot_state_reconstruction_is_private_helper_parity():
    fixture = _fixtures()["S3"]
    next_snapshot = advance_snapshot_exact(
        plant=fixture.plant,
        snapshot=fixture.snapshot,
        raw_action=fixture.raw_action,
    )
    problem = DirectMultipleShootingProblem(
        plant=fixture.plant,
        reference_snapshots=(fixture.snapshot, next_snapshot),
    )
    delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
    public = problem.reconstruct_knot_state(1, delta)
    private = problem._reconstruct_state(1, delta)
    assert derivatives.snapshot_digest(public) == derivatives.snapshot_digest(private)
    assert problem.variable_count == TANGENT_DIMENSION + 15
    assert problem.constraint_count == TANGENT_DIMENSION


def test_terminal_knot_40_direct_owners_require_no_action_or_u39():
    fixture = _fixtures()["S3"]
    references = tuple(
        replace(
            fixture.snapshot,
            time=fixture.snapshot.time + 0.005 * knot,
        )
        for knot in range(41)
    )
    problem = DirectMultipleShootingProblem(
        plant=fixture.plant,
        reference_snapshots=references,
    )
    terminal = problem.reconstruct_knot_state(
        40,
        np.zeros(TANGENT_DIMENSION, dtype=np.float64),
    )
    for owner_id in ("E3_E4_HORIZONTAL", "E3_E4_REVERSAL"):
        evaluation = evaluate_state_owner(
            plant=fixture.plant,
            snapshot=terminal,
            owner_id=owner_id,
        )
        assert evaluation.time == terminal.time
        source = inspect.getsource(derivatives.evaluate_state_owner)
        assert "step_5ms(" not in source
        assert "mj_step(" not in source
