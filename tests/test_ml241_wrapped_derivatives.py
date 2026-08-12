"""Focused ML-241 tests for the single wrapped derivative owner."""

from __future__ import annotations

import inspect
import sys
from pathlib import Path

import numpy as np
import pytest

from loaded_cmj.oracle.derivatives import (
    ACTION_BRANCH_BOUND_ACTIVE,
    ACTION_BRANCH_INTERIOR,
    ACTION_BRANCH_NEAR_KINK,
    ACTION_BRANCH_SLEW_ACTIVE,
    DerivativeDomainError,
    differentiate_owner_output,
    evaluate_wrapped_step_5ms,
    classify_action_projection,
    linearize_step_5ms,
    snapshot_digest,
)
from loaded_cmj.oracle import derivatives

sys.path.insert(0, str(Path(__file__).parent))
from test_ml238_macro_state import fixtures as ml238_fixtures


def _fixtures():
    return ml238_fixtures.__wrapped__()


def _steps():
    return {
        "configuration_translation": 1.0e-5,
        "configuration_rotation_joint": 1.0e-5,
        "cache_so3": 1.0e-5,
        "qvel_translation": 1.0e-4,
        "qvel_rotation_joint": 1.0e-4,
        "drivestate_activation": 1.0e-5,
        "drivestate_tau_prev": 1.0e-3,
        "previous_accepted_action": 1.0e-5,
        "qacc_translation": 1.0e-3,
        "qacc_rotation_joint": 1.0e-3,
    }


def test_wrapped_shapes_and_common_output_frame():
    fixture = _fixtures()["S3"]
    result = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=fixture.snapshot.previous_accepted_action,
        state_steps=_steps(),
        action_steps=1.0e-5,
        state_columns=[0],
        action_columns=[0],
    )
    assert result.A.shape == (132, 132)
    assert result.B.shape == (132, 15)
    assert result.state_validity[0]
    assert result.action_validity[0]
    assert result.scheme == "central_boxminus_at_common_y0"


def test_each_evaluation_restores_identical_base_snapshot():
    fixture = _fixtures()["S3"]
    first = evaluate_wrapped_step_5ms(
        plant=fixture.plant,
        snapshot=fixture.snapshot,
        raw_action=fixture.snapshot.previous_accepted_action,
    )
    second = evaluate_wrapped_step_5ms(
        plant=fixture.plant,
        snapshot=fixture.snapshot,
        raw_action=fixture.snapshot.previous_accepted_action,
    )
    assert first.snapshot_digest == snapshot_digest(fixture.snapshot)
    assert second.snapshot_digest == first.snapshot_digest
    assert first.next_snapshot_digest == second.next_snapshot_digest
    assert first.active_set == second.active_set
    np.testing.assert_array_equal(first.next_snapshot.qpos, second.next_snapshot.qpos)
    np.testing.assert_array_equal(first.next_snapshot.qvel, second.next_snapshot.qvel)
    np.testing.assert_array_equal(first.next_snapshot.qacc_warmstart, second.next_snapshot.qacc_warmstart)


def test_state_perturbation_and_output_difference_use_frozen_manifold_operators():
    source = inspect.getsource(derivatives.linearize_step_5ms)
    assert "boxplus" in source
    assert "boxminus" in source
    assert "plus_eval.next_snapshot -" not in source
    assert "minus_eval.next_snapshot -" not in source
    assert "qpos" not in source
    assert "xmat" not in source


def test_projection_branch_classification_is_source_shaped():
    previous = np.zeros(15)
    raw = np.zeros(15)
    raw[0] = 0.1
    raw[1] = 0.3
    raw[2] = 0.2
    raw[3] = 1.0
    branches = classify_action_projection(previous, raw)
    assert branches[0] == ACTION_BRANCH_INTERIOR
    assert branches[1] == ACTION_BRANCH_SLEW_ACTIVE
    assert branches[2] == ACTION_BRANCH_NEAR_KINK
    assert branches[3] == ACTION_BRANCH_BOUND_ACTIVE


def test_slew_active_raw_input_has_zero_projection_sensitivity_when_preserved():
    fixture = _fixtures()["S3"]
    raw = fixture.snapshot.previous_accepted_action + 0.3
    result = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=raw,
        state_steps=_steps(),
        action_steps=1.0e-5,
        state_columns=[],
        action_columns=[0],
    )
    assert result.action_columns[0].valid
    assert result.base_active_set.action_branches[0] == ACTION_BRANCH_SLEW_ACTIVE
    assert result.accepted_action_jacobian[0, 0] == 0.0


def test_kink_is_rejected_instead_of_averaged():
    fixture = _fixtures()["S3"]
    raw = fixture.snapshot.previous_accepted_action.copy()
    raw[0] += 0.2
    with pytest.raises(DerivativeDomainError) as error:
        linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            raw_action=raw,
            state_steps=_steps(),
            action_steps=1.0e-5,
            state_columns=[],
            action_columns=[0],
        )
    assert error.value.reports[0].reason == "PIECEWISE_BRANCH_SWITCH_INVALID"


def test_contact_changing_state_column_is_rejected():
    fixture = _fixtures()["S0"]
    with pytest.raises(DerivativeDomainError) as error:
        linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            raw_action=fixture.snapshot.previous_accepted_action,
            state_steps=_steps(),
            action_steps=1.0e-5,
            state_columns=[3],
            action_columns=[],
        )
    assert error.value.reports[0].reason == "PHYSICAL_CONTACT_SWITCH_INVALID"


def test_previous_action_tangent_column_is_checked_against_slew_active_set():
    fixture = _fixtures()["S3"]
    result = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=fixture.snapshot.previous_accepted_action,
        state_steps=_steps(),
        action_steps=1.0e-5,
        state_columns=[96],
        action_columns=[],
    )
    assert result.state_columns[0].active_set_preserved


def test_owner_output_sensitivity_uses_source_owner_value():
    fixture = _fixtures()["S3"]
    result = differentiate_owner_output(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=fixture.snapshot.previous_accepted_action,
        owner_id="COM_VELOCITY",
        state_steps=_steps(),
        action_steps=1.0e-5,
        state_columns=[0],
        action_columns=[0],
    )
    assert result.base_value.shape == (3,)
    assert result.state_jacobian.shape == (3, 132)
    assert result.action_jacobian.shape == (3, 15)
    assert np.isfinite(result.state_jacobian[:, 0]).all()
    assert np.isfinite(result.action_jacobian[:, 0]).all()
