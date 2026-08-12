"""Bounded ML-241-G3 stencil and branch-certificate qualification."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

from loaded_cmj.oracle.derivatives import (
    BACKWARD_FEASIBLE_SIDE,
    CENTRAL_INTERIOR,
    DerivativeDomainError,
    FORWARD_FEASIBLE_SIDE,
    PHYSICAL_CONTACT_SWITCH_INVALID,
    PIECEWISE_BRANCH_SWITCH_INVALID,
    TRUE_KINK_INVALID,
    _native_joint_limits,
    linearize_step_5ms,
)
from loaded_cmj.oracle.transcription import ML241_ACTION_STEP, ML241_STATE_STEPS

sys.path.insert(0, str(Path(__file__).parent))
from test_ml238_macro_state import fixtures as ml238_fixtures


FREE = (0, 3, 6, 9, 10, 11, 12)
FIXED = (1, 2, 4, 5, 7, 8, 13, 14)


def _fixtures():
    return ml238_fixtures.__wrapped__()


def _witness_action(snapshot) -> np.ndarray:
    action = np.asarray(snapshot.previous_accepted_action, dtype=np.float64).copy()
    action[list(FIXED)] = 0.0
    return action


def test_activation_and_previous_action_native_bound_stencils_are_feasible_side():
    fixture = _fixtures()["S3"]
    raw = fixture.snapshot.previous_accepted_action

    for attribute, index in (("a_plus", 51), ("a_minus", 66)):
        lower = getattr(fixture.snapshot, attribute).copy()
        lower[0] = 0.0
        lower_result = linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=replace(fixture.snapshot, **{attribute: lower}),
            raw_action=raw,
            state_steps=ML241_STATE_STEPS,
            action_steps=ML241_ACTION_STEP,
            state_columns=[index],
            action_columns=[],
        )
        assert lower_result.state_columns[0].stencil == FORWARD_FEASIBLE_SIDE
        assert lower_result.state_columns[0].valid
        assert np.isfinite(lower_result.A[:, index]).all()
        assert lower_result.transition_evaluation_count == 3

        upper = getattr(fixture.snapshot, attribute).copy()
        upper[0] = 1.0
        upper_result = linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=replace(fixture.snapshot, **{attribute: upper}),
            raw_action=raw,
            state_steps=ML241_STATE_STEPS,
            action_steps=ML241_ACTION_STEP,
            state_columns=[index],
            action_columns=[],
        )
        assert upper_result.state_columns[0].stencil == BACKWARD_FEASIBLE_SIDE
        assert upper_result.state_columns[0].valid
        assert np.isfinite(upper_result.A[:, index]).all()
        assert upper_result.transition_evaluation_count == 3

    for value, stencil in ((-1.0, FORWARD_FEASIBLE_SIDE), (1.0, BACKWARD_FEASIBLE_SIDE), (0.2, CENTRAL_INTERIOR)):
        previous = np.full(15, value, dtype=np.float64)
        result = linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=replace(fixture.snapshot, previous_accepted_action=previous),
            raw_action=previous,
            state_steps=ML241_STATE_STEPS,
            action_steps=ML241_ACTION_STEP,
            state_columns=[96],
            action_columns=[],
        )
        assert result.state_columns[0].stencil == stencil
        assert result.state_columns[0].valid
        assert np.isfinite(result.A[:, 96]).all()

    for value, stencil in ((-1.0, FORWARD_FEASIBLE_SIDE), (1.0, BACKWARD_FEASIBLE_SIDE)):
        raw = np.full(15, value, dtype=np.float64)
        result = linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=replace(fixture.snapshot, previous_accepted_action=raw),
            raw_action=raw,
            state_steps=ML241_STATE_STEPS,
            action_steps=ML241_ACTION_STEP,
            state_columns=[],
            action_columns=[0],
        )
        assert result.action_columns[0].stencil == stencil
        assert result.action_columns[0].valid
        assert np.isfinite(result.B[:, 0]).all()
        assert np.isclose(result.accepted_action_jacobian[0, 0], 1.0)


def test_witness_local_request_is_finite_and_deterministic_without_fixed_columns():
    fixture = _fixtures()["S3"]
    raw = _witness_action(fixture.snapshot)
    first = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=raw,
        state_steps=ML241_STATE_STEPS,
        action_steps=ML241_ACTION_STEP,
        state_columns=[],
        action_columns=FREE,
    )
    second = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=raw,
        state_steps=ML241_STATE_STEPS,
        action_steps=ML241_ACTION_STEP,
        state_columns=[],
        action_columns=FREE,
    )
    assert first.state_columns == ()
    assert tuple(report.index for report in first.action_columns) == FREE
    assert all(report.valid and report.stencil == CENTRAL_INTERIOR for report in first.action_columns)
    assert all(abs(raw[index]) > ML241_ACTION_STEP for index in FREE)
    assert np.isfinite(first.B[:, list(FREE)]).all()
    assert np.isnan(first.A).all()
    assert np.array_equal(first.action_validity[list(FIXED)], np.zeros(len(FIXED), dtype=bool))
    assert np.array_equal(first.B, second.B, equal_nan=True)
    assert first.base_active_set == second.base_active_set


def test_true_zero_kink_rejects_every_explicit_fixed_witness_channel():
    fixture = _fixtures()["S0"]
    with pytest.raises(DerivativeDomainError) as error:
        linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            raw_action=fixture.snapshot.previous_accepted_action,
            state_steps=ML241_STATE_STEPS,
            action_steps=ML241_ACTION_STEP,
            state_columns=[],
            action_columns=FIXED,
        )
    assert [report.index for report in error.value.reports] == list(FIXED)
    assert all(report.reason == TRUE_KINK_INVALID for report in error.value.reports)


def test_a2_authoritative_contact_negative_control_is_rejected():
    fixture = _fixtures()["S0"]
    with pytest.raises(DerivativeDomainError) as error:
        linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            raw_action=fixture.snapshot.previous_accepted_action,
            state_steps=ML241_STATE_STEPS,
            action_steps=ML241_ACTION_STEP,
            state_columns=[2],
            action_columns=[],
        )
    report = error.value.reports[0]
    assert report.reason == PHYSICAL_CONTACT_SWITCH_INVALID
    assert report.base_fingerprint_digest
    assert len(report.sample_fingerprint_digests) == 2


def test_activation_branch_crossing_is_rejected_as_piecewise():
    fixture = _fixtures()["S3"]
    raw = fixture.snapshot.previous_accepted_action.copy()
    raw[0] = 0.18
    previous = fixture.snapshot.previous_accepted_action.copy()
    previous[0] = 0.18
    a_plus = fixture.snapshot.a_plus.copy()
    a_plus[0] = 0.18
    result = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=replace(
            fixture.snapshot,
            previous_accepted_action=previous,
            a_plus=a_plus,
        ),
        raw_action=raw,
        state_steps=ML241_STATE_STEPS,
        action_steps=ML241_ACTION_STEP,
        state_columns=[51],
        action_columns=[],
        allow_nonsmooth=True,
    )
    report = result.state_columns[0]
    assert not report.valid
    assert report.reason == PIECEWISE_BRANCH_SWITCH_INVALID
    assert len(report.sample_fingerprint_digests) == 2


def test_previous_action_slew_boundary_is_rejected_as_piecewise():
    fixture = _fixtures()["S3"]
    raw = fixture.snapshot.previous_accepted_action.copy()
    raw[0] = fixture.snapshot.previous_accepted_action[0] + 0.20
    result = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=raw,
        state_steps=ML241_STATE_STEPS,
        action_steps=ML241_ACTION_STEP,
        state_columns=[96],
        action_columns=[],
        allow_nonsmooth=True,
    )
    report = result.state_columns[0]
    assert not report.valid
    assert report.reason == PIECEWISE_BRANCH_SWITCH_INVALID
    assert len(report.sample_fingerprint_digests) == 2


def test_same_branch_continuous_transition_values_are_not_fingerprinted():
    fixture = _fixtures()["S3"]
    raw = _witness_action(fixture.snapshot)
    result = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=raw,
        state_steps=ML241_STATE_STEPS,
        action_steps=ML241_ACTION_STEP,
        state_columns=[],
        action_columns=[0],
    )
    varied_qvel = np.asarray(fixture.snapshot.qvel, dtype=np.float64).copy()
    varied_qvel[0] += 1e-7
    varied = linearize_step_5ms(
        plant=fixture.plant,
        base_snapshot=replace(fixture.snapshot, qvel=varied_qvel),
        raw_action=raw,
        state_steps=ML241_STATE_STEPS,
        action_steps=ML241_ACTION_STEP,
        state_columns=[],
        action_columns=[0],
    )
    assert result.action_columns[0].valid
    assert varied.action_columns[0].valid
    assert result.action_columns[0].active_set_preserved
    assert varied.action_columns[0].active_set_preserved
    assert result.base_active_set == varied.base_active_set
    assert result.base_next_snapshot_digest != varied.base_next_snapshot_digest
    assert np.isfinite(result.B[:, 0]).all()
    assert np.isfinite(varied.B[:, 0]).all()


def test_solver_constraint_row_order_is_not_physical_limit_identity():
    import mujoco

    limit = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    other = int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    first = type("FakeData", (), {"nefc": 2, "efc_type": np.asarray([limit, other], dtype=np.int32)})()
    second = type("FakeData", (), {"nefc": 2, "efc_type": np.asarray([other, limit], dtype=np.int32)})()
    assert _native_joint_limits(first) is True
    assert _native_joint_limits(second) is True
