"""Focused ML-242 tests for the exact-discrete transcription structure."""

from __future__ import annotations

from dataclasses import replace
import inspect
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from loaded_cmj.oracle import transcription
from loaded_cmj.oracle.transcription import (
    ACTION_DIMENSION,
    A_MINUS_TANGENT_SLICE,
    A_PLUS_TANGENT_SLICE,
    CONTROL_DT_S,
    DirectMultipleShootingProblem,
    DerivativeContractError,
    FULL_ACTIVE_ACTION_INDICES,
    FULL_LAYOUT,
    PREVIOUS_ACCEPTED_ACTION_TANGENT_SLICE,
    SegmentSpec,
    STATE_DIMENSION,
    TranscriptionError,
    WITNESS_FIXED_ACTION_INDICES,
    WITNESS_FREE_ACTION_INDICES,
    WITNESS_LAYOUT,
    advance_snapshot_exact,
    boxminus_endpoint_jacobians,
    qualify_endpoint_directional_maps,
)
from loaded_cmj.oracle.derivatives import (
    CONSTRAINT_CATALOG_ID,
    QACC_DERIVATIVE_NUMERICALLY_NULL,
    TANGENT_LAYOUT_ID,
    snapshot_digest,
)
from loaded_cmj.simulation.snapshot import SNAPSHOT_SCHEMA_VERSION

sys.path.insert(0, str(Path(__file__).parent))
from test_ml238_macro_state import fixtures as ml238_fixtures


@pytest.fixture(scope="module")
def qualified_fixtures():
    return ml238_fixtures.__wrapped__()


def _raw_sequence(fixture, horizon: int) -> tuple[np.ndarray, ...]:
    first = np.asarray(fixture.raw_action, dtype=np.float64).copy()
    second = np.asarray((0.04, -0.03, 0.02) * 5, dtype=np.float64)
    return tuple((first if index % 2 == 0 else second).copy() for index in range(horizon))


def _problem(qualified_fixtures, horizon: int = 1, **kwargs):
    fixture = qualified_fixtures["S3"]
    candidate_actions = _raw_sequence(fixture, horizon)
    action_layout = kwargs.get("action_layout", FULL_LAYOUT)
    if action_layout == FULL_LAYOUT:
        raw_actions = candidate_actions
        actions = raw_actions
    else:
        active = np.asarray(WITNESS_FREE_ACTION_INDICES, dtype=np.int64)
        raw_actions = tuple(
            np.where(np.isin(np.arange(ACTION_DIMENSION), active), action, 0.0)
            for action in candidate_actions
        )
        actions = tuple(action[active].copy() for action in raw_actions)
    references = [fixture.snapshot]
    for action in raw_actions:
        references.append(
            advance_snapshot_exact(
                plant=fixture.plant,
                snapshot=references[-1],
                raw_action=action,
            )
        )
    return DirectMultipleShootingProblem(
        plant=fixture.plant,
        reference_snapshots=references,
        **kwargs,
    ), actions


def _zero_decision(problem, actions):
    return problem.pack_decision(
        [np.zeros(STATE_DIMENSION) for _ in range(problem.horizon)],
        actions,
    )


def _native_reference_problem(fixture, attribute: str, value: float):
    current = fixture.snapshot
    updated = np.full_like(np.asarray(getattr(current, attribute)), value, dtype=np.float64)
    initial = replace(current, **{attribute: updated})
    next_snapshot = replace(initial, time=initial.time + CONTROL_DT_S)
    return _problem_with_refs(fixture, (initial, next_snapshot))


def _r2_problem(qualified_fixtures, *, action_layout: str = FULL_LAYOUT):
    return _problem(
        qualified_fixtures,
        40,
        action_layout=action_layout,
        segments=(
            SegmentSpec(
                0,
                40,
                "SUPPORTED_CONTACT",
                ("E3_E4_HORIZONTAL", "E3_E4_REVERSAL", "SUPPORT_MARGIN"),
            ),
        ),
        include_elastic_slacks=True,
    )


def test_decision_layout_scales_and_slices():
    one = transcription.DecisionLayout(1)
    forty = transcription.DecisionLayout(40)
    assert one.base_dimension == STATE_DIMENSION + ACTION_DIMENSION
    assert forty.base_dimension == 40 * (STATE_DIMENSION + ACTION_DIMENSION)
    with pytest.raises(TranscriptionError):
        one.state_slice(0)
    assert one.action_slice(0) == slice(0, ACTION_DIMENSION)
    assert one.state_slice(1) == slice(ACTION_DIMENSION, ACTION_DIMENSION + STATE_DIMENSION)
    assert forty.state_slice(40) == slice(40 * (STATE_DIMENSION + ACTION_DIMENSION) - STATE_DIMENSION, forty.base_dimension)

    witness = transcription.DecisionLayout(
        1,
        active_action_indices=WITNESS_FREE_ACTION_INDICES,
        fixed_action_values=transcription.WITNESS_FIXED_ACTION_VALUES,
        layout_id=WITNESS_LAYOUT,
    )
    assert witness.active_action_dimension == len(WITNESS_FREE_ACTION_INDICES) == 7
    assert witness.base_dimension == STATE_DIMENSION + len(WITNESS_FREE_ACTION_INDICES)
    assert witness.action_slice(0) == slice(0, 7)
    assert witness.state_slice(1) == slice(7, 139)
    assert witness.fixed_action_values == tuple((index, 0.0) for index in WITNESS_FIXED_ACTION_INDICES)


def test_pack_unpack_and_invalid_inputs(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1)
    state = [np.full(STATE_DIMENSION, 1.0e-7)]
    z = problem.pack_decision(state, actions)
    unpacked = problem.unpack_decision(z)
    np.testing.assert_array_equal(unpacked.states[0], state[0])
    np.testing.assert_array_equal(unpacked.actions[0], actions[0])
    np.testing.assert_array_equal(problem.pack_decision(unpacked.states, unpacked.actions), z)
    with pytest.raises(TranscriptionError):
        problem.pack_decision([], actions)
    with pytest.raises(TranscriptionError):
        problem.unpack_decision(np.zeros(problem.variable_count - 1))
    with pytest.raises(TranscriptionError):
        problem.unpack_decision(np.full(problem.variable_count, np.nan))


def test_reference_count_and_time_schedule_guards(qualified_fixtures):
    fixture = qualified_fixtures["S3"]
    with pytest.raises(TranscriptionError):
        DirectMultipleShootingProblem(plant=fixture.plant, reference_snapshots=(fixture.snapshot,))
    broken = replace(fixture.snapshot, time=fixture.snapshot.time + 1.0e-3)
    with pytest.raises(TranscriptionError):
        _problem_with_refs(fixture, (fixture.snapshot, broken))


def _problem_with_refs(fixture, references, **kwargs):
    return DirectMultipleShootingProblem(
        plant=fixture.plant,
        reference_snapshots=references,
        **kwargs,
    )


@pytest.mark.parametrize("horizon", (1, 5, 10))
def test_exact_zero_defect_replay(qualified_fixtures, horizon):
    problem, actions = _problem(qualified_fixtures, horizon)
    defect = problem.defect_values(_zero_decision(problem, actions))
    assert np.isfinite(defect).all()
    assert float(np.max(np.abs(defect))) < 1.0e-10


def test_fixed_initial_bounds_and_raw_action_bounds(qualified_fixtures):
    problem, _ = _problem(qualified_fixtures, 1)
    lower = problem.variable_lower_bounds()
    upper = problem.variable_upper_bounds()
    np.testing.assert_array_equal(lower[problem.action_slice(0)], -np.ones(ACTION_DIMENSION))
    np.testing.assert_array_equal(upper[problem.action_slice(0)], np.ones(ACTION_DIMENSION))
    with pytest.raises(TranscriptionError):
        problem.state_slice(0)
    assert problem.initial_state is problem.reference_snapshots[0]
    assert problem.decision_layout_record()["initial_state_parameter"]["is_nlp_decision"] is False


def test_x0_is_an_exact_parameter_and_not_a_fixed_decision(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1)
    assert problem.layout.state_knot_indices == (1,)
    assert len(problem.unpack_decision(_zero_decision(problem, actions)).states) == 1
    with pytest.raises(TranscriptionError):
        problem.state_slice(0)
    with pytest.raises(TranscriptionError):
        DirectMultipleShootingProblem(
            plant=problem.plant,
            reference_snapshots=problem.reference_snapshots,
            fixed_initial_state=False,
        )

    z = _zero_decision(problem, actions)
    z[problem.action_slice(0)] = np.linspace(-0.2, 0.2, ACTION_DIMENSION)
    evaluation = problem.interval_evaluation(z, 0)
    assert evaluation.state is problem.initial_state
    assert snapshot_digest(evaluation.state) == snapshot_digest(problem.reference_snapshots[0])


@pytest.mark.parametrize(
    ("attribute", "tangent_slice", "physical_lower", "physical_upper"),
    (
        ("a_plus", A_PLUS_TANGENT_SLICE, 0.0, 1.0),
        ("a_minus", A_MINUS_TANGENT_SLICE, 0.0, 1.0),
        ("previous_accepted_action", PREVIOUS_ACCEPTED_ACTION_TANGENT_SLICE, -1.0, 1.0),
    ),
)
@pytest.mark.parametrize("reference", (0.0, 0.25, 1.0))
def test_native_euclidean_tangent_bounds(
    qualified_fixtures,
    attribute,
    tangent_slice,
    physical_lower,
    physical_upper,
    reference,
):
    fixture = qualified_fixtures["S3"]
    problem = _native_reference_problem(fixture, attribute, reference)
    lower = problem.variable_lower_bounds()[problem.state_slice(1)]
    upper = problem.variable_upper_bounds()[problem.state_slice(1)]
    start, stop = tangent_slice.start, tangent_slice.stop
    np.testing.assert_allclose(lower[start:stop], physical_lower - reference)
    np.testing.assert_allclose(upper[start:stop], physical_upper - reference)


@pytest.mark.parametrize(
    ("attribute", "invalid"),
    (
        ("a_plus", -1.0e-6),
        ("a_plus", 1.0 + 1.0e-6),
        ("a_minus", -1.0e-6),
        ("a_minus", 1.0 + 1.0e-6),
        ("previous_accepted_action", -1.0 - 1.0e-6),
        ("previous_accepted_action", 1.0 + 1.0e-6),
    ),
)
def test_native_euclidean_reference_domain_rejects_out_of_range(qualified_fixtures, attribute, invalid):
    fixture = qualified_fixtures["S3"]
    with pytest.raises(TranscriptionError):
        _native_reference_problem(fixture, attribute, invalid)


def test_manifold_and_owner_defined_coordinates_are_not_box_bounded(qualified_fixtures):
    problem, _ = _problem(qualified_fixtures, 1)
    lower = problem.variable_lower_bounds()[problem.state_slice(1)]
    upper = problem.variable_upper_bounds()[problem.state_slice(1)]
    for start, stop in ((0, 51), (81, 96), (111, 132)):
        assert np.isneginf(lower[start:stop]).all()
        assert np.isposinf(upper[start:stop]).all()


def test_full_action_layout_reconstructs_the_exact_15d_input(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1, action_layout=FULL_LAYOUT)
    assert problem.active_action_indices == FULL_ACTIVE_ACTION_INDICES
    assert problem.fixed_action_indices == ()
    np.testing.assert_array_equal(problem.reconstruct_action(actions[0]), actions[0])
    evaluation = problem.interval_evaluation(_zero_decision(problem, actions), 0)
    np.testing.assert_array_equal(evaluation.raw_action, actions[0])


def test_witness_action_layout_eliminates_fixed_channels_and_reconstructs_exact_zeros(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1, action_layout=WITNESS_LAYOUT)
    assert problem.active_action_indices == WITNESS_FREE_ACTION_INDICES
    assert problem.fixed_action_indices == WITNESS_FIXED_ACTION_INDICES
    assert problem.layout.active_action_dimension == 7
    raw = problem.reconstruct_action(actions[0])
    np.testing.assert_array_equal(raw[np.asarray(WITNESS_FREE_ACTION_INDICES)], actions[0])
    np.testing.assert_array_equal(raw[np.asarray(WITNESS_FIXED_ACTION_INDICES)], np.zeros(8))
    with pytest.raises(TranscriptionError):
        problem.pack_decision([np.zeros(STATE_DIMENSION)], [_raw_sequence(qualified_fixtures["S3"], 1)[0]])
    evaluation = problem.interval_evaluation(_zero_decision(problem, actions), 0)
    np.testing.assert_array_equal(evaluation.raw_action, raw)
    np.testing.assert_allclose(problem.defect_values(_zero_decision(problem, actions)), 0.0, atol=1.0e-10)


def test_witness_derivative_request_excludes_fixed_channels(qualified_fixtures):
    base_problem, actions = _problem(qualified_fixtures, 1, action_layout=WITNESS_LAYOUT)
    fixture = qualified_fixtures["S3"]
    requested = []

    def provider(**kwargs):
        requested.append(tuple(kwargs["action_columns"]))
        active = set(WITNESS_FREE_ACTION_INDICES)
        branches = tuple("INTERIOR" if index in active else "NEAR_KINK" for index in range(ACTION_DIMENSION))
        validity = np.array([index in active for index in range(ACTION_DIMENSION)], dtype=bool)
        B = np.zeros((STATE_DIMENSION, ACTION_DIMENSION))
        B[:, np.asarray(WITNESS_FIXED_ACTION_INDICES)] = np.nan
        A = np.eye(STATE_DIMENSION)
        A[:, 111:132] = 0.0
        return SimpleNamespace(
            A=A,
            B=B,
            tangent_layout_id=TANGENT_LAYOUT_ID,
            snapshot_schema_id=SNAPSHOT_SCHEMA_VERSION,
            transition_owner="src/loaded_cmj/simulation/transition.py:step_5ms",
            constraint_catalog_id=CONSTRAINT_CATALOG_ID,
            qacc_derivative_disposition=QACC_DERIVATIVE_NUMERICALLY_NULL,
            qacc_zero_columns=tuple(range(111, 132)),
            qacc_absolute_error_bound=(
                ("qacc_rotation_joint", 1.0e-7),
                ("qacc_translation", 1.0e-7),
            ),
            qacc_certificate_evidence_id="ML241-test-certificate",
            base_active_set=SimpleNamespace(action_branches=branches),
            base_snapshot_digest=snapshot_digest(kwargs["base_snapshot"]),
            base_raw_action=np.asarray(kwargs["raw_action"]),
            state_validity=np.ones(STATE_DIMENSION, dtype=bool),
            action_validity=validity,
            scheme="central_boxminus_at_common_y0",
            state_step_metadata=dict(transcription.ML241_STATE_STEPS),
            action_step_metadata=(1.0e-4,) * ACTION_DIMENSION,
        )

    problem = DirectMultipleShootingProblem(
        plant=fixture.plant,
        reference_snapshots=base_problem.reference_snapshots,
        action_layout=WITNESS_LAYOUT,
        derivative_provider=provider,
    )
    evaluation = problem.interval_evaluation(_zero_decision(problem, actions), 0)
    result = problem._derivatives(evaluation.state, evaluation.raw_action)
    assert requested == [WITNESS_FREE_ACTION_INDICES]
    assert np.isfinite(result.B[:, np.asarray(WITNESS_FREE_ACTION_INDICES)]).all()
    assert np.isnan(result.B[:, np.asarray(WITNESS_FIXED_ACTION_INDICES)]).all()


@pytest.mark.parametrize("action_layout", (FULL_LAYOUT, WITNESS_LAYOUT))
def test_round_trip_pack_unpack_pack_for_both_layouts(qualified_fixtures, action_layout):
    problem, actions = _problem(qualified_fixtures, 3, action_layout=action_layout)
    states = [np.full(STATE_DIMENSION, 1.0e-8 * (index + 1)) for index in range(problem.horizon)]
    z = problem.pack_decision(states, actions)
    unpacked = problem.unpack_decision(z)
    np.testing.assert_array_equal(problem.pack_decision(unpacked.states, unpacked.actions), z)


def test_r2_full_and_witness_sparse_receipts_are_derived_from_ownership(qualified_fixtures):
    full, _ = _r2_problem(qualified_fixtures, action_layout=FULL_LAYOUT)
    witness, _ = _r2_problem(qualified_fixtures, action_layout=WITNESS_LAYOUT)

    def expected_dynamic_nnz(problem):
        active = problem.layout.active_action_dimension
        return sum(
            STATE_DIMENSION
            * (active + STATE_DIMENSION if interval == 0 else 2 * STATE_DIMENSION + active)
            for interval in range(problem.horizon)
        )

    def expected_extra_nnz(problem):
        total = 0
        for row in problem.constraint_rows[problem.dynamics_row_count :]:
            for kind, index in row.support:
                if kind == "state":
                    total += 0 if index == 0 else STATE_DIMENSION
                elif kind == "action":
                    total += problem.layout.active_action_dimension
                else:
                    raise AssertionError(f"unexpected support kind: {kind}")
            total += row.slack_decision_index is not None
        return total

    for problem in (full, witness):
        rows, cols = problem.jacobian_structure()
        assert rows.size == cols.size
        assert rows.size == expected_dynamic_nnz(problem) + expected_extra_nnz(problem)
        assert problem.constraint_count == problem.dynamics_row_count + 43
        assert problem.variable_count == problem.layout.base_dimension + problem.elastic_slack_count
        assert problem.jacobian_structure_hash() == problem.jacobian_structure_hash()

    assert full.variable_count == 5922
    assert full.constraint_count == 5323
    assert full.jacobian_structure()[0].size == 1461282
    assert witness.variable_count == 5602
    assert witness.constraint_count == 5323
    assert witness.jacobian_structure()[0].size == 1419042

    full_rows, full_cols = full.jacobian_structure()
    witness_rows, witness_cols = witness.jacobian_structure()
    first_full_cols = set(full_cols[: STATE_DIMENSION * (ACTION_DIMENSION + STATE_DIMENSION)])
    assert first_full_cols == (
        set(range(full.action_slice(0).start, full.action_slice(0).stop))
        | set(range(full.state_slice(1).start, full.state_slice(1).stop))
    )
    with pytest.raises(TranscriptionError):
        full.state_slice(0)
    assert not set(witness.active_action_indices).intersection(WITNESS_FIXED_ACTION_INDICES)
    assert witness_rows.size < full_rows.size
    assert witness_cols.size == full_cols.size - 40 * STATE_DIMENSION * len(WITNESS_FIXED_ACTION_INDICES)


def test_problem_reference_and_structure_are_immutable(qualified_fixtures):
    problem, _ = _problem(qualified_fixtures, 1)
    with pytest.raises(AttributeError):
        problem.horizon = 2
    assert isinstance(problem.reference_snapshots, tuple)
    assert isinstance(problem.reference_time_schedule, tuple)


def test_defect_sign_and_numerical_state_constraints(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1)
    z = _zero_decision(problem, actions)
    next_state = problem.state_slice(1)
    delta = np.zeros(STATE_DIMENSION)
    delta[21] = 2.0e-4
    delta[111] = 3.0e-4
    z[next_state] = delta
    defect = problem.defect_values(z)
    np.testing.assert_allclose(defect[21], delta[21], atol=1.0e-10)
    np.testing.assert_allclose(defect[111], delta[111], atol=1.0e-10)
    z[next_state] = 0.0
    np.testing.assert_allclose(problem.defect_values(z), 0.0, atol=1.0e-10)


def test_endpoint_jacobians_zero_and_nonzero_local(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1)
    evaluation = problem.interval_evaluation(_zero_decision(problem, actions), 0)
    zero = boxminus_endpoint_jacobians(
        next_snapshot=evaluation.next_state,
        predicted_snapshot=evaluation.predicted_state,
        model=problem.plant.model,
    )
    assert zero.zero_defect
    assert zero.pred_identity_error < 1.0e-6
    assert zero.next_identity_error < 1.0e-6
    shifted = replace(evaluation.next_state, qvel=evaluation.next_state.qvel + np.full(21, 1.0e-4))
    nonzero = boxminus_endpoint_jacobians(
        next_snapshot=shifted,
        predicted_snapshot=evaluation.predicted_state,
        model=problem.plant.model,
    )
    assert not nonzero.zero_defect
    check = qualify_endpoint_directional_maps(
        result=nonzero,
        next_snapshot=shifted,
        predicted_snapshot=evaluation.predicted_state,
        model=problem.plant.model,
        directions=(np.ones(STATE_DIMENSION), np.arange(1, STATE_DIMENSION + 1, dtype=np.float64)),
    )
    assert check.pred_max_error < 1.0e-6
    assert check.next_max_error < 1.0e-6


def test_block_banded_dynamics_structure_and_hash(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 2)
    rows, cols = problem.jacobian_structure()
    first_width = ACTION_DIMENSION + STATE_DIMENSION
    later_width = STATE_DIMENSION + ACTION_DIMENSION + STATE_DIMENSION
    assert rows.size == STATE_DIMENSION * (first_width + later_width)
    first_action = problem.action_slice(0)
    next_state = problem.state_slice(1)
    support = set(cols[:first_width])
    assert support == set(range(first_action.start, first_action.stop)) | set(range(next_state.start, next_state.stop))
    second_start = STATE_DIMENSION * first_width
    second_rows = rows[second_start:]
    second_cols = cols[second_start:]
    assert second_rows.min() == STATE_DIMENSION
    assert second_cols.max() < problem.state_slice(2).stop
    assert problem.jacobian_structure_hash() == problem.jacobian_structure_hash()


def test_qacc_next_knot_columns_are_structurally_present(qualified_fixtures):
    problem, _ = _problem(qualified_fixtures, 2)
    rows, cols = problem.jacobian_structure()
    qacc_next = set(range(problem.state_slice(1).start + 111, problem.state_slice(1).start + 132))
    assert qacc_next.issubset(set(cols))
    for interval in range(problem.horizon):
        block_width = (
            ACTION_DIMENSION + STATE_DIMENSION
            if interval == 0
            else STATE_DIMENSION + ACTION_DIMENSION + STATE_DIMENSION
        )
        block_start = sum(
            STATE_DIMENSION
            * (
                ACTION_DIMENSION + STATE_DIMENSION
                if prior == 0
                else ACTION_DIMENSION + 2 * STATE_DIMENSION
            )
            for prior in range(interval)
        )
        block_end = block_start + STATE_DIMENSION * block_width
        block_cols = set(cols[block_start:block_end])
        expected = set(range(problem.state_slice(interval + 1).start + 111, problem.state_slice(interval + 1).start + 132))
        assert expected.issubset(block_cols)


def test_constraint_schema_excludes_diagnostics_and_keeps_terminal_metadata(qualified_fixtures):
    problem, _ = _problem(
        qualified_fixtures,
        1,
        segments=(
            SegmentSpec(
                0,
                1,
                "SUPPORTED_CONTACT",
                ("CONTACT_COP_SUPPORT_GEOMETRY", "SUPPORT_MARGIN", "E3_E4_REVERSAL", "E3_E4_HORIZONTAL", "RECOVERY_CAPTURABILITY"),
            ),
        ),
    )
    ids = [row.constraint_id for row in problem.constraint_rows]
    assert "RECOVERY_CAPTURABILITY" not in ids
    assert "E3_E4_REVERSAL" in ids
    assert "E3_E4_HORIZONTAL" in ids
    schema = problem.constraint_schema_record()
    assert schema["capturability"] == "diagnostic_only"
    terminal = {item["constraint_id"]: item["representation"] for item in schema["terminal_items"]}
    assert terminal["E3_E4_SUPPORTED"] == "discrete_or_postcheck_guard_metadata"
    assert terminal["E3_E4_REVERSAL"] == "explicit_smooth_terminal_residual"


def test_approved_slacks_are_schema_only_or_nonnegative(qualified_fixtures):
    problem, _ = _problem(
        qualified_fixtures,
        1,
        segments=(SegmentSpec(0, 1, "SUPPORTED_CONTACT", ("SUPPORT_MARGIN", "E3_E4_REVERSAL", "E3_E4_HORIZONTAL")),),
        include_elastic_slacks=True,
    )
    assert problem.elastic_slack_count == 3
    assert all(item["elastic_use"] for item in problem.elastic_slack_schema)
    assert "E3_E4_REVERSAL" not in {item["constraint_id"] for item in problem.elastic_slack_schema}
    reversal_rows = [row for row in problem.constraint_rows if row.constraint_id == "E3_E4_REVERSAL"]
    assert len(reversal_rows) == 1
    assert reversal_rows[0].group == "smooth_terminal"
    assert reversal_rows[0].slack_decision_index is None
    with pytest.raises(TranscriptionError):
        problem.pack_decision((np.zeros(132), np.zeros(132)), ((0.0,) * 15,), (-1.0,) * 3)
    assert np.all(problem.variable_lower_bounds()[problem.layout.base_dimension:] == 0.0)


def test_ml241_metadata_and_invalid_columns_are_not_silently_zeroed(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1)
    fixture = qualified_fixtures["S3"]
    base = problem.reference_snapshots[0]
    invalid = SimpleNamespace(
        A=np.zeros((132, 132)),
        B=np.zeros((132, 15)),
        tangent_layout_id=TANGENT_LAYOUT_ID,
        snapshot_schema_id=SNAPSHOT_SCHEMA_VERSION,
        transition_owner="src/loaded_cmj/simulation/transition.py:step_5ms",
        constraint_catalog_id=CONSTRAINT_CATALOG_ID,
        qacc_derivative_disposition=QACC_DERIVATIVE_NUMERICALLY_NULL,
        qacc_zero_columns=tuple(range(111, 132)),
        qacc_absolute_error_bound=(
            ("qacc_rotation_joint", 1.0e-7),
            ("qacc_translation", 1.0e-7),
        ),
        qacc_certificate_evidence_id="ML241-test-certificate",
        base_active_set=SimpleNamespace(action_branches=("INTERIOR",) * ACTION_DIMENSION),
        base_snapshot_digest=snapshot_digest(base),
        base_raw_action=np.asarray(actions[0]),
        state_validity=np.ones(132, dtype=bool),
        action_validity=np.zeros(15, dtype=bool),
        scheme="central_boxminus_at_common_y0",
        state_step_metadata=dict(transcription.ML241_STATE_STEPS),
        action_step_metadata=(1.0e-4,) * ACTION_DIMENSION,
    )
    with pytest.raises(DerivativeContractError):
        problem_with_provider = DirectMultipleShootingProblem(
            plant=fixture.plant,
            reference_snapshots=problem.reference_snapshots,
            derivative_provider=lambda **_: invalid,
        )
        problem_with_provider.jacobian_values(_zero_decision(problem_with_provider, actions))


def test_qualified_zero_input_columns_do_not_remove_next_knot_dependency(qualified_fixtures):
    problem, actions = _problem(qualified_fixtures, 1)
    fixture = qualified_fixtures["S3"]

    def qualified_provider(**kwargs):
        state = kwargs["base_snapshot"]
        action = np.asarray(kwargs["raw_action"], dtype=np.float64)
        A = np.eye(STATE_DIMENSION)
        A[:, 111:132] = 0.0
        return SimpleNamespace(
            A=A,
            B=np.zeros((STATE_DIMENSION, ACTION_DIMENSION)),
            tangent_layout_id=TANGENT_LAYOUT_ID,
            snapshot_schema_id=SNAPSHOT_SCHEMA_VERSION,
            transition_owner="src/loaded_cmj/simulation/transition.py:step_5ms",
            constraint_catalog_id=CONSTRAINT_CATALOG_ID,
            qacc_derivative_disposition=QACC_DERIVATIVE_NUMERICALLY_NULL,
            qacc_zero_columns=tuple(range(111, 132)),
            qacc_absolute_error_bound=(
                ("qacc_rotation_joint", 1.0e-7),
                ("qacc_translation", 1.0e-7),
            ),
            qacc_certificate_evidence_id="ML241-test-certificate",
            base_active_set=SimpleNamespace(action_branches=("INTERIOR",) * ACTION_DIMENSION),
            base_snapshot_digest=snapshot_digest(state),
            base_raw_action=action,
            state_validity=np.ones(STATE_DIMENSION, dtype=bool),
            action_validity=np.ones(ACTION_DIMENSION, dtype=bool),
            scheme="central_boxminus_at_common_y0",
            state_step_metadata=dict(transcription.ML241_STATE_STEPS),
            action_step_metadata=(1.0e-4,) * ACTION_DIMENSION,
        )

    problem = DirectMultipleShootingProblem(
        plant=fixture.plant,
        reference_snapshots=problem.reference_snapshots,
        derivative_provider=qualified_provider,
    )
    z = _zero_decision(problem, actions)
    values = problem.jacobian_values(z)
    block_width = ACTION_DIMENSION + STATE_DIMENSION
    row = 111
    row_values = values[row * block_width : (row + 1) * block_width]
    assert np.max(np.abs(row_values[:ACTION_DIMENSION])) < 1.0e-8
    assert abs(row_values[ACTION_DIMENSION + 111] - 1.0) < 1.0e-8


def test_source_boundary_has_no_solver_or_physical_bypass():
    source = inspect.getsource(transcription)
    assert "mujoco.mj_step" not in source
    assert "cyipopt" not in source
    assert "Ipopt" not in source
    assert "scipy" not in source.lower()
    assert "reward" not in source.lower()
    assert "step_5ms(" in source
