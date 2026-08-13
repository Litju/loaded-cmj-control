"""G5 Phase-I rho epigraph contract tests."""

from __future__ import annotations

from hashlib import sha256
from types import SimpleNamespace

import numpy as np
import pytest

from loaded_cmj.oracle.derivatives import (
    CONSTRAINT_CATALOG_ID,
    QACC_DERIVATIVE_NUMERICALLY_NULL,
    TANGENT_LAYOUT_ID,
    snapshot_digest,
)
from loaded_cmj.oracle.transcription import ACTION_DIMENSION, ML241_ACTION_STEP, ML241_STATE_STEPS
from loaded_cmj.simulation.snapshot import SNAPSHOT_SCHEMA_VERSION
from tests.test_ml242_transcription import (
    FULL_LAYOUT,
    STATE_DIMENSION,
    WITNESS_LAYOUT,
    SegmentSpec,
    _problem,
    _r2_problem,
    qualified_fixtures,
)


PRE_G5_STRUCTURE_HASH = {
    FULL_LAYOUT: "3d8ba72e23559d5c236c4be0734704f20efbfd0cbc181536b4b82f838d32a3de",
    WITNESS_LAYOUT: "95881e8313b5ffbb10aa86e5e287607fc9de781f0f5afab452de853b6325c156",
}
PRE_G5_ROW_METADATA_DIGEST = {
    FULL_LAYOUT: "2d9f9175c50ce8fb7827d717fca381639416a4c07b0bbbc90e0f2a32a742d195",
    WITNESS_LAYOUT: "c55060aa061fc4cc0d38cd63ec25c2690e7ad7f409427f08378297c0ec74aaa8",
}
PRE_G5_BOUNDS_DIGEST = {
    FULL_LAYOUT: "264cd5fd261bd4a452032ef989b747cd595203543887373fa08ca38fc6523417",
    WITNESS_LAYOUT: "8f6b9ac9f0595431edf46cbc1ad0668dc816e5d3151479c6104d39de070ac04e",
}
PHASE_I_SCALE_M = 1.0


@pytest.fixture(scope="module")
def g5_problems():
    fixtures = qualified_fixtures.__wrapped__()
    return {
        layout: _r2_problem(fixtures, action_layout=layout)
        for layout in (FULL_LAYOUT, WITNESS_LAYOUT)
    }


def _physical_rows(problem):
    return tuple(row for row in problem.constraint_rows if row.group != "phase_i_epigraph")


def _zero_decision(problem, actions):
    return problem.pack_decision(
        [np.zeros(STATE_DIMENSION) for _ in range(problem.horizon)],
        actions,
        np.zeros(problem.elastic_slack_count),
        rho=0.0,
    )


def _catalog_evaluator(row, _z, _problem):
    if row.group == "phase_i_epigraph":
        raise AssertionError("epigraph rows must not reach the physical evaluator")
    return 0.0


def _zero_catalog_jacobian(row, _z, problem):
    width = 0
    for kind, index in row.support:
        if kind == "state":
            width += 0 if index == 0 else STATE_DIMENSION
        elif kind == "action":
            width += problem.layout.active_action_dimension
    return np.zeros(width + int(row.slack_decision_index is not None))


@pytest.mark.parametrize(
    ("layout", "expected_variables", "expected_rho_column"),
    ((FULL_LAYOUT, 5923, 5922), (WITNESS_LAYOUT, 5603, 5602)),
)
def test_one_rho_is_appended_after_existing_slacks(
    g5_problems, layout, expected_variables, expected_rho_column
):
    problem, _ = g5_problems[layout]

    assert problem.variable_count == expected_variables
    assert problem.rho_variable_count == 1
    assert problem.rho_decision_index == expected_rho_column
    assert problem.layout.rho_decision_index == expected_rho_column
    assert problem.layout.slack_specs[-1].decision_index == expected_rho_column - 1


@pytest.mark.parametrize("layout", (FULL_LAYOUT, WITNESS_LAYOUT))
def test_rho_bounds_are_zero_to_infinity_and_existing_bounds_are_unchanged(g5_problems, layout):
    problem, _ = g5_problems[layout]
    lower = problem.variable_lower_bounds()
    upper = problem.variable_upper_bounds()
    rho = problem.rho_decision_index

    assert lower[rho] == 0.0
    assert np.isposinf(upper[rho])
    assert np.all(lower[[slack.decision_index for slack in problem.layout.slack_specs]] == 0.0)
    assert np.all(
        np.isposinf(upper[[slack.decision_index for slack in problem.layout.slack_specs]])
    )
    assert sha256(lower[:-1].tobytes() + upper[:-1].tobytes()).hexdigest() == PRE_G5_BOUNDS_DIGEST[layout]


@pytest.mark.parametrize("layout", (FULL_LAYOUT, WITNESS_LAYOUT))
def test_exactly_42_epigraph_rows_cover_each_existing_slack_once(g5_problems, layout):
    problem, _ = g5_problems[layout]
    slacks = problem.layout.slack_specs
    rows = problem.phase_i_epigraph_rows

    assert problem.elastic_slack_count == 42
    assert problem.phase_i_epigraph_row_count == 42
    assert len(rows) == 42
    assert [row.slack_decision_index for row in rows] == [slack.decision_index for slack in slacks]
    assert len({row.slack_decision_index for row in rows}) == 42
    assert all(row.rho_decision_index == problem.rho_decision_index for row in rows)
    assert all(row.constraint_id != "E3_E4_REVERSAL" for row in rows)


def test_live_physical_mapping_is_preserved_and_reversal_is_hard(g5_problems):
    problem, _ = g5_problems[FULL_LAYOUT]
    rows = _physical_rows(problem)
    horizontal_slack = next(
        slack for slack in problem.layout.slack_specs if slack.constraint_id == "E3_E4_HORIZONTAL"
    )
    support_slacks = tuple(
        slack for slack in problem.layout.slack_specs if slack.constraint_id == "SUPPORT_MARGIN"
    )

    assert len(rows) == 5323
    horizontal = rows[5280]
    reversal = rows[5281]
    support = rows[5282:5323]
    assert (horizontal.row_index, horizontal.constraint_id, horizontal.knot_index) == (
        5280,
        "E3_E4_HORIZONTAL",
        40,
    )
    assert horizontal.slack_decision_index == horizontal_slack.decision_index
    assert horizontal.support == (("state", 40),)
    assert reversal.slack_decision_index is None
    assert reversal.constraint_id == "E3_E4_REVERSAL"
    assert reversal.knot_index == 40
    assert all(row.constraint_id == "SUPPORT_MARGIN" for row in support)
    assert [row.knot_index for row in support] == list(range(41))
    assert [row.slack_decision_index for row in support] == [
        slack.decision_index for slack in support_slacks
    ]


@pytest.mark.parametrize("layout", (FULL_LAYOUT, WITNESS_LAYOUT))
def test_every_epigraph_uses_one_meter_and_only_its_own_slack_plus_rho(g5_problems, layout):
    problem, _ = g5_problems[layout]
    rows, cols = problem.jacobian_structure()
    epigraph_rows = problem.phase_i_epigraph_rows

    assert problem.phase_i_scale_m == PHASE_I_SCALE_M
    assert {row.phase_i_scale for row in epigraph_rows} == {PHASE_I_SCALE_M}
    assert {row.phase_i_units for row in epigraph_rows} == {"m"}
    for row in epigraph_rows:
        positions = np.flatnonzero(rows == row.row_index)
        assert positions.size == 2
        assert set(cols[positions]) == {row.slack_decision_index, problem.rho_decision_index}


@pytest.mark.parametrize("layout", (FULL_LAYOUT, WITNESS_LAYOUT))
def test_public_records_materialize_the_42_new_rows_and_rho_suffix(g5_problems, layout):
    problem, _ = g5_problems[layout]
    decision_record = problem.decision_layout_record()
    row_record = problem.row_layout_record()

    assert decision_record["rho_variable_count"] == 1
    assert decision_record["rho_decision_index"] == problem.rho_decision_index
    assert decision_record["rho_slice"] == (problem.rho_decision_index, problem.rho_decision_index + 1, 1)
    assert row_record["physical_constraint_row_count"] == 5323
    assert row_record["phase_i_epigraph_row_count"] == 42
    assert len(row_record["rows"]) == problem.constraint_count == 5365
    assert row_record["rows"][5322]["group"] != "phase_i_epigraph"
    assert row_record["rows"][5323]["group"] == "phase_i_epigraph"
    assert row_record["rows"][-1]["row_index"] == 5364
    assert row_record["rows"][-1]["phase_i_scale"] == PHASE_I_SCALE_M
    assert row_record["rows"][-1]["phase_i_units"] == "m"


def test_full_jacobian_is_solver_free_with_a_source_owner_catalog_adapter(qualified_fixtures):
    def derivative_provider(**kwargs):
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
            qacc_certificate_evidence_id="ML241-g5-test-certificate",
            base_active_set=SimpleNamespace(action_branches=("INTERIOR",) * ACTION_DIMENSION),
            base_snapshot_digest=snapshot_digest(state),
            base_raw_action=action,
            state_validity=np.ones(STATE_DIMENSION, dtype=bool),
            action_validity=np.ones(ACTION_DIMENSION, dtype=bool),
            scheme="central_boxminus_at_common_y0",
            state_step_metadata=dict(ML241_STATE_STEPS),
            action_step_metadata=(ML241_ACTION_STEP,) * ACTION_DIMENSION,
        )

    problem, actions = _problem(
        qualified_fixtures,
        1,
        segments=(
            SegmentSpec(
                0,
                1,
                "SUPPORTED_CONTACT",
                ("E3_E4_HORIZONTAL", "E3_E4_REVERSAL", "SUPPORT_MARGIN"),
            ),
        ),
        include_elastic_slacks=True,
        derivative_provider=derivative_provider,
    )
    z = _zero_decision(problem, actions)
    values = problem.jacobian_values(z, catalog_jacobian_evaluator=_zero_catalog_jacobian)

    assert values.shape == problem.jacobian_structure()[0].shape
    assert np.array_equal(
        values[-problem.phase_i_epigraph_row_count * 2 :].reshape(-1, 2),
        np.tile(np.array([-1.0, 1.0]), (problem.phase_i_epigraph_row_count, 1)),
    )


def test_objective_is_rho_and_gradient_is_one_only_at_rho(g5_problems):
    problem, _ = g5_problems[FULL_LAYOUT]
    z = np.zeros(problem.variable_count)
    z[problem.rho_decision_index] = 0.375

    assert problem.objective(z) == 0.375
    gradient = problem.objective_gradient(z)
    assert gradient.shape == (problem.variable_count,)
    assert gradient[problem.rho_decision_index] == 1.0
    assert np.count_nonzero(gradient) == 1


def test_objective_orders_two_otherwise_identical_decisions_by_rho(g5_problems):
    problem, _ = g5_problems[FULL_LAYOUT]
    z_a = np.zeros(problem.variable_count)
    z_b = z_a.copy()
    z_a[problem.rho_decision_index] = 0.125
    z_b[problem.rho_decision_index] = 0.25
    assert problem.objective(z_a) < problem.objective(z_b)


def test_zero_rho_requires_all_existing_slacks_to_be_zero(g5_problems):
    problem, actions = g5_problems[FULL_LAYOUT]
    z = _zero_decision(problem, actions)
    values = problem.constraint_values(z, evaluator=_catalog_evaluator)
    epigraph_values = values[[row.row_index for row in problem.phase_i_epigraph_rows]]
    assert np.all(epigraph_values == 0.0)

    positive_slacks = np.zeros(problem.elastic_slack_count)
    positive_slacks[0] = 1.0e-6
    z_positive = problem.pack_decision(
        [np.zeros(STATE_DIMENSION) for _ in range(problem.horizon)],
        actions,
        positive_slacks,
        rho=0.0,
    )
    values_positive = problem.constraint_values(z_positive, evaluator=_catalog_evaluator)
    assert values_positive[problem.phase_i_epigraph_rows[0].row_index] == -1.0e-6


def test_positive_rho_is_a_shared_one_meter_slack_bound(g5_problems):
    problem, actions = g5_problems[FULL_LAYOUT]
    rho = 0.25
    slacks = np.linspace(0.0, rho, problem.elastic_slack_count)
    z = problem.pack_decision(
        [np.zeros(STATE_DIMENSION) for _ in range(problem.horizon)],
        actions,
        slacks,
        rho=rho,
    )
    values = problem.constraint_values(z, evaluator=_catalog_evaluator)
    epigraph_values = values[[row.row_index for row in problem.phase_i_epigraph_rows]]
    assert np.allclose(epigraph_values, rho - slacks, atol=0.0, rtol=0.0)
    assert np.all(slacks <= rho * PHASE_I_SCALE_M)

    too_large = slacks.copy()
    too_large[-1] = rho + 1.0e-6
    z_too_large = problem.pack_decision(
        [np.zeros(STATE_DIMENSION) for _ in range(problem.horizon)],
        actions,
        too_large,
        rho=rho,
    )
    values_too_large = problem.constraint_values(z_too_large, evaluator=_catalog_evaluator)
    assert np.isclose(
        values_too_large[problem.phase_i_epigraph_rows[-1].row_index],
        -1.0e-6,
        atol=1.0e-15,
        rtol=0.0,
    )


def test_existing_physical_rows_equations_and_sparse_prefix_are_unchanged(g5_problems):
    problem, _ = g5_problems[FULL_LAYOUT]
    physical = _physical_rows(problem)
    rows, cols = problem.jacobian_structure()
    prefix_rows = rows[:1461282]
    prefix_cols = cols[:1461282]
    assert sha256(prefix_rows.tobytes() + prefix_cols.tobytes()).hexdigest() == PRE_G5_STRUCTURE_HASH[FULL_LAYOUT]

    metadata = tuple(
        (
            row.row_index,
            row.group,
            row.constraint_id,
            row.component_index,
            row.interval_index,
            row.knot_index,
            row.support,
            row.sense,
            row.differentiability,
            row.slack_decision_index,
        )
        for row in physical
    )
    assert sha256(repr(metadata).encode()).hexdigest() == PRE_G5_ROW_METADATA_DIGEST[FULL_LAYOUT]

    slacks = np.zeros(problem.elastic_slack_count)
    slacks[0] = 0.2
    slacks[-1] = 0.3
    z = np.zeros(problem.variable_count)
    z[[slack.decision_index for slack in problem.layout.slack_specs]] = slacks
    values = problem.constraint_values(z, evaluator=lambda row, _z, _p: 0.5)
    catalog_rows = tuple(
        row
        for row in physical[problem.dynamics_row_count :]
        if row.group != "phase_i_epigraph"
    )
    for row in catalog_rows:
        expected = 0.5 if row.slack_decision_index is None else 0.5 + z[row.slack_decision_index]
        assert values[row.row_index] == expected


@pytest.mark.parametrize("layout", (FULL_LAYOUT, WITNESS_LAYOUT))
def test_structural_delta_is_one_column_42_rows_and_84_entries(g5_problems, layout):
    problem, _ = g5_problems[layout]
    rows, cols = problem.jacobian_structure()
    old_nnz = 1461282 if layout == FULL_LAYOUT else 1419042
    old_variables = 5922 if layout == FULL_LAYOUT else 5602

    assert problem.variable_count - old_variables == 1
    assert problem.constraint_count - 5323 == 42
    assert rows.size - old_nnz == 84
    assert np.all(rows[old_nnz:] >= 5323)
    assert np.all(rows[old_nnz:] < 5365)
    assert set(cols[old_nnz:]) == {
        problem.rho_decision_index,
        *(slack.decision_index for slack in problem.layout.slack_specs),
    }
    for offset, row in enumerate(problem.phase_i_epigraph_rows):
        row_positions = slice(old_nnz + offset * 2, old_nnz + (offset + 1) * 2)
        assert np.array_equal(rows[row_positions], np.array([row.row_index, row.row_index]))
        assert np.array_equal(
            cols[row_positions],
            np.array([row.slack_decision_index, row.rho_decision_index]),
        )


def test_reversal_value_is_independent_of_all_slacks_and_rho(g5_problems):
    problem, actions = g5_problems[FULL_LAYOUT]
    slacks = np.linspace(0.0, 0.5, problem.elastic_slack_count)
    z = problem.pack_decision(
        [np.zeros(STATE_DIMENSION) for _ in range(problem.horizon)],
        actions,
        slacks,
        rho=0.5,
    )
    values = problem.constraint_values(z, evaluator=lambda row, _z, _p: 0.125)
    reversal = next(row for row in problem.constraint_rows if row.constraint_id == "E3_E4_REVERSAL")
    assert values[reversal.row_index] == 0.125
    assert not any(row.constraint_id == "E3_E4_REVERSAL" for row in problem.phase_i_epigraph_rows)


def test_witness_action_sectors_and_fixed_channels_are_unchanged(g5_problems):
    problem, _ = g5_problems[WITNESS_LAYOUT]
    assert problem.layout.active_action_indices == (0, 3, 6, 9, 10, 11, 12)
    assert problem.layout.fixed_action_values == tuple(
        (index, 0.0) for index in (1, 2, 4, 5, 7, 8, 13, 14)
    )
    for interval in range(problem.horizon):
        bounds = problem.layout.action_slice(interval)
        lower = problem.variable_lower_bounds()[bounds]
        upper = problem.variable_upper_bounds()[bounds]
        assert tuple(lower[:5]) == (0.0,) * 5
        assert tuple(upper[:5]) == (1.0,) * 5
        assert tuple(lower[5:]) == (-1.0, -1.0)
        assert tuple(upper[5:]) == (0.0, 0.0)


def test_g5_evaluation_is_solver_free_and_composition_boundary_is_not_involved(g5_problems):
    problem, actions = g5_problems[FULL_LAYOUT]
    z = _zero_decision(problem, actions)
    assert problem.objective(z) == 0.0
    assert problem.objective_gradient(z)[problem.rho_decision_index] == 1.0
    values = problem.constraint_values(z, evaluator=_catalog_evaluator)
    assert np.isfinite(values).all()
    assert problem.phase_i_epigraph_jacobian_values().shape == (84,)
    assert np.array_equal(
        problem.phase_i_epigraph_jacobian_values().reshape(-1, 2),
        np.tile(np.array([-1.0, 1.0]), (42, 1)),
    )
