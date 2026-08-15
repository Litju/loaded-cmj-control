"""TDD coverage for Candidate-C explicit Drive mode schedules."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

from loaded_cmj.oracle.derivatives import (
    evaluate_wrapped_step_5ms,
    transition_mode_schedule,
)
from loaded_cmj.simulation.tangent import boxminus

sys.path.insert(0, str(Path(__file__).parent))
from test_ml238_macro_state import fixtures as macro_fixtures


@pytest.fixture(scope="module")
def supported_fixture(macro_fixtures):
    return macro_fixtures["S1"]


def test_candidate_c_schedule_has_one_entry_per_frozen_substep(supported_fixture) -> None:
    evaluation = evaluate_wrapped_step_5ms(
        plant=supported_fixture.plant,
        snapshot=supported_fixture.snapshot,
        raw_action=supported_fixture.raw_action,
    )

    schedule = transition_mode_schedule(evaluation.active_set)

    assert len(schedule) == 40
    assert all(len(entry) == 2 for entry in schedule)
    assert all(len(entry[0]) == 15 and len(entry[1]) == 15 for entry in schedule)


def test_candidate_c_scheduled_base_value_matches_exact_transition(supported_fixture) -> None:
    exact = evaluate_wrapped_step_5ms(
        plant=supported_fixture.plant,
        snapshot=supported_fixture.snapshot,
        raw_action=supported_fixture.raw_action,
    )
    schedule = transition_mode_schedule(exact.active_set)
    scheduled = evaluate_wrapped_step_5ms(
        plant=supported_fixture.plant,
        snapshot=supported_fixture.snapshot,
        raw_action=supported_fixture.raw_action,
        torque_velocity_schedule=schedule,
    )

    delta = boxminus(
        scheduled.next_snapshot,
        exact.next_snapshot,
        model=supported_fixture.plant.model,
    )
    assert np.max(np.abs(delta)) <= 1.0e-10
    assert np.array_equal(scheduled.transition.accepted_action, exact.transition.accepted_action)


def test_candidate_c_schedule_validation_is_fail_closed(supported_fixture) -> None:
    evaluation = evaluate_wrapped_step_5ms(
        plant=supported_fixture.plant,
        snapshot=supported_fixture.snapshot,
        raw_action=supported_fixture.raw_action,
    )
    schedule = transition_mode_schedule(evaluation.active_set)

    with pytest.raises(ValueError, match="40"):
        evaluate_wrapped_step_5ms(
            plant=supported_fixture.plant,
            snapshot=supported_fixture.snapshot,
            raw_action=supported_fixture.raw_action,
            torque_velocity_schedule=schedule[:-1],
        )
