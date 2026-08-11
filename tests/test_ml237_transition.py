"""ML-237 contract tests for the shared exact 5 ms transition."""

from __future__ import annotations

import inspect

import mujoco
import numpy as np

from loaded_cmj.biomechanics import effectiveness
from loaded_cmj.runtime import engine
from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import ACTION_DIM, PHYSICS_TIMESTEP_S, SUBSTEPS_PER_CONTROL
from loaded_cmj.simulation.plant import Plant, build_model
from loaded_cmj.simulation.transition import project_accepted_action, step_5ms


def _fixture() -> tuple[mujoco.MjModel, Plant, mujoco.MjData, drive.DriveState]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    return model, plant, data, drive.DriveState()


def _old_reference_step(
    model: mujoco.MjModel,
    plant: Plant,
    data: mujoco.MjData,
    state: drive.DriveState,
    previous: np.ndarray,
    raw: np.ndarray,
) -> np.ndarray:
    """Independent pre-ML237 reference loop; retained as a test negative control."""

    accepted = previous + np.clip(raw - previous, -0.20, 0.20)
    for _ in range(SUBSTEPS_PER_CONTROL):
        result = drive.drive_state_step(
            accepted,
            state,
            plant.anatomical_coordinates(data),
            plant.anatomical_rates(data),
            PHYSICS_TIMESTEP_S,
        )
        realized = np.asarray(result["tau"], dtype=np.float64)
        plant.realized_power_components(data, realized)
        plant.apply_anatomical_torque(data, realized)
        mujoco.mj_step(model, data)
    return accepted


def test_slew_projection_identity_negative_and_mixed_channels() -> None:
    previous = np.zeros(ACTION_DIM, dtype=np.float64)
    raw = np.ones(ACTION_DIM, dtype=np.float64)
    accepted = project_accepted_action(previous, raw)
    assert np.array_equal(accepted, np.full(ACTION_DIM, 0.20))

    for expected, value in zip((0.40, 0.60, 0.80, 1.00), (0.40, 0.60, 0.80, 1.00)):
        accepted = project_accepted_action(accepted, np.full(ACTION_DIM, value))
        assert np.array_equal(accepted, np.full(ACTION_DIM, expected))

    mixed_previous = np.zeros(ACTION_DIM, dtype=np.float64)
    mixed_raw = np.asarray((1.0, -1.0, 0.1) * 5, dtype=np.float64)
    mixed = project_accepted_action(mixed_previous, mixed_raw)
    assert np.array_equal(mixed, np.asarray((0.20, -0.20, 0.10) * 5))


def test_shared_transition_has_one_slew_and_exactly_40_physics_steps() -> None:
    _, plant, data, state = _fixture()
    accepted_seen: list[np.ndarray] = []
    command_seen: list[np.ndarray] = []

    def after_substep(substep, accepted, drive_result, realized, power):
        del substep, drive_result, realized, power
        accepted_seen.append(accepted.copy())
        command_seen.append(state.previous_command.copy())
        return True

    result = step_5ms(
        plant=plant,
        data=data,
        drive_state=state,
        previous_accepted_action=np.zeros(ACTION_DIM),
        raw_action=np.ones(ACTION_DIM),
        on_substep=after_substep,
    )
    assert result.substeps_executed == SUBSTEPS_PER_CONTROL
    assert len(accepted_seen) == SUBSTEPS_PER_CONTROL
    assert np.isclose(
        data.time,
        SUBSTEPS_PER_CONTROL * PHYSICS_TIMESTEP_S,
        atol=1.0e-15,
        rtol=0.0,
    )
    assert all(np.array_equal(value, np.full(ACTION_DIM, 0.20)) for value in accepted_seen)
    assert all(np.array_equal(value, np.full(ACTION_DIM, 0.20)) for value in command_seen)
    assert np.array_equal(result.accepted_action, accepted_seen[-1])


def test_live_transition_is_deterministic_and_matches_independent_old_loop() -> None:
    old_model, old_plant, old_data, old_state = _fixture()
    new_model, new_plant, new_data, new_state = _fixture()
    previous_old = np.zeros(ACTION_DIM, dtype=np.float64)
    previous_new = previous_old.copy()
    actions = (
        np.ones(ACTION_DIM, dtype=np.float64),
        np.asarray((0.4, -0.4, 0.2) * 5, dtype=np.float64),
        np.asarray((-0.6, 0.6, -0.3) * 5, dtype=np.float64),
    )

    for raw in actions:
        old_accepted = _old_reference_step(
            old_model, old_plant, old_data, old_state, previous_old, raw
        )
        new_result = step_5ms(
            plant=new_plant,
            data=new_data,
            drive_state=new_state,
            previous_accepted_action=previous_new,
            raw_action=raw,
        )
        assert np.array_equal(old_accepted, new_result.accepted_action)
        assert np.array_equal(old_data.qpos, new_data.qpos)
        assert np.array_equal(old_data.qvel, new_data.qvel)
        assert np.array_equal(old_state.a_plus, new_state.a_plus)
        assert np.array_equal(old_state.a_minus, new_state.a_minus)
        assert np.array_equal(old_state.tau_prev, new_state.tau_prev)
        assert np.array_equal(old_state.previous_command, new_state.previous_command)
        assert np.array_equal(
            old_plant.contact_wrench_summary(old_data)["plate_wrench"],
            new_plant.contact_wrench_summary(new_data)["plate_wrench"],
        )
        previous_old = old_accepted
        previous_new = new_result.accepted_action


def test_effectiveness_calls_shared_transition(monkeypatch) -> None:
    _, plant, data, state = _fixture()
    action = np.zeros(ACTION_DIM, dtype=np.float64)
    baseline_active = np.asarray(plant.contact_wrench_summary(data)["active_by_foot"], dtype=bool)
    original = effectiveness.step_5ms
    calls = 0

    def wrapped(*args, **kwargs):
        nonlocal calls
        calls += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(effectiveness, "step_5ms", wrapped)
    effectiveness._transition(plant, data, state, action, baseline_active)
    assert calls == 1


def test_production_owners_have_no_second_substep_loop() -> None:
    engine_source = inspect.getsource(engine.RolloutEngine.run)
    effectiveness_source = inspect.getsource(effectiveness._transition)
    assert "step_5ms(" in engine_source
    assert "drive_state_step(" not in engine_source
    assert "range(SUBSTEPS_PER_CONTROL)" not in engine_source
    assert "step_5ms(" in effectiveness_source
    assert "drive_state_step(" not in effectiveness_source
    assert "range(SUBSTEPS_PER_CONTROL)" not in effectiveness_source


def test_transition_owner_does_not_own_policy_events_or_public_observations() -> None:
    import loaded_cmj.simulation.transition as transition

    source = inspect.getsource(transition)
    assert "biomechanics.events" not in source
    assert "CMJEventDetector" not in source
    assert "public_observation" not in source
    assert "PolicyWorker" not in source
