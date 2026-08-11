"""ML-238 exact raw macro-state boundary qualification."""

from __future__ import annotations

from dataclasses import dataclass, replace
import inspect

import mujoco
import numpy as np
import pytest

from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import ACTION_DIM
from loaded_cmj.simulation.plant import Plant, build_model
from loaded_cmj.simulation.snapshot import MacroSnapshot, MacroSnapshotError
from loaded_cmj.simulation.transition import project_accepted_action, step_5ms


@dataclass(frozen=True)
class _Fixture:
    name: str
    plant: Plant
    snapshot: MacroSnapshot
    source_data: mujoco.MjData
    source_drive: drive.DriveState
    previous_action: np.ndarray
    raw_action: np.ndarray


def _copy_data(model: mujoco.MjModel, source: mujoco.MjData) -> mujoco.MjData:
    result = mujoco.MjData(model)
    mujoco.mj_copyData(result, model, source)
    return result


def _drive_from_snapshot(snapshot: MacroSnapshot) -> drive.DriveState:
    return drive.DriveState(
        a_plus=snapshot.a_plus,
        a_minus=snapshot.a_minus,
        tau_prev=snapshot.tau_prev,
        previous_command=snapshot.previous_command,
        override_flags={key: value.copy() for key, value in snapshot.override_flags},
        reversal_phase=snapshot.reversal_phase,
    )


def _contact_identity(data: mujoco.MjData) -> tuple[tuple[int, int], ...]:
    return tuple((int(data.contact[i].geom1), int(data.contact[i].geom2)) for i in range(data.ncon))


def _state_and_outputs(
    fixture: _Fixture,
    data: mujoco.MjData,
    state: drive.DriveState,
    result,
) -> dict[str, object]:
    force, cop, valid = fixture.plant.foot_contact_summary(data)
    wrench = fixture.plant.contact_wrench_summary(data)
    power = fixture.plant.realized_power_components(data, result.realized_anatomical_torque)
    return {
        "accepted_action": result.accepted_action.copy(),
        "realized_torque": result.realized_anatomical_torque.copy(),
        "capacity_lower": result.capacity_lower.copy(),
        "capacity_upper": result.capacity_upper.copy(),
        "qpos": data.qpos.copy(),
        "qvel": data.qvel.copy(),
        "time": float(data.time),
        "qacc_warmstart": data.qacc_warmstart.copy(),
        "ctrl": data.ctrl.copy(),
        "xmat": data.xmat[list(fixture.snapshot.kinematic_body_ids)].copy(),
        "a_plus": state.a_plus.copy(),
        "a_minus": state.a_minus.copy(),
        "tau_prev": state.tau_prev.copy(),
        "previous_command": state.previous_command.copy(),
        "reversal_phase": state.reversal_phase.copy(),
        "override_flags": {key: value.copy() for key, value in state.override_flags.items()},
        "contacts": _contact_identity(data),
        "force": force.copy(),
        "cop": cop.copy(),
        "cop_valid": valid.copy(),
        "plate_wrench": np.asarray(wrench["plate_wrench"], dtype=np.float64).copy(),
        "com": fixture.plant.center_of_mass(data).copy(),
        "power": power,
    }


def _assert_equal_outputs(left: dict[str, object], right: dict[str, object]) -> None:
    for key, value in left.items():
        other = right[key]
        if isinstance(value, dict):
            assert value.keys() == other.keys()
            for child, child_value in value.items():
                assert np.array_equal(child_value, other[child])
        elif isinstance(value, tuple):
            assert value == other
        elif isinstance(value, np.ndarray):
            assert np.array_equal(value, other), key
        else:
            assert value == other, key


def _capture_fixture(
    name: str,
    plant: Plant,
    data: mujoco.MjData,
    state: drive.DriveState,
    previous_action: np.ndarray,
    raw_action: np.ndarray,
) -> _Fixture:
    source_data = _copy_data(plant.model, data)
    source_drive = state.copy()
    previous = previous_action.copy()
    snapshot = MacroSnapshot.capture(
        plant=plant,
        data=source_data,
        drive_state=source_drive,
        previous_accepted_action=previous,
    )
    return _Fixture(name, plant, snapshot, source_data, source_drive, previous, raw_action.copy())


@pytest.fixture(scope="module")
def fixtures() -> dict[str, _Fixture]:
    model = build_model()
    plant = Plant(model)

    stable = plant.make_data()
    plant.reset_supported(stable)
    stable_state = drive.DriveState()
    zero = np.zeros(ACTION_DIM, dtype=np.float64)
    s0 = _capture_fixture("S0", plant, stable, stable_state, zero, zero)

    nonzero = _copy_data(model, stable)
    nonzero_state = stable_state.copy()
    nonzero_raw = np.full(ACTION_DIM, 0.18, dtype=np.float64)
    nonzero_result = step_5ms(
        plant=plant,
        data=nonzero,
        drive_state=nonzero_state,
        previous_accepted_action=zero,
        raw_action=nonzero_raw,
    )
    s1 = _capture_fixture(
        "S1", plant, nonzero, nonzero_state, nonzero_result.accepted_action, nonzero_raw
    )

    history = _copy_data(model, nonzero)
    history_state = nonzero_state.copy()
    history_previous = nonzero_result.accepted_action.copy()
    history_actions = (
        np.asarray((0.40, -0.35, 0.10) * 5, dtype=np.float64),
        np.asarray((-0.30, 0.25, -0.08) * 5, dtype=np.float64),
        np.asarray((0.18, -0.15, 0.04) * 5, dtype=np.float64),
    )
    for raw in history_actions:
        history_result = step_5ms(
            plant=plant,
            data=history,
            drive_state=history_state,
            previous_accepted_action=history_previous,
            raw_action=raw,
        )
        history_previous = history_result.accepted_action.copy()
    s2 = _capture_fixture("S2", plant, history, history_state, history_previous, history_actions[-1])

    moving = _copy_data(model, history)
    moving_state = history_state.copy()
    moving_previous = history_previous.copy()
    moving_raw = np.asarray((-0.55, 0.45, -0.20) * 5, dtype=np.float64)
    moving_result = step_5ms(
        plant=plant,
        data=moving,
        drive_state=moving_state,
        previous_accepted_action=moving_previous,
        raw_action=moving_raw,
    )
    s3 = _capture_fixture(
        "S3", plant, moving, moving_state, moving_result.accepted_action, moving_raw
    )

    contact = plant.make_data()
    contact_reset = plant.reset_fixed_hold(contact)
    contact_state = drive.DriveState(
        a_plus=contact_reset.a_plus,
        a_minus=contact_reset.a_minus,
        tau_prev=contact_reset.previous_torque,
        previous_command=contact_reset.previous_command,
        override_flags=contact_reset.override_flags,
        reversal_phase=contact_reset.reversal_phase,
    )
    s4 = _capture_fixture(
        "S4", plant, contact, contact_state, contact_reset.previous_action, zero
    )

    contact_alt = _copy_data(model, contact)
    contact_alt_state = contact_state.copy()
    contact_alt_previous = contact_reset.previous_action.copy()
    contact_alt_raw = np.zeros(ACTION_DIM, dtype=np.float64)
    contact_alt_raw[0] = 0.25
    contact_alt_raw[3] = -0.20
    contact_alt_result = step_5ms(
        plant=plant,
        data=contact_alt,
        drive_state=contact_alt_state,
        previous_accepted_action=contact_alt_previous,
        raw_action=contact_alt_raw,
    )
    s5 = _capture_fixture(
        "S5", plant, contact_alt, contact_alt_state, contact_alt_result.accepted_action, contact_alt_raw
    )
    assert _contact_identity(s4.source_data) == _contact_identity(s5.source_data)
    assert not np.array_equal(
        plant.contact_wrench_summary(s4.source_data)["plate_wrench"],
        plant.contact_wrench_summary(s5.source_data)["plate_wrench"],
    )
    return {fixture.name: fixture for fixture in (s0, s1, s2, s3, s4, s5)}


def _run_once(fixture: _Fixture) -> tuple[dict[str, object], dict[str, object]]:
    left_data = _copy_data(fixture.plant.model, fixture.source_data)
    left_state = fixture.source_drive.copy()
    left_result = step_5ms(
        plant=fixture.plant,
        data=left_data,
        drive_state=left_state,
        previous_accepted_action=fixture.previous_action.copy(),
        raw_action=fixture.raw_action,
    )

    right_data = mujoco.MjData(fixture.plant.model)
    right_state = _drive_from_snapshot(fixture.snapshot)
    right_previous = fixture.snapshot.restore(
        plant=fixture.plant, data=right_data, drive_state=right_state
    )
    assert np.array_equal(right_previous, fixture.previous_action)
    right_result = step_5ms(
        plant=fixture.plant,
        data=right_data,
        drive_state=right_state,
        previous_accepted_action=right_previous,
        raw_action=fixture.raw_action,
    )
    return (
        _state_and_outputs(fixture, left_data, left_state, left_result),
        _state_and_outputs(fixture, right_data, right_state, right_result),
    )


@pytest.mark.parametrize("fixture_name", ("S0", "S1", "S2", "S3", "S4", "S5"))
def test_one_step_restart_equivalence(fixtures, fixture_name: str) -> None:
    left, right = _run_once(fixtures[fixture_name])
    _assert_equal_outputs(left, right)


def test_snapshot_copy_alias_identity_and_reuse(fixtures) -> None:
    fixture = fixtures["S4"]
    snapshot = fixture.snapshot
    saved = snapshot.qpos.copy(), snapshot.kinematic_xmat.copy(), snapshot.previous_accepted_action.copy()
    data = _copy_data(fixture.plant.model, fixture.source_data)
    state = fixture.source_drive.copy()
    previous = fixture.previous_action.copy()
    data.qpos[0] += 0.01
    data.xmat[list(snapshot.kinematic_body_ids), 0] += 0.01
    state.a_plus[0] = 0.0
    previous[0] = 0.0
    assert np.array_equal(snapshot.qpos, saved[0])
    assert np.array_equal(snapshot.kinematic_xmat, saved[1])
    assert np.array_equal(snapshot.previous_accepted_action, saved[2])
    assert not snapshot.qpos.flags.writeable
    assert not snapshot.kinematic_xmat.flags.writeable

    first_data = mujoco.MjData(fixture.plant.model)
    first_state = _drive_from_snapshot(snapshot)
    first_previous = snapshot.restore(plant=fixture.plant, data=first_data, drive_state=first_state)
    second_data = mujoco.MjData(fixture.plant.model)
    second_state = _drive_from_snapshot(snapshot)
    second_previous = snapshot.restore(
        plant=fixture.plant, data=second_data, drive_state=second_state
    )
    assert np.array_equal(first_previous, second_previous)
    assert np.array_equal(first_data.qpos, second_data.qpos)
    assert np.array_equal(first_data.xmat, second_data.xmat)
    assert np.array_equal(first_state.a_plus, second_state.a_plus)


def test_snapshot_model_dimension_guard(fixtures) -> None:
    snapshot = fixtures["S0"].snapshot
    with pytest.raises(MacroSnapshotError):
        replace(snapshot, dimensions=(24, 21, 15, 0, 0, 0, 0, 0))
    bad_identity = replace(snapshot, model_id="wrong-model")
    with pytest.raises(MacroSnapshotError):
        bad_identity.restore(
            plant=fixtures["S0"].plant,
            data=mujoco.MjData(fixtures["S0"].plant.model),
            drive_state=_drive_from_snapshot(snapshot),
        )


def test_snapshot_rejects_nonfinite_and_hidden_forces(fixtures) -> None:
    fixture = fixtures["S0"]
    data = _copy_data(fixture.plant.model, fixture.source_data)
    data.qpos[0] = np.nan
    with pytest.raises(MacroSnapshotError):
        MacroSnapshot.capture(
            plant=fixture.plant,
            data=data,
            drive_state=fixture.source_drive.copy(),
            previous_accepted_action=fixture.previous_action,
        )
    data = _copy_data(fixture.plant.model, fixture.source_data)
    data.qfrc_applied[0] = 1.0
    with pytest.raises(MacroSnapshotError):
        MacroSnapshot.capture(
            plant=fixture.plant,
            data=data,
            drive_state=fixture.source_drive.copy(),
            previous_accepted_action=fixture.previous_action,
        )
    restored = mujoco.MjData(fixture.plant.model)
    restored.qfrc_applied[0] = 1.0
    restored.xfrc_applied[0, 0] = 1.0
    fixture.snapshot.restore(
        plant=fixture.plant,
        data=restored,
        drive_state=_drive_from_snapshot(fixture.snapshot),
    )
    assert np.array_equal(restored.qfrc_applied, np.zeros_like(restored.qfrc_applied))
    assert np.array_equal(restored.xfrc_applied, np.zeros_like(restored.xfrc_applied))


def test_multistep_restart_equivalence(fixtures) -> None:
    horizons = {1, 2, 5, 10, 20}
    for fixture in fixtures.values():
        left_data = _copy_data(fixture.plant.model, fixture.source_data)
        left_state = fixture.source_drive.copy()
        left_previous = fixture.previous_action.copy()
        right_data = mujoco.MjData(fixture.plant.model)
        right_state = _drive_from_snapshot(fixture.snapshot)
        right_previous = fixture.snapshot.restore(
            plant=fixture.plant, data=right_data, drive_state=right_state
        )
        for step_index in range(1, 21):
            raw = np.asarray(
                (0.06, -0.05, 0.04) * 5,
                dtype=np.float64,
            ) * (1.0 if step_index % 2 else -1.0)
            left_result = step_5ms(
                plant=fixture.plant,
                data=left_data,
                drive_state=left_state,
                previous_accepted_action=left_previous,
                raw_action=raw,
            )
            right_result = step_5ms(
                plant=fixture.plant,
                data=right_data,
                drive_state=right_state,
                previous_accepted_action=right_previous,
                raw_action=raw,
            )
            left_previous = left_result.accepted_action.copy()
            right_previous = right_result.accepted_action.copy()
            if step_index in horizons:
                _assert_equal_outputs(
                    _state_and_outputs(fixture, left_data, left_state, left_result),
                    _state_and_outputs(fixture, right_data, right_state, right_result),
                )


@pytest.mark.parametrize("fixture_name", ("S4", "S5"))
def test_qacc_warmstart_is_causal(fixtures, fixture_name: str) -> None:
    fixture = fixtures[fixture_name]
    reference_data = mujoco.MjData(fixture.plant.model)
    reference_state = _drive_from_snapshot(fixture.snapshot)
    reference_previous = fixture.snapshot.restore(
        plant=fixture.plant, data=reference_data, drive_state=reference_state
    )
    ablated_data = mujoco.MjData(fixture.plant.model)
    ablated_state = _drive_from_snapshot(fixture.snapshot)
    ablated_previous = fixture.snapshot.restore(
        plant=fixture.plant, data=ablated_data, drive_state=ablated_state
    )
    ablated_data.qacc_warmstart[:] = 0.0
    reference_outputs = None
    ablated_outputs = None
    for step_index in range(20):
        raw = fixture.raw_action if step_index == 0 else np.zeros(ACTION_DIM, dtype=np.float64)
        reference_result = step_5ms(
            plant=fixture.plant,
            data=reference_data,
            drive_state=reference_state,
            previous_accepted_action=reference_previous,
            raw_action=raw,
        )
        ablated_result = step_5ms(
            plant=fixture.plant,
            data=ablated_data,
            drive_state=ablated_state,
            previous_accepted_action=ablated_previous,
            raw_action=raw,
        )
        reference_previous = reference_result.accepted_action.copy()
        ablated_previous = ablated_result.accepted_action.copy()
        if step_index == 19:
            reference_outputs = _state_and_outputs(
                fixture, reference_data, reference_state, reference_result
            )
            ablated_outputs = _state_and_outputs(
                fixture, ablated_data, ablated_state, ablated_result
            )
    assert reference_outputs is not None and ablated_outputs is not None
    assert not np.array_equal(reference_outputs["qpos"], ablated_outputs["qpos"])
    assert not np.array_equal(reference_outputs["qvel"], ablated_outputs["qvel"])
    assert not np.array_equal(reference_outputs["plate_wrench"], ablated_outputs["plate_wrench"])


def test_kinematic_cache_is_causal_and_restored(fixtures) -> None:
    fixture = fixtures["S4"]
    alternate_cache = fixtures["S5"].snapshot.kinematic_xmat

    def run(cache: np.ndarray):
        data = mujoco.MjData(fixture.plant.model)
        state = _drive_from_snapshot(fixture.snapshot)
        previous = fixture.snapshot.restore(plant=fixture.plant, data=data, drive_state=state)
        data.xmat[list(fixture.snapshot.kinematic_body_ids), :] = cache
        result = step_5ms(
            plant=fixture.plant,
            data=data,
            drive_state=state,
            previous_accepted_action=previous,
            raw_action=fixture.raw_action,
        )
        return data, result

    reference_data, reference_result = run(fixture.snapshot.kinematic_xmat)
    altered_data, altered_result = run(alternate_cache)
    assert not np.array_equal(
        reference_result.realized_anatomical_torque,
        altered_result.realized_anatomical_torque,
    )
    assert not np.array_equal(reference_data.qpos, altered_data.qpos)


def test_previous_command_is_redundant_and_ctrl_is_overwritten(fixtures) -> None:
    fixture = fixtures["S2"]
    reference_data = mujoco.MjData(fixture.plant.model)
    reference_state = _drive_from_snapshot(fixture.snapshot)
    previous = fixture.snapshot.restore(
        plant=fixture.plant, data=reference_data, drive_state=reference_state
    )
    altered_data = mujoco.MjData(fixture.plant.model)
    altered_state = _drive_from_snapshot(fixture.snapshot)
    altered_previous = fixture.snapshot.restore(
        plant=fixture.plant, data=altered_data, drive_state=altered_state
    )
    altered_state.previous_command[:] = 0.0
    altered_data.ctrl[:] = 0.0
    reference_result = step_5ms(
        plant=fixture.plant,
        data=reference_data,
        drive_state=reference_state,
        previous_accepted_action=previous,
        raw_action=fixture.raw_action,
    )
    altered_result = step_5ms(
        plant=fixture.plant,
        data=altered_data,
        drive_state=altered_state,
        previous_accepted_action=altered_previous,
        raw_action=fixture.raw_action,
    )
    _assert_equal_outputs(
        _state_and_outputs(fixture, reference_data, reference_state, reference_result),
        _state_and_outputs(fixture, altered_data, altered_state, altered_result),
    )


def test_absolute_time_is_raw_identity_but_not_physical_state(fixtures) -> None:
    fixture = fixtures["S4"]
    left_data = mujoco.MjData(fixture.plant.model)
    left_state = _drive_from_snapshot(fixture.snapshot)
    left_previous = fixture.snapshot.restore(plant=fixture.plant, data=left_data, drive_state=left_state)
    right_data = mujoco.MjData(fixture.plant.model)
    right_state = _drive_from_snapshot(fixture.snapshot)
    right_previous = fixture.snapshot.restore(
        plant=fixture.plant, data=right_data, drive_state=right_state
    )
    right_data.time += 1.0
    left_result = step_5ms(
        plant=fixture.plant,
        data=left_data,
        drive_state=left_state,
        previous_accepted_action=left_previous,
        raw_action=fixture.raw_action,
    )
    right_result = step_5ms(
        plant=fixture.plant,
        data=right_data,
        drive_state=right_state,
        previous_accepted_action=right_previous,
        raw_action=fixture.raw_action,
    )
    assert right_data.time != left_data.time
    assert np.array_equal(left_data.qpos, right_data.qpos)
    assert np.array_equal(left_data.qvel, right_data.qvel)
    assert np.array_equal(left_result.realized_anatomical_torque, right_result.realized_anatomical_torque)


def test_snapshot_and_transition_firewalls() -> None:
    import loaded_cmj.simulation.snapshot as snapshot_module
    import loaded_cmj.simulation.transition as transition_module

    capture_parameters = tuple(inspect.signature(MacroSnapshot.capture).parameters)
    assert capture_parameters == ("plant", "data", "drive_state", "previous_accepted_action")
    snapshot_source = inspect.getsource(snapshot_module)
    assert "PolicyWorker" not in snapshot_source
    assert "CMJEventDetector" not in snapshot_source
    assert "RolloutEngine" not in snapshot_source
    assert "scorer" not in snapshot_source.lower()
    assert "mujoco.mj_step" not in snapshot_source
    transition_source = inspect.getsource(transition_module)
    assert transition_source.count("mujoco.mj_step") == 1


def test_ml237_action_slew_semantics_remain_unchanged() -> None:
    previous = np.zeros(ACTION_DIM, dtype=np.float64)
    raw = np.ones(ACTION_DIM, dtype=np.float64)
    assert np.array_equal(project_accepted_action(previous, raw), np.full(ACTION_DIM, 0.20))
