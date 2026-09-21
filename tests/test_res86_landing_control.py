"""RES-86B active-set-safe landing branch/control tests.

MISSION: RES86_LANDING_CAPTURE_IMPLEMENTATION_001
LINEAR ISSUE: RES-86

Narrow, deterministic tests for the landing-control infrastructure:

* native control-cell geometry (0.010 s = 5 native steps);
* exact actuation history snapshot/restore (no reset at handoff);
* exact branch replay identity and truncated-interval identity;
* real-MuJoCo finite-difference epsilon schedule, plus/minus/central mode
  selection and derivative availability;
* exact branch validation, live/validated branch identity and the declared
  fallback telemetry discipline (proposed / validated / applied);
* the fixed constraint-row catalog (per-foot Fz rows never disappear);
* corrected D_EST latency and D_BL sustain event semantics, including negative
  controls and material-reflight detection.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
SRC = TASK_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.actuation import (  # noqa: E402
    N_CHANNELS,
    MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
    MTP_LATE_ACTIVE_FRACTION,
    MTP_TOTAL_POSITIVE_WORK_BUDGET_J,
    V3ActuationAuthority,
    V3ActuationState,
    V3MtpLedgerEntry,
)
from loaded_cmj.v3.active_set_capture import (  # noqa: E402
    DERIVATIVE_CENTRAL_ALLOWED,
    DERIVATIVE_ONE_SIDED_SAME_MODE,
    DERIVATIVE_UNAVAILABLE,
    derivative_eligibility,
)
from loaded_cmj.v3.constants import V3_SYSTEM_WEIGHT_N  # noqa: E402
from loaded_cmj.v3.landing_control import (  # noqa: E402
    CONTROL_COORDINATES,
    CONTROL_INTERVAL_NATIVE_STEPS,
    CONTROL_INTERVAL_S,
    CONSTRAINT_ROW_NAMES,
    FD_EPS_FRACTIONS,
    OBJECTIVE_WEIGHTS,
    OUTPUT_INDEX,
    V3BranchEngine,
    V3LandingConfig,
    V3LandingController,
    V3LandingFault,
    V3LandingPhase,
)
from loaded_cmj.v3.landing_events import (  # noqa: E402
    V3LandingTrace,
    bilateral_establishment_position,
    e10_confirmation_position,
    evaluate_landing_trace,
)
from loaded_cmj.v3.plant import V3Plant  # noqa: E402
from loaded_cmj.v3.launch_runtime import settle_standing_stance  # noqa: E402


def _flat_stance() -> tuple[V3Plant, object, V3ActuationAuthority]:
    plant = V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    authority = V3ActuationAuthority(float(plant.model.opt.timestep))
    return plant, data, authority


def _touchdown_like_state(plant: V3Plant, data) -> None:
    """Config with both feet loaded (flat stance is already bilateral)."""
    settle_standing_stance(plant, data)


# ---------------------------------------------------------------------------
# control-cell geometry
# ---------------------------------------------------------------------------
def test_control_interval_is_five_native_steps():
    assert CONTROL_INTERVAL_S == 0.010
    assert CONTROL_INTERVAL_NATIVE_STEPS == 5
    assert CONTROL_INTERVAL_NATIVE_STEPS * M.NATIVE_DT_S == CONTROL_INTERVAL_S
    assert len(CONTROL_COORDINATES) == 5
    assert set(OBJECTIVE_WEIGHTS).issubset(set(OUTPUT_INDEX))
    assert "com_vz_terminal" in OBJECTIVE_WEIGHTS


# ---------------------------------------------------------------------------
# actuation history snapshot / restore
# ---------------------------------------------------------------------------
def test_mtp_phase_gate_is_transient_and_ledger_stays_cumulative():
    """A flight phase gate must not permanently disable the landing MTP.

    The flight gate zeroes the applied active MTP moment per sample, but the
    cumulative energy ledger is untouched and the remaining late budget is
    still spendable once legal plantar support is re-established.  Only an
    exhausted energy budget latches the channel (AEI-1d).
    """
    authority = V3ActuationAuthority(0.002)
    command = np.zeros(N_CHANNELS)
    command[7] = command[8] = 20.0
    qdot = np.zeros(N_CHANNELS)
    qdot[7] = qdot[8] = 5.0
    passive = (0.0, 0.0)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    # supported pre-flight work
    for _ in range(10):
        applied, _, ledger = authority.apply(
            command, qdot, phase="BRAKING", mtp_active_allowed=True,
            mtp_passive_moment_nm=passive, mtp_ledger=ledger)
    spent_before_flight = ledger[0].active_positive_work_j
    assert spent_before_flight > 0.0
    previous_applied = authority.previous_applied
    # flight: phase-gated for many samples
    for _ in range(40):
        applied, record, ledger = authority.apply(
            command, qdot, phase="FLIGHT", mtp_active_allowed=(False, False),
            mtp_passive_moment_nm=passive, mtp_ledger=ledger)
        assert applied[7] == 0.0 and applied[8] == 0.0
        assert record.mtp_gated == (True, True)
        assert record.saturation_stage[7] in ("mtp_phase_gate",
                                              "safety_override_mtp_phase_gate")
    assert ledger[0].active_gated is False and ledger[1].active_gated is False
    assert ledger[0].active_positive_work_j == spent_before_flight
    # the applied active MTP moment is exactly zero while the phase gate is on,
    # so only the non-MTP torque-rate history is unchanged
    assert np.array_equal(authority.previous_applied[:7], previous_applied[:7])
    assert authority.previous_applied[7] == 0.0 and authority.previous_applied[8] == 0.0
    # legal landing support: the same cumulative ledger may still spend budget
    applied, record, ledger = authority.apply(
        command, qdot, phase="IMPACT_ABSORPTION", mtp_active_allowed=(True, True),
        mtp_passive_moment_nm=passive, mtp_ledger=ledger)
    assert applied[7] != 0.0 and applied[8] != 0.0
    assert record.mtp_gated == (False, False)
    assert ledger[0].active_positive_work_j > spent_before_flight
    # the late active budget is a fraction of the sealed 25 J budget and the
    # cumulative spend never exceeds it
    late_budget = MTP_ACTIVE_POSITIVE_WORK_BUDGET_J * MTP_LATE_ACTIVE_FRACTION
    assert ledger[0].active_positive_work_j <= late_budget + 1.0e-9

    # energy exhaustion is the only latch: drive a fresh authority past budget
    exhausted = V3ActuationAuthority(0.002)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    big = np.zeros(N_CHANNELS)
    big[7] = big[8] = 45.0
    fast = np.zeros(N_CHANNELS)
    fast[7] = fast[8] = 10.0
    for _ in range(2000):
        _, _, ledger = exhausted.apply(
            big, fast, phase="IMPACT_ABSORPTION", mtp_active_allowed=True,
            mtp_passive_moment_nm=passive, mtp_ledger=ledger)
    assert ledger[0].active_gated is True and ledger[1].active_gated is True
    assert ledger[0].active_positive_work_j <= late_budget + 1.0e-9


def test_actuation_snapshot_restore_preserves_history():
    authority = V3ActuationAuthority(0.002)
    desired = np.array([10.0, -20.0, -20.0, -30.0, -30.0, 5.0, 5.0, 1.0, 1.0])
    qdot = np.full(N_CHANNELS, 0.5)
    passive = (0.0, 0.0)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    for _ in range(3):
        _, _, ledger = authority.apply(desired, qdot, phase="LANDING_CAPTURE",
                                       mtp_active_allowed=(True, True),
                                       mtp_passive_moment_nm=passive, mtp_ledger=ledger)
    snapshot = authority.snapshot_state()
    assert snapshot.validate() == []
    state_before = authority.previous_applied
    _, _, ledger = authority.apply(desired * 2.0, qdot, phase="LANDING_CAPTURE",
                                   mtp_active_allowed=(True, True),
                                   mtp_passive_moment_nm=passive, mtp_ledger=ledger)
    assert not np.array_equal(authority.previous_applied, state_before)
    authority.restore_state(snapshot)
    assert np.array_equal(authority.previous_applied, state_before)
    assert authority.step_index == snapshot.step_index
    assert authority.internal_mtp_ledger[0] == snapshot.mtp_ledger[0]
    # a malformed snapshot is rejected, never silently applied
    with pytest.raises(Exception):
        authority.restore_state(V3ActuationState(
            previous_applied_nm=(1.0,), mtp_ledger=snapshot.mtp_ledger,
            step_index=0))


# ---------------------------------------------------------------------------
# real-MuJoCo branch engine: exact restore and truncated interval identity
# ---------------------------------------------------------------------------
def test_branch_engine_exact_restore_matches_live_replay():
    plant, data, authority = _flat_stance()
    engine = V3BranchEngine(plant)
    base = engine.capture_state(data, sample_index=0, time_s=0.0, authority=authority)
    desired = np.array([0.0, -10.0, -10.0, -20.0, -20.0, 5.0, 5.0, 0.0, 0.0])

    outcome = engine.evaluate(base, desired, native_steps=3, phase="LANDING_CAPTURE")
    assert len(outcome.samples) == 3
    assert outcome.native_steps == 3
    assert outcome.mode_signature

    # live replay of exactly the same three steps
    live = plant.make_data()
    import mujoco
    mujoco.mj_setState(plant.model, live, base.state_vector,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    live.ctrl[:] = base.ctrl_nm
    live.qacc_warmstart[:] = base.qacc_warmstart
    mujoco.mj_forward(plant.model, live)
    replay_authority = V3ActuationAuthority(0.002)
    replay_authority.restore_state(base.actuation)
    ledger = replay_authority.internal_mtp_ledger
    for _ in range(3):
        qdot = np.array([live.qvel[plant.idx.vadr[name]] for name in
                         ("trunk_pelvis", "left_hip", "right_hip", "left_knee", "right_knee",
                          "left_ankle", "right_ankle", "left_mtp", "right_mtp")])
        passive = (float(live.qfrc_passive[plant.idx.vadr["left_mtp"]]),
                   float(live.qfrc_passive[plant.idx.vadr["right_mtp"]]))
        applied, _, ledger = replay_authority.apply(
            desired, qdot, phase="LANDING_CAPTURE", mtp_active_allowed=(True, True),
            mtp_passive_moment_nm=passive, mtp_ledger=ledger)
        live.ctrl[:] = applied
        mujoco.mj_step(plant.model, live)
        mujoco.mj_forward(plant.model, live)
    live_vector = np.zeros_like(outcome.terminal_state_vector)
    mujoco.mj_getState(plant.model, live, live_vector, mujoco.mjtState.mjSTATE_INTEGRATION)
    assert np.array_equal(live_vector, outcome.terminal_state_vector)


def test_branch_engine_does_not_mutate_live_data():
    plant, data, authority = _flat_stance()
    engine = V3BranchEngine(plant)
    import mujoco
    size = int(mujoco.mj_stateSize(plant.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    before = np.zeros(size)
    mujoco.mj_getState(plant.model, data, before, mujoco.mjtState.mjSTATE_INTEGRATION)
    base = engine.capture_state(data, sample_index=0, time_s=0.0, authority=authority)
    engine.evaluate(base, np.zeros(N_CHANNELS), phase="LANDING_CAPTURE")
    after = np.zeros(size)
    mujoco.mj_getState(plant.model, data, after, mujoco.mjtState.mjSTATE_INTEGRATION)
    assert np.array_equal(before, after)


# ---------------------------------------------------------------------------
# derivative schedule and mode-compatible selection
# ---------------------------------------------------------------------------
def test_derivative_epsilon_schedule_and_classification():
    plant, data, authority = _flat_stance()
    engine = V3BranchEngine(plant)
    base = engine.capture_state(data, sample_index=0, time_s=0.0, authority=authority)
    controller = V3LandingController(plant, data, config=V3LandingConfig(), actuation=authority)
    frame = M.native_frame(plant, data, 0, 0.0)
    snapshot = M.measure(plant, data)
    baseline = controller._baseline_desired(data, frame, snapshot)
    nominal, columns = controller._build_response(base, baseline, phase="LANDING_CAPTURE")
    assert len(columns) == len(CONTROL_COORDINATES)
    for column in columns:
        assert column.initial_epsilon_nm == pytest.approx(
            FD_EPS_FRACTIONS[0] * controller.config.coordinate_ceiling(column.coordinate))
        assert column.classification in (
            "CENTRAL", "PLUS_ONLY", "MINUS_ONLY", DERIVATIVE_UNAVAILABLE)
        assert column.nominal_mode_hash
        if column.classification != DERIVATIVE_UNAVAILABLE:
            assert column.accepted_epsilon_nm is not None
            assert column.accepted_epsilon_nm in [
                pytest.approx(value) for value in column.epsilons_tried_nm]
            assert column.jacobian is not None
            assert column.jacobian.shape == nominal.outputs.shape
        else:
            # a useful derivative was not fabricated
            assert column.jacobian is None
    # at a quiet flat stance the trunk column is smooth and central
    trunk = next(c for c in columns if c.coordinate == "trunk")
    assert trunk.classification == "CENTRAL"


def test_derivative_eligibility_rule():
    assert derivative_eligibility("A", "A", "A") == DERIVATIVE_CENTRAL_ALLOWED
    assert derivative_eligibility("A", "A", "B") == DERIVATIVE_ONE_SIDED_SAME_MODE
    assert derivative_eligibility("A", "B", "A") == DERIVATIVE_ONE_SIDED_SAME_MODE
    assert derivative_eligibility("A", "B", "B") == DERIVATIVE_UNAVAILABLE
    assert derivative_eligibility("A", "B", "C") == DERIVATIVE_UNAVAILABLE


# ---------------------------------------------------------------------------
# controller: fixed constraint catalog, live identity, fallback telemetry
# ---------------------------------------------------------------------------
def _run_controller_samples(steps: int = 12, config: V3LandingConfig | None = None):
    plant, data, authority = _flat_stance()
    controller = V3LandingController(plant, data, config=config or V3LandingConfig(),
                                     actuation=authority)
    steps_out = []
    frames: list[M.V3NativeFrame] = []
    import mujoco
    for k in range(steps):
        frame = M.native_frame(plant, data, k, k * M.NATIVE_DT_S)
        snapshot = M.measure(plant, data)
        frames.append(frame)
        step = controller.update(frame, snapshot, frames, plant, data)
        steps_out.append(step)
        data.ctrl[:] = step.applied_nm
        mujoco.mj_step(plant.model, data)
        mujoco.mj_forward(plant.model, data)
    return plant, data, controller, steps_out


def test_constraint_row_catalog_is_fixed_in_every_solve():
    _, _, _, steps = _run_controller_samples(12)
    control_steps = [s for s in steps if s.control_update]
    assert control_steps
    for step in control_steps:
        names = [row.name for row in step.constraints]
        assert names == list(CONSTRAINT_ROW_NAMES)
        assert "FZ_LEFT_MIN" in names and "FZ_RIGHT_MIN" in names


def test_live_branch_identity_and_applied_action():
    _, _, _, steps = _run_controller_samples(12)
    identities = [s.live_branch_identity for s in steps if s.live_branch_identity is not None]
    assert identities and all(value is True for value in identities)
    last_validated = None
    for step in steps:
        assert np.allclose(step.applied_nm, np.asarray(step.actuation.applied_nm))
        if step.control_update:
            last_validated = step.validated_desired_nm
        else:
            # held native steps carry the same validated desired action
            assert last_validated is not None
            assert np.allclose(step.validated_desired_nm, last_validated)


def test_fallback_telemetry_records_actual_action(monkeypatch):
    plant, data, authority = _flat_stance()
    controller = V3LandingController(plant, data, config=V3LandingConfig(), actuation=authority)
    import mujoco
    # advance one full control cell so the next sample is a solve sample and a
    # previous applied moment exists
    for k in range(CONTROL_INTERVAL_NATIVE_STEPS):
        frame = M.native_frame(plant, data, k, k * M.NATIVE_DT_S)
        snapshot = M.measure(plant, data)
        step = controller.update(frame, snapshot, [frame], plant, data)
        data.ctrl[:] = step.applied_nm
        mujoco.mj_step(plant.model, data)
        mujoco.mj_forward(plant.model, data)

    previous = np.asarray(controller.actuation.previous_applied, dtype=np.float64).copy()
    original = controller._hard_gate_failures

    def fail_all_but_fallback(outcome, *, bilateral_established,
                              e10_sustain_active=False, prior_support_free_run=0):
        if np.allclose(outcome.desired_nm, previous):
            return []
        return ["MAX_PENETRATION"]

    monkeypatch.setattr(controller, "_hard_gate_failures", fail_all_but_fallback)
    k = CONTROL_INTERVAL_NATIVE_STEPS
    frame = M.native_frame(plant, data, k, k * M.NATIVE_DT_S)
    snapshot = M.measure(plant, data)
    step = controller.update(frame, snapshot, [frame], plant, data)
    assert step.control_update
    assert step.fallback is True
    assert step.fallback_reason
    assert step.failed_gate
    assert np.allclose(step.validated_desired_nm, previous)
    assert np.allclose(step.applied_nm, np.asarray(step.actuation.applied_nm))
    controller._hard_gate_failures = original
    del monkeypatch


# ---------------------------------------------------------------------------
# corrected D_EST / D_BL event semantics
# ---------------------------------------------------------------------------
def _trace(samples: int = 80, *, establishment_offset: int = 0,
           vz_tail_start: int = 0, bilateral_until: int | None = None,
           support_gap: tuple[int, int] | None = None) -> V3LandingTrace:
    time = np.arange(samples, dtype=np.float64) * 0.002
    left = np.full(samples, 500.0)
    right = np.full(samples, 500.0)
    legal = np.full(samples, 4, dtype=np.int64)
    left_legal = np.full(samples, 2, dtype=np.int64)
    right_legal = np.full(samples, 2, dtype=np.int64)
    left[:establishment_offset] = 0.0
    left_legal[:establishment_offset] = 0
    if bilateral_until is not None:
        left[bilateral_until:] = 0.0
        left_legal[bilateral_until:] = 0
    vy = np.zeros(samples)
    vy[:] = -1.0
    vy[vz_tail_start:] = 0.0
    if support_gap is not None:
        a, b = support_gap
        legal[a:b] = 0
        left_legal[a:b] = 0
        right_legal[a:b] = 0
        left[a:b] = 0.0
        right[a:b] = 0.0
    return V3LandingTrace(
        index=np.arange(samples, dtype=np.int64),
        time_s=time,
        legal_plantar_active=legal,
        left_legal_plantar_active=left_legal,
        right_legal_plantar_active=right_legal,
        left_fz_n=left,
        right_fz_n=right,
        prohibited_detected=np.zeros(samples, dtype=np.int64),
        prohibited_active=np.zeros(samples, dtype=np.int64),
        com_world_m=np.tile(np.array([[0.0, 0.0, 1.0]]), (samples, 1)),
        com_velocity_world_m_s=np.stack(
            [np.zeros(samples), np.zeros(samples), vy], axis=1),
        hy_kg_m2_s=np.zeros(samples),
        root_pitch_rad=np.zeros(samples),
        trunk_pitch_rad=np.zeros(samples),
        root_pitch_rate_rad_s=np.zeros(samples),
        trunk_pitch_rate_rad_s=np.zeros(samples),
        total_floor_fz_n=(left + right),
        max_penetration_m=np.zeros(samples),
        rom_margin_min_rad=np.full(samples, 0.5),
        max_abs_moment_ratio=np.full(samples, 0.5),
        max_abs_power_ratio=np.full(samples, 0.5),
        first_contact_position=0,
    )


def test_d_est_latency_bound_at_trace_level():
    on_time = _trace(80, establishment_offset=25, vz_tail_start=25)
    report = evaluate_landing_trace(on_time)
    assert report["d_est"]["value_s"] == pytest.approx(0.050)
    assert report["d_est"]["satisfied"] is True
    late = _trace(80, establishment_offset=26, vz_tail_start=26)
    report = evaluate_landing_trace(late)
    assert report["d_est"]["value_s"] == pytest.approx(0.052)
    assert report["d_est"]["satisfied"] is False
    assert report["status"] == "FAIL"


def test_d_bl_sustain_confirms_at_25_intervals():
    # establish and reach the vz tail immediately; 26 inclusive true samples
    trace = _trace(80, establishment_offset=0, vz_tail_start=0)
    run_onset, e10 = e10_confirmation_position(trace)
    assert run_onset == 0
    assert e10 == 25
    report = evaluate_landing_trace(trace)
    assert report["e10"]["reached"] is True
    assert report["e10"]["d_bl_s"] == pytest.approx(0.050)
    assert report["e10"]["point"]["sample"] == 25


def test_d_bl_24_intervals_must_not_confirm():
    trace = _trace(80, establishment_offset=0, vz_tail_start=0, bilateral_until=25)
    run_onset, e10 = e10_confirmation_position(trace)
    assert e10 is None
    report = evaluate_landing_trace(trace)
    assert report["e10"]["reached"] is False


def test_material_reflight_interval_detected():
    trace = _trace(120, establishment_offset=0, vz_tail_start=0, support_gap=(40, 66))
    report = evaluate_landing_trace(trace)
    assert report["hard_gate_audit"]["gates"]["no_material_reflight"] is False
    assert report["hard_gate_audit"]["material_reflight_intervals"]


def test_prohibited_contact_fails_hard_gate():
    trace = _trace(80, establishment_offset=0, vz_tail_start=0)
    prohibited = np.zeros(80, dtype=np.int64)
    prohibited[10] = 1
    trace = V3LandingTrace(**{**{
        name: getattr(trace, name) for name in trace.__dataclass_fields__},
        "prohibited_detected": prohibited})
    report = evaluate_landing_trace(trace)
    assert report["hard_gate_audit"]["gates"]["no_prohibited_or_fall_contact"] is False


def test_e10_point_gate_values_present():
    trace = _trace(80, establishment_offset=0, vz_tail_start=0)
    report = evaluate_landing_trace(trace)
    point = report["e10"]["point"]
    assert set(point["gates"]) == {
        "max_abs_com_vx", "max_abs_Hy", "max_abs_root_pitch", "max_abs_trunk_pitch",
        "max_abs_root_pitch_rate", "max_abs_trunk_pitch_rate"}
    assert point["all_pass"] is True
    assert np.isfinite(report["first_contact_to_e10_max_abs_hy"])


def test_landing_phase_machine_names():
    assert [phase.value for phase in V3LandingPhase] == [
        "LANDING_WAIT", "IMPACT_ABSORPTION", "LANDING_CAPTURE", "E10_CONFIRMED"]


def test_cumulative_mtp_ledger_survives_the_res86_handoff_exactly():
    """The RES-86 landing continues the RES-85 MTP ledger with no reset.

    The launch is replayed from the sealed RES-85 authority, the handoff
    snapshot is restored onto a fresh authority (the RES-86 landing pattern)
    and the per-foot cumulative active/total positive work is bit-identical.
    A landing-supported command then spends from the *same* cumulative value
    and never exceeds the frozen late-phase budgets.
    """
    from loaded_cmj.v3.landing_runtime import build_handoff_certificate

    certificate, _events = build_handoff_certificate()
    handoff_ledger = certificate.actuation.mtp_ledger
    assert handoff_ledger[0].active_gated is False
    assert handoff_ledger[1].active_gated is False
    assert handoff_ledger[0].active_positive_work_j > 0.0
    assert abs(handoff_ledger[0].active_positive_work_j
               - handoff_ledger[1].active_positive_work_j) < 1.0e-12

    restored = V3ActuationAuthority(0.002)
    restored.restore_state(certificate.actuation)
    assert restored.internal_mtp_ledger == handoff_ledger
    assert restored.previous_applied.tolist() == list(
        certificate.actuation.previous_applied_nm)

    command = np.zeros(N_CHANNELS)
    command[7] = command[8] = 20.0
    qdot = np.zeros(N_CHANNELS)
    qdot[7] = qdot[8] = 2.0
    ledger = restored.internal_mtp_ledger
    for _ in range(5):
        applied, _record, ledger = restored.apply(
            command, qdot, phase="IMPACT_ABSORPTION", mtp_active_allowed=(True, True),
            mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
        assert applied[7] != 0.0 and applied[8] != 0.0
    assert ledger[0].active_positive_work_j > handoff_ledger[0].active_positive_work_j
    late_budget = MTP_ACTIVE_POSITIVE_WORK_BUDGET_J * MTP_LATE_ACTIVE_FRACTION
    for entry in ledger:
        assert entry.active_positive_work_j <= late_budget + 1.0e-9
        assert entry.total_positive_work_j <= MTP_TOTAL_POSITIVE_WORK_BUDGET_J + 1.0e-9


def test_landing_control_space_includes_the_active_mtp_coordinate():
    """The landing control/feasibility space includes the active MTP pair.

    The MTP coordinate is a declared decision variable and the shared authority
    now permits a bounded active MTP moment while the foot is legally supported.
    """
    plant, data, authority = _flat_stance()
    controller = V3LandingController(plant, data, config=V3LandingConfig(),
                                     actuation=authority)
    assert "mtp_pair" in CONTROL_COORDINATES
    assert V3LandingConfig().include_mtp_coordinate is True
    assert controller.config.coordinate_ceiling("mtp_pair") == 45.0
    desired = np.zeros(N_CHANNELS)
    desired[7] = desired[8] = 12.0
    _applied, record, ledger = controller._apply_live(desired, data)
    assert record.mtp_gated == (False, False)
    assert record.mtp_active_applied_nm[0] != 0.0
    assert record.mtp_active_applied_nm[1] != 0.0
    assert ledger[0].active_gated is False and ledger[1].active_gated is False
