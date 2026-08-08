#!/usr/bin/env python3
"""Deterministic event event, termination, raw-metric, and negative-control suite."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path

import numpy as np

_VALIDATION_ROOT = Path(__file__).resolve().parent
_TASK_ROOT = _VALIDATION_ROOT.parent
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.biomechanics.events import (
    BASELINE_DT_S,
    MASS_KG,
    ContactLatchTracker,
    EventName,
    CMJEventDetector,
    BiomechanicalSample,
    TerminationClass,
    detect_contact_chatter,
)
from loaded_cmj.simulation.plant import build_model
from loaded_cmj.simulation.constants import GRAVITY_MAGNITUDE, PHYSICS_TIMESTEP_S

WEIGHT_N = MASS_KG * float(GRAVITY_MAGNITUDE)
ROOT = Path(__file__).resolve().parents[3]


def _plate(total_force_N: float, *, left_fraction: float = 0.5, off_plate_N: float = 0.0) -> np.ndarray:
    left = float(total_force_N) * float(left_fraction)
    right = float(total_force_N) - left
    result = np.zeros((3, 6), dtype=np.float64)
    result[0, 2] = left
    result[1, 2] = right
    result[0, 3] = left * 0.10
    result[1, 3] = -right * 0.10
    result[2, 2] = float(off_plate_N)
    return result


def _force_profile(time_s: float, *, landing_end_s: float = 1.520) -> float:
    """Piecewise force profile used only for deterministic physical fixtures."""
    if time_s < 0.25:
        return WEIGHT_N
    if time_s < 0.65:
        return WEIGHT_N - MASS_KG * 1.25
    if time_s < 1.05:
        return WEIGHT_N + MASS_KG * 1.25
    if time_s < 1.25:
        return WEIGHT_N + MASS_KG * 6.0
    if time_s < 1.30:
        return (WEIGHT_N + MASS_KG * 6.0) * (1.0 - (time_s - 1.25) / 0.05)
    if time_s < 1.46:
        return 0.0
    if time_s < landing_end_s:
        return WEIGHT_N + MASS_KG * 8.0
    return WEIGHT_N


def valid_trace(dt_s: float = 0.000125, *, duration_s: float = 2.70, landing_end_s: float = 1.520) -> list[BiomechanicalSample]:
    """Build a deterministic physical trace with analytic segment dynamics."""
    times = np.arange(0.0, duration_s + 0.5 * dt_s, dt_s, dtype=np.float64)
    total_force = np.asarray([_force_profile(float(t), landing_end_s=landing_end_s) for t in times])
    acceleration = (total_force - WEIGHT_N) / MASS_KG
    velocity = np.zeros_like(times)
    position = np.ones_like(times)
    for index in range(1, len(times)):
        velocity[index] = velocity[index - 1] + 0.5 * (
            acceleration[index - 1] + acceleration[index]
        ) * dt_s
        position[index] = position[index - 1] + 0.5 * (
            velocity[index - 1] + velocity[index]
        ) * dt_s
    return [
        BiomechanicalSample(
            time_s=float(time_s),
            com_position_m=(0.0, 0.0, float(z)),
            com_velocity_mps=(0.0, 0.0, float(vz)),
            plate_wrench_N_Nm=_plate(float(force)),
            sole_separation_m=0.020,
            support_polygon_margin_m=(0.050, 0.050),
        )
        for time_s, z, vz, force in zip(times, position, velocity, total_force)
    ]


def _mutate(samples: list[BiomechanicalSample], fn: Callable[[int, BiomechanicalSample], BiomechanicalSample]) -> list[BiomechanicalSample]:
    return [fn(index, sample) for index, sample in enumerate(samples)]


def _at_or_after(samples: list[BiomechanicalSample], time_s: float) -> int:
    return next(index for index, sample in enumerate(samples) if sample.time_s >= time_s - 1e-12)


def fixture_shallow_non_countermovement() -> list[BiomechanicalSample]:
    samples = valid_trace()
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 0.25 < sample.time_s < 0.35:
            phase = (sample.time_s - 0.25) / 0.10
            vz = -0.04 * (2.0 * phase if phase <= 0.5 else 2.0 * (1.0 - phase))
            return replace(sample, com_velocity_mps=(0.0, 0.0, vz))
        if 0.35 <= sample.time_s < 1.05:
            return replace(sample, com_velocity_mps=(0.0, 0.0, 0.0))
        return sample
    return _mutate(samples, change)


def fixture_false_reversal_noise() -> list[BiomechanicalSample]:
    samples = valid_trace()
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 0.36 <= sample.time_s < 0.375:
            return replace(sample, com_velocity_mps=(0.0, 0.0, 0.02))
        if 0.375 <= sample.time_s < 0.39:
            return replace(sample, com_velocity_mps=(0.0, 0.0, -0.02))
        return sample
    return _mutate(samples, change)


def fixture_premature_unloading() -> list[BiomechanicalSample]:
    samples = valid_trace()
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 1.25 <= sample.time_s < 1.31:
            # Left pad unloads briefly; right remains supported, so no
            # bilateral release/takeoff may be accepted.
            plate = sample.plate_wrench_N_Nm.copy()
            plate[0, :] = 0.0
            return replace(sample, plate_wrench_N_Nm=plate)
        return sample
    return _mutate(samples, change)


def fixture_takeoff_chatter() -> list[BiomechanicalSample]:
    samples = valid_trace()
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 1.265 <= sample.time_s < 1.275:
            plate = sample.plate_wrench_N_Nm.copy()
            plate[0, :] = 0.0
            plate[1, :] = 0.0
            return replace(sample, plate_wrench_N_Nm=plate)
        return sample
    return _mutate(samples, change)


def fixture_flight_contact_inconsistency() -> list[BiomechanicalSample]:
    samples = valid_trace()
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 1.35 <= sample.time_s < 1.36:
            plate = sample.plate_wrench_N_Nm.copy()
            plate[2, 2] = 10.0
            return replace(sample, plate_wrench_N_Nm=plate)
        return sample
    return _mutate(samples, change)


def fixture_ascending_contact_anomaly() -> list[BiomechanicalSample]:
    samples = valid_trace()
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 1.365 <= sample.time_s < 1.375:
            plate = sample.plate_wrench_N_Nm.copy()
            plate[0, 2] = 50.0
            plate[1, 2] = 50.0
            plate[0, 3] = 5.0
            plate[1, 3] = -5.0
            return replace(sample, plate_wrench_N_Nm=plate)
        return sample
    return _mutate(samples, change)


def fixture_asymmetric_landing() -> list[BiomechanicalSample]:
    samples = valid_trace()
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 1.46 <= sample.time_s < 1.61:
            return replace(sample, plate_wrench_N_Nm=_plate(_force_profile(sample.time_s), left_fraction=0.90))
        return sample
    return _mutate(samples, change)


def fixture_landing_bounce() -> list[BiomechanicalSample]:
    return valid_trace(landing_end_s=1.66)


def fixture_interrupted_recovery() -> list[BiomechanicalSample]:
    samples = valid_trace(duration_s=2.20)
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if 2.0 <= sample.time_s < 2.005:
            return replace(sample, support_polygon_margin_m=(0.0, 0.0))
        return sample
    return _mutate(samples, change)


def fixture_physical_fall() -> list[BiomechanicalSample]:
    samples = valid_trace(duration_s=2.0)
    def change(index: int, sample: BiomechanicalSample) -> BiomechanicalSample:
        del index
        if sample.time_s >= 1.70:
            return replace(
                sample,
                com_position_m=(0.0, 0.0, 0.30),
                trunk_tilt_rad=1.30,
                recovery_posture_valid=False,
            )
        return sample
    return _mutate(samples, change)


def fixture_agent_fault() -> list[BiomechanicalSample]:
    samples = valid_trace(duration_s=1.0)
    index = _at_or_after(samples, 0.40)
    return _mutate(samples, lambda i, sample: replace(sample, agent_fault_reason="policy_timeout") if i == index else sample)


def fixture_internal_error() -> list[BiomechanicalSample]:
    samples = valid_trace(duration_s=1.0)
    index = _at_or_after(samples, 0.40)
    return _mutate(
        samples,
        lambda i, sample: replace(sample, com_velocity_mps=(0.0, 0.0, float("nan")))
        if i == index
        else sample,
    )


def _digest(result: object) -> str:
    encoded = json.dumps(result.to_jsonable(), sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def run_suite() -> dict[str, object]:
    evaluator = CMJEventDetector()
    checks: dict[str, object] = {}

    # Force-plate contract and latch primitives.
    no_contact = BiomechanicalSample(0.0, (0, 0, 1), (0, 0, 0), np.zeros((3, 6)))
    _assert(not np.any(no_contact.cop_valid), "no-contact COP must be invalid")
    _assert(np.allclose(no_contact.cop_xy_m, 0.0), "no-contact COP must be [0,0]")
    loaded = BiomechanicalSample(0.0, (0, 0, 1), (0, 0, 0), _plate(WEIGHT_N))
    _assert(np.all(loaded.cop_valid), "bilateral COP must be valid above Fz_COP_min")
    _assert(np.allclose(loaded.cop_xy_m[:, 1], (0.10, -0.10)), "COP moment transport/sign mismatch")
    offplate = BiomechanicalSample(0.0, (0, 0, 1), (0, 0, 0), _plate(WEIGHT_N, off_plate_N=50.0))
    _assert(np.isclose(offplate.support_wrench_N_Nm[2], WEIGHT_N + 50.0), "off-plate wrench was dropped")
    common_origin = np.arange(6, dtype=float) + 1.0
    transported = BiomechanicalSample(
        0.0,
        (0, 0, 1),
        (0, 0, 0),
        _plate(WEIGHT_N),
        whole_support_wrench_N_Nm=common_origin,
    )
    _assert(
        np.allclose(transported.support_wrench_N_Nm, common_origin),
        "common-origin support wrench was not preserved separately from plate COP moments",
    )

    tracker = ContactLatchTracker(0.005)
    transitions = []
    for index, force in enumerate(([0, 0], [3, 3], [3, 3], [0, 0], [0, 0], [0, 0], [0, 0])):
        transitions.extend(tracker.step(index * 0.005, force))
    _assert([(t.kind, t.foot) for t in transitions] == [("on", 0), ("on", 1), ("off", 0), ("off", 1)], "latch/debounce transitions wrong")
    _assert(abs(transitions[0].crossing_time_s - (2.0 / 3.0) * 0.005) < 1e-12, "on transition was not backdated")
    quiet_forces = [[WEIGHT_N / 2, WEIGHT_N / 2]] * 8
    chatter_forces = [[0.0, 0.0], [3.0, 3.0], [3.0, 3.0], [0.0, 0.0], [3.0, 3.0], [3.0, 3.0]]
    _assert(not detect_contact_chatter(np.arange(len(quiet_forces)) * 0.005, quiet_forces), "quiet hold flagged chatter")
    _assert(detect_contact_chatter(np.arange(len(chatter_forces)) * 0.005, chatter_forces), "chatter predicate did not reject dropout")
    checks["force_plate_and_latch"] = "PASS"

    positive = evaluator.evaluate(valid_trace())
    expected_events = [event.value for event in EventName]
    _assert(positive.termination_class is TerminationClass.OBJECTIVE_COMPLETE, "valid trace did not complete")
    _assert(all(positive.event_valid.get(name, False) for name in expected_events), "valid trace missed an event")
    _assert(positive.flags["event_ordering"] and positive.flags["no_phase_overlap"], "event order/phase overlap failed")
    _assert(not positive.flags["contact_chatter"], "valid trace flagged chatter")
    _assert(positive.raw_metrics["propulsion"]["interval_start_time_s"] == positive.events[EventName.UPWARD_REVERSAL.value], "propulsion did not start at t_a=t_rev")
    _assert(positive.raw_metrics["propulsion"]["interval_end_time_s"] == positive.events[EventName.VALID_TAKEOFF.value], "propulsion did not end at takeoff")
    _assert(positive.raw_metrics["propulsion"]["impulse_momentum_residual_relative"] <= 0.005, "propulsion impulse closure failed")
    _assert(positive.raw_metrics["landing"]["impulse_momentum_residual_relative"] <= 0.005, "landing impulse closure failed")
    _assert(positive.raw_metrics["flight"]["ballistic_residual_normalized"] <= 1.0, "ballistic consistency failed")
    force_event_samples = positive.raw_metrics["force_plate"]["event_samples"]
    kinematic_event_samples = positive.raw_metrics["kinematics"]["event_samples"]
    contact_event_samples = positive.raw_metrics["contact_state"]["event_samples"]
    _assert(set(force_event_samples) == set(positive.events), "raw force-plate event projection is incomplete")
    _assert(set(kinematic_event_samples) == set(positive.events), "raw kinematic event projection is incomplete")
    _assert(set(contact_event_samples) == set(positive.events), "raw contact-state event projection is incomplete")
    _assert(
        np.asarray(force_event_samples[EventName.VALID_TAKEOFF.value]["bilateral_wrench_N_Nm"]).shape == (2, 6),
        "bilateral force/moment rows were not retained at takeoff",
    )
    _assert(
        np.asarray(kinematic_event_samples[EventName.APEX.value]["com_position_m"]).shape == (3,),
        "COM position was not retained at apex",
    )
    _assert(
        np.allclose(
            positive.raw_metrics["momentum_arrest"]["momentum_change_kgmps"],
            positive.raw_metrics["landing"]["momentum_change_kgmps"],
        ),
        "momentum-arrest raw metric does not match landing momentum change",
    )
    _assert(
        np.allclose(
            positive.raw_metrics["impulses"]["landing_net_impulse_Ns"],
            positive.raw_metrics["landing"]["net_impulse_Ns"],
        ),
        "landing impulse raw projection is incomplete",
    )
    _assert(
        set(positive.raw_metrics["phase"]["event_times_s"]) == set(positive.events),
        "phase timestamp projection is incomplete",
    )
    _assert(positive.events[EventName.IMPACT_ABSORPTION.value] < positive.events[EventName.CAPTURED_SUPPORTED_STATE.value], "impact absorption must precede capture")
    _assert(positive.events[EventName.CAPTURED_SUPPORTED_STATE.value] < positive.events[EventName.BOUNDED_RECOVERY.value], "capture must precede recovery completion")
    checks["valid_nominal_chain"] = positive.to_jsonable()

    negative: dict[str, object] = {}
    cases: list[tuple[str, list[BiomechanicalSample], TerminationClass, str]] = [
        ("shallow_non_countermovement", fixture_shallow_non_countermovement(), TerminationClass.INCOMPLETE_HORIZON, EventName.UPWARD_REVERSAL.value),
        ("false_reversal_noise", fixture_false_reversal_noise(), TerminationClass.OBJECTIVE_COMPLETE, EventName.UPWARD_REVERSAL.value),
        ("premature_unloading", fixture_premature_unloading(), TerminationClass.INCOMPLETE_HORIZON, EventName.VALID_TAKEOFF.value),
        ("takeoff_contact_chatter", fixture_takeoff_chatter(), TerminationClass.INCOMPLETE_HORIZON, EventName.CONTACT_FREE_FLIGHT.value),
        ("flight_contact_inconsistency", fixture_flight_contact_inconsistency(), TerminationClass.INCOMPLETE_HORIZON, EventName.CONTACT_FREE_FLIGHT.value),
        ("ascending_contact_anomaly", fixture_ascending_contact_anomaly(), TerminationClass.INCOMPLETE_HORIZON, EventName.CONTACT_FREE_FLIGHT.value),
        ("symmetric_landing", valid_trace(), TerminationClass.OBJECTIVE_COMPLETE, EventName.IMPACT_ABSORPTION.value),
        ("asymmetric_landing", fixture_asymmetric_landing(), TerminationClass.OBJECTIVE_COMPLETE, EventName.IMPACT_ABSORPTION.value),
        ("landing_bounce", fixture_landing_bounce(), TerminationClass.INCOMPLETE_HORIZON, EventName.IMPACT_ABSORPTION.value),
        ("interrupted_recovery", fixture_interrupted_recovery(), TerminationClass.INCOMPLETE_HORIZON, EventName.BOUNDED_RECOVERY.value),
    ]
    for name, trace, terminal, event_key in cases:
        result = evaluator.evaluate(trace)
        _assert(result.termination_class is terminal, f"{name}: wrong terminal class {result.termination_class}")
        if name in {"shallow_non_countermovement", "premature_unloading", "takeoff_contact_chatter", "flight_contact_inconsistency", "ascending_contact_anomaly", "landing_bounce", "interrupted_recovery"}:
            _assert(not result.event_valid.get(event_key, False), f"{name}: rejected event was accepted")
        if name == "false_reversal_noise":
            _assert(result.events[EventName.UPWARD_REVERSAL.value] > 0.8, "false reversal noise became the trusted reversal")
        if name == "asymmetric_landing":
            _assert(result.raw_metrics["landing"]["bilateral_impulse_asymmetry"] > 0.20, "asymmetric landing was not measured")
        negative[name] = result.to_jsonable()
    checks["negative_controls"] = negative

    fall = evaluator.evaluate(fixture_physical_fall())
    _assert(fall.termination_class is TerminationClass.PHYSICAL_FALL, "physical fall was not distinguished")
    horizon = evaluator.evaluate(valid_trace(duration_s=0.90))
    _assert(horizon.termination_class is TerminationClass.INCOMPLETE_HORIZON, "horizon incompletion was not distinguished")
    agent_fault = evaluator.evaluate(fixture_agent_fault())
    _assert(agent_fault.termination_class is TerminationClass.AGENT_FAULT, "agent fault was not distinguished")
    internal = evaluator.evaluate(fixture_internal_error())
    _assert(internal.termination_class is TerminationClass.INTERNAL_EVALUATION_ERROR, "internal error was not distinguished")
    both_fault_and_fall = fixture_physical_fall()
    fault_index = _at_or_after(both_fault_and_fall, 1.70)
    both_fault_and_fall[fault_index] = replace(both_fault_and_fall[fault_index], agent_fault_reason="policy_exception")
    _assert(evaluator.evaluate(both_fault_and_fall).termination_class is TerminationClass.AGENT_FAULT, "agent fault precedence failed")
    both_internal_and_agent = fixture_agent_fault()
    internal_index = _at_or_after(both_internal_and_agent, 0.40)
    both_internal_and_agent[internal_index] = replace(
        both_internal_and_agent[internal_index],
        agent_fault_reason="policy_exception",
        com_velocity_mps=(0.0, 0.0, float("nan")),
    )
    _assert(evaluator.evaluate(both_internal_and_agent).termination_class is TerminationClass.INTERNAL_EVALUATION_ERROR, "internal error precedence failed")
    checks["termination_classes_and_precedence"] = {
        "physical_fall": fall.termination_class.value,
        "incomplete_horizon": horizon.termination_class.value,
        "agent_fault": agent_fault.termination_class.value,
        "internal_evaluation_error": internal.termination_class.value,
        "agent_over_fall": TerminationClass.AGENT_FAULT.value,
        "internal_over_agent": TerminationClass.INTERNAL_EVALUATION_ERROR.value,
    }

    # Fresh deterministic replays of the exact same physical trace.
    replay_results = [evaluator.evaluate(valid_trace()) for _ in range(3)]
    replay_digests = [_digest(result) for result in replay_results]
    _assert(len(set(replay_digests)) == 1, "three replay digests differ")
    checks["deterministic_replay"] = {"digests": replay_digests, "identical": True}

    # Qualified refinement copies: only sample resolution changes; the Plant
    # identity and selected qualification profile remain frozen.
    refinements: dict[str, object] = {}
    base = positive
    for dt in (0.0005, 0.00025, 0.000125):
        refined = evaluator.evaluate(valid_trace(dt_s=dt))
        _assert(refined.termination_class is TerminationClass.OBJECTIVE_COMPLETE, f"refinement dt={dt} did not complete")
        for event_name in (
            EventName.COUNTERMOVEMENT_ONSET.value,
            EventName.VALID_COUNTERMOVEMENT.value,
            EventName.UPWARD_REVERSAL.value,
            EventName.VALID_TAKEOFF.value,
            EventName.APEX.value,
            EventName.VALID_LANDING.value,
            EventName.IMPACT_ABSORPTION.value,
            EventName.CAPTURED_SUPPORTED_STATE.value,
            EventName.BOUNDED_RECOVERY.value,
            EventName.COMPLETION.value,
        ):
            _assert(abs(refined.events[event_name] - base.events[event_name]) <= 0.01, f"event refinement drift: {event_name}")
        _assert(abs(refined.raw_metrics["propulsion"]["vertical_impulse_Ns"] - base.raw_metrics["propulsion"]["vertical_impulse_Ns"]) <= 0.50, f"propulsion refinement drift dt={dt}")
        _assert(abs(refined.raw_metrics["landing"]["vertical_impulse_Ns"] - base.raw_metrics["landing"]["vertical_impulse_Ns"]) <= 0.50, f"landing refinement drift dt={dt}")
        refinements[str(dt)] = {
            "termination": refined.termination_class.value,
            "events": refined.events,
            "propulsion_vertical_impulse_Ns": refined.raw_metrics["propulsion"]["vertical_impulse_Ns"],
            "landing_vertical_impulse_Ns": refined.raw_metrics["landing"]["vertical_impulse_Ns"],
        }
    checks["refinement_profiles"] = {
        "selected_profile_dt_s": PHYSICS_TIMESTEP_S,
        "qualified_trace_copy_dt_s": [0.0005, 0.00025, 0.000125],
        "results": refinements,
    }

    model = build_model()
    _assert(int(model.nq) == 25 and int(model.nv) == 21 and int(model.nu) == 15, "XML compilation identity changed")
    _assert(abs(float(model.opt.timestep) - BASELINE_DT_S) <= 1e-15, "selected Plant timestep changed")
    checks["xml_compilation"] = {"nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu), "timestep_s": float(model.opt.timestep)}

    # The production evaluator is deliberately score-free and must expose the
    # explicit restrictions in its result projection.
    _assert(not positive.flags["reachable_torque_interval_used"], "advisory torque interval was trusted")
    _assert(not positive.flags["legacy_reset_fixed_hold_used"], "legacy reset path was trusted")
    checks["scope_restrictions"] = "PASS"
    return checks


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", required=True)
    args = parser.parse_args()
    root = Path(args.evidence_root)
    root.mkdir(parents=True, exist_ok=True)
    report: dict[str, object]
    try:
        report = {"result": "PASS", "suite": "event", "checks": run_suite()}
        print("RESULT=PASS event-EVENTS-TERMINATIONS-RAW-METRICS")
    except Exception as exc:  # noqa: BLE001 - failure is persisted as qualification evidence
        report = {
            "result": "FAIL",
            "suite": "event",
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(f"RESULT=FAIL event-EVENTS-TERMINATIONS-RAW-METRICS ({report['error']})")
    (root / "cmj_event_qualification.json").write_text(
        json.dumps(report, indent=2, sort_keys=True, default=lambda value: value.tolist() if isinstance(value, np.ndarray) else value),
        encoding="utf-8",
    )
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
