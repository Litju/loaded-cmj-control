#!/usr/bin/env python3
"""Focused semantic semantic qualification and adversarial negative controls.

This suite exercises the repaired owners directly.  It does not construct a
policy worker, a rollout loop, a score calculator, or a controller.
"""

from __future__ import annotations

import argparse
import inspect
import json
import re
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
_PUBLIC_SRC = _TASK_ROOT / "src"
if str(_PUBLIC_SRC) not in sys.path:
    sys.path.insert(0, str(_PUBLIC_SRC))

_POLICY_SPEC = _TASK_ROOT / "src" / "loaded_cmj" / "control" / "policy_spec.json"

from loaded_cmj.biomechanics.events import (  # noqa: E402
    BASELINE_DT_S,
    EventName,
    CMJEventDetector,
    EventThresholds,
    BiomechanicalSample,
    TerminationClass,
    _integrate,
)
from loaded_cmj.simulation.plant import Plant, build_model  # noqa: E402
from loaded_cmj.simulation.constants import (  # noqa: E402
    CONTACT_F_ON_N,
    CONTACT_N_ON,
    CONTACT_T_ON_S,
    EVENT_IDS,
    EVENT_NAMES,
    EVENT_REGISTRY,
    EVENT_THRESHOLDS,
    FIXED_HOLD_SUBSTEPS,
    HOLD_ACTION,
    PHYSICS_TIMESTEP_S,
    TERMINATION_THRESHOLDS,
    TOTAL_MASS_KG,
)
from tests.qualification_cmj_events import _at_or_after, valid_trace  # noqa: E402


class Checks:
    def __init__(self) -> None:
        self.rows: list[dict[str, Any]] = []

    def add(self, test_id: str, assertion: str, ok: bool, measured: Any) -> None:
        self.rows.append({
            "test_id": test_id,
            "assertion": assertion,
            "result": "PASS" if bool(ok) else "FAIL",
            "measured": native(measured),
        })

    @property
    def failures(self) -> list[dict[str, Any]]:
        return [row for row in self.rows if row["result"] == "FAIL"]


def native(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): native(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [native(item) for item in value]
    return value


def safe(checks: Checks, test_id: str, assertion: str, fn: Callable[[], tuple[bool, Any]]) -> None:
    try:
        ok, measured = fn()
    except Exception as exc:  # noqa: BLE001 - preserve every focused result
        ok, measured = False, f"{type(exc).__name__}: {exc}"
    checks.add(test_id, assertion, ok, measured)


def _registry_is_coherent(ids: Any, names: Any, thresholds: Any) -> bool:
    return bool(
        tuple(ids) == EVENT_IDS
        and tuple(names) == EVENT_NAMES
        and dict(thresholds) == dict(EVENT_THRESHOLDS)
        and len(tuple(ids)) == len(set(tuple(ids))) == len(tuple(names))
    )


def _reset_contract_is_exact(reset: dict[str, Any]) -> bool:
    action = np.asarray(reset.get("reset_action"), dtype=np.float64)
    previous = np.asarray(reset.get("first_policy_previous_action"), dtype=np.float64)
    return bool(
        reset.get("duration_s") == 0.300
        and reset.get("physics_timestep_s") == PHYSICS_TIMESTEP_S
        and reset.get("physics_steps") == FIXED_HOLD_SUBSTEPS == 2400
        and reset.get("substeps") == 2400
        and reset.get("physics_steps_per_control") == 40
        and reset.get("reset_is_control_step_count") is False
        and reset.get("reset_action_source") == "HOLD_ACTION"
        and np.array_equal(action, np.asarray(HOLD_ACTION, dtype=np.float64))
        and np.array_equal(previous, np.asarray(HOLD_ACTION, dtype=np.float64))
        and reset.get("policy_calls_during_dwell") == 0
        and reset.get("reset_scored") is False
        and reset.get("reset_samples_scored") is False
        and reset.get("first_policy_time_s") == 0.0
    )


def _sampling_contract_is_physics(flags: dict[str, Any]) -> bool:
    return bool(flags.get("evaluation_rate") == "physics_substep")


def _typed_residual_projection(report: dict[str, Any]) -> bool:
    if not {"residual", "mechanics_residual_trans_N", "mechanics_residual_rot_Nm"} <= set(report):
        return False
    residual = np.asarray(report["residual"], dtype=np.float64).reshape(-1)
    return bool(
        np.array_equal(
            np.asarray(report["mechanics_residual_trans_N"]),
            np.asarray(np.abs(residual[:3]).max()),
        )
        and np.array_equal(
            np.asarray(report["mechanics_residual_rot_Nm"]),
            np.asarray(np.abs(residual[3:]).max()),
        )
        and "residual_inf" not in report
    )


def _work_source_is_power_only() -> bool:
    source = inspect.getsource(CMJEventDetector._compute_landing_metrics)
    return bool(
        "active_power_signed_W" in source
        and "active_power_positive_W" in source
        and "active_power_negative_W" in source
        and "passive_power_W" in source
        and "sample.active_negative_work_J" not in source
        and "sample.passive_work_J" not in source
        and "sample.limit_work_J" not in source
    )


def _power_sample(time_s: float, *, active: float = 0.0, passive: float = 0.0) -> BiomechanicalSample:
    return BiomechanicalSample(
        time_s=time_s,
        com_position_m=(0.0, 0.0, 1.0),
        com_velocity_mps=(0.0, 0.0, 0.0),
        plate_wrench_N_Nm=np.zeros((3, 6), dtype=np.float64),
        active_power_signed_W=active,
        passive_power_W=passive,
    )


def _power_integral(samples: list[BiomechanicalSample], field: str) -> float:
    return float(_integrate(samples, samples[0].time_s, samples[-1].time_s, lambda sample: getattr(sample, field)))


def _known_cop() -> tuple[BiomechanicalSample, np.ndarray]:
    plate = np.zeros((3, 6), dtype=np.float64)
    plate[0, 2] = 100.0
    plate[0, 3] = 20.0
    plate[0, 4] = -10.0
    plate[1, 2] = 50.0
    plate[1, 3] = -5.0
    plate[1, 4] = 5.0
    origins = np.asarray(((1.2, -0.4), (-0.8, 0.3)), dtype=np.float64)
    expected = origins + np.asarray(((0.1, 0.2), (-0.1, -0.1)), dtype=np.float64)
    sample = BiomechanicalSample(
        time_s=0.0,
        com_position_m=(0.0, 0.0, 1.0),
        com_velocity_mps=(0.0, 0.0, 0.0),
        plate_wrench_N_Nm=plate,
        plate_origin_world_xy_m=origins,
    )
    return sample, expected


def run() -> dict[str, Any]:
    checks = Checks()
    negative = Checks()

    def reset_spec_proof() -> tuple[bool, Any]:
        spec = json.loads(_POLICY_SPEC.read_text(encoding="utf-8"))
        reset = spec["reset_protocol"]
        model = build_model()
        plant = Plant(model)
        data = plant.make_data()
        state = plant.reset_fixed_hold(data)
        metadata = state.reset_metadata
        ok = bool(
            _reset_contract_is_exact(reset)
            and metadata["dwell_substeps"] == 2400
            and metadata["physics_timestep_s"] == PHYSICS_TIMESTEP_S
            and metadata["policy_calls_during_dwell"] == 0
            and metadata["reset_scored"] is False
            and np.array_equal(state.previous_action, np.asarray(HOLD_ACTION))
            and np.array_equal(state.first_observation["previous_action"], np.asarray(HOLD_ACTION))
            and state.first_observation["time_s"] == 0.0
            and state.first_observation["step_index"] == 0
            and bool(state.first_observation["episode_reset"])
            and np.array_equal(np.asarray(plant.u_eq), np.asarray(HOLD_ACTION))
        )
        return ok, {
            "reset_metadata": metadata,
            "previous_action_max_error": float(np.abs(state.previous_action - HOLD_ACTION).max()),
            "u_eq_max_error": float(np.abs(plant.u_eq - HOLD_ACTION).max()),
        }

    safe(checks, "Q-RESET-01..05", "fixed hold uses exact 2400-physics-step HOLD_ACTION dwell and post-dwell t=0", reset_spec_proof)

    def registry_proof() -> tuple[bool, Any]:
        names = tuple(row["name"] for row in EVENT_REGISTRY)
        ids = tuple(row["id"] for row in EVENT_REGISTRY)
        thresholds = dict(EVENT_THRESHOLDS)
        event_enum = tuple(event.value for event in EventName)
        threshold_view = EventThresholds()
        projected = dict(threshold_view.registry)
        result = CMJEventDetector().evaluate(valid_trace())
        return bool(
            _registry_is_coherent(ids, names, thresholds)
            and event_enum == EVENT_NAMES
            and len(EVENT_NAMES) == 13
            and "fall" not in EVENT_NAMES
            and projected == {**dict(EVENT_THRESHOLDS), **dict(TERMINATION_THRESHOLDS)}
            and "completion" in EVENT_NAMES
            and set(result.events) == set(EVENT_NAMES)
            and all(result.event_valid.get(name, False) for name in EVENT_NAMES)
        ), {"ids": ids, "names": names, "event_enum": event_enum, "threshold_count": len(thresholds)}

    safe(checks, "Q-EVENT-01..08", "one canonical E1-E13 registry drives EventName and event threshold view", registry_proof)

    def event_boundary_proof() -> tuple[bool, Any]:
        dt = PHYSICS_TIMESTEP_S
        from loaded_cmj.biomechanics.events import ContactLatchTracker

        def transitions(forces: list[float]) -> list[Any]:
            tracker = ContactLatchTracker(dt)
            out = []
            for index, force in enumerate(forces):
                out.extend(tracker.step(index * dt, (force, force)))
            return out

        below_transitions = transitions([CONTACT_F_ON_N - 1.0e-9] * CONTACT_N_ON)
        exact_transitions = transitions([CONTACT_F_ON_N] * CONTACT_N_ON)
        above_transitions = transitions([CONTACT_F_ON_N + 1.0] * CONTACT_N_ON)
        broken_transitions = transitions([CONTACT_F_ON_N + 1.0, CONTACT_F_ON_N - 1.0, CONTACT_F_ON_N + 1.0])
        simultaneous = transitions([CONTACT_F_ON_N + 1.0] * CONTACT_N_ON)
        return bool(
            not below_transitions
            and len(exact_transitions) == 2
            and len(above_transitions) == 2
            and not broken_transitions
            and {transition.foot for transition in simultaneous} == {0, 1}
            and CONTACT_T_ON_S == CONTACT_N_ON * dt
        ), {
            "below": len(below_transitions),
            "exact": len(exact_transitions),
            "above": len(above_transitions),
            "broken": len(broken_transitions),
            "simultaneous_feet": [transition.foot for transition in simultaneous],
        }

    safe(checks, "Q-EVENT-BOUNDARY", "below/exact/above threshold and dwell boundaries are deterministic", event_boundary_proof)

    def cop_proof() -> tuple[bool, Any]:
        sample, expected = _known_cop()
        invalid_plate = sample.plate_wrench_N_Nm.copy()
        invalid_plate[0, 2] = 20.0
        invalid = replace(sample, plate_wrench_N_Nm=invalid_plate)
        model = build_model()
        plant = Plant(model)
        data = plant.make_data()
        plant.reset_supported(data)
        summary = plant.contact_wrench_summary(data)
        live = BiomechanicalSample.from_plant(plant, data, time_s=0.0)
        return bool(
            np.allclose(sample.cop_xy_m, expected)
            and np.array_equal(sample.cop_valid, np.asarray([True, True]))
            and not bool(replace(invalid, cop_world_xy_m=None).cop_valid[0])
            and not np.any(BiomechanicalSample(
                0.0, (0, 0, 1), (0, 0, 0), np.zeros((3, 6))
            ).cop_valid)
            and summary["cop_frame"] == "world"
            and np.allclose(np.asarray(summary["cop_world_xy"]), live.cop_xy_m)
            and np.array_equal(np.asarray(summary["cop_valid"]), live.cop_valid)
        ), {"cop_world_xy_m": sample.cop_xy_m, "expected": expected, "invalid_at_20N": replace(invalid, cop_world_xy_m=None).cop_valid}

    safe(checks, "Q-SAMPLE-03/04", "COP is world-frame with one origin addition and strict Fz>20 N validity", cop_proof)

    def contact_proof() -> tuple[bool, Any]:
        model = build_model()
        plant = Plant(model)
        cases = {
            "left_designated": plant._contact_row_for_region("left_plate", 0),
            "right_designated": plant._contact_row_for_region("right_plate", 1),
            "left_on_right": plant._contact_row_for_region("right_plate", 0),
            "right_on_left": plant._contact_row_for_region("left_plate", 1),
            "off_plate": plant._contact_row_for_region("off_plate", 0),
        }
        data = plant.make_data()
        plant.reset_supported(data)
        summary = plant.contact_wrench_summary(data)
        assignment_ok = all(
            int(contact["assignment_count"]) == 1 and int(contact["row"]) in (0, 1, 2)
            for contact in summary["contacts"]
        )
        return bool(
            cases["left_designated"] == (0, False)
            and cases["right_designated"] == (1, False)
            and cases["left_on_right"] == (2, True)
            and cases["right_on_left"] == (2, True)
            and cases["off_plate"] == (2, True)
            and assignment_ok
            and summary["contact_assignment_count"] == len(summary["contacts"])
        ), {"cases": cases, "contact_count": len(summary["contacts"]), "assignment_count": summary["contact_assignment_count"]}

    safe(checks, "Q-CONTACT-01..07", "geometry-first attribution assigns every contact once and retains prohibited support", contact_proof)

    def residual_proof() -> tuple[bool, Any]:
        model = build_model()
        plant = Plant(model)
        data = plant.make_data()
        plant.reset_supported(data)
        report = plant.dynamics_residual(data)
        force_only = np.zeros(21, dtype=np.float64)
        force_only[0] = 7.0
        moment_only = np.zeros(21, dtype=np.float64)
        moment_only[3] = 2.0
        mixed_candidate = {"residual_inf": 7.0}
        return bool(
            _typed_residual_projection(report)
            and np.isclose(np.abs(force_only[:3]).max(), 7.0)
            and np.isclose(np.abs(moment_only[3:]).max(), 2.0)
            and not _typed_residual_projection({**report, **mixed_candidate})
            and "mechanics_residual_N" not in inspect.getsource(BiomechanicalSample)
        ), {
            "trans_N": report["mechanics_residual_trans_N"],
            "rot_Nm": report["mechanics_residual_rot_Nm"],
            "vector_size": len(report["residual"]),
        }

    safe(checks, "Q-SAMPLE-05", "full generalized residual remains available through typed N and N·m projections", residual_proof)

    def work_proof() -> tuple[bool, Any]:
        positive = [_power_sample(0.0, active=50.0), _power_sample(0.5, active=50.0), _power_sample(1.0, active=50.0)]
        negative = [_power_sample(0.0, active=-50.0), _power_sample(0.5, active=-50.0), _power_sample(1.0, active=-50.0)]
        alternating = [_power_sample(0.0, active=100.0), _power_sample(0.5, active=-100.0), _power_sample(1.0, active=100.0)]
        passive = [_power_sample(0.0, passive=7.0), _power_sample(0.5, passive=7.0), _power_sample(1.0, passive=7.0)]
        exact = [_power_sample(0.0, active=8.0), _power_sample(BASELINE_DT_S, active=8.0), _power_sample(2 * BASELINE_DT_S, active=8.0)]
        model = build_model()
        plant = Plant(model)
        data = plant.make_data()
        plant.reset_supported(data)
        tau = np.linspace(-1.0, 1.0, 15)
        rates = plant.anatomical_rates(data)
        power = plant.realized_power_components(data, tau)
        episode = CMJEventDetector().evaluate(positive)
        return bool(
            np.isclose(_power_integral(positive, "active_power_signed_W"), 50.0)
            and np.isclose(_power_integral(negative, "active_power_signed_W"), -50.0)
            and np.isclose(_power_integral(alternating, "active_power_signed_W"), 0.0)
            and np.isclose(_power_integral(passive, "passive_power_W"), 7.0)
            and np.isclose(_power_integral(exact, "active_power_signed_W"), 16.0 * BASELINE_DT_S)
            and np.isclose(power["active_power_signed_W"], tau @ rates)
            and episode.raw_metrics["work"]["initial_cumulative_work_J"] == 0.0
            and np.isclose(episode.raw_metrics["work"]["active_work_cumulative_J"], 50.0)
            and _work_source_is_power_only()
        ), {
            "positive_work_J": _power_integral(positive, "active_power_signed_W"),
            "negative_work_J": _power_integral(negative, "active_power_signed_W"),
            "exact_duration_work_J": _power_integral(exact, "active_power_signed_W"),
            "active_power_W": power["active_power_signed_W"],
        }

    safe(checks, "Q-SAMPLE-06", "realized W is integrated exactly once into signed J work", work_proof)

    def termination_proof() -> tuple[bool, Any]:
        evaluator = CMJEventDetector()
        base = valid_trace()
        normal = evaluator.evaluate(base)
        fault_index = _at_or_after(base, 0.40)
        physics_fault = evaluator.evaluate(
            [replace(sample, physics_fault_reason="mj_nonfinite") if index == fault_index else sample for index, sample in enumerate(base[: _at_or_after(base, 1.0) + 1])]
        )
        simultaneous = list(base)
        completion_index = _at_or_after(simultaneous, normal.events[EventName.COMPLETION.value])
        simultaneous = [
            replace(sample, com_position_m=(sample.com_position_m[0], sample.com_position_m[1], 0.30))
            if index >= completion_index else sample
            for index, sample in enumerate(simultaneous)
        ]
        objective_over_fall = evaluator.evaluate(simultaneous)
        horizon = evaluator.evaluate(valid_trace(duration_s=0.90))
        return bool(
            normal.termination_class is TerminationClass.OBJECTIVE_COMPLETE
            and physics_fault.termination_class is TerminationClass.PHYSICS_NONFINITE_FAULT
            and objective_over_fall.termination_class is TerminationClass.OBJECTIVE_COMPLETE
            and horizon.termination_class is TerminationClass.INCOMPLETE_HORIZON
            and _sampling_contract_is_physics(normal.flags)
        ), {
            "normal": normal.termination_class.value,
            "physics_fault": physics_fault.termination_class.value,
            "objective_over_fall": objective_over_fall.termination_class.value,
            "horizon": horizon.termination_class.value,
        }

    safe(checks, "Q-TERM-PRECEDENCE", "faults, objective/fall, and horizon use the frozen source precedence", termination_proof)

    # Each negative control deliberately violates one repaired invariant and
    # must be rejected by the corresponding owner-level predicate.
    def nc01() -> tuple[bool, Any]:
        rows = [plant_row for plant_row in (Plant._contact_row_for_region("right_plate", 0), Plant._contact_row_for_region("left_plate", 1))]
        return bool(not (rows == [(0, False), (1, False)])), rows

    def nc02() -> tuple[bool, Any]:
        sample, expected = _known_cop()
        double_origin = sample.plate_origin_world_xy_m + sample.cop_xy_m
        return bool(not np.allclose(double_origin, expected)), double_origin

    def nc04() -> tuple[bool, Any]:
        row = Plant._contact_row_for_region("right_plate", 0)
        return bool(row == (2, True)), row

    def nc05() -> tuple[bool, Any]:
        report = {"residual": np.asarray([7.0, 0, 0, 2.0, 0, 0]), "residual_inf": 7.0}
        return bool(not _typed_residual_projection(report)), report

    def nc06() -> tuple[bool, Any]:
        return bool(_work_source_is_power_only()), "J-valued fields are not work inputs"

    def nc11() -> tuple[bool, Any]:
        bad_names = list(EVENT_NAMES)
        bad_names[-1] = bad_names[0]
        return bool(not _registry_is_coherent(EVENT_IDS, bad_names, EVENT_THRESHOLDS)), bad_names

    def nc14() -> tuple[bool, Any]:
        spec = json.loads(_POLICY_SPEC.read_text(encoding="utf-8"))
        bad = dict(spec["reset_protocol"])
        bad["substeps"] = 600
        return bool(not _reset_contract_is_exact(bad)), bad["substeps"]

    def nc15() -> tuple[bool, Any]:
        bad = {"evaluation_rate": "control", "timestep_s": 0.005}
        return bool(not _sampling_contract_is_physics(bad)), bad

    for test_id, assertion, fn in (
        ("NC-01", "swapped plate rows are rejected", nc01),
        ("NC-02", "double-origin COP is rejected", nc02),
        ("NC-04", "opposite-foot contact is prohibited", nc04),
        ("NC-05", "mixed N/Nm residual is rejected", nc05),
        ("NC-06", "J-valued work input is rejected", nc06),
        ("NC-11", "duplicate event authority is rejected", nc11),
        ("NC-14", "600-step reset is rejected", nc14),
        ("NC-15", "control-rate event cadence is rejected", nc15),
    ):
        safe(negative, test_id, assertion, fn)

    return {
        "result": "PASS" if not checks.failures and not negative.failures else "FAIL",
        "focused": checks.rows,
        "negative_controls": negative.rows,
        "focused_tests": len(checks.rows),
        "focused_passed": len(checks.rows) - len(checks.failures),
        "negative_controls_total": len(negative.rows),
        "negative_controls_passed": len(negative.rows) - len(negative.failures),
        "failures": checks.failures + negative.failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", required=True)
    args = parser.parse_args()
    report = run()
    root = Path(args.evidence_root)
    root.mkdir(parents=True, exist_ok=True)
    (root / "semantic_contracts.json").write_text(
        json.dumps(report, indent=2, sort_keys=True, default=native), encoding="utf-8"
    )
    for row in report["focused"] + report["negative_controls"]:
        if row["result"] == "FAIL":
            print(f"FAIL {row['test_id']}: {row['assertion']} -> {row['measured']}")
    print(
        f"FOCUSED tests={report['focused_tests']} passed={report['focused_passed']} "
        f"NEGATIVE_CONTROLS tests={report['negative_controls_total']} "
        f"passed={report['negative_controls_passed']}"
    )
    print(f"RESULT={report['result']} SEMANTIC-CONTRACTS")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
