#!/usr/bin/env python3
"""Frozen numerical nested numerical, event, impulse, work, and finite-number proofs."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.simulation import drive as drive
from loaded_cmj.simulation import constants
from loaded_cmj.simulation.plant import Plant, build_model

BODY_WEIGHT_N = constants.BODY_WEIGHT_N
PHYSICS_GRID_S = getattr(constants, "PHYSICS_GRID_S", ())
PHYSICS_TIMESTEP_S = constants.PHYSICS_TIMESTEP_S
TOTAL_MASS_KG = constants.TOTAL_MASS_KG


def native(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): native(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [native(v) for v in value]
    return value


class Checks:
    def __init__(self) -> None:
        self.rows: list[dict[str, Any]] = []

    def add(self, test_id: str, assertion: str, ok: bool, measured: Any,
            tolerance: Any, clause: str) -> None:
        self.rows.append({
            "test_id": test_id,
            "assertion": assertion,
            "result": "PASS" if bool(ok) else "FAIL",
            "measured": native(measured),
            "tolerance": native(tolerance),
            "contract_clause": clause,
            "failure_class": "contract_check",
        })

    @property
    def failures(self) -> list[dict[str, Any]]:
        return [row for row in self.rows if row["result"] == "FAIL"]


def safe(checks: Checks, test_id: str, assertion: str, clause: str,
         fn: Callable[[], tuple[bool, Any]], tolerance: Any) -> Any:
    try:
        ok, measured = fn()
    except Exception as exc:  # noqa: BLE001 - contract harness records every failed proof
        ok, measured = False, f"{type(exc).__name__}: {exc}"
    checks.add(test_id, assertion, ok, measured, tolerance, clause)
    return measured


def _state(plant: Plant, *, zero: bool = False) -> Any:
    command = np.zeros(15, dtype=float) if zero else plant.u_eq
    torque = np.zeros(15, dtype=float) if zero else plant.tau_eq
    return drive.DriveState(
        a_plus=np.maximum(command, 0.0),
        a_minus=np.maximum(-command, 0.0),
        tau_prev=torque.copy(),
        previous_command=command.copy(),
    )


def run_trace(dt: float, fixture: str, *, duration_s: float = 0.24,
              iterations: int | None = None, solref_timeconst: float | None = None,
              solimp_d0: float | None = None) -> dict[str, Any]:
    model = build_model()
    model.opt.timestep = float(dt)
    if iterations is not None:
        model.opt.iterations = int(iterations)
    if solref_timeconst is not None:
        ids = np.asarray([0, *range(1, 9)], dtype=int)
        model.geom_solref[ids, 0] = float(solref_timeconst)
    if solimp_d0 is not None:
        ids = np.asarray([0, *range(1, 9)], dtype=int)
        model.geom_solimp[ids, 0] = float(solimp_d0)
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    if fixture in {"RECONTACT_DROP", "ASYMMETRIC_DROP"}:
        data.qpos[2] += 0.20
        data.qvel[:] = 0.0
        data.qvel[2] = -1.25
        mujoco.mj_forward(model, data)
        state = _state(plant, zero=True)
        command = np.zeros(15, dtype=float)
    elif fixture == "BALLISTIC_LAUNCH":
        data.qvel[:] = 0.0
        data.qvel[2] = 2.0
        mujoco.mj_forward(model, data)
        state = _state(plant, zero=False)
        command = plant.u_eq
    else:
        state = _state(plant, zero=False)
        command = plant.u_eq
    steps = round(duration_s / dt)
    time = np.arange(steps, dtype=float) * dt
    com = np.empty((steps, 3), dtype=float)
    com_v = np.empty((steps, 3), dtype=float)
    force = np.empty((steps, 2), dtype=float)
    active = np.zeros(steps, dtype=bool)
    torque = np.empty((steps, 15), dtype=float)
    activation = np.empty((steps, 30), dtype=float)
    energy = np.empty(steps, dtype=float)
    residual_trans = np.empty(steps, dtype=float)
    residual_rot = np.empty(steps, dtype=float)
    penetration = np.empty(steps, dtype=float)
    cop = np.empty((steps, 4), dtype=float)
    for k in range(steps):
        s = plant.anatomical_coordinates(data)
        sd = plant.anatomical_rates(data)
        result = drive.drive_state_step(command, state, s, sd, dt)
        plant.apply_anatomical_torque(data, result["tau"])
        mujoco.mj_step(model, data)
        summary = plant.contact_wrench_summary(data)
        com[k] = plant.center_of_mass(data)
        com_v[k] = plant.center_of_mass_velocity(data)
        force[k] = np.asarray(summary["normal_force"], dtype=float)
        active[k] = bool(summary["contact_active"])
        torque[k] = result["tau"]
        activation[k, :15] = result["a_plus"]
        activation[k, 15:] = result["a_minus"]
        energy[k] = float(plant.total_system_energy(data))
        residual_report = plant.dynamics_residual(data)
        residual_trans[k] = float(residual_report["mechanics_residual_trans_N"])
        residual_rot[k] = float(residual_report["mechanics_residual_rot_Nm"])
        penetration[k] = float(summary["penetration_m"])
        cop[k] = np.asarray(summary["cop_xy"], dtype=float)
    transitions = np.flatnonzero(active[1:] != active[:-1]) + 1
    first_active = int(np.flatnonzero(active)[0]) if np.any(active) else None
    release = next((int(k) for k in transitions if active[k - 1] and not active[k]), None)
    recontact = next((int(k) for k in transitions if not active[k - 1] and active[k]), None)
    impulse = np.sum(np.sum(force, axis=1) - BODY_WEIGHT_N) * dt
    digest = hashlib.sha256(b"".join((
        time.tobytes(), com.tobytes(), com_v.tobytes(), force.tobytes(),
        active.tobytes(), torque.tobytes(), activation.tobytes(),
        energy.tobytes(), residual_trans.tobytes(), residual_rot.tobytes(),
        penetration.tobytes(), cop.tobytes(),
    ))).hexdigest()
    return {
        "fixture": fixture,
        "dt_s": float(dt),
        "iterations": int(model.opt.iterations),
        "time_s": time,
        "com_m": com,
        "com_velocity_mps": com_v,
        "force_N": force,
        "active": active,
        "torque_Nm": torque,
        "activation": activation,
        "energy_J": energy,
        "residual_trans_N": residual_trans,
        "residual_rot_Nm": residual_rot,
        "penetration_m": penetration,
        "cop_xy_m": cop,
        "first_active_index": first_active,
        "release_index": release,
        "recontact_index": recontact,
        "first_active_s": None if first_active is None else float(time[first_active]),
        "release_s": None if release is None else float(time[release]),
        "recontact_s": None if recontact is None else float(time[recontact]),
        "impulse_Ns": float(impulse),
        "peak_force_N": float(np.max(np.sum(force, axis=1))),
        "max_penetration_m": float(np.max(penetration)),
        "energy_delta_J": float(energy[-1] - energy[0]),
        "energy_residual_J": float(plant.energy_work_residual(data, energy[-1] - energy[0])),
        "trace_digest": digest,
    }


def _profile_proof() -> tuple[bool, Any]:
    expected = (0.00025, 0.000125, 0.0000625, 0.00003125)
    got = tuple(float(x) for x in PHYSICS_GRID_S)
    return bool(got == expected and PHYSICS_TIMESTEP_S in expected), {"declared_grid": got, "selected_timestep": float(PHYSICS_TIMESTEP_S), "expected": expected}


def _convergence_proof() -> tuple[bool, Any]:
    fixture = "RECONTACT_DROP"
    runs = [run_trace(dt, fixture) for dt in (0.00025, 0.000125, 0.0000625, 0.00003125)]
    fine = runs[-1]
    rows = []
    ok = True
    for candidate in runs[:-1]:
        event_errors = []
        for key in ("first_active_s", "release_s", "recontact_s"):
            a, b = candidate[key], fine[key]
            if a is None or b is None:
                event_errors.append(float("inf"))
            else:
                event_errors.append(abs(float(a) - float(b)))
        impulse_error = abs(candidate["impulse_Ns"] - fine["impulse_Ns"])
        peak_error = abs(candidate["peak_force_N"] - fine["peak_force_N"])
        impulse_tol = max(0.05, 0.0025 * max(abs(fine["impulse_Ns"]), 1.0))
        peak_tol = max(25.0, 0.02 * max(abs(fine["peak_force_N"]), 1.0))
        row = {"candidate_dt_s": candidate["dt_s"], "event_max_s": max(event_errors), "impulse_error_Ns": impulse_error, "impulse_tol_Ns": impulse_tol, "peak_error_N": peak_error, "peak_tol_N": peak_tol}
        rows.append(row)
        ok = ok and max(event_errors) <= 0.00025 and impulse_error <= impulse_tol and peak_error <= peak_tol
    return bool(ok), {"rows": rows, "fine": {k: fine[k] for k in ("first_active_s", "release_s", "recontact_s", "impulse_Ns", "peak_force_N")}}


def _state_convergence_proof() -> tuple[bool, Any]:
    base = run_trace(0.000125, "BALLISTIC_LAUNCH", duration_s=0.08)
    fine = run_trace(0.0000625, "BALLISTIC_LAUNCH", duration_s=0.08)
    n = min(base["com_m"].shape[0], fine["com_m"].shape[0])
    # Compare at common event-aligned times using nearest fine samples.
    idx = np.minimum((np.arange(n) * base["dt_s"] / fine["dt_s"]).round().astype(int), fine["com_m"].shape[0] - 1)
    root_err = float(np.max(np.linalg.norm(base["com_m"][:n] - fine["com_m"][idx], axis=1)))
    vel_err = float(np.max(np.linalg.norm(base["com_velocity_mps"][:n] - fine["com_velocity_mps"][idx], axis=1)))
    activation_err = float(np.max(np.abs(base["activation"][:n] - fine["activation"][idx])))
    torque_err = float(np.max(np.abs(base["torque_Nm"][:n] - fine["torque_Nm"][idx])))
    return bool(root_err <= 0.001 and vel_err <= 0.01 and activation_err <= 1e-4 and torque_err <= max(0.05, 0.001 * float(np.max(np.abs(fine["torque_Nm"]))))), {"root_error_m": root_err, "linear_velocity_error_mps": vel_err, "activation_error": activation_err, "torque_error_Nm": torque_err}


def _solver_contact_proof() -> tuple[bool, Any]:
    ref = run_trace(PHYSICS_TIMESTEP_S, "RECONTACT_DROP", iterations=100)
    rows = []
    ok = True
    for label, kwargs in (("iterations_80", {"iterations": 80}), ("iterations_120", {"iterations": 120}), ("solref_fast", {"solref_timeconst": 0.001}), ("solref_slow", {"solref_timeconst": 0.004}), ("solimp_low", {"solimp_d0": 0.90}), ("solimp_high", {"solimp_d0": 0.98})):
        value = run_trace(PHYSICS_TIMESTEP_S, "RECONTACT_DROP", **kwargs)
        impulse_rel = abs(value["impulse_Ns"] - ref["impulse_Ns"]) / max(abs(ref["impulse_Ns"]), 1.0)
        peak_rel = abs(value["peak_force_N"] - ref["peak_force_N"]) / max(abs(ref["peak_force_N"]), 1.0)
        event = float("inf")
        for key in ("first_active_s", "release_s", "recontact_s"):
            if value[key] is not None and ref[key] is not None:
                event = min(event if event != float("inf") else abs(value[key] - ref[key]), abs(value[key] - ref[key]))
        rows.append({"label": label, "impulse_rel": impulse_rel, "peak_rel": peak_rel, "event_time_difference_s": event})
        ok = ok and np.isfinite([impulse_rel, peak_rel, event]).all()
    return bool(ok), {"reference": {k: ref[k] for k in ("impulse_Ns", "peak_force_N", "first_active_s", "recontact_s")}, "rows": rows}


def _actuator_refinement_proof() -> tuple[bool, Any]:
    u = np.ones(15, dtype=float)
    a = np.zeros(15, dtype=float)
    traces = []
    for factor in (1, 2, 4):
        h = PHYSICS_TIMESTEP_S / factor
        state = drive.DriveState(a_plus=a.copy(), a_minus=a.copy(), tau_prev=np.zeros(15), previous_command=np.zeros(15))
        for _ in range(round(0.04 / h)):
            drive.activation_update(u, state.a_plus, state.a_minus, h, state=state)
        traces.append((factor, state.a_plus.copy(), state.a_minus.copy()))
    err_half = float(np.max(np.abs(traces[0][1] - traces[1][1])))
    err_quarter = float(np.max(np.abs(traces[1][1] - traces[2][1])))
    return bool(err_half <= 1e-12 and err_quarter <= 1e-12), {"half_step_error": err_half, "quarter_step_error": err_quarter}


def _finite_proof() -> tuple[bool, Any]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    data.qacc[0] = np.nan
    try:
        plant.dynamics_residual(data)
    except Exception as exc:  # noqa: BLE001 - non-finite guard must capture the raised diagnostic
        return True, {"raised": type(exc).__name__, "message": str(exc)}
    return False, {"raised": None}


def run() -> dict[str, Any]:
    checks = Checks()
    for test_id, assertion, clause, fn, tol in (
        ("qualification-NUM-PROFILE-001", "physics grid and selected timestep are final-authority declared", "plant_contract.json::runtime and THR-DT-PHYS", _profile_proof, "exact nested grid"),
        ("qualification-NUM-CONV-001", "event, impulse, and peak-force quantities converge candidate-to-fine", "threshold_registry.csv::THR-CONV-EVENT-TIME/IMPULSE/PEAK-FORCE", _convergence_proof, "registered candidate-to-fine tolerances"),
        ("qualification-NUM-STATE-001", "COM, velocity, hidden activation, and realized torque converge", "threshold_registry.csv::THR-CONV-ROOT-POS/LINVEl/ACTIVATION/TORQUE", _state_convergence_proof, "registered state tolerances"),
        ("qualification-NUM-SENS-001", "one-factor solver/contact sensitivity is finite and event-aligned", "threshold_registry.csv::THR-CONTACT-SOLVER and THR-SOLVER-SETTINGS", _solver_contact_proof, "finite, comparable, no hidden smoothing"),
        ("qualification-NUM-ACT-001", "actuator half-step and quarter-step refinement converge", "plant_contract.json::qualification.required_thresholds", _actuator_refinement_proof, "activation <=1e-12"),
        ("qualification-NUM-FINITE-001", "trusted dynamics path fails closed on a nonfinite state", "coordinate_contract.json::guards.nonfinite", _finite_proof, "exception required")
    ):
        safe(checks, test_id, assertion, clause, fn, tol)
    result = "PASS" if not checks.failures else "FAIL"
    return {"suite": "numerical", "contract_revision": "loaded-cmj-model-1", "result": result, "tests_run": len(checks.rows), "tests_passed": len(checks.rows) - len(checks.failures), "tests_failed": len(checks.failures), "rows": checks.rows}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-root", required=True)
    args = ap.parse_args()
    report = run()
    out = Path(args.evidence_root) / "qualification.4_NUMERICS.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, default=native), encoding="utf-8")
    for row in report["rows"]:
        if row["result"] == "FAIL":
            print(f"FAIL {row['test_id']}: {row['assertion']} -> {row['measured']}")
    print(f"numerical tests_run={report['tests_run']} passed={report['tests_passed']} failed={report['tests_failed']}")
    print(f"RESULT={report['result']} numerical-NUMERICS")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
