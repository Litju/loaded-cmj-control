#!/usr/bin/env python3
"""Frozen actuator contract proofs for the final-authority actuator model.

This file is deliberately independent of score calculation and controller code. It is
also used as the shared numerical fixture helper by the other qualification proofs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.simulation import drive as drive
from loaded_cmj.simulation.constants import ACTION_DIM, PHYSICS_TIMESTEP_S


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


def digest_arrays(*arrays: np.ndarray) -> str:
    h = hashlib.sha256()
    for value in arrays:
        arr = np.asarray(value)
        h.update(str(arr.dtype).encode())
        h.update(np.asarray(arr.shape, dtype=np.int64).tobytes())
        h.update(arr.tobytes())
    return h.hexdigest()


class Checks:
    def __init__(self) -> None:
        self.rows: list[dict[str, Any]] = []

    def add(
        self,
        test_id: str,
        assertion: str,
        ok: bool,
        measured: Any,
        tolerance: Any,
        clause: str,
        failure_class: str = "contract_check",
    ) -> None:
        self.rows.append(
            {
                "test_id": test_id,
                "assertion": assertion,
                "result": "PASS" if bool(ok) else "FAIL",
                "measured": native(measured),
                "tolerance": native(tolerance),
                "contract_clause": clause,
                "failure_class": failure_class,
            }
        )

    @property
    def failures(self) -> list[dict[str, Any]]:
        return [row for row in self.rows if row["result"] == "FAIL"]


def safe(checks: Checks, test_id: str, assertion: str, clause: str,
         fn: Callable[[], tuple[bool, Any]], tolerance: Any) -> None:
    try:
        ok, measured = fn()
    except Exception as exc:  # noqa: BLE001 - an absent pre-state contract is evidence
        ok, measured = False, f"{type(exc).__name__}: {exc}"
    checks.add(test_id, assertion, ok, measured, tolerance, clause)


def _state(*, signed: np.ndarray | None = None) -> Any:
    signed_value = np.zeros(ACTION_DIM, dtype=np.float64) if signed is None else np.asarray(signed, dtype=np.float64)
    return drive.DriveState(
        a_plus=np.maximum(signed_value, 0.0),
        a_minus=np.maximum(-signed_value, 0.0),
        tau_prev=np.zeros(ACTION_DIM, dtype=np.float64),
        previous_command=signed_value.copy(),
    )


def _activation_proof() -> tuple[bool, Any]:
    state = _state()
    command = np.zeros(ACTION_DIM, dtype=np.float64)
    command[0] = 1.0
    a_plus, a_minus = drive.activation_update(command, state.a_plus, state.a_minus, PHYSICS_TIMESTEP_S)
    tau_act = float(np.asarray(drive.TAU_ACT_POS).reshape(-1)[0])
    expected = 1.0 - math.exp(-PHYSICS_TIMESTEP_S / tau_act)
    return bool(
        np.allclose(a_plus[0], expected, rtol=0.0, atol=2e-15)
        and np.all(a_minus == 0.0)
        and np.all((a_plus >= 0.0) & (a_plus <= 1.0))
    ), {"observed": a_plus[0], "expected": expected, "a_minus_max": float(a_minus.max())}


def _reversal_proof() -> tuple[bool, Any]:
    state = _state(signed=np.ones(ACTION_DIM, dtype=np.float64))
    state.tau_prev[:] = 0.0
    s = np.zeros(ACTION_DIM, dtype=np.float64)
    sd = np.zeros(ACTION_DIM, dtype=np.float64)
    plus = np.ones(ACTION_DIM, dtype=np.float64)
    minus = -plus
    drives: list[float] = []
    crossings: list[float] = []
    torques: list[float] = []
    for _ in range(700):
        result = drive.drive_state_step(minus, state, s, sd, PHYSICS_TIMESTEP_S)
        drives.append(float(result["drive"][0]))
        torques.append(float(result["tau"][0]))
        if result.get("zero_crossing_time_s") is not None:
            crossings.append(float(result["zero_crossing_time_s"]))
    arr = np.asarray(drives)
    tau = np.asarray(torques)
    first_negative = int(np.flatnonzero(arr < 0.0)[0]) if np.any(arr < 0.0) else None
    first_tau_negative = int(np.flatnonzero(tau < 0.0)[0]) if np.any(tau < 0.0) else None
    no_jump = first_negative is None or first_negative > 0
    return bool(
        np.isfinite(arr).all()
        and np.all(np.abs(arr) <= 1.0 + 1e-14)
        and no_jump
        and bool(crossings)
        and first_tau_negative is not None
        and first_tau_negative > 0
        and np.any(np.abs(arr) <= 1e-14)
    ), {
        "first_negative_drive_sample": first_negative,
        "first_negative_torque_sample": first_tau_negative,
        "zero_crossing_times_s": crossings[:4],
        "drive_min_max": [float(arr.min()), float(arr.max())],
    }


def _velocity_proof() -> tuple[bool, Any]:
    samples = np.linspace(-8.0, 8.0, 801)
    values = np.asarray(drive.torque_velocity(samples), dtype=np.float64)
    h = 1e-6
    left = float(drive.torque_velocity(np.asarray([-h]))[0])
    right = float(drive.torque_velocity(np.asarray([h]))[0])
    dleft = float((drive.torque_velocity(np.asarray([-h]))[0] - drive.torque_velocity(np.asarray([-2.0 * h]))[0]) / h)
    dright = float((drive.torque_velocity(np.asarray([2.0 * h]))[0] - drive.torque_velocity(np.asarray([h]))[0]) / h)
    fmin = float(getattr(drive, "FV_MIN", getattr(drive, "F_MIN", 0.0)))
    fecc = float(getattr(drive, "F_ECC", getattr(drive, "F_ECC_MAX", 1.0)))
    return bool(
        np.isfinite(values).all()
        and np.all(values > 0.0)
        and float(values.max()) <= fecc + 1e-12
        and float(values[samples >= 4.0].min()) >= fmin - 1e-12
        and abs(left - right) <= 1e-10
        and abs(dleft - dright) <= 2e-5
        and float(drive.torque_velocity(np.asarray([0.0]))[0]) == 1.0
    ), {
        "min": float(values.min()),
        "max": float(values.max()),
        "f_min": fmin,
        "f_ecc": fecc,
        "zero_value": float(drive.torque_velocity(np.asarray([0.0]))[0]),
        "derivative_left": dleft,
        "derivative_right": dright,
    }


def _power_interval_proof() -> tuple[bool, Any]:
    tau = np.asarray([100.0, -100.0, 25.0, -25.0], dtype=np.float64)
    eta = np.asarray([2.0, 2.0, -2.0, -2.0], dtype=np.float64)
    ppos = np.asarray([50.0, 50.0, 50.0, 50.0], dtype=np.float64)
    pneg = np.asarray([30.0, 30.0, 30.0, 30.0], dtype=np.float64)
    got = np.asarray(drive.project_signed_power(tau, eta, ppos, pneg), dtype=np.float64)
    power = got * eta
    return bool(
        np.all(power <= ppos + 1e-12)
        and np.all(power >= -pneg - 1e-12)
        and np.allclose(got, np.asarray([25.0, -15.0, 15.0, -25.0]), rtol=0.0, atol=1e-12)
    ), {"torque": got, "power": power}


def _rate_capacity_proof() -> tuple[bool, Any]:
    state = _state()
    state.tau_prev[:] = 0.0
    command = np.ones(ACTION_DIM, dtype=np.float64)
    s = np.zeros(ACTION_DIM, dtype=np.float64)
    sd = np.linspace(-3.0, 3.0, ACTION_DIM)
    result = drive.drive_state_step(command, state, s, sd, PHYSICS_TIMESTEP_S)
    tau = np.asarray(result["tau"], dtype=np.float64)
    lower, upper = drive.capacity_envelope(s, sd)
    flags = result["override_flags"]
    rate = float(np.max(np.abs(tau - result["tau_previous"])))
    return bool(
        np.isfinite(tau).all()
        and np.all(tau >= lower - 1e-9)
        and np.all(tau <= upper + 1e-9)
        and set(flags) >= {"rate_override_by_capacity", "rate_override_by_power"}
        and np.all(np.sign(tau) >= 0.0)
    ), {"tau": tau, "lower": lower, "upper": upper, "max_increment": rate, "flags": flags}


def _reachable_proof() -> tuple[bool, Any]:
    rng = np.random.default_rng(20260806)
    z = rng.uniform(-0.5, 0.5, ACTION_DIM)
    s = rng.normal(0.0, 0.15, ACTION_DIM)
    sd = rng.normal(0.0, 1.0, ACTION_DIM)
    previous = rng.normal(0.0, 10.0, ACTION_DIM)

    def held_torque(command_value: float) -> np.ndarray:
        state = drive.DriveState(
            z=z,
            tau_prev=previous,
            previous_command=np.zeros(ACTION_DIM, dtype=np.float64),
        )
        command = np.full(ACTION_DIM, command_value, dtype=np.float64)
        result = None
        for _ in range(40):
            result = drive.drive_state_step(command, state, s, sd, PHYSICS_TIMESTEP_S)
        assert result is not None
        return np.asarray(result["tau"], dtype=np.float64)

    endpoints = np.stack([held_torque(-1.0), held_torque(1.0)])
    lo = endpoints.min(axis=0)
    hi = endpoints.max(axis=0)
    samples = np.linspace(-1.0, 1.0, 201)
    dense = np.stack([held_torque(float(u)) for u in samples])
    return bool(
        np.isfinite(lo).all()
        and np.isfinite(hi).all()
        and np.all(dense >= lo - 1e-8)
        and np.all(dense <= hi + 1e-8)
        and np.all(lo <= hi + 1e-12)
    ), {"max_below": float(np.max(lo - dense)), "max_above": float(np.max(dense - hi))}


def run() -> dict[str, Any]:
    checks = Checks()
    fields = set(getattr(drive.DriveState, "__dataclass_fields__", {}))
    required = {"a_plus", "a_minus", "tau_prev", "previous_command", "override_flags", "reversal_phase"}
    checks.add(
        "qualification-ACT-STATE-001",
        "DriveState carries separate directional activation, command, torque, override, and reversal state",
        required <= fields,
        sorted(fields),
        sorted(required),
        "actuator-drive-model.md::hidden state and command",
    )
    safe(checks, "qualification-ACT-EXP-001", "directional activation uses the exact exponential substep", "actuator_contract.json::activation.exact_substep_update", _activation_proof, "absolute 2e-15")
    safe(checks, "qualification-ACT-REV-001", "reversal retains hidden residual state and records a continuous zero crossing", "actuator-drive-model.md::reversal and frozen substep order", _reversal_proof, "finite; exact crossing recorded")
    safe(checks, "qualification-ACT-TV-001", "torque-velocity curve is bounded, positive, and C1 at zero", "11_TORQUE_VELOCITY_POWER_AND_RATE_MODEL.md::directional torque-velocity curve", _velocity_proof, "C1 derivative residual <=2e-5")
    safe(checks, "qualification-ACT-POWER-001", "signed two-sided power interval handles both velocity signs", "11_TORQUE_VELOCITY_POWER_AND_RATE_MODEL.md::exact two-sided power interval", _power_interval_proof, "signed interval")
    safe(checks, "qualification-ACT-LIMIT-001", "ordered torque pipeline records hard-cap overrides and remains finite", "actuator_contract.json::limiter_priority", _rate_capacity_proof, "finite; capacity/rate/power bounds")
    safe(checks, "qualification-ACT-REACH-001", "one-step reachable torque interval contains dense command sweep", "actuator_contract.json::rate and reachable torque qualification", _reachable_proof, "all 201 commands contained")
    result = "PASS" if not checks.failures else "FAIL"
    return {
        "suite": "actuator",
        "contract_revision": "loaded-cmj-model-1",
        "result": result,
        "tests_run": len(checks.rows),
        "tests_passed": len(checks.rows) - len(checks.failures),
        "tests_failed": len(checks.failures),
        "rows": checks.rows,
        "trace_digest": digest_arrays(np.asarray([row["result"] == "PASS" for row in checks.rows], dtype=np.uint8)),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-root", required=True)
    args = ap.parse_args()
    report = run()
    out = Path(args.evidence_root) / "qualification.1_DRIVES.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, default=native), encoding="utf-8")
    for row in report["rows"]:
        if row["result"] == "FAIL":
            print(f"FAIL {row['test_id']}: {row['assertion']} -> {row['measured']}")
    print(f"actuator tests_run={report['tests_run']} passed={report['tests_passed']} failed={report['tests_failed']}")
    print(f"RESULT={report['result']} actuator-DRIVES")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
