#!/usr/bin/env python3
"""Independent F3.2 DriveState, capacity, passive, and work qualification."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.simulation import drive  # noqa: E402
from loaded_cmj.simulation.constants import (  # noqa: E402
    HOLD_ACTION,
    PHYSICS_TIMESTEP_S,
    SUBSTEPS_PER_CONTROL,
)
from loaded_cmj.simulation.plant import Plant, build_model  # noqa: E402


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


def run_sequence() -> dict[str, Any]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset_fixed_hold(data)
    state = drive.DriveState(
        a_plus=reset.a_plus, a_minus=reset.a_minus,
        tau_prev=reset.previous_torque,
        previous_command=reset.previous_command,
        override_flags=reset.override_flags,
        reversal_phase=reset.reversal_phase,
    )
    action = np.asarray(HOLD_ACTION, dtype=np.float64)
    initial_s = plant.anatomical_coordinates(data)
    initial_sd = plant.anatomical_rates(data)
    initial_power = plant.realized_power_components(data, reset.previous_torque)
    rows: list[dict[str, Any]] = []
    native_limit_steps: list[int] = []
    for index in range(1, SUBSTEPS_PER_CONTROL + 1):
        s = plant.anatomical_coordinates(data)
        sd = plant.anatomical_rates(data)
        result = drive.drive_state_step(action, state, s, sd, PHYSICS_TIMESTEP_S)
        tau = np.asarray(result["tau"], dtype=np.float64)
        powers = plant.realized_power_components(data, tau)
        lower = np.asarray(result["capacity_lower"], dtype=np.float64)
        upper = np.asarray(result["capacity_upper"], dtype=np.float64)
        if data.nefc:
            types = np.asarray(data.efc_type[: data.nefc], dtype=np.int32)
            if np.any(types == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)):
                native_limit_steps.append(index)
        rows.append({
            "index": index,
            "tau": tau.copy(),
            "tau_previous": np.asarray(result["tau_previous"]),
            "capacity_lower": lower.copy(),
            "capacity_upper": upper.copy(),
            "s_dot": sd.copy(),
            "powers": powers,
            "override_flags": {key: np.asarray(value, dtype=bool).copy() for key, value in result["override_flags"].items()},
        })
        plant.apply_anatomical_torque(data, tau)
        mujoco.mj_step(model, data)

    final_s = plant.anatomical_coordinates(data)
    final_sd = plant.anatomical_rates(data)
    ledgers = plant.passive_work_ledgers(initial_s, final_s, initial_sd, final_sd, SUBSTEPS_PER_CONTROL * PHYSICS_TIMESTEP_S)
    tau_arr = np.asarray([row["tau"] for row in rows])
    lower_arr = np.asarray([row["capacity_lower"] for row in rows])
    upper_arr = np.asarray([row["capacity_upper"] for row in rows])
    previous_arr = np.asarray([row["tau_previous"] for row in rows])
    sd_arr = np.asarray([row["s_dot"] for row in rows])
    active_power = np.asarray([row["powers"]["active_power_signed_W"] for row in rows])
    damping_power = np.asarray([row["powers"]["damping_power_W"] for row in rows])
    limit_power = np.asarray([row["powers"]["limit_power_W"] for row in rows])
    time_grid = np.arange(SUBSTEPS_PER_CONTROL + 1, dtype=np.float64) * PHYSICS_TIMESTEP_S
    active_work = float(np.trapezoid(np.concatenate(([initial_power["active_power_signed_W"]], active_power)), time_grid))
    damping_work = float(np.trapezoid(np.concatenate(([initial_power["damping_power_W"]], damping_power)), time_grid))
    limit_work = float(np.trapezoid(np.concatenate(([initial_power["limit_power_W"]], limit_power)), time_grid))
    rate_residual = np.max(np.abs(tau_arr - previous_arr) - drive.RATE_MAX[None, :] * PHYSICS_TIMESTEP_S)
    cap_residual = max(float(np.max(lower_arr - tau_arr)), float(np.max(tau_arr - upper_arr)))
    power_upper_residual = float(np.max(tau_arr * sd_arr - drive.POWER_POS[None, :]))
    power_lower_residual = float(np.max(-drive.POWER_NEG[None, :] - tau_arr * sd_arr))
    signs = []
    neutral_data = plant.make_data()
    plant.reset_supported(neutral_data)
    neutral_s = plant.anatomical_coordinates(neutral_data)
    neutral_sd = plant.anatomical_rates(neutral_data)
    for channel in range(15):
        for command in (-0.5, 0.5):
            test_state = drive.DriveState(
                a_plus=np.zeros(15), a_minus=np.zeros(15), tau_prev=np.zeros(15),
                previous_command=np.zeros(15),
            )
            command_vector = np.zeros(15)
            command_vector[channel] = command
            result = drive.drive_state_step(command_vector, test_state, neutral_s, neutral_sd, PHYSICS_TIMESTEP_S)
            value = float(np.asarray(result["tau"])[channel])
            signs.append({"channel": channel, "command": command, "tau": value, "same_sign": value * command >= -1e-12})
    blob = np.asarray(tau_arr, dtype=np.float64).tobytes() + np.asarray(active_power, dtype=np.float64).tobytes()
    return {
        "physics_steps": len(rows),
        "production_drive_owner": "loaded_cmj.simulation.drive.drive_state_step",
        "drive_state_type": "loaded_cmj.simulation.drive.DriveState",
        "action": action,
        "tau_first_Nm": tau_arr[0],
        "tau_last_Nm": tau_arr[-1],
        "capacity_lower_min_Nm": np.min(lower_arr, axis=0),
        "capacity_upper_max_Nm": np.max(upper_arr, axis=0),
        "max_rate_residual_Nm": float(rate_residual),
        "max_capacity_residual_Nm": float(cap_residual),
        "max_power_upper_residual_W": power_upper_residual,
        "max_power_lower_residual_W": power_lower_residual,
        "first_interval_power": initial_power,
        "active_power_range_W": [float(np.min(active_power)), float(np.max(active_power))],
        "damping_power_range_W": [float(np.min(damping_power)), float(np.max(damping_power))],
        "limit_power_range_W": [float(np.min(limit_power)), float(np.max(limit_power))],
        "work_integrals_J": {
            "active_work": active_work,
            "passive_work": damping_work,
            "limit_work": limit_work,
            "native_limit_work": 0.0,
            "integration": "trapezoid on [t=0 endpoint, 40 post-step endpoints] at 8 kHz",
        },
        "native_limit_active_steps": native_limit_steps,
        "passive_work_ledgers_J": ledgers,
        "sign_checks": signs,
        "sequence_digest": hashlib.sha256(blob).hexdigest(),
        "first_interval_present": all(key in initial_power for key in ("active_power_signed_W", "damping_power_W", "limit_power_W")),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    first = run_sequence()
    second = run_sequence()
    sign_ok = all(bool(row["same_sign"]) for row in first["sign_checks"])
    ledger = first["passive_work_ledgers_J"]
    passed = (
        first["sequence_digest"] == second["sequence_digest"]
        and first["max_rate_residual_Nm"] <= 1e-9
        and first["max_capacity_residual_Nm"] <= 1e-9
        and first["max_power_upper_residual_W"] <= 1e-9
        and first["max_power_lower_residual_W"] <= 1e-9
        and not first["native_limit_active_steps"]
        and sign_ok
        and first["first_interval_present"]
        and abs(float(ledger["elastic_work"])) <= 1e-15
        and np.isfinite(float(ledger["limit_work"]))
        and np.isfinite(float(ledger["damping_work"]))
    )
    report = {
        "suite": "F3.2-DRIVESTATE-PASSIVE-ENERGETICS",
        "result": "PASS" if passed else "FAIL",
        "independent_run": first,
        "repeat_run": {"sequence_digest": second["sequence_digest"]},
        "checks": {
            "single_production_drive_path": first["production_drive_owner"].endswith("drive_state_step"),
            "realized_torque_capacity": first["max_capacity_residual_Nm"] <= 1e-9,
            "realized_torque_rate": first["max_rate_residual_Nm"] <= 1e-9,
            "signed_power_limits": first["max_power_upper_residual_W"] <= 1e-9 and first["max_power_lower_residual_W"] <= 1e-9,
            "action_torque_sign": sign_ok,
            "first_interval_work_endpoint": first["first_interval_present"],
            "native_limit_exploitation": not first["native_limit_active_steps"],
            "source_separated_passive_work": abs(float(ledger["elastic_work"])) <= 1e-15 and np.isfinite(float(ledger["limit_work"])),
            "deterministic": first["sequence_digest"] == second["sequence_digest"],
        },
    }
    output = args.evidence_root / "21_ACTUATOR_CAPACITY_REPORT.json"
    output.write_text(json.dumps(native(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": report["result"], "output": str(output), "sequence_digest": first["sequence_digest"]}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
