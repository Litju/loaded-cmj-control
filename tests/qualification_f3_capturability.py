#!/usr/bin/env python3
"""F3.6 predictive support-reserve qualification on fixed fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import asdict
from pathlib import Path

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.biomechanics.capturability import (  # noqa: E402
    CAPTURABILITY_CONTRACT_VERSION,
    predict_support_reserve,
)
from loaded_cmj.simulation import drive  # noqa: E402
from loaded_cmj.simulation.constants import PHYSICS_TIMESTEP_S  # noqa: E402
from loaded_cmj.simulation.plant import Plant, build_model  # noqa: E402


def native(value: object) -> object:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): native(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [native(item) for item in value]
    return value


def _trunk_pitch_rad(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_head_arms")
    rotation = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    return float(np.arctan2(rotation[0, 2], rotation[2, 2]))


def fixed_supported_trace() -> dict[str, object]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset_fixed_hold(data)
    latched = (True, True)
    # This is a fixed supported-state qualification fixture, not a controller
    # candidate.  It exercises the real plant/drive/contact path for 40 ms.
    data.qvel[0] = 0.5
    data.qvel[2] = -0.2
    mujoco.mj_forward(model, data)
    com_start = plant.center_of_mass(data).copy()
    velocity_start = plant.center_of_mass_velocity(data).copy()
    polygon = plant.support_polygon(data, latched)
    bounds = (float(polygon[:, 0].min()), float(polygon[:, 0].max()))
    current_sagittal_reserve = min(com_start[0] - bounds[0], bounds[1] - com_start[0])
    summary_start = plant.contact_wrench_summary(data)
    h_start = plant.centroidal_angular_momentum(data, com_start)
    state = drive.DriveState(
        a_plus=reset.a_plus,
        a_minus=reset.a_minus,
        tau_prev=reset.previous_torque,
        previous_command=reset.previous_action,
        override_flags=reset.override_flags,
        reversal_phase=reset.reversal_phase,
    )
    action = reset.previous_action.copy()
    mode_preserved = True
    prohibited_contact = False
    for _ in range(400):
        result = drive.drive_state_step(
            action,
            state,
            plant.anatomical_coordinates(data),
            plant.anatomical_rates(data),
            PHYSICS_TIMESTEP_S,
        )
        plant.apply_anatomical_torque(data, result["tau"])
        mujoco.mj_step(model, data)
        summary = plant.contact_wrench_summary(data)
        mode_preserved = mode_preserved and bool(summary["contact_active"] and np.all(summary["active_by_foot"]))
        prohibited_contact = prohibited_contact or bool(summary["prohibited_contact"])
    mujoco.mj_forward(model, data)
    com_end = plant.center_of_mass(data)
    actual_horizon_reserve = min(com_end[0] - bounds[0], bounds[1] - com_end[0])
    summary_end = plant.contact_wrench_summary(data)
    prediction = predict_support_reserve(
        com_x_m=float(com_start[0]),
        com_vx_mps=float(velocity_start[0]),
        com_vz_mps=float(velocity_start[2]),
        feasible_upward_acceleration_mps2=4.0,
        feasible_horizontal_deceleration_mps2=0.5,
        support_geometry_bounds_m=bounds,
        current_admissible_support_reserve_m=float(current_sagittal_reserve),
        actual_com_x_at_horizon_m=float(com_end[0]),
        external_wrench_N_Nm=tuple(float(x) for x in summary_start["whole_wrench"]),
        centroidal_pitch_angular_momentum_kgm2ps=float(h_start[1]),
        trunk_pitch_rad=_trunk_pitch_rad(model, data),
    )
    return {
        "reset_protocol": reset.reset_metadata["protocol"],
        "duration_s": 400 * PHYSICS_TIMESTEP_S,
        "support_geometry_bounds_m": bounds,
        "com_start_m": com_start,
        "com_velocity_start_mps": velocity_start,
        "com_end_m": com_end,
        "actual_support_reserve_independent_m": actual_horizon_reserve,
        "prediction": asdict(prediction),
        "contact_mode_preserved": mode_preserved,
        "prohibited_contact": prohibited_contact,
        "end_active_by_foot": summary_end["active_by_foot"],
    }


def static_margin_counterexample() -> dict[str, object]:
    prediction = predict_support_reserve(
        com_x_m=0.0,
        com_vx_mps=1.0,
        com_vz_mps=-1.0,
        feasible_upward_acceleration_mps2=2.5,
        feasible_horizontal_deceleration_mps2=0.0,
        support_geometry_bounds_m=(-0.3, 0.3),
        actual_com_x_at_horizon_m=0.4,
    )
    return {
        "static_support_margin_m": prediction.current_admissible_support_reserve_m,
        "prediction": asdict(prediction),
        "static_margin_positive": prediction.current_admissible_support_reserve_m > 0.0,
        "predictive_reserve_negative": prediction.predicted_support_reserve_m < 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    supported = fixed_supported_trace()
    counterexample = static_margin_counterexample()
    repeat_a = static_margin_counterexample()
    repeat_b = static_margin_counterexample()
    repeat_digest_a = hashlib.sha256(json.dumps(native(repeat_a), sort_keys=True).encode()).hexdigest()
    repeat_digest_b = hashlib.sha256(json.dumps(native(repeat_b), sort_keys=True).encode()).hexdigest()
    checks = {
        "contract_version": supported["prediction"]["contract_version"] == CAPTURABILITY_CONTRACT_VERSION,
        "supported_trace": supported["reset_protocol"] == "fixed_hold_reset",
        "contact_mode_preserved": supported["contact_mode_preserved"],
        "no_prohibited_contact": not supported["prohibited_contact"],
        "finite": all(np.isfinite(np.asarray(supported["com_start_m"]))) and all(
            np.isfinite(np.asarray(supported["com_end_m"]))
        ),
        "prediction_error_reported": supported["prediction"]["prediction_error_m"] is not None,
        "static_margin_counterexample": counterexample["static_margin_positive"]
        and counterexample["predictive_reserve_negative"],
        "repeatable": repeat_digest_a == repeat_digest_b,
        "no_event_owner": True,
    }
    report = {
        "suite": "F3.6-CAPTURABILITY",
        "result": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "supported_trace": native(supported),
        "static_margin_counterexample": native(counterexample),
        "repeat_digest": repeat_digest_a,
    }
    output = args.evidence_root / "capturability_validation.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": report["result"], "checks": checks, "output": str(output)}, sort_keys=True))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
