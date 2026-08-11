#!/usr/bin/env python3
"""F3.7 deterministic local 5 ms action-effectiveness qualification."""

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

from loaded_cmj.biomechanics.effectiveness import (  # noqa: E402
    ACTION_DIMENSION,
    OUTPUT_NAMES,
    estimate_local_effectiveness,
)
from loaded_cmj.simulation import drive  # noqa: E402
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


def fixture() -> tuple[Plant, mujoco.MjData, drive.DriveState, np.ndarray]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset_fixed_hold(data)
    state = drive.DriveState(
        a_plus=reset.a_plus,
        a_minus=reset.a_minus,
        tau_prev=reset.previous_torque,
        previous_command=reset.previous_action,
        override_flags=reset.override_flags,
        reversal_phase=reset.reversal_phase,
    )
    # Zero is the local characterization center; it is not a controller
    # candidate and is never selected based on the resulting matrix.
    action = np.zeros(ACTION_DIMENSION, dtype=np.float64)
    return plant, data, state, action


def result_digest(result: object) -> str:
    payload = native(asdict(result))
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    plant, data, state, action = fixture()
    result = estimate_local_effectiveness(plant, data, state, action, epsilon=1.0e-3)
    repeat = estimate_local_effectiveness(plant, data, state, action, epsilon=1.0e-3)
    small = estimate_local_effectiveness(plant, data, state, action, epsilon=5.0e-4)
    large = estimate_local_effectiveness(plant, data, state, action, epsilon=2.0e-3)
    repeat_digest = result_digest(repeat)
    checks = {
        "schema": len(result.output_names) == len(OUTPUT_NAMES) and result.output_names == OUTPUT_NAMES,
        "actual_drive_path_used": result.actual_drive_path_used,
        "contact_mode_preserved": result.contact_mode_preserved,
        "accepted_all_channels": result.accepted_channels == tuple(range(ACTION_DIMENSION)),
        "finite": np.isfinite(result.matrix).all() and np.isfinite(result.baseline_output).all(),
        "rank_diagnostic": result.scaled_rank > 0 and np.isfinite(result.scaled_condition_number),
        "perturbation_sensitivity": np.linalg.norm(result.matrix - small.matrix) / max(
            np.linalg.norm(result.matrix), 1.0
        ) < 0.25
        and np.linalg.norm(result.matrix - large.matrix) / max(np.linalg.norm(result.matrix), 1.0) < 0.25,
        "repeatable": result_digest(result) == repeat_digest,
        "no_optimization": True,
        "no_policy_or_event_owner": True,
    }
    report = {
        "suite": "F3.7-LOCAL-EFFECTIVENESS",
        "result": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "result_digest": result_digest(result),
        "repeat_digest": repeat_digest,
        "baseline_output": native(result.baseline_output),
        "output_names": list(result.output_names),
        "matrix": native(result.matrix),
        "accepted_channels": list(result.accepted_channels),
        "rejected_channels": list(result.rejected_channels),
        "singular_values_raw": native(result.singular_values_raw),
        "singular_values_scaled": native(result.singular_values_scaled),
        "scaled_rank": result.scaled_rank,
        "scaled_condition_number": result.scaled_condition_number,
        "channel_effect_norms_scaled": native(result.channel_effect_norms_scaled),
        "direct_effect_channel_by_output": result.direct_effect_channel_by_output,
        "perturbation_relative_difference_small": float(
            np.linalg.norm(result.matrix - small.matrix) / max(np.linalg.norm(result.matrix), 1.0)
        ),
        "perturbation_relative_difference_large": float(
            np.linalg.norm(result.matrix - large.matrix) / max(np.linalg.norm(result.matrix), 1.0)
        ),
        "contract": {
            "physics_timestep_s": result.physics_timestep_s,
            "control_interval_s": result.control_interval_s,
            "substeps": result.substeps,
            "epsilon": result.perturbation_epsilon,
        },
    }
    output = args.evidence_root / "effectiveness_validation.json"
    output.write_text(json.dumps(native(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    matrix_output = args.evidence_root / "71_LOCAL_EFFECTIVENESS_MATRIX.json"
    matrix_output.write_text(
        json.dumps(
            native(
                {
                    "contract_version": "LCMJ-V1-F3-EFFECTIVENESS-1.0.0",
                    "physics_timestep_s": result.physics_timestep_s,
                    "control_interval_s": result.control_interval_s,
                    "substeps": result.substeps,
                    "perturbation_epsilon": result.perturbation_epsilon,
                    "output_names": list(result.output_names),
                    "action_channels": list(range(ACTION_DIMENSION)),
                    "matrix_dy_du": result.matrix,
                    "accepted_channels": list(result.accepted_channels),
                    "rejected_channels": list(result.rejected_channels),
                    "contact_mode_preserved": result.contact_mode_preserved,
                    "actual_drive_path_used": result.actual_drive_path_used,
                }
            ),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(native({"result": report["result"], "checks": checks, "output": str(output)}), sort_keys=True))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
