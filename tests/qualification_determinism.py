#!/usr/bin/env python3
"""Frozen three-instance deterministic replay proof for qualification."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_VALIDATION_ROOT = Path(__file__).resolve().parent
_TASK_ROOT = _VALIDATION_ROOT.parent
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))
if str(_VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(_VALIDATION_ROOT))

from loaded_cmj.simulation import drive as drive
from loaded_cmj.simulation.plant import Plant, build_model
from loaded_cmj.simulation.constants import PHYSICS_TIMESTEP_S


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


def replay_once() -> tuple[str, dict[str, str]]:
    model = build_model()
    model.opt.timestep = PHYSICS_TIMESTEP_S
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset(seed=0, metadata=None)
    plant.reset_supported(data)
    state = drive.DriveState(
        a_plus=np.maximum(plant.u_eq, 0.0),
        a_minus=np.maximum(-plant.u_eq, 0.0),
        tau_prev=plant.tau_eq.copy(),
        previous_command=plant.u_eq.copy(),
    )
    command = plant.u_eq.copy()
    h = hashlib.sha256()
    event = hashlib.sha256()
    metric = hashlib.sha256()
    score = hashlib.sha256()
    h.update(np.asarray(reset.qpos, dtype=float).tobytes())
    h.update(np.asarray(reset.qvel, dtype=float).tobytes())
    for step in range(400):
        if step == 80:
            command = plant.u_eq.copy()
            command[9:13] *= -0.5
        if step == 240:
            command = plant.u_eq.copy()
        result = drive.drive_state_step(command, state, plant.anatomical_coordinates(data), plant.anatomical_rates(data), PHYSICS_TIMESTEP_S)
        plant.apply_anatomical_torque(data, result["tau"])
        mujoco.mj_step(model, data)
        summary = plant.contact_wrench_summary(data)
        arrays = (
            np.asarray(data.qpos, dtype=float), np.asarray(data.qvel, dtype=float),
            np.asarray(result["a_plus"], dtype=float), np.asarray(result["a_minus"], dtype=float),
            np.asarray(result["tau"], dtype=float), np.asarray(summary["whole_wrench"], dtype=float),
        )
        for arr in arrays:
            h.update(arr.tobytes())
        event.update(np.asarray([int(bool(summary["contact_active"]))], dtype=np.uint8).tobytes())
        metric.update(np.asarray([float(np.sum(summary["normal_force"])), float(plant.total_system_energy(data))], dtype=np.float64).tobytes())
        score.update(np.asarray([float(np.max(np.abs(result["tau"]))), float(np.sum(result["override_flags"]["rate_override_by_capacity"])), float(np.sum(result["override_flags"]["rate_override_by_power"]))], dtype=np.float64).tobytes())
    return h.hexdigest(), {"trace": h.hexdigest(), "event": event.hexdigest(), "metric": metric.hexdigest(), "score": score.hexdigest()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-root", required=True)
    args = ap.parse_args()
    try:
        runs = [replay_once() for _ in range(3)]
    except Exception as exc:  # noqa: BLE001 - replay failure is itself evidence
        report = {
            "suite": "numerical",
            "test_id": "qualification-NUM-REPLAY-001",
            "contract_revision": "loaded-cmj-model-1",
            "result": "FAIL",
            "error": f"{type(exc).__name__}: {exc}",
            "projection_identical": False,
            "contract_clause": "threshold_registry.csv::THR-CONV-REPLAY",
        }
        out = Path(args.evidence_root) / "qualification.4_DETERMINISM.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, default=native), encoding="utf-8")
        print(f"RESULT=FAIL numerical-DETERMINISM ({report['error']})")
        return 1
    trace_digests = [item[0] for item in runs]
    projections = [item[1] for item in runs]
    ok = len(set(trace_digests)) == 1 and len({p["event"] for p in projections}) == 1 and len({p["metric"] for p in projections}) == 1 and len({p["score"] for p in projections}) == 1
    report = {
        "suite": "numerical",
        "test_id": "qualification-NUM-REPLAY-001",
        "contract_revision": "loaded-cmj-model-1",
        "result": "PASS" if ok else "FAIL",
        "runs": projections,
        "trace_digests": trace_digests,
        "projection_identical": bool(ok),
        "contract_clause": "threshold_registry.csv::THR-CONV-REPLAY",
    }
    out = Path(args.evidence_root) / "qualification.4_DETERMINISM.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, default=native), encoding="utf-8")
    print(f"numerical deterministic_replay={'PASS' if ok else 'FAIL'}")
    print(f"RESULT={report['result']} numerical-DETERMINISM")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
