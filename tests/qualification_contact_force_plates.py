#!/usr/bin/env python3
"""Frozen contact live-contact, wrench, COP, and support-latch proofs."""

from __future__ import annotations

import argparse
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
         fn: Callable[[], tuple[bool, Any]], tolerance: Any) -> None:
    try:
        ok, measured = fn()
    except Exception as exc:  # noqa: BLE001 - contract harness records every failed proof
        ok, measured = False, f"{type(exc).__name__}: {exc}"
    checks.add(test_id, assertion, ok, measured, tolerance, clause)


def _sign_fixture() -> tuple[bool, Any]:
    from loaded_cmj.simulation.plant import contact_wrench_from_raw

    frame = np.eye(3, dtype=float)
    raw = np.asarray([3.0, -2.0, 11.0, 0.4, -0.3, 0.2], dtype=float)
    point = np.asarray([0.2, -0.1, 0.0], dtype=float)
    origin = np.asarray([0.0, 0.0, 0.0], dtype=float)
    geom1 = contact_wrench_from_raw(frame, raw, system_geom_index=1, contact_point=point, plate_origin=origin)
    geom0 = contact_wrench_from_raw(frame, -raw, system_geom_index=0, contact_point=point, plate_origin=origin)
    return bool(np.array_equal(geom1, geom0) and np.array_equal(geom1[:3], raw[:3])), {"geom1": geom1, "geom0_equal_and_opposite_fixture": geom0, "residual": float(np.abs(geom1 - geom0).max())}


def _partition_proof() -> tuple[bool, Any]:
    model = build_model()
    plant = Plant(model)
    points = np.asarray([
        [0.0, 0.0, 0.0],
        [-0.29, 0.10, 0.0],
        [0.29, -0.10, 0.0],
        [0.0, 0.30, 0.0],
        [0.0, -0.30, 0.0],
        [1.0, 1.0, 0.0],
    ])
    labels = [plant.classify_contact_point(p) for p in points]
    return bool(labels[0] == "left_plate" and labels[1] == "left_plate" and labels[2] == "right_plate" and labels[3] == "off_plate" and labels[4] == "off_plate" and labels[5] == "off_plate" and set(labels) <= {"left_plate", "right_plate", "off_plate"}), {"points": points, "labels": labels}


def _summary_proof() -> tuple[bool, Any]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    summary = plant.contact_wrench_summary(data)
    required = {"contacts", "whole_wrench", "plate_wrench", "normal_force", "cop_xy", "cop_valid", "contact_active", "penetration_m", "slip_speed_mps"}
    present = set(summary)
    finite = all(
        np.isfinite(np.asarray(value, dtype=float)).all()
        for key, value in summary.items()
        if key not in {"contacts", "contact_region", "cop_frame"}
    )
    normal = np.asarray(summary["normal_force"], dtype=float)
    return bool(required <= present and finite and np.all(normal >= -1e-8) and np.asarray(summary["whole_wrench"]).shape == (6,) and np.asarray(summary["plate_wrench"]).shape == (3, 6)), {"keys": sorted(present), "normal_force": normal, "whole_wrench": summary["whole_wrench"], "plate_wrench": summary["plate_wrench"], "contact_count": len(summary["contacts"])}


def _support_proof() -> tuple[bool, Any]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset_fixed_hold(data)
    summary = plant.contact_wrench_summary(data)
    fz = np.asarray(summary["normal_force"], dtype=float)
    cop_valid = np.asarray(summary["cop_valid"], dtype=bool)
    return bool(
        np.all(fz > 0.0)
        and bool(cop_valid.all())
        and np.all(np.isfinite(np.asarray(summary["cop_xy"], dtype=float)))
        and bool(reset.contact_cop_valid.all())
    ), {"normal_force": fz, "cop_xy": summary["cop_xy"], "cop_valid": cop_valid, "reset_metadata": reset.reset_metadata}


def _latch_proof() -> tuple[bool, Any]:
    model = build_model()
    plant = Plant(model)
    latch = np.zeros(2, dtype=bool)
    on_steps = np.zeros(2, dtype=np.int64)
    off_steps = np.zeros(2, dtype=np.int64)
    transitions = []
    for step in range(2):
        result = plant.update_support_latch(latch, on_steps, off_steps, np.asarray([100.0, 100.0]), PHYSICS_TIMESTEP_S)
        latch, on_steps, off_steps = result["latch"], result["on_steps"], result["off_steps"]
        transitions.extend(result["transitions"])
    latched = latch.copy()
    for step in range(4):
        result = plant.update_support_latch(latch, on_steps, off_steps, np.asarray([0.0, 0.0]), PHYSICS_TIMESTEP_S)
        latch, on_steps, off_steps = result["latch"], result["on_steps"], result["off_steps"]
        transitions.extend(result["transitions"])
    return bool(np.array_equal(latched, [True, True]) and not latch.any() and len(transitions) == 4), {"latched_after_on": latched, "latch_after_off": latch, "transitions": transitions}


def _ballistic_contact_proof() -> tuple[bool, Any]:
    model = build_model()
    model.opt.timestep = PHYSICS_TIMESTEP_S
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    data.qpos[2] += 0.25
    data.qvel[:] = 0.0
    data.qvel[2] = 2.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    active_samples = 0
    no_contact_samples = 0
    recontact_samples = 0
    previous_active = False
    peak = 0.0
    for _ in range(round(0.9 / PHYSICS_TIMESTEP_S)):
        mujoco.mj_step(model, data)
        summary = plant.contact_wrench_summary(data)
        active = bool(summary["contact_active"])
        peak = max(peak, float(np.asarray(summary["normal_force"]).sum()))
        active_samples += int(active)
        no_contact_samples += int(not active)
        if previous_active is False and active and no_contact_samples > 0:
            recontact_samples += 1
        previous_active = active
    return bool(active_samples > 0 and no_contact_samples > 0 and recontact_samples >= 1 and np.isfinite(peak)), {"active_samples": active_samples, "contact_free_samples": no_contact_samples, "recontact_transitions": recontact_samples, "peak_normal_force_N": peak}


def run() -> dict[str, Any]:
    checks = Checks()
    for test_id, assertion, clause, fn, tol in (
        ("qualification-CON-SIGN-001", "both MuJoCo geom orders produce the same environment-on-system wrench", "13_GROUND_CONTACT_AND_FORCE_PLATE_MODEL.md::per-contact environment-on-system wrench", _sign_fixture, "exact sign conversion"),
        ("qualification-CON-PART-001", "projected physical contacts partition exactly into left/right/off-plate", "force_plate_contract.json::virtual_partition", _partition_proof, "one assignment per point"),
        ("qualification-CON-WRENCH-001", "live addressable contact summary exposes force, wrench, COP, penetration, and slip", "force_plate_contract.json::wrench and cop", _summary_proof, "finite complete diagnostic"),
        ("qualification-CON-SUPPORT-001", "settled bilateral support has positive force and valid COP", "14_ACTIVE_CONTACT_SUPPORT_AND_COP_PREDICATES.md::support and COP", _support_proof, "both feet active; COP valid"),
        ("qualification-CON-LATCH-001", "support latch uses hysteresis/debounce and releases without chatter", "contact_predicate_contract.json::support_contact_on_latch/off_latch", _latch_proof, "2-on/4-off fixture"),
        ("qualification-CON-FLIGHT-001", "ballistic fixture contains a contact-free interval and descending recontact", "21_FLIGHT_VALIDITY_MODEL.md::genuine flight predicates", _ballistic_contact_proof, "finite release and recontact")
    ):
        safe(checks, test_id, assertion, clause, fn, tol)
    result = "PASS" if not checks.failures else "FAIL"
    return {"suite": "contact", "contract_revision": "loaded-cmj-model-1", "result": result, "tests_run": len(checks.rows), "tests_passed": len(checks.rows) - len(checks.failures), "tests_failed": len(checks.failures), "rows": checks.rows}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-root", required=True)
    args = ap.parse_args()
    report = run()
    out = Path(args.evidence_root) / "qualification.3_CONTACT.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, default=native), encoding="utf-8")
    for row in report["rows"]:
        if row["result"] == "FAIL":
            print(f"FAIL {row['test_id']}: {row['assertion']} -> {row['measured']}")
    print(f"contact tests_run={report['tests_run']} passed={report['tests_passed']} failed={report['tests_failed']}")
    print(f"RESULT={report['result']} contact-CONTACT")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
