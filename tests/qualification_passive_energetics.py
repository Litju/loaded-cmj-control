#!/usr/bin/env python3
"""Frozen passive passive-mechanics and source-separated energy proofs."""

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


def make_plant() -> tuple[Plant, mujoco.MjData]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    return plant, data


def _gradient_proof() -> tuple[bool, Any]:
    plant, _ = make_plant()
    rng = np.random.default_rng(20260806)
    worst = 0.0
    rows = []
    for scale in (0.5, 1.0, 1.1, 1.4):
        s = rng.normal(size=15) * scale
        grad = np.asarray(plant.passive_potential_gradient(s), dtype=float)
        h = 1e-6
        fd = np.empty(15)
        for i in range(15):
            plus = s.copy(); plus[i] += h
            minus = s.copy(); minus[i] -= h
            fd[i] = (float(plant.passive_potential(plus)) - float(plant.passive_potential(minus))) / (2.0 * h)
        error = float(np.max(np.abs(grad - fd)))
        worst = max(worst, error)
        rows.append({"scale": scale, "phi": float(plant.region_phi(s)), "error": error})
    return bool(worst <= 5e-5), {"worst_abs_gradient_error": worst, "rows": rows}


def _potential_onset_proof() -> tuple[bool, Any]:
    plant, _ = make_plant()
    inside = np.zeros(15, dtype=float)
    outside = np.zeros(15, dtype=float)
    outside[0] = plant.region_radius_vector()[0] * 1.25
    v0 = float(plant.passive_potential(inside))
    g0 = np.asarray(plant.passive_potential_gradient(inside))
    v1 = float(plant.passive_potential(outside))
    g1 = np.asarray(plant.passive_potential_gradient(outside))
    taper = float(plant.outward_capacity_taper(outside, 0))
    return bool(
        v0 == 0.0
        and np.array_equal(g0, np.zeros(15))
        and v1 > 0.0
        and np.isfinite(g1).all()
        and np.linalg.norm(g1) > 0.0
        and 0.0 <= taper <= 1.0
    ), {"V_inside": v0, "grad_inside_inf": float(np.abs(g0).max()), "V_outside": v1, "grad_outside_inf": float(np.abs(g1).max()), "outward_taper": taper}


def _damping_proof() -> tuple[bool, Any]:
    plant, _ = make_plant()
    rng = np.random.default_rng(31)
    worst_power = -float("inf")
    rows = []
    for _ in range(200):
        rate = rng.normal(size=15)
        tau = np.asarray(plant.passive_damping_torque(rate), dtype=float)
        power = float(tau @ rate)
        worst_power = max(worst_power, power)
        rows.append(power)
    return bool(np.isfinite(rows).all() and worst_power <= 1e-12), {"max_damping_power": worst_power}


def _work_energy_proof() -> tuple[bool, Any]:
    plant, _ = make_plant()
    rng = np.random.default_rng(91)
    s0 = rng.normal(size=15) * 0.2
    s1 = s0 + rng.normal(size=15) * 1e-4
    sd0 = rng.normal(size=15)
    sd1 = rng.normal(size=15)
    dt = 1e-4
    ledgers = plant.passive_work_ledgers(s0, s1, sd0, sd1, dt)
    v0 = float(plant.passive_potential(s0))
    v1 = float(plant.passive_potential(s1))
    conservative_closure = float(ledgers["elastic_work"] + (v1 - v0))
    return bool(
        np.isfinite(np.asarray(list(ledgers.values()), dtype=float)).all()
        and float(ledgers["damping_work"]) <= 1e-12
        and abs(conservative_closure) <= 1e-9
        and abs(float(ledgers["active_work"])) <= 1e-15
    ), {"ledgers": ledgers, "conservative_closure_J": conservative_closure}


def _force_source_proof() -> tuple[bool, Any]:
    plant, data = make_plant()
    report = plant.passive_force_components(data)
    q = np.asarray(report["qfrc_total"], dtype=float)
    return bool(
        q.shape == (plant.model.nv,)
        and np.isfinite(q).all()
        and np.allclose(q[0:6], 0.0, atol=0.0, rtol=0.0)
        and float(np.abs(np.asarray(report["load_force"], dtype=float)).max()) == 0.0
        and report["root_direct_force"] is False
    ), {
        "qfrc_shape": list(q.shape),
        "root_inf": float(np.abs(q[0:6]).max()),
        "load_force": report["load_force"],
        "root_direct_force": report["root_direct_force"],
    }


def _perturbation_proof() -> tuple[bool, Any]:
    plant, data = make_plant()
    base = plant.anatomical_coordinates(data)
    perturb = base.copy()
    perturb[9] += plant.region_radius_vector()[9] * 1.15
    tau = np.asarray(plant.passive_elastic_torque(perturb), dtype=float)
    restoring = float(tau[9] * (base[9] - perturb[9]))
    return bool(np.isfinite(tau).all() and restoring > 0.0), {"restoring_product": restoring, "tau": tau}


def run() -> dict[str, Any]:
    checks = Checks()
    for test_id, assertion, clause, fn, tol in (
        ("qualification-PAS-GRAD-001", "passive potential gradient matches independent finite difference", "06_ANATOMICAL_REGION_AND_PASSIVE_LIMIT_MODEL.md::passive force", _gradient_proof, "max error <=5e-5"),
        ("qualification-PAS-ONSET-001", "potential and gradient are exactly zero inside region and restorative outside", "joint_region_contract.json::passive_potential and outward_capacity_taper", _potential_onset_proof, "exact zero inside; finite outside"),
        ("qualification-PAS-DAMP-001", "viscous damping is positive-semidefinite dissipative", "energy_accounting_contract.json::PASS-DAMPING", _damping_proof, "v.T*tau <=0"),
        ("qualification-PAS-WORK-001", "passive work is source-separated and conservative closure holds", "16_ENERGY_POWER_AND_WORK_ACCOUNTING.md::passive ledgers", _work_energy_proof, "closure <=1e-9 J"),
        ("qualification-PAS-FORCE-001", "passive diagnostics contain no floating-root or direct-load force", "plant_contract.json::passive_registry and 12_GENERALIZED_FORCE_AND_VIRTUAL_WORK_MODEL.md", _force_source_proof, "root/load force exactly zero"),
        ("qualification-PAS-RESP-001", "out-of-region perturbation produces a restorative elastic response", "06_ANATOMICAL_REGION_AND_PASSIVE_LIMIT_MODEL.md::generalized passive force", _perturbation_proof, "positive restoring work")
    ):
        safe(checks, test_id, assertion, clause, fn, tol)
    result = "PASS" if not checks.failures else "FAIL"
    return {"suite": "passive", "contract_revision": "loaded-cmj-model-1", "result": result, "tests_run": len(checks.rows), "tests_passed": len(checks.rows) - len(checks.failures), "tests_failed": len(checks.failures), "rows": checks.rows}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-root", required=True)
    args = ap.parse_args()
    report = run()
    out = Path(args.evidence_root) / "qualification.2_PASSIVE.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, default=native), encoding="utf-8")
    for row in report["rows"]:
        if row["result"] == "FAIL":
            print(f"FAIL {row['test_id']}: {row['assertion']} -> {row['measured']}")
    print(f"passive tests_run={report['tests_run']} passed={report['tests_passed']} failed={report['tests_failed']}")
    print(f"RESULT={report['result']} passive-PASSIVE")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
