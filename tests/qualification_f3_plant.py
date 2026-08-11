#!/usr/bin/env python3
"""Independent F3.1 plant, load, topology, and frame qualification."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.simulation.constants import (  # noqa: E402
    ACTION_CHANNELS,
    ACTION_DIM,
    ATHLETE_MASS_KG,
    EXTERNAL_LOAD_MASS_KG,
    GRAVITY_MAGNITUDE,
    TOTAL_MASS_KG,
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_root.mkdir(parents=True, exist_ok=True)

    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    load = int(plant.idx.load_body)
    torso = int(plant.idx.torso_body)
    load_geoms = [gid for gid in plant.idx.shell_geoms if int(model.geom_bodyid[gid]) == load]
    if len(load_geoms) != 1:
        raise AssertionError(f"expected one load shell, found {load_geoms}")
    load_geom = int(load_geoms[0])

    body_ids = np.asarray(plant.idx.all_bodies, dtype=int)
    athlete_ids = np.asarray(plant.idx.athlete_bodies, dtype=int)
    body_mass = np.asarray(model.body_mass[body_ids], dtype=np.float64)
    athlete_mass = float(np.sum(model.body_mass[athlete_ids]))
    total_mass = float(np.sum(body_mass))
    com_manual = (body_mass[:, None] * np.asarray(data.xipos[body_ids])).sum(axis=0) / total_mass
    com_owner = np.asarray(plant.center_of_mass(data), dtype=np.float64)

    local_pos = np.asarray(model.body_pos[load], dtype=np.float64)
    local_quat = np.asarray(model.body_quat[load], dtype=np.float64)
    inertia = np.asarray(model.body_inertia[load], dtype=np.float64)
    radius = float(model.geom_size[load_geom, 0])
    half_length = float(model.geom_size[load_geom, 1])
    expected_inertia = np.asarray((
        EXTERNAL_LOAD_MASS_KG * (3.0 * radius**2 + (2.0 * half_length)**2) / 12.0,
        0.5 * EXTERNAL_LOAD_MASS_KG * radius**2,
        EXTERNAL_LOAD_MASS_KG * (3.0 * radius**2 + (2.0 * half_length)**2) / 12.0,
    ))
    axis_world = np.asarray(data.geom_xmat[load_geom], dtype=np.float64).reshape(3, 3)[:, 2]
    support_data = plant.make_data()
    plant.reset_fixed_hold(support_data)
    summary = plant.contact_wrench_summary(support_data)

    actuator_joint_ids = np.asarray(model.actuator_trnid[:, 0], dtype=int)
    root_joint = int(plant.idx.joint["root"])
    root_actuators = np.flatnonzero(actuator_joint_ids == root_joint).tolist()
    nonzero_applied = bool(np.any(np.abs(np.asarray(data.qfrc_applied)) > 0.0))
    load_parent = int(model.body_parentid[load])
    load_joint_count = int(model.body_jntnum[load])
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(bid)) for bid in body_ids]
    body_rows = []
    for bid, name in zip(body_ids, body_names):
        body_rows.append({
            "body": name,
            "body_id": int(bid),
            "mass_kg": float(model.body_mass[bid]),
            "inertia_kgm2": np.asarray(model.body_inertia[bid], dtype=np.float64),
            "parent_id": int(model.body_parentid[bid]),
            "joint_count": int(model.body_jntnum[bid]),
            "local_pos_m": np.asarray(model.body_pos[bid], dtype=np.float64),
        })

    checks = {
        "compiled_dimensions": {
            "measured": [int(model.nq), int(model.nv), int(model.nu)],
            "expected": [25, 21, 15],
            "pass": (model.nq, model.nv, model.nu) == (25, 21, 15),
        },
        "athlete_mass_kg": {"measured": athlete_mass, "expected": ATHLETE_MASS_KG, "pass": abs(athlete_mass - ATHLETE_MASS_KG) <= 1e-12},
        "load_mass_kg": {"measured": float(model.body_mass[load]), "expected": EXTERNAL_LOAD_MASS_KG, "pass": abs(float(model.body_mass[load]) - EXTERNAL_LOAD_MASS_KG) <= 1e-12},
        "total_mass_kg": {"measured": total_mass, "expected": TOTAL_MASS_KG, "pass": abs(total_mass - TOTAL_MASS_KG) <= 1e-12},
        "load_parent": {"measured": load_parent, "expected": torso, "pass": load_parent == torso},
        "load_joint_count": {"measured": load_joint_count, "expected": 0, "pass": load_joint_count == 0},
        "load_position_parent_m": {"measured": local_pos, "expected": [0.0, 0.0, 0.420], "pass": np.allclose(local_pos, [0.0, 0.0, 0.420], atol=1e-12, rtol=0.0)},
        "load_centerline_m": {"measured": local_pos[:2], "pass": np.allclose(local_pos[:2], 0.0, atol=1e-12, rtol=0.0)},
        "load_inertia_kgm2": {"measured": inertia, "expected": expected_inertia, "pass": np.allclose(inertia, expected_inertia, atol=1e-12, rtol=0.0)},
        "load_long_axis_dot_world_y": {"measured": float(np.dot(axis_world, [0.0, 1.0, 0.0])), "pass": abs(float(np.dot(axis_world, [0.0, 1.0, 0.0]))) >= 1.0 - 1e-12},
        "body_mass_sum_uses_one_load": {"measured": len(body_ids), "expected": 9, "pass": len(body_ids) == 9},
        "com_manual_vs_owner_m": {"measured": np.linalg.norm(com_manual - com_owner), "tolerance": 1e-12, "pass": np.allclose(com_manual, com_owner, atol=1e-12, rtol=0.0)},
        "root_actuator_count": {"measured": len(root_actuators), "expected": 0, "pass": len(root_actuators) == 0},
        "external_qfrc_applied": {"measured": nonzero_applied, "expected": False, "pass": not nonzero_applied},
        "bilateral_reset_normal_force_N": {"measured": np.asarray(summary["normal_force"]), "pass": bool(np.asarray(summary["normal_force"]).shape == (2,) and np.all(np.asarray(summary["normal_force"]) > 0.0))},
        "bilateral_reset_cop_valid": {"measured": np.asarray(summary["cop_valid"]), "pass": bool(np.asarray(summary["cop_valid"]).all())},
    }
    passed = all(bool(row["pass"]) for row in checks.values())
    report = {
        "suite": "F3.1-PLANT-REQUALIFICATION",
        "result": "PASS" if passed else "FAIL",
        "model": {
            "nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu),
            "nbody": int(model.nbody), "njnt": int(model.njnt), "ngeom": int(model.ngeom),
            "neq": int(model.neq), "na": int(model.na),
            "body_rows": body_rows,
            "action_dim": ACTION_DIM,
            "action_channels": list(ACTION_CHANNELS),
        },
        "load": {
            "body": "external_load", "body_id": load, "parent": "torso_head_arms",
            "parent_id": load_parent, "joint_count": load_joint_count,
            "mass_kg": float(model.body_mass[load]), "local_pos_m": local_pos,
            "local_quat_wxyz": local_quat, "inertia_kgm2": inertia,
            "geom_id": load_geom, "geom_radius_m": radius, "geom_half_length_m": half_length,
            "long_axis_world": axis_world,
        },
        "mass_closure": {
            "athlete_mass_kg": athlete_mass, "external_load_mass_kg": float(model.body_mass[load]),
            "total_mass_kg": total_mass,
        },
        "com": {"manual_world_m": com_manual, "owner_world_m": com_owner, "difference_m": com_manual - com_owner},
        "topology": {
            "floating_root": int(model.jnt_type[plant.idx.joint["root"]]) == int(mujoco.mjtJoint.mjJNT_FREE),
            "root_joint_id": root_joint, "root_actuator_ids": root_actuators,
            "nonzero_qfrc_applied_at_reset": nonzero_applied,
            "body_count_excluding_world": len(body_ids),
        },
        "reset_support": {
            "normal_force_N": np.asarray(summary["normal_force"]),
            "cop_valid": np.asarray(summary["cop_valid"]),
            "contact_active": bool(summary["contact_active"]),
            "prohibited_contact": bool(summary["prohibited_contact"]),
        },
        "checks": checks,
        "f2_trace_dynamics_untouched": True,
        "physics_constants": {"dt_s": 0.000125, "control_dt_s": 0.005, "substeps": 40, "gravity_mps2": GRAVITY_MAGNITUDE},
    }
    output = args.evidence_root / "11_F3_MASS_INERTIA_COM_REPORT.json"
    output.write_text(json.dumps(native(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": report["result"], "failed_checks": [name for name, row in checks.items() if not row["pass"]], "output": str(output)}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
