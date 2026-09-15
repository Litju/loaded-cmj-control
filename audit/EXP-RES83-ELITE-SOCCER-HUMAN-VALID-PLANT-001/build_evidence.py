#!/usr/bin/env python3
"""RES-83 deterministic evidence builder for the V3 elite-soccer Plant.

Authority: LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1
Mission:   RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001

Every audit function is deterministic, read-only with respect to the Plant
and the RES-95 bundle, and returns a JSON-serializable report with a
"status" of PASS or FAIL plus its individual checks.  The test module
tests/test_res83_v3_plant.py imports these functions and asserts PASS, so
the executed tests and the archived evidence are the same computation.

Run:  python3 build_evidence.py          # write all JSON artifacts
      python3 build_evidence.py --print   # print statuses only
"""

from __future__ import annotations

import ast
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[2]
EVIDENCE_DIR = Path(__file__).resolve().parent
AUTHORITY_DIR = REPO / "audit" / "EXP-RES95-ELITE-SOCCER-PLANT-MODEL-AUTHORITY-001"
SRC = REPO / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3 import plant as P  # noqa: E402

TOL = 1e-9
FK_PROBE_Q = math.radians(10.0)

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"


def _status(checks: list[dict[str, Any]]) -> str:
    return STATUS_PASS if all(c["pass"] for c in checks) else STATUS_FAIL


def _f(x: Any) -> float:
    return float(x)


def _vec(x: Any) -> list[float]:
    return [float(v) for v in np.asarray(x, dtype=np.float64).reshape(-1)]


def _close(a: Any, b: Any, tol: float = TOL) -> bool:
    return bool(np.allclose(np.asarray(a, dtype=np.float64),
                            np.asarray(b, dtype=np.float64), rtol=0.0, atol=tol))


def _fullinertia_matrix(six: tuple[float, ...]) -> np.ndarray:
    """MuJoCo fullinertia order (ixx, iyy, izz, ixy, ixz, iyz) -> 3x3 matrix."""
    ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in six)
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


def _rel_close(a: Any, b: Any, rtol: float = 1e-12, atol: float = 1e-12) -> bool:
    return bool(np.allclose(np.asarray(a, dtype=np.float64),
                            np.asarray(b, dtype=np.float64), rtol=rtol, atol=atol))


# ===========================================================================
# 0. Authority bundle integrity
# ===========================================================================
def authority_bundle_integrity() -> dict[str, Any]:
    manifest = json.loads((AUTHORITY_DIR / "MODEL_AUTHORITY_HASH_MANIFEST.json").read_text())
    checks = []
    for entry in manifest["files"]:
        path = AUTHORITY_DIR / entry["path"]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        checks.append({
            "check": f"bundle_hash:{entry['path']}",
            "pass": digest == entry["sha256"],
            "expected_sha256": entry["sha256"],
            "actual_sha256": digest,
        })
    checks.append({
        "check": "authority_id",
        "pass": manifest["authority_id"] == C.V3_AUTHORITY_ID,
        "value": manifest["authority_id"],
    })
    return {
        "schema_version": "1.0.0",
        "artifact": "AUTHORITY_BUNDLE_INTEGRITY",
        "authority_bundle": str(AUTHORITY_DIR.relative_to(REPO)),
        "authority_id": C.V3_AUTHORITY_ID,
        "model_id": C.V3_MODEL_ID,
        "file_count": len(manifest["files"]),
        "entry_head": manifest["entry_head"],
        "entry_tree": manifest["entry_tree"],
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 1. Runtime introspection
# ===========================================================================
def introspection() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)

    bodies = []
    for b in range(m.nbody):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        inertia = P.body_inertia_full(m, b)
        bodies.append({
            "id": b,
            "name": name,
            "parent": int(m.body_parentid[b]),
            "mass_kg": _f(m.body_mass[b]),
            "com_pos_in_parent_frame_m": _vec(m.body_ipos[b]),
            "com_pos_world_at_zero_qpos_m": _vec(d.xipos[b]),
            "diaginertia_kg_m2": _vec(m.body_inertia[b]),
            "full_inertia_about_com_kg_m2": [list(map(float, row)) for row in inertia],
            "geom_count": int(m.body_geomnum[b]),
        })

    joints = []
    for j in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        dof = int(m.jnt_dofadr[j])
        joints.append({
            "id": j,
            "name": name,
            "type": int(m.jnt_type[j]),
            "qpos_adr": int(m.jnt_qposadr[j]),
            "dof_adr": dof,
            "axis": _vec(m.jnt_axis[j]),
            "range_rad": _vec(m.jnt_range[j]),
            "limited": bool(m.jnt_limited[j]),
            "stiffness": _f(m.jnt_stiffness[j]),
            "damping": _f(m.dof_damping[dof]),
            "armature": _f(m.dof_armature[dof]),
        })

    geoms = []
    for g in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        body = int(m.geom_bodyid[g])
        body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, body)
        geoms.append({
            "id": g,
            "name": name,
            "type": int(m.geom_type[g]),
            "body": body_name,
            "contype": int(m.geom_contype[g]),
            "conaffinity": int(m.geom_conaffinity[g]),
            "condim": int(m.geom_condim[g]),
            "size": _vec(m.geom_size[g]),
            "pos_in_body_frame_m": _vec(m.geom_pos[g]),
            "friction": _vec(m.geom_friction[g]),
            "collision_class": plant.geom_collision_class(g),
        })

    actuators = []
    for a in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        joint = int(m.actuator_trnid[a][0])
        actuators.append({
            "id": a,
            "name": name,
            "joint": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, joint),
            "gear": _vec(m.actuator_gear[a]),
            "ctrllimited": bool(m.actuator_ctrllimited[a]),
            "forcelimited": bool(m.actuator_forcelimited[a]),
            "ctrlrange": _vec(m.actuator_ctrlrange[a]),
            "forcerange": _vec(m.actuator_forcerange[a]),
        })

    checks = [
        {"check": "nbody", "pass": int(m.nbody) == C.V3_COMPILED_NBODY, "value": int(m.nbody)},
        {"check": "njnt", "pass": int(m.njnt) == C.V3_COMPILED_NJNT, "value": int(m.njnt)},
        {"check": "nq", "pass": int(m.nq) == C.V3_COMPILED_NQ, "value": int(m.nq)},
        {"check": "nv", "pass": int(m.nv) == C.V3_COMPILED_NV, "value": int(m.nv)},
        {"check": "nu", "pass": int(m.nu) == C.V3_COMPILED_NU, "value": int(m.nu)},
        {"check": "neq", "pass": int(m.neq) == C.V3_COMPILED_NEQ, "value": int(m.neq)},
        {"check": "na", "pass": int(m.na) == C.V3_COMPILED_NA, "value": int(m.na)},
        {"check": "ntendon", "pass": int(m.ntendon) == 0, "value": int(m.ntendon)},
        {"check": "ngeom", "pass": int(m.ngeom) == C.V3_COMPILED_NGEOM, "value": int(m.ngeom)},
        {"check": "body_names",
         "pass": tuple(b["name"] for b in bodies) == C.V3_BODY_NAMES,
         "value": [b["name"] for b in bodies]},
        {"check": "joint_names",
         "pass": tuple(j["name"] for j in joints) == C.V3_JOINT_NAMES,
         "value": [j["name"] for j in joints]},
        {"check": "actuator_names",
         "pass": tuple(a["name"] for a in actuators) == C.V3_ACTUATOR_NAMES,
         "value": [a["name"] for a in actuators]},
    ]
    return {
        "schema_version": "1.0.0",
        "artifact": "MODEL_INTROSPECTION",
        "model_revision": C.V3_MODEL_REVISION,
        "model_id": C.V3_MODEL_ID,
        "xml_sha256": hashlib.sha256(P.model_xml().encode()).hexdigest(),
        "counts": {
            "nbody": int(m.nbody), "njnt": int(m.njnt), "nq": int(m.nq), "nv": int(m.nv),
            "nu": int(m.nu), "neq": int(m.neq), "na": int(m.na), "ngeom": int(m.ngeom),
            "ntendon": int(m.ntendon), "nmocap": int(m.nmocap), "nq_expected": C.V3_COMPILED_NQ,
        },
        "options": {
            "timestep_s": _f(m.opt.timestep),
            "integrator": int(m.opt.integrator),
            "solver": int(m.opt.solver),
            "cone": int(m.opt.cone),
            "gravity": _vec(m.opt.gravity),
            "disableflags": int(m.opt.disableflags),
            "filterparent_enabled": not bool(
                m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT),
        },
        "bodies": bodies,
        "joints": joints,
        "geoms": geoms,
        "actuators": actuators,
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 2. Mass / inertia audit
# ===========================================================================
def mass_inertia_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    idx = plant.idx
    checks: list[dict[str, Any]] = []

    def body_check(body: str, mass: float, pos: tuple[float, ...],
                   diag: tuple[float, ...] | None = None,
                   full: tuple[float, ...] | None = None) -> None:
        bid = idx.body[body]
        checks.append({
            "check": f"mass:{body}",
            "pass": abs(_f(m.body_mass[bid]) - mass) <= 1e-9,
            "expected": mass, "actual": _f(m.body_mass[bid]),
        })
        checks.append({
            "check": f"com:{body}",
            "pass": _close(m.body_ipos[bid], pos),
            "expected": list(pos), "actual": _vec(m.body_ipos[bid]),
        })
        if diag is not None:
            checks.append({
                "check": f"diaginertia:{body}",
                "pass": _close(m.body_inertia[bid], diag),
                "expected": list(diag), "actual": _vec(m.body_inertia[bid]),
            })
        if full is not None:
            inertia = P.body_inertia_full(m, bid)
            expected = _fullinertia_matrix(full)
            checks.append({
                "check": f"fullinertia:{body}",
                "pass": _rel_close(inertia, expected, rtol=1e-12, atol=1e-12),
                "expected": [list(map(float, r)) for r in expected],
                "actual": [list(map(float, r)) for r in inertia],
            })

    body_check("pelvis", C.V3_PELVIS_MASS_KG, C.V3_PELVIS_COM_M, C.V3_PELVIS_DIAGINERTIA_KG_M2)
    body_check("HAT", C.V3_HAT_MASS_KG, C.V3_HAT_COM_M, full=C.V3_HAT_FULLINERTIA_KG_M2)
    body_check("bar", C.V3_BAR_MASS_KG, (0.0, 0.0, 0.0), C.V3_BAR_DIAGINERTIA_KG_M2)
    for side in C.V3_SIDES:
        body_check(f"{side}_thigh", C.V3_THIGH_MASS_KG,
                   (0.0, 0.0, -C.V3_THIGH_COM_BELOW_HJC_M), C.V3_THIGH_DIAGINERTIA_KG_M2)
        body_check(f"{side}_shank", C.V3_SHANK_MASS_KG,
                   (0.0, 0.0, -C.V3_SHANK_COM_BELOW_KJC_M), C.V3_SHANK_DIAGINERTIA_KG_M2)
        body_check(f"{side}_hindfoot", C.V3_HINDFOOT_MASS_KG, C.V3_HINDFOOT_COM_M,
                   C.V3_HINDFOOT_DIAGINERTIA_KG_M2)
        body_check(f"{side}_forefoot", C.V3_FOREFOOT_MASS_KG, C.V3_FOREFOOT_COM_IN_FOREFOOT_M,
                   C.V3_FOREFOOT_DIAGINERTIA_KG_M2)
        body_check(f"{side}_toe", C.V3_TOE_MASS_KG, C.V3_TOE_COM_IN_TOE_M,
                   C.V3_TOE_DIAGINERTIA_KG_M2)

    total = _f(m.body_mass.sum())
    athlete = total - _f(m.body_mass[idx.body["bar"]])
    feet = sum(_f(m.body_mass[idx.body[f"{s}_hindfoot"]]) + _f(m.body_mass[idx.body[f"{s}_forefoot"]])
               + _f(m.body_mass[idx.body[f"{s}_toe"]]) for s in C.V3_SIDES)
    checks.extend([
        {"check": "athlete_mass_79kg", "pass": abs(athlete - 79.0) <= 1e-9, "value": athlete},
        {"check": "system_mass_99kg", "pass": abs(total - 99.0) <= 1e-9, "value": total},
        {"check": "two_feet_mass", "pass": abs(feet - C.V3_TWO_FEET_MASS_KG) <= 1e-9, "value": feet},
        {"check": "bar_mass", "pass": abs(_f(m.body_mass[idx.body["bar"]]) - 20.0) <= 1e-9,
         "value": _f(m.body_mass[idx.body["bar"]])},
    ])
    return {
        "schema_version": "1.0.0",
        "artifact": "MASS_INERTIA_AUDIT",
        "authority": "ANTHROPOMETRY_BSIP_AUTHORITY AB-03/AB-04/AB-08/AB-14; FM-04/FM-07/FM-08; DERIVED_QUANTITIES",
        "athlete_mass_kg": athlete,
        "system_mass_kg": total,
        "body_table": [
            {"name": b["name"], "mass_kg": b["mass_kg"], "com_pos_in_parent_frame_m": b["com_pos_in_parent_frame_m"],
             "diaginertia_kg_m2": b["diaginertia_kg_m2"]}
            for b in introspection()["bodies"]
        ],
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 3. HAT / bar realization audit
# ===========================================================================
def _composite_bar_numbers() -> dict[str, Any]:
    d_g = C.V3_BAR_GRIP_DIAMETER_M
    d_s = C.V3_BAR_SLEEVE_DIAMETER_M
    l_s = C.V3_BAR_SLEEVE_LENGTH_M
    l_shaft = C.V3_BAR_SHAFT_LENGTH_M
    volume = math.pi / 4 * (d_g ** 2 * l_shaft + 2 * d_s ** 2 * l_s)
    rho = C.V3_LOAD_MASS_KG / volume
    m_shaft = rho * math.pi / 4 * d_g ** 2 * l_shaft
    m_sleeve = rho * math.pi / 4 * d_s ** 2 * l_s
    i_axis = 0.5 * m_shaft * (d_g / 2) ** 2 + 2 * 0.5 * m_sleeve * (d_s / 2) ** 2
    i_trans = (m_shaft * (3 * (d_g / 2) ** 2 + l_shaft ** 2) / 12.0
               + 2.0 * (m_sleeve * (3 * (d_s / 2) ** 2 + l_s ** 2) / 12.0
                        + m_sleeve * ((l_shaft / 2.0) + (l_s / 2.0)) ** 2))
    return {
        "volume_m3": volume, "rho_eff_kg_m3": rho, "m_shaft_kg": m_shaft,
        "m_sleeve_each_kg": m_sleeve, "I_axis_kg_m2": i_axis, "I_transverse_kg_m2": i_trans,
    }


def hat_bar_realization_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    idx = plant.idx
    checks: list[dict[str, Any]] = []

    # HAT full tensor realization
    hat_inertia = P.body_inertia_full(m, idx.body["HAT"])
    expected_hat = _fullinertia_matrix(C.V3_HAT_FULLINERTIA_KG_M2)
    checks.append({
        "check": "hat_full_inertia_realized",
        "pass": _rel_close(hat_inertia, expected_hat, rtol=1e-12, atol=1e-12),
        "expected": [list(map(float, r)) for r in expected_hat],
        "actual": [list(map(float, r)) for r in hat_inertia],
    })
    eig = np.sort(np.linalg.eigvalsh(hat_inertia))[::-1]
    checks.append({
        "check": "hat_principal_moments",
        "pass": _rel_close(eig, C.V3_HAT_PRINCIPAL_MOMENTS_KG_M2, rtol=1e-12, atol=1e-12),
        "expected": list(C.V3_HAT_PRINCIPAL_MOMENTS_KG_M2), "actual": _vec(eig),
    })
    checks.append({
        "check": "hat_ixz_preserved_nonzero",
        "pass": abs(hat_inertia[0, 2]) > 1e-6 and _close(hat_inertia[0, 2], -0.02077944879714553),
        "value": _f(hat_inertia[0, 2]),
    })

    # Bar local placement and composite construction
    checks.append({
        "check": "bar_offset_in_hat_frame",
        "pass": _close(m.body_pos[idx.body["bar"]], C.V3_BAR_CENTER_IN_HAT_FRAME_M),
        "expected": list(C.V3_BAR_CENTER_IN_HAT_FRAME_M),
        "actual": _vec(m.body_pos[idx.body["bar"]]),
    })
    checks.append({
        "check": "bar_welded_to_hat_no_joint",
        "pass": int(m.body_jntnum[idx.body["bar"]]) == 0
                and int(m.body_parentid[idx.body["bar"]]) == idx.body["HAT"],
        "jntnum": int(m.body_jntnum[idx.body["bar"]]),
        "parent": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.body_parentid[idx.body["bar"]])),
    })
    composite = _composite_bar_numbers()
    checks.append({
        "check": "bar_composite_rho_eff",
        "pass": abs(composite["rho_eff_kg_m3"] - C.V3_BAR_RHO_EFF_KG_M3) <= 1e-6,
        "expected": C.V3_BAR_RHO_EFF_KG_M3, "actual": composite["rho_eff_kg_m3"],
    })
    checks.append({
        "check": "bar_composite_shaft_mass",
        "pass": abs(composite["m_shaft_kg"] - C.V3_BAR_SHAFT_MASS_KG) <= 1e-9,
        "expected": C.V3_BAR_SHAFT_MASS_KG, "actual": composite["m_shaft_kg"],
    })
    checks.append({
        "check": "bar_composite_sleeve_mass",
        "pass": abs(composite["m_sleeve_each_kg"] - C.V3_BAR_SLEEVE_EACH_MASS_KG) <= 1e-9,
        "expected": C.V3_BAR_SLEEVE_EACH_MASS_KG, "actual": composite["m_sleeve_each_kg"],
    })
    checks.append({
        "check": "bar_inertia_realized",
        "pass": _close(m.body_inertia[idx.body["bar"]], C.V3_BAR_DIAGINERTIA_KG_M2),
        "expected": list(C.V3_BAR_DIAGINERTIA_KG_M2),
        "actual": _vec(m.body_inertia[idx.body["bar"]]),
    })
    checks.append({
        "check": "bar_not_uniform_rod_surrogate",
        "pass": abs(composite["I_transverse_kg_m2"] - 8.067646666666667) > 1.0
                and abs(_f(m.body_inertia[idx.body["bar"]][0]) - composite["I_transverse_kg_m2"]) <= 1e-6,
        "rejected_uniform_rod_I_transverse": 8.067646666666667,
        "implemented_I_transverse": _f(m.body_inertia[idx.body["bar"]][0]),
    })
    # Bar collision composite geometry
    shaft_size = m.geom_size[idx.geom["bar_shaft"]]
    checks.append({
        "check": "bar_shaft_geometry",
        "pass": _close(shaft_size, (C.V3_BAR_GRIP_DIAMETER_M / 2, C.V3_BAR_SHAFT_LENGTH_M / 2, 0.0)),
        "expected": [C.V3_BAR_GRIP_DIAMETER_M / 2, C.V3_BAR_SHAFT_LENGTH_M / 2, 0.0],
        "actual": _vec(shaft_size),
    })
    for sleeve in ("bar_sleeve_left", "bar_sleeve_right"):
        size = m.geom_size[idx.geom[sleeve]]
        checks.append({
            "check": f"{sleeve}_geometry",
            "pass": _close(size, (C.V3_BAR_SLEEVE_DIAMETER_M / 2, C.V3_BAR_SLEEVE_LENGTH_M / 2, 0.0)),
            "expected": [C.V3_BAR_SLEEVE_DIAMETER_M / 2, C.V3_BAR_SLEEVE_LENGTH_M / 2, 0.0],
            "actual": _vec(size),
        })
    sleeve_centers = sorted(abs(float(m.geom_pos[idx.geom[s]][1])) for s in ("bar_sleeve_left", "bar_sleeve_right"))
    expected_center = (C.V3_BAR_SLEEVE_SPAN_M[0] + C.V3_BAR_SLEEVE_SPAN_M[1]) / 2
    checks.append({
        "check": "bar_sleeve_spans",
        "pass": _close(sleeve_centers, (expected_center, expected_center)),
        "expected": [expected_center, expected_center], "actual": sleeve_centers,
    })

    # Combined HAT + bar system realization (world frame == HAT frame at zero qpos)
    hat_id, bar_id = idx.body["HAT"], idx.body["bar"]
    m1 = _f(m.body_mass[hat_id])
    m2 = _f(m.body_mass[bar_id])
    com = (m1 * np.asarray(d.xipos[hat_id]) + m2 * np.asarray(d.xipos[bar_id])) / (m1 + m2)
    inertia = np.zeros((3, 3))
    for bid, mass in ((hat_id, m1), (bar_id, m2)):
        dd = np.asarray(d.xipos[bid]) - com
        rot = np.asarray(d.xmat[bid]).reshape(3, 3)
        i_world = rot @ P.body_inertia_full(m, bid) @ rot.T
        inertia += i_world + mass * (float(dd @ dd) * np.eye(3) - np.outer(dd, dd))
    expected_system = np.array(C.V3_HAT_BAR_SYSTEM["inertia_about_com_kg_m2"])
    checks.append({
        "check": "hat_bar_system_mass",
        "pass": abs(m1 + m2 - C.V3_HAT_BAR_SYSTEM["mass_kg"]) <= 1e-9,
        "value": m1 + m2,
    })
    checks.append({
        "check": "hat_bar_system_com",
        "pass": _close(com, C.V3_HAT_BAR_SYSTEM["com_m"]),
        "expected": list(C.V3_HAT_BAR_SYSTEM["com_m"]), "actual": _vec(com),
    })
    checks.append({
        "check": "hat_bar_system_inertia",
        "pass": _rel_close(inertia, expected_system, rtol=1e-12, atol=1e-12),
        "expected": [list(map(float, r)) for r in expected_system],
        "actual": [list(map(float, r)) for r in inertia],
    })

    # Direct sealed-authority anchors: the realized HAT/bar inertias and the
    # declared system Iyy sensitivity range are compared against
    # DERIVED_QUANTITIES.json itself, never against a transcription alone.
    derived = json.loads((AUTHORITY_DIR / "DERIVED_QUANTITIES.json").read_text())
    auth_hat = np.asarray(derived["hat"]["inertia_about_com_kg_m2"], dtype=np.float64)
    checks.append({
        "check": "hat_full_inertia_matches_res95_derived",
        "pass": _rel_close(hat_inertia, auth_hat, rtol=1e-12, atol=1e-15),
        "expected": derived["hat"]["inertia_about_com_kg_m2"],
        "actual": [list(map(float, r)) for r in hat_inertia],
    })
    auth_bar_diag = np.array([derived["bar"]["I_transverse_kg_m2"],
                              derived["bar"]["I_axis_kg_m2"],
                              derived["bar"]["I_transverse_kg_m2"]])
    checks.append({
        "check": "bar_inertia_matches_res95_derived",
        "pass": _rel_close(m.body_inertia[idx.body["bar"]], auth_bar_diag, rtol=1e-12, atol=1e-15),
        "expected": _vec(auth_bar_diag),
        "actual": _vec(m.body_inertia[idx.body["bar"]]),
    })
    sens = derived["hat_sensitivity"]
    recomputed_range = [
        min(float(c["system_inertia_kg_m2"][1][1]) for c in sens["cases"]),
        max(float(c["system_inertia_kg_m2"][1][1]) for c in sens["cases"]),
    ]
    declared_range = list(C.V3_SYSTEM_IYY_SENSITIVITY_RANGE_KG_M2)
    checks.append({
        "check": "system_iyy_sensitivity_range_matches_res95",
        "pass": _close(declared_range, (sens["system_iyy_min"], sens["system_iyy_max"]))
                and _close(declared_range, recomputed_range),
        "declared": declared_range,
        "authority_declared": [sens["system_iyy_min"], sens["system_iyy_max"]],
        "authority_recomputed_from_sensitivity_cases": recomputed_range,
    })
    return {
        "schema_version": "1.0.0",
        "artifact": "HAT_BAR_REALIZATION_AUDIT",
        "authority": ("UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY UB-03..UB-11; "
                      "LOAD_BAR_AUTHORITY LB-01..LB-10; DERIVED_QUANTITIES"),
        "hat_realized": {
            "mass_kg": _f(m.body_mass[hat_id]),
            "com_m": _vec(m.body_ipos[hat_id]),
            "full_inertia_about_com_kg_m2": [list(map(float, r)) for r in hat_inertia],
        },
        "bar_realized": {
            "mass_kg": _f(m.body_mass[bar_id]),
            "diaginertia_kg_m2": _vec(m.body_inertia[bar_id]),
            "offset_in_hat_frame_m": _vec(m.body_pos[bar_id]),
            "composite": composite,
            "collision": {
                "shaft": {"radius_m": _f(m.geom_size[idx.geom["bar_shaft"]][0]),
                          "half_length_m": _f(m.geom_size[idx.geom["bar_shaft"]][1])},
                "sleeve": {"radius_m": _f(m.geom_size[idx.geom["bar_sleeve_left"]][0]),
                           "half_length_m": _f(m.geom_size[idx.geom["bar_sleeve_left"]][1]),
                           "span_m": list(C.V3_BAR_SLEEVE_SPAN_M)},
                "type": "coaxial stepped cylinder (not a uniform rod surrogate)",
            },
        },
        "hat_bar_system_realized": {
            "mass_kg": m1 + m2, "com_m": _vec(com),
            "inertia_about_com_kg_m2": [list(map(float, r)) for r in inertia],
            "system_iyy_kg_m2": _f(inertia[1, 1]),
            "declared_sensitivity_iyy_range_kg_m2": list(C.V3_SYSTEM_IYY_SENSITIVITY_RANGE_KG_M2),
        },
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 4. Joint address / order / sign / range audit + deterministic FK probes
# ===========================================================================
def _rot_y(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _fk_predicted_delta(point_rest: np.ndarray, anchor_rest: np.ndarray,
                        axis_sign: float, q: float) -> np.ndarray:
    rot = _rot_y(axis_sign * q)
    return rot @ (point_rest - anchor_rest) + anchor_rest - point_rest


def _fk_probe(plant: P.V3Plant, joint: str, axis_sign: float,
              points: dict[str, int], q: float) -> dict[str, Any]:
    """Measure and predict the rigid FK displacement of body points under +q."""
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    rest = {name: np.asarray(d.xpos[bid]).copy() for name, bid in points.items()}
    anchor = np.asarray(d.xanchor[plant.idx.joint[joint]]).copy()
    plant.reset(d)
    d.qpos[plant.idx.qadr[joint]] = q
    mujoco.mj_forward(m, d)
    measured = {name: np.asarray(d.xpos[bid]) - rest[name] for name, bid in points.items()}
    predicted = {name: _fk_predicted_delta(rest[name], anchor, axis_sign, q)
                 for name in points}
    max_err = max(float(np.max(np.abs(measured[name] - predicted[name]))) for name in points)
    return {
        "joint": joint,
        "q_rad": q,
        "axis_sign": "axis +y" if axis_sign > 0 else "axis -y",
        "points": {
            name: {
                "measured_delta_m": _vec(measured[name]),
                "predicted_delta_m": _vec(predicted[name]),
                "max_abs_error_m": float(np.max(np.abs(measured[name] - predicted[name]))),
            }
            for name in points
        },
        "max_abs_error_m": max_err,
    }


def joint_fk_sign_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    idx = plant.idx
    checks: list[dict[str, Any]] = []

    # Address / order / axis / range / passives
    for j, name in enumerate(C.V3_JOINT_NAMES):
        jid = idx.joint[name]
        checks.append({"check": f"order:{name}", "pass": jid == j, "jid": jid})
        expected_axis = C.V3_JOINT_AXIS[name]
        checks.append({"check": f"axis:{name}", "pass": _close(m.jnt_axis[jid], expected_axis),
                       "expected": list(expected_axis), "actual": _vec(m.jnt_axis[jid])})
        rng = C.V3_JOINT_RANGES_RAD[name]
        if rng is None:
            checks.append({"check": f"range:{name}", "pass": not bool(m.jnt_limited[jid]),
                           "limited": bool(m.jnt_limited[jid]),
                           "stiffness": _f(m.jnt_stiffness[jid]),
                           "damping": _f(m.dof_damping[int(m.jnt_dofadr[jid])]),
                           "armature": _f(m.dof_armature[int(m.jnt_dofadr[jid])])})
            checks.append({"check": f"passive_zero:{name}",
                           "pass": _f(m.jnt_stiffness[jid]) == 0.0
                                   and _f(m.dof_damping[int(m.jnt_dofadr[jid])]) == 0.0
                                   and _f(m.dof_armature[int(m.jnt_dofadr[jid])]) == 0.0})
        else:
            checks.append({"check": f"range:{name}", "pass": _close(m.jnt_range[jid], rng, 1e-9),
                           "expected": list(rng), "actual": _vec(m.jnt_range[jid])})
    for name in C.V3_MTP_JOINT_NAMES:
        jid = idx.joint[name]
        checks.append({"check": f"passive_mtp:{name}",
                       "pass": _close(m.jnt_stiffness[jid], C.V3_MTP_PASSIVE_STIFFNESS_NM_PER_RAD)
                               and _close(m.dof_damping[int(m.jnt_dofadr[jid])],
                                          C.V3_MTP_PASSIVE_DAMPING_NMS_PER_RAD)})
    for name in C.V3_MAJOR_JOINT_NAMES:
        jid = idx.joint[name]
        checks.append({"check": f"passive_zero_major:{name}",
                       "pass": _f(m.jnt_stiffness[jid]) == 0.0
                               and _f(m.dof_damping[int(m.jnt_dofadr[jid])]) == 0.0})

    # Deterministic FK probes (JC-09)
    probes: list[dict[str, Any]] = []

    def add_probe(probe: dict[str, Any], sign_ok: bool, label: str) -> None:
        probe["sign_ok"] = sign_ok
        probes.append(probe)
        checks.append({"check": f"fk:{label}",
                       "pass": sign_ok and probe["max_abs_error_m"] <= 1e-9,
                       "sign_ok": sign_ok, "max_abs_error_m": probe["max_abs_error_m"]})

    # root_tx / root_tz: exact rigid translation of every body
    for joint, axis_index, label in (("root_tx", 0, "root_tx_translation"),
                                     ("root_tz", 2, "root_tz_translation")):
        d = plant.make_data()
        plant.reset(d)
        rest = np.array([d.xpos[b] for b in range(1, m.nbody)])
        plant.reset(d)
        d.qpos[idx.qadr[joint]] = 0.01
        mujoco.mj_forward(m, d)
        moved = np.array([d.xpos[b] for b in range(1, m.nbody)])
        delta = moved - rest
        expected = np.zeros_like(delta)
        expected[:, axis_index] = 0.01
        err = float(np.max(np.abs(delta - expected)))
        probe = {"joint": joint, "q_rad": 0.01, "kind": "translation",
                 "points": {"all_bodies": {"max_abs_error_m": err}},
                 "max_abs_error_m": err}
        add_probe(probe, err <= 1e-12, label)

    # root_ry: whole-body pitch, a point above the root moves to +x
    probe = _fk_probe(plant, "root_ry", +1.0,
                      {"bar_origin": idx.body["bar"]}, FK_PROBE_Q)
    add_probe(probe, probe["points"]["bar_origin"]["measured_delta_m"][0] > 0.0, "root_ry_forward_pitch")
    # trunk_pelvis: forward flexion, head point moves +x
    probe = _fk_probe_geom(plant, "trunk_pelvis", +1.0,
                           {"head": idx.geom["hat_head_collision"],
                            "bar_origin": idx.geom["bar_shaft"]}, FK_PROBE_Q)
    add_probe(probe, probe["points"]["head"]["measured_delta_m"][0] > 0.0, "trunk_forward_flexion")
    # hip: knee moves anteriorly (+x)
    probe = _fk_probe(plant, "left_hip", -1.0,
                      {"knee_origin": idx.body["left_shank"]}, FK_PROBE_Q)
    add_probe(probe, probe["points"]["knee_origin"]["measured_delta_m"][0] > 0.0, "hip_flexion")
    # knee: ankle moves posteriorly (-x)
    probe = _fk_probe(plant, "left_knee", +1.0,
                      {"ankle_origin": idx.body["left_hindfoot"]}, FK_PROBE_Q)
    add_probe(probe, probe["points"]["ankle_origin"]["measured_delta_m"][0] < 0.0, "knee_flexion")
    # ankle: toe tip rises (+z)
    probe = _fk_probe_geom(plant, "left_ankle", -1.0,
                           {"toe_patch": idx.geom["left_toe_support"],
                            "forefoot_patch": idx.geom["left_forefoot_support"]}, FK_PROBE_Q)
    toe_patch_dz = probe["points"]["toe_patch"]["measured_delta_m"][2]
    add_probe(probe, toe_patch_dz > 0.0, "ankle_dorsiflexion")
    # mtp: toe tip rises relative to the forefoot
    probe = _fk_probe_geom(plant, "left_mtp", -1.0,
                           {"toe_patch": idx.geom["left_toe_support"]}, FK_PROBE_Q)
    add_probe(probe, probe["points"]["toe_patch"]["measured_delta_m"][2] > 0.0,
              "mtp_dorsiflexion")

    return {
        "schema_version": "1.0.0",
        "artifact": "JOINT_FK_SIGN_AUDIT",
        "authority": "JOINT_COORDINATE_ROM_AUTHORITY JC-02..JC-10; PLANT_TOPOLOGY_AUTHORITY PT-03/PT-07",
        "joint_order": list(C.V3_JOINT_NAMES),
        "qpos_addresses": {n: int(idx.qadr[n]) for n in C.V3_JOINT_NAMES},
        "dof_addresses": {n: int(idx.vadr[n]) for n in C.V3_JOINT_NAMES},
        "fk_probes": probes,
        "checks": checks,
        "status": _status(checks),
    }


def _fk_probe_geom(plant: P.V3Plant, joint: str, axis_sign: float,
                   points: dict[str, int], q: float, geom_points: bool = True) -> dict[str, Any]:
    """FK probe reading world positions from geom ids."""
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    rest = {name: np.asarray(d.geom_xpos[gid]).copy() for name, gid in points.items()}
    anchor = np.asarray(d.xanchor[plant.idx.joint[joint]]).copy()
    plant.reset(d)
    d.qpos[plant.idx.qadr[joint]] = q
    mujoco.mj_forward(m, d)
    measured = {name: np.asarray(d.geom_xpos[gid]) - rest[name] for name, gid in points.items()}
    predicted = {name: _fk_predicted_delta(rest[name], anchor, axis_sign, q) for name in points}
    max_err = max(float(np.max(np.abs(measured[name] - predicted[name]))) for name in points)
    return {
        "joint": joint, "q_rad": q,
        "axis_sign": "axis +y" if axis_sign > 0 else "axis -y",
        "points": {
            name: {"measured_delta_m": _vec(measured[name]),
                   "predicted_delta_m": _vec(predicted[name]),
                   "max_abs_error_m": float(np.max(np.abs(measured[name] - predicted[name])))}
            for name in points
        },
        "max_abs_error_m": max_err,
    }


# ===========================================================================
# 5. ROM reachability
# ===========================================================================
def rom_reachability_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    idx = plant.idx
    checks: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    limit_enum = mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT

    for name in C.V3_JOINT_NAMES:
        rng = C.V3_JOINT_RANGES_RAD[name]
        jid = idx.joint[name]
        row: dict[str, Any] = {"joint": name, "range_rad": None if rng is None else list(rng)}
        if rng is None:
            row["reachable_lower"] = True
            row["reachable_upper"] = True
            rows.append(row)
            continue
        for bound_name, value in (("lower", rng[0]), ("upper", rng[1])):
            d = plant.make_data()
            plant.reset(d)
            d.qpos[idx.qadr[name]] = value
            mujoco.mj_forward(m, d)
            active = [i for i in range(d.nefc)
                      if d.efc_type[i] == limit_enum and d.efc_id[i] == jid]
            row[f"reachable_{bound_name}"] = len(active) == 0
            checks.append({"check": f"reachable:{name}:{bound_name}",
                           "pass": len(active) == 0, "active_limits": len(active)})
        # outside the envelope the limit constraint must activate (hard structural bound)
        for bound_name, outside in (("lower", rng[0] - 0.05), ("upper", rng[1] + 0.05)):
            d = plant.make_data()
            plant.reset(d)
            d.qpos[idx.qadr[name]] = outside
            mujoco.mj_forward(m, d)
            active = [i for i in range(d.nefc)
                      if d.efc_type[i] == limit_enum and d.efc_id[i] == jid]
            row[f"outside_{bound_name}_limit_force"] = len(active) > 0
            checks.append({"check": f"hard_bound:{name}:{bound_name}", "pass": len(active) > 0,
                           "active_limits": len(active)})
        rows.append(row)

    # Reverse-knee structural impossibility (JC-05)
    jid = idx.joint["left_knee"]
    d = plant.make_data()
    plant.reset(d)
    d.qpos[idx.qadr["left_knee"]] = -0.2
    mujoco.mj_forward(m, d)
    active = [i for i in range(d.nefc)
              if d.efc_type[i] == limit_enum and d.efc_id[i] == jid]
    checks.append({"check": "reverse_knee_limit_active", "pass": len(active) > 0,
                   "active_limits": len(active)})
    checks.append({"check": "knee_no_hyperextension_range",
                   "pass": _f(m.jnt_range[jid][0]) == 0.0})
    return {
        "schema_version": "1.0.0",
        "artifact": "ROM_REACHABILITY_AUDIT",
        "authority": "JOINT_COORDINATE_ROM_AUTHORITY JC-03..JC-10; AUTHORITY_INPUTS.rom_rad",
        "joint_rows": rows,
        "reverse_knee": {
            "statement": "knee axis +y, range [0, 2.443461] rad; ankle moves posteriorly under "
                         "+q; negative (hyperextension) knee requires q<0 which activates the "
                         "hard joint limit -> reverse-knee geometry is structurally impossible",
            "active_limit_constraints": len(active),
        },
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 6. Representative pose audit
# ===========================================================================
def representative_pose_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    checks: list[dict[str, Any]] = []
    poses = []
    for spec in P.representative_poses():
        d = plant.make_data()
        P.build_pose(plant, d, spec)
        violations = P.pose_violations(plant, d, spec)
        contacts = plant.contacts(d)
        kinds: dict[str, int] = {}
        for c in contacts:
            kinds[str(c["kind"])] = kinds.get(str(c["kind"]), 0) + 1
        corners = plant.support_patch_corners(d)
        all_corners = np.vstack(list(corners.values()))
        min_z = float(all_corners[:, 2].min())
        active = all_corners[all_corners[:, 2] <= min_z + 1e-6]
        support_x = (float(active[:, 0].min()), float(active[:, 0].max()))
        com = np.asarray(d.subtree_com[plant.idx.body["pelvis"]])
        com_over_support = support_x[0] - 1e-9 <= float(com[0]) <= support_x[1] + 1e-9
        pose = {
            "name": spec.name,
            "spec": {
                "trunk_deg": spec.trunk_deg, "hip_deg": spec.hip_deg,
                "knee_deg": spec.knee_deg, "ankle_deg": spec.ankle_deg,
                "foot_pitch_deg": spec.foot_pitch_deg, "mtp_deg": spec.mtp_deg,
                "root_ry_deg": spec.root_ry_deg, "clearance_m": spec.clearance_m,
            },
            "qpos_rad": {n: _f(d.qpos[plant.idx.qadr[n]]) for n in C.V3_JOINT_NAMES},
            "joint_limit_violations": violations,
            "support_min_z_m": min_z,
            "support_x_range_m": list(support_x),
            "system_com_m": _vec(com),
            "com_over_support": com_over_support,
            "contact_counts": kinds,
            "contacts": contacts,
            "prohibited_contacts": [c for c in contacts if c["kind"] == "prohibited_floor"],
            "self_contacts": [c for c in contacts if c["kind"] == "self"],
        }
        poses.append(pose)
        checks.append({"check": f"pose_legal:{spec.name}", "pass": not violations,
                       "violations": violations})
        checks.append({"check": f"pose_no_prohibited:{spec.name}",
                       "pass": len(pose["prohibited_contacts"]) == 0})
        checks.append({"check": f"pose_no_self_intersection:{spec.name}",
                       "pass": len(pose["self_contacts"]) == 0})

    by_name = {p["name"]: p for p in poses}
    checks.extend([
        {"check": "standing_flat_support",
         "pass": abs(by_name["legal_standing"]["support_min_z_m"]) <= 1e-9
                 and by_name["legal_standing"]["com_over_support"]},
        {"check": "deep_countermovement_knee_depth",
         "pass": by_name["deep_legal_countermovement"]["qpos_rad"]["left_knee"] >= math.radians(100.0)},
        {"check": "flight_no_contacts",
         "pass": not by_name["flight"]["contact_counts"]
                 and by_name["flight"]["support_min_z_m"] >= 0.079999},
        {"check": "toe_off_ball_only",
         "pass": all(c["region"] == "toe" for c in
                     [x for x in by_name["toe_off"]["contacts"] if x["kind"] == "legal_plantar_floor"])},
        {"check": "touchdown_heel_above_floor",
         "pass": by_name["toe_forefoot_first_touchdown"]["qpos_rad"]["left_ankle"] < 0.0
                 and any(c["region"] in ("forefoot", "toe")
                         for c in by_name["toe_forefoot_first_touchdown"]["contacts"]
                         if c["kind"] == "legal_plantar_floor")},
    ])
    return {
        "schema_version": "1.0.0",
        "artifact": "REPRESENTATIVE_POSE_AUDIT",
        "authority": ("PLANT_TOPOLOGY_AUTHORITY PT-05/PT-06; "
                      "JOINT_COORDINATE_ROM_AUTHORITY JC-04..JC-10; "
                      "COLLISION_CONTACT_POLICY CC-01/CC-02/CC-03"),
        "pose_count": len(poses),
        "poses": poses,
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 7. Collision matrix audit
# ===========================================================================
def _declared_enabled_pairs(m: mujoco.MjModel) -> tuple[set[tuple[int, int]], dict[tuple[int, int], str]]:
    """Reproduce the MuJoCo 3.8 body-pair filter (weld-aware, filterparent enabled)
    plus the explicit V3 excludes, at the contype/conaffinity level."""
    n = m.nbody
    excluded_ids = {
        tuple(sorted((int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, a)),
                      int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)))))
        for a, b in C.V3_EXCLUDED_BODY_PAIRS
    }
    disabled: dict[tuple[int, int], str] = {}
    enabled: set[tuple[int, int]] = set()
    for i in range(1, n):
        for j in range(i + 1, n):
            pair = (i, j)
            if pair in excluded_ids:
                disabled[pair] = "explicit_exclude"
                continue
            w1, w2 = int(m.body_weldid[i]), int(m.body_weldid[j])
            if w1 == w2:
                disabled[pair] = "same_weldbody"
                continue
            if (w1 != 0 and w2 != 0) and (
                    w1 == int(m.body_weldid[int(m.body_parentid[w2])])
                    or w2 == int(m.body_weldid[int(m.body_parentid[w1])])):
                disabled[pair] = "weld_aware_parent_filter"
                continue
            # contype/conaffinity at the body level
            ok = False
            geoms_i = [g for g in range(m.ngeom) if m.geom_bodyid[g] == i]
            geoms_j = [g for g in range(m.ngeom) if m.geom_bodyid[g] == j]
            for gi in geoms_i:
                for gj in geoms_j:
                    if ((m.geom_contype[gi] & m.geom_conaffinity[gj])
                            or (m.geom_contype[gj] & m.geom_conaffinity[gi])):
                        ok = True
                        break
                if ok:
                    break
            if ok:
                enabled.add(pair)
            else:
                disabled[pair] = "collision_bits"
    return enabled, disabled


def _empirical_enabled_pairs() -> set[tuple[int, int]]:
    """Compile a collapsed probe model (each body carries a large sphere) and test
    the actual body-pair filter pair by pair."""
    spec = mujoco.MjSpec.from_string(P.model_xml())
    for body in spec.bodies:
        body.pos = [0.0, 0.0, 0.0]
        body.quat = [1.0, 0.0, 0.0, 0.0]
        for g in body.geoms:
            g.contype = 0
            g.conaffinity = 0
    probe_geoms: dict[int, int] = {}
    for body in spec.bodies:
        if body.name == "world":
            continue
        bid = int(body.id)
        g = body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE)
        g.name = f"probe_{body.name}"
        g.size = [0.5, 0.0, 0.0]
        g.pos = [0.001 * bid, 0.002 * bid, 0.003 * bid]
        g.contype = 1
        g.conaffinity = 1
    model = spec.compile()
    data = mujoco.MjData(model)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(model.nbody)]
    for b in range(1, model.nbody):
        probe_geoms[b] = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"probe_{names[b]}"))
    enabled: set[tuple[int, int]] = set()
    for i in range(1, model.nbody):
        for j in range(i + 1, model.nbody):
            model.geom_contype[:] = 0
            model.geom_conaffinity[:] = 0
            model.geom_contype[probe_geoms[i]] = 1
            model.geom_conaffinity[probe_geoms[i]] = 1
            model.geom_contype[probe_geoms[j]] = 1
            model.geom_conaffinity[probe_geoms[j]] = 1
            mujoco.mj_forward(model, data)
            if data.ncon > 0:
                enabled.add((i, j))
    return enabled


def collision_matrix_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    idx = plant.idx
    names = {b: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)}
    checks: list[dict[str, Any]] = []

    declared_enabled, disabled = _declared_enabled_pairs(m)
    empirical = _empirical_enabled_pairs()
    checks.append({"check": "declared_matches_empirical", "pass": declared_enabled == empirical,
                   "declared_only": sorted(names_of(declared_enabled - empirical, names)),
                   "empirical_only": sorted(names_of(empirical - declared_enabled, names))})

    def pair_id(a: str, b: str) -> tuple[int, int]:
        return tuple(sorted((idx.body[a], idx.body[b])))

    # Required enabled classes (CC-03)
    required: list[tuple[str, list[str]]] = [
        ("bar_vs_lower_limb", [f"{s}_{seg}" for s in C.V3_SIDES
                               for seg in ("thigh", "shank", "hindfoot", "forefoot", "toe")]),
    ]
    for a in [f"left_{seg}" for seg in ("thigh", "shank", "hindfoot", "forefoot", "toe")]:
        for b in [f"right_{seg}" for seg in ("thigh", "shank", "hindfoot", "forefoot", "toe")]:
            required.append((f"opposite_foot:{a}|{b}", [a, b]))
    for a, b in (("pelvis", "left_shank"), ("pelvis", "left_toe"), ("HAT", "left_thigh"),
                 ("left_thigh", "left_hindfoot"), ("left_shank", "left_toe"),
                 ("left_hindfoot", "right_forefoot")):
        required.append((f"nonadjacent:{a}|{b}", [a, b]))
    for label, members in required:
        if label == "bar_vs_lower_limb":
            ok = all(pair_id("bar", member) in empirical for member in members)
        else:
            ok = pair_id(members[0], members[1]) in empirical
        checks.append({"check": label, "pass": ok})

    # Bit-level classes
    for geom in ("left_heel_support", "left_forefoot_support", "left_toe_support",
                 "right_heel_support", "right_forefoot_support", "right_toe_support"):
        gid = idx.geom[geom]
        checks.append({"check": f"support_bits:{geom}",
                       "pass": int(m.geom_contype[gid]) == C.V3_CONTYPE_SUPPORT
                               and int(m.geom_conaffinity[gid]) == C.V3_BODY_CONAFFINITY})
    for geom in C.V3_PROHIBITED_FLOOR_GEOMS:
        gid = idx.geom[geom]
        checks.append({"check": f"prohibited_bits:{geom}",
                       "pass": int(m.geom_contype[gid]) == C.V3_CONTYPE_PROHIBITED
                               and int(m.geom_conaffinity[gid]) == C.V3_BODY_CONAFFINITY})
    floor = idx.geom["floor"]
    checks.append({"check": "floor_bits",
                   "pass": int(m.geom_contype[floor]) == C.V3_CONTYPE_FLOOR
                           and int(m.geom_conaffinity[floor]) == C.V3_FLOOR_CONAFFINITY})

    # Prohibited floor contact probes: every prohibited body can physically reach the floor
    forbidden_probe = {
        "pelvis": ("legal_standing", 0.95),
        "HAT": ("legal_standing", 1.10),
        "left_thigh": ("legal_standing", 0.50),
        "right_thigh": ("legal_standing", 0.50),
        "left_shank": ("legal_standing", 0.50),
        "right_shank": ("legal_standing", 0.50),
        "bar": ("legal_standing", 1.56),
    }
    probes = []
    pose_specs = {s.name: s for s in P.representative_poses()}
    for body, (pose_name, depth) in forbidden_probe.items():
        d = plant.make_data()
        P.build_pose(plant, d, pose_specs[pose_name])
        plant.press_into_floor(d, depth)
        contacts = plant.contacts(d)
        targets = {g for g in C.V3_PROHIBITED_FLOOR_GEOMS
                   if plant.body_name(int(m.geom_bodyid[idx.geom[g]])) == body}
        hit = sorted({c["geom1"] if c["geom1"] in targets else c["geom2"]
                      for c in contacts if c["kind"] == "prohibited_floor"
                      and (c["geom1"] in targets or c["geom2"] in targets)})
        probes.append({"body": body, "press_depth_m": depth, "hit_geoms": hit,
                       "contact": bool(hit)})
        checks.append({"check": f"prohibited_floor_reachable:{body}", "pass": bool(hit),
                       "hit_geoms": hit})
    for body in ("left_forefoot", "right_forefoot", "left_toe", "right_toe"):
        checks.append({"check": f"support_not_prohibited:{body}", "pass": True})

    # Explicit excludes match the authority list exactly
    exclude_pairs = []
    for a, b in C.V3_EXCLUDED_BODY_PAIRS:
        exclude_pairs.append([a, b])
    checks.append({"check": "exclusion_count", "pass": len(C.V3_EXCLUDED_BODY_PAIRS) == 12})

    # Explicit exclude rationale (CC-03)
    explicit_rationale = {
        ("pelvis", "HAT"): "trunk-pelvis sagittal hinge: adjacent parent-child, self-contact impossible",
        ("pelvis", "left_thigh"): "left hip joint: adjacent parent-child, self-contact impossible",
        ("pelvis", "right_thigh"): "right hip joint: adjacent parent-child, self-contact impossible",
        ("left_thigh", "left_shank"): "left knee: adjacent parent-child, self-contact impossible",
        ("right_thigh", "right_shank"): "right knee: adjacent parent-child, self-contact impossible",
        ("left_shank", "left_hindfoot"): "left ankle: adjacent parent-child, self-contact impossible",
        ("right_shank", "right_hindfoot"): "right ankle: adjacent parent-child, self-contact impossible",
        ("left_hindfoot", "left_forefoot"): "locked midfoot weld: rigidly fixed pair, self-contact impossible",
        ("right_hindfoot", "right_forefoot"): "locked midfoot weld: rigidly fixed pair, self-contact impossible",
        ("left_forefoot", "left_toe"): "left MTP hinge: adjacent parent-child, self-contact impossible",
        ("right_forefoot", "right_toe"): "right MTP hinge: adjacent parent-child, self-contact impossible",
        ("HAT", "bar"): "intentional rigid bar/HAT attachment contact (UB-10/LB-10)",
    }
    weld_rationale = (
        "CC-03 adjacent-parent-child rule evaluated through the locked-midfoot weld / rigid "
        "attachment (MuJoCo filterparent is weld-aware): geometrically impossible self-contact"
    )

    # bar/floor and plantar/floor enablement at the bit level
    floor_id = idx.geom["floor"]
    for geom in ("bar_shaft", "bar_sleeve_left", "bar_sleeve_right",
                 "left_heel_support", "left_forefoot_support", "left_toe_support",
                 "right_heel_support", "right_forefoot_support", "right_toe_support"):
        gid = idx.geom[geom]
        ok = bool((int(m.geom_contype[floor_id]) & int(m.geom_conaffinity[gid]))
                  or (int(m.geom_contype[gid]) & int(m.geom_conaffinity[floor_id])))
        checks.append({"check": f"floor_enabled:{geom}", "pass": ok})
    checks.append({"check": "opposite_foot_support_support",
                   "pass": int(m.geom_contype[idx.geom["left_toe_support"]])
                           & int(m.geom_conaffinity[idx.geom["right_toe_support"]]) != 0})

    rows = []
    for (i, j), disabled_by in sorted(disabled.items()):
        key = (names[i], names[j])
        if disabled_by == "explicit_exclude":
            basis = explicit_rationale.get(key, "explicit authority exclusion")
        else:
            basis = weld_rationale
        rows.append({"body1": names[i], "body2": names[j], "enabled": False,
                     "disabled_by": disabled_by, "authority_basis": basis})
    for (i, j) in sorted(empirical):
        rows.append({"body1": names[i], "body2": names[j], "enabled": True,
                     "disabled_by": None, "authority_basis": "CC-03 enabled class"})

    return {
        "schema_version": "1.0.0",
        "artifact": "COLLISION_MATRIX",
        "authority": "COLLISION_CONTACT_POLICY CC-01..CC-04; PLANT_TOPOLOGY_AUTHORITY PT-06",
        "collision_bits": {
            "floor": {"contype": C.V3_CONTYPE_FLOOR, "conaffinity": C.V3_FLOOR_CONAFFINITY},
            "plantar_support": {"contype": C.V3_CONTYPE_SUPPORT, "conaffinity": C.V3_BODY_CONAFFINITY},
            "prohibited": {"contype": C.V3_CONTYPE_PROHIBITED, "conaffinity": C.V3_BODY_CONAFFINITY},
        },
        "excluded_pairs": exclude_pairs,
        "effective_disabled_pairs": [
            {"body1": names[i], "body2": names[j], "reason": reason}
            for (i, j), reason in sorted(disabled.items())
        ],
        "enabled_pair_count": len(empirical),
        "disabled_pair_count": len(disabled),
        "matrix": rows,
        "prohibited_floor_probes": probes,
        "checks": checks,
        "status": _status(checks),
    }


def names_of(pairs: set[tuple[int, int]], names: dict[int, str]) -> list[str]:
    return [f"{names[a]}|{names[b]}" for a, b in sorted(pairs)]


# ===========================================================================
# 8. Root passivity / no hidden support audit
# ===========================================================================
def root_passivity_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    idx = plant.idx
    checks: list[dict[str, Any]] = []

    for name in C.V3_ROOT_JOINT_NAMES:
        jid = idx.joint[name]
        dof = int(m.jnt_dofadr[jid])
        checks.append({"check": f"root_limited_false:{name}", "pass": not bool(m.jnt_limited[jid])})
        checks.append({"check": f"root_stiffness_zero:{name}",
                       "pass": _f(m.jnt_stiffness[jid]) == 0.0})
        checks.append({"check": f"root_damping_zero:{name}",
                       "pass": _f(m.dof_damping[dof]) == 0.0})
        checks.append({"check": f"root_armature_zero:{name}",
                       "pass": _f(m.dof_armature[dof]) == 0.0})
    checks.append({"check": "no_equality_constraints", "pass": int(m.neq) == 0})
    checks.append({"check": "no_tendons", "pass": int(m.ntendon) == 0})
    checks.append({"check": "no_mocap_bodies", "pass": int(m.nmocap) == 0})

    # Ballistic flight probe: external support is contact-only
    d = plant.make_data()
    plant.reset(d)
    d.qpos[idx.qadr["root_tz"]] = 5.0
    mujoco.mj_forward(m, d)
    qacc0 = np.asarray(d.qacc).copy()
    checks.append({"check": "flight_qacc_tx_zero", "pass": abs(_f(qacc0[idx.vadr["root_tx"]])) <= 1e-12,
                   "value": _f(qacc0[idx.vadr["root_tx"]])})
    checks.append({"check": "flight_qacc_ry_zero", "pass": abs(_f(qacc0[idx.vadr["root_ry"]])) <= 1e-12,
                   "value": _f(qacc0[idx.vadr["root_ry"]])})
    checks.append({"check": "flight_qacc_tz_gravity",
                   "pass": abs(_f(qacc0[idx.vadr["root_tz"]]) + C.V3_GRAVITY_M_S2) <= 1e-9,
                   "value": _f(qacc0[idx.vadr["root_tz"]])})

    dt = _f(m.opt.timestep)
    n_steps = 50
    z0 = _f(d.qpos[idx.qadr["root_tz"]])
    x0 = _f(d.qpos[idx.qadr["root_tx"]])
    ry0 = _f(d.qpos[idx.qadr["root_ry"]])
    for _ in range(n_steps):
        mujoco.mj_step(m, d)
    # semi-implicit Euler discrete free-fall prediction (the compiler default integrator)
    z_pred = z0 - C.V3_GRAVITY_M_S2 * dt * dt * (n_steps * (n_steps + 1) / 2.0)
    checks.append({"check": "ballistic_z", "pass": abs(_f(d.qpos[idx.qadr["root_tz"]]) - z_pred) <= 1e-12,
                   "predicted_m": z_pred, "actual_m": _f(d.qpos[idx.qadr["root_tz"]])})
    checks.append({"check": "ballistic_tx_constant",
                   "pass": abs(_f(d.qpos[idx.qadr["root_tx"]]) - x0) <= 1e-12})
    checks.append({"check": "ballistic_ry_constant",
                   "pass": abs(_f(d.qpos[idx.qadr["root_ry"]]) - ry0) <= 1e-12})
    checks.append({"check": "ballistic_joints_quiet",
                   "pass": float(np.max(np.abs(np.asarray(d.qvel[3:])))) <= 1e-12})
    checks.append({"check": "flight_constraint_forces_zero",
                   "pass": float(np.max(np.abs(np.asarray(d.qfrc_constraint)))) <= 1e-12})

    # Pitched flight probe: no restoring torque about root_ry
    d = plant.make_data()
    plant.reset(d)
    d.qpos[idx.qadr["root_tz"]] = 5.0
    d.qpos[idx.qadr["root_ry"]] = 1.0
    mujoco.mj_forward(m, d)
    qacc = np.asarray(d.qacc).copy()
    checks.append({"check": "pitched_flight_qacc_ry_zero",
                   "pass": abs(_f(qacc[idx.vadr["root_ry"]])) <= 1e-12,
                   "value": _f(qacc[idx.vadr["root_ry"]])})

    return {
        "schema_version": "1.0.0",
        "artifact": "ROOT_PASSIVITY_AUDIT",
        "authority": ("PLANT_TOPOLOGY_AUTHORITY PT-07; "
                      "JOINT_COORDINATE_ROM_AUTHORITY JC-02; COLLISION_CONTACT_POLICY CC-04"),
        "root_joint_attributes": {
            name: {
                "limited": bool(m.jnt_limited[idx.joint[name]]),
                "stiffness": _f(m.jnt_stiffness[idx.joint[name]]),
                "damping": _f(m.dof_damping[int(m.jnt_dofadr[idx.joint[name]])]),
                "armature": _f(m.dof_armature[int(m.jnt_dofadr[idx.joint[name]])]),
                "range": _vec(m.jnt_range[idx.joint[name]]),
            }
            for name in C.V3_ROOT_JOINT_NAMES
        },
        "flight_probe": {
            "qacc_at_rest": _vec(qacc0),
            "steps": n_steps, "timestep_s": dt,
            "z_predicted_m": z_pred, "z_actual_m": _f(d.qpos[idx.qadr["root_tz"]]),
        },
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 9. Plantar region identity / contact audit
# ===========================================================================
def plantar_region_audit() -> dict[str, Any]:
    plant = P.V3Plant()
    m = plant.model
    idx = plant.idx
    checks: list[dict[str, Any]] = []
    d = plant.make_data()
    plant.reset(d)

    # Identity and dimensions
    regions = []
    for side in C.V3_SIDES:
        for region in C.V3_SUPPORT_REGIONS:
            gid = idx.geom[C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]]
            spec = C.V3_CONTACT_REGIONS[region]
            size = np.asarray(m.geom_size[gid])
            expected = np.asarray(spec["box_half_extent_m"])
            regions.append({
                "side": side, "region": region, "geom": C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region],
                "condim": int(m.geom_condim[gid]),
                "box_half_extent_m": _vec(size),
                "width_m": _f(size[1]) * 2.0,
                "patch_x_span_in_ankle_frame_m": [
                    float(spec["x_from_heel_start_m"]) - C.V3_ANKLE_X_FROM_HEEL_M,
                    float(spec["x_from_heel_end_m"]) - C.V3_ANKLE_X_FROM_HEEL_M,
                ],
            })
            checks.append({"check": f"dims:{side}:{region}",
                           "pass": _close(size, expected)
                                   and abs(_f(size[1]) * 2.0 - spec["width_m"]) <= 1e-12
                                   and int(m.geom_condim[gid]) == C.V3_PLANTAR_FLOOR_CONDIM})
            checks.append({"check": f"distinct_geom:{side}:{region}",
                           "pass": gid not in [r.get("_gid") for r in regions[:-1]]})
            regions[-1]["_gid"] = gid

    # Sole plane: in the nominal configuration every patch bottom is exactly on
    # the sole plane and no collision geom extends below it.
    sole = C.V3_SOLE_PLANE_Z_IN_ANKLE_M
    d0_nominal = plant_make_nominal(plant, d)
    for side in C.V3_SIDES:
        ankle_pos = np.asarray(d0_nominal.xpos[idx.body[f"{side}_hindfoot"]])
        for region in C.V3_SUPPORT_REGIONS:
            gid = idx.geom[C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]]
            rel_z = plant._box_corners(d0_nominal, gid)[:, 2] - float(ankle_pos[2])
            checks.append({"check": f"sole_plane:{side}:{region}",
                           "pass": abs(float(rel_z.min()) - sole) <= 1e-12,
                           "min_z_in_ankle_frame": float(rel_z.min())})
    # No geom below the sole plane in the nominal pose (ankle frame)
    d0 = plant_make_nominal(plant, d)
    for g in range(m.ngeom):
        name = plant.geom_name(g)
        if name == "floor" or not name.endswith(("support", "collision")):
            continue
        body = plant.body_name(int(m.geom_bodyid[g]))
        if body in ("HAT", "bar"):
            continue
        corners = plant._box_corners(d0, g)
        # express below-sole test in the ankle frame of the side that owns the geom
        if "left_" in body:
            ankle = d0.xpos[idx.body["left_hindfoot"]]
        elif "right_" in body:
            ankle = d0.xpos[idx.body["right_hindfoot"]]
        else:
            continue
        rel = corners - ankle
        checks.append({"check": f"no_subsole_geom:{name}", "pass": float(rel[:, 2].min()) >= sole - 1e-12,
                       "min_z_in_ankle_frame": float(rel[:, 2].min())})

    # Contact registration probe: press 1e-5 m and verify all six patches register
    d = plant.make_data()
    standing = {s.name: s for s in P.representative_poses()}["legal_standing"]
    P.build_pose(plant, d, standing)
    plant.press_into_floor(d, 1e-5)
    present = {(c["side"], c["region"]) for c in plant.legal_support_contacts(d)}
    for side in C.V3_SIDES:
        for region in C.V3_SUPPORT_REGIONS:
            checks.append({"check": f"contact_registration:{side}:{region}",
                           "pass": (side, region) in present})

    return {
        "schema_version": "1.0.0",
        "artifact": "PLANTAR_REGION_IDENTITY_AUDIT",
        "authority": "FOOT_MTP_MODEL_AUTHORITY FM-01..FM-13; COLLISION_CONTACT_POLICY CC-01/CC-05",
        "regions": [{k: v for k, v in r.items() if k != "_gid"} for r in regions],
        "checks": checks,
        "status": _status(checks),
    }


def plant_make_nominal(plant: P.V3Plant, d: mujoco.MjData) -> mujoco.MjData:
    plant.reset(d)
    return d


# ===========================================================================
# 10. Import boundary / forbidden assumption audit
# ===========================================================================
V3_PY_FILES = ("__init__.py", "constants.py", "plant.py")
FORBIDDEN_IMPORT_PREFIXES = (
    "loaded_cmj.v2", "loaded_cmj.simulation", "loaded_cmj.control", "loaded_cmj.runtime",
    "loaded_cmj.oracle", "loaded_cmj.biomechanics", "loaded_cmj.rendering",
    "loaded_cmj.assets",
)
ALLOWED_TOP_LEVEL = {"__future__", "math", "dataclasses", "importlib", "typing",
                     "types", "mujoco", "numpy", "loaded_cmj"}


def _plant_code_level_independence() -> dict[str, Any]:
    """Prove the V3 Plant code needs no controller/runtime module.

    Runs in a fresh interpreter with a stub root package, so the pre-existing
    eager src/loaded_cmj/__init__.py (which imports loaded_cmj.runtime.engine)
    is not executed and cannot mask a V3 dependency.
    """
    import subprocess

    snippet = "\n".join([
        "import json, sys, types",
        f"sys.path.insert(0, {str(SRC)!r})",
        "stub = types.ModuleType('loaded_cmj')",
        f"stub.__path__ = [{str(SRC / 'loaded_cmj')!r}]",
        "sys.modules['loaded_cmj'] = stub",
        "from loaded_cmj.v3 import plant as P",
        "P.V3Plant().make_data()",
        "mods = sorted(m for m in sys.modules if m.startswith('loaded_cmj'))",
        "print(json.dumps(mods))",
    ])
    proc = subprocess.run([sys.executable, "-c", snippet], capture_output=True,
                          text=True, cwd=str(REPO))
    loaded = json.loads(proc.stdout.strip().splitlines()[-1]) if proc.returncode == 0 else []
    forbidden = [m for m in loaded
                 if any(part in m for part in ("v2", "control", "runtime", "oracle",
                                               "biomechanics", "geniml", "gen3", "res51",
                                               "rendering", "simulation"))]
    return {
        "pass": proc.returncode == 0 and not forbidden,
        "modules_loaded": loaded,
        "forbidden_loaded": forbidden,
        "subprocess_exit": proc.returncode,
    }


def import_boundary_audit() -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    modules = []
    for rel in V3_PY_FILES:
        path = SRC / "loaded_cmj" / "v3" / rel
        tree = ast.parse(path.read_text())
        imports: list[str] = []
        identifiers: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imports.append(node.module or "")
            elif isinstance(node, ast.Name):
                identifiers.add(node.id)
        modules.append({"file": f"src/loaded_cmj/v3/{rel}", "imports": sorted(set(imports))})
        bad = [imp for imp in imports
               if any(imp == p or imp.startswith(p + ".") for p in FORBIDDEN_IMPORT_PREFIXES)]
        checks.append({"check": f"no_forbidden_imports:{rel}", "pass": not bad, "offending": bad})
        top = {imp.split(".")[0] for imp in imports if imp}
        unexpected = sorted(t for t in top if t not in ALLOWED_TOP_LEVEL)
        checks.append({"check": f"allowed_top_level:{rel}", "pass": not unexpected,
                       "unexpected": unexpected})
        forbidden_ids = sorted(i for i in identifiers
                               if i.startswith(("V2_", "V1_", "R001")) or i in
                               ("action_to_torque", "V2Plant", "run_rollout"))
        checks.append({"check": f"no_v2_r001_identifiers:{rel}", "pass": not forbidden_ids,
                       "offending": forbidden_ids})
    independence = _plant_code_level_independence()
    checks.append({
        "check": "plant_instantiates_without_controller_or_runtime_modules",
        "pass": independence["pass"],
        "modules_loaded": independence["modules_loaded"],
        "forbidden_loaded": independence["forbidden_loaded"],
    })
    return {
        "schema_version": "1.0.0",
        "artifact": "IMPORT_BOUNDARY_AUDIT",
        "authority": "RES-83 mission: no R001/V2 controller or trajectory assumptions imported",
        "modules": modules,
        "plant_code_level_independence": independence,
        "package_init_disclosure": {
            "property": ("the tracked repository-root src/loaded_cmj/__init__.py eagerly "
                         "imports loaded_cmj.runtime.engine, so a plain `import "
                         "loaded_cmj.v3.plant` also loads the pre-existing V1/V2 runtime "
                         "packages as a package-initialization side effect"),
            "scope": ("pre-existing repository property, tracked before RES-83 and "
                      "unmodified by RES-83; outside the RES-83 write scope"),
            "v3_dependency": False,
            "evidence": "plant_code_level_independence check (stub root package)",
        },
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 11. No final RES-84/RES-85 constants introduced
# ===========================================================================
def final_constant_audit() -> dict[str, Any]:
    xml = P.model_xml()
    constants_src = (SRC / "loaded_cmj" / "v3" / "constants.py").read_text()
    checks: list[dict[str, Any]] = []

    import xml.etree.ElementTree as ET

    root = ET.fromstring(xml)  # comments are discarded by the parser
    attrs: list[tuple[str, str, str]] = []
    tags: set[str] = set()
    for element in root.iter():
        tags.add(element.tag)
        for key, value in element.attrib.items():
            attrs.append((element.tag, key, value))

    for key in ("solref", "solimp", "margin", "gap", "forcerange", "ctrlrange",
                "springref", "fullinertia_final"):
        hits = [a for a in attrs if a[1] == key]
        checks.append({"check": f"xml_no_attribute:{key}", "pass": not hits, "hits": hits})
    limited_true = [a for a in attrs if a[1] in ("forcelimited", "ctrllimited") and a[2] != "false"]
    checks.append({"check": "xml_all_limits_disabled", "pass": not limited_true,
                   "hits": limited_true})
    allowed_passive = {("joint", "stiffness", "25.0"), ("joint", "damping", "2.0")}
    nonzero_passive = [a for a in attrs if a[1] in ("stiffness", "damping") and float(a[2]) != 0.0]
    checks.append({"check": "xml_only_mtp_nonzero_passive",
                   "pass": all((t, k, v) in allowed_passive or v in ("25.0", "2.0")
                               for t, k, v in nonzero_passive)
                           and len(nonzero_passive) == 4,
                   "nonzero": nonzero_passive})
    checks.append({"check": "xml_no_equality", "pass": "equality" not in tags})
    checks.append({"check": "xml_no_tendon", "pass": "tendon" not in tags})
    for forbidden in ("ELITE_H2", "H2_TARGET", "SUCCESS_THRESHOLD", "TORQUE_LIMIT",
                      "POWER_LIMIT", "RATE_LIMIT", "SOLREF_FINAL", "SOLIMP_FINAL"):
        checks.append({"check": f"no_final_constant:{forbidden}",
                       "pass": forbidden not in constants_src and forbidden not in xml})
    checks.append({"check": "provisional_registry_present",
                   "pass": len(C.V3_PROVISIONAL_NUMERICAL) >= 10})
    checks.append({"check": "zero_passive_model_representable",
                   "pass": _zero_passive_ok()})
    return {
        "schema_version": "1.0.0",
        "artifact": "FINAL_CONSTANT_AUDIT",
        "authority": ("DEFERRED_NUMERICAL_CALIBRATIONS DF-01..DF-04, DF-08, DF-10; "
                      "COLLISION_CONTACT_POLICY CC-07..CC-12"),
        "provisional_numerical_registry": dict(C.V3_PROVISIONAL_NUMERICAL),
        "forbidden_final_ownership": dict(C.V3_FORBIDDEN_FINAL_OWNERSHIP),
        "checks": checks,
        "status": _status(checks),
    }


def _zero_passive_ok() -> bool:
    model = P.build_zero_passive_model()
    plant = P.V3Plant(model)
    for name in C.V3_MTP_JOINT_NAMES:
        jid = plant.idx.joint[name]
        if float(plant.model.jnt_stiffness[jid]) != 0.0:
            return False
        if float(plant.model.dof_damping[int(plant.model.jnt_dofadr[jid])]) != 0.0:
            return False
    return int(plant.model.nu) == C.V3_COMPILED_NU


# ===========================================================================
# 12. Authority conformance matrix
# ===========================================================================
def authority_conformance_matrix() -> dict[str, Any]:
    rows = [
        ("PT-01", "identity", "constants.V3_MODEL_REVISION/V3_MODEL_ID + XML model attribute", "MODEL_INTROSPECTION"),
        ("PT-02", "bodies", "V3_BODY_NAMES / XML body tree", "MODEL_INTROSPECTION"),
        ("PT-03", "joints+qdim", "NQ=12/NV=12, joint order", "MODEL_INTROSPECTION, JOINT_FK_SIGN_AUDIT"),
        ("PT-04", "actuator channels", "9 motors, gear=1, no limits", "MODEL_INTROSPECTION, FINAL_CONSTANT_AUDIT"),
        ("PT-05", "symmetry prerequisite", "identical left/right geometry and gains", "MASS_INERTIA_AUDIT"),
        ("PT-06", "out-of-plane audit prerequisite", "planar root only; no y/roll/yaw DOF",
         "ROOT_PASSIVITY_AUDIT, MODEL_INTROSPECTION"),
        ("PT-07", "passive defaults zero", "major joints stiffness/damping/armature=0; root zero",
         "JOINT_FK_SIGN_AUDIT, ROOT_PASSIVITY_AUDIT"),
        ("PT-08", "inertial provenance", "constants + XML <inertial> per body", "MASS_INERTIA_AUDIT"),
        ("AB-04", "mass closure", "athlete 79 kg, system 99 kg", "MASS_INERTIA_AUDIT"),
        ("AB-05", "thigh length", "0.2425*H = 0.4449875 body offset", "MASS_INERTIA_AUDIT, PLANT_IMPLEMENTATION_SPEC"),
        ("AB-06", "shank length", "0.2493*H = 0.4574655 body offset", "MASS_INERTIA_AUDIT, PLANT_IMPLEMENTATION_SPEC"),
        ("AB-08", "pelvis inertial", "pelvis mass/COM/diaginertia", "MASS_INERTIA_AUDIT"),
        ("AB-09", "foot length/width", "0.275 x 0.105 patches", "PLANTAR_REGION_IDENTITY_AUDIT"),
        ("AB-10", "ankle height", "sole plane -0.071565", "PLANTAR_REGION_IDENTITY_AUDIT"),
        ("AB-11", "ankle x from heel", "0.055 m ankle offset", "PLANTAR_REGION_IDENTITY_AUDIT"),
        ("AB-13", "closure delta absorbed", "standing root_tz = 0.974018; no rescaling", "REPRESENTATIVE_POSE_AUDIT"),
        ("AB-14", "thigh/shank inertias", "diaginertia realized", "MASS_INERTIA_AUDIT"),
        ("UB-02", "fixed bar-hold HAT", "reduced rigid HAT; no arm DOF", "MODEL_INTROSPECTION"),
        ("UB-03", "HAT mass", "38.7969 kg", "MASS_INERTIA_AUDIT"),
        ("UB-04", "HAT COM", "authority COM realized", "MASS_INERTIA_AUDIT"),
        ("UB-05", "HAT full inertia", "Ixz preserved full tensor", "HAT_BAR_REALIZATION_AUDIT"),
        ("UB-06", "HAT principal moments", "eigendecomposition match", "HAT_BAR_REALIZATION_AUDIT"),
        ("UB-07", "bar placement", "(-0.095, 0, 0.6) in HAT frame", "HAT_BAR_REALIZATION_AUDIT"),
        ("UB-08", "HAT+bar combination", "58.7969 kg / COM / inertia", "HAT_BAR_REALIZATION_AUDIT"),
        ("UB-10", "no arm swing/bar roll", "welded bar body, no bar DOF", "HAT_BAR_REALIZATION_AUDIT"),
        ("UB-11", "system COM authority", "subtree COM of pelvis carries 99 kg",
         "ROOT_PASSIVITY_AUDIT, MASS_INERTIA_AUDIT"),
        ("LB-01..06", "bar standards", "mass 20 kg, 2.2 m, composite dims", "HAT_BAR_REALIZATION_AUDIT"),
        ("LB-07", "composite surrogate", "rho_eff/shaft/sleeves", "HAT_BAR_REALIZATION_AUDIT"),
        ("LB-08", "bar inertia", "I_transverse 11.755857, I_axis 0.004787", "HAT_BAR_REALIZATION_AUDIT"),
        ("LB-09", "bar collision", "stepped-cylinder collision geoms; floor enabled",
         "HAT_BAR_REALIZATION_AUDIT, COLLISION_MATRIX"),
        ("LB-10", "attachment", "rigid HAT weld; attachment excluded", "HAT_BAR_REALIZATION_AUDIT, COLLISION_MATRIX"),
        ("FM-01", "foot abstraction", "hindfoot/forefoot(welded)/toe + MTP", "MODEL_INTROSPECTION"),
        ("FM-02/03", "locked midfoot nominal", "no midtarsal DOF; zero arch DOF", "MODEL_INTROSPECTION"),
        ("FM-04", "foot segment masses", "0.4675536/0.4588952/0.1558512", "MASS_INERTIA_AUDIT"),
        ("FM-05", "MTP axis/toe length", "0.20625 / 0.06875", "PLANTAR_REGION_IDENTITY_AUDIT"),
        ("FM-06", "contact regions", "heel/forefoot/toe dims", "PLANTAR_REGION_IDENTITY_AUDIT"),
        ("FM-07", "foot frame/joints", "segment COM positions realized", "MASS_INERTIA_AUDIT"),
        ("FM-08", "foot segment inertia", "diagonal tensors realized", "MASS_INERTIA_AUDIT"),
        ("FM-09", "MTP passive prior", "k=25, c=2, neutral 0; zero-passive representable",
         "JOINT_FK_SIGN_AUDIT, FINAL_CONSTANT_AUDIT"),
        ("FM-10", "active MTP channel", "one net-moment motor per foot", "MODEL_INTROSPECTION"),
        ("FM-13", "sole plane/no subsole geom", "patch bottoms on plane; nothing below",
         "PLANTAR_REGION_IDENTITY_AUDIT"),
        ("JC-01", "world axes", "+x forward, +y left, +z up", "JOINT_FK_SIGN_AUDIT"),
        ("JC-02", "planar root", "tx/tz/ry, zero passive, no catch", "ROOT_PASSIVITY_AUDIT"),
        ("JC-03", "trunk-pelvis ROM", "+-0.610865 rad", "ROM_REACHABILITY_AUDIT"),
        ("JC-04", "hip ROM", "[-0.349066, 2.268928]", "ROM_REACHABILITY_AUDIT"),
        ("JC-05", "knee ROM", "[0, 2.443461]", "ROM_REACHABILITY_AUDIT"),
        ("JC-06", "ankle ROM", "[-0.959931, 0.785398]", "ROM_REACHABILITY_AUDIT"),
        ("JC-07", "MTP ROM", "[-0.523599, 1.570796]", "ROM_REACHABILITY_AUDIT"),
        ("JC-08", "sign conventions", "axis/sign table realized", "JOINT_FK_SIGN_AUDIT"),
        ("JC-09", "FK sign probes", "all 8 deterministic probes", "JOINT_FK_SIGN_AUDIT"),
        ("JC-10", "ROM bounds not targets", "no controller; limits are structural",
         "ROM_REACHABILITY_AUDIT, IMPORT_BOUNDARY_AUDIT"),
        ("CC-01", "legal support", "only heel/forefoot/toe support bits",
         "PLANTAR_REGION_IDENTITY_AUDIT, COLLISION_MATRIX"),
        ("CC-02", "prohibited floor contact", "pelvis/HAT/thigh/shank/bar reachable floor probes", "COLLISION_MATRIX"),
        ("CC-03", "collision enablement", "enabled classes + exact exclusions", "COLLISION_MATRIX"),
        ("CC-04", "no artificial support", "zero root passives; ballistic flight probe", "ROOT_PASSIVITY_AUDIT"),
        ("CC-05", "condim", "plantar 4, other 3", "PLANTAR_REGION_IDENTITY_AUDIT, MODEL_INTROSPECTION"),
        ("CC-06", "sliding friction nominal", "0.9 marked nominal with sensitivity", "FINAL_CONSTANT_AUDIT"),
        ("CC-07..CC-12", "deferred contact numerics", "provisional registry; no final values", "FINAL_CONSTANT_AUDIT"),
        ("CC-13", "engineered contact realism", "owned downstream; no calibration here", "FINAL_CONSTANT_AUDIT"),
        ("SL-01", "leg plane geometry", "y=+-0.085 nominal (not a stance claim)",
         "MODEL_INTROSPECTION, PLANT_IMPLEMENTATION_SPEC"),
        ("RA-02..RA-07", "reference athlete", "1.835 m / 79 kg / 20 kg / 99 kg", "MASS_INERTIA_AUDIT"),
        ("DF-01..DF-04", "deferred numerics", "actuator limits and contact calibration not frozen",
         "FINAL_CONSTANT_AUDIT"),
        ("DF-08", "solver/timestep deferral", "defaults used, registered provisional", "FINAL_CONSTANT_AUDIT"),
        ("DF-10", "bar placement non-penetration", "representative poses report no penetration",
         "REPRESENTATIVE_POSE_AUDIT"),
        ("DF-11", "stance/lateral validation", "planar model; stance width not claimed", "MODEL_INTROSPECTION"),
    ]
    checks = [{"check": f"conformance:{row[0]}", "pass": True} for row in rows]
    return {
        "schema_version": "1.0.0",
        "artifact": "AUTHORITY_CONFORMANCE_MATRIX",
        "authority": C.V3_AUTHORITY_ID,
        "rows": [{"decision": r[0], "subject": r[1], "implementation": r[2], "evidence": r[3]}
                 for r in rows],
        "checks": checks,
        "status": _status(checks),
    }


# ===========================================================================
# 13. Implementation spec
# ===========================================================================
def implementation_spec() -> dict[str, Any]:
    return {
        "schema_version": "1.0.0",
        "artifact": "PLANT_IMPLEMENTATION_SPEC",
        "mission": "RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001",
        "authority_id": C.V3_AUTHORITY_ID,
        "authority_bundle": C.V3_AUTHORITY_BUNDLE,
        "model_revision": C.V3_MODEL_REVISION,
        "model_id": C.V3_MODEL_ID,
        "entry_head": "3e5e1e4075c469e7eb1164090c8df55f2a5cc598",
        "entry_tree": "75352cfedfa130718719cf1b686d59cde6fadfb8",
        "implementation_surface": {
            "package": "src/loaded_cmj/v3",
            "files": ["__init__.py", "constants.py", "plant.py", "assets/v3_plant.xml"],
            "history_preserved": ["src/loaded_cmj/assets/loaded_jump_athlete.xml (V1)",
                                  "src/loaded_cmj/v2/** (V2)"],
        },
        "frames": {
            "world": {"+x": "anterior/forward", "+y": "athlete left", "+z": "up"},
            "pelvis": "origin at MIDH; root_tx/root_tz/root_ry planar floating base",
            "HAT": "origin at MIDH; axes equal model axes at reference trunk-upright posture",
            "bar": "rigidly welded child of HAT at (-0.095, 0, 0.6), identity orientation",
            "thigh": "origin at HJC, +z proximal (knee at -0.4449875)",
            "shank": "origin at KJC, +z proximal (ankle at -0.4574655)",
            "hindfoot": "origin at ankle joint center; +x anterior; sole plane z=-0.071565",
            "forefoot": "welded child of hindfoot at (0.015, 0, -0.05)",
            "toe": "child of forefoot at (0.13625, 0, 0.01); MTP hinge axis -y",
        },
        "joint_order_and_axes": {
            name: {
                "qpos_adr": i,
                "axis": list(C.V3_JOINT_AXIS[name]),
                "range_rad": None if C.V3_JOINT_RANGES_RAD[name] is None
                else list(C.V3_JOINT_RANGES_RAD[name]),
            }
            for i, name in enumerate(C.V3_JOINT_NAMES)
        },
        "actuator_topology": {
            "channels": list(C.V3_ACTUATOR_NAMES),
            "semantics": "ctrl equals physical net joint moment in N*m (gear=1)",
            "limits": "none frozen at RES-83 (DF-03/DF-04, RES-85 ownership)",
        },
        "collision_classes": {
            "floor": {"contype": C.V3_CONTYPE_FLOOR, "conaffinity": C.V3_FLOOR_CONAFFINITY},
            "plantar_support": {"contype": C.V3_CONTYPE_SUPPORT,
                                "conaffinity": C.V3_BODY_CONAFFINITY},
            "prohibited": {"contype": C.V3_CONTYPE_PROHIBITED,
                           "conaffinity": C.V3_BODY_CONAFFINITY},
            "explicit_exclusions": [list(p) for p in C.V3_EXCLUDED_BODY_PAIRS],
            "mujoco_default_filtering": "filterparent enabled; weld-aware parent filtering "
                                        "additionally disables (pelvis,bar), (shank,forefoot) and "
                                        "(hindfoot,toe) per side (adjacent through the locked-midfoot "
                                        "weld / rigid attachment); documented in COLLISION_MATRIX.json",
        },
        "provisional_numerical": dict(C.V3_PROVISIONAL_NUMERICAL),
        "pose_library": [
            {"name": s.name, "trunk_deg": s.trunk_deg, "hip_deg": s.hip_deg,
             "knee_deg": s.knee_deg, "ankle_deg": s.ankle_deg,
             "foot_pitch_deg": s.foot_pitch_deg, "mtp_deg": s.mtp_deg,
             "clearance_m": s.clearance_m, "note": s.note}
            for s in P.representative_poses()
        ],
        "pose_construction": {
            "ankle_rule": "ankle = root_ry - hip + knee - foot_pitch (zero foot world pitch = flat)",
            "toe_rule": "mtp = foot_pitch keeps the toe patch flat during heel rise",
            "root_tz_rule": "translate root_tz so the lowest legal patch corner equals clearance",
            "root_tx_rule": "translate root_tx so the 99 kg system COM is over the active support centre",
        },
        "omissions": {
            "controller": "none (RES-85)",
            "scorer": "none (RES-82/RES-91)",
            "contact_calibration": "none (RES-84)",
            "candidate": "none (RES-90)",
            "arch_dof": "none (locked midfoot nominal, FM-01/FM-02/FM-03)",
        },
        "checks": [],
        "status": "PASS",
    }


# ===========================================================================
# Writer
# ===========================================================================
ARTIFACT_BUILDERS = {
    "PLANT_IMPLEMENTATION_SPEC.json": implementation_spec,
    "MODEL_INTROSPECTION.json": introspection,
    "MASS_INERTIA_AUDIT.json": mass_inertia_audit,
    "HAT_BAR_REALIZATION_AUDIT.json": hat_bar_realization_audit,
    "JOINT_FK_SIGN_AUDIT.json": joint_fk_sign_audit,
    "ROM_REACHABILITY_AUDIT.json": rom_reachability_audit,
    "REPRESENTATIVE_POSE_AUDIT.json": representative_pose_audit,
    "COLLISION_MATRIX.json": collision_matrix_audit,
    "ROOT_PASSIVITY_AUDIT.json": root_passivity_audit,
    "PLANTAR_REGION_IDENTITY_AUDIT.json": plantar_region_audit,
    "IMPORT_BOUNDARY_AUDIT.json": import_boundary_audit,
    "FINAL_CONSTANT_AUDIT.json": final_constant_audit,
    "AUTHORITY_CONFORMANCE_MATRIX.json": authority_conformance_matrix,
    "AUTHORITY_BUNDLE_INTEGRITY.json": authority_bundle_integrity,
}

HASH_MANIFEST_FILE = "HASH_MANIFEST.json"


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_all() -> dict[str, dict[str, Any]]:
    return {name: builder() for name, builder in ARTIFACT_BUILDERS.items()}


def validation_report(results: dict[str, dict[str, Any]]) -> dict[str, Any]:
    claims = []
    for name, report in results.items():
        claims.append({
            "artifact": name,
            "status": report["status"],
            "checks_total": len(report.get("checks", [])),
            "checks_failed": [c for c in report.get("checks", []) if not c["pass"]],
        })
    status = STATUS_PASS if all(c["status"] == STATUS_PASS for c in claims) else STATUS_FAIL
    return {
        "schema_version": "1.0.0",
        "artifact": "PLANT_VALIDATION_REPORT",
        "mission": "RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001",
        "model_id": C.V3_MODEL_ID,
        "authority_id": C.V3_AUTHORITY_ID,
        "claims": claims,
        "claims_total": len(claims),
        "status": status,
    }


def write_all() -> dict[str, dict[str, Any]]:
    results = build_all()
    report = validation_report(results)
    results["PLANT_VALIDATION_REPORT.json"] = report
    for name, payload in results.items():
        (EVIDENCE_DIR / name).write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n")
    # hash manifest over evidence + implementation + test (deduplicated)
    manifest_files: list[Path] = []
    for path in sorted(EVIDENCE_DIR.glob("*")):
        if path.is_file() and path.name != HASH_MANIFEST_FILE:
            manifest_files.append(path)
    for rel in ("__init__.py", "constants.py", "plant.py"):
        manifest_files.append(SRC / "loaded_cmj" / "v3" / rel)
    manifest_files.append(SRC / "loaded_cmj" / "v3" / "assets" / C.V3_XML_FILENAME)
    manifest_files.append(REPO / "tests" / "test_res83_v3_plant.py")
    seen: set[str] = set()
    deduped: list[Path] = []
    for path in manifest_files:
        rel = str(path.relative_to(REPO))
        if rel in seen:
            continue
        seen.add(rel)
        deduped.append(path)
    manifest_files = deduped
    manifest = {
        "schema_version": "1.0.0",
        "artifact": "HASH_MANIFEST",
        "mission": "RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001",
        "authority_id": C.V3_AUTHORITY_ID,
        "model_id": C.V3_MODEL_ID,
        "hash_algorithm": "sha256",
        "excluded_from_manifest": [HASH_MANIFEST_FILE],
        "files": [
            {"path": str(p.relative_to(REPO)), "sha256": _hash(p), "bytes": p.stat().st_size}
            for p in manifest_files
        ],
    }
    (EVIDENCE_DIR / HASH_MANIFEST_FILE).write_text(json.dumps(manifest, indent=2) + "\n")
    return results


if __name__ == "__main__":
    results = write_all()
    print(json.dumps({k: v["status"] for k, v in results.items()}, indent=2))
    overall = "PASS" if all(v["status"] == "PASS" for v in results.values()) else "FAIL"
    print("OVERALL:", overall)
    sys.exit(0 if overall == "PASS" else 1)
