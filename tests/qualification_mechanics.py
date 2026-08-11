#!/usr/bin/env python3
"""mechanics -- mechanics qualification.

contract: loaded-cmj-model-1 mechanics mechanics and proof projection

Writes JSON evidence under --evidence-root and prints one RESULT= line.

Usage:
    uv run python tests/qualification_mechanics.py \
        --evidence-root "$EVIDENCE_ROOT"
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.simulation import so3 as so3_map  # noqa: E402
from loaded_cmj.simulation.plant import Plant, build_model, model_path  # noqa: E402
from loaded_cmj.simulation.constants import (  # noqa: E402
    ACTUATOR_RANGE_NM,
    ATHLETE_BODY_NAMES,
    ATHLETE_MASS_KG,
    BALL_ADMISSIBLE_BOX_RAD,
    BALL_CONE_LIMIT_RAD,
    BALL_JOINT_NAMES,
    BODY_NAMES,
    EXTERNAL_LOAD_MASS_KG,
    FIXED_HOLD_SUBSTEPS,
    CONTACT_F_OFF_N,
    CONTACT_F_ON_N,
    CONTACT_GAP_MAX_M,
    CONTACT_T_OFF_S,
    CONTACT_T_ON_S,
    HOLD_ACTION,
    HINGE_AXES_CHILD_FRAME,
    HINGE_JOINT_NAMES,
    HINGE_RANGES_RAD,
    JOINT_NAMES,
    MJ_ACTUATOR_NAMES,
    RESET_QPOS,
    SEGMENT_GEOMETRY_M,
    TOTAL_MASS_KG,
    BODY_WEIGHT_N,
    PHYSICS_TIMESTEP_S,
)


class Recorder:
    def __init__(self) -> None:
        self.rows: list[dict] = []

    def check(
        self,
        test_id: str,
        what: str,
        ok: bool,
        measured,
        tolerance=None,
        *,
        contract_clause: str | None = None,
        failure_class: str | None = None,
    ) -> bool:
        row = {
            "test_id": test_id, "assertion": what, "result": "PASS" if ok else "FAIL",
            "measured": measured, "tolerance": tolerance,
        }
        if contract_clause is not None:
            row["contract_clause"] = contract_clause
        if failure_class is not None:
            row["failure_class"] = failure_class
        self.rows.append(row)
        return bool(ok)

    @property
    def failures(self) -> list[dict]:
        return [r for r in self.rows if r["result"] == "FAIL"]


def _json_default(value):
    """Serialize deterministic NumPy evidence without dropping dimensions."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"cannot serialize {type(value).__name__}")


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-root", required=True)
    args = ap.parse_args()
    root = Path(args.evidence_root)
    (root / "mechanics").mkdir(parents=True, exist_ok=True)

    rec = Recorder()
    measurements: dict = {}

    # ---------------------------------------------------------- TEST-PLANT-001
    model = build_model()
    xml_sha = hashlib.sha256(model_path().read_bytes()).hexdigest()
    measurements["model_sha256"] = xml_sha
    measurements["mujoco_version"] = mujoco.__version__
    measurements["numpy_version"] = np.__version__
    rec.check("TEST-PLANT-001", "MJCF compiles", True, xml_sha)

    plant = Plant(model)
    idx = plant.idx
    data = plant.make_data()

    # ------------------------------------------------------ FINAL AUTHORITY mechanics
    # These checks are deliberately recorded before the retained predecessor
    # tests.  They close the G0 finding that the old harness could pass while
    # omitting the final neutral-reference, force-duality, reset, and residual
    # obligations.  A failure here is an intended authority gap in the
    # starting implementation, never an import/fixture failure.
    final_action_channels = (
        "lumbar_flexion", "lumbar_lateral", "lumbar_axial",
        "left_hip_flexion", "left_hip_abduction", "left_hip_rotation",
        "right_hip_flexion", "right_hip_abduction", "right_hip_rotation",
        "left_knee_flexion", "right_knee_flexion",
        "left_ankle_sagittal", "right_ankle_sagittal",
        "left_ankle_frontal", "right_ankle_frontal",
    )
    expected_axis_determinants = {"lumbar": 1.0, "left_hip": -1.0, "right_hip": -1.0}
    axis_dets = {
        joint: float(np.linalg.det(so3_map.anatomical_axes(joint)))
        for joint in BALL_JOINT_NAMES
    }
    rec.check(
        "TEST-AUTH-COORD-001",
        "final signed anatomical channel matrices have the authority O(3) parity",
        all(abs(axis_dets[j] - expected_axis_determinants[j]) <= 1e-12
            for j in BALL_JOINT_NAMES),
        axis_dets,
        "det(S_A) = {lumbar:+1,left_hip:-1,right_hip:-1}",
        contract_clause="coordinate_contract.json signed_anatomical_channel_matrices",
        failure_class="intended_final_authority_gap",
    )

    try:
        action_channels = tuple(plant.action_channel_order)
        channel_ok = action_channels == final_action_channels
    except AttributeError as exc:
        action_channels = str(exc)
        channel_ok = False
    rec.check(
        "TEST-AUTH-ACTION-001",
        "15 anatomical drive channels expose the final authority order",
        channel_ok,
        action_channels,
        "exact final action_contract.json order",
        contract_clause="action_contract.json shape/order",
        failure_class="intended_final_authority_gap",
    )

    plant.reset_supported(data)
    neutral_coordinates = {}
    neutral_ok = True
    try:
        for joint in BALL_JOINT_NAMES:
            reference = plant.neutral_reference_quaternion(joint)
            current = plant.relative_rotation_quaternion(data, joint)
            neutral_coordinates[joint] = so3_map.anatomical_coordinate(
                joint, current, reference_quat=reference
            ).tolist()
        neutral_ok = all(
            np.max(np.abs(np.asarray(neutral_coordinates[joint]))) <= 1e-10
            for joint in BALL_JOINT_NAMES
        )
    except Exception as exc:  # noqa: BLE001 - recorded as an intended source gap
        neutral_coordinates = {"error": f"{type(exc).__name__}: {exc}"}
        neutral_ok = False
    measurements["neutral_coordinates"] = neutral_coordinates
    rec.check(
        "TEST-AUTH-COORD-002",
        "reset ball coordinates are zero in the declared neutral-relative chart",
        neutral_ok,
        neutral_coordinates,
        "max abs xi <= 1e-10 rad",
        contract_clause="coordinate_contract.json ball_joint_model neutral/reference",
        failure_class="intended_final_authority_gap",
    )

    reference_gap_ok = False
    reference_gap_measurement = None
    try:
        reference = so3_map.exp_so3(np.array([0.17, -0.11, 0.08]))
        delta = so3_map.exp_so3(np.array([-0.09, 0.13, 0.05]))
        current = _qmul(reference, delta)
        got = so3_map.anatomical_coordinate(
            "left_hip", current, reference_quat=reference
        )
        want = so3_map.anatomical_axes("left_hip").T @ so3_map.log_so3(delta)
        reference_gap_measurement = float(np.max(np.abs(got - want)))
        reference_gap_ok = reference_gap_measurement <= 1e-10
    except Exception as exc:  # noqa: BLE001 - recorded as an intended source gap
        reference_gap_measurement = f"{type(exc).__name__}: {exc}"
    rec.check(
        "TEST-AUTH-COORD-003",
        "SO(3) coordinates use R_ref.T @ R_PC rather than an absolute chart",
        reference_gap_ok,
        reference_gap_measurement,
        "<= 1e-10 rad",
        contract_clause="coordinate_contract.json relative_rotation/neutral",
        failure_class="intended_final_authority_gap",
    )

    load_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "load_shell"
    )
    load_geom_type = int(model.geom_type[load_geom_id])
    load_cylinder_ok = load_geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER)
    rec.check(
        "TEST-AUTH-LOAD-001",
        "external load uses the declared homogeneous-cylinder surrogate",
        load_cylinder_ok,
        {"geom_type": load_geom_type, "expected": int(mujoco.mjtGeom.mjGEOM_CYLINDER)},
        "exact",
        contract_clause="load_contract.json geometry/inertia_at_com",
        failure_class="intended_final_authority_gap",
    )

    force_map_ok = False
    force_map_measurement = None
    try:
        reference = plant.neutral_reference_quaternion("left_hip")
        current = so3_map.exp_so3(np.array([0.08, -0.12, 0.06]))
        omega = np.array([0.4, -0.2, 0.7])
        tau = np.array([41.0, -23.0, 17.0])
        E = so3_map.anatomical_velocity_map(
            "left_hip", current, reference_quat=reference
        )
        tau_mj = so3_map.dual_torque(
            "left_hip", current, tau, reference_quat=reference
        )
        residual = float(abs(tau @ (E @ omega) - tau_mj @ omega))
        expected = (
            so3_map.relative_rotation_matrix(reference, current).T
            @ so3_map.anatomical_axes("left_hip") @ tau
        )
        map_error = float(np.max(np.abs(tau_mj - expected)))
        force_map_measurement = {"power_residual": residual, "map_error": map_error}
        force_map_ok = residual <= 1e-10 and map_error <= 1e-10
    except Exception as exc:  # noqa: BLE001 - recorded as an intended source gap
        force_map_measurement = f"{type(exc).__name__}: {exc}"
    rec.check(
        "TEST-AUTH-FORCE-001",
        "ball generalized-force map is E(q).T and preserves virtual-work power",
        force_map_ok,
        force_map_measurement,
        "power/map residual <= 1e-10",
        contract_clause="generalized_force_contract.json ball_map",
        failure_class="intended_final_authority_gap",
    )

    residual_ok = False
    residual_measurement = None
    try:
        residual_report = plant.dynamics_residual(data)
        residual_measurement = residual_report
        residual_ok = (
            np.asarray(residual_report["residual"], dtype=float).shape == (model.nv,)
            and np.isfinite(residual_report["residual"]).all()
        )
    except Exception as exc:  # noqa: BLE001 - recorded as an intended source gap
        residual_measurement = f"{type(exc).__name__}: {exc}"
    rec.check(
        "TEST-AUTH-FORCE-002",
        "continuous generalized force balance exposes a finite nv=21 residual",
        residual_ok,
        residual_measurement,
        "shape (21,), finite",
        contract_clause="force_accounting_contract.json FA-RESIDUAL",
        failure_class="intended_final_authority_gap",
    )

    reset_api_ok = False
    reset_api_measurement = None
    try:
        reset_state = plant.reset(seed=0, metadata=None)
        hold_action = np.asarray(HOLD_ACTION, dtype=float)
        reset_api_measurement = {
            "previous_action_equal_hold_action": bool(
                np.array_equal(reset_state.previous_action, hold_action)
            ),
            "hold_action_shape": list(hold_action.shape),
            "qvel_inf": float(np.abs(reset_state.qvel).max()),
        }
        reset_api_ok = (
            reset_api_measurement["previous_action_equal_hold_action"]
            and hold_action.shape == (15,)
            and np.isfinite(hold_action).all()
            and reset_api_measurement["qvel_inf"] == 0.0
        )
    except Exception as exc:  # noqa: BLE001 - recorded as an intended source gap
        reset_api_measurement = f"{type(exc).__name__}: {exc}"
    rec.check(
        "TEST-AUTH-RESET-001",
        "reset(seed=0, metadata=None) selects deterministic HOLD_ACTION as previous action",
        reset_api_ok,
        reset_api_measurement,
        "exact action equality; qvel == 0",
        contract_clause="reset_contract.json tuple and deterministic selection",
        failure_class="intended_final_authority_gap",
    )

    # ---------------------------------------------------------- TEST-PLANT-002
    dims = {"nq": model.nq, "nv": model.nv, "nu": model.nu, "nbody": model.nbody,
            "njnt": model.njnt, "ngeom": model.ngeom, "neq": model.neq, "na": model.na}
    want = {"nq": 25, "nv": 21, "nu": 15, "nbody": 10, "njnt": 10,
            "ngeom": 16, "neq": 0, "na": 0}
    measurements["dimensions"] = {k: int(v) for k, v in dims.items()}
    rec.check("TEST-PLANT-002", "compiled dimensions",
              all(int(dims[k]) == v for k, v in want.items()), measurements["dimensions"], "exact")

    # ---------------------------------------------------------- TEST-PLANT-003
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(1, model.nbody)]
    measurements["body_names"] = names
    rec.check("TEST-PLANT-003", "exact nine-body inventory, no arm/hand/toe body",
              names == list(BODY_NAMES), names, "exact")

    # ---------------------------------------------------------- TEST-PLANT-004
    jnames = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    measurements["joint_names"] = jnames
    no_mtp = not any("mtp" in n.lower() or "toe" in n.lower() for n in jnames + names)
    rec.check("TEST-PLANT-004", "exact ten-joint inventory, no MTP joint",
              jnames == list(JOINT_NAMES) and no_mtp, jnames, "exact")
    anames = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    rec.check("TEST-PLANT-004b", "exact fifteen-actuator inventory",
              anames == list(MJ_ACTUATOR_NAMES), anames, "exact")

    # ---------------------------------------------------------- TEST-PLANT-005
    load = idx.load_body
    rec.check("TEST-PLANT-005", "external_load is rigid: no joint, no actuator, neq == 0",
              int(model.body_jntnum[load]) == 0 and int(model.neq) == 0,
              {"body_jntnum": int(model.body_jntnum[load]), "neq": int(model.neq)}, "exact")

    # ---------------------------------------------------------- TEST-PLANT-006
    world_children = [names[b - 1] for b in range(1, model.nbody)
                      if int(model.body_parentid[b]) == 0]
    rec.check("TEST-PLANT-006", "no world support: pelvis is the only child of world",
              world_children == ["pelvis"], world_children, "exact")

    # ---------------------------------------------------------- TEST-PLANT-007
    root_jid = idx.joint["root"]
    root_driven = any(int(model.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_JOINT)
                      and int(model.actuator_trnid[a, 0]) == root_jid for a in range(model.nu))
    # actuation moment arm rows 0..5 are structurally zero
    plant.reset_supported(data)
    data.ctrl[:] = 1.0
    mujoco.mj_forward(model, data)
    root_rows = float(np.abs(data.qfrc_actuator[0:6]).max())
    measurements["root_actuation_rows_max"] = root_rows
    rec.check("TEST-PLANT-007", "no root actuation; actuation rows 0..5 structurally zero",
              (not root_driven) and root_rows == 0.0, root_rows, "exact 0.0")
    data.ctrl[:] = 0.0

    # ---------------------------------------------------------- TEST-PLANT-009
    rec.check("TEST-PLANT-009", "no gravcomp, no equality, no deformable",
              float(np.abs(model.body_gravcomp).max()) == 0.0 and int(model.neq) == 0
              and int(getattr(model, "nflex", 0)) == 0,
              {"gravcomp": float(np.abs(model.body_gravcomp).max()),
               "neq": int(model.neq), "nflex": int(getattr(model, "nflex", 0))}, "exact")

    # ---------------------------------------------------------- TEST-PLANT-010
    plant.reset_supported(data)
    rng = np.random.default_rng(0)
    worst_norm = 0.0
    for _ in range(2000):
        delta = rng.normal(size=model.nv) * 0.05
        mujoco.mj_integratePos(model, data.qpos, delta, 1.0)
        for jn in ("root",) + BALL_JOINT_NAMES:
            a = idx.qadr[jn] + (3 if jn == "root" else 0)
            worst_norm = max(worst_norm, abs(float(np.linalg.norm(data.qpos[a:a + 4])) - 1.0))
    measurements["manifold_max_quat_norm_error"] = worst_norm
    rec.check("TEST-PLANT-010", "manifold integration preserves unit quaternion norm",
              worst_norm <= 1e-9, worst_norm, 1e-9)

    # --------------------------------------------- TEST-PLANT-011/012/013 mass
    athlete = float(sum(model.body_mass[idx.body[n]] for n in ATHLETE_BODY_NAMES))
    load_mass = float(model.body_mass[load])
    total = float(model.body_mass.sum())
    measurements["mass_kg"] = {"athlete": athlete, "load": load_mass, "total": total}
    rec.check("TEST-PLANT-011", "athlete mass closure == 75.0",
              abs(athlete - ATHLETE_MASS_KG) <= 1e-9, athlete, 1e-9)
    rec.check("TEST-PLANT-012", "external load mass closure == 20.0",
              abs(load_mass - EXTERNAL_LOAD_MASS_KG) <= 1e-9, load_mass, 1e-9)
    rec.check("TEST-PLANT-013", "total mass closure == 95.0",
              abs(total - TOTAL_MASS_KG) <= 1e-9, total, 1e-9)

    # ------------------------------------------------- TEST-PLANT-014 COM/geom
    zero = np.zeros(model.nq)
    zero[idx.qadr["root"] + 3] = 1.0
    for jn in BALL_JOINT_NAMES:
        zero[idx.qadr[jn]] = 1.0
    zero[idx.qadr["root"] + 2] = SEGMENT_GEOMETRY_M["pelvis_origin_height_at_full_extension"]
    data.qpos[:] = zero
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    full_ext = float(data.xpos[idx.pelvis_body][2])
    ids = list(idx.all_bodies)
    com_manual = np.asarray(model.body_mass[ids] @ data.xipos[ids] / model.body_mass[ids].sum())
    com_mj = np.asarray(data.subtree_com[idx.pelvis_body])
    com_err = float(np.abs(com_manual - com_mj).max())
    measurements["full_extension_pelvis_height_m"] = full_ext
    measurements["com_reconstruction_error_m"] = com_err
    rec.check("TEST-PLANT-014", "COM closure and full-extension pelvis height == 0.930",
              abs(full_ext - 0.930) <= 1e-9 and com_err <= 1e-9,
              {"pelvis_height": full_ext, "com_err": com_err}, 1e-9)

    # -------------------------------------------------- TEST-PLANT-015 inertia
    inertia_ok = True
    inertia_rows = {}
    for n in BODY_NAMES:
        ia = np.asarray(model.body_inertia[idx.body[n]], dtype=float)
        pos = bool((ia > 0).all())
        tri = bool(ia[0] + ia[1] >= ia[2] - 1e-12 and ia[0] + ia[2] >= ia[1] - 1e-12
                   and ia[1] + ia[2] >= ia[0] - 1e-12)
        inertia_rows[n] = {"diaginertia": ia.tolist(), "spd": pos, "triangle": tri}
        inertia_ok = inertia_ok and pos and tri
    measurements["inertia"] = inertia_rows
    rec.check("TEST-PLANT-015", "every inertia SPD with triangle inequalities",
              inertia_ok, "see mechanics_measurements.json", 1e-12)
    # mass matrix symmetry and positive definiteness at the reset pose
    plant.reset_supported(data)
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    sym = float(np.abs(M - M.T).max())
    eig = float(np.linalg.eigvalsh(0.5 * (M + M.T)).min())
    measurements["mass_matrix"] = {"asymmetry": sym, "min_eigenvalue": eig}
    rec.check("TEST-PLANT-015b", "mass matrix symmetric and positive definite",
              sym <= 1e-12 and eig > 0.0, measurements["mass_matrix"], 1e-12)

    # ---------------------------------------------- TEST-PLANT-016 joint domain
    domain_ok = True
    for jn in HINGE_JOINT_NAMES:
        jid = idx.joint[jn]
        axis = np.asarray(model.jnt_axis[jid], dtype=float)
        rng_ = np.asarray(model.jnt_range[jid], dtype=float)
        domain_ok = domain_ok and np.allclose(axis, HINGE_AXES_CHILD_FRAME[jn], atol=1e-12)
        domain_ok = domain_ok and np.allclose(rng_, HINGE_RANGES_RAD[jn], atol=1e-9)
        domain_ok = domain_ok and bool(model.jnt_limited[jid])
    cone_rows = {}
    for jn in BALL_JOINT_NAMES:
        box = BALL_ADMISSIBLE_BOX_RAD[jn]
        corners = [math.sqrt(a * a + b * b + c * c)
                   for a in box[0] for b in box[1] for c in box[2]]
        worst = max(corners)
        cone = float(model.jnt_range[idx.joint[jn]][1])
        cone_rows[jn] = {"box_corner_norm": worst, "cone_limit": cone, "contains": worst < cone}
        domain_ok = domain_ok and worst < cone
        domain_ok = domain_ok and abs(cone - BALL_CONE_LIMIT_RAD[jn]) <= 1e-9
    measurements["ball_cone_containment"] = cone_rows
    rec.check("TEST-PLANT-016", "axes/ranges exact; cone strictly contains the anatomical box",
              domain_ok, cone_rows, "exact")

    # ------------------------------------------- TEST-PLANT-017 collision groups
    floor = idx.floor_geom
    coll_ok = int(model.geom_contype[floor]) == 1 and int(model.geom_conaffinity[floor]) == 2
    for gid in list(idx.left_pads) + list(idx.right_pads) + list(idx.shell_geoms):
        coll_ok = coll_ok and int(model.geom_contype[gid]) == 2
        coll_ok = coll_ok and int(model.geom_conaffinity[gid]) == 1
    measurements["exclude_pairs"] = int(model.nexclude)
    rec.check("TEST-PLANT-017", "collision groups: only floor-vs-body pairs can collide",
              coll_ok and int(model.nexclude) == 9,
              {"groups_ok": coll_ok, "nexclude": int(model.nexclude)}, "exact")

    # ------------------------------------------- TEST-PLANT-018 rigid-load check
    worst_rel = 0.0
    for _ in range(500):
        q = np.array(RESET_QPOS, dtype=float)
        delta = rng.normal(size=model.nv) * 0.10
        data.qpos[:] = q
        mujoco.mj_integratePos(model, data.qpos, delta, 1.0)
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        # the load's contribution to the torso-assembly first moment, computed
        # independently from body_mass/xipos, must match MuJoCo's subtree values
        m_load = float(model.body_mass[load])
        indep = m_load * np.asarray(data.xipos[load])
        sub = (float(model.body_subtreemass[idx.torso_body])
               * np.asarray(data.subtree_com[idx.torso_body])
               - float(model.body_mass[idx.torso_body]) * np.asarray(data.xipos[idx.torso_body]))
        denom = max(1e-9, float(np.abs(indep).max()))
        worst_rel = max(worst_rel, float(np.abs(indep - sub).max()) / denom)
    measurements["rigid_load_max_rel_error"] = worst_rel
    rec.check("TEST-PLANT-018", "rigid-load consistency over 500 poses",
              worst_rel <= 1e-8, worst_rel, 1e-8)

    # -------------------------------------------------- TEST-MAP-001 .. 006
    worst_fd = 0.0
    worst_fd_left = 0.0
    for jn in BALL_JOINT_NAMES:
        for _ in range(500):
            ax = rng.normal(size=3)
            ax /= np.linalg.norm(ax)
            phi = ax * rng.uniform(0.01, 2.30)
            q = so3_map.exp_so3(phi)
            om = rng.normal(size=3)
            om /= np.linalg.norm(om)
            hstep = 1e-6
            fd = (so3_map.anatomical_coordinate(jn, _qmul(q, so3_map.exp_so3(om * hstep)))
                  - so3_map.anatomical_coordinate(jn, _qmul(q, so3_map.exp_so3(-om * hstep)))
                  ) / (2 * hstep)
            worst_fd = max(worst_fd, float(np.abs(so3_map.anatomical_rate(jn, q, om) - fd).max()))
            th = float(np.linalg.norm(phi))
            px = so3_map._skew(phi)
            c = 1.0 / th ** 2 - (1.0 + math.cos(th)) / (2 * th * math.sin(th))
            jl = np.eye(3) - 0.5 * px + c * (px @ px)
            worst_fd_left = max(
                worst_fd_left,
                float(np.abs((so3_map.anatomical_axes(jn).T @ jl) @ om - fd).max()))
    measurements["so3_fd_max_error_radps"] = worst_fd
    measurements["so3_fd_max_error_left_jacobian"] = worst_fd_left
    rec.check("TEST-MAP-001", "T_j equals the finite-difference eta_dot (right Jacobian)",
              worst_fd <= 1e-6, worst_fd, 1e-6)
    rec.check("TEST-MAP-006", "meta-test: TEST-MAP-001 fails with the LEFT Jacobian",
              worst_fd_left > 1e-6, worst_fd_left, "> 1e-6")

    # bilateral mirroring: a mirrored state gives identical anatomical coordinates
    mirror_err = 0.0
    for _ in range(200):
        eta = rng.normal(size=3) * 0.3
        ql = so3_map.exp_so3(so3_map.anatomical_axes("left_hip") @ eta)
        qr = so3_map.exp_so3(so3_map.anatomical_axes("right_hip") @ eta)
        mirror_err = max(mirror_err, float(np.abs(
            so3_map.anatomical_coordinate("left_hip", ql)
            - so3_map.anatomical_coordinate("right_hip", qr)).max()))
    measurements["so3_bilateral_mirror_error"] = mirror_err
    rec.check("TEST-MAP-002", "bilateral mirroring lives entirely in A_j",
              mirror_err <= 1e-9, mirror_err, 1e-9)

    worst_pow = 0.0
    for jn in BALL_JOINT_NAMES:
        for _ in range(700):
            ax = rng.normal(size=3)
            ax /= np.linalg.norm(ax)
            q = so3_map.exp_so3(ax * rng.uniform(0.01, 2.30))
            om = rng.normal(size=3)
            tau = rng.normal(size=3) * 100.0
            lhs = float(so3_map.dual_torque(jn, q, tau) @ om)
            rhs = float(tau @ so3_map.anatomical_velocity(jn, q, om))
            worst_pow = max(worst_pow, abs(lhs - rhs) / max(1.0, abs(rhs)))
    measurements["so3_power_identity_max_rel_error"] = worst_pow
    rec.check("TEST-MAP-003", "dual-map power identity", worst_pow <= 1e-9, worst_pow, 1e-9)

    worst_step = 0.0
    axis = np.array([0.3, -0.5, 0.81])
    axis /= np.linalg.norm(axis)
    prev = None
    for k in range(10001):
        th = 2.9 * k / 10000.0
        q = so3_map.exp_so3(axis * th)
        if k % 2 == 1:
            q = -q
        eta = so3_map.anatomical_coordinate("left_hip", q)
        if prev is not None:
            worst_step = max(worst_step, float(np.abs(eta - prev).max()))
        prev = eta
    measurements["so3_hemisphere_max_step_rad"] = worst_step
    rec.check("TEST-MAP-004", "hemisphere continuity under +/-Q", worst_step <= 1e-3,
              worst_step, 1e-3)

    cond_rows = {}
    force_rows = {}
    grid_ok = True
    for jn in BALL_JOINT_NAMES:
        box = BALL_ADMISSIBLE_BOX_RAD[jn]
        axes = [np.linspace(lo, hi, 21) for lo, hi in box]
        worst_cond = 0.0
        worst_force = 0.0
        aid = MJ_ACTUATOR_NAMES[{"lumbar": 0, "left_hip": 3, "right_hip": 6}[jn]]
        frange = float(ACTUATOR_RANGE_NM[aid][1])
        from loaded_cmj.simulation.drive import TAU_BAR_NEG, TAU_BAR_POS
        c0 = {"lumbar": 0, "left_hip": 3, "right_hip": 6}[jn]
        tbar = np.maximum(TAU_BAR_POS[c0:c0 + 3], TAU_BAR_NEG[c0:c0 + 3])
        for a in axes[0]:
            for b in axes[1]:
                for c in axes[2]:
                    eta = np.array([a, b, c])
                    q = so3_map.exp_so3(so3_map.anatomical_axes(jn) @ eta)
                    tmat = so3_map.tangent_map(jn, q, check=False)
                    worst_cond = max(worst_cond, so3_map.condition_number(tmat))
                    for sgn in ((1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
                                (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1)):
                        tau = tbar * np.array(sgn, dtype=float)
                        worst_force = max(
                            worst_force,
                            float(np.abs(so3_map.dual_torque(jn, q, tau)).max()),
                        )
        cond_rows[jn] = worst_cond
        force_rows[jn] = {"max_abs_generalized_Nm": worst_force,
                          "forcerange": frange, "ratio": worst_force / frange}
        grid_ok = grid_ok and worst_cond <= 5.0 and worst_force <= 0.91 * frange
    measurements["so3_max_condition_number"] = cond_rows
    measurements["so3_max_generalized_torque"] = force_rows
    rec.check("TEST-MAP-005", "cond_2(T_j) <= 5.0 and ||T^T tau||_inf <= 0.91 x forcerange",
              grid_ok, {"cond": cond_rows, "force": force_rows}, "cond<=5.0, ratio<=0.91")

    # ------------------------------------------------------------ TEST-PAS-005
    rec.check("TEST-PAS-005", "root stiffness / damping / armature exactly zero",
              float(model.jnt_stiffness[root_jid]) == 0.0
              and float(np.abs(model.dof_damping[0:6]).max()) == 0.0
              and float(np.abs(model.dof_armature[0:6]).max()) == 0.0,
              {"stiffness": float(model.jnt_stiffness[root_jid]),
               "damping": float(np.abs(model.dof_damping[0:6]).max()),
               "armature": float(np.abs(model.dof_armature[0:6]).max())}, "exact 0.0")

    # -------------------------------------------------------------- TEST-RST-001
    plant.reset_supported(data)
    try:
        report = plant.assert_reset_admissible(data)
        rec.check("TEST-RST-001", "supported reset admissibility R1-R10", True, report,
                  "RESET_CONTRACT.md section 3")
        measurements["reset_admissibility"] = report
    except Exception as exc:  # noqa: BLE001
        rec.check("TEST-RST-001", "supported reset admissibility R1-R10", False, str(exc),
                  "RESET_CONTRACT.md section 3")

    # ------------------------------------------------------------- TEST-RST-006
    # AST scan, not a text scan: the assertion is that mj_inverse is never
    # called by the public trusted source tree.
    import ast
    forbidden = []
    public_source = _TASK_ROOT / "src" / "loaded_cmj"
    for path in sorted(public_source.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Attribute) and node.attr == "mj_inverse":
                forbidden.append(str(path.relative_to(_TASK_ROOT)))
            elif isinstance(node, ast.Name) and node.id == "mj_inverse":
                forbidden.append(str(path.relative_to(_TASK_ROOT)))
    rec.check("TEST-GUARD-006", "mj_inverse never called in trusted public source",
              not forbidden, forbidden, "exact")

    # ------------------------------------------------------------- TEST-RST-002
    # Independently instrument the production fixed-hold path and report every
    # Reset criteria. The action is open loop and all torques traverse the
    # unchanged production actuator pipeline.
    from loaded_cmj.simulation import drive as drive
    plant.reset_supported(data)
    hold_action = np.asarray(HOLD_ACTION, dtype=float).copy()
    drive_state = drive.DriveState(
        a_plus=np.maximum(hold_action, 0.0),
        a_minus=np.maximum(-hold_action, 0.0),
        tau_prev=plant.tau_eq,
        previous_command=hold_action,
    )
    z = (drive_state.a_plus - drive_state.a_minus).copy()
    tau_prev = drive_state.tau_prev.copy()
    com0 = plant.center_of_mass(data).copy()
    qvel_hist: list[float] = []
    qacc_hist: list[float] = []
    grf_hist: list[float] = []
    pen_hist: list[float] = []
    latch = np.zeros(2, dtype=bool)
    on_steps = np.zeros(2, dtype=np.int64)
    off_steps = np.zeros(2, dtype=np.int64)
    on_required = int(round(CONTACT_T_ON_S / PHYSICS_TIMESTEP_S))
    off_required = int(round(CONTACT_T_OFF_S / PHYSICS_TIMESTEP_S))
    latch_after_50ms: list[bool] = []
    no_limit_contact = True
    all_finite = True
    applied = 0.0
    limit_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    for step in range(FIXED_HOLD_SUBSTEPS):
        s_now = plant.anatomical_coordinates(data)
        sd_now = plant.anatomical_rates(data)
        result = drive.drive_state_step(
            hold_action, drive_state, s_now, sd_now, PHYSICS_TIMESTEP_S
        )
        z = np.asarray(result["drive"], dtype=np.float64)
        tau_prev = np.asarray(result["tau"], dtype=np.float64)
        plant.apply_anatomical_torque(data, tau_prev)
        mujoco.mj_step(model, data)
        qvel_hist.append(float(np.abs(data.qvel).max()))
        qacc_hist.append(float(np.abs(data.qacc).max()))
        fz = plant.foot_contact_summary(data)[0]
        grf_hist.append(float(fz.sum()))
        pen_hist.append(plant.max_pad_penetration(data))
        applied = max(applied, float(np.abs(data.qfrc_applied).max()),
                      float(np.abs(data.xfrc_applied).max()))
        gaps = plant.pad_gaps(data).reshape(2, 4).min(axis=1)
        for foot in range(2):
            if not latch[foot]:
                on_steps[foot] = (on_steps[foot] + 1
                                  if fz[foot] >= CONTACT_F_ON_N
                                  and gaps[foot] <= CONTACT_GAP_MAX_M else 0)
                if on_steps[foot] >= on_required:
                    latch[foot] = True
                    off_steps[foot] = 0
            else:
                off_steps[foot] = off_steps[foot] + 1 if fz[foot] <= CONTACT_F_OFF_N else 0
                if off_steps[foot] >= off_required:
                    latch[foot] = False
                    on_steps[foot] = 0
        if (step + 1) * PHYSICS_TIMESTEP_S >= 0.050:
            latch_after_50ms.append(bool(latch.all()))
        if data.nefc:
            no_limit_contact = no_limit_contact and not bool(
                np.any(np.asarray(data.efc_type[:data.nefc], dtype=np.int32) == limit_type)
            )
        all_finite = all_finite and all(np.isfinite(x).all() for x in (
            data.qpos, data.qvel, data.qacc, data.ctrl, data.qfrc_applied,
            data.xfrc_applied, z, tau_prev, fz,
        ))
    com_end = plant.center_of_mass(data)
    values = {
        "D1": float(np.linalg.norm(com_end[0:2] - com0[0:2])),
        "D2": float(abs(com_end[2] - com0[2])),
        "D3": float(max(qvel_hist[-100:])),
        "D4": float(max(qacc_hist[-100:])),
        "D5": bool(latch_after_50ms and all(latch_after_50ms)),
        "D6": float(np.mean(grf_hist[-100:])),
        "D7": bool(no_limit_contact),
        "D8": float(max(pen_hist)),
        "D9": bool(all_finite),
        "D10": float(applied),
    }
    thresholds = {
        "D1": "<= 0.005 m", "D2": "<= 0.004 m", "D3": "<= 0.02",
        "D4": "<= 1.0", "D5": "both ON continuously from 0.050 s",
        "D6": "931.95 N +/-2%", "D7": "no active joint-limit constraint",
        "D8": "<= 0.001 m", "D9": "all states finite", "D10": "exact 0.0",
    }
    passes = {
        "D1": values["D1"] <= 0.005,
        "D2": values["D2"] <= 0.004,
        "D3": values["D3"] <= 0.02,
        "D4": values["D4"] <= 1.0,
        "D5": values["D5"],
        "D6": abs(values["D6"] - BODY_WEIGHT_N) / BODY_WEIGHT_N <= 0.02,
        "D7": values["D7"],
        "D8": values["D8"] <= 0.001,
        "D9": values["D9"],
        "D10": values["D10"] == 0.0,
    }
    dwell = {
        key: {"measurement": values[key], "threshold": thresholds[key],
              "result": "PASS" if passes[key] else "FAIL"}
        for key in (f"D{i}" for i in range(1, 11))
    }
    dwell.update({"z_G_start_m": float(com0[2]), "z_G_end_m": float(com_end[2])})
    measurements["fixed_hold_dwell"] = dwell
    rec.check("TEST-RST-002", "fixed-hold dwell reports and passes D1-D10",
              all(passes.values()), dwell, "RESET_CONTRACT.md section 4.2")

    # ------------------------------------------------------------- TEST-RST-003
    # Contract-defined static inverse dynamics at the settled pose, with zero
    # generalized velocity and acceleration.  Map ball generalized moments
    # back to anatomical coordinates using the same dual transform.
    settled_qpos = data.qpos.copy()
    rne_data = plant.make_data()
    rne_data.qpos[:] = settled_qpos
    rne_data.qvel[:] = 0.0
    mujoco.mj_forward(model, rne_data)
    required = np.zeros(model.nv, dtype=np.float64)
    mujoco.mj_rne(model, rne_data, 0, required)
    hip_total = 0.0
    for joint in ("left_hip", "right_hip"):
        qadr, vadr = idx.qadr[joint], idx.vadr[joint]
        current = plant.relative_rotation_quaternion(rne_data, joint)
        E = so3_map.anatomical_velocity_map(
            joint,
            current,
            reference_quat=plant.neutral_reference_quaternion(joint),
        )
        eta_moment = np.linalg.solve(E.T, required[vadr:vadr + 3])
        hip_total += abs(float(eta_moment[0]))
    knee_total = sum(abs(float(required[idx.vadr[j]]))
                     for j in ("left_knee_flexion", "right_knee_flexion"))
    ankle_total = sum(abs(float(required[idx.vadr[j]]))
                      for j in ("left_ankle_dorsiflexion", "right_ankle_dorsiflexion"))
    support_ratios = {
        "hip": hip_total / (2.0 * drive.TAU_BAR_NEG[3]),
        "knee": knee_total / (2.0 * drive.TAU_BAR_NEG[9]),
        "ankle": ankle_total / (2.0 * drive.TAU_BAR_NEG[11]),
    }
    measurements["static_support_reserve"] = support_ratios
    rec.check("TEST-RST-003", "static hip/knee/ankle support demand ratios <= 0.45",
              all(v <= 0.45 for v in support_ratios.values()), support_ratios, "<= 0.45")

    # ------------------------------------------------------------- TEST-RST-005
    settled = []
    for _ in range(3):
        d2 = plant.make_data()
        settled.append(plant.reset_fixed_hold(d2))

    def _same_observation(a: dict, b: dict) -> bool:
        if a.keys() != b.keys():
            return False
        return all(np.array_equal(a[k], b[k]) if isinstance(a[k], np.ndarray)
                   else a[k] == b[k] for k in a)

    base = settled[0]
    det = all(
        np.array_equal(base.qpos, x.qpos)
        and np.array_equal(base.qvel, x.qvel)
        and np.array_equal(base.drive_state, x.drive_state)
        and np.array_equal(base.previous_torque, x.previous_torque)
        and np.array_equal(base.previous_action, x.previous_action)
        and np.array_equal(base.contact_force_N, x.contact_force_N)
        and np.array_equal(base.contact_cop_xy_m, x.contact_cop_xy_m)
        and np.array_equal(base.contact_cop_valid, x.contact_cop_valid)
        and _same_observation(base.first_observation, x.first_observation)
        for x in settled[1:]
    )
    reset_digest = hashlib.sha256(b"".join((
        base.qpos.tobytes(), base.qvel.tobytes(), base.drive_state.tobytes(),
        base.previous_torque.tobytes(), base.previous_action.tobytes(),
        base.contact_force_N.tobytes(), base.contact_cop_xy_m.tobytes(),
        base.contact_cop_valid.tobytes(),
    ))).hexdigest()
    measurements["settled_reset"] = {
        "sha256": reset_digest,
        "qpos": base.qpos.tolist(),
        "qvel": base.qvel.tolist(),
        "drive_state": base.drive_state.tolist(),
        "previous_torque": base.previous_torque.tolist(),
        "previous_action": base.previous_action.tolist(),
        "contact_force_N": base.contact_force_N.tolist(),
        "contact_cop_xy_m": base.contact_cop_xy_m.tolist(),
        "contact_cop_valid": base.contact_cop_valid.tolist(),
    }
    rec.check("TEST-RST-005", "three settled resets are bit-identical across all required state",
              det, reset_digest, "bit-identical")

    # Affected supported-start/contact subgate at the actual scored initial state.
    supported_ok = (bool(base.contact_cop_valid.all())
                    and float(np.abs(base.qvel).max()) <= 0.02
                    and plant.support_margin(data, plant.center_of_mass(data), (True, True)) >= 0.020)
    rec.check("TEST-CON-001", "settled scored start is bilateral, slow, and inside support",
              supported_ok,
              {"both_contact": bool(base.contact_cop_valid.all()),
               "qvel_inf": float(np.abs(base.qvel).max()),
               "com_margin_m": plant.support_margin(
                   data, plant.center_of_mass(data), (True, True))},
              "both contact; qvel<=0.02; margin>=0.020 m")

    # ------------------------------------------------------------------ output
    failures = rec.failures
    result = "PASS" if not failures else "FAIL"
    (root / "mechanics" / "mechanics_test_results.json").write_text(json.dumps({
        "suite": "mechanics", "contract_revision": "loaded-cmj-model-1", "result": result,
        "tests_run": len(rec.rows),
        "tests_passed": len(rec.rows) - len(failures),
        "tests_failed": len(failures),
        "model_sha256": xml_sha,
        "rows": rec.rows,
    }, indent=2, default=_json_default), encoding="utf-8")
    (root / "mechanics" / "mechanics_measurements.json").write_text(
        json.dumps(measurements, indent=2, default=_json_default), encoding="utf-8")

    for row in failures:
        print(f"  FAIL {row['test_id']}: {row['assertion']} -> {row['measured']}")
    print(f"mechanics tests_run={len(rec.rows)} passed={len(rec.rows) - len(failures)} "
          f"failed={len(failures)}")
    print(f"RESULT={result} mechanics-MECHANICS")
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
