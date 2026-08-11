"""Local 132-D geometry for the frozen ML-238 macro snapshot.

This module owns only local coordinates.  It does not advance MuJoCo, apply
actions, enforce path constraints, or change the raw MacroSnapshot schema.
"""

from __future__ import annotations

from dataclasses import replace

import mujoco
import numpy as np

from loaded_cmj.simulation.constants import BALL_JOINT_NAMES
from loaded_cmj.simulation.snapshot import MacroSnapshot
from loaded_cmj.simulation.so3 import exp_so3, log_so3, matrix_to_quaternion


TANGENT_LAYOUT_ID = "LCMJ-V1-TANGENT-STATE-132-1.0.0"
TANGENT_DIMENSION = 132

CONFIGURATION_SLICE = slice(0, 21)
CACHE_SO3_SLICE = slice(21, 30)
QVEL_SLICE = slice(30, 51)
DRIVESTATE_SLICE = slice(51, 96)
PREVIOUS_ACTION_SLICE = slice(96, 111)
WARMSTART_SLICE = slice(111, 132)

SO3_PERTURBATION_CONVENTION = "RIGHT"
SO3_BRANCH_MAX_RAD = 1.0
ROTATION_ORTHONORMALITY_TOL = 1e-10
ROTATION_DETERMINANT_TOL = 1e-10

_CACHE_BODY_IDS = (1, 2, 4, 7)
_CACHE_RELATIONSHIPS = (
    ("lumbar", 1, 2),
    ("left_hip", 1, 4),
    ("right_hip", 1, 7),
)
_IDENTITY3 = np.eye(3, dtype=np.float64)


class TangentStateError(ValueError):
    """Raised when a tangent operation receives invalid state or coordinates."""


class SO3BranchError(TangentStateError):
    """Raised when a relative SO(3) pair leaves the declared local branch."""


def _validate_delta(delta: np.ndarray) -> np.ndarray:
    try:
        value = np.asarray(delta, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TangentStateError("delta must be a finite float vector") from exc
    if value.shape != (TANGENT_DIMENSION,):
        raise TangentStateError(
            f"delta shape {value.shape} != ({TANGENT_DIMENSION},)"
        )
    if not np.isfinite(value).all():
        raise TangentStateError("delta contains NaN or Inf")
    return value


def _validate_model(model: mujoco.MjModel, snapshot: MacroSnapshot) -> None:
    if (int(model.nq), int(model.nv), int(model.nu)) != (25, 21, 15):
        raise TangentStateError("model dimensions are not the qualified V1 model")
    if snapshot.dimensions[:3] != (25, 21, 15):
        raise TangentStateError("snapshot dimensions are not the qualified V1 model")
    if snapshot.kinematic_body_ids != _CACHE_BODY_IDS:
        raise TangentStateError(
            "snapshot cache rows do not match the frozen body order "
            f"{_CACHE_BODY_IDS}"
        )
    for joint, parent_expected, child_expected in _CACHE_RELATIONSHIPS:
        if joint not in BALL_JOINT_NAMES:
            raise TangentStateError(f"cache joint {joint!r} is not a frozen ball joint")
        jid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint))
        if jid < 0:
            raise TangentStateError(f"cache joint {joint!r} is missing from the model")
        child = int(model.jnt_bodyid[jid])
        parent = int(model.body_parentid[child])
        if (parent, child) != (parent_expected, child_expected):
            raise TangentStateError(
                f"cache relationship for {joint!r} is {(parent, child)}, "
                f"expected {(parent_expected, child_expected)}"
            )


def _validate_rotation(rotation: np.ndarray, name: str) -> np.ndarray:
    value = np.asarray(rotation, dtype=np.float64)
    if value.shape != (3, 3) or not np.isfinite(value).all():
        raise TangentStateError(f"{name} is not a finite 3x3 rotation")
    orth_error = float(np.max(np.abs(value.T @ value - _IDENTITY3)))
    determinant_error = abs(float(np.linalg.det(value)) - 1.0)
    if (
        orth_error > ROTATION_ORTHONORMALITY_TOL
        or determinant_error > ROTATION_DETERMINANT_TOL
    ):
        raise TangentStateError(
            f"{name} is not in SO(3): "
            f"orth_error={orth_error:.3e}, det_error={determinant_error:.3e}"
        )
    return value


def _cache_rows_and_relatives(
    snapshot: MacroSnapshot,
) -> tuple[dict[int, np.ndarray], tuple[np.ndarray, ...]]:
    rows: dict[int, np.ndarray] = {}
    for body_id, raw_row in zip(snapshot.kinematic_body_ids, snapshot.kinematic_xmat):
        rows[int(body_id)] = _validate_rotation(
            np.asarray(raw_row, dtype=np.float64).reshape(3, 3),
            f"cache body {body_id}",
        ).copy()
    relatives = []
    for joint, parent, child in _CACHE_RELATIONSHIPS:
        relative = rows[parent].T @ rows[child]
        relatives.append(_validate_rotation(relative, f"{joint} relative rotation").copy())
    return rows, tuple(relatives)


def _exp_matrix(phi: np.ndarray) -> np.ndarray:
    quaternion = exp_so3(phi)
    raw = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(raw, quaternion)
    return _validate_rotation(raw.reshape(3, 3), "Exp(phi)").copy()


def _relative_log(relative_a: np.ndarray, relative_b: np.ndarray, name: str) -> np.ndarray:
    error = _validate_rotation(relative_a.T @ relative_b, f"{name} relative error")
    phi = log_so3(matrix_to_quaternion(error))
    theta = float(np.linalg.norm(phi))
    if theta >= SO3_BRANCH_MAX_RAD:
        raise SO3BranchError(
            f"{name} relative error angle {theta:.16g} rad reaches "
            f"the strict local guard {SO3_BRANCH_MAX_RAD:.16g} rad"
        )
    return np.asarray(phi, dtype=np.float64)


def boxplus(
    snapshot: MacroSnapshot,
    delta: np.ndarray,
    *,
    model: mujoco.MjModel,
) -> MacroSnapshot:
    """Apply one local 132-D tangent perturbation without mutating snapshot."""
    if not isinstance(snapshot, MacroSnapshot):
        raise TangentStateError("boxplus requires a MacroSnapshot")
    value = _validate_delta(delta)
    _validate_model(model, snapshot)

    rows, relatives = _cache_rows_and_relatives(snapshot)
    if not np.any(value):
        return replace(snapshot)

    qpos = snapshot.qpos.copy()
    if np.any(value[CONFIGURATION_SLICE]):
        mujoco.mj_integratePos(model, qpos, value[CONFIGURATION_SLICE], 1.0)
    if not np.isfinite(qpos).all():
        raise TangentStateError("MuJoCo qpos integration produced NaN or Inf")

    for index, (joint, parent, child) in enumerate(_CACHE_RELATIONSHIPS):
        phi = value[CACHE_SO3_SLICE][3 * index : 3 * index + 3]
        theta = float(np.linalg.norm(phi))
        if theta >= SO3_BRANCH_MAX_RAD:
            raise SO3BranchError(
                f"{joint} perturbation angle {theta:.16g} rad reaches "
                f"the strict local guard {SO3_BRANCH_MAX_RAD:.16g} rad"
            )
        if np.any(phi):
            relative_new = _validate_rotation(
                relatives[index] @ _exp_matrix(phi),
                f"{joint} perturbed relative rotation",
            )
            rows[child] = _validate_rotation(
                rows[parent] @ relative_new,
                f"{joint} reconstructed child rotation",
            ).copy()

    cache_xmat = snapshot.kinematic_xmat.copy()
    for row_index, body_id in enumerate(snapshot.kinematic_body_ids):
        if body_id in rows and body_id != _CACHE_BODY_IDS[0]:
            cache_xmat[row_index] = rows[body_id].reshape(9)

    drive_delta = value[DRIVESTATE_SLICE]
    return replace(
        snapshot,
        qpos=qpos,
        kinematic_xmat=cache_xmat,
        qvel=snapshot.qvel + value[QVEL_SLICE],
        a_plus=snapshot.a_plus + drive_delta[0:15],
        a_minus=snapshot.a_minus + drive_delta[15:30],
        tau_prev=snapshot.tau_prev + drive_delta[30:45],
        previous_accepted_action=(
            snapshot.previous_accepted_action + value[PREVIOUS_ACTION_SLICE]
        ),
        qacc_warmstart=snapshot.qacc_warmstart + value[WARMSTART_SLICE],
    )


def boxminus(
    snapshot_b: MacroSnapshot,
    snapshot_a: MacroSnapshot,
    *,
    model: mujoco.MjModel,
) -> np.ndarray:
    """Return snapshot_b boxminus snapshot_a in the frozen 132-D layout."""
    if not isinstance(snapshot_b, MacroSnapshot) or not isinstance(
        snapshot_a, MacroSnapshot
    ):
        raise TangentStateError("boxminus requires two MacroSnapshot values")
    _validate_model(model, snapshot_a)
    _validate_model(model, snapshot_b)

    delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
    mujoco.mj_differentiatePos(
        model,
        delta[CONFIGURATION_SLICE],
        1.0,
        snapshot_a.qpos,
        snapshot_b.qpos,
    )

    _, relatives_a = _cache_rows_and_relatives(snapshot_a)
    _, relatives_b = _cache_rows_and_relatives(snapshot_b)
    for index, (joint, _, _) in enumerate(_CACHE_RELATIONSHIPS):
        delta[CACHE_SO3_SLICE][3 * index : 3 * index + 3] = _relative_log(
            relatives_a[index], relatives_b[index], joint
        )

    delta[QVEL_SLICE] = snapshot_b.qvel - snapshot_a.qvel
    delta[DRIVESTATE_SLICE] = np.concatenate(
        (
            snapshot_b.a_plus - snapshot_a.a_plus,
            snapshot_b.a_minus - snapshot_a.a_minus,
            snapshot_b.tau_prev - snapshot_a.tau_prev,
        )
    )
    delta[PREVIOUS_ACTION_SLICE] = (
        snapshot_b.previous_accepted_action - snapshot_a.previous_accepted_action
    )
    delta[WARMSTART_SLICE] = snapshot_b.qacc_warmstart - snapshot_a.qacc_warmstart
    if not np.isfinite(delta).all():
        raise TangentStateError("boxminus produced NaN or Inf")
    return delta
