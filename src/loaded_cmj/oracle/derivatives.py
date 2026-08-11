"""Privileged local derivatives of the frozen wrapped 5 ms transition.

This module owns only the finite-difference wrapper.  It deliberately delegates
state geometry, restart, action projection, actuator dynamics, torque mapping,
and physics stepping to their frozen owners.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np

from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import ACTION_DIM
from loaded_cmj.simulation.plant import Plant
from loaded_cmj.simulation.snapshot import MacroSnapshot
from loaded_cmj.simulation.tangent import (
    CACHE_SO3_SLICE,
    CONFIGURATION_SLICE,
    DRIVESTATE_SLICE,
    PREVIOUS_ACTION_SLICE,
    QVEL_SLICE,
    TANGENT_DIMENSION,
    WARMSTART_SLICE,
    boxminus,
    boxplus,
)
from loaded_cmj.simulation.transition import (
    ACCEPTED_ACTION_MAX_STEP,
    TransitionResult,
    project_accepted_action,
    step_5ms,
)


DERIVATIVE_API_VERSION = "LCMJ-V1-WRAPPED-DERIVATIVE-1.0.0"
QACC_DERIVATIVE_UNADJUDICATED = "UNADJUDICATED"
QACC_DERIVATIVE_QUALIFIED_NONZERO = "QUALIFIED_NONZERO"
QACC_DERIVATIVE_NUMERICALLY_NULL = "NUMERICALLY_NULL_WITH_BOUNDED_ERROR"
QACC_DERIVATIVE_MATERIAL_BUT_UNIDENTIFIABLE = "MATERIAL_BUT_UNIDENTIFIABLE"
QACC_DERIVATIVE_PRIOR_GATE_DEFECT = "PRIOR_GATE_DEFECT"
TANGENT_LAYOUT_ID = "LCMJ-V1-TANGENT-STATE-132-1.0.0"
SNAPSHOT_SCHEMA_ID = "LCMJ-V1-MACRO-SNAPSHOT-1.0.0"
CONSTRAINT_CATALOG_ID = "LCMJ-V1-ORACLE-CONSTRAINT-CATALOG-1.0.0"
CENTRAL_DIFFERENCE_SCHEME = "central_boxminus_at_common_y0"
_ACTION_BOUND = 1.0
_KINK_TOLERANCE = 1.0e-10

ACTION_BRANCH_INTERIOR = "INTERIOR"
ACTION_BRANCH_SLEW_ACTIVE = "SLEW_ACTIVE"
ACTION_BRANCH_BOUND_ACTIVE = "ACTION_BOUND_ACTIVE"
ACTION_BRANCH_NEAR_KINK = "NEAR_KINK"

# These are the fixed-mode continuous owner values that are actually exposed
# by the frozen transition/Plant seam.  Discrete event predicates, dwell/order
# metadata, and capturability remain outside this list by contract.
OWNER_OUTPUT_IDS = (
    "ACTION_RAW_BOX",
    "ACTION_ACCEPTED_SLEW",
    "DRIVE_ACTIVATION_RANGE",
    "DRIVE_REALIZED_TORQUE_CAPACITY",
    "JOINT_ANATOMICAL_POSITION",
    "JOINT_ANATOMICAL_RATE",
    "CONTACT_COP_SUPPORT_GEOMETRY",
    "SUPPORT_MARGIN",
    "COM_VELOCITY",
    "LINEAR_MOMENTUM",
    "CENTROIDAL_H",
    "CENTROIDAL_HDOT",
    "ACTIVE_POWER",
    "E3_E4_REVERSAL",
    "E3_E4_HORIZONTAL",
)

# The tangent metadata mixes physical units.  These keys intentionally keep
# translations, rotations, activations, torques, and warm-start accelerations
# separate so a caller must provide a unit-aware step contract.
STATE_STEP_BLOCKS = (
    "configuration_translation",
    "configuration_rotation_joint",
    "cache_so3",
    "qvel_translation",
    "qvel_rotation_joint",
    "drivestate_activation",
    "drivestate_tau_prev",
    "previous_accepted_action",
    "qacc_translation",
    "qacc_rotation_joint",
)
_STATE_BLOCK_INDEXES: tuple[tuple[str, tuple[int, ...]], ...] = (
    ("configuration_translation", tuple(range(0, 3))),
    ("configuration_rotation_joint", tuple(range(3, 21))),
    ("cache_so3", tuple(range(21, 30))),
    ("qvel_translation", tuple(range(30, 33))),
    ("qvel_rotation_joint", tuple(range(33, 51))),
    ("drivestate_activation", tuple(range(51, 81))),
    ("drivestate_tau_prev", tuple(range(81, 96))),
    ("previous_accepted_action", tuple(range(96, 111))),
    ("qacc_translation", tuple(range(111, 114))),
    ("qacc_rotation_joint", tuple(range(114, 132))),
)
_STATE_BLOCK_FOR_INDEX = {
    index: block for block, indexes in _STATE_BLOCK_INDEXES for index in indexes
}


class DerivativeDomainError(ValueError):
    """A requested ordinary derivative leaves the qualified local domain."""

    def __init__(self, message: str, *, reports: Sequence["ColumnQualification"] = ()) -> None:
        super().__init__(message)
        self.reports = tuple(reports)


@dataclass(frozen=True)
class ActiveSetFingerprint:
    """Relevant piecewise regime receipt for one wrapped trajectory."""

    contact_steps: tuple[tuple[tuple[int, int, str, int | None, int, bool], ...], ...]
    prohibited_contact_steps: tuple[bool, ...]
    cop_valid_steps: tuple[tuple[bool, bool], ...]
    support_active_steps: tuple[tuple[bool, bool], ...]
    friction_steps: tuple[bool, ...]
    native_joint_limit_steps: tuple[tuple[int, ...], ...]
    drive_flag_steps: tuple[tuple[tuple[str, tuple[bool, ...]], ...], ...]
    action_branches: tuple[str, ...]

    @property
    def digest(self) -> str:
        payload = {
            "contact_steps": self.contact_steps,
            "prohibited_contact_steps": self.prohibited_contact_steps,
            "cop_valid_steps": self.cop_valid_steps,
            "support_active_steps": self.support_active_steps,
            "friction_steps": self.friction_steps,
            "native_joint_limit_steps": self.native_joint_limit_steps,
            "drive_flag_steps": self.drive_flag_steps,
            "action_branches": self.action_branches,
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class TransitionEvaluation:
    """One exact wrapped transition evaluation and its qualification receipt."""

    next_snapshot: MacroSnapshot
    transition: TransitionResult
    active_set: ActiveSetFingerprint
    restored_previous_action: np.ndarray
    snapshot_digest: str
    next_snapshot_digest: str
    owner_outputs: Mapping[str, np.ndarray]


@dataclass(frozen=True)
class ColumnQualification:
    """Validity and provenance for one central finite-difference column."""

    axis: str
    index: int
    step: float
    block: str
    valid: bool
    reason: str
    active_set_preserved: bool
    plus_fingerprint_digest: str | None
    minus_fingerprint_digest: str | None


@dataclass(frozen=True)
class WrappedLinearization:
    """Qualified local A/B matrices for one base snapshot/action pair."""

    A: np.ndarray
    B: np.ndarray
    accepted_action_jacobian: np.ndarray
    state_validity: np.ndarray
    action_validity: np.ndarray
    state_columns: tuple[ColumnQualification, ...]
    action_columns: tuple[ColumnQualification, ...]
    base_snapshot_digest: str
    base_next_snapshot_digest: str
    base_raw_action: np.ndarray
    base_accepted_action: np.ndarray
    base_active_set: ActiveSetFingerprint
    state_step_metadata: Mapping[str, float]
    action_step_metadata: tuple[float, ...]
    scheme: str
    tangent_layout_id: str
    snapshot_schema_id: str
    transition_owner: str
    constraint_catalog_id: str
    transition_evaluation_count: int
    qacc_derivative_disposition: str = QACC_DERIVATIVE_UNADJUDICATED
    qacc_zero_columns: tuple[int, ...] = ()
    qacc_absolute_error_bound: tuple[tuple[str, float], ...] = ()
    qacc_certificate_evidence_id: str | None = None

    @property
    def a_shape(self) -> tuple[int, int]:
        return tuple(int(x) for x in self.A.shape)

    @property
    def b_shape(self) -> tuple[int, int]:
        return tuple(int(x) for x in self.B.shape)

    @property
    def a_sha256(self) -> str:
        return _array_digest(self.A)

    @property
    def b_sha256(self) -> str:
        return _array_digest(self.B)

    @property
    def p_sha256(self) -> str:
        return _array_digest(self.accepted_action_jacobian)

    @property
    def nonsmooth_state_columns(self) -> tuple[int, ...]:
        return tuple(int(r.index) for r in self.state_columns if not r.valid)

    @property
    def nonsmooth_action_columns(self) -> tuple[int, ...]:
        return tuple(int(r.index) for r in self.action_columns if not r.valid)

    def certify_qacc_numerical_null(
        self,
        *,
        evidence_id: str,
        absolute_error_bound: Mapping[str, float],
    ) -> "WrappedLinearization":
        """Adopt a completed external qacc null certificate.

        This is the sole owner-side operation that can write qacc input
        columns as zero.  It requires all 21 existing central samples to be
        finite and fixed-mode; the numerical certificate itself remains an
        evidence artifact outside this owner.
        """

        if self.qacc_derivative_disposition != QACC_DERIVATIVE_UNADJUDICATED:
            raise DerivativeDomainError("qacc derivative disposition is already sealed")
        if not str(evidence_id).strip():
            raise DerivativeDomainError("qacc null certificate requires an evidence id")
        qacc_indexes = tuple(range(WARMSTART_SLICE.start, WARMSTART_SLICE.stop))
        reports = {int(report.index): report for report in self.state_columns}
        if any(index not in reports for index in qacc_indexes):
            raise DerivativeDomainError("qacc null certificate requires all 21 qacc reports")
        if any(
            not reports[index].valid or not reports[index].active_set_preserved
            for index in qacc_indexes
        ):
            raise DerivativeDomainError("qacc null certificate requires fixed-mode finite qacc reports")
        bounds = tuple(
            sorted((str(key), float(value)) for key, value in absolute_error_bound.items())
        )
        required_bounds = {"qacc_translation", "qacc_rotation_joint"}
        if not required_bounds.issubset({key for key, _ in bounds}):
            raise DerivativeDomainError("qacc null certificate requires translation and rotation/joint bounds")
        if any(not np.isfinite(value) or value <= 0.0 for _, value in bounds):
            raise DerivativeDomainError("qacc null certificate bounds must be finite and positive")
        certified = np.asarray(self.A, dtype=np.float64).copy()
        certified[:, WARMSTART_SLICE] = 0.0
        certified = _readonly_array(certified)
        return replace(
            self,
            A=certified,
            qacc_derivative_disposition=QACC_DERIVATIVE_NUMERICALLY_NULL,
            qacc_zero_columns=qacc_indexes,
            qacc_absolute_error_bound=bounds,
            qacc_certificate_evidence_id=str(evidence_id),
        )


@dataclass(frozen=True)
class OwnerOutputSensitivity:
    """Fixed-mode source-owner sensitivity in the same wrapped domain."""

    owner_id: str
    base_value: np.ndarray
    state_jacobian: np.ndarray
    action_jacobian: np.ndarray
    state_validity: np.ndarray
    action_validity: np.ndarray
    state_columns: tuple[ColumnQualification, ...]
    action_columns: tuple[ColumnQualification, ...]
    base_snapshot_digest: str
    base_next_snapshot_digest: str
    state_step_metadata: Mapping[str, float]
    action_step_metadata: tuple[float, ...]
    scheme: str
    transition_evaluation_count: int

    @property
    def output_shape(self) -> tuple[int, ...]:
        return tuple(int(x) for x in self.base_value.shape)


@dataclass
class _EvaluationWorkspace:
    data: mujoco.MjData
    drive_state: drive.DriveState


def _readonly_array(value: np.ndarray, *, dtype: Any = np.float64) -> np.ndarray:
    result = np.asarray(value, dtype=dtype).copy()
    result.setflags(write=False)
    return result


def _finite_vector(value: Sequence[float] | np.ndarray, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).reshape(size).copy()
    if not np.isfinite(result).all():
        raise DerivativeDomainError(f"{name} contains NaN or Inf")
    return result


def _array_digest(value: np.ndarray) -> str:
    array = np.ascontiguousarray(np.asarray(value))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(json.dumps(array.shape, separators=(",", ":")).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def snapshot_digest(snapshot: MacroSnapshot) -> str:
    """Hash all raw snapshot fields for deterministic fixture provenance."""

    digest = hashlib.sha256()
    digest.update(snapshot.schema_version.encode("utf-8"))
    digest.update(snapshot.model_id.encode("utf-8"))
    digest.update(snapshot.model_revision.encode("utf-8"))
    digest.update(snapshot.mujoco_version.encode("utf-8"))
    digest.update(snapshot.model_xml_sha256.encode("ascii"))
    for value in (
        snapshot.qpos,
        snapshot.qvel,
        snapshot.qacc_warmstart,
        snapshot.ctrl,
        snapshot.kinematic_body_ids,
        snapshot.kinematic_xmat,
        snapshot.a_plus,
        snapshot.a_minus,
        snapshot.tau_prev,
        snapshot.previous_command,
        snapshot.reversal_phase,
        snapshot.previous_accepted_action,
        snapshot.dimensions,
        snapshot.mujoco_state_mask,
        snapshot.mujoco_state_size,
        snapshot.integrator,
        snapshot.solver,
        snapshot.solver_iterations,
        snapshot.solver_ls_iterations,
    ):
        if isinstance(value, np.ndarray):
            digest.update(str(value.dtype).encode("ascii"))
            digest.update(np.ascontiguousarray(value).tobytes(order="C"))
        else:
            digest.update(json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8"))
    digest.update(np.asarray(snapshot.time, dtype=np.float64).tobytes())
    for key, flags in snapshot.override_flags:
        digest.update(key.encode("utf-8"))
        digest.update(np.ascontiguousarray(flags).tobytes(order="C"))
    return digest.hexdigest()


def classify_action_projection(
    previous_accepted_action: Sequence[float] | np.ndarray,
    raw_action: Sequence[float] | np.ndarray,
    *,
    tolerance: float = _KINK_TOLERANCE,
) -> tuple[str, ...]:
    """Classify each channel using the source projection's exact branches."""

    previous = _finite_vector(previous_accepted_action, ACTION_DIM, "previous accepted action")
    raw = _finite_vector(raw_action, ACTION_DIM, "raw action")
    if np.any(np.abs(previous) > _ACTION_BOUND + tolerance):
        raise DerivativeDomainError("previous accepted action leaves [-1, 1]")
    if np.any(np.abs(raw) > _ACTION_BOUND + tolerance):
        raise DerivativeDomainError("raw action leaves [-1, 1]")
    delta = raw - previous
    result: list[str] = []
    for raw_value, delta_value in zip(raw, delta, strict=True):
        near_bound = abs(abs(float(raw_value)) - _ACTION_BOUND) <= tolerance
        near_slew = abs(abs(float(delta_value)) - ACCEPTED_ACTION_MAX_STEP) <= tolerance
        if near_bound and near_slew:
            result.append(ACTION_BRANCH_NEAR_KINK)
        elif near_bound:
            result.append(ACTION_BRANCH_BOUND_ACTIVE)
        elif near_slew:
            result.append(ACTION_BRANCH_NEAR_KINK)
        elif abs(float(delta_value)) > ACCEPTED_ACTION_MAX_STEP + tolerance:
            result.append(ACTION_BRANCH_SLEW_ACTIVE)
        else:
            result.append(ACTION_BRANCH_INTERIOR)
    return tuple(result)


def _state_step_vector(state_steps: Mapping[str, float]) -> np.ndarray:
    missing = [key for key in STATE_STEP_BLOCKS if key not in state_steps]
    extra = [key for key in state_steps if key not in STATE_STEP_BLOCKS]
    if missing or extra:
        raise DerivativeDomainError(
            f"state step metadata keys mismatch; missing={missing}, extra={extra}"
        )
    result = np.empty(TANGENT_DIMENSION, dtype=np.float64)
    for block, indexes in _STATE_BLOCK_INDEXES:
        step = float(state_steps[block])
        if not np.isfinite(step) or step <= 0.0:
            raise DerivativeDomainError(f"state step for {block} must be positive and finite")
        result[list(indexes)] = step
    return result


def _action_step_vector(action_steps: float | Sequence[float] | np.ndarray) -> np.ndarray:
    result = np.asarray(action_steps, dtype=np.float64)
    if result.ndim == 0:
        result = np.full(ACTION_DIM, float(result), dtype=np.float64)
    else:
        result = result.reshape(ACTION_DIM).copy()
    if not np.isfinite(result).all() or np.any(result <= 0.0):
        raise DerivativeDomainError("action steps must be positive and finite")
    return result


def _native_joint_limits(data: mujoco.MjData) -> tuple[int, ...]:
    limit_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    return tuple(
        int(index)
        for index in range(int(data.nefc))
        if int(data.efc_type[index]) == limit_type
    )


def _step_fingerprint(
    plant: Plant,
    data: mujoco.MjData,
    drive_result: Mapping[str, Any],
) -> tuple[
    tuple[tuple[int, int, str, int | None, int, bool], ...],
    bool,
    tuple[bool, bool],
    tuple[bool, bool],
    bool,
    tuple[int, ...],
    tuple[tuple[str, tuple[bool, ...]], ...],
]:
    summary = plant.contact_wrench_summary(data)
    contacts = tuple(
        (
            int(entry["geom1"]),
            int(entry["geom2"]),
            str(entry["region"]),
            None if entry["designated_foot"] is None else int(entry["designated_foot"]),
            int(entry["row"]),
            bool(entry["prohibited_contact"]),
        )
        for entry in summary["contacts"]
    )
    cop_valid = tuple(bool(value) for value in np.asarray(summary["cop_valid"], dtype=bool).reshape(2))
    support_active = tuple(
        bool(value) for value in np.asarray(summary["active_by_foot"], dtype=bool).reshape(2)
    )
    flags = tuple(
        (str(key), tuple(bool(value) for value in np.asarray(values, dtype=bool).reshape(ACTION_DIM)))
        for key, values in sorted(dict(drive_result["override_flags"]).items())
    )
    return (
        contacts,
        bool(summary["prohibited_contact"]),
        cop_valid,
        support_active,
        bool(summary["friction_feasible"]),
        _native_joint_limits(data),
        flags,
    )


def _make_active_set(
    *,
    steps: Sequence[
        tuple[
            tuple[tuple[int, int, str, int | None, int, bool], ...],
            bool,
            tuple[bool, bool],
            tuple[bool, bool],
            bool,
            tuple[int, ...],
            tuple[tuple[str, tuple[bool, ...]], ...],
        ]
    ],
    action_branches: tuple[str, ...],
) -> ActiveSetFingerprint:
    return ActiveSetFingerprint(
        contact_steps=tuple(step[0] for step in steps),
        prohibited_contact_steps=tuple(step[1] for step in steps),
        cop_valid_steps=tuple(step[2] for step in steps),
        support_active_steps=tuple(step[3] for step in steps),
        friction_steps=tuple(step[4] for step in steps),
        native_joint_limit_steps=tuple(step[5] for step in steps),
        drive_flag_steps=tuple(step[6] for step in steps),
        action_branches=action_branches,
    )


def _owner_output_values(
    *,
    plant: Plant,
    data: mujoco.MjData,
    raw_action: np.ndarray,
    accepted_action: np.ndarray,
    drive_result: Mapping[str, Any],
    power: Mapping[str, float],
    owner_ids: Sequence[str],
) -> dict[str, np.ndarray]:
    """Read approved continuous owner values at the final substep."""

    unknown = sorted(set(owner_ids).difference(OWNER_OUTPUT_IDS))
    if unknown:
        raise DerivativeDomainError(f"owner output is not approved: {unknown}")
    summary: Mapping[str, Any] | None = None
    com_velocity: np.ndarray | None = None
    com: np.ndarray | None = None

    def contact_summary() -> Mapping[str, Any]:
        nonlocal summary
        if summary is None:
            summary = plant.contact_wrench_summary(data)
        return summary

    def get_com_velocity() -> np.ndarray:
        nonlocal com_velocity
        if com_velocity is None:
            com_velocity = np.asarray(plant.center_of_mass_velocity(data), dtype=np.float64)
        return com_velocity

    def get_com() -> np.ndarray:
        nonlocal com
        if com is None:
            com = np.asarray(plant.center_of_mass(data), dtype=np.float64)
        return com

    result: dict[str, np.ndarray] = {}
    for owner_id in owner_ids:
        if owner_id == "ACTION_RAW_BOX":
            value = raw_action
        elif owner_id == "ACTION_ACCEPTED_SLEW":
            value = accepted_action
        elif owner_id == "DRIVE_ACTIVATION_RANGE":
            value = np.concatenate((drive_result["a_plus"], drive_result["a_minus"]))
        elif owner_id == "DRIVE_REALIZED_TORQUE_CAPACITY":
            value = np.concatenate((drive_result["capacity_lower"], drive_result["capacity_upper"]))
        elif owner_id == "JOINT_ANATOMICAL_POSITION":
            value = plant.anatomical_coordinates(data)
        elif owner_id == "JOINT_ANATOMICAL_RATE":
            value = plant.anatomical_rates(data)
        elif owner_id == "CONTACT_COP_SUPPORT_GEOMETRY":
            value = contact_summary()["cop_world_xy"]
        elif owner_id in ("SUPPORT_MARGIN", "E3_E4_HORIZONTAL"):
            value = np.asarray(
                [
                    plant.support_margin(
                        data,
                        get_com(),
                        tuple(bool(x) for x in contact_summary()["active_by_foot"]),
                    )
                ],
                dtype=np.float64,
            )
        elif owner_id == "COM_VELOCITY":
            value = get_com_velocity()
        elif owner_id == "LINEAR_MOMENTUM":
            value = plant.linear_momentum(data)
        elif owner_id == "CENTROIDAL_H":
            value = plant.centroidal_angular_momentum(data, get_com())
        elif owner_id == "CENTROIDAL_HDOT":
            value = plant.centroidal_hdot_from_external_wrench(data, get_com())
        elif owner_id == "ACTIVE_POWER":
            value = np.asarray([power["active_power_signed_W"]], dtype=np.float64)
        elif owner_id == "E3_E4_REVERSAL":
            value = np.asarray([get_com_velocity()[2]], dtype=np.float64)
        else:  # pragma: no cover - guarded by the approved-ID check above.
            raise DerivativeDomainError(f"owner output is not implemented: {owner_id}")
        result[owner_id] = _readonly_array(np.asarray(value, dtype=np.float64).reshape(-1))
    return result


def _workspace(plant: Plant) -> _EvaluationWorkspace:
    return _EvaluationWorkspace(mujoco.MjData(plant.model), drive.DriveState())


def evaluate_wrapped_step_5ms(
    *,
    plant: Plant,
    snapshot: MacroSnapshot,
    raw_action: Sequence[float] | np.ndarray,
    workspace: _EvaluationWorkspace | None = None,
    owner_ids: Sequence[str] = (),
) -> TransitionEvaluation:
    """Restore one snapshot and run the authoritative exact transition once."""

    raw = _finite_vector(raw_action, ACTION_DIM, "raw action")
    owner_ids = tuple(str(owner_id) for owner_id in owner_ids)
    branches = classify_action_projection(snapshot.previous_accepted_action, raw)
    active_steps: list[Any] = []
    final_owner_outputs: dict[str, np.ndarray] = {}
    work = _workspace(plant) if workspace is None else workspace
    restored_previous = snapshot.restore(
        plant=plant,
        data=work.data,
        drive_state=work.drive_state,
    )

    def record_substep(
        _substep: int,
        _accepted: np.ndarray,
        drive_result: Mapping[str, Any],
        _realized: np.ndarray,
        power: Mapping[str, float],
    ) -> bool:
        active_steps.append(_step_fingerprint(plant, work.data, drive_result))
        if _substep == 39 and owner_ids:
            final_owner_outputs.update(
                _owner_output_values(
                    plant=plant,
                    data=work.data,
                    raw_action=raw,
                    accepted_action=_accepted,
                    drive_result=drive_result,
                    power=power,
                    owner_ids=owner_ids,
                )
            )
        return True

    transition = step_5ms(
        plant=plant,
        data=work.data,
        drive_state=work.drive_state,
        previous_accepted_action=restored_previous,
        raw_action=raw,
        on_substep=record_substep,
    )
    if transition.substeps_executed != 40 or len(active_steps) != 40:
        raise DerivativeDomainError("wrapped transition did not execute exactly 40 substeps")
    next_snapshot = MacroSnapshot.capture(
        plant=plant,
        data=work.data,
        drive_state=work.drive_state,
        previous_accepted_action=transition.accepted_action,
    )
    return TransitionEvaluation(
        next_snapshot=next_snapshot,
        transition=transition,
        active_set=_make_active_set(steps=active_steps, action_branches=branches),
        restored_previous_action=_readonly_array(restored_previous),
        snapshot_digest=snapshot_digest(snapshot),
        next_snapshot_digest=snapshot_digest(next_snapshot),
        owner_outputs=final_owner_outputs,
    )


def _column_result(
    *,
    axis: str,
    index: int,
    step: float,
    base: TransitionEvaluation,
    plus: TransitionEvaluation | None,
    minus: TransitionEvaluation | None,
    block: str,
    reason: str = "",
) -> ColumnQualification:
    plus_digest = None if plus is None else plus.active_set.digest
    minus_digest = None if minus is None else minus.active_set.digest
    preserved = (
        plus is not None
        and minus is not None
        and plus.active_set == base.active_set
        and minus.active_set == base.active_set
    )
    if reason:
        valid = False
    elif not preserved:
        valid = False
        reason = "NONSMOOTH_ACTIVE_SET_CROSSING"
    else:
        valid = True
        reason = "SMOOTH_FIXED_ACTIVE_SET"
    return ColumnQualification(
        axis=axis,
        index=int(index),
        step=float(step),
        block=block,
        valid=valid,
        reason=reason,
        active_set_preserved=preserved,
        plus_fingerprint_digest=plus_digest,
        minus_fingerprint_digest=minus_digest,
    )


def _raise_invalid_columns(
    reports: Sequence[ColumnQualification],
    *,
    allow_nonsmooth: bool,
) -> None:
    invalid = tuple(report for report in reports if not report.valid)
    if invalid and not allow_nonsmooth:
        summary = ", ".join(f"{r.axis}[{r.index}]={r.reason}" for r in invalid)
        raise DerivativeDomainError(f"ordinary central derivative rejected: {summary}", reports=invalid)


def linearize_step_5ms(
    *,
    plant: Plant,
    base_snapshot: MacroSnapshot,
    raw_action: Sequence[float] | np.ndarray,
    state_steps: Mapping[str, float],
    action_steps: float | Sequence[float] | np.ndarray,
    state_columns: Sequence[int] | None = None,
    action_columns: Sequence[int] | None = None,
    allow_nonsmooth: bool = False,
) -> WrappedLinearization:
    """Finite-difference the exact wrapped transition in the frozen tangent frame.

    Every plus/minus evaluation restores its own snapshot into the same fresh
    workspace.  Both output perturbations are computed with ``boxminus`` at the
    single base next-state frame before their centered difference is formed.
    """

    raw = _finite_vector(raw_action, ACTION_DIM, "raw action")
    step_vector = _state_step_vector(state_steps)
    action_step_vector = _action_step_vector(action_steps)
    state_indexes = tuple(range(TANGENT_DIMENSION)) if state_columns is None else tuple(int(i) for i in state_columns)
    action_indexes = tuple(range(ACTION_DIM)) if action_columns is None else tuple(int(i) for i in action_columns)
    if any(index < 0 or index >= TANGENT_DIMENSION for index in state_indexes):
        raise DerivativeDomainError("state column index outside [0, 132)")
    if any(index < 0 or index >= ACTION_DIM for index in action_indexes):
        raise DerivativeDomainError("action column index outside [0, 15)")
    if len(set(state_indexes)) != len(state_indexes) or len(set(action_indexes)) != len(action_indexes):
        raise DerivativeDomainError("duplicate derivative column index")

    work = _workspace(plant)
    base = evaluate_wrapped_step_5ms(
        plant=plant,
        snapshot=base_snapshot,
        raw_action=raw,
        workspace=work,
    )
    base_next = base.next_snapshot
    A = np.full((TANGENT_DIMENSION, TANGENT_DIMENSION), np.nan, dtype=np.float64)
    B = np.full((TANGENT_DIMENSION, ACTION_DIM), np.nan, dtype=np.float64)
    P = np.full((ACTION_DIM, ACTION_DIM), np.nan, dtype=np.float64)
    state_reports: list[ColumnQualification] = []
    action_reports: list[ColumnQualification] = []
    evaluations = 1

    for index in state_indexes:
        h = float(step_vector[index])
        block = _STATE_BLOCK_FOR_INDEX[index]
        delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
        delta[index] = h
        plus_eval: TransitionEvaluation | None = None
        minus_eval: TransitionEvaluation | None = None
        reason = ""
        try:
            plus_snapshot = boxplus(base_snapshot, delta, model=plant.model)
            minus_snapshot = boxplus(base_snapshot, -delta, model=plant.model)
            plus_eval = evaluate_wrapped_step_5ms(
                plant=plant, snapshot=plus_snapshot, raw_action=raw, workspace=work
            )
            minus_eval = evaluate_wrapped_step_5ms(
                plant=plant, snapshot=minus_snapshot, raw_action=raw, workspace=work
            )
            evaluations += 2
        except (DerivativeDomainError, ValueError, drive.DriveModelError) as exc:
            reason = f"PERTURBATION_REJECTED:{type(exc).__name__}"
        report = _column_result(
            axis="A",
            index=index,
            step=h,
            base=base,
            plus=plus_eval,
            minus=minus_eval,
            block=block,
            reason=reason,
        )
        state_reports.append(report)
        if report.valid:
            d_plus = boxminus(plus_eval.next_snapshot, base_next, model=plant.model)
            d_minus = boxminus(minus_eval.next_snapshot, base_next, model=plant.model)
            A[:, index] = (d_plus - d_minus) / (2.0 * h)

    for index in action_indexes:
        h = float(action_step_vector[index])
        branches = base.active_set.action_branches
        reason = ""
        plus_eval = None
        minus_eval = None
        if branches[index] in (ACTION_BRANCH_NEAR_KINK, ACTION_BRANCH_BOUND_ACTIVE):
            reason = f"ACTION_PROJECTION_{branches[index]}"
        else:
            plus_action = raw.copy()
            minus_action = raw.copy()
            plus_action[index] += h
            minus_action[index] -= h
            if np.any(np.abs(plus_action) > _ACTION_BOUND) or np.any(np.abs(minus_action) > _ACTION_BOUND):
                reason = "ACTION_BOX_BOUND_CROSSED"
            else:
                try:
                    plus_eval = evaluate_wrapped_step_5ms(
                        plant=plant, snapshot=base_snapshot, raw_action=plus_action, workspace=work
                    )
                    minus_eval = evaluate_wrapped_step_5ms(
                        plant=plant, snapshot=base_snapshot, raw_action=minus_action, workspace=work
                    )
                    evaluations += 2
                except (DerivativeDomainError, ValueError, drive.DriveModelError) as exc:
                    reason = f"PERTURBATION_REJECTED:{type(exc).__name__}"
        report = _column_result(
            axis="B",
            index=index,
            step=h,
            base=base,
            plus=plus_eval,
            minus=minus_eval,
            block="raw_action",
            reason=reason,
        )
        action_reports.append(report)
        if report.valid:
            d_plus = boxminus(plus_eval.next_snapshot, base_next, model=plant.model)
            d_minus = boxminus(minus_eval.next_snapshot, base_next, model=plant.model)
            B[:, index] = (d_plus - d_minus) / (2.0 * h)

    for index in action_indexes:
        h = float(action_step_vector[index])
        if base.active_set.action_branches[index] in (
            ACTION_BRANCH_NEAR_KINK,
            ACTION_BRANCH_BOUND_ACTIVE,
        ):
            continue
        plus_action = raw.copy()
        minus_action = raw.copy()
        plus_action[index] += h
        minus_action[index] -= h
        if np.any(np.abs(plus_action) > _ACTION_BOUND) or np.any(np.abs(minus_action) > _ACTION_BOUND):
            continue
        plus_accepted = project_accepted_action(base_snapshot.previous_accepted_action, plus_action)
        minus_accepted = project_accepted_action(base_snapshot.previous_accepted_action, minus_action)
        P[:, index] = (plus_accepted - minus_accepted) / (2.0 * h)

    _raise_invalid_columns((*state_reports, *action_reports), allow_nonsmooth=allow_nonsmooth)
    selected_state_finite = all(np.isfinite(A[:, index]).all() for index in state_indexes)
    selected_action_finite = all(np.isfinite(B[:, index]).all() for index in action_indexes)
    if not allow_nonsmooth and not (selected_state_finite and selected_action_finite):
        raise DerivativeDomainError("qualified wrapped derivative contains NaN or Inf")

    A = _readonly_array(A)
    B = _readonly_array(B)
    P = _readonly_array(P)
    state_validity_values = np.zeros(TANGENT_DIMENSION, dtype=bool)
    for report in state_reports:
        state_validity_values[report.index] = report.valid
    action_validity_values = np.zeros(ACTION_DIM, dtype=bool)
    for report in action_reports:
        action_validity_values[report.index] = report.valid
    state_validity = _readonly_array(state_validity_values, dtype=bool)
    action_validity = _readonly_array(action_validity_values, dtype=bool)
    return WrappedLinearization(
        A=A,
        B=B,
        accepted_action_jacobian=P,
        state_validity=state_validity,
        action_validity=action_validity,
        state_columns=tuple(state_reports),
        action_columns=tuple(action_reports),
        base_snapshot_digest=base.snapshot_digest,
        base_next_snapshot_digest=base.next_snapshot_digest,
        base_raw_action=_readonly_array(raw),
        base_accepted_action=_readonly_array(base.transition.accepted_action),
        base_active_set=base.active_set,
        state_step_metadata={key: float(state_steps[key]) for key in STATE_STEP_BLOCKS},
        action_step_metadata=tuple(float(value) for value in action_step_vector),
        scheme=CENTRAL_DIFFERENCE_SCHEME,
        tangent_layout_id=TANGENT_LAYOUT_ID,
        snapshot_schema_id=SNAPSHOT_SCHEMA_ID,
        transition_owner="src/loaded_cmj/simulation/transition.py:step_5ms",
        constraint_catalog_id=CONSTRAINT_CATALOG_ID,
        transition_evaluation_count=evaluations,
    )


def differentiate_owner_output(
    *,
    plant: Plant,
    base_snapshot: MacroSnapshot,
    raw_action: Sequence[float] | np.ndarray,
    owner_id: str,
    state_steps: Mapping[str, float],
    action_steps: float | Sequence[float] | np.ndarray,
    state_columns: Sequence[int] | None = None,
    action_columns: Sequence[int] | None = None,
    allow_nonsmooth: bool = False,
) -> OwnerOutputSensitivity:
    """Differentiate one ML-240-approved continuous source-owner value.

    The owner is sampled during the final substep of the same wrapped
    ``step_5ms`` evaluation used by the A/B owner.  This function is kept
    explicit and narrow so it cannot become a second transition or a generic
    output-Jacobian framework.
    """

    if owner_id not in OWNER_OUTPUT_IDS:
        raise DerivativeDomainError(f"owner output is not approved: {owner_id}")
    raw = _finite_vector(raw_action, ACTION_DIM, "raw action")
    step_vector = _state_step_vector(state_steps)
    action_step_vector = _action_step_vector(action_steps)
    state_indexes = tuple(range(TANGENT_DIMENSION)) if state_columns is None else tuple(int(i) for i in state_columns)
    action_indexes = tuple(range(ACTION_DIM)) if action_columns is None else tuple(int(i) for i in action_columns)
    if any(index < 0 or index >= TANGENT_DIMENSION for index in state_indexes):
        raise DerivativeDomainError("state owner-output column index outside [0, 132)")
    if any(index < 0 or index >= ACTION_DIM for index in action_indexes):
        raise DerivativeDomainError("action owner-output column index outside [0, 15)")
    if len(set(state_indexes)) != len(state_indexes) or len(set(action_indexes)) != len(action_indexes):
        raise DerivativeDomainError("duplicate owner-output derivative column index")

    work = _workspace(plant)
    base = evaluate_wrapped_step_5ms(
        plant=plant,
        snapshot=base_snapshot,
        raw_action=raw,
        workspace=work,
        owner_ids=(owner_id,),
    )
    base_value = np.asarray(base.owner_outputs[owner_id], dtype=np.float64).reshape(-1)
    if owner_id == "CONTACT_COP_SUPPORT_GEOMETRY" and not all(
        all(step) for step in base.active_set.cop_valid_steps
    ):
        raise DerivativeDomainError("COP owner output is not differentiable while COP validity is false")

    state_shape = (base_value.size, TANGENT_DIMENSION)
    action_shape = (base_value.size, ACTION_DIM)
    state_jacobian = np.full(state_shape, np.nan, dtype=np.float64)
    action_jacobian = np.full(action_shape, np.nan, dtype=np.float64)
    state_reports: list[ColumnQualification] = []
    action_reports: list[ColumnQualification] = []
    evaluations = 1

    for index in state_indexes:
        h = float(step_vector[index])
        delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
        delta[index] = h
        plus_eval: TransitionEvaluation | None = None
        minus_eval: TransitionEvaluation | None = None
        reason = ""
        try:
            plus_snapshot = boxplus(base_snapshot, delta, model=plant.model)
            minus_snapshot = boxplus(base_snapshot, -delta, model=plant.model)
            plus_eval = evaluate_wrapped_step_5ms(
                plant=plant,
                snapshot=plus_snapshot,
                raw_action=raw,
                workspace=work,
                owner_ids=(owner_id,),
            )
            minus_eval = evaluate_wrapped_step_5ms(
                plant=plant,
                snapshot=minus_snapshot,
                raw_action=raw,
                workspace=work,
                owner_ids=(owner_id,),
            )
            evaluations += 2
        except (DerivativeDomainError, ValueError, drive.DriveModelError) as exc:
            reason = f"PERTURBATION_REJECTED:{type(exc).__name__}"
        report = _column_result(
            axis="OWNER_STATE",
            index=index,
            step=h,
            base=base,
            plus=plus_eval,
            minus=minus_eval,
            block=_STATE_BLOCK_FOR_INDEX[index],
            reason=reason,
        )
        state_reports.append(report)
        if report.valid:
            state_jacobian[:, index] = (
                np.asarray(plus_eval.owner_outputs[owner_id])
                - np.asarray(minus_eval.owner_outputs[owner_id])
            ) / (2.0 * h)

    for index in action_indexes:
        h = float(action_step_vector[index])
        reason = ""
        plus_eval = None
        minus_eval = None
        if base.active_set.action_branches[index] in (
            ACTION_BRANCH_NEAR_KINK,
            ACTION_BRANCH_BOUND_ACTIVE,
        ):
            reason = f"ACTION_PROJECTION_{base.active_set.action_branches[index]}"
        else:
            plus_action = raw.copy()
            minus_action = raw.copy()
            plus_action[index] += h
            minus_action[index] -= h
            if np.any(np.abs(plus_action) > _ACTION_BOUND) or np.any(np.abs(minus_action) > _ACTION_BOUND):
                reason = "ACTION_BOX_BOUND_CROSSED"
            else:
                try:
                    plus_eval = evaluate_wrapped_step_5ms(
                        plant=plant,
                        snapshot=base_snapshot,
                        raw_action=plus_action,
                        workspace=work,
                        owner_ids=(owner_id,),
                    )
                    minus_eval = evaluate_wrapped_step_5ms(
                        plant=plant,
                        snapshot=base_snapshot,
                        raw_action=minus_action,
                        workspace=work,
                        owner_ids=(owner_id,),
                    )
                    evaluations += 2
                except (DerivativeDomainError, ValueError, drive.DriveModelError) as exc:
                    reason = f"PERTURBATION_REJECTED:{type(exc).__name__}"
        report = _column_result(
            axis="OWNER_ACTION",
            index=index,
            step=h,
            base=base,
            plus=plus_eval,
            minus=minus_eval,
            block="raw_action",
            reason=reason,
        )
        action_reports.append(report)
        if report.valid:
            action_jacobian[:, index] = (
                np.asarray(plus_eval.owner_outputs[owner_id])
                - np.asarray(minus_eval.owner_outputs[owner_id])
            ) / (2.0 * h)

    _raise_invalid_columns((*state_reports, *action_reports), allow_nonsmooth=allow_nonsmooth)
    selected_state_finite = all(
        np.isfinite(state_jacobian[:, index]).all() for index in state_indexes
    )
    selected_action_finite = all(
        np.isfinite(action_jacobian[:, index]).all() for index in action_indexes
    )
    if not allow_nonsmooth and not (selected_state_finite and selected_action_finite):
        raise DerivativeDomainError("qualified owner-output sensitivity contains NaN or Inf")
    state_validity_values = np.zeros(TANGENT_DIMENSION, dtype=bool)
    for report in state_reports:
        state_validity_values[report.index] = report.valid
    action_validity_values = np.zeros(ACTION_DIM, dtype=bool)
    for report in action_reports:
        action_validity_values[report.index] = report.valid
    return OwnerOutputSensitivity(
        owner_id=owner_id,
        base_value=_readonly_array(base_value),
        state_jacobian=_readonly_array(state_jacobian),
        action_jacobian=_readonly_array(action_jacobian),
        state_validity=_readonly_array(state_validity_values, dtype=bool),
        action_validity=_readonly_array(action_validity_values, dtype=bool),
        state_columns=tuple(state_reports),
        action_columns=tuple(action_reports),
        base_snapshot_digest=base.snapshot_digest,
        base_next_snapshot_digest=base.next_snapshot_digest,
        state_step_metadata={key: float(state_steps[key]) for key in STATE_STEP_BLOCKS},
        action_step_metadata=tuple(float(value) for value in action_step_vector),
        scheme=CENTRAL_DIFFERENCE_SCHEME,
        transition_evaluation_count=evaluations,
    )


__all__ = [
    "ACTION_BRANCH_BOUND_ACTIVE",
    "ACTION_BRANCH_INTERIOR",
    "ACTION_BRANCH_NEAR_KINK",
    "ACTION_BRANCH_SLEW_ACTIVE",
    "ActiveSetFingerprint",
    "CENTRAL_DIFFERENCE_SCHEME",
    "CONSTRAINT_CATALOG_ID",
    "DERIVATIVE_API_VERSION",
    "QACC_DERIVATIVE_MATERIAL_BUT_UNIDENTIFIABLE",
    "QACC_DERIVATIVE_NUMERICALLY_NULL",
    "QACC_DERIVATIVE_PRIOR_GATE_DEFECT",
    "QACC_DERIVATIVE_QUALIFIED_NONZERO",
    "QACC_DERIVATIVE_UNADJUDICATED",
    "DerivativeDomainError",
    "ColumnQualification",
    "OWNER_OUTPUT_IDS",
    "OwnerOutputSensitivity",
    "SNAPSHOT_SCHEMA_ID",
    "STATE_STEP_BLOCKS",
    "TANGENT_LAYOUT_ID",
    "TransitionEvaluation",
    "WrappedLinearization",
    "classify_action_projection",
    "differentiate_owner_output",
    "evaluate_wrapped_step_5ms",
    "linearize_step_5ms",
    "snapshot_digest",
]
