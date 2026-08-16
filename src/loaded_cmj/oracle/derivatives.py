"""Privileged local derivatives of frozen transition and state-owner maps.

Transition-wrapped derivatives retain the exact 5 ms wrapper.  Direct
state-owner derivatives restore a MacroSnapshot and use only ``mj_forward``
before reading the canonical current-state owner.  Both modes delegate state
geometry and restart semantics to their frozen owners.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
from types import MappingProxyType
from typing import Any, Callable, Mapping, Sequence

import mujoco
import numpy as np

from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import ACTION_DIM, SUBSTEPS_PER_CONTROL
from loaded_cmj.simulation.plant import Plant, SupportMarginBranchCertificate
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
    TorqueVelocitySchedule,
)


DERIVATIVE_API_VERSION = "LCMJ-V1-WRAPPED-DERIVATIVE-1.0.0"
DIRECT_STATE_OWNER = "DIRECT_STATE_OWNER"
TRANSITION_WRAPPED_OWNER = "TRANSITION_WRAPPED_OWNER"
QACC_DERIVATIVE_UNADJUDICATED = "UNADJUDICATED"
QACC_DERIVATIVE_QUALIFIED_NONZERO = "QUALIFIED_NONZERO"
QACC_DERIVATIVE_NUMERICALLY_NULL = "NUMERICALLY_NULL_WITH_BOUNDED_ERROR"
QACC_DERIVATIVE_MATERIAL_BUT_UNIDENTIFIABLE = "MATERIAL_BUT_UNIDENTIFIABLE"
QACC_DERIVATIVE_PRIOR_GATE_DEFECT = "PRIOR_GATE_DEFECT"
TANGENT_LAYOUT_ID = "LCMJ-V1-TANGENT-STATE-132-1.0.0"
SNAPSHOT_SCHEMA_ID = "LCMJ-V1-MACRO-SNAPSHOT-1.0.0"
CONSTRAINT_CATALOG_ID = "LCMJ-V1-ORACLE-CONSTRAINT-CATALOG-1.0.0"
CENTRAL_DIFFERENCE_SCHEME = "central_boxminus_at_common_y0"
QACC_ZERO_COLUMNS = tuple(range(WARMSTART_SLICE.start, WARMSTART_SLICE.stop))
QACC_ERROR_BOUNDS = MappingProxyType(
    {
        "qacc_translation": 1.0e-7,
        "qacc_rotation_joint": 1.0e-7,
    }
)
QACC_CERTIFICATE_EVIDENCE_ID = "ML241-20260810T222908Z-qacc-null-certificate"
_ACTION_BOUND = 1.0
_KINK_TOLERANCE = 1.0e-10
_NATIVE_DOMAIN_TOLERANCE = 1.0e-12
_TORQUE_VELOCITY_NORMALIZED_NEAR_ZERO = 1.0e-6

# The first seven values are the primary stencil/qualification classes.  The
# remaining values are deliberately more specific rejection diagnostics.
CENTRAL_INTERIOR = "CENTRAL_INTERIOR"
FORWARD_FEASIBLE_SIDE = "FORWARD_FEASIBLE_SIDE"
BACKWARD_FEASIBLE_SIDE = "BACKWARD_FEASIBLE_SIDE"
FIXED_ELIMINATED = "FIXED_ELIMINATED"
TRUE_KINK_INVALID = "TRUE_KINK_INVALID"
PHYSICAL_CONTACT_SWITCH_INVALID = "PHYSICAL_CONTACT_SWITCH_INVALID"
MANIFOLD_INVALID = "MANIFOLD_INVALID"
NUMERICAL_CERTIFICATE_ONLY = "NUMERICAL_CERTIFICATE_ONLY"
PIECEWISE_BRANCH_SWITCH_INVALID = "PIECEWISE_BRANCH_SWITCH_INVALID"
NATIVE_DOMAIN_INVALID = "NATIVE_DOMAIN_INVALID"
NONFINITE_EVALUATION = "NONFINITE_EVALUATION"
SCHEDULED_MODE_SMOOTH = "SMOOTH_FIXED_MODE_SCHEDULE"
SUPPORT_HULL_KINK_INVALID = "SUPPORT_HULL_KINK_INVALID"
SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID = "SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID"
SUPPORT_HULL_PROJECTION_BRANCH_SWITCH_INVALID = "SUPPORT_HULL_PROJECTION_BRANCH_SWITCH_INVALID"
SUPPORT_ACTIVE_SET_SWITCH_INVALID = "SUPPORT_ACTIVE_SET_SWITCH_INVALID"

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
_SUPPORT_GEOMETRY_OWNER_IDS = frozenset({"SUPPORT_MARGIN", "E3_E4_HORIZONTAL"})
_DIRECT_STATE_OWNER_IDS = frozenset(
    {"SUPPORT_MARGIN", "E3_E4_HORIZONTAL", "E3_E4_REVERSAL"}
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
    native_joint_limit_steps: tuple[bool, ...]
    drive_flag_steps: tuple[tuple[tuple[str, tuple[bool, ...]], ...], ...]
    action_branches: tuple[str, ...]
    drive_branch_steps: tuple[tuple[tuple[str, tuple[str, ...]], ...], ...] = ()
    transition_branch_steps: tuple[tuple[tuple[str, tuple[str, ...]], ...], ...] = ()

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
            "drive_branch_steps": self.drive_branch_steps,
            "transition_branch_steps": self.transition_branch_steps,
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
    support_margin_branch_steps: tuple[SupportMarginBranchCertificate, ...] = ()
    drive_guard_steps: tuple[tuple[tuple[float, ...], tuple[float, ...]], ...] = ()


@dataclass(frozen=True)
class StateOwnerEvaluation:
    """One direct current-state owner evaluation and its branch receipt."""

    owner_id: str
    value: np.ndarray
    owner_outputs: Mapping[str, np.ndarray]
    active_set: ActiveSetFingerprint
    support_margin_branch_steps: tuple[SupportMarginBranchCertificate, ...]
    snapshot_digest: str
    owner_output_digest: str
    time: float
    qacc_warmstart_digest: str
    evaluation_mode: str = DIRECT_STATE_OWNER


@dataclass(frozen=True)
class ColumnQualification:
    """Validity and provenance for one requested derivative column."""

    axis: str
    index: int
    step: float
    block: str
    valid: bool
    reason: str
    active_set_preserved: bool
    plus_fingerprint_digest: str | None
    minus_fingerprint_digest: str | None
    stencil: str = CENTRAL_INTERIOR
    native_domain_legal: bool = True
    base_fingerprint_digest: str | None = None
    sample_fingerprint_digests: tuple[str, ...] = ()

    @property
    def classification(self) -> str:
        """Compatibility/readability alias for the primary stencil class."""

        return self.stencil


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
        qacc_indexes = QACC_ZERO_COLUMNS
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
        bound_by_name = dict(bounds)
        qacc_values = np.asarray(self.A, dtype=np.float64)[:, WARMSTART_SLICE]
        if not np.isfinite(qacc_values).all():
            raise DerivativeDomainError("qacc null certificate requires finite current qacc derivatives")
        observed_bounds = {
            "qacc_translation": float(np.max(np.abs(qacc_values[:, :3]))),
            "qacc_rotation_joint": float(np.max(np.abs(qacc_values[:, 3:]))),
        }
        exceeded = tuple(
            key
            for key, observed in observed_bounds.items()
            if observed > bound_by_name[key]
        )
        if exceeded:
            details = ", ".join(
                f"{key}={observed_bounds[key]:.17g}>{bound_by_name[key]:.17g}"
                for key in exceeded
            )
            raise DerivativeDomainError(
                f"qacc current derivative exceeds authorized error bound: {details}"
            )
        certified = np.asarray(self.A, dtype=np.float64).copy()
        certified[:, WARMSTART_SLICE] = 0.0
        certified = _readonly_array(certified)
        qualified_reports = tuple(
            replace(
                report,
                stencil=(
                    NUMERICAL_CERTIFICATE_ONLY
                    if WARMSTART_SLICE.start <= int(report.index) < WARMSTART_SLICE.stop
                    else report.stencil
                ),
            )
            for report in self.state_columns
        )
        return replace(
            self,
            A=certified,
            state_columns=qualified_reports,
            qacc_derivative_disposition=QACC_DERIVATIVE_NUMERICALLY_NULL,
            qacc_zero_columns=qacc_indexes,
            qacc_absolute_error_bound=bounds,
            qacc_certificate_evidence_id=str(evidence_id),
        )


@dataclass(frozen=True)
class OwnerOutputSensitivity:
    """Fixed-mode source-owner sensitivity with explicit evaluation mode."""

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
    evaluation_mode: str = TRANSITION_WRAPPED_OWNER
    owner_output_digest: str = ""
    base_branch_certificate: SupportMarginBranchCertificate | None = None
    qacc_derivative_disposition: str = QACC_DERIVATIVE_UNADJUDICATED
    qacc_zero_columns: tuple[int, ...] = ()
    qacc_absolute_error_bound: tuple[tuple[str, float], ...] = ()

    @property
    def output_shape(self) -> tuple[int, ...]:
        return tuple(int(x) for x in self.base_value.shape)

    @property
    def requested_state_columns(self) -> tuple[int, ...]:
        return tuple(int(report.index) for report in self.state_columns)

    @property
    def requested_action_columns(self) -> tuple[int, ...]:
        return tuple(int(report.index) for report in self.action_columns)


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


def _classify_slew_projection(
    previous_accepted_action: Sequence[float] | np.ndarray,
    raw_action: Sequence[float] | np.ndarray,
    *,
    tolerance: float = _KINK_TOLERANCE,
) -> tuple[str, ...]:
    """Classify only the accepted-action slew branch.

    Raw-box endpoint labels belong to ``classify_action_projection``.  They
    are deliberately kept separate here so a feasible-side raw-box stencil
    cannot be mistaken for a slew-branch crossing.
    """

    previous = _finite_vector(previous_accepted_action, ACTION_DIM, "previous accepted action")
    raw = _finite_vector(raw_action, ACTION_DIM, "raw action")
    delta = raw - previous
    result: list[str] = []
    for delta_value in delta:
        absolute = abs(float(delta_value))
        if abs(absolute - ACCEPTED_ACTION_MAX_STEP) <= tolerance:
            result.append(ACTION_BRANCH_NEAR_KINK)
        elif absolute > ACCEPTED_ACTION_MAX_STEP + tolerance:
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


@dataclass(frozen=True)
class _StencilPlan:
    """A fixed, pre-evaluation finite-difference stencil decision."""

    classification: str
    offsets: tuple[int, ...]
    native_domain_legal: bool = True
    reason: str = ""

    @property
    def valid(self) -> bool:
        return not self.reason


@dataclass(frozen=True)
class _StepFingerprint:
    """Discrete branch state observed at one physics substep.

    This intentionally excludes all continuous margins, forces, penetrations,
    COP coordinates, velocities, and torque magnitudes.  ``native_joint_limit``
    is an owner predicate only; no ``efc`` row/address is retained.
    """

    contacts: tuple[tuple[int, int, str, int | None, int, bool], ...]
    prohibited_contact: bool
    cop_valid: tuple[bool, bool]
    support_active: tuple[bool, bool]
    friction_feasible: bool
    native_joint_limit: bool
    drive_flags: tuple[tuple[str, tuple[bool, ...]], ...]
    drive_branches: tuple[tuple[str, tuple[str, ...]], ...]
    transition_branches: tuple[tuple[str, tuple[str, ...]], ...]
    support_margin_branch: SupportMarginBranchCertificate | None = None
    drive_guard_values: tuple[tuple[float, ...], tuple[float, ...]] = ((), ())


def _native_joint_limits(data: mujoco.MjData) -> bool:
    """Return the physical native-limit predicate without solver row identity.

    MuJoCo's ``efc_type`` is consulted only as the Plant's existing native
    limit-status observable.  The internal constraint row number is not part
    of the certificate, digest, or equality relation.
    """

    limit_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    return bool(
        int(data.nefc) > 0
        and np.any(np.asarray(data.efc_type[: data.nefc], dtype=np.int32) == limit_type)
    )


def _discrete_sign(values: Sequence[float] | np.ndarray, *, tolerance: float) -> tuple[str, ...]:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if not np.isfinite(array).all():
        raise DerivativeDomainError("branch certificate received a non-finite value")
    return tuple(
        "POSITIVE" if value > tolerance else "NEGATIVE" if value < -tolerance else "NEAR_ZERO"
        for value in array
    )


def _flag_labels(values: Sequence[bool] | np.ndarray) -> tuple[str, ...]:
    return tuple("ACTIVE" if bool(value) else "INACTIVE" for value in np.asarray(values, dtype=bool).reshape(-1))


def _step_fingerprint(
    plant: Plant,
    data: mujoco.MjData,
    drive_result: Mapping[str, Any],
    *,
    raw_action: np.ndarray,
    accepted_action: np.ndarray,
    slew_branches: tuple[str, ...],
    old_a_plus: np.ndarray,
    old_a_minus: np.ndarray,
) -> _StepFingerprint:
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
    support_margin_diagnostic = plant.support_margin_diagnostics(
        data,
        plant.center_of_mass(data),
        support_active,
    )
    flags = tuple(
        (str(key), tuple(bool(value) for value in np.asarray(values, dtype=bool).reshape(ACTION_DIM)))
        for key, values in sorted(dict(drive_result["override_flags"]).items())
    )
    new_a_plus = _finite_vector(drive_result["a_plus"], ACTION_DIM, "drive a_plus")
    new_a_minus = _finite_vector(drive_result["a_minus"], ACTION_DIM, "drive a_minus")
    old_drive = old_a_plus - old_a_minus
    new_drive = new_a_plus - new_a_minus
    raw = _finite_vector(raw_action, ACTION_DIM, "raw action")
    accepted = _finite_vector(accepted_action, ACTION_DIM, "accepted action")
    reported_modes = drive_result.get("torque_velocity_modes")
    reported_normalized = drive_result.get("torque_velocity_normalized")
    if reported_modes is None or reported_normalized is None:
        raise DerivativeDomainError(
            "Drive branch receipt lacks pre-step torque-velocity metadata"
        )
    drive_branches = (
        ("raw_action_sign", _discrete_sign(raw, tolerance=_KINK_TOLERANCE)),
        ("accepted_action_sign", _discrete_sign(accepted, tolerance=_KINK_TOLERANCE)),
        (
            "activation_plus",
            tuple("RISING" if max(float(u), 0.0) > float(old) else "DECAYING" for u, old in zip(accepted, old_a_plus, strict=True)),
        ),
        (
            "activation_minus",
            tuple("RISING" if max(float(-u), 0.0) > float(old) else "DECAYING" for u, old in zip(accepted, old_a_minus, strict=True)),
        ),
        ("old_drive_sign", _discrete_sign(old_drive, tolerance=_KINK_TOLERANCE)),
        ("new_drive_sign", _discrete_sign(new_drive, tolerance=_KINK_TOLERANCE)),
        (
            "drive_zero_crossing",
            _flag_labels(dict(drive_result["override_flags"]).get("zero_crossing", np.zeros(ACTION_DIM, dtype=bool))),
        ),
        (
            "torque_velocity_positive",
            tuple(reported_modes[0]),
        ),
        (
            "torque_velocity_negative",
            tuple(reported_modes[1]),
        ),
        (
            "power_projection",
            _flag_labels(dict(drive_result["override_flags"]).get("power_override", np.zeros(ACTION_DIM, dtype=bool))),
        ),
        (
            "capacity_clipping",
            _flag_labels(dict(drive_result["override_flags"]).get("hard_capacity", np.zeros(ACTION_DIM, dtype=bool))),
        ),
        (
            "torque_rate_clipping",
            _flag_labels(dict(drive_result["override_flags"]).get("rate_override", np.zeros(ACTION_DIM, dtype=bool))),
        ),
        (
            "hard_power_clipping",
            _flag_labels(dict(drive_result["override_flags"]).get("hard_power", np.zeros(ACTION_DIM, dtype=bool))),
        ),
    )
    transition_branches = (
        ("raw_action_box", tuple("IN_DOMAIN" for _ in range(ACTION_DIM))),
        ("accepted_action_slew", slew_branches),
    )
    return _StepFingerprint(
        contacts=contacts,
        prohibited_contact=bool(summary["prohibited_contact"]),
        cop_valid=cop_valid,
        support_active=support_active,
        friction_feasible=bool(summary["friction_feasible"]),
        native_joint_limit=_native_joint_limits(data),
        drive_flags=flags,
        drive_branches=drive_branches,
        transition_branches=transition_branches,
        support_margin_branch=support_margin_diagnostic.derivative_branch_certificate,
        drive_guard_values=(
            tuple(float(value) for value in reported_normalized[0]),
            tuple(float(value) for value in reported_normalized[1]),
        ),
    )


def _make_active_set(
    *,
    steps: Sequence[_StepFingerprint],
    action_branches: tuple[str, ...],
) -> ActiveSetFingerprint:
    return ActiveSetFingerprint(
        contact_steps=tuple(step.contacts for step in steps),
        prohibited_contact_steps=tuple(step.prohibited_contact for step in steps),
        cop_valid_steps=tuple(step.cop_valid for step in steps),
        support_active_steps=tuple(step.support_active for step in steps),
        friction_steps=tuple(step.friction_feasible for step in steps),
        native_joint_limit_steps=tuple(step.native_joint_limit for step in steps),
        drive_flag_steps=tuple(step.drive_flags for step in steps),
        action_branches=action_branches,
        drive_branch_steps=tuple(step.drive_branches for step in steps),
        transition_branch_steps=tuple(step.transition_branches for step in steps),
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


def _direct_active_set(
    data: mujoco.MjData,
    summary: Mapping[str, Any],
) -> ActiveSetFingerprint:
    """Build the one-sample physical branch receipt for a direct owner."""

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
    cop_valid = tuple(
        bool(value)
        for value in np.asarray(summary["cop_valid"], dtype=bool).reshape(2)
    )
    support_active = tuple(
        bool(value)
        for value in np.asarray(summary["active_by_foot"], dtype=bool).reshape(2)
    )
    return ActiveSetFingerprint(
        contact_steps=(contacts,),
        prohibited_contact_steps=(bool(summary["prohibited_contact"]),),
        cop_valid_steps=(cop_valid,),
        support_active_steps=(support_active,),
        friction_steps=(bool(summary["friction_feasible"]),),
        native_joint_limit_steps=(_native_joint_limits(data),),
        drive_flag_steps=(),
        action_branches=(),
        drive_branch_steps=(),
        transition_branch_steps=(),
    )


def evaluate_state_owner(
    *,
    plant: Plant,
    snapshot: MacroSnapshot,
    owner_id: str,
) -> StateOwnerEvaluation:
    """Evaluate one approved physical owner at the exact current state.

    The snapshot is restored into fresh mutable MuJoCo/DriveState storage and
    ``mj_forward`` realizes derived state.  There is intentionally no action,
    accepted-action projection, drive update, or physical time advance in this
    evaluator.
    """

    owner_id = str(owner_id)
    if owner_id not in _DIRECT_STATE_OWNER_IDS:
        raise DerivativeDomainError(
            f"direct state owner is not approved: {owner_id}"
        )
    data = plant.make_data()
    drive_state = drive.DriveState()
    snapshot.restore(plant=plant, data=data, drive_state=drive_state)
    mujoco.mj_forward(plant.model, data)

    summary = plant.contact_wrench_summary(data)
    active = tuple(bool(value) for value in summary["active_by_foot"])
    support_branch: tuple[SupportMarginBranchCertificate, ...] = ()
    if owner_id in _SUPPORT_GEOMETRY_OWNER_IDS:
        com = np.asarray(plant.center_of_mass(data), dtype=np.float64)
        diagnostic = plant.support_margin_diagnostics(data, com, active)
        value = np.asarray(
            [plant.support_margin(data, com, active)],
            dtype=np.float64,
        )
        support_branch = (diagnostic.derivative_branch_certificate,)
    else:
        value = np.asarray(
            [plant.center_of_mass_velocity(data)[2]],
            dtype=np.float64,
        )
    if not np.isfinite(value).all():
        raise DerivativeDomainError("direct owner evaluation is nonfinite")
    value = _readonly_array(value)
    owner_outputs = {owner_id: value}
    return StateOwnerEvaluation(
        owner_id=owner_id,
        value=value,
        owner_outputs=owner_outputs,
        active_set=_direct_active_set(data, summary),
        support_margin_branch_steps=support_branch,
        snapshot_digest=snapshot_digest(snapshot),
        owner_output_digest=_array_digest(value),
        time=float(data.time),
        qacc_warmstart_digest=_array_digest(data.qacc_warmstart),
    )


def _workspace(plant: Plant) -> _EvaluationWorkspace:
    return _EvaluationWorkspace(mujoco.MjData(plant.model), drive.DriveState())


def evaluate_wrapped_step_5ms(
    *,
    plant: Plant,
    snapshot: MacroSnapshot,
    raw_action: Sequence[float] | np.ndarray,
    workspace: _EvaluationWorkspace | None = None,
    owner_ids: Sequence[str] = (),
    torque_velocity_schedule: TorqueVelocitySchedule | None = None,
) -> TransitionEvaluation:
    """Restore one snapshot and run the authoritative exact transition once."""

    raw = _finite_vector(raw_action, ACTION_DIM, "raw action")
    owner_ids = tuple(str(owner_id) for owner_id in owner_ids)
    branches = classify_action_projection(snapshot.previous_accepted_action, raw)
    slew_branches = _classify_slew_projection(snapshot.previous_accepted_action, raw)
    active_steps: list[_StepFingerprint] = []
    final_owner_outputs: dict[str, np.ndarray] = {}
    work = _workspace(plant) if workspace is None else workspace
    restored_previous = snapshot.restore(
        plant=plant,
        data=work.data,
        drive_state=work.drive_state,
    )
    previous_a_plus = np.asarray(snapshot.a_plus, dtype=np.float64).copy()
    previous_a_minus = np.asarray(snapshot.a_minus, dtype=np.float64).copy()

    def record_substep(
        _substep: int,
        _accepted: np.ndarray,
        drive_result: Mapping[str, Any],
        _realized: np.ndarray,
        power: Mapping[str, float],
    ) -> bool:
        nonlocal previous_a_plus, previous_a_minus
        active_steps.append(
            _step_fingerprint(
                plant,
                work.data,
                drive_result,
                raw_action=raw,
                accepted_action=_accepted,
                slew_branches=slew_branches,
                old_a_plus=previous_a_plus,
                old_a_minus=previous_a_minus,
            )
        )
        previous_a_plus = _finite_vector(drive_result["a_plus"], ACTION_DIM, "drive a_plus")
        previous_a_minus = _finite_vector(drive_result["a_minus"], ACTION_DIM, "drive a_minus")
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
        torque_velocity_schedule=torque_velocity_schedule,
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
        support_margin_branch_steps=tuple(
            step.support_margin_branch for step in active_steps
            if step.support_margin_branch is not None
        ),
        drive_guard_steps=tuple(step.drive_guard_values for step in active_steps),
    )


def _native_state_coordinate(snapshot: MacroSnapshot, index: int) -> tuple[float, float, float] | None:
    """Return ``(value, lower, upper)`` only for native Euclidean owners."""

    if 51 <= index < 66:
        return float(snapshot.a_plus[index - 51]), 0.0, 1.0
    if 66 <= index < 81:
        return float(snapshot.a_minus[index - 66]), 0.0, 1.0
    if 96 <= index < 111:
        return float(snapshot.previous_accepted_action[index - 96]), -1.0, 1.0
    return None


def _is_native_legal(value: float, lower: float, upper: float) -> bool:
    return bool(
        value >= lower - _NATIVE_DOMAIN_TOLERANCE
        and value <= upper + _NATIVE_DOMAIN_TOLERANCE
    )


def _state_stencil_plan(snapshot: MacroSnapshot, index: int, h: float) -> _StencilPlan:
    domain = _native_state_coordinate(snapshot, index)
    if domain is None:
        return _StencilPlan(CENTRAL_INTERIOR, (-1, 1))
    value, lower, upper = domain
    plus_1 = value + h
    minus_1 = value - h
    if _is_native_legal(plus_1, lower, upper) and _is_native_legal(minus_1, lower, upper):
        return _StencilPlan(CENTRAL_INTERIOR, (-1, 1))
    at_lower = value <= lower + _NATIVE_DOMAIN_TOLERANCE
    at_upper = value >= upper - _NATIVE_DOMAIN_TOLERANCE
    if at_lower and not _is_native_legal(minus_1, lower, upper):
        if _is_native_legal(value + 2.0 * h, lower, upper):
            return _StencilPlan(FORWARD_FEASIBLE_SIDE, (1, 2))
        return _StencilPlan(
            NATIVE_DOMAIN_INVALID,
            (),
            native_domain_legal=False,
            reason=NATIVE_DOMAIN_INVALID,
        )
    if at_upper and not _is_native_legal(plus_1, lower, upper):
        if _is_native_legal(value - 2.0 * h, lower, upper):
            return _StencilPlan(BACKWARD_FEASIBLE_SIDE, (-1, -2))
        return _StencilPlan(
            NATIVE_DOMAIN_INVALID,
            (),
            native_domain_legal=False,
            reason=NATIVE_DOMAIN_INVALID,
        )
    return _StencilPlan(
        NATIVE_DOMAIN_INVALID,
        (),
        native_domain_legal=False,
        reason=NATIVE_DOMAIN_INVALID,
    )


def _action_stencil_plan(
    raw_value: float,
    h: float,
    *,
    previous_value: float,
) -> _StencilPlan:
    del previous_value  # projection branch is certified by sampled evaluations
    # The retained Drive state has distinct directional channels at raw u=0.
    # A declared stencil spanning that surface is invalid before either side is
    # averaged.  This is intentionally conservative at the exact zero kink.
    if raw_value - h < -_KINK_TOLERANCE and raw_value + h > _KINK_TOLERANCE:
        return _StencilPlan(TRUE_KINK_INVALID, (), reason=TRUE_KINK_INVALID)
    plus_1 = raw_value + h
    minus_1 = raw_value - h
    plus_legal = _is_native_legal(plus_1, -_ACTION_BOUND, _ACTION_BOUND)
    minus_legal = _is_native_legal(minus_1, -_ACTION_BOUND, _ACTION_BOUND)
    if plus_legal and minus_legal:
        return _StencilPlan(CENTRAL_INTERIOR, (-1, 1))
    at_lower = raw_value <= -_ACTION_BOUND + _NATIVE_DOMAIN_TOLERANCE
    at_upper = raw_value >= _ACTION_BOUND - _NATIVE_DOMAIN_TOLERANCE
    if at_lower and not minus_legal:
        if _is_native_legal(raw_value + 2.0 * h, -_ACTION_BOUND, _ACTION_BOUND):
            return _StencilPlan(FORWARD_FEASIBLE_SIDE, (1, 2))
        return _StencilPlan(NATIVE_DOMAIN_INVALID, (), native_domain_legal=False, reason=NATIVE_DOMAIN_INVALID)
    if at_upper and not plus_legal:
        if _is_native_legal(raw_value - 2.0 * h, -_ACTION_BOUND, _ACTION_BOUND):
            return _StencilPlan(BACKWARD_FEASIBLE_SIDE, (-1, -2))
        return _StencilPlan(NATIVE_DOMAIN_INVALID, (), native_domain_legal=False, reason=NATIVE_DOMAIN_INVALID)
    return _StencilPlan(NATIVE_DOMAIN_INVALID, (), native_domain_legal=False, reason=NATIVE_DOMAIN_INVALID)


def _sample_values(plan: _StencilPlan, h: float) -> tuple[float, ...]:
    return tuple(float(offset) * h for offset in plan.offsets)


def _difference_from_samples(
    values: Sequence[np.ndarray],
    plan: _StencilPlan,
    h: float,
) -> np.ndarray:
    if plan.classification == CENTRAL_INTERIOR:
        return (np.asarray(values[1]) - np.asarray(values[0])) / (2.0 * h)
    if plan.classification == FORWARD_FEASIBLE_SIDE:
        return (4.0 * np.asarray(values[0]) - np.asarray(values[1])) / (2.0 * h)
    if plan.classification == BACKWARD_FEASIBLE_SIDE:
        return (-4.0 * np.asarray(values[0]) + np.asarray(values[1])) / (2.0 * h)
    raise DerivativeDomainError(f"cannot evaluate invalid stencil {plan.classification}")


def _physical_branch_changed(base: ActiveSetFingerprint, sample: ActiveSetFingerprint) -> bool:
    return any(
        left != right
        for left, right in (
            (base.contact_steps, sample.contact_steps),
            (base.prohibited_contact_steps, sample.prohibited_contact_steps),
            (base.cop_valid_steps, sample.cop_valid_steps),
            (base.support_active_steps, sample.support_active_steps),
            (base.friction_steps, sample.friction_steps),
        )
    )


def _same_support_margin_active_set(
    base: TransitionEvaluation | StateOwnerEvaluation,
    sample: TransitionEvaluation | StateOwnerEvaluation,
) -> bool:
    """Compare the owner regime while treating raw contacts as provenance.

    The support-margin owner is defined by the latched physical support
    geometry and its scalar branch certificate. Every other active-set field
    remains part of the certificate; only raw MuJoCo contact rows are omitted
    from this owner-specific comparison.
    """

    return replace(
        base.active_set,
        contact_steps=sample.active_set.contact_steps,
    ) == sample.active_set


def _support_margin_branch_rejection_reason(
    base: TransitionEvaluation | StateOwnerEvaluation,
    sample: TransitionEvaluation | StateOwnerEvaluation,
) -> str:
    """Compare Plant provenance of the mathematical support-margin branch.

    The canonical hull cycle is retained for diagnostics, but its vertex
    representation is not itself a scalar branch.  Redundant collinear
    source points can therefore enter or leave that cycle without changing
    the active physical support primitive.
    """
    base_steps = base.support_margin_branch_steps
    sample_steps = sample.support_margin_branch_steps
    if not base_steps and not sample_steps:
        return ""
    if len(base_steps) != len(sample_steps):
        return SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID
    for base_branch, sample_branch in zip(base_steps, sample_steps, strict=True):
        if base_branch is None or sample_branch is None:
            return NONFINITE_EVALUATION
        if not base_branch.finite or not sample_branch.finite:
            return NONFINITE_EVALUATION
        if base_branch.support_active_set != sample_branch.support_active_set:
            return SUPPORT_ACTIVE_SET_SWITCH_INVALID
        if base_branch.support_point_count != sample_branch.support_point_count:
            return SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID
        if base_branch.branch_family != sample_branch.branch_family:
            return SUPPORT_HULL_KINK_INVALID
        if (
            base_branch.exact_tie
            or sample_branch.exact_tie
            or base_branch.norm_zero_kink
            or sample_branch.norm_zero_kink
        ):
            return SUPPORT_HULL_KINK_INVALID
        if base_branch.projection_regimes != sample_branch.projection_regimes:
            return SUPPORT_HULL_PROJECTION_BRANCH_SWITCH_INVALID
        if (
            base_branch.active_minimizer_set != sample_branch.active_minimizer_set
            or base_branch.unique_active_minimizer
            != sample_branch.unique_active_minimizer
        ):
            return SUPPORT_HULL_KINK_INVALID
    return ""


def _branch_rejection_reason(
    base: TransitionEvaluation | StateOwnerEvaluation,
    samples: Sequence[TransitionEvaluation | StateOwnerEvaluation],
    *,
    support_geometry: bool = False,
    support_margin_owner: bool = False,
) -> str:
    for sample in samples:
        if support_geometry:
            support_reason = _support_margin_branch_rejection_reason(base, sample)
            if support_reason:
                return support_reason
        if _physical_branch_changed(base.active_set, sample.active_set) and not (
            support_geometry
            and support_margin_owner
            and _same_support_margin_active_set(base, sample)
        ):
            return PHYSICAL_CONTACT_SWITCH_INVALID
    return PIECEWISE_BRANCH_SWITCH_INVALID


def transition_mode_schedule(
    active_set: ActiveSetFingerprint,
) -> TorqueVelocitySchedule:
    """Return the immutable exact Drive mode schedule for one transition."""
    if not active_set.drive_branch_steps:
        raise DerivativeDomainError("transition has no Drive mode schedule")
    schedule = []
    for substep, branches in enumerate(active_set.drive_branch_steps):
        values = dict(branches)
        try:
            positive = tuple(values["torque_velocity_positive"])
            negative = tuple(values["torque_velocity_negative"])
        except KeyError as exc:
            raise DerivativeDomainError(
                f"substep {substep} lacks torque-velocity mode ownership"
            ) from exc
        if len(positive) != ACTION_DIM or len(negative) != ACTION_DIM:
            raise DerivativeDomainError(
                f"substep {substep} torque-velocity schedule has wrong dimension"
            )
        schedule.append((positive, negative))
    if len(schedule) != SUBSTEPS_PER_CONTROL:
        raise DerivativeDomainError(
            f"transition mode schedule has {len(schedule)} substeps; expected {SUBSTEPS_PER_CONTROL}"
        )
    return tuple(schedule)


def _base_drive_mode_guard_is_clear(
    evaluation: TransitionEvaluation,
) -> bool:
    """Reject a scheduled derivative at the exact Drive branch boundary."""
    if len(evaluation.drive_guard_steps) != SUBSTEPS_PER_CONTROL:
        return False
    for positive, negative in evaluation.drive_guard_steps:
        for value in (*positive, *negative):
            if abs(abs(float(value)) - _TORQUE_VELOCITY_NORMALIZED_NEAR_ZERO) <= _KINK_TOLERANCE:
                return False
    return True


def _scheduled_drive_mode_only(
    base: TransitionEvaluation,
    samples: Sequence[TransitionEvaluation],
) -> bool:
    """Return whether samples differ only in the schedulable Drive mode."""
    if not _base_drive_mode_guard_is_clear(base):
        return False
    allowed = {"torque_velocity_positive", "torque_velocity_negative"}
    for sample in samples:
        if _physical_branch_changed(base.active_set, sample.active_set):
            return False
        if base.active_set.native_joint_limit_steps != sample.active_set.native_joint_limit_steps:
            return False
        if base.active_set.drive_flag_steps != sample.active_set.drive_flag_steps:
            return False
        if base.active_set.action_branches != sample.active_set.action_branches:
            return False
        if base.active_set.transition_branch_steps != sample.active_set.transition_branch_steps:
            return False
        if _support_margin_branch_rejection_reason(base, sample):
            return False
        for base_step, sample_step in zip(
            base.active_set.drive_branch_steps,
            sample.active_set.drive_branch_steps,
            strict=True,
        ):
            base_values = dict(base_step)
            sample_values = dict(sample_step)
            if base_values.keys() != sample_values.keys():
                return False
            for key in base_values:
                if key not in allowed and base_values[key] != sample_values[key]:
                    return False
    return True


def _scheduled_column_samples(
    *,
    plant: Plant,
    base_snapshot: MacroSnapshot,
    raw_action: np.ndarray,
    index: int,
    step: float,
    plan: _StencilPlan,
    schedule: TorqueVelocitySchedule,
) -> tuple[TransitionEvaluation, ...]:
    result = []
    for offset in plan.offsets:
        delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
        delta[index] = float(offset) * step
        candidate = boxplus(base_snapshot, delta, model=plant.model)
        result.append(
            evaluate_wrapped_step_5ms(
                plant=plant,
                snapshot=candidate,
                raw_action=raw_action,
                torque_velocity_schedule=schedule,
            )
        )
    return tuple(result)


def _same_certificate_for_column(
    base: TransitionEvaluation | StateOwnerEvaluation,
    sample: TransitionEvaluation | StateOwnerEvaluation,
    *,
    axis: str,
    index: int,
    plan: _StencilPlan,
    support_geometry: bool = False,
    support_margin_owner: bool = False,
) -> bool:
    if support_geometry and _support_margin_branch_rejection_reason(base, sample):
        return False
    if sample.active_set == base.active_set:
        return True
    if (
        support_geometry
        and support_margin_owner
        and _same_support_margin_active_set(base, sample)
    ):
        return True
    # An action-box/previous-action endpoint is a native domain boundary, not
    # a physical projection kink.  The feasible-side sample necessarily has a
    # different endpoint label (BOUND_ACTIVE -> INTERIOR), which is allowed as
    # long as it does not enter the exact slew kink.
    boundary_coordinate = axis in {"B", "OWNER_ACTION"} or 96 <= index < 111
    if not boundary_coordinate or plan.classification not in {
        FORWARD_FEASIBLE_SIDE,
        BACKWARD_FEASIBLE_SIDE,
    }:
        return False
    if (
        index >= len(base.active_set.action_branches)
        or index >= len(sample.active_set.action_branches)
    ):
        return False
    if base.active_set.action_branches[index] != ACTION_BRANCH_BOUND_ACTIVE:
        return False
    if sample.active_set.action_branches[index] == ACTION_BRANCH_NEAR_KINK:
        return False
    base_without_endpoint = replace(
        base.active_set,
        action_branches=sample.active_set.action_branches,
    )
    return base_without_endpoint == sample.active_set


def _column_result(
    *,
    axis: str,
    index: int,
    step: float,
    base: TransitionEvaluation | StateOwnerEvaluation,
    plus: TransitionEvaluation | StateOwnerEvaluation | None,
    minus: TransitionEvaluation | StateOwnerEvaluation | None,
    samples: Sequence[TransitionEvaluation | StateOwnerEvaluation],
    block: str,
    plan: _StencilPlan,
    reason: str = "",
    support_geometry: bool = False,
    support_margin_owner: bool = False,
) -> ColumnQualification:
    plus_digest = None if plus is None else plus.active_set.digest
    minus_digest = None if minus is None else minus.active_set.digest
    sample_digests = tuple(sample.active_set.digest for sample in samples)
    preserved = bool(samples) and all(
        _same_certificate_for_column(
            base,
            sample,
            axis=axis,
            index=index,
            plan=plan,
            support_geometry=support_geometry,
            support_margin_owner=support_margin_owner,
        )
        for sample in samples
    )
    if reason:
        valid = False
    elif not samples:
        valid = False
        reason = plan.reason or NATIVE_DOMAIN_INVALID
    elif not preserved:
        valid = False
        reason = _branch_rejection_reason(
            base,
            samples,
            support_geometry=support_geometry,
            support_margin_owner=support_margin_owner,
        )
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
        stencil=plan.classification,
        native_domain_legal=plan.native_domain_legal,
        base_fingerprint_digest=base.active_set.digest,
        sample_fingerprint_digests=sample_digests,
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


@dataclass(frozen=True)
class _StateOwnerDerivativeCore:
    """Common ML-241 state-column difference result for both owner modes."""

    base: TransitionEvaluation | StateOwnerEvaluation
    base_value: np.ndarray
    jacobian: np.ndarray
    reports: tuple[ColumnQualification, ...]
    evaluations: int


def _differentiate_owner_state_columns(
    *,
    plant: Plant,
    base_snapshot: MacroSnapshot,
    owner_id: str,
    state_steps: Mapping[str, float],
    state_indexes: Sequence[int],
    evaluator: Callable[[MacroSnapshot], TransitionEvaluation | StateOwnerEvaluation],
    support_geometry: bool,
    support_margin_owner: bool = False,
    base_evaluation: TransitionEvaluation | StateOwnerEvaluation | None = None,
    allow_nonsmooth: bool = False,
) -> _StateOwnerDerivativeCore:
    """Run the certified state FD policy against an owner evaluator strategy."""

    step_vector = _state_step_vector(state_steps)
    indexes = tuple(int(index) for index in state_indexes)
    base = evaluator(base_snapshot) if base_evaluation is None else base_evaluation
    base_value = np.asarray(base.owner_outputs[owner_id], dtype=np.float64).reshape(-1)
    jacobian = np.full(
        (base_value.size, TANGENT_DIMENSION),
        np.nan,
        dtype=np.float64,
    )
    reports: list[ColumnQualification] = []
    evaluations = 1

    def exception_reason(exc: Exception) -> str:
        message = str(exc).lower()
        if "nan" in message or "inf" in message or "finite" in message:
            return NONFINITE_EVALUATION
        if isinstance(exc, (ValueError, RuntimeError)):
            return MANIFOLD_INVALID
        return f"PERTURBATION_REJECTED:{type(exc).__name__}"

    for index in indexes:
        h = float(step_vector[index])
        plan = _state_stencil_plan(base_snapshot, index, h)
        samples: list[TransitionEvaluation | StateOwnerEvaluation] = []
        offset_to_sample: dict[
            int, TransitionEvaluation | StateOwnerEvaluation
        ] = {}
        reason = plan.reason
        if plan.valid:
            for offset in plan.offsets:
                delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
                delta[index] = float(offset) * h
                try:
                    candidate = boxplus(base_snapshot, delta, model=plant.model)
                    evaluation = evaluator(candidate)
                    samples.append(evaluation)
                    offset_to_sample[offset] = evaluation
                    evaluations += 1
                except (
                    DerivativeDomainError,
                    ValueError,
                    RuntimeError,
                    drive.DriveModelError,
                ) as exc:
                    reason = exception_reason(exc)
                    break
        report = _column_result(
            axis="OWNER_STATE",
            index=index,
            step=h,
            base=base,
            plus=offset_to_sample.get(1),
            minus=offset_to_sample.get(-1),
            samples=samples,
            block=_STATE_BLOCK_FOR_INDEX[index],
            plan=plan,
            reason=reason,
            support_geometry=support_geometry,
            support_margin_owner=support_margin_owner,
        )
        if report.valid:
            try:
                values = [
                    np.asarray(sample.owner_outputs[owner_id]) - base_value
                    for sample in samples
                ]
                value = _difference_from_samples(values, plan, h)
                if not np.isfinite(value).all():
                    raise FloatingPointError("owner state derivative is non-finite")
                jacobian[:, index] = value
            except (
                DerivativeDomainError,
                ValueError,
                RuntimeError,
                FloatingPointError,
            ) as exc:
                report = replace(
                    report,
                    valid=False,
                    reason=(
                        NONFINITE_EVALUATION
                        if isinstance(exc, FloatingPointError)
                        else MANIFOLD_INVALID
                    ),
                )
        reports.append(report)

    _raise_invalid_columns(reports, allow_nonsmooth=allow_nonsmooth)
    selected_finite = all(
        np.isfinite(jacobian[:, index]).all() for index in indexes
    )
    if not allow_nonsmooth and not selected_finite:
        raise DerivativeDomainError(
            "qualified owner-output state sensitivity contains NaN or Inf"
        )
    return _StateOwnerDerivativeCore(
        base=base,
        base_value=base_value,
        jacobian=jacobian,
        reports=tuple(reports),
        evaluations=evaluations,
    )


def _direct_qacc_receipt(
    *,
    reports: Sequence[ColumnQualification],
    jacobian: np.ndarray,
) -> tuple[str, tuple[int, ...], tuple[tuple[str, float], ...]]:
    """Seal only a completed direct 21-column numerical null qualification."""

    qacc_columns = QACC_ZERO_COLUMNS
    by_index = {int(report.index): report for report in reports}
    if any(index not in by_index for index in qacc_columns):
        return QACC_DERIVATIVE_UNADJUDICATED, (), ()
    if any(not by_index[index].valid for index in qacc_columns):
        return QACC_DERIVATIVE_UNADJUDICATED, (), ()
    translation_bound = float(
        np.max(np.abs(jacobian[:, 111:114]))
    )
    rotation_joint_bound = float(
        np.max(np.abs(jacobian[:, 114:132]))
    )
    if any(
        not np.isfinite(jacobian[:, index]).all()
        or np.any(np.abs(jacobian[:, index]) > 1.0e-12)
        for index in qacc_columns
    ):
        return QACC_DERIVATIVE_UNADJUDICATED, (), ()
    return (
        QACC_DERIVATIVE_NUMERICALLY_NULL,
        qacc_columns,
        (
            ("qacc_translation", translation_bound),
            ("qacc_rotation_joint", rotation_joint_bound),
        ),
    )


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
    """Finite-difference the exact wrapped transition in a qualified domain.

    Only requested columns are sampled.  Every valid sample is expressed in
    the base next-state tangent frame with canonical ``boxminus``; no raw
    quaternion subtraction or illegal-domain clipping is permitted.
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
    base = evaluate_wrapped_step_5ms(plant=plant, snapshot=base_snapshot, raw_action=raw, workspace=work)
    base_next = base.next_snapshot
    A = np.full((TANGENT_DIMENSION, TANGENT_DIMENSION), np.nan, dtype=np.float64)
    B = np.full((TANGENT_DIMENSION, ACTION_DIM), np.nan, dtype=np.float64)
    P = np.full((ACTION_DIM, ACTION_DIM), np.nan, dtype=np.float64)
    state_reports: list[ColumnQualification] = []
    action_reports: list[ColumnQualification] = []
    evaluations = 1

    def exception_reason(exc: Exception) -> str:
        message = str(exc).lower()
        if "nan" in message or "inf" in message or "finite" in message:
            return NONFINITE_EVALUATION
        if isinstance(exc, (ValueError, RuntimeError)):
            return MANIFOLD_INVALID
        return f"PERTURBATION_REJECTED:{type(exc).__name__}"

    for index in state_indexes:
        h = float(step_vector[index])
        plan = _state_stencil_plan(base_snapshot, index, h)
        samples: list[TransitionEvaluation] = []
        reason = plan.reason
        offset_to_sample: dict[int, TransitionEvaluation] = {}
        if plan.valid:
            for offset in plan.offsets:
                delta = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
                delta[index] = float(offset) * h
                try:
                    candidate = boxplus(base_snapshot, delta, model=plant.model)
                    evaluation = evaluate_wrapped_step_5ms(
                        plant=plant, snapshot=candidate, raw_action=raw, workspace=work
                    )
                    samples.append(evaluation)
                    offset_to_sample[offset] = evaluation
                    evaluations += 1
                except (DerivativeDomainError, ValueError, RuntimeError, drive.DriveModelError) as exc:
                    reason = exception_reason(exc)
                    break
        report = _column_result(
            axis="A",
            index=index,
            step=h,
            base=base,
            plus=offset_to_sample.get(1),
            minus=offset_to_sample.get(-1),
            samples=samples,
            block=_STATE_BLOCK_FOR_INDEX[index],
            plan=plan,
            reason=reason,
        )
        derivative_samples = samples
        if (
            not allow_nonsmooth
            and not report.valid
            and report.reason == PIECEWISE_BRANCH_SWITCH_INVALID
            and _scheduled_drive_mode_only(base, samples)
        ):
            try:
                schedule = transition_mode_schedule(base.active_set)
                scheduled_samples = list(
                    _scheduled_column_samples(
                        plant=plant,
                        base_snapshot=base_snapshot,
                        raw_action=raw,
                        index=index,
                        step=h,
                        plan=plan,
                        schedule=schedule,
                    )
                )
                if not _scheduled_drive_mode_only(base, scheduled_samples):
                    raise DerivativeDomainError(
                        "scheduled transition changed a non-Drive active-set branch"
                    )
                derivative_samples = scheduled_samples
                evaluations += len(derivative_samples)
                report = replace(report, valid=True, reason=SCHEDULED_MODE_SMOOTH)
            except (DerivativeDomainError, ValueError, RuntimeError, drive.DriveModelError):
                derivative_samples = samples
        if report.valid:
            try:
                deltas = [
                    boxminus(sample.next_snapshot, base_next, model=plant.model)
                    for sample in derivative_samples
                ]
                value = _difference_from_samples(deltas, plan, h)
                if not np.isfinite(value).all():
                    raise FloatingPointError("state derivative is non-finite")
                A[:, index] = value
            except (DerivativeDomainError, ValueError, RuntimeError, FloatingPointError) as exc:
                report = replace(report, valid=False, reason=NONFINITE_EVALUATION if isinstance(exc, FloatingPointError) else MANIFOLD_INVALID)
        state_reports.append(report)

    for index in action_indexes:
        h = float(action_step_vector[index])
        plan = _action_stencil_plan(
            float(raw[index]),
            h,
            previous_value=float(base_snapshot.previous_accepted_action[index]),
        )
        samples = []
        reason = plan.reason
        offset_to_sample = {}
        if not reason and base.active_set.action_branches[index] == ACTION_BRANCH_NEAR_KINK:
            reason = PIECEWISE_BRANCH_SWITCH_INVALID
        if plan.valid and not reason:
            for offset in plan.offsets:
                candidate_action = raw.copy()
                candidate_action[index] += float(offset) * h
                try:
                    evaluation = evaluate_wrapped_step_5ms(
                        plant=plant,
                        snapshot=base_snapshot,
                        raw_action=candidate_action,
                        workspace=work,
                    )
                    samples.append(evaluation)
                    offset_to_sample[offset] = evaluation
                    evaluations += 1
                except (DerivativeDomainError, ValueError, RuntimeError, drive.DriveModelError) as exc:
                    reason = exception_reason(exc)
                    break
        report = _column_result(
            axis="B",
            index=index,
            step=h,
            base=base,
            plus=offset_to_sample.get(1),
            minus=offset_to_sample.get(-1),
            samples=samples,
            block="raw_action",
            plan=plan,
            reason=reason,
        )
        derivative_samples = samples
        if report.valid:
            try:
                deltas = [
                    boxminus(sample.next_snapshot, base_next, model=plant.model)
                    for sample in derivative_samples
                ]
                value = _difference_from_samples(deltas, plan, h)
                if not np.isfinite(value).all():
                    raise FloatingPointError("action derivative is non-finite")
                B[:, index] = value
                accepted_values = [
                    project_accepted_action(
                        base_snapshot.previous_accepted_action,
                        np.asarray(raw + np.eye(ACTION_DIM, dtype=np.float64)[index] * (float(offset) * h)),
                    )
                    for offset in plan.offsets
                ]
                accepted_base = project_accepted_action(
                    base_snapshot.previous_accepted_action,
                    raw,
                )
                P[:, index] = _difference_from_samples(
                    [value - accepted_base for value in accepted_values],
                    plan,
                    h,
                )
            except (DerivativeDomainError, ValueError, RuntimeError, FloatingPointError) as exc:
                report = replace(report, valid=False, reason=NONFINITE_EVALUATION if isinstance(exc, FloatingPointError) else MANIFOLD_INVALID)
        action_reports.append(report)

    _raise_invalid_columns((*state_reports, *action_reports), allow_nonsmooth=allow_nonsmooth)
    selected_state_finite = all(np.isfinite(A[:, index]).all() for index in state_indexes)
    selected_action_finite = all(np.isfinite(B[:, index]).all() for index in action_indexes)
    if not allow_nonsmooth and not (selected_state_finite and selected_action_finite):
        raise DerivativeDomainError("qualified wrapped derivative contains NaN or Inf")

    state_validity_values = np.zeros(TANGENT_DIMENSION, dtype=bool)
    for report in state_reports:
        state_validity_values[report.index] = report.valid
    action_validity_values = np.zeros(ACTION_DIM, dtype=bool)
    for report in action_reports:
        action_validity_values[report.index] = report.valid
    linearization = WrappedLinearization(
        A=_readonly_array(A),
        B=_readonly_array(B),
        accepted_action_jacobian=_readonly_array(P),
        state_validity=_readonly_array(state_validity_values, dtype=bool),
        action_validity=_readonly_array(action_validity_values, dtype=bool),
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
    if set(QACC_ZERO_COLUMNS).issubset(state_indexes):
        return linearization.certify_qacc_numerical_null(
            evidence_id=QACC_CERTIFICATE_EVIDENCE_ID,
            absolute_error_bound=QACC_ERROR_BOUNDS,
        )
    return linearization


def differentiate_state_owner_output(
    *,
    plant: Plant,
    base_snapshot: MacroSnapshot,
    owner_id: str,
    state_steps: Mapping[str, float],
    state_columns: Sequence[int] | None = None,
    allow_nonsmooth: bool = False,
) -> OwnerOutputSensitivity:
    """Differentiate an approved owner as a function of the current state.

    This is the state-only sibling of :func:`differentiate_owner_output`.
    Both entry points use the same ML-241 stencil, manifold perturbation,
    branch-certificate, and finite-difference core; only their evaluator
    strategy differs.
    """

    owner_id = str(owner_id)
    if owner_id not in _DIRECT_STATE_OWNER_IDS:
        raise DerivativeDomainError(
            f"direct state owner is not approved: {owner_id}"
        )
    state_indexes = (
        tuple(range(TANGENT_DIMENSION))
        if state_columns is None
        else tuple(int(index) for index in state_columns)
    )
    if any(index < 0 or index >= TANGENT_DIMENSION for index in state_indexes):
        raise DerivativeDomainError(
            "direct state owner column index outside [0, 132)"
        )
    if len(set(state_indexes)) != len(state_indexes):
        raise DerivativeDomainError(
            "duplicate direct state owner derivative column index"
        )

    core = _differentiate_owner_state_columns(
        plant=plant,
        base_snapshot=base_snapshot,
        owner_id=owner_id,
        state_steps=state_steps,
        state_indexes=state_indexes,
        evaluator=lambda snapshot: evaluate_state_owner(
            plant=plant,
            snapshot=snapshot,
            owner_id=owner_id,
        ),
        support_geometry=owner_id in _SUPPORT_GEOMETRY_OWNER_IDS,
        support_margin_owner=owner_id == "SUPPORT_MARGIN",
        allow_nonsmooth=allow_nonsmooth,
    )
    base = core.base
    if not isinstance(base, StateOwnerEvaluation):  # pragma: no cover
        raise DerivativeDomainError("direct owner evaluator returned a transition result")
    state_validity_values = np.zeros(TANGENT_DIMENSION, dtype=bool)
    for report in core.reports:
        state_validity_values[report.index] = report.valid
    action_jacobian = np.full((core.base_value.size, ACTION_DIM), np.nan, dtype=np.float64)
    qacc_disposition, qacc_zero_columns, qacc_bounds = _direct_qacc_receipt(
        reports=core.reports,
        jacobian=core.jacobian,
    )
    base_branch_certificate = (
        base.support_margin_branch_steps[0]
        if len(base.support_margin_branch_steps) == 1
        else None
    )
    return OwnerOutputSensitivity(
        owner_id=owner_id,
        base_value=_readonly_array(core.base_value),
        state_jacobian=_readonly_array(core.jacobian),
        action_jacobian=_readonly_array(action_jacobian),
        state_validity=_readonly_array(state_validity_values, dtype=bool),
        action_validity=_readonly_array(np.zeros(ACTION_DIM, dtype=bool), dtype=bool),
        state_columns=core.reports,
        action_columns=(),
        base_snapshot_digest=base.snapshot_digest,
        base_next_snapshot_digest="",
        state_step_metadata={key: float(state_steps[key]) for key in STATE_STEP_BLOCKS},
        action_step_metadata=(),
        scheme=CENTRAL_DIFFERENCE_SCHEME,
        transition_evaluation_count=core.evaluations,
        evaluation_mode=DIRECT_STATE_OWNER,
        owner_output_digest=base.owner_output_digest,
        base_branch_certificate=base_branch_certificate,
        qacc_derivative_disposition=qacc_disposition,
        qacc_zero_columns=qacc_zero_columns,
        qacc_absolute_error_bound=qacc_bounds,
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

    action_shape = (base_value.size, ACTION_DIM)
    action_jacobian = np.full(action_shape, np.nan, dtype=np.float64)
    state_reports: list[ColumnQualification] = []
    action_reports: list[ColumnQualification] = []
    evaluations = 1

    def exception_reason(exc: Exception) -> str:
        message = str(exc).lower()
        if "nan" in message or "inf" in message or "finite" in message:
            return NONFINITE_EVALUATION
        if isinstance(exc, (ValueError, RuntimeError)):
            return MANIFOLD_INVALID
        return f"PERTURBATION_REJECTED:{type(exc).__name__}"

    state_core = _differentiate_owner_state_columns(
        plant=plant,
        base_snapshot=base_snapshot,
        owner_id=owner_id,
        state_steps=state_steps,
        state_indexes=state_indexes,
        evaluator=lambda snapshot: evaluate_wrapped_step_5ms(
            plant=plant,
            snapshot=snapshot,
            raw_action=raw,
            workspace=work,
            owner_ids=(owner_id,),
        ),
        support_geometry=owner_id in _SUPPORT_GEOMETRY_OWNER_IDS,
        base_evaluation=base,
        allow_nonsmooth=allow_nonsmooth,
    )
    base = state_core.base
    base_value = state_core.base_value
    state_jacobian = state_core.jacobian
    state_reports = list(state_core.reports)
    evaluations = state_core.evaluations

    for index in action_indexes:
        h = float(action_step_vector[index])
        plan = _action_stencil_plan(
            float(raw[index]),
            h,
            previous_value=float(base_snapshot.previous_accepted_action[index]),
        )
        samples = []
        offset_to_sample = {}
        reason = plan.reason
        if not reason and base.active_set.action_branches[index] == ACTION_BRANCH_NEAR_KINK:
            reason = PIECEWISE_BRANCH_SWITCH_INVALID
        if plan.valid and not reason:
            for offset in plan.offsets:
                candidate_action = raw.copy()
                candidate_action[index] += float(offset) * h
                try:
                    evaluation = evaluate_wrapped_step_5ms(
                        plant=plant,
                        snapshot=base_snapshot,
                        raw_action=candidate_action,
                        workspace=work,
                        owner_ids=(owner_id,),
                    )
                    samples.append(evaluation)
                    offset_to_sample[offset] = evaluation
                    evaluations += 1
                except (DerivativeDomainError, ValueError, RuntimeError, drive.DriveModelError) as exc:
                    reason = exception_reason(exc)
                    break
        report = _column_result(
            axis="OWNER_ACTION",
            index=index,
            step=h,
            base=base,
            plus=offset_to_sample.get(1),
            minus=offset_to_sample.get(-1),
            samples=samples,
            block="raw_action",
            plan=plan,
            reason=reason,
            support_geometry=owner_id in _SUPPORT_GEOMETRY_OWNER_IDS,
        )
        if report.valid:
            try:
                values = [np.asarray(sample.owner_outputs[owner_id]) - base_value for sample in samples]
                value = _difference_from_samples(values, plan, h)
                if not np.isfinite(value).all():
                    raise FloatingPointError("owner action derivative is non-finite")
                action_jacobian[:, index] = value
            except (DerivativeDomainError, ValueError, RuntimeError, FloatingPointError) as exc:
                report = replace(report, valid=False, reason=NONFINITE_EVALUATION if isinstance(exc, FloatingPointError) else MANIFOLD_INVALID)
        action_reports.append(report)

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
        evaluation_mode=TRANSITION_WRAPPED_OWNER,
        owner_output_digest=_array_digest(base_value),
    )


__all__ = [
    "ACTION_BRANCH_BOUND_ACTIVE",
    "ACTION_BRANCH_INTERIOR",
    "ACTION_BRANCH_NEAR_KINK",
    "ACTION_BRANCH_SLEW_ACTIVE",
    "BACKWARD_FEASIBLE_SIDE",
    "CENTRAL_INTERIOR",
    "ActiveSetFingerprint",
    "CENTRAL_DIFFERENCE_SCHEME",
    "CONSTRAINT_CATALOG_ID",
    "DERIVATIVE_API_VERSION",
    "DIRECT_STATE_OWNER",
    "FIXED_ELIMINATED",
    "FORWARD_FEASIBLE_SIDE",
    "MANIFOLD_INVALID",
    "NATIVE_DOMAIN_INVALID",
    "NONFINITE_EVALUATION",
    "NUMERICAL_CERTIFICATE_ONLY",
    "PHYSICAL_CONTACT_SWITCH_INVALID",
    "SCHEDULED_MODE_SMOOTH",
    "PIECEWISE_BRANCH_SWITCH_INVALID",
    "SUPPORT_ACTIVE_SET_SWITCH_INVALID",
    "SUPPORT_HULL_KINK_INVALID",
    "SUPPORT_HULL_PROJECTION_BRANCH_SWITCH_INVALID",
    "SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID",
    "QACC_DERIVATIVE_MATERIAL_BUT_UNIDENTIFIABLE",
    "QACC_DERIVATIVE_NUMERICALLY_NULL",
    "QACC_DERIVATIVE_PRIOR_GATE_DEFECT",
    "QACC_DERIVATIVE_QUALIFIED_NONZERO",
    "QACC_DERIVATIVE_UNADJUDICATED",
    "QACC_CERTIFICATE_EVIDENCE_ID",
    "QACC_ERROR_BOUNDS",
    "QACC_ZERO_COLUMNS",
    "DerivativeDomainError",
    "ColumnQualification",
    "OWNER_OUTPUT_IDS",
    "OwnerOutputSensitivity",
    "StateOwnerEvaluation",
    "SNAPSHOT_SCHEMA_ID",
    "STATE_STEP_BLOCKS",
    "TANGENT_LAYOUT_ID",
    "TRANSITION_WRAPPED_OWNER",
    "TRUE_KINK_INVALID",
    "TransitionEvaluation",
    "WrappedLinearization",
    "classify_action_projection",
    "differentiate_state_owner_output",
    "differentiate_owner_output",
    "evaluate_state_owner",
    "evaluate_wrapped_step_5ms",
    "linearize_step_5ms",
    "snapshot_digest",
    "transition_mode_schedule",
]
