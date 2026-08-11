"""Solver-independent exact-discrete direct-multiple-shooting structure.

The module stores local tangent offsets around immutable :class:`MacroSnapshot`
references.  It evaluates defects by restoring a reconstructed snapshot and
calling the shared five-millisecond transition.  It contains no solver,
objective, physical equation, or alternate state representation.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from hashlib import sha256
from types import MappingProxyType
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.oracle.constraints import CATALOG, ConstraintCatalog, ConstraintSpec
from loaded_cmj.oracle.derivatives import (
    CONSTRAINT_CATALOG_ID,
    QACC_DERIVATIVE_NUMERICALLY_NULL,
    STATE_STEP_BLOCKS,
    TANGENT_LAYOUT_ID as DERIVATIVE_TANGENT_LAYOUT_ID,
    WrappedLinearization,
    linearize_step_5ms,
    snapshot_digest,
)
from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import ACTION_DIM, CONTROL_PERIOD_S
from loaded_cmj.simulation.plant import Plant
from loaded_cmj.simulation.snapshot import MacroSnapshot, SNAPSHOT_SCHEMA_VERSION
from loaded_cmj.simulation.tangent import (
    TANGENT_DIMENSION,
    boxminus,
    boxplus,
)
from loaded_cmj.simulation.transition import step_5ms


TRANSCRIPTION_SCHEMA_ID = "LCMJ-V1-SP-SDDT-TRANSCRIPTION-1.0.0"
STATE_DIMENSION = TANGENT_DIMENSION
CONTROL_DT_S = CONTROL_PERIOD_S
ACTION_DIMENSION = ACTION_DIM
QACC_ZERO_COLUMNS = tuple(range(111, 132))
QACC_DERIVATIVE_DISPOSITION = QACC_DERIVATIVE_NUMERICALLY_NULL
QACC_ERROR_BOUNDS = MappingProxyType(
    {
        "qacc_translation": 1.0e-7,
        "qacc_rotation_joint": 1.0e-7,
    }
)

# These are the exact blockwise steps in the ML-241/ML-242 receipt.  They are
# used only for endpoint maps; they are not a new transition FD contract.
ML241_STATE_STEPS = MappingProxyType(
    {
        "configuration_translation": 1.0e-4,
        "configuration_rotation_joint": 1.0e-6,
        "cache_so3": 1.0e-4,
        "qvel_translation": 1.0e-3,
        "qvel_rotation_joint": 1.0e-3,
        "drivestate_activation": 1.0e-4,
        "drivestate_tau_prev": 1.0e-3,
        "previous_accepted_action": 1.0e-4,
        "qacc_translation": 1.0e-1,
        "qacc_rotation_joint": 1.0e-1,
    }
)
ML241_ACTION_STEP = 1.0e-4
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

# Effective Phase-I elastic allowlist after the pre-ML-244 E3/E4 erratum.  This
# is metadata, not a copied constraint formula and not an instruction to create
# an objective.  Rows absent from this list remain hard constraints without a
# Phase-I slack; their terminal residuals are still assembled normally.
APPROVED_ELASTIC_CONSTRAINT_IDS = (
    "ACTION_RAW_BOX",
    "ACTION_ACCEPTED_SLEW",
    "DRIVE_ACTIVATION_RANGE",
    "DRIVE_REALIZED_TORQUE_CAPACITY",
    "DRIVE_TORQUE_RATE",
    "DRIVE_SIGNED_POWER",
    "CONTACT_NORMAL_FORCE_VALIDITY",
    "CONTACT_COP_SUPPORT_GEOMETRY",
    "SUPPORT_POLYGON",
    "SUPPORT_MARGIN",
    "ENERGY_WORK_RESIDUAL",
    "E3_E4_HORIZONTAL",
)
_SMOOTH_DIFFERENTIABILITY = frozenset({"SMOOTH_MODE_LOCAL", "PIECEWISE_SMOOTH"})
_TRANSITION_OWNER = "src/loaded_cmj/simulation/transition.py:step_5ms"


class TranscriptionError(ValueError):
    """Raised when the fixed transcription contract cannot be evaluated."""


class DerivativeContractError(TranscriptionError):
    """Raised when ML-241 metadata is absent, invalid, or nonsmooth."""


class ConstraintEvaluationRequired(TranscriptionError):
    """Raised when a future source-owner residual adapter was not supplied."""


@dataclass(frozen=True, slots=True)
class SegmentSpec:
    """Immutable scheduled hybrid metadata; no event inference is performed."""

    interval_start: int
    interval_end: int
    expected_contact_mode: str
    applicable_constraint_ids: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class GuardSpec:
    """Metadata for official/postcheck guards and approved continuous guards."""

    knot_index: int
    predicate_ids: tuple[str, ...] = ()
    smooth_residual_ids: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class SlackSpec:
    """One approved elastic slack binding, with no physical expression."""

    constraint_id: str
    row_instance: str
    units: str
    scale: float | str
    scale_source: str
    nonnegative: bool
    decision_index: int


@dataclass(frozen=True, slots=True)
class ConstraintRow:
    """Deterministic residual-row metadata and exact local support."""

    row_index: int
    group: str
    constraint_id: str
    component_index: int
    interval_index: int | None
    knot_index: int | None
    support: tuple[tuple[str, int], ...]
    sense: str
    differentiability: str
    slack_decision_index: int | None = None


@dataclass(frozen=True, slots=True)
class DecisionLayout:
    """Frozen state/action/slack decision-vector slices."""

    horizon: int
    state_dimension: int = STATE_DIMENSION
    action_dimension: int = ACTION_DIMENSION
    slack_specs: tuple[SlackSpec, ...] = ()

    def __post_init__(self) -> None:
        if int(self.horizon) < 1:
            raise TranscriptionError("horizon must be a positive interval count")
        if (self.state_dimension, self.action_dimension) != (132, 15):
            raise TranscriptionError("state/action dimensions are not the frozen V1 dimensions")
        if any(not slack.nonnegative for slack in self.slack_specs):
            raise TranscriptionError("all approved elastic slacks must be nonnegative")
        expected = self.base_dimension
        for slack in self.slack_specs:
            if slack.decision_index != expected:
                raise TranscriptionError("slack decision indices are not contiguous")
            expected += 1

    @property
    def base_dimension(self) -> int:
        return (self.horizon + 1) * self.state_dimension + self.horizon * self.action_dimension

    @property
    def total_dimension(self) -> int:
        return self.base_dimension + len(self.slack_specs)

    def state_slice(self, knot_index: int) -> slice:
        k = _index(knot_index, self.horizon + 1, "state knot")
        start = k * (self.state_dimension + self.action_dimension)
        return slice(start, start + self.state_dimension)

    def action_slice(self, interval_index: int) -> slice:
        k = _index(interval_index, self.horizon, "action interval")
        start = k * (self.state_dimension + self.action_dimension) + self.state_dimension
        return slice(start, start + self.action_dimension)

    def slack_slice(self, slack_index: int | SlackSpec) -> slice:
        if isinstance(slack_index, SlackSpec):
            index = slack_index.decision_index
        else:
            index = int(slack_index)
        if index < self.base_dimension or index >= self.total_dimension:
            raise TranscriptionError("slack index is outside the frozen slack suffix")
        return slice(index, index + 1)

    def pack_decision(
        self,
        state_deltas: Sequence[np.ndarray],
        actions: Sequence[np.ndarray],
        slacks: Sequence[float] = (),
    ) -> np.ndarray:
        states = _validate_matrix_sequence(state_deltas, self.horizon + 1, STATE_DIMENSION, "state deltas")
        commands = _validate_matrix_sequence(actions, self.horizon, ACTION_DIMENSION, "raw actions")
        slack_values = _validate_slacks(slacks, len(self.slack_specs))
        result = np.empty(self.total_dimension, dtype=np.float64)
        for k in range(self.horizon):
            result[self.state_slice(k)] = states[k]
            result[self.action_slice(k)] = commands[k]
        result[self.state_slice(self.horizon)] = states[-1]
        if slack_values.size:
            result[self.base_dimension:] = slack_values
        return result

    def unpack_decision(self, z: Sequence[float] | np.ndarray) -> "UnpackedDecision":
        value = _validate_vector(z, self.total_dimension, "decision vector")
        states = tuple(value[self.state_slice(k)].copy() for k in range(self.horizon + 1))
        actions = tuple(value[self.action_slice(k)].copy() for k in range(self.horizon))
        slacks = value[self.base_dimension:].copy()
        return UnpackedDecision(states=states, actions=actions, slacks=slacks)


@dataclass(frozen=True, slots=True)
class UnpackedDecision:
    """Structured tangent/action/slack view of one solver vector."""

    states: tuple[np.ndarray, ...]
    actions: tuple[np.ndarray, ...]
    slacks: np.ndarray


@dataclass(frozen=True, slots=True)
class IntervalEvaluation:
    """One exact reconstructed interval and its predicted next snapshot."""

    state: MacroSnapshot
    next_state: MacroSnapshot
    predicted_state: MacroSnapshot
    raw_action: np.ndarray
    predicted_time_advance_s: float
    defect: np.ndarray


@dataclass(frozen=True, slots=True)
class EndpointJacobianResult:
    """Finite-difference endpoint maps for one boxminus residual."""

    G_pred: np.ndarray
    G_next: np.ndarray
    steps: np.ndarray
    zero_defect: bool
    defect_norm: float
    pred_identity_error: float
    next_identity_error: float


@dataclass(frozen=True, slots=True)
class DirectionalCheck:
    """Direct endpoint residual comparison for deterministic directions."""

    pred_max_error: float
    next_max_error: float
    epsilons: tuple[float, ...]


DerivativeProvider = Callable[..., WrappedLinearization]
ConstraintEvaluator = Callable[[ConstraintRow, np.ndarray, "DirectMultipleShootingProblem"], float | np.ndarray]


def _index(value: int, upper: int, label: str) -> int:
    index = int(value)
    if index != value or index < 0 or index >= upper:
        raise TranscriptionError(f"{label} {value!r} is outside [0, {upper})")
    return index


def _validate_vector(value: Sequence[float] | np.ndarray, size: int, label: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TranscriptionError(f"{label} must be a float vector") from exc
    if result.shape != (size,):
        raise TranscriptionError(f"{label} shape {result.shape} != ({size},)")
    if not np.isfinite(result).all():
        raise TranscriptionError(f"{label} contains NaN or Inf")
    return result.copy()


def _validate_matrix_sequence(
    values: Sequence[np.ndarray], count: int, width: int, label: str
) -> tuple[np.ndarray, ...]:
    try:
        sequence = tuple(values)
    except TypeError as exc:
        raise TranscriptionError(f"{label} must be a sequence") from exc
    if len(sequence) != count:
        raise TranscriptionError(f"{label} count {len(sequence)} != {count}")
    return tuple(_validate_vector(item, width, f"{label}[{index}]") for index, item in enumerate(sequence))


def _validate_slacks(values: Sequence[float], count: int) -> np.ndarray:
    try:
        result = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TranscriptionError("slacks must be a float vector") from exc
    if result.shape != (count,):
        raise TranscriptionError(f"slacks shape {result.shape} != ({count},)")
    if not np.isfinite(result).all():
        raise TranscriptionError("slacks contain NaN or Inf")
    if np.any(result < 0.0):
        raise TranscriptionError("elastic slacks must be nonnegative")
    return result.copy()


def _drive_from_snapshot(snapshot: MacroSnapshot) -> drive.DriveState:
    return drive.DriveState(
        a_plus=snapshot.a_plus,
        a_minus=snapshot.a_minus,
        tau_prev=snapshot.tau_prev,
        previous_command=snapshot.previous_command,
        override_flags={key: value.copy() for key, value in snapshot.override_flags},
        reversal_phase=snapshot.reversal_phase,
    )


def boxminus_endpoint_jacobians(
    *,
    next_snapshot: MacroSnapshot,
    predicted_snapshot: MacroSnapshot,
    model: mujoco.MjModel,
    steps: Mapping[str, float] = ML241_STATE_STEPS,
) -> EndpointJacobianResult:
    """Differentiate only the two endpoint maps of ``boxminus``.

    No transition is called here.  All endpoint perturbations use the frozen
    ``boxplus`` chart and all outputs use the frozen ``boxminus`` chart.
    """

    if set(steps) != set(STATE_STEP_BLOCKS):
        raise TranscriptionError("endpoint steps do not match the ML-241 block contract")
    step_vector = np.empty(STATE_DIMENSION, dtype=np.float64)
    for block, indexes in _STATE_BLOCK_INDEXES:
        value = float(steps[block])
        if not np.isfinite(value) or value <= 0.0:
            raise TranscriptionError(f"endpoint step for {block} is not positive and finite")
        step_vector[list(indexes)] = value

    base_defect = boxminus(next_snapshot, predicted_snapshot, model=model)
    G_pred = np.empty((STATE_DIMENSION, STATE_DIMENSION), dtype=np.float64)
    G_next = np.empty_like(G_pred)
    for index, step in enumerate(step_vector):
        direction = np.zeros(STATE_DIMENSION, dtype=np.float64)
        direction[index] = step
        pred_plus = boxplus(predicted_snapshot, direction, model=model)
        pred_minus = boxplus(predicted_snapshot, -direction, model=model)
        next_plus = boxplus(next_snapshot, direction, model=model)
        next_minus = boxplus(next_snapshot, -direction, model=model)
        G_pred[:, index] = (
            boxminus(next_snapshot, pred_plus, model=model)
            - boxminus(next_snapshot, pred_minus, model=model)
        ) / (2.0 * step)
        G_next[:, index] = (
            boxminus(next_plus, predicted_snapshot, model=model)
            - boxminus(next_minus, predicted_snapshot, model=model)
        ) / (2.0 * step)
    if not np.isfinite(G_pred).all() or not np.isfinite(G_next).all():
        raise TranscriptionError("boxminus endpoint Jacobian contains NaN or Inf")
    zero = bool(np.linalg.norm(base_defect, ord=np.inf) <= 1.0e-10)
    return EndpointJacobianResult(
        G_pred=G_pred,
        G_next=G_next,
        steps=step_vector,
        zero_defect=zero,
        defect_norm=float(np.linalg.norm(base_defect, ord=np.inf)),
        pred_identity_error=float(np.max(np.abs(G_pred + np.eye(STATE_DIMENSION))),),
        next_identity_error=float(np.max(np.abs(G_next - np.eye(STATE_DIMENSION))),),
    )


def qualify_endpoint_directional_maps(
    *,
    result: EndpointJacobianResult,
    next_snapshot: MacroSnapshot,
    predicted_snapshot: MacroSnapshot,
    model: mujoco.MjModel,
    directions: Sequence[np.ndarray],
    epsilons: Sequence[float] = (1.0e-3, 3.0e-4, 1.0e-4),
) -> DirectionalCheck:
    """Compare endpoint products with direct boxminus perturbations."""

    errors_pred: list[float] = []
    errors_next: list[float] = []
    normalized = tuple(_validate_vector(direction, STATE_DIMENSION, "direction") for direction in directions)
    for direction in normalized:
        norm = float(np.linalg.norm(direction))
        if norm == 0.0:
            raise TranscriptionError("endpoint directional checks require nonzero directions")
        direction = direction / norm
        for epsilon in epsilons:
            h = float(epsilon)
            if not np.isfinite(h) or h <= 0.0:
                raise TranscriptionError("endpoint epsilon must be positive and finite")
            pred_plus = boxplus(predicted_snapshot, h * direction, model=model)
            pred_minus = boxplus(predicted_snapshot, -h * direction, model=model)
            next_plus = boxplus(next_snapshot, h * direction, model=model)
            next_minus = boxplus(next_snapshot, -h * direction, model=model)
            direct_pred = (
                boxminus(next_snapshot, pred_plus, model=model)
                - boxminus(next_snapshot, pred_minus, model=model)
            ) / (2.0 * h)
            direct_next = (
                boxminus(next_plus, predicted_snapshot, model=model)
                - boxminus(next_minus, predicted_snapshot, model=model)
            ) / (2.0 * h)
            errors_pred.append(float(np.max(np.abs(direct_pred - result.G_pred @ direction))))
            errors_next.append(float(np.max(np.abs(direct_next - result.G_next @ direction))))
    return DirectionalCheck(
        pred_max_error=max(errors_pred, default=0.0),
        next_max_error=max(errors_next, default=0.0),
        epsilons=tuple(float(value) for value in epsilons),
    )


def advance_snapshot_exact(
    *,
    plant: Plant,
    snapshot: MacroSnapshot,
    raw_action: Sequence[float] | np.ndarray,
    control_dt: float = CONTROL_DT_S,
) -> MacroSnapshot:
    """Advance one reconstructed snapshot through the shared exact owner."""

    if float(control_dt) != CONTROL_DT_S:
        raise TranscriptionError("exact snapshot advancement requires the frozen 5 ms interval")
    action = _validate_vector(raw_action, ACTION_DIMENSION, "raw action")
    data = mujoco.MjData(plant.model)
    drive_state = _drive_from_snapshot(snapshot)
    previous = snapshot.restore(plant=plant, data=data, drive_state=drive_state)
    try:
        result = step_5ms(
            plant=plant,
            data=data,
            drive_state=drive_state,
            previous_accepted_action=previous,
            raw_action=action,
        )
        if result.substeps_executed != 40:
            raise TranscriptionError("exact transition did not execute 40 physics substeps")
        predicted = MacroSnapshot.capture(
            plant=plant,
            data=data,
            drive_state=drive_state,
            previous_accepted_action=result.accepted_action,
        )
    except (ValueError, RuntimeError) as exc:
        raise TranscriptionError("exact transition rejected the reconstructed state/action") from exc
    actual_advance = float(predicted.time - snapshot.time)
    if not np.isclose(actual_advance, CONTROL_DT_S, rtol=0.0, atol=1.0e-12):
        raise TranscriptionError(
            f"predicted time advance {actual_advance} != frozen control dt {CONTROL_DT_S}"
        )
    return predicted


class DirectMultipleShootingProblem:
    """One fixed-reference exact-discrete transcription problem instance."""

    _sealed: bool = False

    def __setattr__(self, name: str, value: Any) -> None:
        if getattr(self, "_sealed", False):
            raise AttributeError("DirectMultipleShootingProblem is immutable after construction")
        object.__setattr__(self, name, value)

    def __init__(
        self,
        *,
        plant: Plant,
        reference_snapshots: Sequence[MacroSnapshot],
        control_dt: float = CONTROL_DT_S,
        segments: Sequence[SegmentSpec] = (),
        guards: Sequence[GuardSpec] = (),
        catalog: ConstraintCatalog = CATALOG,
        fixed_initial_state: bool = True,
        include_elastic_slacks: bool = False,
        derivative_provider: DerivativeProvider | None = None,
    ) -> None:
        self.plant = plant
        self.control_dt = float(control_dt)
        self.catalog = catalog
        self.fixed_initial_state = bool(fixed_initial_state)
        self.include_elastic_slacks = bool(include_elastic_slacks)
        self.derivative_provider = derivative_provider
        self.reference_snapshots = tuple(reference_snapshots)
        self.horizon = len(self.reference_snapshots) - 1
        self.segments = tuple(segments)
        self.guards = tuple(guards)
        self._validate_references()
        self.time_schedule = tuple(
            float(self.reference_snapshots[0].time + index * self.control_dt)
            for index in range(self.horizon + 1)
        )
        self._validate_schedule()
        self._validate_metadata()
        self._elastic_bindings = self._build_elastic_bindings()
        self.layout = DecisionLayout(self.horizon, slack_specs=self._elastic_bindings)
        self.constraint_rows = self._build_constraint_rows()
        self._sealed = True

    @property
    def variable_count(self) -> int:
        return self.layout.total_dimension

    @property
    def constraint_count(self) -> int:
        return len(self.constraint_rows)

    @property
    def dynamics_row_count(self) -> int:
        return self.horizon * STATE_DIMENSION

    @property
    def reference_time_schedule(self) -> tuple[float, ...]:
        return self.time_schedule

    @property
    def smooth_nlp_constraint_count(self) -> int:
        return sum(row.group in {"smooth_path", "smooth_guard", "smooth_terminal"} for row in self.constraint_rows)

    @property
    def postcheck_discrete_count(self) -> int:
        return sum(
            spec.enforcement in {"POSTCHECK_EVENT_ENGINE", "POSTCHECK_MECHANICS"}
            or spec.differentiability not in _SMOOTH_DIFFERENTIABILITY
            for spec in self.catalog.specs
        )

    @property
    def elastic_slack_count(self) -> int:
        return len(self.layout.slack_specs)

    @property
    def elastic_slack_schema(self) -> tuple[dict[str, Any], ...]:
        return tuple(
            {
                "constraint_id": spec.constraint_id,
                "phase_i_scale": spec.phase_i_scale,
                "phase_i_scale_source": spec.phase_i_scale_source,
                "elastic_use": spec.constraint_id in APPROVED_ELASTIC_CONSTRAINT_IDS,
                "nonnegative": True,
            }
            for spec in self.catalog.specs
            if spec.constraint_id in APPROVED_ELASTIC_CONSTRAINT_IDS
        )

    def _validate_references(self) -> None:
        if self.horizon < 1:
            raise TranscriptionError("reference trajectory needs at least one interval")
        if self.control_dt != CONTROL_DT_S:
            raise TranscriptionError("control dt is not the frozen 5 ms interval")
        model_dims = (int(self.plant.model.nq), int(self.plant.model.nv), int(self.plant.model.nu))
        if model_dims != (25, 21, 15):
            raise TranscriptionError("Plant dimensions are not the frozen V1 model")
        for index, snapshot in enumerate(self.reference_snapshots):
            if not isinstance(snapshot, MacroSnapshot):
                raise TranscriptionError(f"reference snapshot {index} is not MacroSnapshot")
            if snapshot.schema_version != SNAPSHOT_SCHEMA_VERSION:
                raise TranscriptionError(f"reference snapshot {index} has the wrong schema")
            if snapshot.dimensions[:3] != model_dims:
                raise TranscriptionError(f"reference snapshot {index} has incompatible dimensions")
            if not np.isfinite(snapshot.time):
                raise TranscriptionError(f"reference snapshot {index} has non-finite time")

    def _validate_schedule(self) -> None:
        for index, (snapshot, expected) in enumerate(zip(self.reference_snapshots, self.time_schedule, strict=True)):
            if not np.isclose(snapshot.time, expected, rtol=0.0, atol=1.0e-12):
                raise TranscriptionError(
                    f"reference time schedule mismatch at knot {index}: {snapshot.time} != {expected}"
                )

    def _validate_metadata(self) -> None:
        known = {spec.constraint_id for spec in self.catalog.specs}
        for segment in self.segments:
            if not 0 <= segment.interval_start < segment.interval_end <= self.horizon:
                raise TranscriptionError("segment interval is outside the horizon")
            if not segment.expected_contact_mode:
                raise TranscriptionError("segment contact mode is empty")
            unknown = set(segment.applicable_constraint_ids).difference(known)
            if unknown:
                raise TranscriptionError(f"segment contains unknown constraint IDs: {sorted(unknown)}")
        for guard in self.guards:
            _index(guard.knot_index, self.horizon + 1, "guard knot")
            unknown = (set(guard.predicate_ids) | set(guard.smooth_residual_ids)).difference(known)
            if unknown:
                raise TranscriptionError(f"guard contains unknown constraint IDs: {sorted(unknown)}")
            for constraint_id in guard.smooth_residual_ids:
                spec = self.catalog.get(constraint_id)
                if not self._is_smooth_hard(spec) or spec.classification != "HARD_GUARD":
                    raise TranscriptionError(f"guard residual {constraint_id} is not an approved smooth hard guard")

    @staticmethod
    def _is_smooth_hard(spec: ConstraintSpec) -> bool:
        return (
            spec.classification in {"HARD_PATH", "HARD_GUARD", "HARD_TERMINAL"}
            and spec.enforcement == "EXPLICIT_NLP_LATER"
            and spec.differentiability in _SMOOTH_DIFFERENTIABILITY
        )

    def _build_elastic_bindings(self) -> tuple[SlackSpec, ...]:
        if not self.include_elastic_slacks:
            return ()
        # Only rows explicitly represented in this problem receive decision
        # slacks.  The complete ML-240 allowlist remains available as schema
        # metadata, including deferred owner-derived scales.
        bindings: list[SlackSpec] = []
        next_index = self.horizon * (STATE_DIMENSION + ACTION_DIMENSION) + STATE_DIMENSION
        for spec in self.catalog.specs:
            if spec.constraint_id not in APPROVED_ELASTIC_CONSTRAINT_IDS:
                continue
            if spec.enforcement != "EXPLICIT_NLP_LATER":
                continue
            if not self._is_smooth_hard(spec):
                continue
            if not self._constraint_is_applicable(spec.constraint_id):
                continue
            instances = self._constraint_instances(spec)
            for instance in instances:
                bindings.append(
                    SlackSpec(
                        constraint_id=spec.constraint_id,
                        row_instance=instance,
                        units=spec.units,
                        scale=spec.phase_i_scale,
                        scale_source=spec.phase_i_scale_source,
                        nonnegative=True,
                        decision_index=next_index,
                    )
                )
                next_index += 1
        return tuple(bindings)

    def _constraint_is_applicable(self, constraint_id: str) -> bool:
        if not self.segments:
            return False
        return any(constraint_id in segment.applicable_constraint_ids for segment in self.segments)

    def _constraint_instances(self, spec: ConstraintSpec) -> tuple[str, ...]:
        if spec.classification == "HARD_TERMINAL":
            return (f"knot:{self.horizon}",)
        if spec.evaluation_domain == "state":
            return tuple(f"knot:{k}" for k in range(self.horizon + 1))
        if spec.evaluation_domain == "transition":
            return tuple(f"interval:{k}" for k in range(self.horizon))
        return ()

    def _support_for_spec(self, spec: ConstraintSpec, index: int) -> tuple[tuple[str, int], ...]:
        if spec.evaluation_domain == "state":
            return (("state", index),)
        if spec.evaluation_domain == "transition":
            return (("state", index), ("action", index))
        return ()

    def _build_constraint_rows(self) -> tuple[ConstraintRow, ...]:
        rows: list[ConstraintRow] = []
        for interval in range(self.horizon):
            for component in range(STATE_DIMENSION):
                rows.append(
                    ConstraintRow(
                        row_index=len(rows),
                        group="dynamics",
                        constraint_id="DYNAMICS_DEFECT",
                        component_index=component,
                        interval_index=interval,
                        knot_index=None,
                        support=(("state", interval), ("action", interval), ("state", interval + 1)),
                        sense="EQUALITY_H_EQ_0",
                        differentiability="SMOOTH_MODE_LOCAL",
                    )
                )

        seen: set[tuple[str, int | None, int | None, str]] = set()
        for segment in sorted(self.segments, key=lambda item: (item.interval_start, item.interval_end, item.expected_contact_mode)):
            for constraint_id in sorted(segment.applicable_constraint_ids):
                spec = self.catalog.get(constraint_id)
                if not self._is_smooth_hard(spec):
                    continue
                if spec.classification == "HARD_TERMINAL":
                    candidates = ((None, self.horizon),) if segment.interval_end == self.horizon else ()
                    group = "smooth_terminal"
                elif spec.classification == "HARD_GUARD":
                    candidates = tuple(
                        (None, guard.knot_index)
                        for guard in sorted(self.guards, key=lambda item: item.knot_index)
                        if constraint_id in guard.smooth_residual_ids
                    )
                    group = "smooth_guard"
                elif spec.evaluation_domain == "state":
                    candidates = tuple((None, knot) for knot in range(segment.interval_start, segment.interval_end + 1))
                    group = "smooth_path"
                else:
                    candidates = tuple((interval, None) for interval in range(segment.interval_start, segment.interval_end))
                    group = "smooth_path"
                for interval, knot in candidates:
                    key = (constraint_id, interval, knot, group)
                    if key in seen:
                        continue
                    seen.add(key)
                    support_index = knot if knot is not None else int(interval)
                    slack_index = self._slack_index(constraint_id, interval, knot)
                    rows.append(
                        ConstraintRow(
                            row_index=len(rows),
                            group=group,
                            constraint_id=constraint_id,
                            component_index=0,
                            interval_index=interval,
                            knot_index=knot,
                            support=self._support_for_spec(spec, support_index),
                            sense=spec.sense,
                            differentiability=spec.differentiability,
                            slack_decision_index=slack_index,
                        )
                    )
        return tuple(rows)

    def _slack_index(self, constraint_id: str, interval: int | None, knot: int | None) -> int | None:
        wanted = f"knot:{knot}" if knot is not None else f"interval:{interval}"
        for slack in self.layout.slack_specs if hasattr(self, "layout") else self._elastic_bindings:
            if slack.constraint_id == constraint_id and slack.row_instance == wanted:
                return slack.decision_index
        return None

    def state_slice(self, knot_index: int) -> slice:
        return self.layout.state_slice(knot_index)

    def action_slice(self, interval_index: int) -> slice:
        return self.layout.action_slice(interval_index)

    def defect_slice(self, interval_index: int) -> slice:
        k = _index(interval_index, self.horizon, "dynamics interval")
        return slice(k * STATE_DIMENSION, (k + 1) * STATE_DIMENSION)

    def slack_slice(self, slack_index: int | SlackSpec) -> slice:
        return self.layout.slack_slice(slack_index)

    def pack_decision(
        self,
        state_deltas: Sequence[np.ndarray],
        actions: Sequence[np.ndarray],
        slacks: Sequence[float] = (),
    ) -> np.ndarray:
        return self.layout.pack_decision(state_deltas, actions, slacks)

    def unpack_decision(self, z: Sequence[float] | np.ndarray) -> UnpackedDecision:
        return self.layout.unpack_decision(z)

    def _reconstruct_state(self, knot: int, delta: np.ndarray) -> MacroSnapshot:
        try:
            return boxplus(self.reference_snapshots[knot], delta, model=self.plant.model)
        except (ValueError, RuntimeError) as exc:
            raise TranscriptionError(f"knot {knot} leaves the frozen tangent chart") from exc

    def _propagate(self, state: MacroSnapshot, raw_action: np.ndarray) -> MacroSnapshot:
        return advance_snapshot_exact(
            plant=self.plant,
            snapshot=state,
            raw_action=raw_action,
            control_dt=self.control_dt,
        )

    def interval_evaluation(self, z: Sequence[float] | np.ndarray, interval: int) -> IntervalEvaluation:
        k = _index(interval, self.horizon, "dynamics interval")
        decision = self.unpack_decision(z)
        state = self._reconstruct_state(k, decision.states[k])
        next_state = self._reconstruct_state(k + 1, decision.states[k + 1])
        predicted = self._propagate(state, decision.actions[k])
        defect = boxminus(next_state, predicted, model=self.plant.model)
        return IntervalEvaluation(
            state=state,
            next_state=next_state,
            predicted_state=predicted,
            raw_action=decision.actions[k].copy(),
            predicted_time_advance_s=float(predicted.time - state.time),
            defect=defect,
        )

    def defect_values(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        result = np.empty(self.dynamics_row_count, dtype=np.float64)
        for interval in range(self.horizon):
            result[self.defect_slice(interval)] = self.interval_evaluation(z, interval).defect
        return result

    def constraint_values(
        self,
        z: Sequence[float] | np.ndarray,
        *,
        evaluator: ConstraintEvaluator | None = None,
    ) -> np.ndarray:
        value = _validate_vector(z, self.variable_count, "decision vector")
        residuals = np.empty(self.constraint_count, dtype=np.float64)
        residuals[: self.dynamics_row_count] = self.defect_values(value)
        for row in self.constraint_rows[self.dynamics_row_count :]:
            if evaluator is None:
                raise ConstraintEvaluationRequired(
                    f"no source-owner evaluator supplied for catalog row {row.constraint_id}"
                )
            row_value = np.asarray(evaluator(row, value, self), dtype=np.float64).reshape(-1)
            if row_value.shape != (1,) or not np.isfinite(row_value).all():
                raise ConstraintEvaluationRequired(f"evaluator returned invalid value for {row.constraint_id}")
            residuals[row.row_index] = float(row_value[0])
            if row.slack_decision_index is not None:
                residuals[row.row_index] += value[row.slack_decision_index]
        return residuals

    def _derivatives(self, state: MacroSnapshot, action: np.ndarray) -> WrappedLinearization:
        provider = linearize_step_5ms if self.derivative_provider is None else self.derivative_provider
        try:
            result = provider(
                plant=self.plant,
                base_snapshot=state,
                raw_action=action,
                state_steps=ML241_STATE_STEPS,
                action_steps=ML241_ACTION_STEP,
            )
        except Exception as exc:
            if isinstance(exc, DerivativeContractError):
                raise
            raise DerivativeContractError("ML-241 ordinary local derivative assembly was rejected") from exc
        self._validate_derivative_metadata(result, state, action)
        return result

    @staticmethod
    def _validate_derivative_metadata(
        result: WrappedLinearization, state: MacroSnapshot, action: np.ndarray
    ) -> None:
        if result.A.shape != (STATE_DIMENSION, STATE_DIMENSION) or result.B.shape != (STATE_DIMENSION, ACTION_DIMENSION):
            raise DerivativeContractError("ML-241 A/B shapes are not 132x132 and 132x15")
        if result.tangent_layout_id != DERIVATIVE_TANGENT_LAYOUT_ID:
            raise DerivativeContractError("ML-241 tangent-layout identity changed")
        if result.snapshot_schema_id != SNAPSHOT_SCHEMA_VERSION:
            raise DerivativeContractError("ML-241 snapshot-schema identity changed")
        if result.transition_owner != _TRANSITION_OWNER:
            raise DerivativeContractError("ML-241 transition owner changed")
        if result.constraint_catalog_id != CONSTRAINT_CATALOG_ID:
            raise DerivativeContractError("ML-241 constraint-catalog identity changed")
        if result.scheme != "central_boxminus_at_common_y0":
            raise DerivativeContractError("ML-241 derivative output frame changed")
        if tuple(float(value) for value in result.action_step_metadata) != (ML241_ACTION_STEP,) * ACTION_DIMENSION:
            raise DerivativeContractError("ML-241 action derivative steps changed")
        if dict(result.state_step_metadata) != dict(ML241_STATE_STEPS):
            raise DerivativeContractError("ML-241 state derivative steps changed")
        if result.qacc_derivative_disposition != QACC_DERIVATIVE_DISPOSITION:
            raise DerivativeContractError("ML-241 qacc qualified-zero metadata is missing")
        if tuple(result.qacc_zero_columns) != QACC_ZERO_COLUMNS:
            raise DerivativeContractError("ML-241 qacc zero-column metadata is not 111:131")
        bounds = dict(result.qacc_absolute_error_bound)
        if bounds != dict(QACC_ERROR_BOUNDS):
            raise DerivativeContractError("ML-241 qacc error bounds changed")
        if not result.qacc_certificate_evidence_id:
            raise DerivativeContractError("ML-241 qacc certificate evidence identity is missing")
        branches = tuple(result.base_active_set.action_branches)
        if len(branches) != ACTION_DIMENSION or any(
            branch in {"NEAR_KINK", "ACTION_BOUND_ACTIVE"} for branch in branches
        ):
            raise DerivativeContractError("ML-241 action branch is not an ordinary smooth local branch")
        if result.base_snapshot_digest != snapshot_digest(state) or result.base_raw_action.shape != (ACTION_DIMENSION,):
            raise DerivativeContractError("ML-241 base-point metadata is incomplete")
        if not np.array_equal(np.asarray(result.base_raw_action), action):
            raise DerivativeContractError("ML-241 derivative base action does not match the decision")
        if not np.isfinite(result.A).all() or not np.isfinite(result.B).all():
            raise DerivativeContractError("ordinary local derivative contains invalid columns")
        if not np.asarray(result.state_validity, dtype=bool).all():
            invalid = np.flatnonzero(~np.asarray(result.state_validity, dtype=bool)).tolist()
            raise DerivativeContractError(f"ordinary local A columns are nonsmooth/invalid: {invalid}")
        if not np.asarray(result.action_validity, dtype=bool).all():
            invalid = np.flatnonzero(~np.asarray(result.action_validity, dtype=bool)).tolist()
            raise DerivativeContractError(f"ordinary local B columns are nonsmooth/invalid: {invalid}")
        if not np.array_equal(result.A[:, QACC_ZERO_COLUMNS], np.zeros((STATE_DIMENSION, len(QACC_ZERO_COLUMNS)))):
            raise DerivativeContractError("qualified-zero qacc columns were altered")

    def jacobian_values(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        value = _validate_vector(z, self.variable_count, "decision vector")
        entries: list[float] = []
        decision = self.unpack_decision(value)
        for interval in range(self.horizon):
            evaluation = self.interval_evaluation(value, interval)
            linearization = self._derivatives(evaluation.state, evaluation.raw_action)
            endpoint = boxminus_endpoint_jacobians(
                next_snapshot=evaluation.next_state,
                predicted_snapshot=evaluation.predicted_state,
                model=self.plant.model,
            )
            J_state = endpoint.G_pred @ linearization.A
            J_action = endpoint.G_pred @ linearization.B
            J_next = endpoint.G_next
            for component in range(STATE_DIMENSION):
                entries.extend(J_state[component].tolist())
                entries.extend(J_action[component].tolist())
                entries.extend(J_next[component].tolist())
        for row in self.constraint_rows[self.dynamics_row_count :]:
            raise ConstraintEvaluationRequired(
                f"no source-owner Jacobian adapter supplied for catalog row {row.constraint_id}"
            )
        expected = len(self.jacobian_structure()[0])
        if len(entries) != expected:
            raise TranscriptionError(f"Jacobian value count {len(entries)} != structure count {expected}")
        result = np.asarray(entries, dtype=np.float64)
        if not np.isfinite(result).all():
            raise TranscriptionError("assembled Jacobian contains NaN or Inf")
        return result

    def jacobian_matvec(
        self,
        z: Sequence[float] | np.ndarray,
        direction: Sequence[float] | np.ndarray,
    ) -> np.ndarray:
        """Evaluate ``J(z) @ direction`` from the frozen COO ordering."""

        vector = _validate_vector(direction, self.variable_count, "Jacobian direction")
        rows, cols = self.jacobian_structure()
        values = self.jacobian_values(z)
        result = np.zeros(self.constraint_count, dtype=np.float64)
        np.add.at(result, rows, values * vector[cols])
        return result

    def jacobian_structure(self) -> tuple[np.ndarray, np.ndarray]:
        rows: list[int] = []
        cols: list[int] = []
        for interval in range(self.horizon):
            state_start = self.layout.state_slice(interval).start
            action_start = self.layout.action_slice(interval).start
            next_start = self.layout.state_slice(interval + 1).start
            for component in range(STATE_DIMENSION):
                row = interval * STATE_DIMENSION + component
                rows.extend([row] * (STATE_DIMENSION + ACTION_DIMENSION + STATE_DIMENSION))
                cols.extend(range(state_start, state_start + STATE_DIMENSION))
                cols.extend(range(action_start, action_start + ACTION_DIMENSION))
                cols.extend(range(next_start, next_start + STATE_DIMENSION))
        for row in self.constraint_rows[self.dynamics_row_count :]:
            for kind, index in row.support:
                slc = self.layout.state_slice(index) if kind == "state" else self.layout.action_slice(index)
                rows.extend([row.row_index] * (slc.stop - slc.start))
                cols.extend(range(slc.start, slc.stop))
            if row.slack_decision_index is not None:
                rows.append(row.row_index)
                cols.append(row.slack_decision_index)
        return np.asarray(rows, dtype=np.int64), np.asarray(cols, dtype=np.int64)

    def jacobian_structure_hash(self) -> str:
        rows, cols = self.jacobian_structure()
        digest = sha256()
        digest.update(rows.tobytes(order="C"))
        digest.update(cols.tobytes(order="C"))
        return digest.hexdigest()

    def jacobian_rows(self) -> np.ndarray:
        return self.jacobian_structure()[0]

    def jacobian_cols(self) -> np.ndarray:
        return self.jacobian_structure()[1]

    def variable_lower_bounds(self) -> np.ndarray:
        lower = np.full(self.variable_count, -np.inf, dtype=np.float64)
        for interval in range(self.horizon):
            lower[self.layout.action_slice(interval)] = -1.0
        if self.fixed_initial_state:
            lower[self.layout.state_slice(0)] = 0.0
        for slack in self.layout.slack_specs:
            lower[slack.decision_index] = 0.0
        return lower

    def variable_upper_bounds(self) -> np.ndarray:
        upper = np.full(self.variable_count, np.inf, dtype=np.float64)
        for interval in range(self.horizon):
            upper[self.layout.action_slice(interval)] = 1.0
        if self.fixed_initial_state:
            upper[self.layout.state_slice(0)] = 0.0
        return upper

    def constraint_lower_bounds(self) -> np.ndarray:
        lower = np.full(self.constraint_count, -np.inf, dtype=np.float64)
        lower[: self.dynamics_row_count] = 0.0
        for row in self.constraint_rows[self.dynamics_row_count :]:
            if row.sense == "MARGIN_G_GE_0":
                lower[row.row_index] = 0.0
            elif row.sense == "EQUALITY_H_EQ_0":
                lower[row.row_index] = 0.0
        return lower

    def constraint_upper_bounds(self) -> np.ndarray:
        upper = np.full(self.constraint_count, np.inf, dtype=np.float64)
        upper[: self.dynamics_row_count] = 0.0
        for row in self.constraint_rows[self.dynamics_row_count :]:
            if row.sense == "EQUALITY_H_EQ_0":
                upper[row.row_index] = 0.0
        return upper

    def decision_layout_record(self) -> dict[str, Any]:
        return {
            "schema_id": TRANSCRIPTION_SCHEMA_ID,
            "state_dimension": STATE_DIMENSION,
            "action_dimension": ACTION_DIMENSION,
            "horizon": self.horizon,
            "control_dt_s": self.control_dt,
            "ordering": "delta_x[0], u[0], ..., delta_x[N-1], u[N-1], delta_x[N], slack_suffix",
            "base_dimension": self.layout.base_dimension,
            "total_dimension": self.layout.total_dimension,
            "fixed_initial_state": self.fixed_initial_state,
            "state_slices": [self.layout.state_slice(k).indices(self.layout.total_dimension) for k in range(self.horizon + 1)],
            "action_slices": [self.layout.action_slice(k).indices(self.layout.total_dimension) for k in range(self.horizon)],
            "slack_slices": [self.layout.slack_slice(slack).indices(self.layout.total_dimension) for slack in self.layout.slack_specs],
        }

    def row_layout_record(self) -> dict[str, Any]:
        return {
            "dynamics_row_count": self.dynamics_row_count,
            "constraint_count": self.constraint_count,
            "rows": [
                {
                    "row_index": row.row_index,
                    "group": row.group,
                    "constraint_id": row.constraint_id,
                    "component_index": row.component_index,
                    "interval_index": row.interval_index,
                    "knot_index": row.knot_index,
                    "support": list(row.support),
                    "sense": row.sense,
                    "differentiability": row.differentiability,
                    "slack_decision_index": row.slack_decision_index,
                }
                for row in self.constraint_rows
            ],
        }

    def constraint_schema_record(self) -> dict[str, Any]:
        terminal = []
        for identifier in ("E3_E4_SUPPORTED", "E3_E4_REVERSAL", "E3_E4_HORIZONTAL"):
            spec = self.catalog.get(identifier)
            terminal.append(
                {
                    "constraint_id": identifier,
                    "classification": spec.classification,
                    "enforcement": spec.enforcement,
                    "differentiability": spec.differentiability,
                    "representation": (
                        "explicit_smooth_terminal_residual"
                        if self._is_smooth_hard(spec)
                        else "discrete_or_postcheck_guard_metadata"
                    ),
                    "sense": spec.sense,
                    "owner_module": spec.owner_module,
                    "owner_symbol": spec.owner_symbol,
                    "phase_applicability": list(spec.phase_applicability),
                }
            )
        dispositions = []
        for spec in self.catalog.specs:
            if spec.enforcement == "INTRINSIC_TRANSITION":
                disposition = "intrinsic_transition"
            elif spec.enforcement == "EXPLICIT_NLP_LATER":
                disposition = "explicit_nlp_later"
            elif spec.enforcement == "AGGREGATE_NLP_LATER":
                disposition = "aggregate_sequence_metadata"
            elif spec.enforcement == "POSTCHECK_EVENT_ENGINE":
                disposition = "postcheck_event_engine"
            elif spec.enforcement == "POSTCHECK_MECHANICS":
                disposition = "postcheck_mechanics"
            else:
                disposition = "metadata_only"
            dispositions.append({"constraint_id": spec.constraint_id, "disposition": disposition})
        return {
            "catalog_id": self.catalog.catalog_id,
            "constraint_count": len(self.catalog.specs),
            "capturability": "diagnostic_only",
            "terminal_items": terminal,
            "dispositions": dispositions,
            "ordinary_smooth_rows": [row.constraint_id for row in self.constraint_rows if row.group != "dynamics"],
            "discrete_posttrace_rows_excluded": True,
        }

    def segment_guard_record(self) -> dict[str, Any]:
        return {
            "segments": [
                {
                    "interval_start": item.interval_start,
                    "interval_end": item.interval_end,
                    "expected_contact_mode": item.expected_contact_mode,
                    "applicable_constraint_ids": list(item.applicable_constraint_ids),
                }
                for item in self.segments
            ],
            "guards": [
                {
                    "knot_index": item.knot_index,
                    "predicate_ids": list(item.predicate_ids),
                    "smooth_residual_ids": list(item.smooth_residual_ids),
                }
                for item in self.guards
            ],
            "dynamic_phase_inference": False,
        }


def pack_decision(
    layout: DecisionLayout,
    state_deltas: Sequence[np.ndarray],
    actions: Sequence[np.ndarray],
    slacks: Sequence[float] = (),
) -> np.ndarray:
    """Pack through the one frozen layout owner."""

    return layout.pack_decision(state_deltas, actions, slacks)


def unpack_decision(layout: DecisionLayout, z: Sequence[float] | np.ndarray) -> UnpackedDecision:
    """Unpack through the one frozen layout owner."""

    return layout.unpack_decision(z)


__all__ = [
    "ACTION_DIMENSION",
    "advance_snapshot_exact",
    "APPROVED_ELASTIC_CONSTRAINT_IDS",
    "CONTROL_DT_S",
    "ConstraintEvaluationRequired",
    "ConstraintRow",
    "DecisionLayout",
    "DerivativeContractError",
    "DirectMultipleShootingProblem",
    "DirectionalCheck",
    "EndpointJacobianResult",
    "GuardSpec",
    "IntervalEvaluation",
    "ML241_ACTION_STEP",
    "ML241_STATE_STEPS",
    "QACC_DERIVATIVE_DISPOSITION",
    "QACC_ERROR_BOUNDS",
    "QACC_ZERO_COLUMNS",
    "SegmentSpec",
    "SlackSpec",
    "STATE_DIMENSION",
    "TRANSCRIPTION_SCHEMA_ID",
    "TranscriptionError",
    "UnpackedDecision",
    "boxminus_endpoint_jacobians",
    "pack_decision",
    "qualify_endpoint_directional_maps",
    "unpack_decision",
]
