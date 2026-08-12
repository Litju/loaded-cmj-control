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
    PREVIOUS_ACTION_SLICE,
    TANGENT_DIMENSION,
    boxminus,
    boxplus,
)
from loaded_cmj.simulation.transition import step_5ms


TRANSCRIPTION_SCHEMA_ID = "LCMJ-V1-SP-SDDT-TRANSCRIPTION-1.0.0"
STATE_DIMENSION = TANGENT_DIMENSION
CONTROL_DT_S = CONTROL_PERIOD_S
ACTION_DIMENSION = ACTION_DIM
FULL_LAYOUT = "FULL_LAYOUT"
WITNESS_LAYOUT = "WITNESS_LAYOUT"
FULL_ACTIVE_ACTION_INDICES = tuple(range(ACTION_DIMENSION))
WITNESS_FREE_ACTION_INDICES = (0, 3, 6, 9, 10, 11, 12)
WITNESS_FIXED_ACTION_INDICES = tuple(
    index for index in FULL_ACTIVE_ACTION_INDICES if index not in WITNESS_FREE_ACTION_INDICES
)
WITNESS_FIXED_ACTION_VALUES = tuple((index, 0.0) for index in WITNESS_FIXED_ACTION_INDICES)

# These are the native Euclidean coordinates owned by the DriveState/snapshot
# contract.  Configuration, cached SO(3), velocities, and qacc remain
# manifold-, cache-, or owner-defined and therefore receive no invented box.
A_PLUS_TANGENT_SLICE = slice(51, 66)
A_MINUS_TANGENT_SLICE = slice(66, 81)
PREVIOUS_ACCEPTED_ACTION_TANGENT_SLICE = PREVIOUS_ACTION_SLICE
_NATIVE_EUCLIDEAN_STATE_DOMAINS = (
    ("a_plus", A_PLUS_TANGENT_SLICE, 0.0, 1.0),
    ("a_minus", A_MINUS_TANGENT_SLICE, 0.0, 1.0),
    ("previous_accepted_action", PREVIOUS_ACCEPTED_ACTION_TANGENT_SLICE, -1.0, 1.0),
)
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
    """Frozen state/action/slack decision-vector slices.

    Knot zero is an exact fixed parameter supplied by the problem owner.  The
    first decision block is therefore the active raw action for interval zero,
    followed by the decision state at knot one.  ``active_action_indices`` and
    ``fixed_action_values`` are the sole owner of full-versus-witness action
    reconstruction.
    """

    horizon: int
    state_dimension: int = STATE_DIMENSION
    action_dimension: int = ACTION_DIMENSION
    slack_specs: tuple[SlackSpec, ...] = ()
    active_action_indices: tuple[int, ...] = FULL_ACTIVE_ACTION_INDICES
    fixed_action_values: tuple[tuple[int, float], ...] = ()
    layout_id: str = FULL_LAYOUT

    def __post_init__(self) -> None:
        if int(self.horizon) < 1:
            raise TranscriptionError("horizon must be a positive interval count")
        if (self.state_dimension, self.action_dimension) != (132, 15):
            raise TranscriptionError("state/action dimensions are not the frozen V1 dimensions")
        active = tuple(int(index) for index in self.active_action_indices)
        if not active or len(set(active)) != len(active) or tuple(sorted(active)) != active:
            raise TranscriptionError("active action indices must be a nonempty sorted tuple")
        if any(index < 0 or index >= self.action_dimension for index in active):
            raise TranscriptionError("active action index is outside the raw action domain")
        if self.layout_id not in {FULL_LAYOUT, WITNESS_LAYOUT}:
            raise TranscriptionError(f"unknown decision layout {self.layout_id!r}")
        expected_fixed = tuple(
            index for index in range(self.action_dimension) if index not in active
        )
        fixed = tuple((int(index), float(value)) for index, value in self.fixed_action_values)
        if not fixed and expected_fixed:
            fixed = tuple((index, 0.0) for index in expected_fixed)
        if tuple(index for index, _ in fixed) != expected_fixed:
            raise TranscriptionError("fixed action values do not cover the eliminated channels")
        if any(
            not np.isfinite(value) or abs(value) > 1.0
            for _, value in fixed
        ):
            raise TranscriptionError("fixed raw action values must be finite and lie in [-1, 1]")
        if self.layout_id == FULL_LAYOUT and active != FULL_ACTIVE_ACTION_INDICES:
            raise TranscriptionError("FULL_LAYOUT must expose all raw action channels")
        if self.layout_id == WITNESS_LAYOUT and active != WITNESS_FREE_ACTION_INDICES:
            raise TranscriptionError("WITNESS_LAYOUT must expose the authorized seven channels")
        object.__setattr__(self, "active_action_indices", active)
        object.__setattr__(self, "fixed_action_values", fixed)
        if any(not slack.nonnegative for slack in self.slack_specs):
            raise TranscriptionError("all approved elastic slacks must be nonnegative")
        expected = self.base_dimension
        for slack in self.slack_specs:
            if slack.decision_index != expected:
                raise TranscriptionError("slack decision indices are not contiguous")
            expected += 1

    @property
    def base_dimension(self) -> int:
        return self.horizon * (self.state_dimension + self.active_action_dimension)

    @property
    def active_action_dimension(self) -> int:
        return len(self.active_action_indices)

    @property
    def state_knot_indices(self) -> tuple[int, ...]:
        return tuple(range(1, self.horizon + 1))

    @property
    def total_dimension(self) -> int:
        return self.base_dimension + len(self.slack_specs)

    def state_slice(self, knot_index: int) -> slice:
        k = _index(knot_index, self.horizon + 1, "state knot")
        if k == 0:
            raise TranscriptionError("knot zero is the fixed initial-state parameter, not a decision")
        start = (k - 1) * (self.state_dimension + self.active_action_dimension)
        start += self.active_action_dimension
        return slice(start, start + self.state_dimension)

    def action_slice(self, interval_index: int) -> slice:
        k = _index(interval_index, self.horizon, "action interval")
        start = k * (self.state_dimension + self.active_action_dimension)
        return slice(start, start + self.active_action_dimension)

    def reconstruct_action(
        self, active_action: Sequence[float] | np.ndarray
    ) -> np.ndarray:
        """Insert active controls into one exact 15-D raw Plant action."""

        active = _validate_vector(
            active_action, self.active_action_dimension, "active raw action"
        )
        result = np.empty(self.action_dimension, dtype=np.float64)
        result[:] = np.nan
        for position, index in enumerate(self.active_action_indices):
            result[index] = active[position]
        for index, value in self.fixed_action_values:
            result[index] = value
        if not np.isfinite(result).all():
            raise TranscriptionError("action reconstruction left an unassigned channel")
        return result

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
        states = _validate_matrix_sequence(state_deltas, self.horizon, STATE_DIMENSION, "state deltas x[1:N]")
        commands = _validate_matrix_sequence(
            actions, self.horizon, self.active_action_dimension, "active raw actions"
        )
        slack_values = _validate_slacks(slacks, len(self.slack_specs))
        result = np.empty(self.total_dimension, dtype=np.float64)
        for k in range(self.horizon):
            result[self.action_slice(k)] = commands[k]
            result[self.state_slice(k + 1)] = states[k]
        if slack_values.size:
            result[self.base_dimension:] = slack_values
        return result

    def unpack_decision(self, z: Sequence[float] | np.ndarray) -> "UnpackedDecision":
        value = _validate_vector(z, self.total_dimension, "decision vector")
        states = tuple(value[self.state_slice(k)].copy() for k in self.state_knot_indices)
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
        action_layout: str = FULL_LAYOUT,
    ) -> None:
        if not fixed_initial_state:
            raise TranscriptionError("ML242 requires the authoritative initial state as a fixed parameter")
        if action_layout not in {FULL_LAYOUT, WITNESS_LAYOUT}:
            raise TranscriptionError(f"unknown action layout {action_layout!r}")
        self.plant = plant
        self.control_dt = float(control_dt)
        self.catalog = catalog
        self.fixed_initial_state = True
        self.include_elastic_slacks = bool(include_elastic_slacks)
        self.derivative_provider = derivative_provider
        self.action_layout = action_layout
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
        active_action_indices = (
            FULL_ACTIVE_ACTION_INDICES
            if action_layout == FULL_LAYOUT
            else WITNESS_FREE_ACTION_INDICES
        )
        fixed_action_values = (
            () if action_layout == FULL_LAYOUT else WITNESS_FIXED_ACTION_VALUES
        )
        self.layout = DecisionLayout(
            self.horizon,
            slack_specs=(),
            active_action_indices=active_action_indices,
            fixed_action_values=fixed_action_values,
            layout_id=action_layout,
        )
        self._elastic_bindings = self._build_elastic_bindings()
        self.layout = DecisionLayout(
            self.horizon,
            slack_specs=self._elastic_bindings,
            active_action_indices=active_action_indices,
            fixed_action_values=fixed_action_values,
            layout_id=action_layout,
        )
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
    def initial_state(self) -> MacroSnapshot:
        """Return the exact authoritative E3 snapshot parameter."""

        return self.reference_snapshots[0]

    @property
    def active_action_indices(self) -> tuple[int, ...]:
        return self.layout.active_action_indices

    @property
    def fixed_action_indices(self) -> tuple[int, ...]:
        return tuple(index for index, _ in self.layout.fixed_action_values)

    def reconstruct_action(
        self, active_action: Sequence[float] | np.ndarray
    ) -> np.ndarray:
        return self.layout.reconstruct_action(active_action)

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
            if np.any(snapshot.a_plus < 0.0) or np.any(snapshot.a_plus > 1.0):
                raise TranscriptionError(f"reference snapshot {index} has a_plus outside [0, 1]")
            if np.any(snapshot.a_minus < 0.0) or np.any(snapshot.a_minus > 1.0):
                raise TranscriptionError(f"reference snapshot {index} has a_minus outside [0, 1]")
            if np.any(np.abs(snapshot.previous_accepted_action) > 1.0):
                raise TranscriptionError(
                    f"reference snapshot {index} has previous_accepted_action outside [-1, 1]"
                )

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
        next_index = self.layout.base_dimension
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
        if spec.classification == "HARD_TERMINAL":
            return (("state", index),)
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
        if knot == 0:
            raise TranscriptionError("knot zero is supplied as the exact initial-state parameter")
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
        state = (
            self.initial_state
            if k == 0
            else self._reconstruct_state(k, decision.states[k - 1])
        )
        next_state = self._reconstruct_state(k + 1, decision.states[k])
        raw_action = self.layout.reconstruct_action(decision.actions[k])
        predicted = self._propagate(state, raw_action)
        defect = boxminus(next_state, predicted, model=self.plant.model)
        return IntervalEvaluation(
            state=state,
            next_state=next_state,
            predicted_state=predicted,
            raw_action=raw_action,
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

    def _derivatives(
        self,
        state: MacroSnapshot,
        action: np.ndarray,
        *,
        state_columns: Sequence[int] | None = None,
    ) -> WrappedLinearization:
        provider = linearize_step_5ms if self.derivative_provider is None else self.derivative_provider
        try:
            result = provider(
                plant=self.plant,
                base_snapshot=state,
                raw_action=action,
                state_steps=ML241_STATE_STEPS,
                action_steps=ML241_ACTION_STEP,
                state_columns=state_columns,
                action_columns=self.layout.active_action_indices,
            )
        except Exception as exc:
            if isinstance(exc, DerivativeContractError):
                raise
            raise DerivativeContractError("ML-241 ordinary local derivative assembly was rejected") from exc
        requested_state = (
            tuple(range(STATE_DIMENSION)) if state_columns is None else tuple(int(index) for index in state_columns)
        )
        self._validate_derivative_metadata(
            result,
            state,
            action,
            requested_state,
            self.layout.active_action_indices,
        )
        return result

    @staticmethod
    def _validate_derivative_metadata(
        result: WrappedLinearization,
        state: MacroSnapshot,
        action: np.ndarray,
        requested_state_columns: Sequence[int],
        active_action_indices: Sequence[int],
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
        requested_state = tuple(int(index) for index in requested_state_columns)
        qacc_requested = bool(set(requested_state).intersection(QACC_ZERO_COLUMNS))
        if qacc_requested:
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
        active = tuple(int(index) for index in active_action_indices)
        if len(branches) != ACTION_DIMENSION or any(branches[index] == "NEAR_KINK" for index in active):
            raise DerivativeContractError("ML-241 action branch is not an ordinary smooth local branch")
        if result.base_snapshot_digest != snapshot_digest(state) or result.base_raw_action.shape != (ACTION_DIMENSION,):
            raise DerivativeContractError("ML-241 base-point metadata is incomplete")
        if not np.array_equal(np.asarray(result.base_raw_action), action):
            raise DerivativeContractError("ML-241 derivative base action does not match the decision")
        state_validity = np.asarray(result.state_validity, dtype=bool)
        if requested_state and not state_validity[np.asarray(requested_state, dtype=np.int64)].all():
            invalid = [index for index in requested_state if not bool(state_validity[index])]
            raise DerivativeContractError(f"ordinary local A columns are nonsmooth/invalid: {invalid}")
        action_validity = np.asarray(result.action_validity, dtype=bool)
        if not action_validity[np.asarray(active, dtype=np.int64)].all():
            invalid = [
                index for index in active if not bool(action_validity[index])
            ]
            raise DerivativeContractError(f"ordinary local B columns are nonsmooth/invalid: {invalid}")
        state_finite = (
            np.isfinite(result.A[:, np.asarray(requested_state, dtype=np.int64)]).all()
            if requested_state
            else True
        )
        if not state_finite or not np.isfinite(
            np.asarray(result.B)[:, np.asarray(active, dtype=np.int64)]
        ).all():
            raise DerivativeContractError("ordinary local derivative contains invalid active columns")
        if qacc_requested and not np.array_equal(
            result.A[:, QACC_ZERO_COLUMNS], np.zeros((STATE_DIMENSION, len(QACC_ZERO_COLUMNS)))
        ):
            raise DerivativeContractError("qualified-zero qacc columns were altered")

    def jacobian_values(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        value = _validate_vector(z, self.variable_count, "decision vector")
        entries: list[float] = []
        active = np.asarray(self.layout.active_action_indices, dtype=np.int64)
        for interval in range(self.horizon):
            evaluation = self.interval_evaluation(value, interval)
            requested_state = () if interval == 0 else tuple(range(STATE_DIMENSION))
            linearization = self._derivatives(
                evaluation.state,
                evaluation.raw_action,
                state_columns=requested_state,
            )
            endpoint = boxminus_endpoint_jacobians(
                next_snapshot=evaluation.next_state,
                predicted_snapshot=evaluation.predicted_state,
                model=self.plant.model,
            )
            J_state = endpoint.G_pred @ linearization.A if interval > 0 else None
            J_action = endpoint.G_pred @ linearization.B[:, active]
            J_next = endpoint.G_next
            for component in range(STATE_DIMENSION):
                if interval > 0:
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
        active_width = self.layout.active_action_dimension
        for interval in range(self.horizon):
            action_start = self.layout.action_slice(interval).start
            next_start = self.layout.state_slice(interval + 1).start
            for component in range(STATE_DIMENSION):
                row = interval * STATE_DIMENSION + component
                if interval > 0:
                    rows.extend([row] * STATE_DIMENSION)
                    state_start = self.layout.state_slice(interval).start
                    cols.extend(range(state_start, state_start + STATE_DIMENSION))
                rows.extend([row] * (active_width + STATE_DIMENSION))
                cols.extend(range(action_start, action_start + active_width))
                cols.extend(range(next_start, next_start + STATE_DIMENSION))
        for row in self.constraint_rows[self.dynamics_row_count :]:
            for kind, index in row.support:
                if kind == "state":
                    if index == 0:
                        continue
                    slc = self.layout.state_slice(index)
                elif kind == "action":
                    slc = self.layout.action_slice(index)
                else:
                    raise TranscriptionError(f"unknown Jacobian support kind {kind!r}")
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
        for knot in self.layout.state_knot_indices:
            state_slice = self.layout.state_slice(knot)
            reference = self.reference_snapshots[knot]
            for attribute, tangent_slice, physical_lower, _ in _NATIVE_EUCLIDEAN_STATE_DOMAINS:
                values = np.asarray(getattr(reference, attribute), dtype=np.float64)
                start = state_slice.start + tangent_slice.start
                stop = state_slice.start + tangent_slice.stop
                lower[start:stop] = physical_lower - values
        for slack in self.layout.slack_specs:
            lower[slack.decision_index] = 0.0
        return lower

    def variable_upper_bounds(self) -> np.ndarray:
        upper = np.full(self.variable_count, np.inf, dtype=np.float64)
        for interval in range(self.horizon):
            upper[self.layout.action_slice(interval)] = 1.0
        for knot in self.layout.state_knot_indices:
            state_slice = self.layout.state_slice(knot)
            reference = self.reference_snapshots[knot]
            for attribute, tangent_slice, _, physical_upper in _NATIVE_EUCLIDEAN_STATE_DOMAINS:
                values = np.asarray(getattr(reference, attribute), dtype=np.float64)
                start = state_slice.start + tangent_slice.start
                stop = state_slice.start + tangent_slice.stop
                upper[start:stop] = physical_upper - values
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
            "active_action_dimension": self.layout.active_action_dimension,
            "active_action_indices": list(self.layout.active_action_indices),
            "fixed_action_values": [
                [index, value] for index, value in self.layout.fixed_action_values
            ],
            "layout_id": self.layout.layout_id,
            "horizon": self.horizon,
            "control_dt_s": self.control_dt,
            "ordering": "u_active[0], delta_x[1], ..., u_active[N-1], delta_x[N], slack_suffix",
            "base_dimension": self.layout.base_dimension,
            "total_dimension": self.layout.total_dimension,
            "initial_state_parameter": {
                "knot_index": 0,
                "role": "FIXED_PARAMETER",
                "is_nlp_decision": False,
            },
            "fixed_initial_state": True,
            "state_knot_indices": list(self.layout.state_knot_indices),
            "state_slices": [self.layout.state_slice(k).indices(self.layout.total_dimension) for k in self.layout.state_knot_indices],
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
    "A_PLUS_TANGENT_SLICE",
    "A_MINUS_TANGENT_SLICE",
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
    "FULL_ACTIVE_ACTION_INDICES",
    "FULL_LAYOUT",
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
    "PREVIOUS_ACCEPTED_ACTION_TANGENT_SLICE",
    "WITNESS_FIXED_ACTION_INDICES",
    "WITNESS_FIXED_ACTION_VALUES",
    "WITNESS_FREE_ACTION_INDICES",
    "WITNESS_LAYOUT",
    "boxminus_endpoint_jacobians",
    "pack_decision",
    "qualify_endpoint_directional_maps",
    "unpack_decision",
]
