"""Compose the target-side E3->E4 physical constraint rows.

The composition seam owns only task-row selection and source-owner routing.
State reconstruction remains the public ML-242 problem API, physical values
remain Plant-owned, and state-only sensitivities remain ML-241-owned.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.oracle.constraints import (
    CATALOG,
    adapt_com_velocity,
    adapt_contact_wrench_summary,
    adapt_support_margin,
)
from loaded_cmj.oracle.derivatives import (
    DIRECT_STATE_OWNER,
    DerivativeDomainError,
    OwnerOutputSensitivity,
    differentiate_state_owner_output,
    snapshot_digest,
)
from loaded_cmj.oracle.transcription import (
    APPROVED_ELASTIC_CONSTRAINT_IDS,
    ML241_STATE_STEPS,
    STATE_DIMENSION,
    DirectMultipleShootingProblem,
)
from loaded_cmj.simulation import drive
from loaded_cmj.simulation.snapshot import MacroSnapshot

E3E4_PHYSICAL_ROW_COUNT = 43
E3E4_OWNER_IDS = (
    "E3_E4_HORIZONTAL",
    "E3_E4_REVERSAL",
    "SUPPORT_MARGIN",
)
_SUPPORT_OWNER_IDS = frozenset({"SUPPORT_MARGIN", "E3_E4_HORIZONTAL"})
_DERIVATIVE_OWNER = "ML241.differentiate_state_owner_output"


class E3E4CompositionError(ValueError):
    """Raised when the live ML-240/ML-242 row contract is not target-side E3->E4."""


@dataclass(frozen=True, slots=True)
class PhysicalConstraintOwnerReceipt:
    """Live row identity and source-owner disposition for one physical row."""

    row_index: int
    physical_index: int
    constraint_id: str
    semantic_name: str
    units: str
    sense: str
    hard_or_elastic: str
    knot_index: int
    value_owner: str
    derivative_owner: str
    decision_state_columns: tuple[int, ...]
    decision_action_columns: tuple[int, ...]
    support: tuple[tuple[str, int], ...]
    differentiability: str
    classification: str
    enforcement: str
    slack_decision_index: int | None


@dataclass(frozen=True, slots=True)
class PhysicalConstraintDerivative:
    """One validated ML-241 direct-state owner result on a row support."""

    row: PhysicalConstraintOwnerReceipt
    owner_sensitivity: OwnerOutputSensitivity | None
    state_columns: tuple[int, ...]
    state_values: np.ndarray

    def __post_init__(self) -> None:
        values = np.asarray(self.state_values, dtype=np.float64).copy()
        values.setflags(write=False)
        object.__setattr__(self, "state_values", values)


DerivativeOwner = Callable[..., OwnerOutputSensitivity]


def _restore_forward(plant: Any, snapshot: MacroSnapshot) -> Any:
    """Materialize one current snapshot for the existing Plant adapters."""

    data = plant.make_data()
    drive_state = drive.DriveState(
        a_plus=snapshot.a_plus,
        a_minus=snapshot.a_minus,
        tau_prev=snapshot.tau_prev,
        previous_command=snapshot.previous_command,
        override_flags={key: value.copy() for key, value in snapshot.override_flags},
        reversal_phase=snapshot.reversal_phase,
    )
    snapshot.restore(plant=plant, data=data, drive_state=drive_state)
    mujoco.mj_forward(plant.model, data)
    return data


def _owner_value_owner(constraint_id: str) -> str:
    if constraint_id in _SUPPORT_OWNER_IDS:
        return "Plant.contact_wrench_summary -> Plant.support_margin"
    if constraint_id == "E3_E4_REVERSAL":
        return "Plant.center_of_mass_velocity"
    raise E3E4CompositionError(f"unsupported E3->E4 owner: {constraint_id}")


def _hard_or_elastic(constraint_id: str) -> str:
    if constraint_id in APPROVED_ELASTIC_CONSTRAINT_IDS:
        return "ELASTIC"
    if constraint_id == "E3_E4_REVERSAL":
        return "HARD_NONELASTIC"
    raise E3E4CompositionError(f"missing hard/elastic disposition: {constraint_id}")


def _decision_state_columns(knot_index: int) -> tuple[int, ...]:
    return () if knot_index == 0 else tuple(range(STATE_DIMENSION))


class E3E4PhysicalConstraintComposer:
    """Compose exactly the 43 target-side current-state physical rows."""

    def __init__(
        self,
        problem: DirectMultipleShootingProblem,
        *,
        derivative_owner: DerivativeOwner | None = None,
    ) -> None:
        self.problem = problem
        self.plant = problem.plant
        self.catalog = getattr(problem, "catalog", CATALOG)
        physical_rows = tuple(problem.constraint_rows[problem.dynamics_row_count :])
        self._rows = self._validate_rows(physical_rows)
        self._receipts = tuple(
            self._receipt(index, row) for index, row in enumerate(self._rows)
        )
        self._derivative_owner = (
            differentiate_state_owner_output
            if derivative_owner is None
            else derivative_owner
        )

    def _validate_rows(self, rows: tuple[Any, ...]) -> tuple[Any, ...]:
        if len(rows) != E3E4_PHYSICAL_ROW_COUNT:
            raise E3E4CompositionError(
                f"expected {E3E4_PHYSICAL_ROW_COUNT} physical rows, received {len(rows)}"
            )
        expected = (
            ("E3_E4_HORIZONTAL", 40),
            ("E3_E4_REVERSAL", 40),
            *(("SUPPORT_MARGIN", knot) for knot in range(41)),
        )
        for physical_index, (row, (constraint_id, knot)) in enumerate(
            zip(rows, expected, strict=True)
        ):
            spec = self.catalog.get(constraint_id)
            if row.row_index != 5280 + physical_index:
                raise E3E4CompositionError(
                    f"physical row {physical_index} has noncanonical row index"
                )
            if row.constraint_id != constraint_id or row.knot_index != knot:
                raise E3E4CompositionError(
                    f"physical row {physical_index} has noncanonical identity"
                )
            if row.support != (("state", knot),):
                raise E3E4CompositionError(
                    f"physical row {physical_index} has noncanonical local support"
                )
            if row.component_index != 0 or row.interval_index is not None:
                raise E3E4CompositionError(
                    f"physical row {physical_index} has noncanonical component support"
                )
            if row.differentiability != spec.differentiability:
                raise E3E4CompositionError(
                    f"physical row {physical_index} disagrees with catalog differentiability"
                )
            if row.sense != spec.sense:
                raise E3E4CompositionError(
                    f"physical row {physical_index} disagrees with catalog sense"
                )
            if constraint_id == "E3_E4_REVERSAL" and row.slack_decision_index is not None:
                raise E3E4CompositionError("E3_E4_REVERSAL cannot have a decision slack")
        return rows

    def _receipt(self, index: int, row: Any) -> PhysicalConstraintOwnerReceipt:
        spec = self.catalog.get(row.constraint_id)
        return PhysicalConstraintOwnerReceipt(
            row_index=int(row.row_index),
            physical_index=index,
            constraint_id=row.constraint_id,
            semantic_name=spec.name,
            units=spec.units,
            sense=spec.sense,
            hard_or_elastic=_hard_or_elastic(row.constraint_id),
            knot_index=int(row.knot_index),
            value_owner=_owner_value_owner(row.constraint_id),
            derivative_owner=_DERIVATIVE_OWNER,
            decision_state_columns=_decision_state_columns(int(row.knot_index)),
            decision_action_columns=(),
            support=tuple((str(kind), int(index)) for kind, index in row.support),
            differentiability=row.differentiability,
            classification=spec.classification,
            enforcement=spec.enforcement,
            slack_decision_index=row.slack_decision_index,
        )

    def physical_constraint_owner_receipt(
        self,
    ) -> tuple[PhysicalConstraintOwnerReceipt, ...]:
        """Return the immutable physical-row receipt in ML-242 order."""

        return self._receipts

    def _knot_states(self, z: Sequence[float] | np.ndarray) -> tuple[MacroSnapshot, ...]:
        decision = self.problem.unpack_decision(z)
        states = [self.problem.initial_state]
        states.extend(
            self.problem.reconstruct_knot_state(knot, decision.states[knot - 1])
            for knot in range(1, self.problem.horizon + 1)
        )
        return tuple(states)

    def _value(self, constraint_id: str, data: Any) -> float:
        if constraint_id in _SUPPORT_OWNER_IDS:
            summary = adapt_contact_wrench_summary(self.plant, data)
            active = tuple(bool(value) for value in summary["active_by_foot"])
            com = self.plant.center_of_mass(data)
            return float(adapt_support_margin(self.plant, data, com, active))
        if constraint_id == "E3_E4_REVERSAL":
            return float(adapt_com_velocity(self.plant, data)[2])
        raise E3E4CompositionError(f"unsupported E3->E4 owner: {constraint_id}")

    def physical_constraint_values(
        self,
        z: Sequence[float] | np.ndarray,
    ) -> np.ndarray:
        """Return 43 finite current-state physical values in row order."""

        states = self._knot_states(z)
        data_by_knot = {
            knot: _restore_forward(self.plant, states[knot])
            for knot in sorted({receipt.knot_index for receipt in self._receipts})
        }
        values = np.asarray(
            [
                self._value(
                    receipt.constraint_id,
                    data_by_knot[receipt.knot_index],
                )
                for receipt in self._receipts
            ],
            dtype=np.float64,
        )
        if values.shape != (E3E4_PHYSICAL_ROW_COUNT,) or not np.isfinite(values).all():
            raise E3E4CompositionError("physical E3->E4 owner values are not finite")
        return values

    @staticmethod
    def _validate_sensitivity(
        sensitivity: OwnerOutputSensitivity,
        *,
        receipt: PhysicalConstraintOwnerReceipt,
        snapshot: MacroSnapshot,
        requested_columns: tuple[int, ...],
        state_steps: Any,
    ) -> None:
        if not isinstance(sensitivity, OwnerOutputSensitivity):
            raise DerivativeDomainError("ML241 returned a result without owner provenance")
        if sensitivity.owner_id != receipt.constraint_id:
            raise DerivativeDomainError("ML241 owner provenance does not match the row")
        if sensitivity.evaluation_mode != DIRECT_STATE_OWNER:
            raise DerivativeDomainError("physical row received transition-wrapped provenance")
        if sensitivity.base_snapshot_digest != snapshot_digest(snapshot):
            raise DerivativeDomainError("ML241 base snapshot provenance does not match the knot")
        if sensitivity.base_next_snapshot_digest != "":
            raise DerivativeDomainError("physical row received a next-snapshot provenance")
        if sensitivity.requested_action_columns != () or sensitivity.action_columns != ():
            raise DerivativeDomainError("physical row received action derivative provenance")
        if sensitivity.action_step_metadata != () or sensitivity.action_validity.any():
            raise DerivativeDomainError("physical row received action derivative metadata")
        if sensitivity.requested_state_columns != requested_columns:
            raise DerivativeDomainError("ML241 requested state support does not match the row")
        if not set(requested_columns).issubset(receipt.decision_state_columns):
            raise DerivativeDomainError("requested state support exceeds the row support")
        if dict(sensitivity.state_step_metadata) != dict(state_steps):
            raise DerivativeDomainError("ML241 state-step provenance does not match the request")
        if sensitivity.base_value.shape != (1,) or not np.isfinite(sensitivity.base_value).all():
            raise DerivativeDomainError("ML241 owner base value is not finite")
        jacobian = np.asarray(sensitivity.state_jacobian, dtype=np.float64)
        if jacobian.shape != (1, STATE_DIMENSION):
            raise DerivativeDomainError("ML241 owner state Jacobian has the wrong shape")
        if requested_columns and not np.isfinite(jacobian[:, requested_columns]).all():
            raise DerivativeDomainError("ML241 owner state Jacobian is nonfinite")
        if sensitivity.owner_output_digest == "":
            raise DerivativeDomainError("ML241 owner output digest is missing")
        for report in sensitivity.state_columns:
            if report.index in requested_columns and (
                not report.valid or not report.native_domain_legal
            ):
                raise DerivativeDomainError(
                    f"ML241 rejected {receipt.constraint_id} state column {report.index}"
                )
        if receipt.constraint_id in _SUPPORT_OWNER_IDS:
            certificate = sensitivity.base_branch_certificate
            if certificate is None or not certificate.finite:
                raise DerivativeDomainError("ML241 support branch certificate is invalid")
            if certificate.exact_tie or certificate.norm_zero_kink:
                raise DerivativeDomainError("ML241 support branch certificate contains a kink")
        elif sensitivity.base_branch_certificate is not None:
            raise DerivativeDomainError("non-support owner carries support branch provenance")

    def physical_constraint_jacobian_values(
        self,
        z: Sequence[float] | np.ndarray,
        *,
        physical_row_indices: Sequence[int] | None = None,
        state_columns: Sequence[int] | None = None,
        state_steps: Any = None,
    ) -> tuple[PhysicalConstraintDerivative, ...]:
        """Return validated ML-241 direct-state owner results only."""

        states = self._knot_states(z)
        if physical_row_indices is None:
            selected_indexes = tuple(range(E3E4_PHYSICAL_ROW_COUNT))
        else:
            selected_indexes = tuple(int(index) for index in physical_row_indices)
        if len(set(selected_indexes)) != len(selected_indexes) or any(
            index < 0 or index >= E3E4_PHYSICAL_ROW_COUNT
            for index in selected_indexes
        ):
            raise E3E4CompositionError("physical row selection is outside [0, 43)")
        requested_columns = (
            tuple(range(STATE_DIMENSION))
            if state_columns is None
            else tuple(int(index) for index in state_columns)
        )
        if len(set(requested_columns)) != len(requested_columns) or any(
            index < 0 or index >= STATE_DIMENSION for index in requested_columns
        ):
            raise E3E4CompositionError("state derivative selection is outside [0, 132)")
        canonical_state_steps = ML241_STATE_STEPS if state_steps is None else state_steps
        result: list[PhysicalConstraintDerivative] = []
        cache: dict[tuple[str, int, tuple[int, ...]], OwnerOutputSensitivity] = {}
        for physical_index in selected_indexes:
            receipt = self._receipts[physical_index]
            if not receipt.decision_state_columns:
                result.append(
                    PhysicalConstraintDerivative(
                        row=receipt,
                        owner_sensitivity=None,
                        state_columns=(),
                        state_values=np.zeros((1, 0), dtype=np.float64),
                    )
                )
                continue
            if not set(requested_columns).issubset(receipt.decision_state_columns):
                raise E3E4CompositionError(
                    f"requested state columns exceed row support for {receipt.constraint_id}"
                )
            key = (receipt.constraint_id, receipt.knot_index, requested_columns)
            sensitivity = cache.get(key)
            if sensitivity is None:
                sensitivity = self._derivative_owner(
                    plant=self.plant,
                    base_snapshot=states[receipt.knot_index],
                    owner_id=receipt.constraint_id,
                    state_steps=canonical_state_steps,
                    state_columns=requested_columns,
                    allow_nonsmooth=False,
                )
                cache[key] = sensitivity
            self._validate_sensitivity(
                sensitivity,
                receipt=receipt,
                snapshot=states[receipt.knot_index],
                requested_columns=requested_columns,
                state_steps=canonical_state_steps,
            )
            values = np.asarray(sensitivity.state_jacobian, dtype=np.float64)[
                :, requested_columns
            ]
            result.append(
                PhysicalConstraintDerivative(
                    row=receipt,
                    owner_sensitivity=sensitivity,
                    state_columns=requested_columns,
                    state_values=values,
                )
            )
        return tuple(result)


__all__ = [
    "E3E4CompositionError",
    "E3E4PhysicalConstraintComposer",
    "E3E4_PHYSICAL_ROW_COUNT",
    "E3E4_OWNER_IDS",
    "PhysicalConstraintDerivative",
    "PhysicalConstraintOwnerReceipt",
]
