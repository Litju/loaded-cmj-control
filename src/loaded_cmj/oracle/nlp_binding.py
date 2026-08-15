"""Production E3-to-E4 NLP binding without solver side effects.

This module owns only the boundary assembly.  The direct multiple-shooting
problem remains the owner of layout, bounds, defects, rho, slacks, and sparse
structure; the physical composer remains the owner of Plant and ML241 source
callbacks.  The WITNESS initializer is a deterministic local branch-
interiorization rule; it performs no search, rollout, or solver call.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from types import MappingProxyType
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.oracle.composition import (
    E3E4PhysicalConstraintComposer,
    PhysicalConstraintDerivative,
    PhysicalConstraintOwnerReceipt,
)
from loaded_cmj.oracle.transcription import (
    FULL_ACTIVE_ACTION_INDICES,
    FULL_LAYOUT,
    ML241_STATE_STEPS,
    STATE_DIMENSION,
    WITNESS_FIXED_ACTION_INDICES,
    WITNESS_FREE_ACTION_INDICES,
    WITNESS_SIGN_SECTOR_BOUNDS,
    WITNESS_LAYOUT,
    DirectMultipleShootingProblem,
)
from loaded_cmj.simulation.drive import DriveState
from loaded_cmj.simulation.tangent import boxminus, boxplus


FULL_VARIABLE_COUNT = 5923
WITNESS_VARIABLE_COUNT = 5603
CONSTRAINT_COUNT = 5365
JACOBIAN_NNZ = {
    FULL_LAYOUT: 1_461_366,
    WITNESS_LAYOUT: 1_419_126,
}
ELASTIC_SLACK_COUNT = 42
EPIGRAPH_ROW_COUNT = 42

# The single frozen WITNESS synthesis rule.  These are tangent-state indices,
# not new decision variables or Plant parameters.
WITNESS_RIGHT_ANKLE_EVERSION_STATE_INDEX = 18
WITNESS_A_MINUS_9_STATE_INDEX = 75
WITNESS_A_MINUS_10_STATE_INDEX = 76
WITNESS_RIGHT_ANKLE_EVERSION_BIAS = 4.0 * ML241_STATE_STEPS["configuration_rotation_joint"]
WITNESS_CONTACT_STENCIL_MULTIPLIER = 2.0
WITNESS_NATIVE_STENCIL_MULTIPLIER = 2.0
WITNESS_MAX_INITIALIZER_DISPLACEMENT = 2.0e-4

# This is the frozen E3 continuation profile.  It is intentionally stored as
# a tuple so initialization cannot mutate the authority between constructions.
E3_E4_INITIAL_ACTION_PROFILE = (
    0.9,
    -6.11633784740375e-16,
    -6.95350155350338e-16,
    0.16806307252533137,
    -8.4925855384516195e-16,
    2.2612741252453442e-15,
    0.16806307252528851,
    7.997180028038798e-16,
    -2.4452693620033278e-15,
    0.11091999487016607,
    0.11091999487016607,
    -0.016134519367529535,
    -0.01613451936758624,
    1.0529934603434907e-15,
    -1.174872765636595e-15,
)
E3_E4_INITIAL_ACTION_PROFILE = tuple(float(value) for value in E3_E4_INITIAL_ACTION_PROFILE)


class NLPBindingError(ValueError):
    """Raised when the live G5 problem cannot satisfy the production binding."""


class E3E4NLPBinding:
    """Bind one live FULL or WITNESS G5 problem to canonical source owners."""

    def __init__(
        self,
        problem: DirectMultipleShootingProblem,
    ) -> None:
        if not isinstance(problem, DirectMultipleShootingProblem):
            raise NLPBindingError("production binding requires DirectMultipleShootingProblem")
        self.problem = problem
        self.composer = E3E4PhysicalConstraintComposer(problem)
        self._physical_index_by_row = MappingProxyType(
            {
                receipt.row_index: receipt.physical_index
                for receipt in self.composer.physical_constraint_owner_receipt()
            }
        )
        self._validate_g5_identity()

    def _validate_g5_identity(self) -> None:
        if self.problem.action_layout not in {FULL_LAYOUT, WITNESS_LAYOUT}:
            raise NLPBindingError("production binding received an unknown action layout")
        expected_active = (
            FULL_ACTIVE_ACTION_INDICES
            if self.problem.action_layout == FULL_LAYOUT
            else WITNESS_FREE_ACTION_INDICES
        )
        if tuple(self.problem.active_action_indices) != expected_active:
            raise NLPBindingError("production binding received a noncanonical active action set")
        if self.problem.action_layout == WITNESS_LAYOUT:
            if tuple(self.problem.fixed_action_indices) != WITNESS_FIXED_ACTION_INDICES:
                raise NLPBindingError("production binding received noncanonical fixed witness controls")
            if tuple(self.problem.layout.active_action_bounds) != tuple(
                WITNESS_SIGN_SECTOR_BOUNDS[index] for index in WITNESS_FREE_ACTION_INDICES
            ):
                raise NLPBindingError("production binding received noncanonical witness sign sectors")
        expected_n = {
            FULL_LAYOUT: FULL_VARIABLE_COUNT,
            WITNESS_LAYOUT: WITNESS_VARIABLE_COUNT,
        }[self.problem.action_layout]
        if self.problem.variable_count != expected_n:
            raise NLPBindingError(
                f"{self.problem.action_layout} variable count {self.problem.variable_count} != {expected_n}"
            )
        if self.problem.constraint_count != CONSTRAINT_COUNT:
            raise NLPBindingError(
                f"constraint count {self.problem.constraint_count} != {CONSTRAINT_COUNT}"
            )
        if self.problem.horizon != 40:
            raise NLPBindingError("production E3->E4 binding requires horizon N=40")
        if self.problem.elastic_slack_count != ELASTIC_SLACK_COUNT:
            raise NLPBindingError("production binding requires the frozen 42 elastic slacks")
        if self.problem.phase_i_epigraph_row_count != EPIGRAPH_ROW_COUNT:
            raise NLPBindingError("production binding requires the frozen 42 epigraph rows")
        if self.problem.rho_variable_count != 1:
            raise NLPBindingError("production binding requires exactly one rho variable")
        if len(self.composer.physical_constraint_owner_receipt()) != 43:
            raise NLPBindingError("production binding requires exactly 43 physical rows")
        rows, cols = self.problem.jacobian_structure()
        if len(rows) != JACOBIAN_NNZ[self.problem.action_layout] or len(cols) != len(rows):
            raise NLPBindingError("production binding received an unexpected sparse structure")

    @property
    def action_layout(self) -> str:
        return self.problem.action_layout

    @property
    def variable_count(self) -> int:
        return self.problem.variable_count

    @property
    def constraint_count(self) -> int:
        return self.problem.constraint_count

    @property
    def elastic_slack_count(self) -> int:
        return self.problem.elastic_slack_count

    @property
    def elastic_slack_schema(self) -> tuple[dict[str, Any], ...]:
        return self.problem.elastic_slack_schema

    @property
    def phase_i_epigraph_row_count(self) -> int:
        return self.problem.phase_i_epigraph_row_count

    @property
    def rho_variable_count(self) -> int:
        return self.problem.rho_variable_count

    def variable_lower_bounds(self) -> np.ndarray:
        return self.problem.variable_lower_bounds()

    def variable_upper_bounds(self) -> np.ndarray:
        return self.problem.variable_upper_bounds()

    def constraint_lower_bounds(self) -> np.ndarray:
        return self.problem.constraint_lower_bounds()

    def constraint_upper_bounds(self) -> np.ndarray:
        return self.problem.constraint_upper_bounds()

    def jacobian_rows(self) -> np.ndarray:
        return self.problem.jacobian_rows()

    def jacobian_cols(self) -> np.ndarray:
        return self.problem.jacobian_cols()

    def jacobian_structure(self) -> tuple[np.ndarray, np.ndarray]:
        return self.problem.jacobian_structure()

    def jacobian_structure_hash(self) -> str:
        return self.problem.jacobian_structure_hash()

    def initial_vector(self, mode: str | None = None) -> np.ndarray:
        """Assemble the frozen deterministic user x0 for this layout.

        FULL retains the pre-G6 zero-tangent initializer.  WITNESS uses one
        deterministic state-domain interiorization rule at knots 1..40; knot
        zero remains the exact frozen E3 parameter.
        """

        requested_mode = self.action_layout if mode is None else str(mode)
        if requested_mode != self.action_layout:
            raise NLPBindingError(
                f"requested initializer mode {requested_mode!r} does not match {self.action_layout!r}"
            )
        raw_profile = np.asarray(E3_E4_INITIAL_ACTION_PROFILE, dtype=np.float64)
        active = tuple(self.problem.active_action_indices)
        active_profile = raw_profile[np.asarray(active, dtype=np.int64)]
        if self.action_layout == WITNESS_LAYOUT:
            states = tuple(
                self._witness_initializer_state_delta(knot)
                for knot in self.problem.layout.state_knot_indices
            )
        else:
            states = tuple(
                np.zeros(STATE_DIMENSION, dtype=np.float64)
                for _ in range(self.problem.horizon)
            )
        actions = tuple(active_profile.copy() for _ in range(self.problem.horizon))
        slacks = tuple(0.0 for _ in range(self.problem.elastic_slack_count))
        result = self.problem.pack_decision(states, actions, slacks, rho=0.0)
        self._validate_initial_vector(result)
        return result

    def _witness_initializer_state_delta(self, knot: int) -> np.ndarray:
        """Synthesize one bounded derivative-domain-valid WITNESS state.

        The rule is intentionally fixed before evaluating the completed
        Jacobian:

        1. break the mirrored-pad hull degeneracy with coordinate 18 by
           ``4 * h_configuration_rotation_joint``;
        2. inspect the resulting designated pad-floor contacts and move only
           root translation-z by ``-(max_contact_distance + 2*h_z)``;
        3. place only ``a_minus[9:11]`` at ``2*h_drivestate_activation``.

        No candidate magnitudes are searched and no callback result is used to
        alter the rule.  A missing designated contact or a displacement above
        the frozen bound is a fail-closed initializer error.
        """

        if knot < 1 or knot > self.problem.horizon:
            raise NLPBindingError(f"WITNESS initializer knot {knot} is outside 1..{self.problem.horizon}")
        reference = self.problem.reference_snapshots[knot]
        model = self.problem.plant.model
        delta = np.zeros(STATE_DIMENSION, dtype=np.float64)
        delta[WITNESS_RIGHT_ANKLE_EVERSION_STATE_INDEX] = WITNESS_RIGHT_ANKLE_EVERSION_BIAS

        biased = boxplus(reference, delta, model=model)
        data = self.problem.plant.make_data()
        biased.restore(
            plant=self.problem.plant,
            data=data,
            drive_state=DriveState(),
        )
        mujoco.mj_forward(model, data)
        contact_summary = self.problem.plant.contact_wrench_summary(data)
        designated_distances = [
            float(data.contact[int(entry["index"])].dist)
            for entry in contact_summary["contacts"]
            if entry["designated_foot"] is not None
        ]
        if not designated_distances:
            raise NLPBindingError(
                f"WITNESS initializer knot {knot} has no designated pad-floor contact"
            )
        delta[2] = -(
            max(designated_distances)
            + WITNESS_CONTACT_STENCIL_MULTIPLIER
            * ML241_STATE_STEPS["configuration_translation"]
        )
        delta[WITNESS_A_MINUS_9_STATE_INDEX] = (
            WITNESS_NATIVE_STENCIL_MULTIPLIER
            * ML241_STATE_STEPS["drivestate_activation"]
            - float(reference.a_minus[9])
        )
        delta[WITNESS_A_MINUS_10_STATE_INDEX] = (
            WITNESS_NATIVE_STENCIL_MULTIPLIER
            * ML241_STATE_STEPS["drivestate_activation"]
            - float(reference.a_minus[10])
        )

        synthesized = boxplus(reference, delta, model=model)
        # Package through the existing tangent contract rather than relying on
        # raw Euclidean arithmetic to represent a state decision.
        packaged = boxminus(synthesized, reference, model=model)
        if not np.isfinite(packaged).all():
            raise NLPBindingError(f"WITNESS initializer knot {knot} is non-finite")
        if np.max(np.abs(packaged)) > WITNESS_MAX_INITIALIZER_DISPLACEMENT:
            raise NLPBindingError(
                f"WITNESS initializer knot {knot} exceeds the frozen displacement bound"
            )
        return packaged

    def _validate_initial_vector(self, value: np.ndarray) -> None:
        self._decision_vector(value, "production initializer")

    def _decision_vector(
        self,
        value: Sequence[float] | np.ndarray,
        label: str,
    ) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (self.variable_count,) or not np.isfinite(vector).all():
            raise NLPBindingError(f"{label} must be a finite vector of length {self.variable_count}")
        lower = self.variable_lower_bounds()
        upper = self.variable_upper_bounds()
        if np.any(vector < lower) or np.any(vector > upper):
            raise NLPBindingError(f"{label} is outside the declared variable bounds")
        return vector.copy()

    def objective(self, z: Sequence[float] | np.ndarray) -> float:
        return self.problem.objective(self._decision_vector(z, "objective input"))

    def gradient(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        return self.problem.objective_gradient(self._decision_vector(z, "gradient input"))

    def _physical_value_evaluator(self, values: np.ndarray):
        def evaluate(row: Any, _z: np.ndarray, _problem: DirectMultipleShootingProblem) -> float:
            try:
                physical_index = self._physical_index_by_row[row.row_index]
            except KeyError as exc:
                raise NLPBindingError(f"row {row.row_index} is not a canonical physical row") from exc
            return float(values[physical_index])

        return evaluate

    def constraint_values(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        value = self._decision_vector(z, "constraint input")
        values = self.composer.physical_constraint_values(value)
        return self.problem.constraint_values(
            value,
            evaluator=self._physical_value_evaluator(values),
        )

    def constraints(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        return self.constraint_values(z)

    @staticmethod
    def _physical_jacobian_evaluator(
        derivatives: tuple[PhysicalConstraintDerivative, ...],
        physical_index_by_row: Mapping[int, int],
    ):
        by_index = {item.row.physical_index: item for item in derivatives}

        def evaluate(row: Any, _z: np.ndarray, _problem: DirectMultipleShootingProblem) -> np.ndarray:
            try:
                physical_index = physical_index_by_row[row.row_index]
                derivative = by_index[physical_index]
            except KeyError as exc:
                raise NLPBindingError(f"row {row.row_index} is not a canonical physical row") from exc
            if derivative.row.row_index != row.row_index:
                raise NLPBindingError("physical Jacobian provenance does not match the row")
            values: list[float] = []
            for kind, knot in row.support:
                if kind != "state":
                    raise NLPBindingError("production physical rows may not acquire action support")
                if knot == 0:
                    if derivative.state_columns or derivative.state_values.shape != (1, 0):
                        raise NLPBindingError("knot-zero physical row received decision-state support")
                else:
                    expected = tuple(range(STATE_DIMENSION))
                    if derivative.state_columns != expected:
                        raise NLPBindingError("physical state derivative support changed")
                    state_values = np.asarray(derivative.state_values, dtype=np.float64)
                    if state_values.shape != (1, STATE_DIMENSION):
                        raise NLPBindingError("physical state derivative has the wrong shape")
                    values.extend(state_values[0].tolist())
            if row.slack_decision_index is not None:
                values.append(1.0)
            return np.asarray(values, dtype=np.float64)

        return evaluate

    def jacobian_values(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        value = self._decision_vector(z, "Jacobian input")
        derivatives = self.composer.physical_constraint_jacobian_values(value)
        evaluator = self._physical_jacobian_evaluator(
            derivatives,
            self._physical_index_by_row,
        )
        return self.problem.jacobian_values(value, catalog_jacobian_evaluator=evaluator)

    def jacobian(self, z: Sequence[float] | np.ndarray) -> np.ndarray:
        return self.jacobian_values(z)

    def physical_constraint_owner_receipt(self) -> tuple[PhysicalConstraintOwnerReceipt, ...]:
        """Return the live G4R2/ML241 owner receipt used by this binding."""

        return self.composer.physical_constraint_owner_receipt()


__all__ = [
    "CONSTRAINT_COUNT",
    "E3_E4_INITIAL_ACTION_PROFILE",
    "E3E4NLPBinding",
    "ELASTIC_SLACK_COUNT",
    "EPIGRAPH_ROW_COUNT",
    "FULL_VARIABLE_COUNT",
    "JACOBIAN_NNZ",
    "NLPBindingError",
    "WITNESS_VARIABLE_COUNT",
]
