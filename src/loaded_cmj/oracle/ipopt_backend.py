"""Thin offline cyipopt adapter for the frozen ML-242 transcription.

This module owns no plant, constraint, or derivative equations.  It only
connects the already-qualified transcription callbacks to ``cyipopt``.
Importing the module does not import cyipopt; the dependency is deliberately
loaded only by :meth:`IpoptAdapter.solve` at the offline oracle boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from math import isfinite
from types import MappingProxyType
from typing import Any, Callable, Mapping

import numpy as np

from .transcription import QACC_ERROR_BOUNDS as _QACC_ERROR_BOUNDS


ML242_FREEZE_ID = "674c62a406d99b9ba706ec9a62d8602f2df066ef1eff15a4fa87fb3d2145755a"
ML242_SPARSITY_PATTERN_SHA256 = "b194ddce914f94ef98cf25e7784e583f496c8dc474401bef34e90214536933c4"
N40_VARIABLE_COUNT = 6012
N40_CONSTRAINT_COUNT = 5280
N40_JACOBIAN_NNZ = 1473120
QACC_ZERO_COLUMNS = tuple(range(111, 132))
QACC_DERIVATIVE_DISPOSITION = "NUMERICALLY_NULL_WITH_BOUNDED_ERROR"
QACC_ABSOLUTE_ERROR_BOUNDS = MappingProxyType(dict(_QACC_ERROR_BOUNDS))
HESSIAN_CONTRACT = "IPOPT_LIMITED_MEMORY"
DEFAULT_CALLBACK_BUDGET: Mapping[str, int] = MappingProxyType(
    {
        "objective": 1000,
        "gradient": 1000,
        "constraints": 1000,
        "jacobian": 1000,
        "intermediate": 1000,
    }
)


DEFAULT_SOLVER_OPTIONS: Mapping[str, int | float | str] = MappingProxyType(
    {
        "hessian_approximation": "limited-memory",
        "linear_solver": "mumps",
        "nlp_scaling_method": "gradient-based",
        "obj_scaling_factor": 1.0,
        "tol": 1.0e-7,
        "constr_viol_tol": 1.0e-7,
        "dual_inf_tol": 1.0e-6,
        "compl_inf_tol": 1.0e-6,
        "acceptable_tol": 1.0e-6,
        "acceptable_iter": 3,
        "max_iter": 100,
        # The existing ML-242 qualification-only same-Plant derivative
        "max_cpu_time": 30.0,
        "print_level": 0,
        "sb": "yes",
        "check_derivatives_for_naninf": "yes",
        "jacobian_approximation": "exact",
        "mu_strategy": "adaptive",
        "bound_relax_factor": 0.0,
        "bound_push": 0.01,
        "bound_frac": 0.01,
        "slack_bound_push": 0.01,
        "slack_bound_frac": 0.01,
        "least_square_init_primal": "no",
        "warm_start_init_point": "no",
        "derivative_test": "none",
        "option_file_name": "",
    }
)


class SolverContractError(ValueError):
    """The adapter or backend configuration violates the frozen contract."""


class SparseCallbackError(SolverContractError):
    """A sparse callback has the wrong shape, order, or finite-value contract."""


class UnresolvedPhysicalScaleError(SolverContractError):
    """A Phase-I problem requested a physical scale that is still unresolved."""


@dataclass
class CallbackCounters:
    objective: int = 0
    gradient: int = 0
    constraints: int = 0
    jacobian: int = 0
    intermediate: int = 0

    def as_dict(self) -> dict[str, int]:
        return {
            "objective": self.objective,
            "gradient": self.gradient,
            "constraints": self.constraints,
            "jacobian": self.jacobian,
            "intermediate": self.intermediate,
        }


@dataclass(frozen=True)
class IpoptSolveReceipt:
    """Small deterministic receipt returned by one backend attempt."""

    x: np.ndarray
    info: Mapping[str, Any]
    status: int
    status_text: str
    classification: str
    iterations: int | None
    counters: Mapping[str, int]


def sparse_structure_hash(rows: np.ndarray, cols: np.ndarray) -> str:
    """Hash the exact int64 row/column callback order used by ML-242."""

    digest = sha256()
    digest.update(np.asarray(rows, dtype=np.int64).tobytes(order="C"))
    digest.update(np.asarray(cols, dtype=np.int64).tobytes(order="C"))
    return digest.hexdigest()


def require_resolved_phase_i_scale(constraint_id: str, phase_i_scale: object) -> None:
    """Reject unresolved physical normalization for a requested Phase-I row."""

    if constraint_id == "E3_E4_REVERSAL" and (
        phase_i_scale is None
        or str(phase_i_scale).strip().upper() in {"UNRESOLVED", "UNRESOLVED_PRE_ML244"}
    ):
        raise UnresolvedPhysicalScaleError(
            "E3_E4_REVERSAL Phase-I scale is UNRESOLVED_PRE_ML244; "
            "ML-243 may not instantiate that elastic problem"
        )


def classify_ipopt_status(status: int, status_msg: object = "") -> str:
    """Translate Ipopt's integer result without conflating failure and infeasibility."""

    code = int(status)
    text = status_msg.decode(errors="replace") if isinstance(status_msg, bytes) else str(status_msg)
    if code == 0:
        return "SOLVE_SUCCESS"
    if code == 1:
        return "ACCEPTABLE_SUCCESS"
    if code == 2:
        return "INFEASIBLE_PROBLEM"
    if code == -1:
        return "MAX_ITER"
    if code in {-4, 5}:
        return "MAX_TIME/BUDGET"
    if code == -2:
        return "RESTORATION_FAILURE"
    if code in {-10, -11, -12, -13}:
        return "INVALID_CALLBACK"
    if code == -100 or "evaluation" in text.lower():
        return "EVALUATION_ERROR"
    if code in {-3, -99, -102, -199, 3, 4}:
        return "NUMERICAL_FAILURE"
    return "UNKNOWN"


class IpoptAdapter:
    """Direct adapter over one ``DirectMultipleShootingProblem`` instance."""

    def __init__(
        self,
        problem: Any,
        objective: Callable[[np.ndarray], float],
        gradient: Callable[[np.ndarray], np.ndarray],
        *,
        options: Mapping[str, int | float | str] = DEFAULT_SOLVER_OPTIONS,
        evaluation_budget: Mapping[str, int] = DEFAULT_CALLBACK_BUDGET,
        verify_frozen_ml242: bool = True,
        expected_structure_hash: str | None = None,
    ) -> None:
        self.problem = problem
        self._objective = objective
        self._gradient = gradient
        self.options = dict(DEFAULT_SOLVER_OPTIONS)
        self.options.update(options)
        self.evaluation_budget = {
            name: int(value) for name, value in evaluation_budget.items()
        }
        if set(self.evaluation_budget) != set(DEFAULT_CALLBACK_BUDGET) or any(
            value <= 0 for value in self.evaluation_budget.values()
        ):
            raise SolverContractError("evaluation budget must define positive limits for every callback")
        self.counters = CallbackCounters()
        self._last_iteration: int | None = None
        self._verify_frozen_ml242 = bool(verify_frozen_ml242)
        self._validate_options()

        self.n = int(problem.variable_count)
        self.m = int(problem.constraint_count)
        self.variable_lower_bounds = self._finite_or_inf_vector(
            problem.variable_lower_bounds(), self.n, "variable lower bounds"
        )
        self.variable_upper_bounds = self._finite_or_inf_vector(
            problem.variable_upper_bounds(), self.n, "variable upper bounds"
        )
        self.constraint_lower_bounds = self._finite_or_inf_vector(
            problem.constraint_lower_bounds(), self.m, "constraint lower bounds"
        )
        self.constraint_upper_bounds = self._finite_or_inf_vector(
            problem.constraint_upper_bounds(), self.m, "constraint upper bounds"
        )
        self._rows = self._integer_vector(problem.jacobian_rows(), "Jacobian rows")
        self._cols = self._integer_vector(problem.jacobian_cols(), "Jacobian columns")
        self.structure_hash = self._validate_structure(self._rows, self._cols)

        if expected_structure_hash is not None and self.structure_hash != expected_structure_hash:
            raise SparseCallbackError(
                f"Jacobian structure hash {self.structure_hash} != expected {expected_structure_hash}"
            )
        if verify_frozen_ml242:
            self._validate_ml242_identity(expected_structure_hash)
        self._guard_active_phase_i_scales()

    def _validate_options(self) -> None:
        if self.options.get("hessian_approximation") != "limited-memory":
            raise SolverContractError("HESSIAN_CONTRACT requires Ipopt limited-memory")
        if self.options.get("linear_solver") != "mumps":
            raise SolverContractError("ML-243 freezes the MUMPS linear solver")
        if self.options.get("jacobian_approximation") != "exact":
            raise SolverContractError("ML-243 requires exact sparse Jacobian callbacks")
        frozen = {
            "bound_relax_factor": 0.0,
            "bound_push": 0.01,
            "bound_frac": 0.01,
            "slack_bound_push": 0.01,
        }
        for name, expected in frozen.items():
            if self.options.get(name) != expected:
                raise SolverContractError(
                    f"ML-243 freezes {name}={expected!r} for native-domain initialization"
                )
        for name, expected in (
            ("slack_bound_frac", 0.01),
            ("least_square_init_primal", "no"),
            ("warm_start_init_point", "no"),
            ("option_file_name", ""),
        ):
            if self.options.get(name) != expected:
                raise SolverContractError(f"ML-243 freezes {name}={expected!r}")
        if self._verify_frozen_ml242 and self.options.get("derivative_test") != "none":
            raise SolverContractError("ML-243 target callbacks require derivative_test=none")

    @staticmethod
    def _integer_vector(value: object, label: str) -> np.ndarray:
        array = np.asarray(value)
        if array.ndim != 1 or array.dtype.kind not in "iu":
            raise SparseCallbackError(f"{label} must be a one-dimensional integer array")
        return np.asarray(array, dtype=np.int64).copy()

    @staticmethod
    def _finite_or_inf_vector(value: object, length: int, label: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (length,) or np.isnan(array).any():
            raise SolverContractError(f"{label} must be a finite/inf vector of length {length}")
        return array.copy()

    def _validate_structure(self, rows: np.ndarray, cols: np.ndarray) -> str:
        if rows.size == 0 or rows.size != cols.size:
            raise SparseCallbackError("Jacobian row/column arrays must be nonempty and equal length")
        if rows.min() < 0 or rows.max() >= self.m:
            raise SparseCallbackError("Jacobian row index is outside constraint bounds")
        if cols.min() < 0 or cols.max() >= self.n:
            raise SparseCallbackError("Jacobian column index is outside variable bounds")
        order = np.lexsort((cols, rows))
        if np.any((rows[order][1:] == rows[order][:-1]) & (cols[order][1:] == cols[order][:-1])):
            raise SparseCallbackError("Jacobian structure contains duplicate row/column pairs")
        return sparse_structure_hash(rows, cols)

    def _validate_ml242_identity(self, expected_structure_hash: str | None) -> None:
        observed_freeze_id = getattr(self.problem, "ml242_freeze_id", ML242_FREEZE_ID)
        if observed_freeze_id != ML242_FREEZE_ID:
            raise SolverContractError(f"ML-242 freeze ID mismatch: {observed_freeze_id}")
        if expected_structure_hash is None and self.n == N40_VARIABLE_COUNT and self.m == N40_CONSTRAINT_COUNT:
            if self._rows.size != N40_JACOBIAN_NNZ or self.structure_hash != ML242_SPARSITY_PATTERN_SHA256:
                raise SparseCallbackError("N=40 ML-242 sparsity identity mismatch")
        problem_hash = getattr(self.problem, "jacobian_structure_hash", None)
        if callable(problem_hash) and problem_hash() != self.structure_hash:
            raise SparseCallbackError("problem-reported Jacobian structure hash disagrees with callbacks")

    def _guard_active_phase_i_scales(self) -> None:
        if int(getattr(self.problem, "elastic_slack_count", 0)) <= 0:
            return
        schema = getattr(self.problem, "elastic_slack_schema", ())
        for record in schema:
            if record.get("constraint_id") == "E3_E4_REVERSAL":
                require_resolved_phase_i_scale(record["constraint_id"], record.get("phase_i_scale"))

    def _vector(self, value: object, length: int, label: str) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (length,) or not np.isfinite(vector).all():
            raise SolverContractError(f"{label} must be a finite vector of length {length}")
        return vector.copy()

    def _decision_vector(self, value: object, label: str) -> np.ndarray:
        vector = self._vector(value, self.n, label)
        if np.any(vector < self.variable_lower_bounds) or np.any(
            vector > self.variable_upper_bounds
        ):
            raise SolverContractError(f"{label} is outside the declared variable bounds")
        return vector

    def _count(self, name: str) -> None:
        current = getattr(self.counters, name) + 1
        if current > self.evaluation_budget[name]:
            raise SolverContractError(f"{name} callback budget exceeded")
        setattr(self.counters, name, current)

    def objective(self, x: object) -> float:
        self._count("objective")
        value = self._decision_vector(x, "objective input")
        try:
            result = float(self._objective(value))
        except SolverContractError:
            raise
        except Exception as exc:
            raise SolverContractError("objective callback rejected the decision point") from exc
        if not isfinite(result):
            raise SolverContractError("objective returned NaN or Inf")
        return result

    def gradient(self, x: object) -> np.ndarray:
        self._count("gradient")
        value = self._decision_vector(x, "gradient input")
        try:
            result = self._gradient(value)
        except SolverContractError:
            raise
        except Exception as exc:
            raise SolverContractError("gradient callback rejected the decision point") from exc
        return self._vector(result, self.n, "gradient result")

    def constraints(self, x: object) -> np.ndarray:
        self._count("constraints")
        value = self._decision_vector(x, "constraint input")
        try:
            result = self.problem.constraint_values(value)
        except SolverContractError:
            raise
        except Exception as exc:
            raise SolverContractError("constraints callback rejected the decision point") from exc
        return self._vector(result, self.m, "constraint result")

    def jacobian(self, x: object) -> np.ndarray:
        self._count("jacobian")
        value = self._decision_vector(x, "Jacobian input")
        try:
            result = self.problem.jacobian_values(value)
        except SolverContractError:
            raise
        except Exception as exc:
            raise SolverContractError("Jacobian callback rejected the decision point") from exc
        values = self._vector(result, self._rows.size, "Jacobian values")
        assert len(values) == len(self._rows) == len(self._cols)
        if len(values) != len(self._rows) or len(values) != len(self._cols):
            raise SparseCallbackError(
                f"Jacobian value count {len(values)} != rows {len(self._rows)} != cols {len(self._cols)}"
            )
        return values

    def jacobianstructure(self) -> tuple[np.ndarray, np.ndarray]:
        return self._rows.copy(), self._cols.copy()

    def intermediate(self, *_: object) -> bool:
        self._count("intermediate")
        if len(_) > 1:
            self._last_iteration = int(_[1])
        return True

    def solve(self, initial_x: object) -> IpoptSolveReceipt:
        """Run exactly one deterministic Ipopt attempt from ``initial_x``."""

        try:
            import cyipopt
        except ImportError as exc:  # pragma: no cover - exercised by environment gate
            raise SolverContractError("cyipopt is unavailable at the offline solver boundary") from exc

        x0 = self._decision_vector(initial_x, "initial vector")
        self.counters = CallbackCounters()
        self._last_iteration = None
        backend = cyipopt.Problem(
            self.n,
            self.m,
            self,
            lb=self.variable_lower_bounds,
            ub=self.variable_upper_bounds,
            cl=self.constraint_lower_bounds,
            cu=self.constraint_upper_bounds,
        )
        for name, value in self.options.items():
            backend.add_option(name, value)
        x, info = backend.solve(x0)
        raw_status = int(info.get("status", -199))
        status_msg = info.get("status_msg", "")
        status_text = status_msg.decode(errors="replace") if isinstance(status_msg, bytes) else str(status_msg)
        iterations = info.get("iter_count", self._last_iteration)
        return IpoptSolveReceipt(
            x=np.asarray(x, dtype=np.float64).copy(),
            info=dict(info),
            status=raw_status,
            status_text=status_text,
            classification=classify_ipopt_status(raw_status, status_text),
            iterations=None if iterations is None else int(iterations),
            counters=MappingProxyType(self.counters.as_dict()),
        )
