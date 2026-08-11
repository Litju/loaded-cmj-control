"""ML-243 cyipopt/Ipopt backend qualification tests.

These tests qualify the adapter boundary only.  The same-Plant cases use the
existing ML-242 qualification-only exact-transition provider; they do not
change the production derivative owner or attempt an E3-to-E4 solve.
"""

from __future__ import annotations

import ast
from copy import copy
from hashlib import sha256
import os
from pathlib import Path
import subprocess
import sys
from typing import Callable

import numpy as np
import pytest

from loaded_cmj.oracle.ipopt_backend import (
    DEFAULT_SOLVER_OPTIONS,
    HESSIAN_CONTRACT,
    IpoptAdapter,
    QACC_ABSOLUTE_ERROR_BOUNDS,
    QACC_DERIVATIVE_DISPOSITION,
    QACC_ZERO_COLUMNS,
    SolverContractError,
    SparseCallbackError,
    UnresolvedPhysicalScaleError,
    classify_ipopt_status,
    require_resolved_phase_i_scale,
    sparse_structure_hash,
)
from loaded_cmj.oracle.derivatives import snapshot_digest


ROOT = Path(__file__).resolve().parents[1]
N1_STRUCTURE_HASH = "4570fcf2ceacd7a66b147fc7b6ca0b40736bb1086d4e8f9ed916f787c1990cc4"
ML242_FREEZE_ID = "674c62a406d99b9ba706ec9a62d8602f2df066ef1eff15a4fa87fb3d2145755a"
ML242_SPARSITY_HASH = "b194ddce914f94ef98cf25e7784e583f496c8dc474401bef34e90214536933c4"
SAME_PLANT_ACCEPTABLE_DEFECT_TOL = 2.0e-7
N1_NEXT_STATE_INDEX = 147


class ProblemStub:
    """Small callback-only problem used to test the adapter in isolation."""

    def __init__(
        self,
        *,
        n: int,
        m: int,
        rows: np.ndarray,
        cols: np.ndarray,
        constraint_fn: Callable[[np.ndarray], np.ndarray],
        jacobian_fn: Callable[[np.ndarray], np.ndarray],
        variable_lower: np.ndarray | None = None,
        variable_upper: np.ndarray | None = None,
        constraint_lower: np.ndarray | None = None,
        constraint_upper: np.ndarray | None = None,
    ) -> None:
        self._n = n
        self._m = m
        self._rows = np.asarray(rows, dtype=np.int64)
        self._cols = np.asarray(cols, dtype=np.int64)
        self._constraint_fn = constraint_fn
        self._jacobian_fn = jacobian_fn
        self._variable_lower = (
            np.full(n, -10.0) if variable_lower is None else np.asarray(variable_lower, dtype=np.float64)
        )
        self._variable_upper = (
            np.full(n, 10.0) if variable_upper is None else np.asarray(variable_upper, dtype=np.float64)
        )
        self._constraint_lower = (
            np.full(m, -np.inf)
            if constraint_lower is None
            else np.asarray(constraint_lower, dtype=np.float64)
        )
        self._constraint_upper = (
            np.full(m, np.inf)
            if constraint_upper is None
            else np.asarray(constraint_upper, dtype=np.float64)
        )
        self.elastic_slack_count = 0
        self.elastic_slack_schema: tuple[dict[str, object], ...] = ()

    @property
    def variable_count(self) -> int:
        return self._n

    @property
    def constraint_count(self) -> int:
        return self._m

    def variable_lower_bounds(self) -> np.ndarray:
        return self._variable_lower.copy()

    def variable_upper_bounds(self) -> np.ndarray:
        return self._variable_upper.copy()

    def constraint_lower_bounds(self) -> np.ndarray:
        return self._constraint_lower.copy()

    def constraint_upper_bounds(self) -> np.ndarray:
        return self._constraint_upper.copy()

    def constraint_values(self, z: np.ndarray) -> np.ndarray:
        return np.asarray(self._constraint_fn(z), dtype=np.float64)

    def jacobian_rows(self) -> np.ndarray:
        return self._rows.copy()

    def jacobian_cols(self) -> np.ndarray:
        return self._cols.copy()

    def jacobian_values(self, z: np.ndarray) -> np.ndarray:
        return np.asarray(self._jacobian_fn(z), dtype=np.float64)


def _linear_problem(*, jacobian_fn: Callable[[np.ndarray], np.ndarray] | None = None) -> ProblemStub:
    rows = np.array([0, 0], dtype=np.int64)
    cols = np.array([0, 1], dtype=np.int64)
    return ProblemStub(
        n=2,
        m=1,
        rows=rows,
        cols=cols,
        constraint_fn=lambda x: np.array([x[0] + 2.0 * x[1]]),
        jacobian_fn=(lambda _: np.array([1.0, 2.0])) if jacobian_fn is None else jacobian_fn,
        constraint_lower=np.array([5.0]),
        constraint_upper=np.array([5.0]),
    )


def _linear_adapter(problem: ProblemStub) -> IpoptAdapter:
    return IpoptAdapter(
        problem,
        objective=lambda x: float((x[0] - 1.0) ** 2 + (x[1] - 2.0) ** 2),
        gradient=lambda x: 2.0 * (x - np.array([1.0, 2.0])),
        verify_frozen_ml242=False,
    )


def _nonlinear_problem() -> ProblemStub:
    rows = np.array([0, 0, 1, 1], dtype=np.int64)
    cols = np.array([0, 1, 0, 1], dtype=np.int64)
    return ProblemStub(
        n=2,
        m=2,
        rows=rows,
        cols=cols,
        constraint_fn=lambda x: np.array([x[0] * x[0] - 1.0, x[1] - 1.0]),
        jacobian_fn=lambda x: np.array([2.0 * x[0], 0.0, 0.0, 1.0]),
        variable_lower=np.array([0.1, 0.1]),
        variable_upper=np.array([3.0, 3.0]),
        constraint_lower=np.zeros(2),
        constraint_upper=np.zeros(2),
    )


def _ml242_n40_structure() -> tuple[np.ndarray, np.ndarray]:
    state_dim = 132
    action_dim = 15
    horizon = 40
    rows_parts: list[np.ndarray] = []
    cols_parts: list[np.ndarray] = []
    for interval in range(horizon):
        state_start = interval * (state_dim + action_dim)
        action_start = state_start + state_dim
        next_state_start = action_start + action_dim
        block = np.concatenate(
            (
                np.arange(state_start, action_start, dtype=np.int64),
                np.arange(action_start, next_state_start, dtype=np.int64),
                np.arange(next_state_start, next_state_start + state_dim, dtype=np.int64),
            )
        )
        rows_parts.append(
            np.repeat(
                np.arange(interval * state_dim, (interval + 1) * state_dim, dtype=np.int64),
                block.size,
            )
        )
        cols_parts.append(np.tile(block, state_dim))
    return np.concatenate(rows_parts), np.concatenate(cols_parts)


def _n40_problem() -> ProblemStub:
    rows, cols = _ml242_n40_structure()
    n = 6012
    m = 5280
    problem = ProblemStub(
        n=n,
        m=m,
        rows=rows,
        cols=cols,
        constraint_fn=lambda _: np.zeros(m),
        jacobian_fn=lambda _: np.zeros(rows.size),
        variable_lower=np.full(n, -1.0),
        variable_upper=np.full(n, 1.0),
        constraint_lower=np.zeros(m),
        constraint_upper=np.zeros(m),
    )
    problem.ml242_freeze_id = ML242_FREEZE_ID
    problem.jacobian_structure_hash = lambda: ML242_SPARSITY_HASH
    return problem


def test_synthetic_sparse_linear_known_solution_and_counters() -> None:
    problem = _linear_problem()
    adapter = _linear_adapter(problem)
    receipt = adapter.solve(np.array([0.0, 0.0]))

    assert receipt.classification == "SOLVE_SUCCESS"
    assert np.allclose(receipt.x, np.array([1.0, 2.0]), atol=1.0e-6)
    assert abs(float(problem.constraint_values(receipt.x)[0]) - 5.0) <= 1.0e-7
    assert all(receipt.counters[name] > 0 for name in ("objective", "gradient", "constraints", "jacobian"))
    assert receipt.iterations is not None


def test_synthetic_sparse_jacobian_order_and_hash() -> None:
    problem = _linear_problem()
    adapter = _linear_adapter(problem)
    rows, cols = adapter.jacobianstructure()
    assert np.array_equal(rows, np.array([0, 0], dtype=np.int64))
    assert np.array_equal(cols, np.array([0, 1], dtype=np.int64))
    assert adapter.structure_hash == sparse_structure_hash(rows, cols)
    assert np.array_equal(adapter.jacobian(np.array([0.0, 0.0])), np.array([1.0, 2.0]))


def test_synthetic_nonlinear_solution_and_ipopt_derivative_checker() -> None:
    problem = _nonlinear_problem()
    adapter = IpoptAdapter(
        problem,
        objective=lambda x: 0.5 * float(np.sum((x - 1.0) ** 2)),
        gradient=lambda x: x - 1.0,
        verify_frozen_ml242=False,
        options={
            "derivative_test": "first-order",
            "derivative_test_tol": 1.0e-5,
            "derivative_test_perturbation": 1.0e-8,
            "derivative_test_print_all": "yes",
            "print_level": 5,
        },
    )
    receipt = adapter.solve(np.array([1.2, 0.8]))

    assert receipt.classification == "SOLVE_SUCCESS"
    assert np.allclose(receipt.x, np.ones(2), atol=1.0e-6)
    assert np.max(np.abs(problem.constraint_values(receipt.x))) <= 1.0e-7


@pytest.mark.parametrize(
    ("status", "message", "expected"),
    (
        (0, "", "SOLVE_SUCCESS"),
        (1, "", "ACCEPTABLE_SUCCESS"),
        (2, "", "INFEASIBLE_PROBLEM"),
        (-1, "", "MAX_ITER"),
        (-4, "", "MAX_TIME/BUDGET"),
        (5, "", "MAX_TIME/BUDGET"),
        (-2, "", "RESTORATION_FAILURE"),
        (-10, "", "INVALID_CALLBACK"),
        (-100, "", "EVALUATION_ERROR"),
        (-3, "", "NUMERICAL_FAILURE"),
        (99, "unclassified", "UNKNOWN"),
    ),
)
def test_status_classification(status: int, message: str, expected: str) -> None:
    assert classify_ipopt_status(status, message) == expected


def test_callback_negative_cases_are_rejected() -> None:
    wrong_values = _linear_problem(jacobian_fn=lambda _: np.array([1.0]))
    adapter = _linear_adapter(wrong_values)
    with pytest.raises(SolverContractError):
        adapter.jacobian(np.zeros(2))

    with pytest.raises(SparseCallbackError):
        IpoptAdapter(
            _linear_problem(),
            objective=lambda _: 0.0,
            gradient=lambda x: np.zeros_like(x),
            verify_frozen_ml242=False,
            expected_structure_hash="0" * 64,
        )

    with pytest.raises(SparseCallbackError):
        IpoptAdapter(
            ProblemStub(
                n=2,
                m=1,
                rows=np.array([0, 1]),
                cols=np.array([0, 1]),
                constraint_fn=lambda _: np.zeros(1),
                jacobian_fn=lambda _: np.ones(2),
            ),
            objective=lambda _: 0.0,
            gradient=lambda x: np.zeros_like(x),
            verify_frozen_ml242=False,
        )

    with pytest.raises(SparseCallbackError):
        IpoptAdapter(
            ProblemStub(
                n=2,
                m=1,
                rows=np.array([0, 0]),
                cols=np.array([0, 0]),
                constraint_fn=lambda _: np.zeros(1),
                jacobian_fn=lambda _: np.ones(2),
            ),
            objective=lambda _: 0.0,
            gradient=lambda x: np.zeros_like(x),
            verify_frozen_ml242=False,
        )


def test_frozen_hessian_scaling_and_exact_jacobian_contract() -> None:
    adapter = _linear_adapter(_linear_problem())
    assert HESSIAN_CONTRACT == "IPOPT_LIMITED_MEMORY"
    assert adapter.options["hessian_approximation"] == "limited-memory"
    assert adapter.options["linear_solver"] == "mumps"
    assert adapter.options["jacobian_approximation"] == "exact"
    assert adapter.options["nlp_scaling_method"] == "gradient-based"
    assert adapter.options["obj_scaling_factor"] == 1.0

    with pytest.raises(SolverContractError):
        _ = IpoptAdapter(
            _linear_problem(),
            objective=lambda _: 0.0,
            gradient=lambda x: np.zeros_like(x),
            verify_frozen_ml242=False,
            options={"hessian_approximation": "exact"},
        )


def test_callback_evaluation_budget_is_frozen_and_enforced() -> None:
    adapter = IpoptAdapter(
        _linear_problem(),
        objective=lambda _: 0.0,
        gradient=lambda x: np.zeros_like(x),
        verify_frozen_ml242=False,
        evaluation_budget={
            "objective": 1,
            "gradient": 1,
            "constraints": 1,
            "jacobian": 1,
            "intermediate": 1,
        },
    )
    adapter.objective(np.zeros(2))
    with pytest.raises(SolverContractError):
        adapter.objective(np.zeros(2))


def test_n40_frozen_sparse_structure_without_dense_fallback() -> None:
    problem = _n40_problem()
    adapter = IpoptAdapter(
        problem,
        objective=lambda _: 0.0,
        gradient=lambda x: np.zeros_like(x),
        expected_structure_hash=ML242_SPARSITY_HASH,
    )
    rows, cols = adapter.jacobianstructure()

    assert adapter.n == 6012
    assert adapter.m == 5280
    assert len(rows) == len(cols) == 1_473_120
    assert adapter.structure_hash == ML242_SPARSITY_HASH
    assert not hasattr(adapter, "dense_jacobian")
    source = (ROOT / "src/loaded_cmj/oracle/ipopt_backend.py").read_text()
    assert "np.zeros((" not in source
    assert "scipy.sparse" not in source


def test_unresolved_e3_e4_scale_guard() -> None:
    with pytest.raises(UnresolvedPhysicalScaleError):
        require_resolved_phase_i_scale("E3_E4_REVERSAL", "UNRESOLVED")
    with pytest.raises(UnresolvedPhysicalScaleError):
        require_resolved_phase_i_scale("E3_E4_REVERSAL", "UNRESOLVED_PRE_ML244")
    require_resolved_phase_i_scale("E3_E4_REVERSAL", "NOT_APPLICABLE")
    require_resolved_phase_i_scale("DYNAMICS", "UNRESOLVED")

    problem = _linear_problem()
    problem.elastic_slack_count = 1
    problem.elastic_slack_schema = (
        {"constraint_id": "E3_E4_REVERSAL", "phase_i_scale": "UNRESOLVED"},
    )
    with pytest.raises(UnresolvedPhysicalScaleError):
        _linear_adapter(problem)


def test_qacc_noise_contract_is_consumed_without_removing_state() -> None:
    assert QACC_ZERO_COLUMNS == tuple(range(111, 132))
    assert QACC_DERIVATIVE_DISPOSITION == "NUMERICALLY_NULL_WITH_BOUNDED_ERROR"
    assert QACC_ABSOLUTE_ERROR_BOUNDS == {
        "qacc_translation": 1.0e-7,
        "qacc_rotation_joint": 1.0e-7,
    }


def test_adapter_is_thin_and_has_no_physics_or_duplicate_constraint_model() -> None:
    path = ROOT / "src/loaded_cmj/oracle/ipopt_backend.py"
    source = path.read_text()
    tree = ast.parse(source)
    imported_modules = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    imported_from = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    assert "loaded_cmj.simulation" not in " ".join(imported_modules | imported_from)
    assert "step_5ms" not in source
    assert "J_prop" not in source
    assert "jump_height" not in source
    assert "scipy" not in source.lower()


def test_public_runtime_import_does_not_require_cyipopt() -> None:
    script = """
from importlib.abc import MetaPathFinder
import sys

class BlockCyipopt(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'cyipopt' or fullname.startswith('cyipopt.'):
            raise RuntimeError('public runtime attempted to import cyipopt')
        return None

sys.meta_path.insert(0, BlockCyipopt())
from loaded_cmj.runtime.policy_worker import PolicyWorker
assert callable(PolicyWorker)
"""
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(ROOT / "src")
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_ml241_and_physical_substrate_hashes_are_unchanged_after_reversal_erratum() -> None:
    expected = {
        "src/loaded_cmj/oracle/derivatives.py": "4faf445093f918d403df7d810e0b29243444b569d1d029f73bd3d3de2052bc6f",
        "src/loaded_cmj/simulation/snapshot.py": "8d06a55969dcf15f482480f05688b2245ac2897bc70ca3c0e221e1bea5f368cc",
        "src/loaded_cmj/simulation/tangent.py": "44f7ed049c7cbb2098062669a4193808d575ef1e7aa6bc9a2d16563b0cbdb6c5",
        "src/loaded_cmj/simulation/transition.py": "795f2368b6dfab56ef95838d128998b958b572d9a1f61ab62d7e31b100ef9692",
    }
    for relative_path, digest in expected.items():
        assert sha256((ROOT / relative_path).read_bytes()).hexdigest() == digest, relative_path


@pytest.fixture(scope="module")
def same_plant_case() -> dict[str, object]:
    tests_dir = str(ROOT / "tests")
    if tests_dir not in sys.path:
        sys.path.insert(0, tests_dir)
    from qualification_ml242_transcription import _fixture_data, _independent_provider, _reference_problem

    # S3 is the smallest legacy fixture whose ML-242 qualification provider
    # remains inside the directional-activation chart under its central probe.
    # S4's zero activation is intentionally rejected by the frozen plant model.
    fixture = _fixture_data()["S3"]
    exact_provider = _independent_provider()
    actions = (np.asarray(fixture.raw_action, dtype=np.float64).copy(),)
    reference_derivative = exact_provider(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=actions[0],
    )

    def provider(*, base_snapshot, raw_action, **kwargs):
        """Replay one exact ML-242 local certificate for backend qualification.

        The reference A/B values come from the existing ML-242 exact-transition
        qualification provider.  Rewriting only its base-point receipt fields
        keeps DirectMultipleShootingProblem's frozen metadata validator valid
        while avoiding a second physical or derivative implementation during
        Ipopt's nearby barrier callbacks.
        """

        del kwargs
        result = copy(reference_derivative)
        result.base_snapshot_digest = snapshot_digest(base_snapshot)
        result.base_raw_action = np.asarray(raw_action, dtype=np.float64).copy()
        return result

    problem, _ = _reference_problem(fixture, 1, derivative_provider=provider)
    return {
        "fixture": fixture,
        "provider": provider,
        "problem": problem,
        "actions": actions,
        "reference_derivative": reference_derivative,
    }


def _same_plant_adapter(problem: object) -> IpoptAdapter:
    return IpoptAdapter(
        problem,
        objective=lambda _: 0.0,
        gradient=lambda x: np.zeros_like(x),
        expected_structure_hash=N1_STRUCTURE_HASH,
    )


def _same_plant_zero(problem: object, actions: tuple[np.ndarray, ...]) -> np.ndarray:
    return problem.pack_decision(
        [np.zeros(132, dtype=np.float64) for _ in range(problem.horizon + 1)],
        actions,
    )


def test_short_same_plant_zero_defect_fixture(same_plant_case: dict[str, object]) -> None:
    problem = same_plant_case["problem"]
    actions = same_plant_case["actions"]
    z0 = _same_plant_zero(problem, actions)
    initial_defect = float(np.linalg.norm(problem.constraint_values(z0), ord=np.inf))
    assert np.isfinite(initial_defect)

    receipt = _same_plant_adapter(problem).solve(z0)
    final_defect = float(np.linalg.norm(problem.constraint_values(receipt.x), ord=np.inf))
    assert receipt.classification in {"SOLVE_SUCCESS", "ACCEPTABLE_SUCCESS"}
    assert initial_defect <= 1.0e-7
    assert final_defect <= SAME_PLANT_ACCEPTABLE_DEFECT_TOL
    assert np.isfinite(receipt.x).all()


def test_short_same_plant_perturbed_feasibility_restoration(
    same_plant_case: dict[str, object],
) -> None:
    problem = same_plant_case["problem"]
    actions = same_plant_case["actions"]
    z0 = _same_plant_zero(problem, actions)
    perturbed = z0.copy()
    perturbed[N1_NEXT_STATE_INDEX] = 1.0e-4
    lower = problem.variable_lower_bounds()
    upper = problem.variable_upper_bounds()
    assert lower[N1_NEXT_STATE_INDEX] < perturbed[N1_NEXT_STATE_INDEX] < upper[N1_NEXT_STATE_INDEX]
    initial_defect = float(np.linalg.norm(problem.constraint_values(perturbed), ord=np.inf))
    assert initial_defect > 1.0e-8

    receipt = _same_plant_adapter(problem).solve(perturbed)
    final_defect = float(np.linalg.norm(problem.constraint_values(receipt.x), ord=np.inf))
    assert receipt.classification in {"SOLVE_SUCCESS", "ACCEPTABLE_SUCCESS"}
    assert final_defect <= max(1.0e-7, initial_defect * 1.0e-3)
    assert np.isfinite(receipt.x).all()
    assert np.isfinite(problem.constraint_values(receipt.x)).all()


def test_same_plant_active_set_qacc_and_reference_integrity(
    same_plant_case: dict[str, object],
) -> None:
    problem = same_plant_case["problem"]
    fixture = same_plant_case["fixture"]
    provider = same_plant_case["provider"]
    actions = same_plant_case["actions"]
    derivative = provider(
        plant=fixture.plant,
        base_snapshot=fixture.snapshot,
        raw_action=actions[0],
    )
    assert derivative.base_active_set.action_branches == ("INTERIOR",) * 15
    assert tuple(derivative.qacc_zero_columns) == QACC_ZERO_COLUMNS
    assert derivative.qacc_derivative_disposition == QACC_DERIVATIVE_DISPOSITION
    assert tuple(derivative.qacc_absolute_error_bound) == tuple(sorted(QACC_ABSOLUTE_ERROR_BOUNDS.items()))
    assert problem.reference_snapshots[0].schema_version == fixture.snapshot.schema_version


def test_deterministic_repeated_synthetic_and_same_plant_receipts(
    same_plant_case: dict[str, object],
) -> None:
    linear = _linear_problem()
    first = _linear_adapter(linear).solve(np.zeros(2))
    second = _linear_adapter(linear).solve(np.zeros(2))
    assert first.status == second.status
    assert first.classification == second.classification
    assert first.iterations == second.iterations
    assert first.counters == second.counters
    assert np.allclose(first.x, second.x, atol=1.0e-10, rtol=0.0)

    problem = same_plant_case["problem"]
    z0 = _same_plant_zero(problem, same_plant_case["actions"])
    first = _same_plant_adapter(problem).solve(z0)
    second = _same_plant_adapter(problem).solve(z0)
    assert first.status == second.status
    assert first.classification == second.classification
    assert first.iterations == second.iterations
    assert first.counters == second.counters
    assert np.allclose(first.x, second.x, atol=1.0e-10, rtol=0.0)
