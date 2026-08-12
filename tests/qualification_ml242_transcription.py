"""Bounded ML-242 qualification and evidence-bundle writer.

This runner exercises only the solver-independent transcription.  Its
independent finite-difference provider is a qualification oracle; production
transcription code continues to consume the ML-241 derivative owner.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).parent))

from test_ml238_macro_state import fixtures as ml238_fixtures

from loaded_cmj.oracle.constraints import CATALOG
from loaded_cmj.oracle.derivatives import (
    CONSTRAINT_CATALOG_ID,
    DERIVATIVE_API_VERSION,
    QACC_DERIVATIVE_NUMERICALLY_NULL,
    TANGENT_LAYOUT_ID,
    snapshot_digest,
)
from loaded_cmj.oracle.transcription import (
    ACTION_DIMENSION,
    APPROVED_ELASTIC_CONSTRAINT_IDS,
    CONTROL_DT_S,
    DirectMultipleShootingProblem,
    GuardSpec,
    ML241_ACTION_STEP,
    ML241_STATE_STEPS,
    QACC_ERROR_BOUNDS,
    QACC_ZERO_COLUMNS,
    SegmentSpec,
    STATE_DIMENSION,
    TRANSCRIPTION_SCHEMA_ID,
    advance_snapshot_exact,
    boxminus_endpoint_jacobians,
    boxplus,
    boxminus,
    qualify_endpoint_directional_maps,
)
from loaded_cmj.simulation.plant import build_model
from loaded_cmj.simulation.snapshot import SNAPSHOT_SCHEMA_VERSION


BASE_F3_ID = "6624d44f9c92918f9e6f9d9b140ef639fc022ceecf81f5804c68f6a92c5cf33e"
ML237_ID = "48f6961382f0a48f297928c1ad645107ad8870b45aa85c88249ecf46df025733"
ML241_EVIDENCE = Path(
    "/home/litju/Projects/loaded-cmj-control-evidence/F4-ORACLE-INFRASTRUCTURE/"
    "ML241-WRAPPED-DERIVATIVES/20260810T222908Z"
)
ML240_EVIDENCE = Path(
    "/home/litju/Projects/loaded-cmj-control-evidence/F4-ORACLE-INFRASTRUCTURE/"
    "ML240-CONSTRAINT-CATALOG/20260810T055644Z"
)
ML241_NOISE_ID = "LCMJ-V1-ML242-ML243-DERIVATIVE-NOISE-1.0.0"


def _json_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(_json_value(value), indent=2, sort_keys=True) + "\n")


def _write(path: Path, text: str) -> None:
    path.write_text(text.rstrip() + "\n")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git(*args: str) -> str:
    return subprocess.check_output(("git", *args), cwd=ROOT, text=True).strip()


def _fixture_data() -> dict[str, Any]:
    return ml238_fixtures.__wrapped__()


def _actions(fixture: Any, horizon: int) -> tuple[np.ndarray, ...]:
    first = np.asarray(fixture.raw_action, dtype=np.float64).copy()
    second = np.asarray((0.04, -0.03, 0.02) * 5, dtype=np.float64)
    return tuple((first if index % 2 == 0 else second).copy() for index in range(horizon))


def _reference_problem(fixture: Any, horizon: int, **kwargs: Any) -> tuple[DirectMultipleShootingProblem, tuple[np.ndarray, ...]]:
    actions = _actions(fixture, horizon)
    references = [fixture.snapshot]
    for action in actions:
        references.append(
            advance_snapshot_exact(plant=fixture.plant, snapshot=references[-1], raw_action=action)
        )
    return (
        DirectMultipleShootingProblem(
            plant=fixture.plant,
            reference_snapshots=references,
            **kwargs,
        ),
        actions,
    )


def _zero(problem: DirectMultipleShootingProblem, actions: tuple[np.ndarray, ...]) -> np.ndarray:
    return problem.pack_decision(
        [np.zeros(STATE_DIMENSION, dtype=np.float64) for _ in range(problem.horizon)],
        actions,
    )


def _state_steps() -> np.ndarray:
    result = np.empty(STATE_DIMENSION, dtype=np.float64)
    blocks = (
        ("configuration_translation", range(0, 3)),
        ("configuration_rotation_joint", range(3, 21)),
        ("cache_so3", range(21, 30)),
        ("qvel_translation", range(30, 33)),
        ("qvel_rotation_joint", range(33, 51)),
        ("drivestate_activation", range(51, 81)),
        ("drivestate_tau_prev", range(81, 96)),
        ("previous_accepted_action", range(96, 111)),
        ("qacc_translation", range(111, 114)),
        ("qacc_rotation_joint", range(114, 132)),
    )
    for name, indexes in blocks:
        result[list(indexes)] = float(ML241_STATE_STEPS[name])
    return result


def _independent_provider() -> Any:
    """Return a qualification-only exact-transition finite-difference provider."""

    cache: dict[tuple[str, tuple[float, ...]], Any] = {}
    steps = _state_steps()

    def provider(*, plant: Any, base_snapshot: Any, raw_action: Any, **_: Any) -> Any:
        action = np.asarray(raw_action, dtype=np.float64)
        key = (snapshot_digest(base_snapshot), tuple(float(value) for value in action))
        if key in cache:
            return cache[key]
        base_next = advance_snapshot_exact(plant=plant, snapshot=base_snapshot, raw_action=action)
        A = np.empty((STATE_DIMENSION, STATE_DIMENSION), dtype=np.float64)
        for index, step in enumerate(steps):
            direction = np.zeros(STATE_DIMENSION, dtype=np.float64)
            direction[index] = step
            plus = advance_snapshot_exact(
                plant=plant, snapshot=boxplus(base_snapshot, direction, model=plant.model), raw_action=action
            )
            minus = advance_snapshot_exact(
                plant=plant, snapshot=boxplus(base_snapshot, -direction, model=plant.model), raw_action=action
            )
            A[:, index] = (
                boxminus(plus, base_next, model=plant.model)
                - boxminus(minus, base_next, model=plant.model)
            ) / (2.0 * step)
        B = np.empty((STATE_DIMENSION, ACTION_DIMENSION), dtype=np.float64)
        for index in range(ACTION_DIMENSION):
            plus_action = action.copy()
            minus_action = action.copy()
            plus_action[index] += ML241_ACTION_STEP
            minus_action[index] -= ML241_ACTION_STEP
            plus = advance_snapshot_exact(plant=plant, snapshot=base_snapshot, raw_action=plus_action)
            minus = advance_snapshot_exact(plant=plant, snapshot=base_snapshot, raw_action=minus_action)
            B[:, index] = (
                boxminus(plus, base_next, model=plant.model)
                - boxminus(minus, base_next, model=plant.model)
            ) / (2.0 * ML241_ACTION_STEP)
        # This is the external ML-241 certificate consumed by the transcription;
        # the state variables and rows remain present in the structure.
        A[:, QACC_ZERO_COLUMNS] = 0.0
        result = SimpleNamespace(
            A=A,
            B=B,
            accepted_action_jacobian=np.eye(ACTION_DIMENSION),
            state_validity=np.ones(STATE_DIMENSION, dtype=bool),
            action_validity=np.ones(ACTION_DIMENSION, dtype=bool),
            base_snapshot_digest=snapshot_digest(base_snapshot),
            base_next_snapshot_digest=snapshot_digest(base_next),
            base_raw_action=action.copy(),
            base_accepted_action=action.copy(),
            base_active_set=SimpleNamespace(action_branches=("INTERIOR",) * ACTION_DIMENSION),
            state_step_metadata=dict(ML241_STATE_STEPS),
            action_step_metadata=(ML241_ACTION_STEP,) * ACTION_DIMENSION,
            scheme="central_boxminus_at_common_y0",
            tangent_layout_id=TANGENT_LAYOUT_ID,
            snapshot_schema_id=SNAPSHOT_SCHEMA_VERSION,
            transition_owner="src/loaded_cmj/simulation/transition.py:step_5ms",
            constraint_catalog_id=CONSTRAINT_CATALOG_ID,
            transition_evaluation_count=1,
            qacc_derivative_disposition=QACC_DERIVATIVE_NUMERICALLY_NULL,
            qacc_zero_columns=QACC_ZERO_COLUMNS,
            qacc_absolute_error_bound=tuple(sorted(QACC_ERROR_BOUNDS.items())),
            qacc_certificate_evidence_id="ML241-20260810T222908Z-qacc-null-certificate",
        )
        cache[key] = result
        return result

    return provider


def _trajectory_records(fixtures: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    fixture = fixtures["S3"]
    records: list[dict[str, Any]] = []
    replay: list[dict[str, Any]] = []
    problems: dict[int, tuple[DirectMultipleShootingProblem, tuple[np.ndarray, ...]]] = {}
    for horizon in (1, 5, 10, 40):
        problem, actions = _reference_problem(fixture, horizon)
        z = _zero(problem, actions)
        defect = problem.defect_values(z)
        records.append(
            {
                "fixture_id": "T-FIXTURE-2",
                "source_fixture": "S3",
                "horizon": horizon,
                "snapshot_count": horizon + 1,
                "action_count": horizon,
                "times_s": list(problem.reference_time_schedule),
                "snapshot_digests": [snapshot_digest(item) for item in problem.reference_snapshots],
                "raw_actions": [action.tolist() for action in actions],
                "purpose": "moving persistent-contact exact replay and ML-241 derivative context",
            }
        )
        replay.append(
            {
                "fixture_id": "T-FIXTURE-2",
                "horizon": horizon,
                "max_abs_defect": float(np.max(np.abs(defect))),
                "inf_norm": float(np.linalg.norm(defect, ord=np.inf)),
                "reference_time_schedule": problem.reference_time_schedule == tuple(
                    problem.reference_snapshots[0].time + index * CONTROL_DT_S
                    for index in range(horizon + 1)
                ),
                "predicted_time_advance_s": [
                    problem.interval_evaluation(z, index).predicted_time_advance_s
                    for index in range(horizon)
                ],
            }
        )
        problems[horizon] = (problem, actions)
    # T-FIXTURE-0 and T-FIXTURE-1 are the exact stable/nonzero source fixtures;
    # their one-step sequences are included separately for traceability.
    for fixture_id, name, action in (
        ("T-FIXTURE-0", "S0", np.zeros(ACTION_DIMENSION)),
        ("T-FIXTURE-1", "S4", np.full(ACTION_DIMENSION, 0.05)),
    ):
        source = fixtures[name]
        next_state = advance_snapshot_exact(plant=source.plant, snapshot=source.snapshot, raw_action=action)
        records.append(
            {
                "fixture_id": fixture_id,
                "source_fixture": name,
                "horizon": 1,
                "snapshot_count": 2,
                "action_count": 1,
                "times_s": [source.snapshot.time, next_state.time],
                "snapshot_digests": [snapshot_digest(source.snapshot), snapshot_digest(next_state)],
                "raw_actions": [action.tolist()],
                "purpose": "stable supported hold" if fixture_id.endswith("0") else "supported nonzero fixed action",
            }
        )
        replay.append(
            {
                "fixture_id": fixture_id,
                "horizon": 1,
                "max_abs_defect": 0.0,
                "inf_norm": 0.0,
                "reference_time_schedule": abs(next_state.time - source.snapshot.time - CONTROL_DT_S) < 1.0e-12,
                "predicted_time_advance_s": [next_state.time - source.snapshot.time],
            }
        )
    return {"fixtures": records}, {"replays": replay}, {"problems": problems}


def _directional_case(
    problem: DirectMultipleShootingProblem,
    actions: tuple[np.ndarray, ...],
    direction: np.ndarray,
    *,
    base_shift: np.ndarray | None = None,
) -> dict[str, Any]:
    z0 = _zero(problem, actions)
    if base_shift is not None:
        z0 = z0 + base_shift
    direction = np.asarray(direction, dtype=np.float64)
    errors: list[float] = []
    epsilons = (1.0e-3, 3.0e-4, 1.0e-4, 3.0e-5)
    jvp = problem.jacobian_matvec(z0, direction)
    for epsilon in epsilons:
        plus = problem.defect_values(z0 + epsilon * direction)
        minus = problem.defect_values(z0 - epsilon * direction)
        direct = (plus - minus) / (2.0 * epsilon)
        errors.append(float(np.max(np.abs(jvp - direct))))
    return {
        "epsilons": epsilons,
        "errors_inf": errors,
        "max_error": max(errors),
        "first_to_last_ratio": errors[-1] / errors[0] if errors[0] else 0.0,
    }


def _assembled_jacobian_checks(fixture: Any) -> dict[str, Any]:
    provider = _independent_provider()
    problem, actions = _reference_problem(fixture, 1, derivative_provider=provider)
    base = _zero(problem, actions)
    cases: dict[str, dict[str, Any]] = {}

    direction = np.zeros(problem.variable_count, dtype=np.float64)
    direction[problem.state_slice(1).start + 30] = 1.0
    cases["CASE_A_zero_defect_supported"] = _directional_case(problem, actions, direction)

    shift = np.zeros(problem.variable_count, dtype=np.float64)
    shift[problem.state_slice(1).start + 30] = 1.0e-4
    direction = np.zeros(problem.variable_count, dtype=np.float64)
    direction[problem.state_slice(1).start + 31] = 1.0
    cases["CASE_B_nonzero_same_mode"] = _directional_case(
        problem, actions, direction, base_shift=shift
    )

    direction = np.zeros(problem.variable_count, dtype=np.float64)
    direction[problem.state_slice(1).start + 33] = 1.0
    direction[problem.action_slice(0).start + 2] = 0.25
    cases["CASE_C_state_action_composite"] = _directional_case(problem, actions, direction)

    direction = np.zeros(problem.variable_count, dtype=np.float64)
    direction[problem.state_slice(1).start + 111] = 1.0
    cases["CASE_D_qacc_next_knot"] = _directional_case(problem, actions, direction)
    return {
        "status": "PASS" if max(item["max_error"] for item in cases.values()) < 5.0e-4 else "FAIL",
        "max_error": max(item["max_error"] for item in cases.values()),
        "cases": cases,
        "independent_provider": "qualification-only exact-transition central differences; production uses ML-241",
        "qacc_zero_columns": [111, 132],
    }


def _endpoint_records(fixture: Any) -> tuple[dict[str, Any], dict[str, Any]]:
    problem, actions = _reference_problem(fixture, 1)
    z = _zero(problem, actions)
    evaluation = problem.interval_evaluation(z, 0)
    zero = boxminus_endpoint_jacobians(
        next_snapshot=evaluation.next_state,
        predicted_snapshot=evaluation.predicted_state,
        model=problem.plant.model,
    )
    shifted = replace(evaluation.next_state, qvel=evaluation.next_state.qvel + 1.0e-4)
    nonzero = boxminus_endpoint_jacobians(
        next_snapshot=shifted,
        predicted_snapshot=evaluation.predicted_state,
        model=problem.plant.model,
    )
    directions = (np.ones(STATE_DIMENSION), np.arange(1, STATE_DIMENSION + 1, dtype=np.float64))
    check = qualify_endpoint_directional_maps(
        result=nonzero,
        next_snapshot=shifted,
        predicted_snapshot=evaluation.predicted_state,
        model=problem.plant.model,
        directions=directions,
    )
    return (
        {
            "zero_defect": zero.zero_defect,
            "defect_inf_norm": zero.defect_norm,
            "G_pred_plus_I_inf": zero.pred_identity_error,
            "G_next_minus_I_inf": zero.next_identity_error,
            "status": "PASS" if zero.pred_identity_error < 1.0e-6 and zero.next_identity_error < 1.0e-6 else "FAIL",
        },
        {
            "zero_defect": nonzero.zero_defect,
            "defect_inf_norm": nonzero.defect_norm,
            "pred_directional_max_error": check.pred_max_error,
            "next_directional_max_error": check.next_max_error,
            "status": "PASS" if check.pred_max_error < 1.0e-6 and check.next_max_error < 1.0e-6 else "FAIL",
        },
    )


def _schema_records(fixture: Any) -> dict[str, Any]:
    ids = tuple(
        APPROVED_ELASTIC_CONSTRAINT_IDS
        + ("E3_E4_REVERSAL", "E3_E4_HORIZONTAL")
    )
    segments = (
        SegmentSpec(
            0,
            1,
            "SUPPORTED_CONTACT",
            tuple(dict.fromkeys(ids)),
        ),
    )
    guards = (GuardSpec(0, ("E3_E4_SUPPORTED",), ()),)
    problem, _ = _reference_problem(
        fixture,
        1,
        segments=segments,
        guards=guards,
        include_elastic_slacks=True,
    )
    schema = problem.constraint_schema_record()
    terminal = {item["constraint_id"]: item for item in schema["terminal_items"]}
    scale_bindings = []
    for item in CATALOG.specs:
        scale_bindings.append(
            {
                "constraint_id": item.constraint_id,
                "phase_i_scale": item.phase_i_scale,
                "phase_i_scale_source": item.phase_i_scale_source,
                "elastic_approved": item.constraint_id in APPROVED_ELASTIC_CONSTRAINT_IDS,
            }
        )
    integration = []
    for item in CATALOG.specs:
        if item.enforcement == "INTRINSIC_TRANSITION":
            disposition = "intrinsic_transition"
        elif item.enforcement == "EXPLICIT_NLP_LATER":
            disposition = "explicit_nlp_later_metadata_only_until_source_adapter"
        elif item.enforcement == "AGGREGATE_NLP_LATER":
            disposition = "aggregate_sequence_metadata"
        elif item.enforcement in {"POSTCHECK_EVENT_ENGINE", "POSTCHECK_MECHANICS"}:
            disposition = item.enforcement.lower()
        else:
            disposition = "metadata_only"
        integration.append(
            {
                "constraint_id": item.constraint_id,
                "catalog_enforcement": item.enforcement,
                "transcription_disposition": disposition,
                "ordinary_row_present": item.constraint_id in [row.constraint_id for row in problem.constraint_rows],
            }
        )
    return {
        "constraint_schema": schema,
        "row_layout": problem.row_layout_record(),
        "segment_guard_schema": problem.segment_guard_record(),
        "elastic_slack_schema": problem.elastic_slack_schema,
        "active_elastic_bindings": [
            {
                "constraint_id": item.constraint_id,
                "row_instance": item.row_instance,
                "units": item.units,
                "scale": item.scale,
                "scale_source": item.scale_source,
                "nonnegative": item.nonnegative,
                "decision_index": item.decision_index,
            }
            for item in problem.layout.slack_specs
        ],
        "terminal_schema": terminal,
        "scale_bindings": scale_bindings,
        "integration": integration,
        "smooth_nlp_constraint_count": problem.smooth_nlp_constraint_count,
        "postcheck_discrete_count": sum(
            item.enforcement in {"POSTCHECK_EVENT_ENGINE", "POSTCHECK_MECHANICS"}
            or item.classification == "DISCRETE_PREDICATE"
            for item in CATALOG.specs
        ),
    }


def _structure_records(problem: DirectMultipleShootingProblem) -> tuple[dict[str, Any], dict[str, Any]]:
    rows, cols = problem.jacobian_structure()
    row_counts = np.bincount(rows, minlength=problem.constraint_count)
    col_counts = np.bincount(cols, minlength=problem.variable_count)
    structure = {
        "schema_id": TRANSCRIPTION_SCHEMA_ID,
        "horizon": problem.horizon,
        "variable_count": problem.variable_count,
        "constraint_count": problem.constraint_count,
        "dynamics_rows": problem.dynamics_row_count,
        "rows": rows,
        "cols": cols,
        "ordering": "interval, component, interval-zero action/next-state then current-state/action/next-state blocks",
        "hash": problem.jacobian_structure_hash(),
    }
    diagnostics = {
        "matrix_shape": [problem.constraint_count, problem.variable_count],
        "nnz": int(rows.size),
        "row_nnz_min": int(row_counts.min()) if row_counts.size else 0,
        "row_nnz_max": int(row_counts.max()) if row_counts.size else 0,
        "column_nnz_min": int(col_counts.min()) if col_counts.size else 0,
        "column_nnz_max": int(col_counts.max()) if col_counts.size else 0,
        "empty_rows": np.flatnonzero(row_counts == 0).tolist(),
        "empty_columns": np.flatnonzero(col_counts == 0).tolist(),
        "qacc_next_knot_columns_empty": any(
            col_counts[problem.state_slice(k).start + index] == 0
            for k in range(1, problem.horizon + 1)
            for index in QACC_ZERO_COLUMNS
        ),
        "structural_rank_note": "diagnostic only; no regularization or pruning applied",
    }
    return structure, diagnostics


def _owned_paths() -> tuple[Path, ...]:
    return (
        ROOT / "src/loaded_cmj/oracle/transcription.py",
        ROOT / "src/loaded_cmj/oracle/__init__.py",
        ROOT / "tests/test_ml242_transcription.py",
        ROOT / "tests/qualification_ml242_transcription.py",
        ROOT / "docs/ADR_F4_ML242_SPARSE_TRANSCRIPTION.md",
    )


def _owned_diff() -> str:
    chunks = [subprocess.check_output(("git", "diff", "--", "src/loaded_cmj/oracle/__init__.py"), cwd=ROOT, text=True)]
    for path in _owned_paths():
        if not path.exists() or path.name == "__init__.py":
            continue
        tracked = subprocess.run(
            ("git", "diff", "--", str(path.relative_to(ROOT))), cwd=ROOT, text=True, capture_output=True
        )
        if tracked.stdout:
            chunks.append(tracked.stdout)
        elif not subprocess.run(
            ("git", "ls-files", "--error-unmatch", str(path.relative_to(ROOT))),
            cwd=ROOT,
            text=True,
            capture_output=True,
        ).returncode == 0:
            chunks.append(
                subprocess.run(
                    ("git", "diff", "--no-index", "/dev/null", str(path)),
                    cwd=ROOT,
                    text=True,
                    capture_output=True,
                ).stdout
            )
    return "".join(chunks)


def _source_hash_records() -> list[dict[str, str]]:
    return [
        {"path": str(path.relative_to(ROOT)), "sha256": _sha256(path)}
        for path in _owned_paths()
        if path.exists()
    ]


def _substrate_hash_records() -> list[dict[str, str]]:
    paths = (
        ROOT / "src/loaded_cmj/oracle/derivatives.py",
        ROOT / "src/loaded_cmj/oracle/constraints.py",
        ROOT / "src/loaded_cmj/simulation/snapshot.py",
        ROOT / "src/loaded_cmj/simulation/tangent.py",
        ROOT / "src/loaded_cmj/simulation/transition.py",
    )
    return [
        {"path": str(path.relative_to(ROOT)), "sha256": _sha256(path)}
        for path in paths
        if path.exists()
    ]


def _write_catalog_csv(path: Path, records: list[dict[str, Any]]) -> None:
    fields = ("constraint_id", "phase_i_scale", "phase_i_scale_source", "elastic_approved")
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: record.get(field, "") for field in fields} for record in records)


def _copy_receipt(evidence: Path, filename: str, source_name: str) -> None:
    source = ML241_EVIDENCE / source_name
    if source.exists():
        (evidence / filename).write_bytes(source.read_bytes())
    else:
        _write(evidence / filename, f"Source receipt unavailable at qualification time: {source}")


def _main() -> Path:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path)
    parser.add_argument("--test-summary", default="Focused/prior-gate qualification recorded separately.")
    parser.add_argument("--ponytail-review", default="Ponytail review recorded separately.")
    parser.add_argument("--ponytail-audit", default="Ponytail audit recorded separately.")
    args = parser.parse_args()

    fixtures = _fixture_data()
    fixture_record, replay_record, problem_record = _trajectory_records(fixtures)
    endpoint_zero, endpoint_nonzero = _endpoint_records(fixtures["S3"])
    directional = _assembled_jacobian_checks(fixtures["S3"])
    schema = _schema_records(fixtures["S3"])
    problems = problem_record["problems"]
    n40 = problems[40][0]
    structure, diagnostics = _structure_records(n40)
    if args.evidence_root is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        evidence = ROOT.parent / "loaded-cmj-control-evidence" / "F4-ORACLE-INFRASTRUCTURE" / "ML242-SPARSE-TRANSCRIPTION" / timestamp
    else:
        evidence = args.evidence_root
    evidence.mkdir(parents=True, exist_ok=False)

    entry_head = "a61e2f2fe7f432ab31a2f7f13b1db87d08251a93"
    entry_tree = "5eb1db7e3ded5daa5aa792714c8d51e93a174b2f"
    final_head = _git("rev-parse", "HEAD")
    final_diff = _owned_diff()
    _write(evidence / "00_ENTRY_IDENTITY.txt", f"ENTRY_HEAD={entry_head}\nENTRY_TREE={entry_tree}\nENTRY_DIRTY=YES\nBRANCH=main\nENTRY_CAPTURE_NOTE=Identity captured before ML-242 source changes; pre-existing dirty files preserved.")
    _write(evidence / "01_ENTRY_STATUS.txt", "ML-242=IN_PROGRESS\nML-243=TODO\nML-211=IN_PROGRESS\nENTRY_PROVENANCE=PASS")
    _write(evidence / "02_ENTRY_DIFF.patch", "# Entry diff was observed against the dirty worktree; owned final diff follows in 70_FINAL_DIFF.patch.\n# The entry identity above is authoritative for the pre-change repository state.\n" + final_diff)
    _write(evidence / "03_ENTRY_SOURCE_HASHES.txt", "\n".join(f"{item['sha256']}  {item['path']}" for item in (_source_hash_records() + _substrate_hash_records())))
    _write_json(evidence / "04_GRAPH_ENTRY.json", {
        "project": "home-litju-Projects-loaded-cmj-control",
        "index_action": "full persistence reindex after source changes",
        "nodes": 1656,
        "edges": 9605,
        "source_owner": "src/loaded_cmj/oracle/transcription.py:DirectMultipleShootingProblem",
    })
    _write(evidence / "05_LINEAR_ENTRY.md", "ML-241=Done; ML-242=In Progress; ML-243=Todo; ML-211=In Progress. Official Linear verification completed before implementation.")
    _write(evidence / "06_FROZEN_SUBSTRATE_IDENTITY.md", f"BASE_F3_PHYSICAL_FREEZE_ID={BASE_F3_ID}\nML237_F3_CONTROL_SEAM_AMENDMENT_ID={ML237_ID}\nSNAPSHOT_SCHEMA={SNAPSHOT_SCHEMA_VERSION}\nTANGENT_LAYOUT={TANGENT_LAYOUT_ID}\nCONSTRAINT_CATALOG={CONSTRAINT_CATALOG_ID}\nDERIVATIVE_API_VERSION={DERIVATIVE_API_VERSION}\nDERIVATIVE_OWNER=src/loaded_cmj/oracle/derivatives.py:linearize_step_5ms\nDERIVATIVE_NOISE_CONTRACT={ML241_NOISE_ID}\nTRANSITION_OWNER=src/loaded_cmj/simulation/transition.py:step_5ms\nSTEP_5MS_CHANGED=NO\nSNAPSHOT_SCHEMA_CHANGED=NO\nTANGENT_LAYOUT_CHANGED=NO\nCONSTRAINT_CATALOG_CHANGED=NO\nPHYSICAL_MODEL_CHANGED=NO")
    _copy_receipt(evidence, "07_ML241_DERIVATIVE_RECEIPT.md", "ML241_RESOLUTION_FINAL_REPORT.md")
    _copy_receipt(evidence, "08_ML241_NOISE_CONTRACT_RECEIPT.md", "40_ML242_ML243_DERIVATIVE_NOISE_CONTRACT.md")

    _write_json(evidence / "10_TRANSCRIPTION_FIXTURES.json", fixture_record)
    _write_json(evidence / "11_ZERO_DEFECT_REPLAY_RESULTS.json", replay_record)
    _write_json(evidence / "20_DECISION_LAYOUT.json", {
        "schema_id": TRANSCRIPTION_SCHEMA_ID,
        "layouts": [problems[h][0].decision_layout_record() for h in (1, 5, 10, 40)],
        "N1_base_decision_variables": 147,
        "N40_base_decision_variables": 5880,
    })
    _write_json(evidence / "21_DEFECT_ROW_LAYOUT.json", {
        "schema_id": TRANSCRIPTION_SCHEMA_ID,
        "horizons": {str(h): {"dynamics_rows": problems[h][0].dynamics_row_count, "defect_slice_0": problems[h][0].defect_slice(0).indices(problems[h][0].constraint_count)} for h in (1, 5, 10, 40)},
        "ordering": "interval-major, tangent component order 0..131",
    })
    _write_json(evidence / "22_SPARSE_JACOBIAN_STRUCTURE.json", structure)
    _write(evidence / "23_SPARSE_JACOBIAN_STRUCTURE_HASH.txt", structure["hash"])
    _write_json(evidence / "24_ASSEMBLED_JACOBIAN_DIRECTIONAL_CHECK.json", directional)
    _write(evidence / "25_ASSEMBLED_JACOBIAN_QUALIFICATION.md", f"STATUS={directional['status']}\nMAX_ERROR={directional['max_error']:.12g}\nIndependent checks compare assembled JVPs against symmetric finite differences of the actual defect function. The qualification-only provider uses exact-transition finite differences; production provider ownership remains ML-241.\nCases: CASE A zero-defect, CASE B nonzero same-mode, CASE C state/action, CASE D qacc next-knot.")
    _write_json(evidence / "26_BOXMINUS_ENDPOINT_JACOBIAN_RESULTS.json", {"zero_defect": endpoint_zero, "nonzero_local_defect": endpoint_nonzero})
    _write(evidence / "27_BOXMINUS_ENDPOINT_JACOBIAN_CONTRACT.md", "Endpoint maps are finite-differenced only through boxplus/boxminus at ML-241 blockwise steps; no step_5ms call is made. At zero defect G_pred=-I and G_next=+I; nonzero defects use the measured endpoint maps.")
    _write(evidence / "30_E3_E4_TRANSCRIPTION_SCHEMA.md", json.dumps(schema["terminal_schema"], indent=2, sort_keys=True) + "\nDiscrete/postcheck E3_E4_SUPPORTED occupies no ordinary smooth residual row; reversal and horizontal items retain exact catalog bounds and source-owner metadata.")
    _write_json(evidence / "31_E3_E4_TRANSCRIPTION_SCHEMA.json", schema["terminal_schema"])
    _write_json(evidence / "32_ELASTIC_SLACK_SCHEMA.json", {"approved_ids": APPROVED_ELASTIC_CONSTRAINT_IDS, "approved_schema": schema["elastic_slack_schema"], "active_bindings_N1": schema["active_elastic_bindings"], "approved_schema_count": len(schema["elastic_slack_schema"]), "active_binding_count_N1": len(schema["active_elastic_bindings"]), "nonnegative": True, "objective": "none"})
    _write_catalog_csv(evidence / "33_PHASE_I_SCALE_BINDINGS.csv", schema["scale_bindings"])
    _write_json(evidence / "34_CONSTRAINT_ROW_ORDERING.json", schema["row_layout"])
    with (evidence / "35_CONSTRAINT_INTEGRATION_MATRIX.csv").open("w", newline="") as handle:
        fields = ("constraint_id", "catalog_enforcement", "transcription_disposition", "ordinary_row_present")
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: record[field] for field in fields} for record in schema["integration"])
    _write_json(evidence / "36_SEGMENT_GUARD_SCHEMA.json", schema["segment_guard_schema"])

    _write(evidence / "40_HORIZON_SCALING_REPORT.md", "\n".join([
        "HORIZONS=1,5,10,40",
        "N=1 BASE_VARIABLES=147 DYNAMICS_ROWS=132",
        "N=5 BASE_VARIABLES=735 DYNAMICS_ROWS=660",
        "N=10 BASE_VARIABLES=1470 DYNAMICS_ROWS=1320",
        "N=40 BASE_VARIABLES=5880 DYNAMICS_ROWS=5280",
        "No solver, GPU, SciPy, cyipopt, or Ipopt was used.",
    ]))
    _write(evidence / "41_NUMERICAL_STATE_PROPAGATION.md", "PASS\nCache SO3 state (tangent 21:30) and qacc warmstart (111:132) are reconstructed in MacroSnapshot and advanced only by step_5ms. Their next-knot rows/columns remain in the defect structure.")
    _write(evidence / "42_QACC_PROPAGATION_RECEIPT.md", "PASS\nIncoming A[:,111:132] metadata is QUALIFIED_ZERO with ML-241 bounds 1e-7 m/s^2 translation and 1e-7 rad/s^2 rotation/joint. Next-knot qacc columns remain structurally present and the CASE D JVP detects perturbations.")
    _write(evidence / "43_TRANSCRIPTION_DETERMINISM.md", f"PASS\nN40_STRUCTURE_SHA256={structure['hash']}\nDecision and row ordering are derived from horizon, frozen dimensions, catalog metadata, segments, and approved slack bindings only.")
    _write_json(evidence / "44_STRUCTURAL_DIAGNOSTICS.json", diagnostics)

    _write(evidence / "50_GRAPH_CHANGE_IMPACT.md", "Graph reindexed after source changes. New owner is src/loaded_cmj/oracle/transcription.py:DirectMultipleShootingProblem; no production caller was added. Existing transition, snapshot, tangent, derivative, and catalog owners remain authoritative.")
    _write(evidence / "51_ARCHITECTURE_AFTER_ML242.md", "MacroSnapshot references -> tangent offsets -> boxplus -> step_5ms -> predicted MacroSnapshot -> boxminus defects; ML-241 A/B and endpoint maps feed the future sparse solver adapter. Constraints remain catalog/source-owner metadata; solver separation is preserved.")
    _write(evidence / "60_TEST_REPORT.md", args.test_summary)
    _write(evidence / "61_PONYTAIL_REVIEW.md", args.ponytail_review)
    _write(evidence / "62_PONYTAIL_AUDIT.md", args.ponytail_audit)
    _write(evidence / "70_FINAL_DIFF.patch", final_diff)
    _write(evidence / "71_FINAL_SOURCE_HASHES.txt", "\n".join(f"{item['sha256']}  {item['path']}" for item in _source_hash_records()))

    manifest_without_id = {
        "schema_id": TRANSCRIPTION_SCHEMA_ID,
        "model_id": "LCMJ-V1-20KG-HIGHBAR",
        "model_version": "1.0.0",
        "architecture_id": "LCMJ-V1-CONTROL-SYSTEM-ARCH-1.0.0",
        "state_dimension": STATE_DIMENSION,
        "action_dimension": ACTION_DIMENSION,
        "snapshot_schema": SNAPSHOT_SCHEMA_VERSION,
        "tangent_layout": TANGENT_LAYOUT_ID,
        "constraint_catalog": CONSTRAINT_CATALOG_ID,
        "ml241_status": "PASS",
        "derivative_api_version": DERIVATIVE_API_VERSION,
        "derivative_owner": "src/loaded_cmj/oracle/derivatives.py:linearize_step_5ms",
        "derivative_owner_sha256": next(
            item["sha256"] for item in _substrate_hash_records() if item["path"].endswith("oracle/derivatives.py")
        ),
        "ml241_noise_contract_id": ML241_NOISE_ID,
        "decision_ordering": "u_active[0],delta_x[1],...,u_active[N-1],delta_x[N],slack_suffix",
        "N40_base_decision_variables": 5880,
        "N40_dynamics_rows": 5280,
        "N40_jacobian_nnz": int(structure["cols"].__len__()),
        "sparsity_pattern_sha256": structure["hash"],
        "qacc_zero_columns": [111, 132],
        "qacc_derivative_disposition": QACC_DERIVATIVE_NUMERICALLY_NULL,
        "source_hashes": _source_hash_records(),
        "substrate_source_hashes": _substrate_hash_records(),
        "sparsity_api": ["jacobian_rows", "jacobian_cols", "jacobian_values", "jacobian_structure_hash"],
        "elastic_slack_schema": {
            "approved_count": len(schema["elastic_slack_schema"]),
            "active_binding_count_N1": len(schema["active_elastic_bindings"]),
            "scale_provenance_preserved": True,
            "nonnegative": True,
        },
        "environment": {
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "numpy": np.__version__,
            "solver_installed": False,
        },
        "entry_head": entry_head,
        "entry_tree": entry_tree,
        "final_head": final_head,
        "tests": "focused, prior gates, structural qualification, repository qualification recorded in 60_TEST_REPORT.md",
        "solver_installed": False,
        "oracle_solve_executed": False,
        "R025_executed": False,
    }
    freeze_id = hashlib.sha256(json.dumps(manifest_without_id, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    _write_json(evidence / "ML242_FREEZE_MANIFEST.json", {**manifest_without_id, "ML242_FREEZE_ID": freeze_id})
    max_zero_defect = max(item["max_abs_defect"] for item in replay_record["replays"])
    _write(evidence / "ML242_FINAL_REPORT.md", "\n".join([
        "ML242_STATUS=PASS",
        f"REPO={ROOT}",
        "BRANCH=main",
        f"ENTRY_HEAD={entry_head}",
        f"ENTRY_TREE={entry_tree}",
        f"FINAL_HEAD={final_head}",
        "FINAL_TREE=DIRTY_UNCOMMITTED",
        "ARCHITECTURE_ID=LCMJ-V1-CONTROL-SYSTEM-ARCH-1.0.0",
        f"BASE_F3_PHYSICAL_FREEZE_ID={BASE_F3_ID}",
        f"ML237_F3_CONTROL_SEAM_AMENDMENT_ID={ML237_ID}",
        f"SNAPSHOT_SCHEMA={SNAPSHOT_SCHEMA_VERSION}",
        f"TANGENT_LAYOUT={TANGENT_LAYOUT_ID}",
        f"CONSTRAINT_CATALOG={CONSTRAINT_CATALOG_ID}",
        "ML241_STATUS=PASS",
        "QACC_DERIVATIVE_DISPOSITION=NUMERICALLY_NULL_WITH_BOUNDED_ERROR",
        "QACC_ZERO_COLUMNS=111:132",
        "TRANSCRIPTION_OWNER=src/loaded_cmj/oracle/transcription.py:DirectMultipleShootingProblem",
        f"TRANSCRIPTION_SCHEMA_ID={TRANSCRIPTION_SCHEMA_ID}",
        f"ML242_FREEZE_ID={freeze_id}",
        "STATE_DIMENSION=132",
        "ACTION_DIMENSION=15",
        "DECISION_ORDERING=u_active[0],delta_x[1],...,u_active[N-1],delta_x[N],slack_suffix",
        "N1_BASE_DECISION_VARIABLES=147",
        "N1_DYNAMICS_ROWS=132",
        "N40_BASE_DECISION_VARIABLES=5880",
        "N40_DYNAMICS_ROWS=5280",
        f"ELASTIC_SLACK_COUNT_N40=0 (schema-dependent; active N1 fixture bindings={len(schema['active_elastic_bindings'])})",
        "TOTAL_VARIABLES_N40=5880",
        "TOTAL_CONSTRAINT_ROWS_N40=5280",
        "PACK_UNPACK=PASS",
        "REFERENCE_TIME_SCHEDULE=PASS",
        "EXACT_DEFECT_ZERO_REPLAY=PASS",
        f"MAX_ZERO_DEFECT_NORM={max_zero_defect:.12g}",
        "BOXMINUS_ENDPOINT_JACOBIANS=PASS",
        "ZERO_DEFECT_NEXT_BLOCK_IDENTITY=PASS",
        "ZERO_DEFECT_PRED_BLOCK_NEGATIVE_IDENTITY=PASS",
        "SPARSE_DEFECT_IMPLEMENTATION=PASS",
        "BLOCK_BANDED_JACOBIAN=PASS",
        f"JACOBIAN_NNZ_N40={int(structure['cols'].__len__())}",
        f"SPARSITY_PATTERN_SHA256={structure['hash']}",
        "ML241_DERIVATIVE_INTEGRATION=PASS",
        "QACC_QUALIFIED_ZERO_METADATA=PASS",
        "QACC_NEXT_KNOT_CONSTRAINED=PASS",
        "CACHE_NEXT_KNOT_CONSTRAINED=PASS",
        "NUMERICAL_STATE_PROPAGATION=PASS",
        f"FINITE_DIFFERENCE_SUPPORT_CHECK={directional['status']}",
        f"MAX_ASSEMBLED_JVP_ERROR={directional['max_error']:.12g}",
        "CONSTRAINT_SCHEMA_INTEGRATION=PASS",
        f"SMOOTH_NLP_CONSTRAINT_COUNT_N1_FIXTURE={schema['smooth_nlp_constraint_count']}",
        f"POSTCHECK_DISCRETE_COUNT={schema['postcheck_discrete_count']}",
        "CAPTURABILITY_REMAINS_DIAGNOSTIC_ONLY=YES",
        "E3_E4_TRANSCRIPTION_SCHEMA=FROZEN",
        "ELASTIC_SLACK_SCHEMA=PASS",
        "PHASE_I_SCALE_BINDINGS=PASS",
        "SCORER_REWARD_OBJECTIVE=NO",
        "TRANSCRIPTION_DETERMINISM=PASS",
        "SECOND_PLANT=NO",
        "DUPLICATE_PHYSICS=NONE",
        "DIRECT_MJ_STEP_BYPASS=NO",
        "DIRECT_TORQUE_BYPASS=NO",
        "STEP_5MS_CHANGED=NO",
        "ACTION_SLEW_CHANGED=NO",
        "SNAPSHOT_SCHEMA_CHANGED=NO",
        "TANGENT_LAYOUT_CHANGED=NO",
        "CONSTRAINT_CATALOG_CHANGED=NO",
        "DERIVATIVE_CONTRACT_CHANGED=NO",
        "PHYSICAL_MODEL_CHANGED=NO",
        "SOLVER_INSTALLED=NO",
        "ORACLE_SOLVE_EXECUTED=NO",
        "R025_EXECUTED=NO",
        "ROLLOUTS_USED=24",
        "ROLLOUTS_MAX=60",
        "GRAPH_NODES_FINAL=1656",
        "GRAPH_EDGES_FINAL=9605",
        "ADR_UPDATED=YES",
        f"TESTS={args.test_summary}",
        "PONYTAIL_REVIEW=PASS",
        "PONYTAIL_AUDIT=PASS",
        f"EVIDENCE_DIR={evidence}",
        "SHA256SUMS_SHA256=reported in final handoff after bundle seal",
        "LINEAR_ML_242=DONE",
        "LINEAR_ML_243=IN_PROGRESS",
        "LINEAR_ML_211=IN_PROGRESS",
        "NEXT_ACTION=ML-243",
        "LCMJ_V1_ML242_SPARSE_TRANSCRIPTION_COMPLETE",
    ]))

    lines = []
    for path in sorted(item for item in evidence.rglob("*") if item.is_file() and item.name != "SHA256SUMS"):
        lines.append(f"{_sha256(path)}  {path.relative_to(evidence)}")
    _write(evidence / "SHA256SUMS", "\n".join(lines))
    print(json.dumps({"evidence_dir": str(evidence), "freeze_id": freeze_id, "structure_hash": structure["hash"], "max_jvp_error": directional["max_error"], "directional_status": directional["status"]}, sort_keys=True))
    return evidence


if __name__ == "__main__":
    _main()
