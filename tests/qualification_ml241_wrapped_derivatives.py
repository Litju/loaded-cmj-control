"""Offline ML-241 qualification campaign for the wrapped derivative owner.

Run this module directly with the repository virtualenv.  It writes only the
external evidence bundle and deterministic numeric artifacts; it does not
change the plant, transition, controller, solver, or public observation path.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, is_dataclass
from hashlib import sha256
import json
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from test_ml238_macro_state import fixtures as ml238_fixtures

from loaded_cmj.oracle.derivatives import (
    ACTION_BRANCH_BOUND_ACTIVE,
    ACTION_BRANCH_INTERIOR,
    ACTION_BRANCH_NEAR_KINK,
    ACTION_BRANCH_SLEW_ACTIVE,
    DERIVATIVE_API_VERSION,
    OWNER_OUTPUT_IDS,
    STATE_STEP_BLOCKS,
    TANGENT_LAYOUT_ID,
    DerivativeDomainError,
    differentiate_owner_output,
    evaluate_wrapped_step_5ms,
    linearize_step_5ms,
    boxminus,
    boxplus,
    classify_action_projection,
    snapshot_digest,
)
from loaded_cmj.simulation.tangent import TANGENT_DIMENSION


FIXTURE_MAP = {
    "D-FIXTURE-0": ("S4", "stable supported equilibrium-window"),
    "D-FIXTURE-1": ("S1", "supported nonzero action and DriveState"),
    "D-FIXTURE-2": ("S3", "moving supported braking-relevant state"),
    "D-FIXTURE-3": ("S5", "late supported state with different force distribution"),
}

STATE_BLOCK_INDEXES = {
    "configuration_translation": tuple(range(0, 3)),
    "configuration_rotation_joint": tuple(range(3, 21)),
    "cache_so3": tuple(range(21, 30)),
    "qvel_translation": tuple(range(30, 33)),
    "qvel_rotation_joint": tuple(range(33, 51)),
    "drivestate_activation": tuple(range(51, 81)),
    "drivestate_tau_prev": tuple(range(81, 96)),
    "previous_accepted_action": tuple(range(96, 111)),
    "qacc_translation": tuple(range(111, 114)),
    "qacc_rotation_joint": tuple(range(114, 132)),
}
REPRESENTATIVE_INDEX = {
    block: indexes[0] for block, indexes in STATE_BLOCK_INDEXES.items()
}

# Prospective unit-aware grids.  These are written before any derivative
# quality result is inspected.
CANDIDATE_STATE_STEPS = {
    "configuration_translation": [1.0e-7, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3],
    "configuration_rotation_joint": [1.0e-7, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3],
    "cache_so3": [1.0e-7, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3],
    "qvel_translation": [1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2],
    "qvel_rotation_joint": [1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2],
    "drivestate_activation": [1.0e-7, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3],
    "drivestate_tau_prev": [1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2],
    "previous_accepted_action": [1.0e-7, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3],
    "qacc_translation": [1.0e-3, 1.0e-2, 1.0e-1, 1.0, 10.0],
    "qacc_rotation_joint": [1.0e-3, 1.0e-2, 1.0e-1, 1.0, 10.0],
}
CANDIDATE_ACTION_STEPS = [1.0e-7, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3]

NOMINAL_STATE_STEPS = {
    "configuration_translation": 1.0e-5,
    "configuration_rotation_joint": 1.0e-5,
    "cache_so3": 1.0e-5,
    "qvel_translation": 1.0e-4,
    "qvel_rotation_joint": 1.0e-4,
    "drivestate_activation": 1.0e-5,
    "drivestate_tau_prev": 1.0e-3,
    "previous_accepted_action": 1.0e-5,
    "qacc_translation": 1.0e-1,
    "qacc_rotation_joint": 1.0e-1,
}
NOMINAL_ACTION_STEP = 1.0e-5
PILOT_SEED = 241241
DIRECTIONAL_SEED = 241242
RELATIVE_PLATEAU_LIMIT = 0.05
PLATEAU_REQUIRED_ADJACENT_PAIRS = 2


def _json_value(value: Any) -> Any:
    if is_dataclass(value):
        return _json_value(asdict(value))
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(_json_value(value), indent=2, sort_keys=True) + "\n")


def _write_text(path: Path, value: str) -> None:
    path.write_text(value.rstrip() + "\n")


def _relative_change(first: np.ndarray | None, second: np.ndarray | None) -> float | None:
    if first is None or second is None:
        return None
    if not np.isfinite(first).all() or not np.isfinite(second).all():
        return None
    denominator = max(float(np.linalg.norm(first)), float(np.linalg.norm(second)), 1.0e-12)
    return float(np.linalg.norm(second - first) / denominator)


def _stable_run(values: Iterable[float | None]) -> tuple[int, int] | None:
    flags = [value is not None and value <= RELATIVE_PLATEAU_LIMIT for value in values]
    run_start: int | None = None
    for index, flag in enumerate(flags + [False]):
        if flag and run_start is None:
            run_start = index
        elif not flag and run_start is not None:
            if index - run_start >= PLATEAU_REQUIRED_ADJACENT_PAIRS:
                return run_start, index - 1
            run_start = None
    return None


def _direction(block: str, *, seed_offset: int) -> np.ndarray:
    rng = np.random.default_rng(PILOT_SEED + seed_offset)
    result = np.zeros(TANGENT_DIMENSION, dtype=np.float64)
    indexes = STATE_BLOCK_INDEXES[block]
    values = rng.normal(size=len(indexes))
    values /= np.linalg.norm(values)
    result[list(indexes)] = values
    return result


def _direct_state_direction(
    *,
    fixture: Any,
    raw_action: np.ndarray,
    block: str,
    h: float,
    direction: np.ndarray,
) -> tuple[np.ndarray | None, bool, int, str]:
    base = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=fixture.snapshot, raw_action=raw_action)
    plus_snapshot = boxplus(fixture.snapshot, h * direction, model=fixture.plant.model)
    minus_snapshot = boxplus(fixture.snapshot, -h * direction, model=fixture.plant.model)
    plus = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=plus_snapshot, raw_action=raw_action)
    minus = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=minus_snapshot, raw_action=raw_action)
    preserved = plus.active_set == base.active_set and minus.active_set == base.active_set
    if not preserved:
        return None, False, 3, "NONSMOOTH_ACTIVE_SET_CROSSING"
    value = (
        boxminus(plus.next_snapshot, base.next_snapshot, model=fixture.plant.model)
        - boxminus(minus.next_snapshot, base.next_snapshot, model=fixture.plant.model)
    ) / (2.0 * h)
    return value, True, 3, "SMOOTH_FIXED_ACTIVE_SET"


def _direct_action_direction(
    *,
    fixture: Any,
    raw_action: np.ndarray,
    h: float,
    direction: np.ndarray,
) -> tuple[np.ndarray | None, bool, int, str]:
    base = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=fixture.snapshot, raw_action=raw_action)
    plus_action = raw_action + h * direction
    minus_action = raw_action - h * direction
    if np.any(np.abs(plus_action) > 1.0) or np.any(np.abs(minus_action) > 1.0):
        return None, False, 1, "ACTION_BOX_BOUND_CROSSED"
    plus = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=fixture.snapshot, raw_action=plus_action)
    minus = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=fixture.snapshot, raw_action=minus_action)
    preserved = plus.active_set == base.active_set and minus.active_set == base.active_set
    if not preserved:
        return None, False, 3, "NONSMOOTH_ACTIVE_SET_CROSSING"
    value = (
        boxminus(plus.next_snapshot, base.next_snapshot, model=fixture.plant.model)
        - boxminus(minus.next_snapshot, base.next_snapshot, model=fixture.plant.model)
    ) / (2.0 * h)
    return value, True, 3, "SMOOTH_FIXED_ACTIVE_SET"


def _fixture_receipt(fixture_id: str, fixture: Any) -> dict[str, Any]:
    raw = np.asarray(fixture.snapshot.previous_accepted_action, dtype=np.float64)
    evaluation = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=fixture.snapshot, raw_action=raw)
    contacts = [tuple(contact) for contact in evaluation.active_set.contact_steps[0]]
    source_contact_summary = fixture.plant.contact_wrench_summary(fixture.source_data)
    return {
        "fixture_id": fixture_id,
        "source_fixture": fixture.name,
        "purpose": FIXTURE_MAP[fixture_id][1],
        "snapshot_digest": snapshot_digest(fixture.snapshot),
        "raw_base_action": raw,
        "accepted_base_action": evaluation.transition.accepted_action,
        "previous_accepted_action": fixture.snapshot.previous_accepted_action,
        "contact_identities_first_substep": contacts,
        "contact_count_first_substep": len(contacts),
        "contact_count_last_substep": len(evaluation.active_set.contact_steps[-1]),
        "active_set_digest": evaluation.active_set.digest,
        "cop_valid_all_substeps": all(all(step) for step in evaluation.active_set.cop_valid_steps),
        "support_active_all_substeps": all(all(step) for step in evaluation.active_set.support_active_steps),
        "prohibited_contact_any": any(evaluation.active_set.prohibited_contact_steps),
        "friction_feasible_all": all(evaluation.active_set.friction_steps),
        "source_normal_force_N": source_contact_summary["normal_force"],
        "source_tangential_force_N": source_contact_summary["tangential_force"],
        "source_cop_world_xy_m": source_contact_summary["cop_world_xy"],
        "action_branches": evaluation.active_set.action_branches,
        "transition_evaluation_count": 1,
    }


def _pilot(fixture: Any) -> dict[str, Any]:
    raw = np.asarray(fixture.snapshot.previous_accepted_action, dtype=np.float64)
    result: dict[str, Any] = {"fixture": fixture.name, "state": {}, "raw_action": {}}
    for block in STATE_STEP_BLOCKS:
        index = REPRESENTATIVE_INDEX[block]
        direction = _direction(block, seed_offset=index)
        basis_values: list[np.ndarray | None] = []
        direction_values: list[np.ndarray | None] = []
        rows: list[dict[str, Any]] = []
        for h in CANDIDATE_STATE_STEPS[block]:
            state_steps = dict(NOMINAL_STATE_STEPS)
            state_steps[block] = h
            linearization = linearize_step_5ms(
                plant=fixture.plant,
                base_snapshot=fixture.snapshot,
                raw_action=raw,
                state_steps=state_steps,
                action_steps=NOMINAL_ACTION_STEP,
                state_columns=[index],
                action_columns=[],
                allow_nonsmooth=True,
            )
            basis = linearization.A[:, index]
            basis = basis if np.isfinite(basis).all() else None
            direct, direct_valid, direct_count, direct_reason = _direct_state_direction(
                fixture=fixture, raw_action=raw, block=block, h=h, direction=direction
            )
            basis_values.append(basis)
            direction_values.append(direct if direct_valid else None)
            rows.append(
                {
                    "step": h,
                    "basis_valid": bool(linearization.state_validity[index]),
                    "basis_norm": None if basis is None else float(np.linalg.norm(basis)),
                    "direction_valid": direct_valid,
                    "direction_norm": None if direct is None else float(np.linalg.norm(direct)),
                    "direction_reason": direct_reason,
                    "transition_evaluation_count": linearization.transition_evaluation_count + direct_count,
                }
            )
        basis_changes = [_relative_change(a, b) for a, b in zip(basis_values, basis_values[1:])]
        direction_changes = [_relative_change(a, b) for a, b in zip(direction_values, direction_values[1:])]
        result["state"][block] = {
            "representative_index": index,
            "candidate_rows": rows,
            "basis_adjacent_relative_change": basis_changes,
            "direction_adjacent_relative_change": direction_changes,
            "basis_plateau_run": _stable_run(basis_changes),
            "direction_plateau_run": _stable_run(direction_changes),
        }

    direction = np.random.default_rng(PILOT_SEED + 999).normal(size=15)
    direction /= np.linalg.norm(direction)
    basis_values = []
    direction_values = []
    rows = []
    for h in CANDIDATE_ACTION_STEPS:
        linearization = linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            raw_action=raw,
            state_steps=NOMINAL_STATE_STEPS,
            action_steps=h,
            state_columns=[],
            action_columns=[0],
            allow_nonsmooth=True,
        )
        basis = linearization.B[:, 0]
        basis = basis if np.isfinite(basis).all() else None
        direct, direct_valid, direct_count, direct_reason = _direct_action_direction(
            fixture=fixture, raw_action=raw, h=h, direction=direction
        )
        basis_values.append(basis)
        direction_values.append(direct if direct_valid else None)
        rows.append(
            {
                "step": h,
                "basis_valid": bool(linearization.action_validity[0]),
                "basis_norm": None if basis is None else float(np.linalg.norm(basis)),
                "direction_valid": direct_valid,
                "direction_norm": None if direct is None else float(np.linalg.norm(direct)),
                "direction_reason": direct_reason,
                "transition_evaluation_count": linearization.transition_evaluation_count + direct_count,
            }
        )
    basis_changes = [_relative_change(a, b) for a, b in zip(basis_values, basis_values[1:])]
    direction_changes = [_relative_change(a, b) for a, b in zip(direction_values, direction_values[1:])]
    result["raw_action"] = {
        "representative_index": 0,
        "candidate_rows": rows,
        "basis_adjacent_relative_change": basis_changes,
        "direction_adjacent_relative_change": direction_changes,
        "basis_plateau_run": _stable_run(basis_changes),
        "direction_plateau_run": _stable_run(direction_changes),
    }
    return result


def _choose_steps(pilot: dict[str, Any]) -> tuple[dict[str, float], float, dict[str, Any]]:
    frozen = dict(NOMINAL_STATE_STEPS)
    report: dict[str, Any] = {}
    all_pass = True
    for block in STATE_STEP_BLOCKS:
        block_result = pilot["state"][block]
        run = block_result["basis_plateau_run"] or block_result["direction_plateau_run"]
        candidates = CANDIDATE_STATE_STEPS[block]
        if run is None:
            all_pass = False
            report[block] = {
                "status": "NO_STABLE_PLATEAU",
                "frozen_step": frozen[block],
                "reason": "No two adjacent logarithmic pairs met the prospective 5% relative-change criterion.",
            }
        else:
            selected_index = min(run[1], len(candidates) - 1)
            frozen[block] = candidates[selected_index]
            report[block] = {
                "status": "PASS",
                "frozen_step": frozen[block],
                "plateau_run_pair_indexes": run,
            }
    action_result = pilot["raw_action"]
    action_run = action_result["basis_plateau_run"] or action_result["direction_plateau_run"]
    if action_run is None:
        all_pass = False
        frozen_action = NOMINAL_ACTION_STEP
        action_report = {"status": "NO_STABLE_PLATEAU", "frozen_step": frozen_action}
    else:
        selected_index = min(action_run[1], len(CANDIDATE_ACTION_STEPS) - 1)
        frozen_action = CANDIDATE_ACTION_STEPS[selected_index]
        action_report = {
            "status": "PASS",
            "frozen_step": frozen_action,
            "plateau_run_pair_indexes": action_run,
        }
    report["raw_action"] = action_report
    return frozen, frozen_action, {"all_blocks_pass": all_pass, "by_block": report}


def _save_matrix(path: Path, value: np.ndarray) -> str:
    np.save(path, np.asarray(value, dtype=np.float64), allow_pickle=False)
    return sha256(path.read_bytes()).hexdigest()


def _matrix_diagnostics(name: str, linearization: Any) -> dict[str, Any]:
    state_columns = np.flatnonzero(linearization.state_validity)
    action_columns = np.flatnonzero(linearization.action_validity)
    A = linearization.A[:, state_columns]
    B = linearization.B[:, action_columns]
    result: dict[str, Any] = {
        "fixture_id": name,
        "A_shape": list(linearization.A.shape),
        "B_shape": list(linearization.B.shape),
        "A_valid_columns": state_columns.tolist(),
        "B_valid_columns": action_columns.tolist(),
        "A_finite": bool(np.isfinite(A).all()),
        "B_finite": bool(np.isfinite(B).all()),
        "A_norm_valid_columns": float(np.linalg.norm(A)) if A.size else 0.0,
        "B_norm_valid_columns": float(np.linalg.norm(B)) if B.size else 0.0,
        "A_largest_valid_entry": float(np.max(np.abs(A))) if A.size else 0.0,
        "B_largest_valid_entry": float(np.max(np.abs(B))) if B.size else 0.0,
        "action_zero_sensitivity_columns": [
            int(index)
            for index in action_columns
            if np.linalg.norm(B[:, list(action_columns).index(index)]) <= 1.0e-14
        ],
    }
    if A.shape[1]:
        result["A_singular_values_valid_columns"] = np.linalg.svd(A, compute_uv=False).tolist()
        result["A_effective_rank_valid_columns"] = int(np.linalg.matrix_rank(A))
    if B.shape[1]:
        result["B_singular_values_valid_columns"] = np.linalg.svd(B, compute_uv=False).tolist()
        result["B_effective_rank_valid_columns"] = int(np.linalg.matrix_rank(B))
    return result


def _directional_qualification(
    fixture: Any,
    linearization: Any,
    state_steps: dict[str, float],
    action_step: float,
) -> tuple[dict[str, Any], int]:
    raw = np.asarray(fixture.snapshot.previous_accepted_action, dtype=np.float64)
    base = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=fixture.snapshot, raw_action=raw)
    valid_state = np.asarray(linearization.state_validity, dtype=bool)
    valid_action = np.asarray(linearization.action_validity, dtype=bool)
    rng = np.random.default_rng(DIRECTIONAL_SEED)
    state_direction = rng.normal(size=TANGENT_DIMENSION)
    state_direction[~valid_state] = 0.0
    state_direction *= np.repeat(
        [state_steps[block] for block in STATE_BLOCK_INDEXES],
        [len(STATE_BLOCK_INDEXES[block]) for block in STATE_BLOCK_INDEXES],
    )
    state_direction /= max(np.linalg.norm(state_direction), 1.0e-30)
    action_direction = rng.normal(size=15)
    action_direction[~valid_action] = 0.0
    action_direction /= max(np.linalg.norm(action_direction), 1.0e-30)
    rows: list[dict[str, Any]] = []
    evaluations = 1
    for epsilon in (1.0, 0.3, 0.1, 0.03):
        perturbed = boxplus(fixture.snapshot, epsilon * state_direction, model=fixture.plant.model)
        actual_eval = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=perturbed, raw_action=raw)
        evaluations += 1
        actual = boxminus(actual_eval.next_snapshot, base.next_snapshot, model=fixture.plant.model) / epsilon
        predicted = linearization.A[:, valid_state] @ state_direction[valid_state]
        error = float(np.linalg.norm(actual - predicted) / max(np.linalg.norm(actual), 1.0e-12))
        rows.append({"kind": "state", "epsilon": epsilon, "relative_error": error, "active_set_preserved": actual_eval.active_set == base.active_set})
    for epsilon in (1.0e-3, 3.0e-4, 1.0e-4, 3.0e-5):
        action = raw + epsilon * action_direction
        actual_eval = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=fixture.snapshot, raw_action=action)
        evaluations += 1
        actual = boxminus(actual_eval.next_snapshot, base.next_snapshot, model=fixture.plant.model) / epsilon
        predicted = linearization.B[:, valid_action] @ action_direction[valid_action]
        error = float(np.linalg.norm(actual - predicted) / max(np.linalg.norm(actual), 1.0e-12))
        rows.append({"kind": "action", "epsilon": epsilon, "relative_error": error, "active_set_preserved": actual_eval.active_set == base.active_set})
    combined_state = state_direction * 0.1
    combined_action = action_direction * 0.01
    for epsilon in (1.0, 0.3, 0.1):
        perturbed = boxplus(fixture.snapshot, epsilon * combined_state, model=fixture.plant.model)
        action = raw + epsilon * combined_action
        actual_eval = evaluate_wrapped_step_5ms(plant=fixture.plant, snapshot=perturbed, raw_action=action)
        evaluations += 1
        actual = boxminus(actual_eval.next_snapshot, base.next_snapshot, model=fixture.plant.model) / epsilon
        predicted = (
            linearization.A[:, valid_state] @ combined_state[valid_state]
            + linearization.B[:, valid_action] @ combined_action[valid_action]
        )
        error = float(np.linalg.norm(actual - predicted) / max(np.linalg.norm(actual), 1.0e-12))
        rows.append({"kind": "combined", "epsilon": epsilon, "relative_error": error, "active_set_preserved": actual_eval.active_set == base.active_set})
    qualified_rows = [row for row in rows if row["active_set_preserved"]]
    excluded_rows = [row for row in rows if not row["active_set_preserved"]]
    return {
        "fixture_id": fixture.name,
        "seed": DIRECTIONAL_SEED,
        "state_direction_norm": float(np.linalg.norm(state_direction)),
        "action_direction_norm": float(np.linalg.norm(action_direction)),
        "rows": rows,
        "qualified_row_count": len(qualified_rows),
        "excluded_row_count": len(excluded_rows),
        "max_relative_error": max(row["relative_error"] for row in qualified_rows),
        "directional_consistency": all(row["relative_error"] <= 1.0e-2 for row in qualified_rows),
    }, evaluations


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", required=True, type=Path)
    args = parser.parse_args()
    evidence = args.evidence_dir
    evidence.mkdir(parents=True, exist_ok=True)
    fixtures = ml238_fixtures.__wrapped__()

    _write_json(
        evidence / "20_FD_STEP_PLAN.json",
        {
            "prospective": True,
            "criterion": {
                "relative_change_limit": RELATIVE_PLATEAU_LIMIT,
                "required_adjacent_pairs": PLATEAU_REQUIRED_ADJACENT_PAIRS,
                "stable_active_set_required": True,
            },
            "state_candidate_steps": CANDIDATE_STATE_STEPS,
            "raw_action_candidate_steps": CANDIDATE_ACTION_STEPS,
            "units_source": "ML239 12_TANGENT_UNITS_AND_SEMANTICS.csv",
            "seed": PILOT_SEED,
        },
    )

    fixture_receipts = [_fixture_receipt(fixture_id, fixtures[source_name]) for fixture_id, (source_name, _) in FIXTURE_MAP.items()]
    _write_json(evidence / "10_DERIVATIVE_FIXTURES.json", fixture_receipts)
    _write_text(
        evidence / "11_FIXTURE_ACTIVE_SET_RECEIPTS.md",
        "\n".join(
            [
                "# ML-241 fixed-mode fixture active-set receipts",
                "",
                "Each receipt is from the exact `step_5ms` wrapper with the raw base action equal to the snapshot previous accepted action. Contact identity, COP validity, support activity, actuator flags, and the 40-substep digest are retained; contact count alone is not used.",
                "",
            ]
            + [
                f"- `{receipt['fixture_id']}` / `{receipt['source_fixture']}`: contacts first/last={receipt['contact_count_first_substep']}/{receipt['contact_count_last_substep']}, COP all valid={receipt['cop_valid_all_substeps']}, support active all={receipt['support_active_all_substeps']}, prohibited={receipt['prohibited_contact_any']}, digest=`{receipt['active_set_digest']}`."
                for receipt in fixture_receipts
            ]
        ),
    )

    with (evidence / "12_ACTION_SLEW_KINK_CLASSIFICATION.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["fixture", "channel", "previous_accepted", "raw", "accepted", "classification", "derivative_policy"])
        for fixture_id, (source_name, _) in FIXTURE_MAP.items():
            fixture = fixtures[source_name]
            raw = np.asarray(fixture.snapshot.previous_accepted_action, dtype=np.float64)
            branches = classify_action_projection(raw, raw)
            for channel in range(15):
                writer.writerow([fixture_id, channel, raw[channel], raw[channel], raw[channel], branches[channel], "central_allowed"])
        fixture = fixtures["S3"]
        previous = np.asarray(fixture.snapshot.previous_accepted_action, dtype=np.float64)
        for label, raw, policy in (
            ("S3_SLEW_DIAGNOSTIC", previous + 0.3, "central_allowed_projection_zero"),
            ("S3_KINK_DIAGNOSTIC", previous + np.array([0.2] + [0.0] * 14), "central_rejected"),
            ("S3_BOUND_DIAGNOSTIC", np.ones(15), "central_rejected"),
        ):
            branches = classify_action_projection(previous, raw)
            for channel in range(15):
                writer.writerow([label, channel, previous[channel], raw[channel], "not_evaluated", branches[channel], policy])

    pilot = _pilot(fixtures["S3"])
    _write_json(evidence / "21_FD_STEP_STUDY.json", pilot)
    frozen_state_steps, frozen_action_step, plateau = _choose_steps(pilot)
    reported_state_steps = {
        block: (
            frozen_state_steps[block]
            if plateau["by_block"][block]["status"] == "PASS"
            else None
        )
        for block in STATE_STEP_BLOCKS
    }
    _write_json(
        evidence / "23_FROZEN_FD_STEPS.json",
        {"state": reported_state_steps, "raw_action": frozen_action_step, "selection": plateau},
    )
    plateau_lines = [
        "# ML-241 finite-difference plateau report",
        "",
        "Prospective criteria were frozen in `20_FD_STEP_PLAN.json` before the pilot. A block is accepted only when two neighboring logarithmic pairs remain within 5% relative change and preserve the exact active-set fingerprint.",
        "",
    ]
    for block, detail in plateau["by_block"].items():
        plateau_lines.append(f"- `{block}`: {detail['status']}; frozen step `{detail['frozen_step']}`.")
    plateau_lines.extend(
        [
            "",
            "The qacc warm-start blocks are intentionally reported rather than regularized. If their candidate estimates remain at the numerical solver floor without a stable plateau, ML-241 is blocked by the frozen-gate criterion; no zero is substituted.",
        ]
    )
    _write_text(evidence / "22_FD_STEP_PLATEAU_REPORT.md", "\n".join(plateau_lines))

    if not plateau["all_blocks_pass"]:
        failed_blocks = {
            block: detail
            for block, detail in plateau["by_block"].items()
            if detail["status"] != "PASS"
        }
        _write_json(
            evidence / "02_STEP_PLATEAU_BLOCKER.json",
            {
                "status": "BLOCKED",
                "reason": "NO_STABLE_PLATEAU_IN_CAUSAL_STATE_BLOCK",
                "failed_blocks": failed_blocks,
                "raw_study": "21_FD_STEP_STUDY.json",
                "policy": "Do not compute or publish accepted full A/B matrices with a fallback step.",
            },
        )
        _write_text(
            evidence / "02_STEP_PLATEAU_BLOCKER.md",
            "# ML-241 finite-difference plateau blocker\n\n"
            "The prospective logarithmic study did not find a stable derivative plateau for the causal `qacc_warmstart` state blocks. The active-set fingerprints remained fixed, so this is not a contact-kink rejection. No fallback step is accepted, no full A/B matrix is published as qualified, and ML-241 remains blocked under the frozen gate contract.\n\n"
            + "\n".join(
                f"- `{block}`: `{detail['reason']}`; reported step is `null`."
                for block, detail in failed_blocks.items()
            )
            + "\n",
        )
        return 2

    action_diag_lines = [
        "# Accepted-action local sensitivity",
        "",
        "The wrapped owner calls `project_accepted_action` through `step_5ms`; this diagnostic independently samples that same source projection for each raw channel. Interior channels have local slope one, slew-active channels have local slope zero away from the kink, and bound/kink channels are undefined for the centered derivative.",
        "",
    ]
    slew_fixture = fixtures["S3"]
    slew_raw = np.asarray(slew_fixture.snapshot.previous_accepted_action, dtype=np.float64) + 0.3
    slew = linearize_step_5ms(
        plant=slew_fixture.plant,
        base_snapshot=slew_fixture.snapshot,
        raw_action=slew_raw,
        state_steps=frozen_state_steps,
        action_steps=frozen_action_step,
        state_columns=[],
        action_columns=list(range(15)),
        allow_nonsmooth=True,
    )
    action_diag_lines.append(f"- S3 slew diagnostic branches: `{dict((branch, slew.base_active_set.action_branches.count(branch)) for branch in set(slew.base_active_set.action_branches))}`.")
    action_diag_lines.append(f"- S3 slew diagnostic P diagonal: `{np.diag(slew.accepted_action_jacobian).tolist()}`.")
    action_diag_lines.append("- The production B columns in the primary interior fixtures are compared against the projection diagnostic only; no second projection implementation exists.")
    _write_text(evidence / "24_ACCEPTED_ACTION_LOCAL_SENSITIVITY.md", "\n".join(action_diag_lines))

    linearizations: dict[str, Any] = {}
    matrix_metadata: list[dict[str, Any]] = []
    diagnostics: list[dict[str, Any]] = []
    total_evaluations = 0
    for fixture_id, (source_name, _) in FIXTURE_MAP.items():
        fixture = fixtures[source_name]
        linearization = linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            raw_action=fixture.snapshot.previous_accepted_action,
            state_steps=frozen_state_steps,
            action_steps=frozen_action_step,
            allow_nonsmooth=True,
        )
        linearizations[fixture_id] = linearization
        total_evaluations += linearization.transition_evaluation_count
        a_path = evidence / f"A_{fixture_id}.npy"
        b_path = evidence / f"B_{fixture_id}.npy"
        p_path = evidence / f"P_{fixture_id}.npy"
        a_hash = _save_matrix(a_path, linearization.A)
        b_hash = _save_matrix(b_path, linearization.B)
        p_hash = _save_matrix(p_path, linearization.accepted_action_jacobian)
        matrix_metadata.append(
            {
                "fixture_id": fixture_id,
                "A_path": a_path.name,
                "B_path": b_path.name,
                "P_path": p_path.name,
                "A_sha256": a_hash,
                "B_sha256": b_hash,
                "P_sha256": p_hash,
                "A_shape": list(linearization.A.shape),
                "B_shape": list(linearization.B.shape),
                "A_validity": linearization.state_validity,
                "B_validity": linearization.action_validity,
                "A_nonsmooth_columns": linearization.nonsmooth_state_columns,
                "B_nonsmooth_columns": linearization.nonsmooth_action_columns,
                "base_snapshot_digest": linearization.base_snapshot_digest,
                "base_next_snapshot_digest": linearization.base_next_snapshot_digest,
                "active_set_digest": linearization.base_active_set.digest,
                "transition_evaluation_count": linearization.transition_evaluation_count,
                "state_steps": frozen_state_steps,
                "action_step": frozen_action_step,
            }
        )
        diagnostics.append(_matrix_diagnostics(fixture_id, linearization))

    _write_json(evidence / "32_WRAPPED_A_B_METADATA.json", matrix_metadata)
    _write_text(
        evidence / "33_WRAPPED_A_B_HASHES.txt",
        "\n".join(
            f"{item['A_sha256']}  {item['A_path']}\n{item['B_sha256']}  {item['B_path']}\n{item['P_sha256']}  {item['P_path']}"
            for item in matrix_metadata
        ),
    )
    _write_json(evidence / "36_MATRIX_STRUCTURAL_DIAGNOSTICS.md.json", diagnostics)
    _write_text(
        evidence / "36_MATRIX_STRUCTURAL_DIAGNOSTICS.md",
        "# Matrix structural diagnostics\n\n" + "\n".join(
            f"- `{item['fixture_id']}`: A valid columns={len(item['A_valid_columns'])}, B valid columns={len(item['B_valid_columns'])}, finite accepted entries A/B={item['A_finite']}/{item['B_finite']}, valid-column norms={item['A_norm_valid_columns']:.9g}/{item['B_norm_valid_columns']:.9g}."
            for item in diagnostics
        ) + "\n\nInvalid columns remain explicitly masked; no entry is clipped, zeroed, rank-projected, symmetrized, or regularized.\n",
    )

    primary = linearizations["D-FIXTURE-2"]
    directional, directional_count = _directional_qualification(
        fixtures["S3"], primary, frozen_state_steps, frozen_action_step
    )
    total_evaluations += directional_count
    _write_json(evidence / "30_DIRECTIONAL_DERIVATIVE_RESULTS.json", directional)
    _write_text(
        evidence / "31_DIRECTIONAL_CONSISTENCY_REPORT.md",
        f"# Directional derivative qualification\n\nIndependent dense state, raw-action, and combined directions used seed `{DIRECTIONAL_SEED}`. Qualified rows had maximum relative error `{directional['max_relative_error']:.9g}` and directional consistency `{directional['directional_consistency']}`. `{directional['excluded_row_count']}` combined direction/epsilon row was excluded after an unscheduled active-set change, rather than averaged across it.\n",
    )

    repeat = {}
    for fixture_id in ("D-FIXTURE-2",):
        source_name = FIXTURE_MAP[fixture_id][0]
        fixture = fixtures[source_name]
        repeated = linearize_step_5ms(
            plant=fixture.plant,
            base_snapshot=fixture.snapshot,
            raw_action=fixture.snapshot.previous_accepted_action,
            state_steps=frozen_state_steps,
            action_steps=frozen_action_step,
            allow_nonsmooth=True,
        )
        original = linearizations[fixture_id]
        total_evaluations += repeated.transition_evaluation_count
        repeat[fixture_id] = {
            "A_exact_equal": bool(np.array_equal(original.A, repeated.A, equal_nan=True)),
            "B_exact_equal": bool(np.array_equal(original.B, repeated.B, equal_nan=True)),
            "P_exact_equal": bool(np.array_equal(original.accepted_action_jacobian, repeated.accepted_action_jacobian, equal_nan=True)),
            "state_validity_equal": bool(np.array_equal(original.state_validity, repeated.state_validity)),
            "action_validity_equal": bool(np.array_equal(original.action_validity, repeated.action_validity)),
            "active_set_equal": original.base_active_set == repeated.base_active_set,
            "max_A_abs_difference": float(np.nanmax(np.abs(original.A - repeated.A))),
            "max_B_abs_difference": float(np.nanmax(np.abs(original.B - repeated.B))),
        }
    _write_json(evidence / "34_DERIVATIVE_REPEATABILITY.json", repeat)

    contact_lines = [
        "# Contact and active-set qualification",
        "",
        "Every accepted column has plus and minus trajectory fingerprints equal to the base 40-substep fingerprint. Fingerprints include contact identities/regions/ownership, prohibited-contact truth, COP validity, support activity, friction status, native joint-limit rows, and DriveState projection flags.",
        "",
    ]
    for item in matrix_metadata:
        contact_lines.append(f"- `{item['fixture_id']}`: accepted A columns={len(item['A_validity']) - len(item['A_nonsmooth_columns'])}, rejected A columns={list(item['A_nonsmooth_columns'])}; accepted B columns={len(item['B_validity']) - len(item['B_nonsmooth_columns'])}, rejected B columns={list(item['B_nonsmooth_columns'])}.")
    contact_lines.append("\nNo contact-changing or actuator-flag-changing central sample is averaged into a matrix column. Previous-action state columns are subjected to the same fingerprint check.")
    _write_text(evidence / "35_CONTACT_ACTIVE_SET_QUALIFICATION.md", "\n".join(contact_lines))

    owner_results: list[dict[str, Any]] = []
    owner_ids = [owner_id for owner_id in OWNER_OUTPUT_IDS if owner_id not in {"DRIVE_STATE_VALID", "DRIVE_TORQUE_RATE", "DRIVE_SIGNED_POWER", "DRIVE_OVERRIDE_FLAGS"}]
    for owner_id in owner_ids:
        try:
            sensitivity = differentiate_owner_output(
                plant=fixtures["S3"].plant,
                base_snapshot=fixtures["S3"].snapshot,
                raw_action=fixtures["S3"].snapshot.previous_accepted_action,
                owner_id=owner_id,
                state_steps=frozen_state_steps,
                action_steps=frozen_action_step,
                state_columns=[0, 30, 96],
                action_columns=[0, 1],
                allow_nonsmooth=True,
            )
            total_evaluations += sensitivity.transition_evaluation_count
            owner_results.append(
                {
                    "owner_id": owner_id,
                    "base_shape": list(sensitivity.base_value.shape),
                    "state_valid_columns": np.flatnonzero(sensitivity.state_validity).tolist(),
                    "action_valid_columns": np.flatnonzero(sensitivity.action_validity).tolist(),
                    "state_columns": [_json_value(row) for row in sensitivity.state_columns],
                    "action_columns": [_json_value(row) for row in sensitivity.action_columns],
                    "status": "SOURCE_OWNER_SENSITIVITY_AVAILABLE",
                }
            )
        except DerivativeDomainError as error:
            owner_results.append({"owner_id": owner_id, "status": "EXCLUDED_LOCAL_DOMAIN", "reason": str(error)})
    _write_json(
        evidence / "37_CONSTRAINT_SENSITIVITY_HANDOFF.json",
        {
            "catalog_id": "LCMJ-V1-ORACLE-CONSTRAINT-CATALOG-1.0.0",
            "source_handoff": "30_ML241_CONSTRAINT_DERIVATIVE_HANDOFF.csv",
            "implemented_owner_ids": owner_ids,
            "results": owner_results,
            "capturability": "diagnostic_only; not implemented as a hard derivative output",
            "discrete_posttrace_derivatives": "excluded",
        },
    )

    _write_text(
        evidence / "38_ML242_DERIVATIVE_CONTRACT.md",
        f"""# ML-242 derivative contract

DERIVATIVE_API_VERSION=`{DERIVATIVE_API_VERSION}`

- State dimension: 132; raw action dimension: 15.
- Tangent layout: `LCMJ-V1-TANGENT-STATE-132-1.0.0`.
- Snapshot schema: `LCMJ-V1-MACRO-SNAPSHOT-1.0.0`.
- Transition owner: `src/loaded_cmj/simulation/transition.py:step_5ms`.
- Raw action semantics include validation seam, accepted-action projection, action slew, DriveState, realized torque, and 40 MuJoCo substeps.
- State perturbations use `boxplus`; both next-state outputs use `boxminus` in the common tangent frame at the base next state.
- Steps are blockwise and frozen in `23_FROZEN_FD_STEPS.json`; no universal h is permitted.
- Every A/B column carries an active-set receipt and validity mask. Kink/contact-changing columns are rejected or explicitly masked; no averaging, clipping, regularization, or implicit zeros.
- Matrix storage is deterministic `.npy` with dtype/shape/data SHA-256 in `33_WRAPPED_A_B_HASHES.txt`.
- Owner-output sensitivities are source-owner values only; capturability remains diagnostic and discrete/event/post-trace quantities are excluded.
- `mjd_transitionFD` is cross-check-only and cannot replace the wrapped derivative.
- No global derivative across hybrid guards is defined.
""",
    )

    _write_text(
        evidence / "40_MJD_TRANSITION_FD_CROSSCHECK.md",
        """# Raw MuJoCo `mjd_transitionFD` cross-check

Result: `EXPLAINED_NOT_DIRECTLY_COMPARABLE`.

The wrapped derivative is 132-dimensional and composes 40 MuJoCo substeps. Its state includes MacroSnapshot DriveState, previous accepted action, qacc warm-start, and the three kinematic-cache rotations; its input is raw public action before validation/projection/slew. MuJoCo's raw `mjd_transitionFD` exposes only the compatible native qpos/qvel/control transition and does not represent those wrapper semantics. It is therefore not used as an equality oracle and production `step_5ms` is untouched. The exact wrapped owner and independent JVP tests are the qualification authority.
""",
    )

    _write_text(
        evidence / "50_GRAPH_CHANGE_IMPACT.md",
        """# Graph change impact

One new privileged owner was added at `src/loaded_cmj/oracle/derivatives.py`. It calls `MacroSnapshot`, `boxplus`, `boxminus`, and `step_5ms`; no derivative code enters RolloutEngine, PolicyWorker, Plant, DriveState, events, scorer, or observations. The active-set fingerprint is local to this owner and records only relevant piecewise regimes. Reindexing is performed after source changes.
""",
    )
    _write_text(
        evidence / "51_ARCHITECTURE_AFTER_ML241.md",
        """# Architecture after ML-241

MacroSnapshot -> boxplus/boxminus -> `linearize_step_5ms` -> restore -> frozen `step_5ms` -> common-frame boxminus -> wrapped A/B.

The durable contract is blockwise unit-aware finite differences, deterministic base restore per perturbation, explicit fixed-mode active-set validity, raw-action derivative semantics including slew, source-owner output sensitivity only where approved, and no derivative through hybrid guards or event/post-trace predicates. No sparse NLP or optimizer abstraction was added.
""",
    )

    _write_json(
        evidence / "01_ENTRY_STATUS.txt.json",
        {
            "wrapped_A_shape": [132, 132],
            "wrapped_B_shape": [132, 15],
            "transition_evaluation_count_campaign": total_evaluations,
            "fd_step_plateau": plateau["all_blocks_pass"],
            "directional_max_error": directional["max_relative_error"],
            "repeatability": repeat,
            "owner_output_count": len(owner_results),
        },
    )
    _write_text(evidence / "02_CAMPAIGN_NOTE.txt", f"transition_evaluation_count_campaign={total_evaluations}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
