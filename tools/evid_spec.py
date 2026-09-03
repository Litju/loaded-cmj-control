#!/usr/bin/env python3
"""Experiment predeclaration vs execution separation (R0.1 §2B + §6).

Three artifacts:

  experiment_spec.json   — immutable BEFORE execution
  run_record.json        — what actually executed
  result_assessment.json — post-hoc interpretation only

The recorder carries EXPERIMENT_SPEC_SHA256 in the run record and proves it
executed the declared experiment. If execution parameters differ,
SPEC_EXECUTION_MATCH=FAIL; the result is preserved as exploratory evidence
but cannot claim qualification under the mismatched spec.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

from tools.evid_canonical import canonical_bytes

REQUIRED_SPEC_FIELDS = [
    "EXPERIMENT_ID",
    "EXPERIMENT_VERSION",
    "MISSION",
    "AUTHORITY_COMMIT_SHA",
    "AUTHORITY_COMMIT_TREE",
    "HYPOTHESIS",
    "START_STATE_AUTHORITY",
    "ALLOWED_VARIABLES",
    "FROZEN_VARIABLES",
    "SEARCH_METHOD",
    "CANDIDATE_ORDERING",
    "BUDGET_DEFINITION",
    "HARD_GATES",
    "OBJECTIVE_HIERARCHY",
    "STOPPING_RULE",
    "QUALIFICATION_OR_DIAGNOSTIC",
    "EXPECTED_OUTPUTS",
    "SPEC_CREATED_AT",
]

# Fields compared for spec/run binding (execution-relevant subset).
BOUND_FIELDS = [
    "EXPERIMENT_ID",
    "EXPERIMENT_VERSION",
    "AUTHORITY_COMMIT_SHA",
    "AUTHORITY_COMMIT_TREE",
    "HORIZON_S",
    "BRANCH_TIME_S",
    "CONTROL_LAW",
    "CONTROL_LAW_PARAMS",
    "PHYSICS_DT",
    "CONTROL_DT",
    "SUBSTEPS_PER_CONTROL",
    "CONTINUATION_CONTROL_STEPS",
]


def spec_sha256(spec: dict) -> str:
    """EXPERIMENT_SPEC_SHA256 = SHA256(canonical bytes of full spec dict)."""
    return hashlib.sha256(canonical_bytes(spec)).hexdigest()


def validate_spec(spec: dict) -> list[str]:
    errors: list[str] = []
    for f in REQUIRED_SPEC_FIELDS:
        if f not in spec:
            errors.append(f"missing required field {f}")
    if "EXPERIMENT_SPEC_SHA256" in spec:
        errors.append("spec must not contain EXPERIMENT_SPEC_SHA256 before sealing")
    bd = spec.get("BUDGET_DEFINITION", {})
    if not isinstance(bd, dict):
        errors.append("BUDGET_DEFINITION must be an object with explicit MAX_* counters")
    else:
        for k in [
            "MAX_FULL_EPISODE_QUALIFICATION_RUNS",
            "MAX_BRANCH_ROLLOUTS",
            "MAX_OBJECTIVE_EVALUATIONS",
            "MAX_SOLVER_MAJOR_ITERATIONS",
            "MAX_TRANSITION_JACOBIAN_EVALUATIONS",
        ]:
            if k not in bd:
                errors.append(f"BUDGET_DEFINITION missing {k}")
            elif not isinstance(bd[k], int) or bd[k] < 0:
                errors.append(f"BUDGET_DEFINITION[{k}] must be a non-negative int")
        if "BUDGET" in spec or "candidate_budget" in spec:
            errors.append("bare BUDGET/candidate_budget without unit definition is forbidden")
    return errors


def seal_spec(spec: dict) -> dict:
    """Return a sealed copy with EXPERIMENT_SPEC_SHA256 added (immutable)."""
    errors = validate_spec(spec)
    if errors:
        raise ValueError("invalid experiment_spec: " + "; ".join(errors))
    sealed = dict(spec)
    sealed["EXPERIMENT_SPEC_SHA256"] = spec_sha256(spec)
    return sealed


def check_spec_run_match(spec: dict, run: dict) -> tuple[str, list[str]]:
    """Compare execution-relevant fields. Returns (PASS|FAIL, mismatches)."""
    mismatches: list[str] = []
    declared_sha = run.get("EXPERIMENT_SPEC_SHA256") or spec.get("EXPERIMENT_SPEC_SHA256")
    recomputed = spec_sha256({k: v for k, v in spec.items() if k != "EXPERIMENT_SPEC_SHA256"})
    if declared_sha is not None and declared_sha != recomputed:
        mismatches.append(
            f"EXPERIMENT_SPEC_SHA256 mismatch: run carries {declared_sha} != recomputed {recomputed}"
        )
    for f in BOUND_FIELDS:
        if f in spec or f in run:
            sv = spec.get(f, "__ABSENT__")
            rv = run.get(f, "__ABSENT__")
            if json.dumps(sv, sort_keys=True, default=str) != json.dumps(rv, sort_keys=True, default=str):
                mismatches.append(f"{f}: spec={sv!r} run={rv!r}")
    # authority gate: run commit must equal spec authority
    if run.get("ACTUAL_COMMIT_SHA") is not None and spec.get("AUTHORITY_COMMIT_SHA") is not None:
        if run["ACTUAL_COMMIT_SHA"] != spec["AUTHORITY_COMMIT_SHA"]:
            mismatches.append(
                f"AUTHORITY_COMMIT_SHA: spec={spec['AUTHORITY_COMMIT_SHA']} run={run['ACTUAL_COMMIT_SHA']}"
            )
    if run.get("ACTUAL_COMMIT_TREE") is not None and spec.get("AUTHORITY_COMMIT_TREE") is not None:
        if run["ACTUAL_COMMIT_TREE"] != spec["AUTHORITY_COMMIT_TREE"]:
            mismatches.append(
                f"AUTHORITY_COMMIT_TREE: spec={spec['AUTHORITY_COMMIT_TREE']} run={run['ACTUAL_COMMIT_TREE']}"
            )
    return ("PASS" if not mismatches else "FAIL", mismatches)


def write_json(path: Path, obj: dict) -> None:
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n")


def load_json(path: Path) -> dict:
    return json.loads(Path(path).read_text())
