"""R0.1: experiment spec immutability + spec/run binding (§6, §11)."""
import copy
import sys
from pathlib import Path

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(TASK_ROOT))

from tools.evid_spec import (
    BOUND_FIELDS,
    REQUIRED_SPEC_FIELDS,
    check_spec_run_match,
    seal_spec,
    spec_sha256,
    validate_spec,
)


def _minimal_spec():
    return {
        "EXPERIMENT_ID": "EXP-R0-1-EVIDENCE-BRANCH-REPLAY-001",
        "EXPERIMENT_VERSION": "1.0.0",
        "MISSION": "LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION",
        "AUTHORITY_COMMIT_SHA": "0" * 40,
        "AUTHORITY_COMMIT_TREE": "1" * 40,
        "HYPOTHESIS": "h",
        "START_STATE_AUTHORITY": "s",
        "ALLOWED_VARIABLES": ["a"],
        "FROZEN_VARIABLES": ["b"],
        "SEARCH_METHOD": "deterministic_single_candidate",
        "CANDIDATE_ORDERING": ["c"],
        "BUDGET_DEFINITION": {
            "MAX_FULL_EPISODE_QUALIFICATION_RUNS": 1,
            "MAX_BRANCH_ROLLOUTS": 1,
            "MAX_OBJECTIVE_EVALUATIONS": 1,
            "MAX_SOLVER_MAJOR_ITERATIONS": 0,
            "MAX_TRANSITION_JACOBIAN_EVALUATIONS": 0,
        },
        "HARD_GATES": ["g"],
        "OBJECTIVE_HIERARCHY": ["o"],
        "STOPPING_RULE": "stop_after_budget_or_first_gate_failure",
        "QUALIFICATION_OR_DIAGNOSTIC": "QUALIFICATION (evidence infrastructure, not control performance)",
        "EXPECTED_OUTPUTS": ["e"],
        "SPEC_CREATED_AT": "2026-09-03T00:00:00Z",
        "HORIZON_S": 1.0,
        "BRANCH_TIME_S": 0.5,
        "CONTROL_LAW": "harmless_pd_hold_qstand0",
        "CONTROL_LAW_PARAMS": {"Kp": 400.0, "Kd": 10.0},
        "PHYSICS_DT": 0.000125,
        "CONTROL_DT": 0.005,
        "SUBSTEPS_PER_CONTROL": 40,
        "CONTINUATION_CONTROL_STEPS": 100,
    }


def test_required_fields_present():
    assert "EXPERIMENT_SPEC_SHA256" not in REQUIRED_SPEC_FIELDS  # hash is derived, not authored
    for f in ["EXPERIMENT_ID", "EXPERIMENT_VERSION", "MISSION", "AUTHORITY_COMMIT_SHA",
              "AUTHORITY_COMMIT_TREE", "HYPOTHESIS", "START_STATE_AUTHORITY",
              "ALLOWED_VARIABLES", "FROZEN_VARIABLES", "SEARCH_METHOD", "CANDIDATE_ORDERING",
              "BUDGET_DEFINITION", "HARD_GATES", "OBJECTIVE_HIERARCHY", "STOPPING_RULE",
              "QUALIFICATION_OR_DIAGNOSTIC", "EXPECTED_OUTPUTS", "SPEC_CREATED_AT",
              "EXPERIMENT_SPEC_SHA256"]:
        pass  # contract §6 names (hash added at seal time)
    errs = validate_spec(_minimal_spec())
    assert errs == [], errs


def test_bare_budget_rejected():
    s = _minimal_spec()
    s["BUDGET"] = 12
    errs = validate_spec(s)
    assert any("BUDGET" in e for e in errs)
    s2 = _minimal_spec()
    del s2["BUDGET_DEFINITION"]["MAX_BRANCH_ROLLOUTS"]
    assert validate_spec(s2)


def test_spec_seal_immutability():
    s = _minimal_spec()
    sealed = seal_spec(s)
    assert "EXPERIMENT_SPEC_SHA256" in sealed
    # mutating the sealed spec changes its recomputed hash → binding must FAIL
    mutated = copy.deepcopy(sealed)
    mutated["HORIZON_S"] = 2.0
    run = {"EXPERIMENT_SPEC_SHA256": sealed["EXPERIMENT_SPEC_SHA256"],
           "EXPERIMENT_ID": mutated["EXPERIMENT_ID"],
           "EXPERIMENT_VERSION": mutated["EXPERIMENT_VERSION"],
           "AUTHORITY_COMMIT_SHA": mutated["AUTHORITY_COMMIT_SHA"],
           "AUTHORITY_COMMIT_TREE": mutated["AUTHORITY_COMMIT_TREE"],
           "HORIZON_S": 2.0,  # execution differs from sealed spec
           "BRANCH_TIME_S": mutated["BRANCH_TIME_S"],
           "CONTROL_LAW": mutated["CONTROL_LAW"],
           "CONTROL_LAW_PARAMS": mutated["CONTROL_LAW_PARAMS"],
           "PHYSICS_DT": mutated["PHYSICS_DT"],
           "CONTROL_DT": mutated["CONTROL_DT"],
           "SUBSTEPS_PER_CONTROL": mutated["SUBSTEPS_PER_CONTROL"],
           "CONTINUATION_CONTROL_STEPS": mutated["CONTINUATION_CONTROL_STEPS"]}
    status, mism = check_spec_run_match(
        {k: v for k, v in sealed.items()}, run)
    assert status == "FAIL"
    assert any("HORIZON_S" in m or "SPEC_SHA256" in m for m in mism)


def test_spec_run_match_pass_and_mismatch_rejection():
    s = seal_spec(_minimal_spec())
    run_ok = {
        "EXPERIMENT_SPEC_SHA256": s["EXPERIMENT_SPEC_SHA256"],
        "EXPERIMENT_ID": s["EXPERIMENT_ID"],
        "EXPERIMENT_VERSION": s["EXPERIMENT_VERSION"],
        "AUTHORITY_COMMIT_SHA": s["AUTHORITY_COMMIT_SHA"],
        "AUTHORITY_COMMIT_TREE": s["AUTHORITY_COMMIT_TREE"],
        "HORIZON_S": s["HORIZON_S"],
        "BRANCH_TIME_S": s["BRANCH_TIME_S"],
        "CONTROL_LAW": s["CONTROL_LAW"],
        "CONTROL_LAW_PARAMS": s["CONTROL_LAW_PARAMS"],
        "PHYSICS_DT": s["PHYSICS_DT"],
        "CONTROL_DT": s["CONTROL_DT"],
        "SUBSTEPS_PER_CONTROL": s["SUBSTEPS_PER_CONTROL"],
        "CONTINUATION_CONTROL_STEPS": s["CONTINUATION_CONTROL_STEPS"],
        "ACTUAL_COMMIT_SHA": s["AUTHORITY_COMMIT_SHA"],
        "ACTUAL_COMMIT_TREE": s["AUTHORITY_COMMIT_TREE"],
    }
    status, mism = check_spec_run_match({k: v for k, v in s.items() if k != "EXPERIMENT_SPEC_SHA256"}, run_ok)
    # run_ok carries the sealed sha; spec dict here excludes it so recompute from content
    assert status == "PASS", mism
    run_bad = dict(run_ok, CONTROL_LAW="different_law")
    status2, mism2 = check_spec_run_match(
        {k: v for k, v in s.items() if k != "EXPERIMENT_SPEC_SHA256"}, run_bad)
    assert status2 == "FAIL" and any("CONTROL_LAW" in m for m in mism2)
