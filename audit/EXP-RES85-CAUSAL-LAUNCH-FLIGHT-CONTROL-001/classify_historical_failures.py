"""Classify the repository-suite failures recorded during the RES-85 qualification.

MISSION: RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001

The RES-85 qualification ran:

    pytest tests/ -q -p no:randomly \
        --ignore=tests/test_ml241_qacc_resolution.py \
        --ignore=tests/test_public_support_wrench_contract.py
        -> 859 collected, 834 passed, 25 failed, 0 RES-85-owned failures

    2 modules fail at collection:
      * tests/test_ml241_qacc_resolution.py (reads an absent external session path)
      * tests/test_public_support_wrench_contract.py (imports a name that does not
        exist in the historical V1 plant module)

Every failure above was reproduced identically in a detached worktree at
ENTRY_HEAD ``b0eccb8a6eeda40950665b3be517854f8b48baab``, except
``test_v2_1_res16_true_standing.py::test_01_captured_satisfies_old_but_fails_new``
which is path-dependent: it fails in the worktree only because the external
evidence root resolves relative to the worktree parent, and passes when that
root resolves.  No RES-85 test, no RES-83/RES-84 test and no V3 module test
fails.

This script re-emits the classification deterministically from the recorded
run; it never re-runs the suite (which is a 55-minute operation) and it never
hides a failure.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "FULL_SUITE_CLASSIFICATION.json"

CATEGORY_EXTERNAL_SESSION = "PRE_EXISTING_EXTERNAL_SESSION_ARTIFACT_ABSENT"
CATEGORY_EVIDENCE_CONTRACT = "PRE_EXISTING_EXTERNAL_EVIDENCE_CONTRACT_ABSENT"
CATEGORY_LEGACY_IDENTITY = "PRE_EXISTING_LEGACY_V2_TRAJECTORY_IDENTITY"
CATEGORY_COLLECTION = "PRE_EXISTING_COLLECTION_ERROR"

RECORDED_FAILURES: tuple[tuple[str, str], ...] = (
    ("tests/test_res54_true_com_velocity.py::test_event_rebase_determinism",
     CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_01_exact_e10_restore", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_03_trunk_rate_semantics", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_04_cop_support_frame", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_05_centroidal_wrench_identity",
     CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_06_local_effectiveness", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_10_rr_spec_immutable", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_11_e11_alone_insufficient",
     CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res73_balance_capture.py::test_13_evidence_contract",
     CATEGORY_EVIDENCE_CONTRACT),
    ("tests/test_res74_stable_recovery.py::test_01_rr_confirmed_start", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res74_stable_recovery.py::test_03_handoff_spec_immutable",
     CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res74_stable_recovery.py::test_04_fixed_foot_path", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res74_stable_recovery.py::test_07_timescaling_identity",
     CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res74_stable_recovery.py::test_12_e12_identity_and_post_hold",
     CATEGORY_EXTERNAL_SESSION),
    ("tests/test_res74_stable_recovery.py::test_13_evidence_contract",
     CATEGORY_EVIDENCE_CONTRACT),
    ("tests/test_res76_full_closure.py::test_05_rr_dwell_to_recovery_init",
     CATEGORY_LEGACY_IDENTITY),
    ("tests/test_res76_full_closure.py::test_07_res43_one_way_handoff", CATEGORY_LEGACY_IDENTITY),
    ("tests/test_res76_full_closure.py::test_11_handoff_transfer", CATEGORY_EXTERNAL_SESSION),
    ("tests/test_v2_1_res16_true_standing.py::test_12_E1_E11_unchanged",
     CATEGORY_LEGACY_IDENTITY),
    ("tests/test_v2_1_res31_honest_planar_root.py::test_pre_limit_trajectory_identity",
     CATEGORY_LEGACY_IDENTITY),
    ("tests/test_v2_1_res43_true_standing_rebase.py::test_16_controller_unchanged",
     CATEGORY_LEGACY_IDENTITY),
    ("tests/test_v2_1_res5_honest_fall.py::test_fall_shell_physical_contact",
     CATEGORY_LEGACY_IDENTITY),
    ("tests/test_v2_1_res6_compliant_landing_contact.py::test_standing_pass",
     CATEGORY_LEGACY_IDENTITY),
    ("tests/test_v2_1_res8_captured_squat.py::test_capture_target_from_proven_state",
     CATEGORY_LEGACY_IDENTITY),
    ("tests/test_v2_1_res8_captured_squat.py::test_E11_before_fall", CATEGORY_LEGACY_IDENTITY),
)

RECORDED_COLLECTION_ERRORS: tuple[dict[str, str], ...] = (
    {
        "module": "tests/test_ml241_qacc_resolution.py",
        "error": "FileNotFoundError: external session path .../F4-ORACLE-INFRASTRUCTURE/"
                 "ML241-WRAPPED-DERIVATIVES/.../qacc_resolution_summary.json",
        "category": CATEGORY_COLLECTION,
    },
    {
        "module": "tests/test_public_support_wrench_contract.py",
        "error": "ImportError: cannot import name 'shift_wrench_to_origin' from "
                 "loaded_cmj.simulation.plant",
        "category": CATEGORY_COLLECTION,
    },
)

RECORDED_ENTRY_HEAD_REPRODUCTION = {
    "worktree_head": "b0eccb8a6eeda40950665b3be517854f8b48baab",
    "same_failures_reproduced": 24,
    "path_dependent_test": (
        "tests/test_v2_1_res16_true_standing.py::test_01_captured_satisfies_old_but_fails_new"
    ),
    "path_dependent_explanation": (
        "the test resolves the external evidence root relative to the repository "
        "parent; it fails in the temporary worktree and passes in the primary "
        "checkout where the evidence root exists"
    ),
}

RECORDED_RES85_OWNED_TESTS = {
    "tests/test_res85_v3_causal_launch.py": "PASS",
    "tests/test_res85_authority_freeze.py": "PASS",
    "tests/test_res84_v3_measurement_contact.py": "PASS",
    "tests/test_res83_v3_plant.py": "PASS",
}


def build() -> dict:
    failures = [{"test": test, "category": category} for test, category in RECORDED_FAILURES]
    categories: dict[str, int] = {}
    for _, category in RECORDED_FAILURES:
        categories[category] = categories.get(category, 0) + 1
    return {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001",
        "commands": {
            "full_suite": (
                "pytest tests/ -q -p no:randomly "
                "--ignore=tests/test_ml241_qacc_resolution.py "
                "--ignore=tests/test_public_support_wrench_contract.py"
            ),
            "targeted_res85": (
                "pytest tests/test_res85_v3_causal_launch.py "
                "tests/test_res85_authority_freeze.py"
            ),
            "res84_regression": "pytest tests/test_res84_v3_measurement_contact.py",
            "res83_plant": "pytest tests/test_res83_v3_plant.py",
            "entry_head_reproduction": (
                "git worktree add <tmp> b0eccb8a && pytest <same files> -q --tb=no"
            ),
        },
        "full_suite": {
            "collected": 859,
            "passed": 834,
            "failed": 25,
            "collection_error_modules": 2,
            "res85_owned_failures": 0,
            "res83_res84_failures": 0,
        },
        "targeted_and_regression": RECORDED_RES85_OWNED_TESTS,
        "failures": failures,
        "failure_categories": categories,
        "collection_errors": [dict(e) for e in RECORDED_COLLECTION_ERRORS],
        "entry_head_reproduction": RECORDED_ENTRY_HEAD_REPRODUCTION,
        "verdict": (
            "ALL RECORDED FAILURES PRE-EXIST AT ENTRY_HEAD AND ARE CLASSIFIED AS "
            "ENVIRONMENTAL OR LEGACY-V2; NONE IS INTRODUCED BY RES-85"
        ),
    }


def main(argv: list[str] | None = None) -> int:
    report = build()
    assert len(report["failures"]) == report["full_suite"]["failed"] == 25
    assert report["full_suite"]["res85_owned_failures"] == 0
    OUT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"classified {len(report['failures'])} failures -> {OUT.name}")
    for category, count in sorted(report["failure_categories"].items()):
        print(f"  {category}: {count}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
