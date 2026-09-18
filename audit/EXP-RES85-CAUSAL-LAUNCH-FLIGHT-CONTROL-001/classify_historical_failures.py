"""Classify the repository-suite failures recorded during the RES-85C qualification.

MISSION: RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001

The RES-85C qualification ran in the primary checkout at the candidate state:

    pytest tests/ -q -p no:randomly \
        --ignore=tests/test_ml241_qacc_resolution.py \
        --ignore=tests/test_public_support_wrench_contract.py
        -> 890 collected, 823 passed, 60 failed, 7 skipped,
           0 RES-85C/RES-85/RES-83/RES-84-owned failures

    2 modules are excluded at collection:
      * tests/test_ml241_qacc_resolution.py (reads an absent external session path)
      * tests/test_public_support_wrench_contract.py (imports a name that does not
        exist in the historical V1 plant module)

Every failure above was reproduced at ENTRY_HEAD
``e487369f6861d9c9bc27f9f3d92b981fb3684293`` in a detached worktree (with the
untracked owner test files copied verbatim): the 25 failures already recorded
during RES-85 reproduce node-for-node and reason-for-reason, and the 35
additional failures from untracked owner-controller tests and absent
external/legacy dependencies reproduce node-for-node.  No failing test imports
any RES-85C change surface (``loaded_cmj.v3``).

This script re-emits the classification deterministically from the recorded
runs; it never re-runs the suite (a ~1-hour operation) and it never hides a
failure.
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
CATEGORY_OWNER_INTERFACE = "PRE_EXISTING_OWNER_CONTROLLER_INTERFACE_MISMATCH"
CATEGORY_SOLVER_BACKEND = "PRE_EXISTING_EXTERNAL_SOLVER_BACKEND_ABSENT"
CATEGORY_ORACLE_IDENTITY = "PRE_EXISTING_LEGACY_ORACLE_TRANSCRIPTION_IDENTITY"
CATEGORY_V1_QUALIFICATION = "PRE_EXISTING_LEGACY_V1_QUALIFICATION"

RECORDED_FAILURES: tuple[tuple[str, str, str], ...] = (
    # --- 25 failures recorded during RES-85 (reproduced at ENTRY_HEAD) ---------
    ("tests/test_res54_true_com_velocity.py::test_event_rebase_determinism",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_01_exact_e10_restore",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_03_trunk_rate_semantics",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_04_cop_support_frame",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_05_centroidal_wrench_identity",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_06_local_effectiveness",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_10_rr_spec_immutable",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_11_e11_alone_insufficient",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res73_balance_capture.py::test_13_evidence_contract",
     CATEGORY_EVIDENCE_CONTRACT, "external evidence bundle contract absent"),
    ("tests/test_res74_stable_recovery.py::test_01_rr_confirmed_start",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res74_stable_recovery.py::test_03_handoff_spec_immutable",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res74_stable_recovery.py::test_04_fixed_foot_path",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res74_stable_recovery.py::test_07_timescaling_identity",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res74_stable_recovery.py::test_12_e12_identity_and_post_hold",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_res74_stable_recovery.py::test_13_evidence_contract",
     CATEGORY_EVIDENCE_CONTRACT, "external evidence bundle contract absent"),
    ("tests/test_res76_full_closure.py::test_05_rr_dwell_to_recovery_init",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_res76_full_closure.py::test_07_res43_one_way_handoff",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_res76_full_closure.py::test_11_handoff_transfer",
     CATEGORY_EXTERNAL_SESSION, "absent external session artifact"),
    ("tests/test_v2_1_res16_true_standing.py::test_12_E1_E11_unchanged",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_v2_1_res31_honest_planar_root.py::test_pre_limit_trajectory_identity",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_v2_1_res43_true_standing_rebase.py::test_16_controller_unchanged",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_v2_1_res5_honest_fall.py::test_fall_shell_physical_contact",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_v2_1_res6_compliant_landing_contact.py::test_standing_pass",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_v2_1_res8_captured_squat.py::test_capture_target_from_proven_state",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    ("tests/test_v2_1_res8_captured_squat.py::test_E11_before_fall",
     CATEGORY_LEGACY_IDENTITY, "legacy V2 trajectory identity"),
    # --- 35 additional failures of untracked owner tests / absent dependencies -
    ("tests/test_progressive_braking_controller.py::test_braking_onset_at_upward_bw_crossing_while_descending",
     CATEGORY_OWNER_INTERFACE, "owner debug_state braking_onset_time_s differs from expected"),
    ("tests/test_progressive_braking_controller.py::test_no_late_depth_takeover_without_weight_crossing",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_progressive_braking_controller.py::test_braking_onset_requires_sustained_countermovement_descent",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_progressive_braking_controller.py::test_no_phase_regression_from_braking",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_progressive_braking_controller.py::test_braking_reversal_enters_propulsion_and_never_returns",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_progressive_braking_controller.py::test_propulsion_flight_landing_prep_absorb_chain",
     CATEGORY_OWNER_INTERFACE, "owner pfip action schema lacks 'braking_onset_time_s'"),
    ("tests/test_progressive_braking_controller.py::test_absorption_to_recovery_after_capture_dwell",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches RECOVERY (phase frontier drift)"),
    ("tests/test_progressive_braking_controller.py::test_flight_actions_are_independent_of_contact_inputs",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches FLIGHT (phase frontier drift)"),
    ("tests/test_progressive_braking_controller.py::test_progressive_ratchet_respects_reachable_increment",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_progressive_braking_controller.py::test_extension_freezes_once_force_requirement_is_met",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_progressive_braking_controller.py::test_impulse_accounting_matches_manual_integration",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_progressive_braking_controller.py::test_progressive_mode_disables_allocator_vertical_vote",
     CATEGORY_OWNER_INTERFACE, "owner pfip allocator signature drift"),
    ("tests/test_progressive_braking_controller.py::test_controller_revision_identifies_candidate",
     CATEGORY_OWNER_INTERFACE, "owner pfip revision is '1.0', test expects R049"),
    ("tests/test_progressive_braking_controller.py::test_full_event_frontier_ordering_is_forward_only",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches ABSORPTION (phase frontier drift)"),
    ("tests/test_progressive_braking_controller.py::test_reset_clears_braking_state",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_hip_braking_polarity.py::test_r049_reference_sources_are_distinct_and_valid",
     CATEGORY_OWNER_INTERFACE, "owner hip policy CONTROLLER_REVISION lacks R049"),
    ("tests/test_hip_braking_polarity.py::test_pre_braking_behavior_is_byte_identical",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_hip_braking_polarity.py::test_only_authorized_channels_diverge_and_knees_never_change",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_hip_braking_polarity.py::test_committed_extension_support_cannot_be_erased_under_deficit",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_hip_braking_polarity.py::test_hip_delta_matches_exact_polarity_correction_while_unclamped",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_hip_braking_polarity.py::test_actions_finite_bounded_through_extended_braking",
     CATEGORY_OWNER_INTERFACE, "owner action schema lacks 'brake_level'"),
    ("tests/test_knee_rate_feasibility.py::test_defect_removed_once_support_is_committed",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_knee_rate_feasibility.py::test_no_unrelated_channel_changes_vs_frozen_r047",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_knee_rate_feasibility.py::test_extension_direction_residual_is_not_blocked",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_knee_rate_feasibility.py::test_accepted_action_slew_still_bounds_knee_delivery",
     CATEGORY_OWNER_INTERFACE, "owner policy never reaches BRAKING (phase frontier drift)"),
    ("tests/test_res51_centroidal_landing.py::test_e8_state_restore_identity",
     CATEGORY_EXTERNAL_SESSION, "absent /tmp/res51/work/E8_BRANCH_STATE.npz"),
    ("tests/test_res52_soft_contact.py::test_18_profile_run_safety_gates_short_cell",
     CATEGORY_EXTERNAL_SESSION, "absent /tmp/res52/work/EVENTS.json"),
    ("tests/test_g4r2_composition.py::test_historical_evidence_is_unchanged",
     CATEGORY_EXTERNAL_SESSION,
     "absent external evidence file F4-ORACLE-FEASIBILITY/.../ml212_runner.py"),
    ("tests/test_ml243_ipopt_backend.py::test_synthetic_sparse_linear_known_solution_and_counters",
     CATEGORY_SOLVER_BACKEND, "cyipopt unavailable at the offline solver boundary"),
    ("tests/test_ml243_ipopt_backend.py::test_synthetic_nonlinear_solution_and_ipopt_derivative_checker",
     CATEGORY_SOLVER_BACKEND, "cyipopt unavailable at the offline solver boundary"),
    ("tests/test_ml243_ipopt_backend.py::test_short_same_plant_zero_defect_fixture",
     CATEGORY_SOLVER_BACKEND, "cyipopt unavailable at the offline solver boundary"),
    ("tests/test_ml243_ipopt_backend.py::test_short_same_plant_perturbed_feasibility_restoration",
     CATEGORY_ORACLE_IDENTITY, "legacy transcription state-delta count 2 != 1"),
    ("tests/test_ml243_ipopt_backend.py::test_deterministic_repeated_synthetic_and_same_plant_receipts",
     CATEGORY_ORACLE_IDENTITY, "legacy transcription state-delta count 2 != 1"),
    ("tests/test_ml243_ipopt_backend.py::test_ml241_and_physical_substrate_hashes_are_unchanged_after_reversal_erratum",
     CATEGORY_ORACLE_IDENTITY, "legacy oracle derivatives.py substrate hash mismatch"),
    ("tests/test_public_qualification.py::test_qualification_suite_passes[qualification_mechanics.py]",
     CATEGORY_V1_QUALIFICATION, "legacy V1 mechanics qualification subprocess 39/40"),
)

RECORDED_COLLECTION_ERRORS: tuple[dict[str, str], ...] = (
    {
        "module": "tests/test_ml241_qacc_resolution.py",
        "error": "FileNotFoundError: external session path .../F4-ORACLE-INFRASTRUCTURE/"
                 "ML241-WRAPPED-DERIVATIVES/.../qacc_resolution_summary.json",
        "category": CATEGORY_EXTERNAL_SESSION,
    },
    {
        "module": "tests/test_public_support_wrench_contract.py",
        "error": "ImportError: cannot import name 'shift_wrench_to_origin' from "
                 "loaded_cmj.simulation.plant",
        "category": "PRE_EXISTING_COLLECTION_ERROR",
    },
)

RECORDED_ENTRY_HEAD_REPRODUCTION = {
    "worktree_head": "e487369f6861d9c9bc27f9f3d92b981fb3684293",
    "primary_checkout_head": "e487369f6861d9c9bc27f9f3d92b981fb3684293",
    "recorded_pre_res85_reproduction": {
        "worktree_head": "b0eccb8a6eeda40950665b3be517854f8b48baab",
        "same_failures_reproduced": 24,
        "path_dependent_test": (
            "tests/test_v2_1_res16_true_standing.py::"
            "test_01_captured_satisfies_old_but_fails_new"),
        "path_dependent_explanation": (
            "the test resolves the external evidence root relative to the repository "
            "parent; it fails in the temporary worktree and passes in the primary "
            "checkout where the evidence root exists"),
    },
    "res85c_reproduction": {
        "failures_reproduced_node_for_node": 60,
        "of_which_previously_recorded": 25,
        "of_which_untracked_owner_or_absent_dependency": 35,
        "untracked_owner_test_files_copied_verbatim": [
            "tests/test_progressive_braking_controller.py",
            "tests/test_hip_braking_polarity.py",
            "tests/test_knee_rate_feasibility.py",
            "tests/test_res51_centroidal_landing.py",
            "tests/test_res52_soft_contact.py",
        ],
        "note": "no failing test imports the RES-85C change surface loaded_cmj.v3",
    },
}

RECORDED_RES85_OWNED_TESTS = {
    "tests/test_res83_v3_plant.py": "PASS",
    "tests/test_res84_v3_measurement_contact.py": "PASS",
    "tests/test_res85_authority_freeze.py": "PASS",
    "tests/test_res85_v3_causal_launch.py": "PASS",
    "tests/test_res85c_correction.py": "PASS",
    "tests/test_res85d_strict_rom.py": "PASS",
}

RECORDED_TARGETED_COUNTS = {
    "command": (
        "pytest tests/test_res83_v3_plant.py "
        "tests/test_res84_v3_measurement_contact.py "
        "tests/test_res85_authority_freeze.py "
        "tests/test_res85_v3_causal_launch.py "
        "tests/test_res85c_correction.py "
        "tests/test_res85d_strict_rom.py -q --tb=short"
    ),
    "per_file": {
        "tests/test_res83_v3_plant.py": 31,
        "tests/test_res84_v3_measurement_contact.py": 92,
        "tests/test_res85_authority_freeze.py": 19,
        "tests/test_res85_v3_causal_launch.py": 31,
        "tests/test_res85c_correction.py": 28,
        "tests/test_res85d_strict_rom.py": 21,
    },
    "collected": 222,
    "passed": 222,
    "failed": 0,
    "skipped": 0,
}

# ---------------------------------------------------------------------------
# RES-85D final full-suite classification
#
# The RES-85D candidate was classified by ONE full-suite run of the candidate
# worktree (HEAD ffc98526 + the RES-85D working-tree changes) and by a
# node-for-node ENTRY_HEAD reproduction of every failure.  The candidate
# failure set is identical to the RES-85C historical set (60 nodes), every
# failure reproduces at ENTRY_HEAD with the same controlling cause, and no
# failing test file imports the RES-85D change surface `loaded_cmj.v3`; the
# historical categories therefore carry over unchanged.
# ---------------------------------------------------------------------------
RES85D_RECORDED_RUN: dict | None = {
    "command": (
        "pytest tests/ -p no:randomly "
        "--ignore=tests/test_ml241_qacc_resolution.py "
        "--ignore=tests/test_public_support_wrench_contract.py "
        "--junitxml=<junit.xml> --tb=no -rN"
    ),
    "worktree_head": "ffc98526bd2b0da76dbef50891415ebdb345a1fd",
    "candidate_state": (
        "HEAD ffc98526bd2b0da76dbef50891415ebdb345a1fd plus the RES-85D "
        "working-tree implementation, tests and regenerated evidence"
    ),
    "collected": 912,
    "passed": 845,
    "failed": 60,
    "skipped": 7,
    "errors": 0,
    "ignored_collection_modules": [
        "tests/test_ml241_qacc_resolution.py",
        "tests/test_public_support_wrench_contract.py",
    ],
    "duration_s": 1188.44,
    "res85d_owned_failures": 0,
    "classification_recorded_after_run": True,
}
RES85D_RECORDED_FAILURES: tuple[tuple[str, str, str], ...] = RECORDED_FAILURES
RES85D_ENTRY_HEAD_REPRODUCTION: dict | None = {
    "worktree_head": "ffc98526bd2b0da76dbef50891415ebdb345a1fd",
    "method": (
        "git worktree add --detach <tmp> ffc98526bd2b0da76dbef50891415ebdb345a1fd; "
        "copy the untracked owner test files and untracked owner source/tool "
        "dependencies verbatim; run only the candidate failing node ids with "
        "PYTHONPATH=<worktree>/src so the worktree source wins over the editable "
        "install; pytest <60 node ids> -q --tb=line"
    ),
    "failures_reproduced_node_for_node": 60,
    "same_failure_set": True,
    "untracked_files_copied_verbatim": [
        "tests/test_progressive_braking_controller.py",
        "tests/test_hip_braking_polarity.py",
        "tests/test_knee_rate_feasibility.py",
        "tests/test_res51_centroidal_landing.py",
        "tests/test_res52_soft_contact.py",
        "tests/test_res10_physics_sample_sync.py",
        "tests/test_public_support_wrench_contract.py",
        "src/loaded_cmj/control/gen3_reference.py",
        "src/loaded_cmj/control/res51_policy.py",
        "src/loaded_cmj/control/reference_data/",
        "tools/diagnostic_replay.py",
        "tools/reproduce_res10_sync.py",
        "tools/res51/",
        "tools/run_canonical_rollout.py",
    ],
    "controlling_cause_families": {
        "PRE_EXISTING_EXTERNAL_SESSION_ARTIFACT_ABSENT": 17,
        "PRE_EXISTING_EXTERNAL_EVIDENCE_CONTRACT_ABSENT": 2,
        "PRE_EXISTING_EXTERNAL_SOLVER_BACKEND_ABSENT": 3,
        "PRE_EXISTING_LEGACY_ORACLE_TRANSCRIPTION_IDENTITY": 3,
        "PRE_EXISTING_LEGACY_V1_QUALIFICATION": 1,
        "PRE_EXISTING_LEGACY_V2_TRAJECTORY_IDENTITY": 9,
        "PRE_EXISTING_OWNER_CONTROLLER_INTERFACE_MISMATCH": 25,
    },
    "failing_test_files_importing_res85d_change_surface": 0,
    "note": (
        "no failing test file imports loaded_cmj.v3, the only module carrying "
        "the RES-85D controller change; every failure pre-exists at ENTRY_HEAD"
    ),
}

RES85D_OWNED_PREFIXES = (
    "tests/test_res85d_strict_rom.py",
    "tests/test_res85c_correction.py",
    "tests/test_res85_v3_causal_launch.py",
    "tests/test_res85_authority_freeze.py",
    "tests/test_res84_v3_measurement_contact.py",
    "tests/test_res83_v3_plant.py",
)


def _classify_res85d() -> dict:
    if RES85D_RECORDED_RUN is None:
        return {
            "status": "PENDING_FINAL_FULL_SUITE_RUN",
            "note": ("the RES-85C historical full-suite run is reported; the "
                     "RES-85D candidate full-suite run is recorded at seal time"),
        }
    failures = [{"test": test, "category": category, "reason": reason}
                for test, category, reason in RES85D_RECORDED_FAILURES]
    categories: dict[str, int] = {}
    for _, category, _ in RES85D_RECORDED_FAILURES:
        categories[category] = categories.get(category, 0) + 1
    owned = [f["test"] for f in failures
             if f["test"].startswith(RES85D_OWNED_PREFIXES)]
    return {
        "status": "FINAL",
        "run": RES85D_RECORDED_RUN,
        "failures": failures,
        "failure_categories": categories,
        "res85d_owned_failures": len(owned),
        "res85d_owned_failure_ids": owned,
        "entry_head_reproduction": RES85D_ENTRY_HEAD_REPRODUCTION,
        "verdict": (
            "ALL RECORDED RES-85D FAILURES PRE-EXIST AT ENTRY_HEAD OR ARE "
            "ENVIRONMENTAL/NONSCIENTIFIC; NO RES-85D-OWNED FAILURE"
        ),
    }


def build() -> dict:
    active = RES85D_RECORDED_FAILURES if RES85D_RECORDED_RUN else RECORDED_FAILURES
    failures = [{"test": test, "category": category, "reason": reason}
                for test, category, reason in active]
    categories: dict[str, int] = {}
    for _, category, _ in active:
        categories[category] = categories.get(category, 0) + 1
    res85d = _classify_res85d()
    run = RES85D_RECORDED_RUN or {
        "collected": 890, "passed": 823, "failed": 60, "skipped": 7, "errors": 0}
    return {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001",
        "commands": {
            "full_suite": (
                "pytest tests/ -q -p no:randomly "
                "--ignore=tests/test_ml241_qacc_resolution.py "
                "--ignore=tests/test_public_support_wrench_contract.py"
            ),
            "targeted_res85c": (
                "pytest tests/test_res85c_correction.py "
                "tests/test_res85_v3_causal_launch.py "
                "tests/test_res85_authority_freeze.py"
            ),
            "res84_regression": "pytest tests/test_res84_v3_measurement_contact.py",
            "res83_plant": "pytest tests/test_res83_v3_plant.py",
            "entry_head_reproduction": (
                "git worktree add --detach <tmp> ffc98526bd2b0da76dbef50891415ebdb345a1fd "
                "&& cp <untracked owner tests> <tmp>/tests/ "
                "&& pytest <same node ids> -q --tb=line"
            ),
        },
        "full_suite": {
            "collected": run["collected"],
            "passed": run["passed"],
            "failed": run["failed"],
            "skipped": run["skipped"],
            "errors": run.get("errors", 0),
            "collection_error_modules_excluded": 2,
            "res85d_owned_failures": res85d.get("res85d_owned_failures", 0),
            "res85c_owned_failures": 0,
            "res85_owned_failures": 0,
            "res83_res84_failures": 0,
        },
        "res85c_historical_run": {
            "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
            "full_suite": {
                "collected": 890, "passed": 823, "failed": 60, "skipped": 7,
                "collection_error_modules_excluded": 2,
            },
            "failures": failures,
            "failure_categories": categories,
        },
        "res85d_final_run": res85d,
        "targeted_and_regression": RECORDED_RES85_OWNED_TESTS,
        "targeted_counts": RECORDED_TARGETED_COUNTS,
        "failures": failures,
        "failure_categories": categories,
        "collection_errors": [dict(e) for e in RECORDED_COLLECTION_ERRORS],
        "entry_head_reproduction": RECORDED_ENTRY_HEAD_REPRODUCTION,
        "verdict": (
            "ALL RECORDED FAILURES PRE-EXIST AT ENTRY_HEAD AND ARE CLASSIFIED AS "
            "ENVIRONMENTAL, LEGACY-V2/V1, LEGACY-ORACLE OR UNTRACKED-OWNER-INTERFACE; "
            "NONE IS INTRODUCED BY RES-85C"
        ),
    }


def main(argv: list[str] | None = None) -> int:
    report = build()
    assert len(report["failures"]) == report["full_suite"]["failed"] == 60
    assert report["full_suite"]["res85c_owned_failures"] == 0
    assert report["full_suite"]["res85d_owned_failures"] == 0
    assert report["entry_head_reproduction"]["res85c_reproduction"][
        "failures_reproduced_node_for_node"] == 60
    if report["res85d_final_run"]["status"] == "FINAL":
        assert (len(report["res85d_final_run"]["failures"])
                == report["res85d_final_run"]["run"]["failed"])
    OUT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"classified {len(report['failures'])} failures -> {OUT.name}")
    for category, count in sorted(report["failure_categories"].items()):
        print(f"  {category}: {count}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
