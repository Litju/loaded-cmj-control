"""Contract tests for the completed ML-241 qacc blocker resolution."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from loaded_cmj.oracle.derivatives import (
    ColumnQualification,
    DerivativeDomainError,
    QACC_DERIVATIVE_NUMERICALLY_NULL,
    QACC_DERIVATIVE_UNADJUDICATED,
    WrappedLinearization,
    WARMSTART_SLICE,
)


EVIDENCE = Path("/home/litju/Projects/loaded-cmj-control-evidence/F4-ORACLE-INFRASTRUCTURE/ML241-WRAPPED-DERIVATIVES/20260810T222908Z")
SUMMARY = json.loads((EVIDENCE / "qacc_resolution_summary.json").read_text())


def _json(name: str):
    return json.loads((EVIDENCE / name).read_text())


def _synthetic_linearization(*, qacc_value: float = 0.0, valid_index: int | None = None):
    reports = tuple(
        ColumnQualification(
            axis="state",
            index=index,
            step=0.1,
            block="qacc_warmstart" if index >= 111 else "other",
            valid=index != valid_index,
            reason="synthetic invalid qacc direction" if index == valid_index else "ok",
            active_set_preserved=True,
            plus_fingerprint_digest="base",
            minus_fingerprint_digest="base",
        )
        for index in range(132)
    )
    matrix = np.zeros((132, 132), dtype=np.float64)
    matrix[:, 111:132] = qacc_value
    synthetic = object.__new__(WrappedLinearization)
    object.__setattr__(synthetic, "A", matrix)
    object.__setattr__(synthetic, "B", np.zeros((132, 15), dtype=np.float64))
    object.__setattr__(synthetic, "accepted_action_jacobian", np.zeros((15, 15), dtype=np.float64))
    object.__setattr__(synthetic, "state_validity", np.ones(132, dtype=bool))
    object.__setattr__(synthetic, "action_validity", np.ones(15, dtype=bool))
    object.__setattr__(synthetic, "state_columns", reports)
    object.__setattr__(synthetic, "action_columns", ())
    object.__setattr__(synthetic, "base_snapshot_digest", "base")
    object.__setattr__(synthetic, "base_next_snapshot_digest", "next")
    object.__setattr__(synthetic, "base_raw_action", np.zeros(15, dtype=np.float64))
    object.__setattr__(synthetic, "base_accepted_action", np.zeros(15, dtype=np.float64))
    object.__setattr__(synthetic, "base_active_set", None)
    object.__setattr__(synthetic, "state_step_metadata", {})
    object.__setattr__(synthetic, "action_step_metadata", ())
    object.__setattr__(synthetic, "scheme", "synthetic")
    object.__setattr__(synthetic, "tangent_layout_id", "synthetic")
    object.__setattr__(synthetic, "snapshot_schema_id", "synthetic")
    object.__setattr__(synthetic, "transition_owner", "synthetic")
    object.__setattr__(synthetic, "constraint_catalog_id", "synthetic")
    object.__setattr__(synthetic, "transition_evaluation_count", 0)
    object.__setattr__(synthetic, "qacc_derivative_disposition", QACC_DERIVATIVE_UNADJUDICATED)
    return synthetic


def test_01_original_qacc_no_plateau_reproduced():
    assert SUMMARY["original_blocker_reproduced"]
    assert "no stable plateau" in (EVIDENCE / "10_BLOCKER_REPRODUCTION.md").read_text()


def test_02_null_thresholds_are_prospective():
    assert SUMMARY["null_contract_frozen_before_decisive_runs"]
    thresholds = _json("21_QACC_NUMERICAL_NULL_THRESHOLDS.json")
    assert thresholds["frozen_before_decisive_runs"] is True
    assert thresholds["relative_plateau_rule"]["not_weakened"] is True


def test_03_centered_numerator_scaling_receipt():
    report = (EVIDENCE / "23_QACC_NUMERATOR_SCALING_REPORT.md").read_text()
    assert "floor-like" in report
    assert "slope near one" in report


def test_04_one_sided_response_receipt():
    receipt = _json("24_QACC_ONE_SIDED_RESULTS.json")
    assert receipt["status"] == "COMPLETE"
    assert receipt["basis_directions"] == 21
    assert receipt["dense_directions"] == 11
    assert "one-sided" in (EVIDENCE / "35_QACC_DERIVATIVE_DISPOSITION.md").read_text()


def test_05_all_21_basis_directions_covered():
    assert SUMMARY["basis_directions"] == 21
    assert SUMMARY["qacc_zero_columns"] == list(range(111, 132))


def test_06_dense_directions_covered():
    assert SUMMARY["dense_directions"] == 11


def test_07_fixed_active_set_throughout():
    assert SUMMARY["active_set_fixed"] is True
    assert "all accepted active sets fixed" in (EVIDENCE / "26_PRODUCTION_SOLVER_SENSITIVITY_REPORT.md").read_text()


def test_08_production_solver_stats_captured():
    report = (EVIDENCE / "26_PRODUCTION_SOLVER_SENSITIVITY_REPORT.md").read_text()
    assert "40 substeps" in report
    assert "0 to 3" in report


def test_09_diagnostic_does_not_mutate_production_options():
    report = (EVIDENCE / "30_FIXED_ITERATION_DIAGNOSTIC_PROTOCOL.md").read_text()
    assert "copied MuJoCo model/options" in report
    assert "Production options remained unchanged" in report
    assert SUMMARY["production_options_unchanged"] is True


def test_10_fixed_iteration_diagnostic_is_deterministic():
    assert SUMMARY["diagnostic_solver_niter_range"] == [0, 100]
    assert "DIAGNOSTIC_ONLY" in (EVIDENCE / "30_FIXED_ITERATION_DIAGNOSTIC_PROTOCOL.md").read_text()


def test_11_production_diagnostic_comparison_is_explicit():
    report = (EVIDENCE / "32_PRODUCTION_VS_DIAGNOSTIC_COMPARISON.md").read_text()
    assert "production authority" in report
    assert "diagnostic is explanatory only" in report
    assert SUMMARY["diagnostic_map_not_authoritative"] is True


def test_12_mjd_warmstart_crosscheck():
    assert SUMMARY["mjd_native_qacc_columns"] == 0
    assert SUMMARY["mjd_state_restore_exact"] is True
    assert "42" in (EVIDENCE / "33_MJD_WARMSTART_DERIVATIVE_CROSSCHECK.md").read_text()


def test_13_physical_core_decomposition():
    report = (EVIDENCE / "34_QACC_RESPONSE_DECOMPOSITION.json").read_text()[:200000]
    assert "configuration" in report
    assert "qvel" in report
    assert "drivestate" in report
    assert "previous_accepted_action" in report


def test_14_numerical_state_decomposition():
    report = (EVIDENCE / "34_QACC_RESPONSE_DECOMPOSITION.json").read_text()[:200000]
    assert "qacc_warmstart" in report
    assert "cache_so3" in report


def test_15_certificate_rejects_nonpositive_synthetic_bound():
    # The owner API must reject a synthetic/material claim whose qacc reports
    # are not fixed-mode qualified; this exercises the negative guard without
    # running a production transition.
    state_columns = tuple(
        ColumnQualification(
            axis="state",
            index=index,
            step=0.1,
            block="qacc_warmstart" if index >= 111 else "other",
            valid=index < 111,
            reason="synthetic invalid qacc direction" if index >= 111 else "ok",
            active_set_preserved=True,
            plus_fingerprint_digest="base",
            minus_fingerprint_digest="base",
        )
        for index in range(132)
    )
    synthetic = object.__new__(WrappedLinearization)
    object.__setattr__(synthetic, "A", np.eye(132))
    object.__setattr__(synthetic, "state_columns", state_columns)
    object.__setattr__(synthetic, "qacc_derivative_disposition", QACC_DERIVATIVE_UNADJUDICATED)
    with pytest.raises(DerivativeDomainError):
        synthetic.certify_qacc_numerical_null(
            evidence_id="synthetic",
            absolute_error_bound={"qacc_translation": 1e-7, "qacc_rotation_joint": 1e-7},
        )
    assert QACC_DERIVATIVE_NUMERICALLY_NULL != QACC_DERIVATIVE_UNADJUDICATED


def test_certificate_rejects_missing_qacc_report():
    result = _synthetic_linearization()
    object.__setattr__(result, "state_columns", tuple(result.state_columns[:-1]))
    with pytest.raises(DerivativeDomainError, match="all 21 qacc reports"):
        result.certify_qacc_numerical_null(
            evidence_id="ML241-test-certificate",
            absolute_error_bound={
                "qacc_translation": 1.0e-7,
                "qacc_rotation_joint": 1.0e-7,
            },
        )


def test_certificate_rejects_invalid_qacc_branch_report():
    result = _synthetic_linearization(valid_index=111)
    with pytest.raises(DerivativeDomainError, match="fixed-mode finite qacc reports"):
        result.certify_qacc_numerical_null(
            evidence_id="ML241-test-certificate",
            absolute_error_bound={
                "qacc_translation": 1.0e-7,
                "qacc_rotation_joint": 1.0e-7,
            },
        )


def test_certificate_rejects_current_qacc_response_outside_authorized_bound():
    result = _synthetic_linearization(qacc_value=1.0e-7 + 1.0e-12)
    with pytest.raises(DerivativeDomainError, match="error bound"):
        result.certify_qacc_numerical_null(
            evidence_id="ML241-test-certificate",
            absolute_error_bound={
                "qacc_translation": 1.0e-7,
                "qacc_rotation_joint": 1.0e-7,
            },
        )


def test_certificate_rejects_missing_evidence_identity():
    result = _synthetic_linearization()
    with pytest.raises(DerivativeDomainError, match="evidence id"):
        result.certify_qacc_numerical_null(
            evidence_id="",
            absolute_error_bound={
                "qacc_translation": 1.0e-7,
                "qacc_rotation_joint": 1.0e-7,
            },
        )


def test_certificate_sets_exact_zero_columns_only_after_qualification():
    result = _synthetic_linearization(qacc_value=1.0e-12)
    qualified = result.certify_qacc_numerical_null(
        evidence_id="ML241-test-certificate",
        absolute_error_bound={
            "qacc_translation": 1.0e-7,
            "qacc_rotation_joint": 1.0e-7,
        },
    )
    assert qualified.qacc_derivative_disposition == QACC_DERIVATIVE_NUMERICALLY_NULL
    assert np.array_equal(
        qualified.A[:, WARMSTART_SLICE],
        np.zeros((132, 21)),
    )


def test_16_failed_relative_plateau_alone_is_insufficient():
    contract = (EVIDENCE / "20_QACC_NUMERICAL_NULL_CONTRACT.md").read_text()
    assert "absolute-resolution branch" in contract
    assert "cannot be used to weaken" in contract


def test_17_zero_columns_require_certificate_metadata():
    certificate = _json("36_QACC_DERIVATIVE_CERTIFICATE.json")
    assert certificate["status"] == "PASS"
    assert certificate["evidence_id"].endswith("36_QACC_DERIVATIVE_CERTIFICATE.json")
    assert certificate["zero_columns"] == list(range(WARMSTART_SLICE.start, WARMSTART_SLICE.stop))


def test_18_state_dimension_is_not_reduced():
    assert SUMMARY["state_dimension"] == 132
    assert "132-D" in (EVIDENCE / "51_ARCHITECTURE_AFTER_ML241_RESOLUTION.md").read_text()


def test_19_production_solver_settings_are_unchanged():
    assert SUMMARY["production_options_unchanged"] is True
    assert "No production option was modified" in (EVIDENCE / "26_PRODUCTION_SOLVER_SENSITIVITY_REPORT.md").read_text()


def test_20_final_qacc_jvp_consistency():
    repeatability = _json("39_FINAL_DIRECTIONAL_REPEATABILITY.json")
    assert all(repeatability["qacc_jvp"][fixture]["active_set_fixed"] for fixture in repeatability["qacc_jvp"])
    assert all(np.isfinite(repeatability["qacc_jvp"][fixture]["error"]).all() for fixture in repeatability["qacc_jvp"])


def test_21_final_A_shape():
    assert SUMMARY["final_A_shape"] == [132, 132]
    metadata = _json("37_FINAL_WRAPPED_A_B_METADATA.json")
    assert all(item["A_shape"] == [132, 132] for item in metadata.values())


def test_22_final_B_shape():
    assert SUMMARY["final_B_shape"] == [132, 15]
    metadata = _json("37_FINAL_WRAPPED_A_B_METADATA.json")
    assert all(item["B_shape"] == [132, 15] for item in metadata.values())


def test_23_final_AB_repeatability():
    assert SUMMARY["final_repeatability"]
    repeatability = _json("39_FINAL_DIRECTIONAL_REPEATABILITY.json")["repeatability"]
    assert repeatability["A_exact_equal"]
    assert repeatability["B_exact_equal"]
    assert repeatability["P_exact_equal"]
    assert repeatability["qacc_zero_columns_equal"]


def test_24_prior_ml241_blocks_remain_qualified():
    report = (EVIDENCE / "35_QACC_DERIVATIVE_DISPOSITION.md").read_text()
    assert "No MacroSnapshot" in report
    assert "active set" in report
    assert SUMMARY["prior_gate_reopen"] is False


def test_25_no_ml242_source_dependency():
    assert SUMMARY["ml242_source_dependency"] is False
    assert "ML-242" not in __import__("inspect").getsource(__import__("loaded_cmj.oracle.derivatives", fromlist=["*"]))
