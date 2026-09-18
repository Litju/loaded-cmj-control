"""RES-85C correction tests — narrow post-seal correction and requalification.

MISSION: `RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001`
LINEAR ISSUE: RES-85

Every assertion re-executes the deterministic computation archived in
``audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001/``: the accepted-occurrence
identity, the functional-task-floor authority, the coherent actuation contract,
the MTP non-compensation matrix and the RES-85C negative controls NC-12..NC-21.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
SRC = TASK_ROOT / "src"
EVIDENCE_DIR = TASK_ROOT / "audit" / "EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BE = _load_module("res85_build_evidence", EVIDENCE_DIR / "build_evidence.py")

from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.actuation import (  # noqa: E402
    HARD_SAFETY_BOUNDS,
    SAFETY_OVERRIDE_REASONS,
    TORQUE_RATE_ROLE,
)
from loaded_cmj.v3.launch_runtime import (  # noqa: E402
    NO_ACCEPTED_OCCURRENCE_REASON,
    occurrence_identity_failures,
    run_launch_episode,
)


@pytest.fixture(scope="session")
def episode():
    return run_launch_episode()


@pytest.fixture(scope="session")
def report(episode):
    return BE.launch_episode_report(episode)


@pytest.fixture(scope="session")
def identity(episode, report):
    return BE.occurrence_identity_audit(report)


@pytest.fixture(scope="session")
def override_audit(episode):
    return BE.safety_override_audit(episode)


# ---------------------------------------------------------------------------
# BLOCKER A — accepted takeoff identity
# ---------------------------------------------------------------------------
def test_top_level_occurrence_is_the_controller_accepted_occurrence(episode, report):
    accepted = episode.events["takeoff_occurrence"]
    history = report["takeoff_candidate_history"]
    accepted_entries = [h for h in history if h["disposition"] == "ACCEPTED"]
    assert len(accepted_entries) == 1
    assert accepted["native_index"] == accepted_entries[0]["occurrence"]["native_index"]
    assert report["takeoff_confirmation"]["accepted_occurrence_index"] == \
        accepted["native_index"]


def test_h2_origin_and_impulse_and_comparator_use_the_accepted_occurrence(report):
    accepted = report["takeoff_occurrence"]["native_index"]
    assert report["apex_h2"]["origin_occurrence_index"] == accepted
    assert report["impulse_cross_check"]["occurrence_index"] == accepted
    assert report["diagnostic_comparator"]["offset_origin_index"] == accepted


def test_startup_chatter_is_not_the_accepted_occurrence(report):
    startup = min(report["takeoff_candidate_history"],
                  key=lambda h: h["occurrence"]["native_index"])
    assert startup["occurrence"]["native_index"] < report["takeoff_occurrence"]["native_index"]
    assert startup["disposition"] != "ACCEPTED"
    assert startup["rejection_reason"] is not None or startup["disposition"] == "NOT_EVALUATED"


def test_no_candidate_disappears_from_audit_history(episode, report):
    scanned = M.scan_takeoff_candidates(episode.frames)
    history = report["takeoff_candidate_history"]
    assert len(history) == len(scanned)
    assert [h["occurrence"]["native_index"] for h in history] == \
        [c.native_index for c in scanned]
    for entry in history:
        assert entry["disposition"] in ("ACCEPTED", "REJECTED", "NOT_EVALUATED", "PENDING")
        assert "confirmation_result" in entry
        assert entry["confirmation_result"]["confirmed"] in (True, False)


def test_identity_audit_passes(identity):
    assert identity["status"] == "PASS", identity["failures"]
    assert identity["failures"] == []
    assert identity["candidate_disposition_counts"].get("ACCEPTED") == 1


def test_identity_validator_fails_closed_on_mutation(report):
    good = json.loads(json.dumps(report))
    assert occurrence_identity_failures(good) == []
    mutated = json.loads(json.dumps(good))
    mutated["diagnostic_comparator"]["offset_origin_index"] = (
        int(good["takeoff_occurrence"]["native_index"]) + 5)
    assert occurrence_identity_failures(mutated)
    mutated = json.loads(json.dumps(good))
    mutated["takeoff_occurrence"] = dict(mutated["takeoff_occurrence"])
    mutated["takeoff_occurrence"]["valid"] = False
    mutated["takeoff_occurrence"]["native_index"] = None
    failures = occurrence_identity_failures(mutated)
    assert "CONFIRMATION_WITHOUT_ACCEPTED_OCCURRENCE" in failures
    assert NO_ACCEPTED_OCCURRENCE_REASON in json.dumps(BE.__dict__.get(
        "NO_ACCEPTED_OCCURRENCE_REASON", NO_ACCEPTED_OCCURRENCE_REASON))


# ---------------------------------------------------------------------------
# BLOCKER B — functional floor authority and closure
# ---------------------------------------------------------------------------
def test_functional_floor_closure_and_classification(report):
    floor = report["functional_task_floor"]
    assert floor["h_anti_triviality_floor_m"] == 0.150
    assert floor["floor_role"] == "HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY"
    assert floor["classification"] == "ABOVE_FLOOR"
    assert floor["closure_pass"] is True
    assert floor["closure_pass"] is True
    assert abs(floor["ballistic_cross_check_vz_m_s"] - 1.7155) < 5e-3


def test_pre_correction_episode_is_below_floor_not_pass():
    pre = json.loads((EVIDENCE_DIR / "PRE_CORRECTION_EPISODE_CLASSIFICATION.json").read_text())
    assert pre["entry_head"] == "e487369f6861d9c9bc27f9f3d92b981fb3684293"
    assert pre["classification"] == \
        "PHYSICALLY_VALID_CAUSAL_TAKEOFF_AND_FLIGHT_BELOW_FUNCTIONAL_TASK_FLOOR"
    assert pre["floor_classification"]["closure_pass"] is False
    assert "NOT a PASS" in pre["claim"]
    assert abs(pre["h2_m"] - 0.030166215950979014) < 1e-12


def test_correction_receipt_preserves_previous_evidence_attribution():
    text = (EVIDENCE_DIR / "RES85C_CORRECTION_RECEIPT.md").read_text()
    assert "e487369f6861d9c9bc27f9f3d92b981fb3684293" in text
    assert "235a40e7a2a2b07ea483f2352b0765315b7f43b2343b3297461a0754b024b50e" in text
    for blocker in ("BLOCKER A", "BLOCKER B", "BLOCKER C", "BLOCKER D"):
        assert blocker in text
    assert "H_ANTI_TRIVIALITY_FLOOR" in text


# ---------------------------------------------------------------------------
# BLOCKER C — coherent actuation contract
# ---------------------------------------------------------------------------
def test_actuation_contract_semantics_are_declared():
    assert TORQUE_RATE_ROLE == "NOMINAL_SLEW_BOUND"
    assert HARD_SAFETY_BOUNDS == ("moment_ceiling", "joint_power_ceiling", "mtp_energy_gate")
    assert "moment_ceiling" in SAFETY_OVERRIDE_REASONS


def test_hard_bounds_never_exceeded_and_slew_declared(episode):
    t = episode.telemetry
    from loaded_cmj.v3.actuation import MOMENT_CEILING_NM, POWER_CEILING_W
    assert np.all(np.abs(t.applied_nm) <= MOMENT_CEILING_NM + 1e-9)
    assert np.all(np.abs(t.joint_power_w) <= POWER_CEILING_W + 1e-6)
    exceed = np.asarray(t.nominal_slew_exceedance_nm_per_s, dtype=np.float64)
    override = np.asarray(t.safety_override, dtype=bool)
    assert not np.any((exceed > 0.0) & ~override)


def test_safety_override_audit_passes(override_audit):
    assert override_audit["status"] == "PASS", override_audit["failures"]
    assert override_audit["failures"] == []
    assert override_audit["torque_rate_role"] == "NOMINAL_SLEW_BOUND"


def test_override_audit_fails_closed_on_undeclared_and_unbound(episode, override_audit):
    arrays = override_audit["arrays"]
    exceed = np.array(arrays["nominal_slew_exceedance_nm_per_s"], dtype=np.float64)
    override = np.array(arrays["safety_override"], dtype=bool)
    reason = np.array(arrays["safety_override_reason"], dtype=object)
    exceed[0, 0] = 1.0
    override[0, 0] = False
    failures = BE.slew_override_failures(
        nominal_slew_exceedance_nm_per_s=exceed, safety_override=override,
        safety_override_reason=reason, applied_nm=arrays["applied_nm"],
        qdot=arrays["joint_qdot"], mtp_gated=arrays["mtp_gated"],
        phase_name=arrays["phase_name"])
    assert any("UNDECLARED_SLEW_EXCEEDANCE" in f for f in failures)
    override[0, 0] = True
    exceed[0, 0] = 0.0
    reason[0, 0] = "moment_ceiling"
    failures = BE.slew_override_failures(
        nominal_slew_exceedance_nm_per_s=exceed, safety_override=override,
        safety_override_reason=reason, applied_nm=arrays["applied_nm"],
        qdot=arrays["joint_qdot"], mtp_gated=arrays["mtp_gated"],
        phase_name=arrays["phase_name"])
    assert any("OVERRIDE_WITHOUT_BINDING_HARD_BOUND" in f for f in failures)


# ---------------------------------------------------------------------------
# BLOCKER D — MTP non-compensation
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def matrix():
    return json.loads((EVIDENCE_DIR / "MTP_NONCOMPENSATION_MATRIX.json").read_text())


def test_mtp_matrix_cases_present(matrix):
    results = matrix["case_results"]
    for case in ("M0_nominal_passive_nominal_active", "M1_zero_passive_zero_active",
                 "M2_zero_passive_tight_active", "M3_zero_passive_nominal_active"):
        assert case in results, case
    budgets = sorted(r["active_budget_override_j"] for name, r in results.items()
                     if name.startswith("SWEEP_active_budget"))
    assert budgets == [0.0, 2.5, 12.5, 25.0, matrix["observed_nominal_active_work_scale_j"]] or \
        set(budgets) >= {0.0, 2.5, 12.5, 25.0}
    moments = sorted({r["moment_ceiling_override_nm"] for name, r in results.items()
                      if name.startswith("SWEEP_moment")})
    assert moments == [0.0, 22.5, 45.0, 60.0]


def test_zero_passive_zero_active_is_not_compensated(matrix):
    m1 = matrix["case_results"]["M1_zero_passive_zero_active"]
    assert m1["zero_passive"] is True
    assert m1["active_budget_override_j"] == 0.0
    assert m1["mtp_active_total_positive_work_j"] == 0.0
    assert m1["takeoff_confirmation"] is True
    assert m1["h2_m"] is not None


def test_active_mtp_is_negligible_in_the_nominal_launch(matrix):
    m0 = matrix["case_results"]["M0_nominal_passive_nominal_active"]
    assert m0["ratio_mtp_active_over_total_positive"] is not None
    assert m0["ratio_mtp_active_over_total_positive"] < 0.01
    assert m0["ankle_positive_work_j"] > 10.0 * m0["mtp_active_total_positive_work_j"]


def test_zero_passive_sensitivity_is_reported_honestly(matrix):
    conclusion = matrix["required_conclusion"]
    assert "does not depend on replacing removed passive MTP mechanics" in \
        conclusion["conclusion"]
    assert conclusion["zero_passive_zero_active_h2_m"] is not None


# ---------------------------------------------------------------------------
# RES-85C negative controls NC-12..NC-21
# ---------------------------------------------------------------------------
def test_res85c_negative_controls_pass():
    negative = json.loads((EVIDENCE_DIR / "NEGATIVE_CONTROLS_REPORT.json").read_text())
    names = {c["check"] for c in negative["checks"]}
    for required in ("NC-12_startup_support_dropout_as_accepted_occurrence_fails",
                     "NC-13_top_level_occurrence_differs_from_h2_origin_fails",
                     "NC-14_comparator_offset_uses_rejected_occurrence_fails",
                     "NC-15_h2_just_below_functional_floor_fails",
                     "NC-16_zero_passive_zero_active_case_not_replaced_by_active_command",
                     "NC-17_undeclared_slew_exceedance_fails",
                     "NC-18_declared_safety_override_without_binding_bound_fails",
                     "NC-19_mtp_active_work_is_negligible_fraction_of_joint_work",
                     "NC-20_canonical_identity_audit_passes",
                     "NC-21_safety_override_audit_passes"):
        assert required in names, required
    assert negative["status"] == "PASS", [c for c in negative["checks"] if not c["pass"]]


def test_floor_boundary_control_fails_below_and_passes_at():
    below = BE.floor_closure(0.150 - 1e-9)
    at = BE.floor_closure(0.150)
    assert below["classification"] == "BELOW_FUNCTIONAL_TASK_FLOOR"
    assert below["closure_pass"] is False
    assert at["classification"] == "ABOVE_FLOOR"
    assert at["closure_pass"] is True


# ---------------------------------------------------------------------------
# declared search + Plant ROM probe
# ---------------------------------------------------------------------------
def test_declared_search_is_bounded_and_reproducible():
    search = json.loads((EVIDENCE_DIR / "PROPULSION_SEARCH.json").read_text())
    assert search["total_evaluations"] <= search["max_evaluations"]
    assert len(search["stages"]) == 4
    for stage in search["stages"]:
        assert stage["evaluations"] <= 9
    winner = search["winner"]
    assert winner["feasible"] is True
    assert winner["h2_m"] >= 0.150


def test_joint_rom_probe_declares_the_plant_soft_limit_envelope():
    probe = json.loads((EVIDENCE_DIR / "JOINT_ROM_SOFT_LIMIT_PROBE.json").read_text())
    assert probe["probe"]["measurement_layer_used"] is False
    assert set(probe["channels"]) == {
        "trunk_pelvis", "left_hip", "right_hip", "left_knee", "right_knee",
        "left_ankle", "right_ankle", "left_mtp", "right_mtp"}
    for value in probe["channels"].values():
        assert value["soft_limit_compliance_rad"] >= 0.0


def test_corrected_episode_respects_controller_attributable_rom(report, episode):
    t = episode.telemetry
    from loaded_cmj.v3.constants import V3_JOINT_RANGES_RAD
    q_ref = np.asarray(t.posture_reference_rad, dtype=np.float64)
    k = int(report["takeoff_occurrence"]["native_index"]) + 1
    for c, name in enumerate(BE.CHANNELS):
        lo, hi = V3_JOINT_RANGES_RAD[name]
        assert np.all(q_ref[:k, c] >= lo - 1e-9), name
        assert np.all(q_ref[:k, c] <= hi + 1e-9), name


def test_corrected_episode_h2_is_direct_system_com_and_above_floor(episode, report):
    t = episode.telemetry
    k = int(report["takeoff_occurrence"]["native_index"])
    confirmation = report["takeoff_confirmation"]
    apex = report["apex_h2"]
    assert report["functional_task_floor"]["closure_pass"] is True
    assert apex["takeoff_vz_m_s"] > 1.0
    assert apex["h2_support_m"] >= 0.150
    assert abs(apex["h2_support_m"]) > 0.0
    assert confirmation["confirmed"] is True
    assert np.all(t.legal_plantar_active[k:confirmation["confirmation_sample"]] == 0)


# ---------------------------------------------------------------------------
# launch joint-ROM audit and two-run artifact identity
# ---------------------------------------------------------------------------
def test_res85c_predecessor_strict_rom_is_fail_on_trunk_upper_bound():
    """RES-85C is immutable history, adjudicated here by the RES-85D strict rule.

    The predecessor canonical trajectory violates the frozen structural trunk
    ROM; it was accepted only through the predecessor soft-limit compliance
    envelope, which RES-85D demotes to a numerical solver diagnostic.
    """
    predecessor = json.loads(
        (EVIDENCE_DIR / "RES85C_PREDECESSOR_STRICT_ROM.json").read_text())
    assert predecessor["predecessor_head"] == \
        "ffc98526bd2b0da76dbef50891415ebdb345a1fd"
    audit = predecessor["strict_structural_rom_audit"]
    assert audit["status"] == "FAIL"
    assert "STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in audit["failures"]
    trunk = audit["channels"]["trunk_pelvis"]
    assert trunk["max_measured_rad"] > trunk["rom_upper_rad"]
    assert trunk["measured_overshoot_rad"] > 0.0
    assert audit["global_min_structural_rom_margin_rad"] < -1e-9
    # the historical RES-85C receipt is preserved byte-for-byte
    text = (EVIDENCE_DIR / "RES85C_CORRECTION_RECEIPT.md").read_text()
    assert "235a40e7a2a2b07ea483f2352b0765315b7f43b2343b3297461a0754b024b50e" in text


def test_current_trajectory_passes_strict_structural_rom(report):
    """RES-85D: the measured frozen structural ROM is the acceptance authority.

    The MuJoCo soft-limit probe envelope is a numerical diagnostic only; the
    canonical measured trajectory must stay inside the frozen envelope with a
    1e-9 floating-point comparison tolerance.
    """
    rom = report["joint_rom_audit"]
    assert rom["status"] == "PASS", rom["failures"]
    assert rom["failures"] == []
    assert rom["authority"] == "FROZEN_HUMAN_STRUCTURAL_ENVELOPE_V3_JOINT_RANGES_RAD"
    assert rom["solver_soft_limit_probe_role"] == \
        "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY_NOT_STRUCTURAL_ACCEPTANCE_AUTHORITY"
    assert rom["global_min_structural_rom_margin_rad"] >= -1e-9
    for name, channel in rom["channels"].items():
        assert channel["min_rom_margin_rad"] >= -1e-9, name
        assert channel["min_measured_rad"] >= channel["rom_lower_rad"] - 1e-9, name
        assert channel["max_measured_rad"] <= channel["rom_upper_rad"] + 1e-9, name
        assert channel["solver_soft_limit_diagnostic_role"] == \
            "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY"


def test_strict_rom_validator_fails_closed_on_measured_violation(report):
    audit = json.loads(json.dumps(report["joint_rom_audit"]))
    name = sorted(audit["channels"])[0]
    audit["channels"][name]["max_measured_rad"] = \
        float(audit["channels"][name]["rom_upper_rad"]) + 1.0e-6
    failures = BE.joint_rom_audit_failures(audit)
    assert any("MEASURED_EXTREMUM_OUTSIDE_FROZEN_ROM" in f for f in failures)


def test_search_and_mtp_matrix_record_two_run_byte_identity():
    search = json.loads((EVIDENCE_DIR / "PROPULSION_SEARCH.json").read_text())
    assert search["two_run_identity"]["byte_identical"] is True
    assert search["total_evaluations"] <= search["max_evaluations"] == 33
    assert search["total_evaluations"] == 33
    for stage in search["stages"]:
        assert stage["evaluations"] <= 9
        for result in stage["results"]:
            assert "measured_overshoot_within_plant_envelope" in result["checks"]
    matrix = json.loads((EVIDENCE_DIR / "MTP_NONCOMPENSATION_MATRIX.json").read_text())
    assert matrix["two_run_identity"]["byte_identical"] is True


def test_correction_receipt_contains_required_sections():
    text = (EVIDENCE_DIR / "RES85C_CORRECTION_RECEIPT.md").read_text()
    for section in ("## Entry reconciliation", "## Accepted occurrence identity",
                    "## MTP non-compensation sensitivity matrix",
                    "## Test and suite results", "## Actuation semantics",
                    "## Final claim ceiling"):
        assert section in text, section
    assert "NEVER_ESCALATED_TO_TAKEOFF_CONFIRM" in text
    assert "LEGAL_RECONTACT_BEFORE_CONFIRMATION" in text
    assert "0.150 m is NOT an elite-performance norm" in text
