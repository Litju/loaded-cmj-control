"""RES-85 Achievement A tests — frozen scientific control authority.

Authority: LCMJ_RES85_*_V1
Mission:   RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001

Every assertion re-executes the deterministic computation archived in
``audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001/``.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

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


B = _load_module("res85_build_authority_checks", EVIDENCE_DIR / "build_authority_checks.py")


def _authority(name: str) -> dict:
    return json.loads((EVIDENCE_DIR / name).read_text())


# ---------------------------------------------------------------------------
# deterministic check report
# ---------------------------------------------------------------------------
def test_authority_bundle_files_present():
    for name in ("METHOD_COMPARATOR_PANEL.json", "ACTUATION_AUTHORITY.json",
                 "MTP_ENERGY_AUTHORITY.json", "PHASE_MACHINE_AUTHORITY.json",
                 "RES85A_AUTHORITY_RECEIPT.md", "build_authority_checks.py",
                 "seal_evidence.py"):
        assert (EVIDENCE_DIR / name).is_file(), name


def test_authority_check_report_passes():
    artifacts = B.build_all()
    report = artifacts["check_report"]
    assert report["status"] == "PASS", artifacts["redteam"]["checks"]
    for name, sub in report["sub_reports"].items():
        assert sub["status"] == "PASS", (name, sub)
    assert report["redteam_status"] == "PASS", artifacts["redteam"]


def test_redteam_checks_are_named_and_clean():
    artifacts = B.build_all()
    names = {c["check"] for c in artifacts["redteam"]["checks"]}
    assert "no_copied_r001_literals" in names
    assert "no_copied_v2_torque_ceilings" in names
    assert "no_hidden_height_targets" in names
    assert "method_separation_declared" in names
    assert "mtp_energy_not_double_counted" in names
    assert "nominal_control_symmetric" in names
    assert "no_duplicated_event_definitions" in names
    assert "no_time_programmed_primary_transitions" in names
    assert all(c["pass"] for c in artifacts["redteam"]["checks"])
    assert artifacts["redteam"]["status"] == "PASS"


# ---------------------------------------------------------------------------
# H2 authority
# ---------------------------------------------------------------------------
def test_h2_authority_is_frozen_and_method_qualified():
    panel = _authority("METHOD_COMPARATOR_PANEL.json")
    h2 = panel["h2_authority"]
    assert h2["definition"] == "SYSTEM_COM_z(APEX) - SYSTEM_COM_z(TAKEOFF_OCCURRENCE)"
    assert h2["origin_authority"] == "LCMJ_RES84_V3_MEASUREMENT_CONTACT_AUTHORITY_V1"
    assert h2["elite_soccer_plus20_h2_hard_gate"] == "NOT_ESTABLISHED"
    ref = h2["anti_triviality_reference_only"]
    assert ref["value_m"] == 0.150
    assert ref["label"] == "HISTORICAL_ANTI_TRIVIALITY_NEGATIVE_CONTROL_ONLY"
    assert "not an elite-performance target" in ref["role_never"]


def test_no_method_is_converted_into_a_single_target():
    panel = _authority("METHOD_COMPARATOR_PANEL.json")
    assert panel["conversion_policy"]["cross_method_conversion_status"] == "NOT_ESTABLISHED"
    assert panel["conversion_policy"]["single_target_conversion"] == "PROHIBITED"
    ids = [m["method_id"] for m in panel["methods"]]
    assert ids == ["DIRECT_SIMULATOR_SYSTEM_COM", "FORCE_PLATFORM_IMPULSE_MOMENTUM",
                   "FORCE_PLATFORM_FLIGHT_TIME", "BAR_LVT_DISPLACEMENT_VELOCITY"]


# ---------------------------------------------------------------------------
# actuation authority
# ---------------------------------------------------------------------------
def test_nine_channels_frozen_with_ceilings():
    auth = _authority("ACTUATION_AUTHORITY.json")
    channels = {c["channel"]: c for c in auth["channels"]}
    assert list(channels) == ["trunk_pelvis", "left_hip", "right_hip", "left_knee",
                              "right_knee", "left_ankle", "right_ankle",
                              "left_mtp", "right_mtp"]
    expected = {
        "trunk_pelvis": (220.0, 600.0, 3000.0),
        "left_hip": (330.0, 1600.0, 6000.0),
        "right_hip": (330.0, 1600.0, 6000.0),
        "left_knee": (380.0, 1700.0, 8000.0),
        "right_knee": (380.0, 1700.0, 8000.0),
        "left_ankle": (260.0, 1100.0, 6000.0),
        "right_ankle": (260.0, 1100.0, 6000.0),
        "left_mtp": (45.0, 120.0, 1500.0),
        "right_mtp": (45.0, 120.0, 1500.0),
    }
    for name, (moment, power, rate) in expected.items():
        ch = channels[name]
        assert ch["moment_ceiling_nm"] == moment
        assert ch["power_ceiling_w"] == power
        assert ch["torque_rate_ceiling_nm_per_s"] == rate
        assert ch["provenance_class"] == "ENGINEERING_NOMINAL_WITH_SENSITIVITY"
        assert len(ch["sensitivity_values"]) >= 3


def test_v2_and_r001_values_are_not_imported():
    auth = _authority("ACTUATION_AUTHORITY.json")
    assert auth["forbidden_imports"]["v2_torque_limits"].startswith("V2_TORQUE_LIMITS_NM")
    ceilings = [c["moment_ceiling_nm"] for c in auth["channels"]]
    assert 250.0 not in ceilings and 300.0 not in ceilings and 200.0 not in ceilings


def test_enforcement_order_and_symmetry_are_frozen():
    auth = _authority("ACTUATION_AUTHORITY.json")
    assert auth["enforcement_order_frozen"] is True
    assert auth["enforcement_order"][2].startswith("3_constraint_interval_construction")
    assert any("sole history state" in entry for entry in auth["enforcement_order"])
    assert "projection onto the intersection of four intervals" in \
        auth["feasible_interval_note"]
    assert auth["bilateral_symmetry_rules"]["nominal_mode"] == "ENFORCED_FOR_ALL_RES85_PHASES"
    assert auth["bilateral_symmetry_rules"]["mirrored_pairs"] == [
        ["left_hip", "right_hip"], ["left_knee", "right_knee"],
        ["left_ankle", "right_ankle"], ["left_mtp", "right_mtp"]]


# ---------------------------------------------------------------------------
# MTP energy authority
# ---------------------------------------------------------------------------
def test_mtp_ledgers_are_separate_and_identified():
    auth = _authority("MTP_ENERGY_AUTHORITY.json")
    assert auth["definitions"]["active_power"]["definition"] == "tau_active * qdot"
    assert auth["definitions"]["passive_power"]["definition"] == "tau_passive * qdot"
    assert "P_active + P_passive" in auth["definitions"]["total_power"]["identity"]
    assert "W_active + W_passive" in auth["definitions"]["total_work"]["identity"]
    assert len(auth["double_counting_prohibitions"]) == 4


def test_mtp_budgets_and_anti_injection_rule_frozen():
    auth = _authority("MTP_ENERGY_AUTHORITY.json")
    assert auth["budgets_per_foot"]["active_positive_work_budget_j"] == 25.0
    assert auth["budgets_per_foot"]["total_positive_work_budget_j"] == 40.0
    assert auth["late_phase_restriction"]["late_active_positive_work_fraction"] == 0.5
    rule = auth["anti_energy_injection_rule"]
    assert rule["rule_id"] == "AEI-1"
    assert len(rule["enforcement"]) == 6
    assert "gated to exactly 0.0" in rule["enforcement"][3]
    assert "budgets in this authority are model-independent" in \
        auth["zero_passive_sensitivity"]["rule"]


def test_active_mtp_denied_after_support_loss():
    auth = _authority("MTP_ENERGY_AUTHORITY.json")
    gate = auth["supported_phase_active_authority"]["phase_gate"]
    assert gate["TAKEOFF_CONFIRM"]["active_allowed"] is False
    assert gate["FLIGHT"]["active_allowed"] is False
    assert gate["LANDING_PREP"]["active_allowed"] is False


# ---------------------------------------------------------------------------
# phase machine authority
# ---------------------------------------------------------------------------
def test_phase_machine_states_and_order():
    auth = _authority("PHASE_MACHINE_AUTHORITY.json")
    assert [s["state"] for s in auth["states"]] == [
        "STAND", "COUNTERMOVEMENT", "BRAKING", "PROPULSION",
        "TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP"]


def test_transitions_are_causal_not_clocked():
    auth = _authority("PHASE_MACHINE_AUTHORITY.json")
    by_state = {s["state"]: s for s in auth["states"]}
    assert by_state["BRAKING"]["wall_clock_role"] == "none"
    assert by_state["PROPULSION"]["wall_clock_role"] == "none"
    assert by_state["TAKEOFF_CONFIRM"]["wall_clock_role"] == "none"
    assert by_state["FLIGHT"]["wall_clock_role"] == "none"
    assert by_state["LANDING_PREP"]["wall_clock_role"] == "none"
    assert "SYSTEM_COM_vz >= 0.0" in json.dumps(by_state["BRAKING"]["hysteresis_debounce"])
    assert by_state["PROPULSION"]["hysteresis_debounce"]["type"] == "MEASUREMENT_EVENT_ONLY"
    assert by_state["TAKEOFF_CONFIRM"]["hysteresis_debounce"]["dwell_s"] == 0.050


def test_flight_latch_requires_res84_confirmation_and_recontact_reverts():
    auth = _authority("PHASE_MACHINE_AUTHORITY.json")
    confirm = [s for s in auth["states"] if s["state"] == "TAKEOFF_CONFIRM"][0]
    assert "TAKEOFF_CONFIRMATION confirmed == true" in json.dumps(confirm["exit_predicates"])
    assert confirm["reversion_mapping"]["recontact_with_vz_positive"] == "PROPULSION"
    assert confirm["reversion_mapping"]["recontact_with_vz_nonpositive"] == "BRAKING"
    flight = [s for s in auth["states"] if s["state"] == "FLIGHT"][0]
    assert "cannot be un-entered" in flight["entry"]


def test_global_prohibitions_are_declared():
    auth = _authority("PHASE_MACHINE_AUTHORITY.json")
    glob = auth["global_rules"]
    assert "may not read scorer internals" in glob["no_scorer_private_memory"]["rule"]
    assert "may implement minimum dwell/debounce only" in glob["wall_clock_prohibition"]["statement"]
    assert "no local re-implementation" in glob["duplicated_event_definitions"]["prohibition"]
    assert "raises/returns an explicit failure state" in glob["fail_closed"]["rule"]


# ---------------------------------------------------------------------------
# provenance
# ---------------------------------------------------------------------------
def test_source_provenance_pins_sealed_inputs():
    provenance = B.source_provenance()
    assert provenance["plant"]["matches_res83_seal"] is True
    assert provenance["plant"]["res83_sealed_sha256"] == \
        "eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d"
    assert provenance["measurement_authority"]["external_evidence_seal_sha256"] == \
        "223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda"
    assert provenance["measurement_authority"]["authority_id"] == \
        "LCMJ_RES84_V3_MEASUREMENT_CONTACT_AUTHORITY_V1"
    for name in ("METHOD_COMPARATOR_PANEL.json", "ACTUATION_AUTHORITY.json",
                 "MTP_ENERGY_AUTHORITY.json", "PHASE_MACHINE_AUTHORITY.json"):
        assert len(provenance["authority_files"][name]["sha256"]) == 64


def test_receipt_matches_generated_report():
    text = (EVIDENCE_DIR / "RES85A_AUTHORITY_RECEIPT.md").read_text()
    assert "ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED" in text
    assert "HISTORICAL_ANTI_TRIVIALITY_NEGATIVE_CONTROL_ONLY" in text
    assert "eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d" in text
    assert "223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda" in text
    report = json.loads((EVIDENCE_DIR / "AUTHORITY_CHECK_REPORT.json").read_text())
    assert report["status"] == "PASS"
    assert report["redteam_status"] == "PASS"
