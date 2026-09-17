"""RES-85 Achievement B tests — V3 causal launch/flight control.

Authority: LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1
Mission:   RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001

Every assertion re-executes the deterministic computation archived in
``audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001/`` (the episode, the
telemetry and all mandatory negative controls).
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
    CHANNELS,
    MIRRORED_PAIRS,
    MOMENT_CEILING_NM,
    MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
    N_CHANNELS,
    POWER_CEILING_W,
    RATE_CEILING_NM_PER_S,
    V3ActuationAuthority,
    V3ActuationError,
    V3MtpLedgerEntry,
)
from loaded_cmj.v3.controller import (  # noqa: E402
    V3ControllerFault,
    V3Phase,
)
from loaded_cmj.v3.launch_runtime import run_launch_episode  # noqa: E402
from loaded_cmj.v3.plant import V3Plant  # noqa: E402


@pytest.fixture(scope="session")
def episode():
    return run_launch_episode()


@pytest.fixture(scope="session")
def episode_report(episode):
    return BE.launch_episode_report(episode)


@pytest.fixture(scope="session")
def negative():
    return BE.negative_controls()


# ---------------------------------------------------------------------------
# module boundary and fail-closed discipline
# ---------------------------------------------------------------------------
def test_modules_are_v3_local_and_do_not_import_v1_v2():
    for name in ("actuation.py", "controller.py", "launch_runtime.py"):
        source = (SRC / "loaded_cmj" / "v3" / name).read_text()
        for forbidden in ("loaded_cmj.v2", "loaded_cmj.simulation", "loaded_cmj.control",
                          "loaded_cmj.oracle", "loaded_cmj.runtime", "loaded_cmj.biomechanics"):
            assert forbidden not in source, (name, forbidden)


def test_no_broad_except_zero_action():
    for name in ("actuation.py", "controller.py", "launch_runtime.py"):
        source = (SRC / "loaded_cmj" / "v3" / name).read_text()
        assert "except Exception" not in source
        assert "except BaseException" not in source
        assert "except:" not in source
    controller_src = (SRC / "loaded_cmj" / "v3" / "controller.py").read_text()
    assert "return np.zeros" not in controller_src
    assert "V3ControllerFault" in controller_src


def test_fail_closed_on_malformed_observation(episode):
    plant = V3Plant()
    data = plant.make_data()
    from loaded_cmj.v3.controller import V3LaunchController
    from loaded_cmj.v3.launch_runtime import settle_standing_stance

    settle_standing_stance(plant, data)
    controller = V3LaunchController(plant, data)
    frame = M.native_frame(plant, data, 0, 0.0)
    snapshot = M.measure(plant, data)
    bad = M.V3NativeFrame(**{**frame.__dict__, "com_world_m": (float("nan"), 0.0, 1.0)})
    with pytest.raises(V3ControllerFault):
        controller.update(bad, snapshot, [bad], plant, data)


# ---------------------------------------------------------------------------
# real episode
# ---------------------------------------------------------------------------
def test_episode_reaches_confirmed_takeoff_and_flight(episode):
    assert episode.status == "COMPLETED"
    assert episode.fault is None
    phases = episode.phases_visited
    assert phases[0] == "STAND"
    assert V3Phase.FLIGHT.value in phases
    assert phases.index(V3Phase.FLIGHT.value) < phases.index(V3Phase.LANDING_PREP.value)
    assert episode.events["takeoff_occurrence"]["valid"] is True
    assert episode.events["takeoff_confirmation"]["confirmed"] is True
    assert episode.events["res85_claim_end"] is not None


def test_episode_phase_sequence_is_causal(episode):
    t = episode.telemetry
    reasons = [r for r in t.transition_reason if r]
    assert any("QUIET_SUPPORTED_STATE_ESTABLISHED" in r for r in reasons)
    assert any("PHYSICAL_UPWARD_REVERSAL" in r for r in reasons)
    # the flight latch is justified by the RES-84 confirmation reason
    assert any("RES84_TAKEOFF_CONFIRMATION" in r for r in reasons)
    assert any("TAKEOFF_OCCURRENCE" in r for r in reasons)
    # LANDING_PREP entered from actual descent/apex, never a stored timestamp
    assert any("ACTUAL_DESCENT_OR_PREDICTED_TOUCHDOWN" in r for r in reasons)
    reasons_text = " ".join(reasons)
    for banned in ("SCHEDULE", "REPLAY", "WALL_CLOCK"):
        assert banned not in reasons_text


def test_h2_is_method_explicit_and_closes_the_functional_floor(episode_report):
    report = episode_report
    methods = report["method_explicit_h2_report"]
    assert methods["PRIMARY_CANONICAL"]["method_id"] == "DIRECT_SIMULATOR_SYSTEM_COM"
    h2 = methods["PRIMARY_CANONICAL"]["value_m"]
    assert isinstance(h2, float) and h2 > 0.0
    assert methods["ELITE_SOCCER_PLUS20_H2_HARD_GATE"] == "NOT_ESTABLISHED"
    assert methods["ELITE_SOCCER_PLUS20_H2_TARGET"] == "NOT_ESTABLISHED"
    floor = methods["H_ANTI_TRIVIALITY_FLOOR"]
    assert floor["value_m"] == 0.150
    assert floor["role"] == "HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY"
    assert floor["is_elite_performance_norm"] is False
    assert floor["is_optimization_target"] is False
    assert report["functional_task_floor"]["closure_pass"] is True
    assert report["functional_task_floor"]["classification"] == "ABOVE_FLOOR"
    assert methods["BAR_LVT_DISPLACEMENT_VELOCITY"]["status"] == "NOT_APPLICABLE_NATIVE"
    assert report["apex_h2"]["evaluable"] is True
    ballistic = methods["SECONDARY_CROSS_CHECKS"]["BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ"]
    assert abs(ballistic["residual_m"]) < 0.01
    impulse = methods["SECONDARY_CROSS_CHECKS"]["IMPULSE_DERIVED_TAKEOFF_VELOCITY"]
    assert abs(impulse["residual_m_s"]) < 0.05


def test_telemetry_covers_every_native_sample(episode):
    t = episode.telemetry
    n = len(t.index)
    assert n > 1000
    assert np.array_equal(t.index, np.arange(n))
    for field in t.__dataclass_fields__:
        assert getattr(t, field).shape[0] == n, field
    assert np.all(np.isfinite(t.com_world_m))
    assert np.all(np.isfinite(t.applied_nm))
    assert np.all(np.isfinite(t.joint_q))
    assert np.all(np.isfinite(t.joint_qd))
    assert np.all(np.isfinite(t.joint_limit_margin[np.isfinite(t.joint_limit_margin)]))
    # out-of-plane observables are recorded (and may be NaN when undefined)
    assert t.out_of_plane_fy_n.shape == (n,)
    # MTP active/passive ledgers are separate and the identity holds on the
    # signed integrals
    residual = np.abs(t.mtp_total_work_signed_j
                      - t.mtp_active_work_signed_j
                      - t.mtp_passive_work_signed_j).max()
    assert residual <= 1e-12


def test_plant_passive_mtp_matches_fm09_reconstruction(episode):
    t = episode.telemetry
    assert np.nanmax(t.mtp_passive_reconstruction_residual_nm) <= 1e-9


def test_phase_coverage_of_required_telemetry_blocks(episode):
    t = episode.telemetry
    # commanded/applied/rate/margins
    assert t.commanded_nm.shape == (len(t.index), N_CHANNELS)
    assert t.torque_rate_nm_per_s.shape == (len(t.index), N_CHANNELS)
    assert t.moment_margin_nm.shape == (len(t.index), N_CHANNELS)
    assert t.rate_margin_nm.shape == (len(t.index), N_CHANNELS)
    assert t.power_margin_w.shape == (len(t.index), N_CHANNELS)
    # ground wrench per foot and total
    assert t.left_wrench.shape == (len(t.index), 6)
    assert t.right_wrench.shape == (len(t.index), 6)
    assert t.total_wrench.shape == (len(t.index), 6)
    # support / CoP / clearance
    assert t.cop_validity.shape == (len(t.index),)
    assert t.support_mode.shape == (len(t.index),)
    assert t.left_clearance_m.shape == (len(t.index),)
    # takeoff occurrence / confirmation flags
    assert t.takeoff_confirmed.sum() > 0
    assert (t.takeoff_candidate_index >= 0).sum() > 0


# ---------------------------------------------------------------------------
# mandatory negative controls (each maps to a mission bullet)
# ---------------------------------------------------------------------------
def test_nc01_single_sample_support_dropout_never_enters_flight(negative):
    c = [c for c in negative["checks"] if c["check"].startswith("NC-01")][0]
    assert c["pass"] is True, c["detail"]


def test_nc02_comparator_sample_counts(negative):
    checks = {c["check"]: c for c in negative["checks"]}
    assert checks["NC-02_five_true_comparator_samples_reject"]["pass"] is True
    assert checks["NC-02b_six_true_samples_confirm_comparator_only"]["pass"] is True
    assert checks["NC-02c_comparator_never_defines_takeoff_or_flight"]["pass"] is True
    probes = negative["probes"]
    assert probes["NC-02_five_true_samples"]["required_true_samples"] == 6
    assert probes["NC-02_five_true_samples"]["below_threshold_frames"] == 5
    assert probes["NC-02_five_true_samples"]["status"] == "NOT_TRIGGERED"
    assert probes["NC-02b_six_true_samples"]["below_threshold_frames"] >= 6
    assert probes["NC-02b_six_true_samples"]["status"] == "TRIGGERED"


def test_nc03_recontact_before_50ms_rejects(negative):
    probes = negative["probes"]["NC-03_recontact_before_50ms"]
    assert probes["confirmed"] is False
    assert "no_legal_plantar_recontact" in probes["failed_checks"]
    assert probes["phase_probe"]["flight_latched"] is False


def test_nc04_one_foot_supported_is_not_takeoff(negative):
    probes = negative["probes"]["NC-04_one_foot_supported"]
    assert probes["occurrence_valid"] is False
    assert probes["phase_probe"]["flight_latched"] is False


def test_nc05_insufficient_clearance_cannot_confirm(negative):
    probes = negative["probes"]["NC-05_insufficient_clearance"]
    assert probes["confirmed"] is False
    assert "bilateral_clearance_reaches_guard" in probes["failed_checks"]
    assert probes["phase_probe"]["flight_latched"] is False


def test_nc06_prohibited_contact_rejects_and_fails_closed(negative):
    probes = negative["probes"]["NC-06_prohibited_during_confirmation"]
    assert probes["confirmed"] is False
    assert "no_prohibited_contact" in probes["failed_checks"]
    assert probes["phase_probe"]["flight_latched"] is False
    assert probes["phase_probe"]["fault"] is not None


def test_nc07_nonpositive_occurrence_vz_rejects(negative):
    probes = negative["probes"]["NC-07_nonpositive_occurrence_vz"]
    assert probes["occurrence_valid"] is True
    assert probes["occurrence_vz"] <= 0.0
    assert probes["confirmed"] is False
    assert "system_com_vz_positive" in probes["failed_checks"]
    assert probes["phase_probe"]["flight_latched"] is False


def test_nc08_authority_holds_across_phase_handoffs(episode):
    report = BE.actuation_conformance_report(episode)
    assert report["status"] == "PASS", report["checks"]
    checks = {c["check"]: c for c in report["checks"]}
    assert checks["hard_moment_ceiling_never_exceeded"]["pass"]
    assert checks["hard_power_ceiling_never_exceeded"]["pass"]
    assert checks["nominal_slew_exceedance_zero_unless_safety_override"]["pass"]
    assert checks["phase_handoffs_do_not_create_hidden_slew_violations"]["pass"]
    assert checks["bilateral_applied_symmetry_within_tolerance"]["pass"]
    assert report["torque_rate_role"] == "NOMINAL_SLEW_BOUND"


def test_nc09_previous_applied_torque_is_continuous(episode):
    t = episode.telemetry
    dt = M.NATIVE_DT_S
    delta = np.abs(np.diff(t.applied_nm, axis=0))
    limit = RATE_CEILING_NM_PER_S[None, :] * dt
    exceeded = (delta > limit + 1e-6)
    overrides = np.asarray(t.safety_override, dtype=bool)
    for ch in range(N_CHANNELS):
        for i in range(exceeded.shape[0]):
            if exceeded[i, ch]:
                assert overrides[i + 1, ch], (
                    "undeclared slew exceedance", i, ch, delta[i, ch])
    handoff_rows = [i for i in range(len(t.index)) if t.transition_reason[i]]
    for row in handoff_rows:
        if 0 < row < len(t.index):
            assert not np.any(exceeded[row - 1]), (
                "rate continuity broken at handoff", row,
                delta[row - 1][exceeded[row - 1]])


def test_nc10_bilateral_symmetry_of_applied_action(episode):
    t = episode.telemetry
    for i, j in MIRRORED_PAIRS:
        asym = np.abs(t.applied_nm[:, i] - t.applied_nm[:, j]).max()
        assert asym <= 1e-9, (CHANNELS[i], asym)
    report = BE.actuation_conformance_report(episode)
    assert report["max_bilateral_asymmetry_nm"] <= 1e-9


def test_nc11_zero_passive_cannot_be_compensated_unboundedly():
    zero_passive_episode = run_launch_episode(zero_passive=True)
    report = BE.zero_passive_sensitivity_report(zero_passive_episode, {"status": "PASS"})
    assert report["status"] == "PASS", report["checks"]
    checks = {c["check"]: c for c in report["checks"]}
    assert checks["zero_passive_active_work_within_budget"]["pass"]
    assert checks["zero_passive_unbounded_command_is_budget_gated"]["pass"]


# ---------------------------------------------------------------------------
# actuation authority unit behaviour
# ---------------------------------------------------------------------------
def test_authority_rate_limit_from_reset():
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    applied, record, _ = authority.apply(np.full(N_CHANNELS, 1e6), np.zeros(N_CHANNELS),
                                         phase="STAND", mtp_active_allowed=True,
                                         mtp_passive_moment_nm=(0.0, 0.0),
                                         mtp_ledger=ledger)
    assert np.allclose(applied, RATE_CEILING_NM_PER_S * M.NATIVE_DT_S)


def test_authority_moment_ceiling_binds_after_rate_saturation():
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    for _ in range(5000):
        applied, record, ledger = authority.apply(
            np.full(N_CHANNELS, 1e6), np.zeros(N_CHANNELS), phase="STAND",
            mtp_active_allowed=True, mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
    assert np.allclose(applied, MOMENT_CEILING_NM)
    assert np.all(np.abs(applied) <= MOMENT_CEILING_NM + 1e-12)


def test_authority_power_ceiling_binds_with_fast_joint():
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    qdot = np.zeros(N_CHANNELS)
    qdot[3] = 30.0
    for _ in range(200):
        applied, record, ledger = authority.apply(
            np.full(N_CHANNELS, 1000.0), qdot, phase="PROPULSION",
            mtp_active_allowed=True, mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
    assert abs(applied[3] * qdot[3]) <= POWER_CEILING_W[3] + 1e-9
    assert abs(applied[3]) <= POWER_CEILING_W[3] / 30.0 + 1e-9


def test_authority_rejects_malformed_commands():
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    with pytest.raises(V3ActuationError):
        authority.apply(np.zeros(N_CHANNELS - 1), np.zeros(N_CHANNELS), phase="STAND",
                        mtp_active_allowed=True, mtp_passive_moment_nm=(0.0, 0.0),
                        mtp_ledger=ledger)
    with pytest.raises(V3ActuationError):
        authority.apply(np.full(N_CHANNELS, np.nan), np.zeros(N_CHANNELS), phase="STAND",
                        mtp_active_allowed=True, mtp_passive_moment_nm=(0.0, 0.0),
                        mtp_ledger=ledger)
    with pytest.raises(V3ActuationError):
        V3ActuationAuthority(0.0)


def test_authority_mtp_phase_gate_zeroes_active_mtp():
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    applied, record, _ = authority.apply(
        np.full(N_CHANNELS, 45.0), np.zeros(N_CHANNELS), phase="FLIGHT",
        mtp_active_allowed=False, mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
    assert applied[7] == 0.0 and applied[8] == 0.0
    assert record.saturation_stage[7] == "mtp_phase_gate"


def test_authority_mtp_budget_is_hard():
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    qdot = np.zeros(N_CHANNELS)
    qdot[7] = qdot[8] = 10.0
    for _ in range(500):
        applied, record, ledger = authority.apply(
            np.full(N_CHANNELS, 45.0), qdot, phase="BRAKING", mtp_active_allowed=True,
            mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
    assert ledger[0].active_positive_work_j <= MTP_ACTIVE_POSITIVE_WORK_BUDGET_J + 1e-9
    assert ledger[1].active_positive_work_j <= MTP_ACTIVE_POSITIVE_WORK_BUDGET_J + 1e-9
    assert ledger[0].active_gated and ledger[1].active_gated
    assert applied[7] == 0.0 and applied[8] == 0.0


def test_authority_symmetry_projection_records_asymmetry():
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    command = np.zeros(N_CHANNELS)
    command[1] = 40.0   # left_hip
    command[2] = -40.0  # right_hip (mirrored pair)
    applied, record, _ = authority.apply(command, np.zeros(N_CHANNELS), phase="STAND",
                                         mtp_active_allowed=True,
                                         mtp_passive_moment_nm=(0.0, 0.0),
                                         mtp_ledger=ledger)
    assert record.symmetry_asymmetry_nm == 80.0
    assert applied[1] == applied[2]


# ---------------------------------------------------------------------------
# evidence artifacts
# ---------------------------------------------------------------------------
def test_evidence_artifacts_present_and_pass():
    for name in ("V3_LAUNCH_CONTROLLER_SPEC.json", "TELEMETRY_MANIFEST.json",
                 "TELEMETRY_ARRAYS.bin", "LAUNCH_EPISODE_REPORT.json",
                 "OCCURRENCE_IDENTITY_AUDIT.json", "PROPULSION_DEFICIT_REPORT.json",
                 "PROPULSION_SEARCH.json", "MTP_NONCOMPENSATION_MATRIX.json",
                 "JOINT_ROM_SOFT_LIMIT_PROBE.json",
                 "SAFETY_OVERRIDE_AUDIT.json", "PRE_CORRECTION_EPISODE_CLASSIFICATION.json",
                 "MTP_ENERGY_REPORT.json", "ACTUATION_CONFORMANCE_REPORT.json",
                 "NEGATIVE_CONTROLS_REPORT.json", "ZERO_PASSIVE_SENSITIVITY_REPORT.json",
                 "DETERMINISM_REPORT.json", "HASH_MANIFEST.json", "RES85_RECEIPT.md",
                 "RES85C_CORRECTION_RECEIPT.md"):
        assert (EVIDENCE_DIR / name).is_file(), name
    negative = json.loads((EVIDENCE_DIR / "NEGATIVE_CONTROLS_REPORT.json").read_text())
    assert negative["status"] == "PASS", [c for c in negative["checks"] if not c["pass"]]
    conformance = json.loads((EVIDENCE_DIR / "ACTUATION_CONFORMANCE_REPORT.json").read_text())
    assert conformance["status"] == "PASS"
    determinism = json.loads((EVIDENCE_DIR / "DETERMINISM_REPORT.json").read_text())
    assert determinism["status"] == "PASS"
    mtp = json.loads((EVIDENCE_DIR / "MTP_ENERGY_REPORT.json").read_text())
    assert mtp["status"] == "PASS"
    identity = json.loads((EVIDENCE_DIR / "OCCURRENCE_IDENTITY_AUDIT.json").read_text())
    assert identity["status"] == "PASS"
    overrides = json.loads((EVIDENCE_DIR / "SAFETY_OVERRIDE_AUDIT.json").read_text())
    assert overrides["status"] == "PASS"


def test_receipt_states_claim_ceiling():
    text = (EVIDENCE_DIR / "RES85_RECEIPT.md").read_text()
    assert "ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED" in text
    assert "ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED" in text
    assert "No landing" in text and "RES-86" in text
    assert "H_ANTI_TRIVIALITY_FLOOR" in text


def test_historical_failures_are_classified_not_hidden():
    classification = json.loads(
        (EVIDENCE_DIR / "FULL_SUITE_CLASSIFICATION.json").read_text())
    assert classification["full_suite"]["failed"] == 60
    assert classification["full_suite"]["res85c_owned_failures"] == 0
    assert classification["full_suite"]["res85_owned_failures"] == 0
    assert classification["full_suite"]["res83_res84_failures"] == 0
    assert len(classification["failures"]) == 60
    assert len(classification["collection_errors"]) == 2
    reproduction = classification["entry_head_reproduction"]
    assert reproduction["worktree_head"] == "e487369f6861d9c9bc27f9f3d92b981fb3684293"
    assert reproduction["recorded_pre_res85_reproduction"][
        "same_failures_reproduced"] == 24
    assert reproduction["res85c_reproduction"][
        "failures_reproduced_node_for_node"] == 60
    assert "PRE-EXIST" in classification["verdict"]
    for failure in classification["failures"]:
        assert failure["category"].startswith("PRE_EXISTING_")
    assert classification["targeted_and_regression"][
        "tests/test_res85_v3_causal_launch.py"] == "PASS"
    assert classification["targeted_and_regression"][
        "tests/test_res85c_correction.py"] == "PASS"


def test_authority_amendments_are_recorded_and_do_not_relax_limits():
    amendments = json.loads(
        (EVIDENCE_DIR / "AUTHORITY_AMENDMENTS.json").read_text())
    ids = [a["id"] for a in amendments["amendments"]]
    assert ids == ["AMD-01", "AMD-02"]
    for amendment in amendments["amendments"]:
        assert amendment["scientific_effect"].startswith("none")
        assert amendment["evidence"]
    assert "NO CEILING" in amendments["verdict"]
    # the frozen numeric ceilings remain exactly the Achievement-A values
    authority = json.loads((EVIDENCE_DIR / "ACTUATION_AUTHORITY.json").read_text())
    ceilings = [c["moment_ceiling_nm"] for c in authority["channels"]]
    assert ceilings == [220.0, 330.0, 330.0, 380.0, 380.0, 260.0, 260.0, 45.0, 45.0]
