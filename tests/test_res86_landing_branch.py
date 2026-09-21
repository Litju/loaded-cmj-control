"""RES-86A landing branch authority, negative control and regression tests.

MISSION: `RES86A_LANDING_AUTHORITY_BRANCH_AND_RECORDER_FOUNDATION_001`
LINEAR ISSUE: RES-86

The session fixtures re-execute the deterministic computation archived in the
external evidence bundle: the instrumented canonical capture (Run A), the
unmodified canonical runtime (Run B), and the LANDING_PREP continuation
negative control.  The tracked summary
(`audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/BRANCH_CAPTURE_SUMMARY.json`)
binds the exact state hashes and baseline facts.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
for _path in (str(TASK_ROOT), str(TASK_ROOT / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import subprocess  # noqa: E402

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.controller import PHASE_ORDER, V3_CONTROLLER_AUTHORITY_ID  # noqa: E402
from loaded_cmj.v3.plant import V3Plant  # noqa: E402
from tools.res86.capture_landing_branch import (  # noqa: E402
    E8_SAMPLE,
    MISSION_AUTHORED_PATHS,
    PRE_TOUCHDOWN_SAMPLE,
    TOTAL_NATIVE_SAMPLES,
    baseline_negative_control,
    compare_run_a_to_run_b,
    compare_run_b_to_sealed,
    milestone_report,
    run_canonical_reference,
    run_instrumented_capture,
    source_worktree_classification,
)

SUMMARY_PATH = (TASK_ROOT / "audit" / "EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001" /
                "BRANCH_CAPTURE_SUMMARY.json")
SUMMARY = json.loads(SUMMARY_PATH.read_text())


@pytest.fixture(scope="session")
def capture():
    return run_instrumented_capture()


@pytest.fixture(scope="session")
def canonical():
    return run_canonical_reference()


@pytest.fixture(scope="session")
def baseline(capture):
    return baseline_negative_control(capture)


# ---------------------------------------------------------------------------
# 1. exact pre-touchdown / E8 state identities
# ---------------------------------------------------------------------------
def test_pre_touchdown_and_e8_state_identities(capture):
    assert SUMMARY["pre_touchdown"]["sample"] == PRE_TOUCHDOWN_SAMPLE == 790
    assert SUMMARY["first_contact"]["sample"] == E8_SAMPLE == 791
    tables = capture["tables"]
    pre = capture["certificates"][PRE_TOUCHDOWN_SAMPLE]
    e8 = capture["certificates"][E8_SAMPLE]
    assert pre["vector"].shape[0] == SUMMARY["pre_touchdown"]["state_size"] == 142
    assert e8["vector"].shape[0] == SUMMARY["first_contact"]["state_size"] == 142
    assert hashlib.sha256(pre["vector"].tobytes()).hexdigest() == \
        SUMMARY["pre_touchdown"]["state_sha256"]
    assert hashlib.sha256(e8["vector"].tobytes()).hexdigest() == \
        SUMMARY["first_contact"]["state_sha256"]
    assert tables.contact_count(PRE_TOUCHDOWN_SAMPLE) == 0
    assert tables.efc_count(PRE_TOUCHDOWN_SAMPLE) == 0
    assert int(capture["telemetry"]["legal_plantar_active"][PRE_TOUCHDOWN_SAMPLE]) == 0
    assert int(capture["telemetry"]["prohibited_detected"][PRE_TOUCHDOWN_SAMPLE]) == 0
    assert float(capture["telemetry"]["com_velocity_world_m_s"][PRE_TOUCHDOWN_SAMPLE, 2]) < 0.0
    assert SUMMARY["first_contact"]["state_sha256"] == \
        SUMMARY["first_contact"]["state_sha256"]


# ---------------------------------------------------------------------------
# 2. toe-first E8 is legal
# ---------------------------------------------------------------------------
def test_toe_first_first_contact_is_legal(capture):
    records = capture["first_contact_records"]
    assert len(records) > 0
    assert all(r.contact_class == M.V3ContactClass.LEGAL_PLANTAR_FLOOR for r in records)
    assert all(r.active_legal_plantar for r in records)
    assert all(not r.prohibited for r in records)
    assert {r.side for r in records} == {"left", "right"}
    assert {r.region for r in records} == {"toe"}
    assert float(capture["telemetry"]["com_velocity_world_m_s"][E8_SAMPLE, 2]) < -0.10
    assert str(capture["telemetry"]["support_mode"][E8_SAMPLE]) == "BILATERAL"
    assert SUMMARY["first_contact"]["sides"] == ["left", "right"]
    assert SUMMARY["first_contact"]["regions"] == ["toe"]
    assert SUMMARY["first_contact"]["prohibited_detected"] == 0


# ---------------------------------------------------------------------------
# 3. baseline failure is reproduced
# ---------------------------------------------------------------------------
def test_structural_rom_report_inherits_res85d_tolerance(capture, baseline):
    from loaded_cmj.v3.landing_authority import V3_STRUCTURAL_ROM_TOLERANCE_RAD

    rom = baseline["structural_rom"]
    assert rom["tolerance_rad"] == V3_STRUCTURAL_ROM_TOLERANCE_RAD == 1e-9
    assert rom["tolerance_role"] == "FLOATING_POINT_EQUALITY_ONLY_NOT_ANATOMICAL_ROM"
    assert rom["first_violation_sample"] == 950
    assert rom["channels"]["trunk_pelvis"]["violation_count"] > 0


def test_baseline_failure_reproduced(capture, baseline):
    assert baseline["e8_first_contact_sample"] == 791
    assert baseline["first_prohibited_contact_sample"] == 928
    assert baseline["support_loss_sample"] == 905
    pre = baseline["pre_prohibited"]
    assert pre["max_penetration_m"] == SUMMARY["baseline"]["max_penetration_m"]
    assert abs(pre["max_penetration_m"] - 0.0480219) < 1.0e-6
    assert pre["max_abs_com_vx_m_s"] == SUMMARY["baseline"]["max_abs_com_vx_m_s"]
    assert abs(pre["max_abs_com_vx_m_s"] - 0.414985) < 1.0e-5
    assert pre["max_abs_hy_kg_m2_s"] == SUMMARY["baseline"]["max_abs_hy_kg_m2_s"]
    assert abs(pre["max_abs_hy_kg_m2_s"] - 16.8996) < 1.0e-3
    assert baseline["chatter_transitions"] > 8
    assert len(baseline["material_reflight_intervals"]) > 0
    assert baseline["structural_rom"]["first_violation_sample"] is not None
    trunk = baseline["structural_rom"]["channels"]["trunk_pelvis"]
    assert trunk["violation_count"] > 0
    assert trunk["first_violation_sample"] == SUMMARY["baseline"]["structural_rom_first_violation_sample"]
    for side in ("left", "right"):
        mtp = baseline["structural_rom"]["channels"][f"{side}_mtp"]
        assert mtp["violation_count"] > 0, side
    assert baseline["all_samples"]["peak_total_floor_fz_bw"] > 8.0
    outcomes = baseline["gate_outcomes_against_res86_authority"]["outcomes"]
    assert outcomes["L4-G2_no_prohibited_or_fall_contact"] is False
    assert outcomes["L4-G4_max_penetration_m_within_limit"] is False
    assert outcomes["L4-G5_chatter_transitions_within_limit"] is False
    assert outcomes["L4-G5_no_material_reflight"] is False
    assert outcomes["L4-G6_peak_total_floor_fz_within_8_bw"] is False
    assert outcomes["L4-T9_window_max_abs_com_vx_within_limit"] is False
    assert outcomes["L4-T10_window_max_abs_hy_within_limit"] is False
    assert baseline["gate_outcomes_against_res86_authority"]["L4-T8_status"] == \
        "NOT_EVALUABLE_BASELINE_NEVER_REACHES_E10"


# ---------------------------------------------------------------------------
# active-set signature binding and instrumented/canonical identity
# ---------------------------------------------------------------------------
def test_branch_signature_matches_tracked_summary(capture):
    signature = capture["tables"].branch_signature(
        PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1,
        branch_id=SUMMARY["active_set_signature"]["branch_id"],
        executed_interval_id=SUMMARY["active_set_signature"]["executed_interval_id"])
    assert signature == SUMMARY["active_set_signature"]["branch_signature"]


def test_exact_fingerprint_role_and_mode_signature_are_declared(capture):
    summary = SUMMARY["active_set_signature"]
    assert summary["signature_version"] == "RES86_ACTIVE_SET_SIGNATURE_V1"
    tables = capture["tables"]
    exact = tables.branch_signature(
        PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1,
        branch_id=summary["branch_id"], executed_interval_id=summary["executed_interval_id"])
    mode = tables.branch_mode_signature(PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1)
    baseline_mode = tables.branch_mode_signature(E8_SAMPLE, TOTAL_NATIVE_SAMPLES - 1)
    assert exact != mode
    assert mode != baseline_mode
    assert tables.sample_mode_signature(PRE_TOUCHDOWN_SAMPLE) != \
        tables.sample_mode_signature(E8_SAMPLE)


def test_instrumented_capture_equals_canonical_runtime(capture, canonical):
    identity = compare_run_a_to_run_b(capture, canonical)
    assert identity["status"] == "PASS", [c for c in identity["checks"] if not c["identical"]]


# ---------------------------------------------------------------------------
# 10. no RES-83/84/85 regression
# ---------------------------------------------------------------------------
def test_no_res83_res84_res85_regression(canonical):
    plant = V3Plant()
    assert int(plant.model.nbody) == 14
    assert int(plant.model.nq) == 12
    assert int(plant.model.nv) == 12
    assert int(plant.model.nu) == 9
    assert int(plant.model.ngeom) == 17
    assert abs(float(plant.model.body_mass.sum()) - C.V3_SYSTEM_MASS_KG) < 1e-9
    assert M.MUJOCO_CONTACT_SEMANTICS_VERSION == "3.8.0"
    assert M.NATIVE_DT_S == 0.002
    assert M.ACTIVE_FORCE_STRICT_GT_ZERO is True
    assert M.DWELL_TYPE_PHYSICAL_TIME == "PHYSICAL_TIME"
    assert V3_CONTROLLER_AUTHORITY_ID == "LCMJ_RES85_CAUSAL_LAUNCH_CONTROLLER_V1"
    assert [p.value for p in PHASE_ORDER][-1] == "LANDING_PREP"

    sealed = compare_run_b_to_sealed(canonical)
    # The RES-85 qualified launch is preserved bit-exactly in its physics: the
    # sealed state digests, the applied moment sequence and every derived
    # trajectory array match.  The only declared difference is the MTP
    # saturation-stage *label*: the shared authority now reports a transient
    # phase gate as ``mtp_phase_gate`` instead of latching it as ``mtp_budget``,
    # which is the RES-86 authority-bug correction.  No applied moment, state or
    # event changes.
    declared_label_arrays = {"saturation_stage", "saturation_stage_code"}
    mismatches = [c["array"] for c in sealed["checks"] if not c["match"]]
    assert set(mismatches) <= declared_label_arrays, mismatches
    assert sealed["array_count"] >= 60
    applied = [c for c in sealed["checks"] if c["array"] == "applied_nm"]
    assert applied and applied[0]["match"]
    assert sealed["array_count"] - len(mismatches) >= 60

    events = canonical["episode"].events
    assert events["takeoff_occurrence"]["native_index"] == 612
    assert events["takeoff_confirmation"]["confirmation_sample"] == 637
    assert events["apex_h2"]["h2_support_m"] == 0.16021398534364972
    assert events["res85_claim_end"]["sample"] == 791

    qpos = np.asarray(canonical["episode"].telemetry.joint_q, dtype=np.float64)
    end = int(events["res85_claim_end"]["sample"]) + 1
    for name in ("trunk_pelvis", "left_hip", "right_hip", "left_knee",
                 "right_knee", "left_ankle", "right_ankle",
                 "left_mtp", "right_mtp"):
        rng = C.V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        column = qpos[:end, C.V3_JOINT_NAMES.index(name)]
        assert float(column.min()) >= float(rng[0]) - 1.0e-9, name
        assert float(column.max()) <= float(rng[1]) + 1.0e-9, name


# ---------------------------------------------------------------------------
# A1 provenance erratum: mission-authored files are never owner files
# ---------------------------------------------------------------------------
def test_provenance_classification_keeps_mission_files_disjoint():
    classification = source_worktree_classification()
    mission = set(classification["mission_authored_paths_at_entry"])
    owner = set(classification["untracked_owner_files_at_entry"])
    assert mission.isdisjoint(owner)
    assert set(classification["committed_by_this_mission"]) == set(MISSION_AUTHORED_PATHS)
    for path in MISSION_AUTHORED_PATHS:
        assert path not in owner, path


def test_mission_authored_files_are_git_tracked_at_the_achievement_commit():
    """The RES-86A commit authored these files; they were absent at ENTRY_HEAD."""
    entry_head = "314b96494305ff2a8f82c1506a0e96f5f308ae4e"
    achievement = "3f379a3290ec75b76b66a05080eb66707e33a396"
    for path in MISSION_AUTHORED_PATHS:
        before = subprocess.run(
            ["git", "-C", str(TASK_ROOT), "rev-parse", f"{entry_head}:{path}"],
            capture_output=True)
        after = subprocess.run(
            ["git", "-C", str(TASK_ROOT), "rev-parse", f"{achievement}:{path}"],
            capture_output=True)
        assert before.returncode != 0, path
        assert after.returncode == 0, path


def test_milestones_match_sealed_res85_binding(capture, canonical):
    milestones = milestone_report(capture, canonical)
    assert milestones["takeoff_occurrence"]["native_index"] == 612
    assert milestones["takeoff_confirmation"]["confirmation_sample"] == 637
    assert milestones["apex_h2_support_m"] == 0.16021398534364972
    assert milestones["claim_end"]["sample"] == 791
    assert milestones["claim_end"]["time_s"] == pytest.approx(1.582, abs=1e-12)
    assert SUMMARY["milestones"]["claim_end"]["sample"] == 791
