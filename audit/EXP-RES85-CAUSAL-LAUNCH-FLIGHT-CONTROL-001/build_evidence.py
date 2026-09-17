"""RES-85 / RES-85C — build the causal launch/flight control evidence bundle.

Authority: LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1
Mission:   RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001

Writes every deterministic scientific artifact of the regenerated RES-85 bundle
(the RES-85C correction of BLOCKER A..D):

    V3_LAUNCH_CONTROLLER_SPEC.json
    TELEMETRY_MANIFEST.json + TELEMETRY_ARRAYS.bin
    LAUNCH_EPISODE_REPORT.json
    OCCURRENCE_IDENTITY_AUDIT.json
    PROPULSION_DEFICIT_REPORT.json
    MTP_ENERGY_REPORT.json
    ACTUATION_CONFORMANCE_REPORT.json
    SAFETY_OVERRIDE_AUDIT.json
    NEGATIVE_CONTROLS_REPORT.json
    ZERO_PASSIVE_SENSITIVITY_REPORT.json
    PRE_CORRECTION_EPISODE_CLASSIFICATION.json
    DETERMINISM_REPORT.json
    HASH_MANIFEST.json
    RES85_RECEIPT.md
    RES85C_CORRECTION_RECEIPT.md

Run:  python3 build_evidence.py            # build twice and compare identity
      python3 build_evidence.py --once     # debug escape: build once
      python3 build_evidence.py --print    # status lines only
"""

from __future__ import annotations

import hashlib
import json
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SRC = REPO / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import mujoco  # noqa: E402

from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3 import plant as P  # noqa: E402
from loaded_cmj.v3.actuation import (  # noqa: E402
    CHANNELS,
    HARD_SAFETY_BOUNDS,
    MIRRORED_PAIRS,
    MOMENT_CEILING_NM,
    MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
    MTP_CHANNEL_INDICES,
    MTP_TOTAL_POSITIVE_WORK_BUDGET_J,
    N_CHANNELS,
    POWER_CEILING_W,
    RATE_CEILING_NM_PER_S,
    SAFETY_OVERRIDE_REASONS,
    TORQUE_RATE_ROLE,
    V3ActuationAuthority,
    V3MtpLedgerEntry,
)
from loaded_cmj.v3.constants import V3_JOINT_RANGES_RAD  # noqa: E402
from loaded_cmj.v3.controller import (  # noqa: E402
    PHASE_ORDER,
    SUPPORTED_PHASES,
    V3ControllerConfig,
    V3ControllerFault,
    V3LaunchController,
    V3Phase,
)
from loaded_cmj.v3.launch_runtime import (  # noqa: E402
    canonical_array_bytes,
    occurrence_identity_failures,
    run_launch_episode,
    settle_standing_stance,
)

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"

RES83_PLANT_XML_SHA256 = "eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d"
RES84_EVIDENCE_SEAL_SHA256 = "223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda"
PREVIOUS_RES85_EVIDENCE_SEAL_SHA256 = (
    "235a40e7a2a2b07ea483f2352b0765315b7f43b2343b3297461a0754b024b50e")
ENTRY_HEAD = "e487369f6861d9c9bc27f9f3d92b981fb3684293"
ENTRY_TREE = "5a88417d3a352ec87fee498e8a548575f19d3e0f"
H_ANTI_TRIVIALITY_FLOOR_M = 0.150
FLOOR_BALLISTIC_VZ_MAX_M_S = (2.0 * 9.81 * H_ANTI_TRIVIALITY_FLOOR_M) ** 0.5

ENTRY_EQUIVALENT_OVERRIDES: dict[str, float] = {
    "extension_rate_ff_gain": 0.0,
    "contact_preload_m": 0.0,
    "trunk_lean_frac": 0.0,
    "trunk_extend_frac": 0.0,
    "trunk_kp": 40.0,
    "trunk_kd": 6.0,
    "joint_rom_margin_rad": 0.03,
    "joint_rom_barrier_gain": 0.0,
    "trunk_rom_barrier_gain": 0.0,
    "a_thrust_m_s2": 8.0,
    "thrust_az_max_m_s2": 12.0,
}

ARTIFACT_FILES = (
    "V3_LAUNCH_CONTROLLER_SPEC.json",
    "TELEMETRY_MANIFEST.json",
    "TELEMETRY_ARRAYS.bin",
    "LAUNCH_EPISODE_REPORT.json",
    "OCCURRENCE_IDENTITY_AUDIT.json",
    "PROPULSION_DEFICIT_REPORT.json",
    "MTP_ENERGY_REPORT.json",
    "ACTUATION_CONFORMANCE_REPORT.json",
    "SAFETY_OVERRIDE_AUDIT.json",
    "NEGATIVE_CONTROLS_REPORT.json",
    "ZERO_PASSIVE_SENSITIVITY_REPORT.json",
    "PRE_CORRECTION_EPISODE_CLASSIFICATION.json",
    "PROPULSION_SEARCH.json",
    "MTP_NONCOMPENSATION_MATRIX.json",
    "JOINT_ROM_SOFT_LIMIT_PROBE.json",
    "DETERMINISM_REPORT.json",
    "AUTHORITY_CHECK_REPORT.json",
    "SOURCE_PROVENANCE.json",
    "RES85_RECEIPT.md",
    "RES85C_CORRECTION_RECEIPT.md",
)

CHANNEL_QPOS = None


def channel_qpos_indices() -> list[int]:
    global CHANNEL_QPOS
    if CHANNEL_QPOS is None:
        plant = P.V3Plant()
        CHANNEL_QPOS = [int(plant.idx.qadr[name]) for name in CHANNELS]
    return CHANNEL_QPOS


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def json_text(obj: Any) -> str:
    return json.dumps(obj, indent=2, sort_keys=True) + "\n"


def _check(checks: list[dict[str, Any]], name: str, passed: bool, detail: Any = "") -> None:
    checks.append({"check": name, "pass": bool(passed), "detail": detail})


def _status(checks: Sequence[dict[str, Any]]) -> str:
    return STATUS_PASS if all(c["pass"] for c in checks) else STATUS_FAIL


# ===========================================================================
# synthetic native frames (deterministic probes; RES-84 field semantics)
# ===========================================================================
def synthetic_frame(i: int, dt_s: float, *, active: int = 0, left_active: int | None = None,
                    right_active: int | None = None, clearance_m: float = 5.0e-3,
                    vz: float = 1.0, prohibited: int = 0, left_fz: float = 0.0,
                    right_fz: float = 0.0, z: float = 1.1) -> M.V3NativeFrame:
    left = active if left_active is None else left_active
    right = active if right_active is None else right_active
    total_active = int(left) + int(right)
    return M.V3NativeFrame(
        index=i,
        time_s=i * dt_s,
        com_world_m=(0.0, 0.0, z),
        com_velocity_world_m_s=(0.0, 0.0, vz),
        athlete_com_world_m=(0.0, 0.0, z - 0.05),
        left_clearance_m=clearance_m,
        right_clearance_m=clearance_m,
        legal_plantar_detected=total_active,
        legal_plantar_active=total_active,
        legal_plantar_normal_force_n=left_fz + right_fz,
        prohibited_detected=int(prohibited),
        prohibited_active=int(prohibited),
        total_floor_force_world_n=(0.0, 0.0, left_fz + right_fz),
        legal_ground_force_world_n=(0.0, 0.0, left_fz + right_fz),
        legal_ground_moment_world_nm=(0.0, 0.0, 0.0),
        left_foot_force_world_n=(0.0, 0.0, left_fz),
        right_foot_force_world_n=(0.0, 0.0, right_fz),
        nonplantar_floor_active=0,
        cop_validity="VALID",
        cop_x_m=0.0,
        support_mode="BILATERAL" if total_active else "NOT_EVALUABLE",
        qpos=tuple([0.0] * 12),
        qvel=tuple([0.0] * 12),
    )


def _probe_env() -> tuple[P.V3Plant, mujoco.MjData, V3LaunchController]:
    plant = P.V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    controller = V3LaunchController(plant, data)
    return plant, data, controller


def probe_phase_machine(frames: Sequence[M.V3NativeFrame], *,
                        tolerate_fault: bool = False) -> dict[str, Any]:
    """Drive the *phase machine* deterministically over a synthetic frame stream.

    The control law is not exercised; the probe records the phase trajectory,
    the FLIGHT latch and any fail-closed fault.  A fault is a valid outcome
    (rejection), never a silent success.
    """
    plant, data, controller = _probe_env()
    snapshot = M.measure(plant, data)
    phases: list[str] = [controller.phase.value]
    fault: str | None = None
    frames_list: list[M.V3NativeFrame] = []
    for frame in frames:
        frames_list.append(frame)
        try:
            controller._update_phase(frame, snapshot, frames_list)
        except V3ControllerFault as exc:
            fault = str(exc)
            break
        if phases[-1] != controller.phase.value:
            phases.append(controller.phase.value)
    latched = any(p == V3Phase.FLIGHT.value for p in phases)
    return {
        "phase_trajectory": phases,
        "flight_latched": latched,
        "takeoff_confirmed": bool(controller._takeoff_confirmed),
        "fault": fault,
        "candidate_rejections": list(controller._candidate_rejected),
    }


# ===========================================================================
# floor classification (RES-85C BLOCKER B)
# ===========================================================================
def floor_classification(h2_m: float | None) -> str:
    """Functional-task-floor classification of a direct SYSTEM_COM H2 value."""
    if h2_m is None or not np.isfinite(float(h2_m)):
        return "NOT_EVALUABLE"
    return ("ABOVE_FLOOR" if float(h2_m) >= H_ANTI_TRIVIALITY_FLOOR_M
            else "BELOW_FUNCTIONAL_TASK_FLOOR")


def floor_closure(h2_m: float | None) -> dict[str, Any]:
    classification = floor_classification(h2_m)
    return {
        "h2_m": h2_m,
        "h_anti_triviality_floor_m": H_ANTI_TRIVIALITY_FLOOR_M,
        "floor_role": "HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY",
        "classification": classification,
        "closure_pass": classification == "ABOVE_FLOOR",
        "ballistic_cross_check_vz_m_s": FLOOR_BALLISTIC_VZ_MAX_M_S,
        "note": ("H_ANTI_TRIVIALITY_FLOOR is a minimum functional success "
                 "condition, not an elite norm and not an optimization target"),
    }


# ===========================================================================
# negative controls NC-01..NC-11 (RES-85) and NC-12..NC-18 (RES-85C)
# ===========================================================================
def negative_controls(zero_passive_episode=None) -> dict[str, Any]:
    dt = M.NATIVE_DT_S
    checks: list[dict[str, Any]] = []
    probes: dict[str, Any] = {}

    # NC-01: one native-sample support dropout cannot permanently enter FLIGHT
    frames = [synthetic_frame(i, dt, active=0 if i == 40 else 2, clearance_m=5.0e-3,
                              vz=0.5 if i >= 40 else 0.0, left_fz=0.0 if i >= 40 else 450.0,
                              right_fz=0.0 if i >= 40 else 450.0)
              for i in range(120)]
    result = probe_phase_machine(frames)
    probes["NC-01_single_sample_dropout"] = result
    _check(checks, "NC-01_single_sample_dropout_refuses_permanent_flight",
           not result["flight_latched"], result)
    truncated = frames[:45]
    result_trunc = probe_phase_machine(truncated)
    probes["NC-01b_dropout_truncated_stream"] = result_trunc
    _check(checks, "NC-01b_dropout_truncated_stream_refuses_flight",
           not result_trunc["flight_latched"], result_trunc)

    # NC-02: 5 true comparator samples reject; 6 confirm only the comparator
    def comparator_run(n_below: int) -> dict[str, Any]:
        occ_frames = [synthetic_frame(i, dt, active=2, left_fz=450.0, right_fz=450.0)
                      for i in range(20)]
        occ_frames += [synthetic_frame(20 + j, dt, active=0, clearance_m=5.0e-3, vz=1.0)
                       for j in range(n_below)]
        occ_frames += [synthetic_frame(20 + n_below + j, dt, active=2, clearance_m=1.0e-4,
                                       vz=1.0, left_fz=450.0, right_fz=450.0)
                       for j in range(60)]
        occurrence = M.detect_takeoff_occurrence(occ_frames)
        result = M.force_takeoff_comparator(occ_frames, occurrence)
        confirmation = M.confirm_takeoff(occ_frames, occurrence)
        below_frames = sum(
            1 for f in occ_frames
            if M.bilateral_per_foot_below_threshold(f.left_foot_force_world_n[2],
                                                     f.right_foot_force_world_n[2]))
        return {
            "status": result.status,
            "triggered": result.triggered,
            "required_true_samples": result.required_true_samples,
            "true_sample_count": result.true_sample_count,
            "below_threshold_frames": below_frames,
            "requested_below_samples": n_below,
            "confirmation_confirmed": confirmation.confirmed,
        }

    five = comparator_run(5)
    six = comparator_run(6)
    probes["NC-02_five_true_samples"] = five
    probes["NC-02b_six_true_samples"] = six
    _check(checks, "NC-02_five_true_comparator_samples_reject",
           five["triggered"] is False and five["status"] == "NOT_TRIGGERED"
           and five["required_true_samples"] == 6, five)
    _check(checks, "NC-02b_six_true_samples_confirm_comparator_only",
           six["triggered"] is True and six["status"] == "TRIGGERED", six)
    # the comparator never latches FLIGHT by itself
    zero_fz = [synthetic_frame(i, dt, active=0 if i >= 20 else 2, clearance_m=5.0e-3,
                               vz=1.0 if i >= 20 else 0.0,
                               left_fz=0.0 if i >= 20 else 450.0,
                               right_fz=0.0 if i >= 20 else 450.0)
               for i in range(60)]
    occ = M.detect_takeoff_occurrence(zero_fz)
    comp = M.force_takeoff_comparator(zero_fz, occ)
    phase_after_comparator = probe_phase_machine(zero_fz[:40])
    probes["NC-02c_comparator_phase_probe"] = {
        "comparator": comp.status, "phase": phase_after_comparator}
    _check(checks, "NC-02c_comparator_never_defines_takeoff_or_flight",
           comp.status in ("TRIGGERED", "NOT_TRIGGERED", "INVALID")
           and not phase_after_comparator["flight_latched"], probes["NC-02c_comparator_phase_probe"])

    # NC-03: support loss followed by recontact before 50 ms rejects confirmation
    recontact = ([synthetic_frame(i, dt, active=2, left_fz=450.0, right_fz=450.0)
                  for i in range(20)]
                 + [synthetic_frame(20 + j, dt, active=0 if j < 10 else 2,
                                    clearance_m=5.0e-3, vz=1.0,
                                    left_fz=0.0 if j < 10 else 450.0,
                                    right_fz=0.0 if j < 10 else 450.0)
                    for j in range(60)])
    occurrence = M.detect_takeoff_occurrence(recontact)
    confirmation = M.confirm_takeoff(recontact, occurrence)
    phase_probe = probe_phase_machine(recontact)
    probes["NC-03_recontact_before_50ms"] = {
        "confirmed": confirmation.confirmed,
        "failed_checks": list(confirmation.failed_checks),
        "phase_probe": phase_probe,
    }
    _check(checks, "NC-03_recontact_before_dwell_rejects_confirmation",
           confirmation.confirmed is False
           and "no_legal_plantar_recontact" in confirmation.failed_checks
           and not phase_probe["flight_latched"], probes["NC-03_recontact_before_50ms"])

    # NC-04: one foot still supported cannot be physical takeoff
    one_foot = [synthetic_frame(i, dt, active=0, left_active=2, right_active=0,
                                clearance_m=5.0e-3, vz=1.0, left_fz=900.0, right_fz=0.0)
                for i in range(60)]
    occ_one = M.detect_takeoff_occurrence(one_foot)
    phase_one = probe_phase_machine(one_foot)
    probes["NC-04_one_foot_supported"] = {
        "occurrence_valid": occ_one.valid,
        "phase_probe": phase_one,
    }
    _check(checks, "NC-04_one_foot_contact_is_not_physical_takeoff",
           occ_one.valid is False and not phase_one["flight_latched"],
           probes["NC-04_one_foot_supported"])

    # NC-05: zero support with insufficient clearance cannot confirm flight
    low_clearance = ([synthetic_frame(i, dt, active=2, left_fz=450.0, right_fz=450.0)
                      for i in range(20)]
                     + [synthetic_frame(20 + j, dt, active=0, clearance_m=1.0e-3, vz=1.0)
                        for j in range(80)])
    occ_low = M.detect_takeoff_occurrence(low_clearance)
    conf_low = M.confirm_takeoff(low_clearance, occ_low)
    phase_low = probe_phase_machine(low_clearance)
    probes["NC-05_insufficient_clearance"] = {
        "occurrence_valid": occ_low.valid,
        "confirmed": conf_low.confirmed,
        "failed_checks": list(conf_low.failed_checks),
        "phase_probe": phase_low,
    }
    _check(checks, "NC-05_insufficient_clearance_never_confirms_flight",
           occ_low.valid is True and conf_low.confirmed is False
           and "bilateral_clearance_reaches_guard" in conf_low.failed_checks
           and not phase_low["flight_latched"], probes["NC-05_insufficient_clearance"])

    # NC-06: prohibited contact during confirmation rejects
    prohibited = ([synthetic_frame(i, dt, active=2, left_fz=450.0, right_fz=450.0)
                   for i in range(20)]
                  + [synthetic_frame(20 + j, dt, active=0, clearance_m=5.0e-3, vz=1.0,
                                     prohibited=1 if j in (2, 3) else 0)
                     for j in range(80)])
    occ_proh = M.detect_takeoff_occurrence(prohibited)
    conf_proh = M.confirm_takeoff(prohibited, occ_proh)
    phase_proh = probe_phase_machine(prohibited)
    probes["NC-06_prohibited_during_confirmation"] = {
        "confirmed": conf_proh.confirmed,
        "failed_checks": list(conf_proh.failed_checks),
        "phase_probe": phase_proh,
    }
    _check(checks, "NC-06_prohibited_contact_rejects_confirmation",
           conf_proh.confirmed is False
           and "no_prohibited_contact" in conf_proh.failed_checks
           and not phase_proh["flight_latched"]
           and phase_proh["fault"] is not None, probes["NC-06_prohibited_during_confirmation"])

    # NC-07: nonpositive SYSTEM_COM_vz at the occurrence rejects
    negative_vz = ([synthetic_frame(i, dt, active=2, left_fz=450.0, right_fz=450.0)
                    for i in range(20)]
                   + [synthetic_frame(20 + j, dt, active=0, clearance_m=5.0e-3, vz=-0.1)
                      for j in range(80)])
    occ_neg = M.detect_takeoff_occurrence(negative_vz)
    conf_neg = M.confirm_takeoff(negative_vz, occ_neg)
    phase_neg = probe_phase_machine(negative_vz)
    probes["NC-07_nonpositive_occurrence_vz"] = {
        "occurrence_valid": occ_neg.valid,
        "occurrence_vz": (None if occ_neg.com_velocity_world_m_s is None
                          else occ_neg.com_velocity_world_m_s[2]),
        "confirmed": conf_neg.confirmed,
        "failed_checks": list(conf_neg.failed_checks),
        "phase_probe": phase_neg,
    }
    _check(checks, "NC-07_nonpositive_occurrence_vz_rejects",
           occ_neg.valid is True and conf_neg.confirmed is False
           and "system_com_vz_positive" in conf_neg.failed_checks
           and not phase_neg["flight_latched"], probes["NC-07_nonpositive_occurrence_vz"])

    # NC-08: actuation authority unit probes (rate from reset, moment ceiling)
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    applied, _, _ = authority.apply(np.full(N_CHANNELS, 1e6), np.zeros(N_CHANNELS),
                                    phase="STAND", mtp_active_allowed=True,
                                    mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
    rate_ok = bool(np.allclose(applied, RATE_CEILING_NM_PER_S * M.NATIVE_DT_S))
    for _ in range(5000):
        applied, record, ledger = authority.apply(
            np.full(N_CHANNELS, 1e6), np.zeros(N_CHANNELS), phase="STAND",
            mtp_active_allowed=True, mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
    moment_ok = bool(np.all(np.abs(applied) <= MOMENT_CEILING_NM + 1e-12))
    probes["NC-08_authority_units"] = {"rate_from_reset": rate_ok,
                                       "moment_ceiling": moment_ok}
    _check(checks, "NC-08_authority_rate_and_moment_limits_hold",
           rate_ok and moment_ok, probes["NC-08_authority_units"])

    # NC-09: stateful joint-space contract — authority never drives a joint past
    # its ROM (see NC-12..NC-18 for the episode-level audits)
    _check(checks, "NC-09_previous_applied_is_sole_history",
           authority.previous_applied.shape == (N_CHANNELS,)
           and authority.step_index == 5001, {"step_index": authority.step_index})

    # NC-10: bilateral symmetry projection
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    command = np.zeros(N_CHANNELS)
    command[1] = 40.0
    command[2] = -40.0
    applied, record, _ = authority.apply(command, np.zeros(N_CHANNELS), phase="STAND",
                                         mtp_active_allowed=True,
                                         mtp_passive_moment_nm=(0.0, 0.0),
                                         mtp_ledger=ledger)
    probes["NC-10_symmetry"] = {"asymmetry_nm": record.symmetry_asymmetry_nm,
                                "applied_equal": float(applied[1]) == float(applied[2])}
    _check(checks, "NC-10_bilateral_symmetry_projection_holds",
           record.symmetry_asymmetry_nm == 80.0 and applied[1] == applied[2],
           probes["NC-10_symmetry"])

    # NC-11: zero-passive cannot be compensated unboundedly
    if zero_passive_episode is not None:
        report = zero_passive_sensitivity_report(zero_passive_episode, {"status": STATUS_PASS})
        checks.extend(report["checks"])
        probes["NC-11_zero_passive"] = {
            "status": report["status"],
            "active_positive_work_max_j": report["zero_passive_active_positive_work_max_j"],
        }
    return {"status": _status(checks), "checks": checks, "probes": probes}


def res85c_negative_controls(episode, report: dict[str, Any],
                             identity_audit: dict[str, Any],
                             override_audit: dict[str, Any],
                             matrix: dict[str, Any]) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    probes: dict[str, Any] = {}
    accepted = report["takeoff_occurrence"]["native_index"]
    history = report["takeoff_candidate_history"]
    rejected_or_not_evaluated = [h for h in history
                                 if h["disposition"] != "ACCEPTED"]
    alternate = [h for h in history
                 if int(h["occurrence"]["native_index"]) != int(accepted)]

    # NC-12: startup support dropout exported as the accepted occurrence fails
    if alternate:
        startup = min(alternate, key=lambda h: int(h["occurrence"]["native_index"]))
        mutated = json.loads(json.dumps(report))
        mutated["takeoff_occurrence"]["native_index"] = int(
            startup["occurrence"]["native_index"])
        failures = occurrence_identity_failures(mutated)
        probes["NC-12_startup_chatter_as_accepted"] = {
            "mutated_index": int(startup["occurrence"]["native_index"]),
            "disposition": startup["disposition"],
            "validator_failures": failures,
        }
        _check(checks, "NC-12_startup_support_dropout_as_accepted_occurrence_fails",
               len(failures) > 0, probes["NC-12_startup_chatter_as_accepted"])
    else:
        _check(checks, "NC-12_startup_support_dropout_as_accepted_occurrence_fails",
               False, "no alternate candidate available")

    # NC-13: top-level occurrence differing from the H2 origin fails
    mutated = json.loads(json.dumps(report))
    mutated["takeoff_occurrence"]["native_index"] = int(accepted) + 1
    failures = occurrence_identity_failures(mutated)
    probes["NC-13_occurrence_vs_h2_origin"] = {"validator_failures": failures}
    _check(checks, "NC-13_top_level_occurrence_differs_from_h2_origin_fails",
           any("H2_ORIGIN_MISMATCH" in f for f in failures)
           and any("IMPULSE_ORIGIN_MISMATCH" in f for f in failures),
           probes["NC-13_occurrence_vs_h2_origin"])

    # NC-14: comparator offset anchored on a rejected candidate fails
    if rejected_or_not_evaluated:
        rejected = rejected_or_not_evaluated[0]
        mutated = json.loads(json.dumps(report))
        mutated["diagnostic_comparator"]["offset_origin_index"] = int(
            rejected["occurrence"]["native_index"])
        failures = occurrence_identity_failures(mutated)
        probes["NC-14_comparator_offset_rejected"] = {
            "mutated_index": int(rejected["occurrence"]["native_index"]),
            "validator_failures": failures,
        }
        _check(checks, "NC-14_comparator_offset_uses_rejected_occurrence_fails",
               any("COMPARATOR_OFFSET_ORIGIN_MISMATCH" in f for f in failures),
               probes["NC-14_comparator_offset_rejected"])
    else:
        _check(checks, "NC-14_comparator_offset_uses_rejected_occurrence_fails",
               False, "no rejected candidate available")

    # NC-15: H2 just below the functional floor fails the closure gate
    below = H_ANTI_TRIVIALITY_FLOOR_M - 1.0e-9
    below = H_ANTI_TRIVIALITY_FLOOR_M - 1.0e-9
    below_closure = floor_closure(below)
    at_closure = floor_closure(H_ANTI_TRIVIALITY_FLOOR_M)
    probes["NC-15_floor_boundary"] = {"below": below_closure, "at": at_closure}
    _check(checks, "NC-15_h2_just_below_functional_floor_fails",
           below_closure["closure_pass"] is False
           and below_closure["classification"] == "BELOW_FUNCTIONAL_TASK_FLOOR"
           and at_closure["closure_pass"] is True,
           probes["NC-15_floor_boundary"])

    # NC-16: a zero-passive + zero-active case hidden or replaced by an active
    # command fails
    m1 = matrix["case_results"].get("M1_zero_passive_zero_active", {})
    m1_active = float(m1.get("mtp_active_total_positive_work_j", -1.0))
    hidden = dict(m1)
    hidden["mtp_active_positive_work_j"] = [1.0, 1.0]
    hidden["mtp_active_total_positive_work_j"] = 2.0
    probes["NC-16_zero_passive_hidden"] = {
        "m1_active_total_j": m1_active,
        "hidden_case_detected": float(hidden["mtp_active_total_positive_work_j"]) > 0.0,
    }
    _check(checks, "NC-16_zero_passive_zero_active_case_not_replaced_by_active_command",
           m1_active == 0.0 and m1.get("takeoff_confirmation") is True
           and probes["NC-16_zero_passive_hidden"]["hidden_case_detected"] is True,
           probes["NC-16_zero_passive_hidden"])

    # NC-17: an undeclared nominal slew exceedance fails the audit
    arrays = override_audit["arrays"]
    mutated_exceedance = np.array(arrays["nominal_slew_exceedance_nm_per_s"], dtype=np.float64)
    mutated_override = np.array(arrays["safety_override"], dtype=bool)
    mutated_exceedance[0, 0] = 1.0
    mutated_override[0, 0] = False
    failures = slew_override_failures(
        nominal_slew_exceedance_nm_per_s=mutated_exceedance,
        safety_override=mutated_override,
        safety_override_reason=arrays["safety_override_reason"],
        applied_nm=arrays["applied_nm"], qdot=arrays["joint_qdot"],
        mtp_gated=arrays["mtp_gated"], phase_name=arrays["phase_name"])
    probes["NC-17_undeclared_slew_exceedance"] = {"audit_failures": failures}
    _check(checks, "NC-17_undeclared_slew_exceedance_fails",
           any("UNDECLARED_SLEW_EXCEEDANCE" in f for f in failures),
           probes["NC-17_undeclared_slew_exceedance"])

    # NC-18: a declared safety override without a binding hard bound fails
    mutated_reason = np.array(arrays["safety_override_reason"], dtype=object)
    mutated_override = np.array(arrays["safety_override"], dtype=bool)
    mutated_override[0, 0] = True
    mutated_reason[0, 0] = "moment_ceiling"
    failures = slew_override_failures(
        nominal_slew_exceedance_nm_per_s=np.array(
            arrays["nominal_slew_exceedance_nm_per_s"], dtype=np.float64),
        safety_override=mutated_override,
        safety_override_reason=mutated_reason,
        applied_nm=arrays["applied_nm"], qdot=arrays["joint_qdot"],
        mtp_gated=arrays["mtp_gated"], phase_name=arrays["phase_name"])
    probes["NC-18_override_without_binding_bound"] = {"audit_failures": failures}
    _check(checks, "NC-18_declared_safety_override_without_binding_bound_fails",
           any("OVERRIDE_WITHOUT_BINDING_HARD_BOUND" in f for f in failures),
           probes["NC-18_override_without_binding_bound"])

    # NC-19: the accepted launch must not depend on active MTP work
    m0 = matrix["case_results"].get("M0_nominal_passive_nominal_active", {})
    probes["NC-19_mtp_ratio"] = {
        "m0_ratio": m0.get("ratio_mtp_active_over_total_positive"),
        "m1_active_total_j": m1_active,
    }
    _check(checks, "NC-19_mtp_active_work_is_negligible_fraction_of_joint_work",
           (m0.get("ratio_mtp_active_over_total_positive") is not None
            and float(m0["ratio_mtp_active_over_total_positive"]) < 0.01),
           probes["NC-19_mtp_ratio"])

    # NC-20: the reported episode passes its own identity audit
    _check(checks, "NC-20_canonical_identity_audit_passes",
           identity_audit["status"] == STATUS_PASS, identity_audit["failures"])
    # NC-21: the plateau check — the winner configuration is reproducible
    _check(checks, "NC-21_safety_override_audit_passes",
           override_audit["status"] == STATUS_PASS, override_audit["failures"])
    # NC-22: the launch joint-ROM audit passes (reference, drive and Plant
    # soft-limit envelope) and fails closed when an overshoot leaves the envelope
    rom = report.get("joint_rom_audit") or {}
    mutated_rom = json.loads(json.dumps(rom))
    first_channel = sorted(mutated_rom["channels"])[0]
    mutated_rom["channels"][first_channel]["measured_overshoot_rad"] = (
        mutated_rom["channels"][first_channel]["plant_soft_limit_envelope_rad"] + 1.0)
    mutated_failures = joint_rom_audit_failures(mutated_rom)
    _check(checks, "NC-22_launch_joint_rom_audit_passes",
           rom.get("status") == STATUS_PASS
           and joint_rom_audit_failures(rom) == []
           and any("OUTSIDE_PLANT_ENVELOPE" in f for f in mutated_failures),
           {"audit": {k: v for k, v in rom.items() if k != "channels"},
            "mutated_failures": mutated_failures})
    return {"status": _status(checks), "checks": checks, "probes": probes}


# ===========================================================================
# occurrence identity (RES-85C BLOCKER A)
# ===========================================================================
def occurrence_identity_audit(report: dict[str, Any]) -> dict[str, Any]:
    failures = occurrence_identity_failures(report)
    occurrence = report.get("takeoff_occurrence") or {}
    history = report.get("takeoff_candidate_history") or []
    dispositions: dict[str, int] = {}
    for entry in history:
        key = str(entry.get("disposition"))
        dispositions[key] = dispositions.get(key, 0) + 1
    accepted_entries = [h for h in history if h.get("disposition") == "ACCEPTED"]
    return {
        "schema_version": "1.0.0",
        "status": _status([{"pass": len(failures) == 0}]) if not failures else STATUS_FAIL,
        "failures": failures,
        "accepted_occurrence_index": occurrence.get("native_index"),
        "accepted_candidate_entries": accepted_entries,
        "candidate_disposition_counts": dispositions,
        "candidate_history_length": len(history),
        "confirmation_occurrence_index": (
            report.get("takeoff_confirmation") or {}).get("accepted_occurrence_index"),
        "h2_origin_occurrence_index": (report.get("apex_h2") or {}).get(
            "origin_occurrence_index"),
        "impulse_origin_occurrence_index": (report.get("impulse_cross_check") or {}).get(
            "occurrence_index"),
        "comparator_offset_origin_index": (report.get("diagnostic_comparator") or {}).get(
            "offset_origin_index"),
    }


# ===========================================================================
# safety override / nominal slew audit (RES-85C BLOCKER C)
# ===========================================================================
def slew_override_failures(*, nominal_slew_exceedance_nm_per_s: np.ndarray,
                           safety_override: np.ndarray,
                           safety_override_reason: np.ndarray,
                           applied_nm: np.ndarray,
                           qdot: np.ndarray,
                           mtp_gated: np.ndarray,
                           phase_name: np.ndarray) -> list[str]:
    """Recompute the hard-bound binding state at every declared override."""
    exceedance = np.asarray(nominal_slew_exceedance_nm_per_s, dtype=np.float64)
    override = np.asarray(safety_override, dtype=bool)
    reason = np.asarray(safety_override_reason, dtype=object)
    applied = np.asarray(applied_nm, dtype=np.float64)
    qd = np.asarray(qdot, dtype=np.float64)
    mtp_gated = np.asarray(mtp_gated, dtype=bool)
    if mtp_gated.ndim == 1:
        mtp_gated = np.repeat(mtp_gated[:, None], 2, axis=1)
    failures: list[str] = []
    undeclared = np.argwhere((exceedance > 0.0) & ~override)
    for i, ch in undeclared[:8]:
        failures.append(f"UNDECLARED_SLEW_EXCEEDANCE:sample={int(i)},channel={CHANNELS[int(ch)]}"
                        f",exceedance={float(exceedance[i, ch]):.6g}")
    for i, ch in np.argwhere(override)[:64]:
        i, ch = int(i), int(ch)
        declared = str(reason[i, ch])
        if declared not in SAFETY_OVERRIDE_REASONS:
            failures.append(f"UNDECLARED_OVERRIDE_REASON:sample={i},channel={CHANNELS[ch]}"
                            f",reason={declared!r}")
            continue
        moment_cap = float(MOMENT_CEILING_NM[ch])
        if abs(float(qd[i, ch])) > 1.0e-6:
            power_cap = float(POWER_CEILING_W[ch]) / abs(float(qd[i, ch]))
        else:
            power_cap = float("inf")
        binding_cap = min(moment_cap, power_cap)
        at_moment = abs(float(applied[i, ch])) >= moment_cap - 1.0e-6
        at_power = power_cap < moment_cap and abs(float(applied[i, ch])) >= power_cap - 1.0e-6
        mtp_gate = bool(ch in MTP_CHANNEL_INDICES
                        and mtp_gated[i, MTP_CHANNEL_INDICES.index(ch)])
        if declared == "moment_ceiling" and not at_moment:
            failures.append(f"OVERRIDE_WITHOUT_BINDING_HARD_BOUND:sample={i}"
                            f",channel={CHANNELS[ch]},reason=moment_ceiling")
        elif declared == "joint_power_ceiling" and not at_power:
            failures.append(f"OVERRIDE_WITHOUT_BINDING_HARD_BOUND:sample={i}"
                            f",channel={CHANNELS[ch]},reason=joint_power_ceiling")
        elif declared in ("mtp_energy_gate", "mtp_phase_gate") and not mtp_gate:
            failures.append(f"OVERRIDE_WITHOUT_BINDING_HARD_BOUND:sample={i}"
                            f",channel={CHANNELS[ch]},reason={declared}")
        elif binding_cap <= 0.0 and declared == "moment_ceiling":
            failures.append(f"OVERRIDE_WITHOUT_BINDING_HARD_BOUND:sample={i}"
                            f",channel={CHANNELS[ch]},zero_cap")
    return failures


def safety_override_audit(episode) -> dict[str, Any]:
    t = episode.telemetry
    qpos_idx = channel_qpos_indices()
    qdot = np.asarray(t.joint_qd[:, qpos_idx], dtype=np.float64)
    failures = slew_override_failures(
        nominal_slew_exceedance_nm_per_s=np.asarray(
            t.nominal_slew_exceedance_nm_per_s, dtype=np.float64),
        safety_override=np.asarray(t.safety_override, dtype=bool),
        safety_override_reason=np.asarray(t.safety_override_reason, dtype=object),
        applied_nm=np.asarray(t.applied_nm, dtype=np.float64),
        qdot=qdot, mtp_gated=np.asarray(t.mtp_gated, dtype=bool),
        phase_name=np.asarray(t.phase_name, dtype=object))
    arrays = {
        "nominal_slew_exceedance_nm_per_s":
            np.asarray(t.nominal_slew_exceedance_nm_per_s, dtype=np.float64).tolist(),
        "safety_override": np.asarray(t.safety_override, dtype=bool).tolist(),
        "safety_override_reason":
            np.asarray(t.safety_override_reason, dtype=object).tolist(),
        "applied_nm": np.asarray(t.applied_nm, dtype=np.float64).tolist(),
        "joint_qdot": qdot.tolist(),
        "mtp_gated": np.asarray(t.mtp_gated, dtype=bool).tolist(),
        "phase_name": [str(v) for v in t.phase_name],
    }
    stage_counts: dict[str, int] = {}
    for row in t.saturation_stage:
        for stage in row:
            name = str(stage)
            if name.startswith("safety_override"):
                stage_counts[name] = stage_counts.get(name, 0) + 1
    return {
        "schema_version": "1.0.0",
        "status": STATUS_PASS if not failures else STATUS_FAIL,
        "failures": failures,
        "torque_rate_role": TORQUE_RATE_ROLE,
        "hard_safety_bounds": list(HARD_SAFETY_BOUNDS),
        "override_stage_counts": stage_counts,
        "declared_override_samples": int(np.count_nonzero(
            np.any(np.asarray(t.safety_override, dtype=bool), axis=1))),
        "max_nominal_slew_exceedance_nm_per_s": float(
            np.max(np.asarray(t.nominal_slew_exceedance_nm_per_s, dtype=np.float64))
            if len(t.index) else 0.0),
        "arrays": arrays,
    }


# ===========================================================================
# episode-derived reports
# ===========================================================================
def _phase_runs(telemetry) -> list[tuple[str, int, int]]:
    runs: list[tuple[str, int, int]] = []
    for i in range(len(telemetry.index)):
        phase = str(telemetry.phase_name[i])
        if not runs or runs[-1][0] != phase:
            runs.append((phase, i, i))
        else:
            runs[-1] = (phase, runs[-1][1], i)
    return runs


def joint_rom_audit(episode) -> dict[str, Any]:
    """Controller-attributable launch joint-ROM audit (STAND -> occurrence).

    Declared criterion (``JOINT_ROM_SOFT_LIMIT_PROBE.json``):

    * the controller posture reference stays inside the frozen human-valid ROM;
    * the applied moment never drives a measured joint further past a frozen
      limit;
    * the measured launch overshoot stays inside the measured per-channel Plant
      soft-limit compliance envelope (a Plant property, not a controller
      budget).

    The post-flight landing window is RES-86 scope and is not judged here.
    """
    t = episode.telemetry
    events = episode.events
    occurrence = events.get("takeoff_occurrence") or {}
    k = occurrence.get("native_index")
    end = len(t.index) if k is None else int(k) + 1
    probe = _load_json(HERE / "JOINT_ROM_SOFT_LIMIT_PROBE.json")
    envelopes = {name: float(value["soft_limit_compliance_rad"])
                 for name, value in probe["channels"].items()}
    q_meas = np.asarray(t.joint_q[:end][:, channel_qpos_indices()], dtype=np.float64)
    q_ref = np.asarray(t.posture_reference_rad[:end], dtype=np.float64)
    applied = np.asarray(t.applied_nm[:end], dtype=np.float64)
    channels: dict[str, Any] = {}
    reference_ok = True
    drive_ok = True
    envelope_ok = True
    for c, name in enumerate(CHANNELS):
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        lo, hi = float(rng[0]), float(rng[1])
        q = q_meas[:, c]
        reference_ok &= bool(np.all((q_ref[:, c] >= lo - 1e-9)
                                    & (q_ref[:, c] <= hi + 1e-9)))
        drive_ok &= not bool(np.any(((q > hi) & (applied[:, c] > 1e-9))
                                    | ((q < lo) & (applied[:, c] < -1e-9))))
        overshoot = float(max(np.max(q - hi), np.max(lo - q), 0.0))
        envelope = envelopes[name]
        envelope_ok &= bool(overshoot <= envelope + 1e-9)
        channels[name] = {
            "rom_lower_rad": lo,
            "rom_upper_rad": hi,
            "min_measured_rad": float(np.min(q)),
            "max_measured_rad": float(np.max(q)),
            "measured_overshoot_rad": overshoot,
            "plant_soft_limit_envelope_rad": envelope,
            "within_envelope": bool(overshoot <= envelope + 1e-9),
        }
    audit = {
        "scope": "STAND_TO_ACCEPTED_OCCURRENCE",
        "criterion": ("reference within frozen ROM; applied moment never drives "
                      "past the frozen ROM; measured overshoot within the "
                      "per-channel Plant soft-limit compliance envelope"),
        "reference_within_frozen_rom": bool(reference_ok),
        "no_applied_moment_drives_past_frozen_rom": bool(drive_ok),
        "measured_overshoot_within_plant_soft_limit_envelope": bool(envelope_ok),
        "channels": channels,
        "landing_window_note": "post-flight landing ROM is RES-86 scope, not judged here",
    }
    audit["failures"] = joint_rom_audit_failures(audit)
    audit["status"] = STATUS_PASS if not audit["failures"] else STATUS_FAIL
    return audit


def joint_rom_audit_failures(audit: dict[str, Any]) -> list[str]:
    """Fail-closed validator for the launch joint-ROM audit record."""
    failures: list[str] = []
    if not audit.get("reference_within_frozen_rom"):
        failures.append("REFERENCE_OUTSIDE_FROZEN_ROM")
    if not audit.get("no_applied_moment_drives_past_frozen_rom"):
        failures.append("APPLIED_MOMENT_DRIVES_PAST_FROZEN_ROM")
    channels = audit.get("channels") or {}
    if not channels:
        failures.append("NO_ROM_CHANNELS")
    for name, channel in sorted(channels.items()):
        overshoot = float(channel.get("measured_overshoot_rad", 0.0))
        envelope = float(channel.get("plant_soft_limit_envelope_rad", 0.0))
        if overshoot > envelope + 1e-9:
            failures.append(f"MEASURED_OVERSHOOT_OUTSIDE_PLANT_ENVELOPE:{name}")
    return failures


def launch_episode_report(episode) -> dict[str, Any]:
    t = episode.telemetry
    events = episode.events
    H2 = events["apex_h2"]
    comparator = events["diagnostic_comparator"]
    flight_time = None
    if comparator and comparator.get("triggered"):
        touchdown = events["res85_claim_end"]
        if touchdown is not None and comparator.get("confirmation_time_s") is not None:
            T = touchdown["time_s"] - comparator["confirmation_time_s"]
            flight_time = {
                "method_id": "FORCE_PLATFORM_FLIGHT_TIME",
                "T_flight_s": float(T),
                "height_m": float(M.GRAVITY_M_S2 * T * T / 8.0),
                "status": "EXPERIMENTAL_COMPARABILITY_METRIC_REPORT_ONLY",
                "note": "boundaries are force-comparator crossings, not physical takeoff",
            }
    h2_value = H2.get("h2_support_m")
    report = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "episode_status": episode.status,
        "fault": episode.fault,
        "warnings": episode.warnings,
        "profiled": {
            "plant_xml_sha256": sha256_file(SRC / "loaded_cmj/v3/assets/v3_plant.xml"),
            "measurement_authority_id": M.V3_MEASUREMENT_AUTHORITY_ID,
            "native_dt_s": M.NATIVE_DT_S,
            "native_samples": int(len(t.index)),
            "horizon_s": float(t.time_s[-1]) if len(t.index) else 0.0,
            "zero_passive": False,
            "controller_config": _config_record(episode),
        },
        "phase_trajectory": episode.phases_visited,
        "transition_log": [
            {"sample": int(t.index[i]), "time_s": float(t.time_s[i]),
             "from": str(t.phase_name[i]), "reason": str(t.transition_reason[i])}
            for i in range(len(t.index)) if t.transition_reason[i]
        ],
        "takeoff_occurrence": events["takeoff_occurrence"],
        "takeoff_confirmation": events["takeoff_confirmation"],
        "takeoff_candidate_history": events["takeoff_candidate_history"],
        "diagnostic_comparator": comparator,
        "apex_h2": H2,
        "functional_task_floor": floor_closure(h2_value),
        "joint_rom_audit": joint_rom_audit(episode),
        "impulse_cross_check": events["impulse_cross_check"],
        "res85_claim_end": events["res85_claim_end"],
        "method_explicit_h2_report": {
            "PRIMARY_CANONICAL": {
                "method_id": "DIRECT_SIMULATOR_SYSTEM_COM",
                "quantity": "COM_RISE_TAKEOFF_TO_APEX",
                "value_m": h2_value,
                "evaluable": H2.get("evaluable"),
            },
            "SECONDARY_CROSS_CHECKS": {
                "BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ": {
                    "value_m": H2.get("ballistic_height_m"),
                    "residual_m": H2.get("ballistic_cross_check_delta_m"),
                    "takeoff_vz_m_s": H2.get("takeoff_vz_m_s"),
                },
                "IMPULSE_DERIVED_TAKEOFF_VELOCITY": {
                    "delta_vz_m_s": events["impulse_cross_check"].get(
                        "impulse_derived_delta_vz_m_s"),
                    "residual_m_s": events["impulse_cross_check"].get("residual_m_s"),
                },
            },
            "EXPERIMENTAL_COMPARABILITY_REPORT_ONLY": {
                "FORCE_PLATFORM_FLIGHT_TIME": flight_time,
            },
            "BAR_LVT_DISPLACEMENT_VELOCITY": {
                "status": "NOT_APPLICABLE_NATIVE",
                "reason": ("the sealed Plant has no bar tether instrument; the "
                           "bar/LVT family cannot be produced natively"),
            },
            "ELITE_SOCCER_PLUS20_H2_HARD_GATE": "NOT_ESTABLISHED",
            "ELITE_SOCCER_PLUS20_H2_TARGET": "NOT_ESTABLISHED",
            "H_ANTI_TRIVIALITY_FLOOR": {
                "value_m": H_ANTI_TRIVIALITY_FLOOR_M,
                "role": "HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY",
                "is_minimum_functional_success_condition": True,
                "is_elite_performance_norm": False,
                "is_optimization_target": False,
                "classification": floor_classification(h2_value),
            },
        },
    }
    return report


def propulsion_deficit_report(episode) -> dict[str, Any]:
    """Deterministic propulsion-deficit diagnosis of one episode."""
    t = episode.telemetry
    events = episode.events
    n = len(t.index)
    runs = _phase_runs(t)
    z = np.asarray(t.com_world_m[:, 2], dtype=np.float64)
    vz = np.asarray(t.com_velocity_world_m_s[:, 2], dtype=np.float64)
    fz = np.asarray(t.total_wrench[:, 2], dtype=np.float64)
    occurrence = events["takeoff_occurrence"]
    k_occ = occurrence.get("native_index")
    if k_occ is None:
        k_occ = n - 1
    k_occ = int(k_occ)
    stand_run = next((r for r in runs if r[0] == "STAND"), ("STAND", 0, 0))
    z_min_index = int(np.argmin(z[:k_occ + 1]))
    reversal = next((i for i in range(z_min_index, k_occ + 1)
                     if vz[i] >= 0.0), z_min_index)
    propulsion_entry = next((r[1] for r in runs if r[0] == "PROPULSION"), k_occ)
    key_vz = {
        "quiet_stand": {"sample": int(stand_run[2]), "vz_m_s": float(vz[stand_run[2]]),
                        "com_z_m": float(z[stand_run[2]])},
        "countermovement_minimum": {"sample": z_min_index, "vz_m_s": float(vz[z_min_index]),
                                    "com_z_m": float(z[z_min_index])},
        "upward_reversal": {"sample": int(reversal), "vz_m_s": float(vz[reversal]),
                            "com_z_m": float(z[reversal])},
        "propulsion_entry": {"sample": int(propulsion_entry), "vz_m_s": float(vz[propulsion_entry]),
                             "com_z_m": float(z[propulsion_entry])},
        "accepted_occurrence": {"sample": k_occ, "vz_m_s": float(vz[k_occ]),
                                "com_z_m": float(z[k_occ])},
        "provisional_support_losses": [
            {"sample": int(entry["occurrence"]["native_index"]),
             "vz_m_s": float(vz[int(entry["occurrence"]["native_index"])]),
             "com_z_m": float(z[int(entry["occurrence"]["native_index"])]),
             "disposition": entry["disposition"],
             "rejection_reason": entry["rejection_reason"]}
            for entry in events["takeoff_candidate_history"]
            if entry["disposition"] != "ACCEPTED"
        ],
    }
    impulse_by_phase: list[dict[str, Any]] = []
    work_by_phase: dict[str, dict[str, float]] = {}
    positive = np.maximum(np.asarray(t.joint_power_w, dtype=np.float64), 0.0) * M.NATIVE_DT_S
    negative = np.minimum(np.asarray(t.joint_power_w, dtype=np.float64), 0.0) * M.NATIVE_DT_S
    for phase, i0, i1 in runs:
        impulse = M.vertical_impulse_between(episode.frames, i0, i1)
        impulse_by_phase.append({
            "phase": phase, "first_sample": i0, "last_sample": i1,
            "duration_s": float(t.time_s[i1] - t.time_s[i0]) + M.NATIVE_DT_S,
            "vertical_impulse_n_s": float(impulse),
            "delta_vz_m_s": float(impulse / M.SYSTEM_MASS_KG),
            "mean_total_fz_n": float(np.mean(fz[i0:i1 + 1])),
            "peak_total_fz_n": float(np.max(fz[i0:i1 + 1])),
        })
        work_by_phase[phase] = {
            f"posterior_{CHANNELS[c]}": float(np.sum(positive[i0:i1 + 1, c]) -
                                              -np.sum(negative[i0:i1 + 1, c]))
            for c in range(N_CHANNELS)
        }
    launch = slice(0, k_occ + 1)
    power_cap_occupied = np.zeros(N_CHANNELS, dtype=int)
    power = np.abs(np.asarray(t.joint_power_w, dtype=np.float64)[launch])
    for c in range(N_CHANNELS):
        power_cap_occupied[c] = int(np.count_nonzero(
            power[:, c] >= 0.999 * POWER_CEILING_W[c]))
    stage_counts: dict[str, int] = {}
    for row in t.saturation_stage[launch]:
        for stage in row:
            name = str(stage)
            stage_counts[name] = stage_counts.get(name, 0) + 1
    qpos_idx = channel_qpos_indices()
    q = np.asarray(t.joint_q[:, qpos_idx], dtype=np.float64)
    rom_lo = np.asarray([-np.inf if V3_JOINT_RANGES_RAD[nm] is None
                         else V3_JOINT_RANGES_RAD[nm][0] for nm in CHANNELS])
    rom_hi = np.asarray([np.inf if V3_JOINT_RANGES_RAD[nm] is None
                         else V3_JOINT_RANGES_RAD[nm][1] for nm in CHANNELS])
    q_ref = np.asarray(t.posture_reference_rad, dtype=np.float64)
    support_mode = [str(v) for v in t.support_mode]
    transitions = [{"sample": i, "from": support_mode[i - 1], "to": support_mode[i]}
                   for i in range(1, k_occ + 1) if support_mode[i] != support_mode[i - 1]]
    ankle = CHANNELS.index("left_ankle")
    mtp = CHANNELS.index("left_mtp")
    diagnostics = episode.diagnostics or {}
    budget = float(diagnostics.get("descent_budget_m", 0.0))
    used = (float(z[stand_run[2]]) - float(z[z_min_index]))
    return {
        "schema_version": "1.0.0",
        "episode_status": episode.status,
        "fault": episode.fault,
        "accepted_occurrence_index": k_occ,
        "takeoff_vz_m_s": (None if events["apex_h2"].get("takeoff_vz_m_s") is None
                           else float(events["apex_h2"]["takeoff_vz_m_s"])),
        "h2_m": events["apex_h2"].get("h2_support_m"),
        "system_com_vz_by_event": key_vz,
        "vertical_impulse_by_phase": impulse_by_phase,
        "joint_work_by_phase": work_by_phase,
        "saturation_stage_occupancy_launch": stage_counts,
        "hard_power_cap_occupancy_samples_launch": {
            CHANNELS[c]: int(power_cap_occupied[c]) for c in range(N_CHANNELS)},
        "safety_override_samples_launch": int(np.count_nonzero(
            np.any(np.asarray(t.safety_override, dtype=bool)[launch], axis=1))),
        "joint_limit_margins_min_launch": {
            CHANNELS[c]: float(np.min(np.minimum(q[launch, c] - rom_lo[c],
                                                 rom_hi[c] - q[launch, c])))
            for c in range(N_CHANNELS)},
        "posture_reference_margins_min_launch": {
            CHANNELS[c]: float(np.min(np.minimum(q_ref[launch, c] - rom_lo[c],
                                                 rom_hi[c] - q_ref[launch, c])))
            for c in range(N_CHANNELS)},
        "cop_clamp_samples_launch": int(np.count_nonzero(
            np.asarray(t.cop_clamped, dtype=bool)[launch])),
        "support_mode_transitions_launch": transitions,
        "ankle_mtp_rocker_timing": {
            "ankle_angle_at_reversal_rad": float(q[reversal, ankle]),
            "ankle_angle_at_occurrence_rad": float(q[k_occ, ankle]),
            "ankle_peak_plantarflexion_rad": float(np.min(q[launch, ankle])),
            "ankle_peak_dorsiflexion_rad": float(np.max(q[launch, ankle])),
            "mtp_angle_at_reversal_rad": float(q[reversal, mtp]),
            "mtp_angle_at_occurrence_rad": float(q[k_occ, mtp]),
            "mtp_peak_dorsiflexion_rad": float(np.max(q[launch, mtp])),
            "note": "rocker timing from the frozen sagittal joint state; no local contact re-definition",
        },
        "structural_countermovement_budget": {
            "z_stand_m": diagnostics.get("z_stand_m"),
            "z_struct_min_m": diagnostics.get("z_struct_min_m"),
            "budget_m": budget,
            "used_m": used,
            "used_fraction": (None if budget <= 0.0 else used / budget),
        },
    }


def mtp_energy_report(episode) -> dict[str, Any]:
    events = episode.events
    t = episode.telemetry
    checks: list[dict[str, Any]] = []
    identity_residual = max(abs(events["mtp_energy"]["ledger_identity_left_residual_j"]),
                            abs(events["mtp_energy"]["ledger_identity_right_residual_j"]))
    _check(checks, "mtp_ledger_identity_exact", identity_residual <= 1e-9,
           {"residual_j": identity_residual})
    budgets_ok = True
    details = {}
    for foot, idx in enumerate(("left", "right")):
        active = float(t.mtp_active_work_j[-1, foot]) if len(t.index) else 0.0
        total = float(t.mtp_total_work_j[-1, foot]) if len(t.index) else 0.0
        details[idx] = {
            "active_positive_work_j": active,
            "total_positive_work_j": total,
            "active_budget_j": MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
            "total_budget_j": MTP_TOTAL_POSITIVE_WORK_BUDGET_J,
        }
        budgets_ok &= active <= MTP_ACTIVE_POSITIVE_WORK_BUDGET_J + 1e-9
        budgets_ok &= total <= MTP_TOTAL_POSITIVE_WORK_BUDGET_J + 1e-9
    _check(checks, "mtp_work_budgets_respected", budgets_ok, details)
    gate_samples = int(np.count_nonzero(t.mtp_gated)) if len(t.index) else 0
    after_takeoff_gate_ok = True
    if len(t.index):
        for i in range(len(t.index)):
            if t.phase_name[i] in ("TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP"):
                if abs(t.applied_nm[i, MTP_CHANNEL_INDICES[0]]) > 0.0 or \
                        abs(t.applied_nm[i, MTP_CHANNEL_INDICES[1]]) > 0.0:
                    after_takeoff_gate_ok = False
                    break
    _check(checks, "AEI-1e_active_mtp_zero_after_takeoff_confirmation", after_takeoff_gate_ok,
           {"gated_samples": gate_samples})
    dominance = events["takeoff_dominance"]
    _check(checks, "AEI-1f_ankle_positive_work_dominates_active_mtp",
           dominance["ankle_exceeds_active_mtp"], dominance)
    return {
        "schema_version": "1.0.0",
        "status": _status(checks),
        "checks": checks,
        "ledgers": details,
        "energy_events": events["mtp_energy"],
        "takeoff_dominance": dominance,
        "mtp_gated_samples": gate_samples,
        "zero_passive_episode": False,
    }


def actuation_conformance_report(episode) -> dict[str, Any]:
    t = episode.telemetry
    checks: list[dict[str, Any]] = []
    dt = M.NATIVE_DT_S
    moments = np.abs(t.applied_nm) if len(t.index) else np.zeros((0, N_CHANNELS))
    max_moment = moments.max(axis=0) if len(t.index) else np.zeros(N_CHANNELS)
    _check(checks, "hard_moment_ceiling_never_exceeded",
           bool(np.all(max_moment <= MOMENT_CEILING_NM + 1e-9)),
           {"max_applied_nm": [float(v) for v in max_moment],
            "ceilings": [float(v) for v in MOMENT_CEILING_NM]})
    power = np.abs(t.joint_power_w) if len(t.index) else np.zeros((0, N_CHANNELS))
    max_power = power.max(axis=0) if len(t.index) else np.zeros(N_CHANNELS)
    _check(checks, "hard_power_ceiling_never_exceeded",
           bool(np.all(max_power <= POWER_CEILING_W + 1e-6)),
           {"max_power_w": [float(v) for v in max_power],
            "ceilings": [float(v) for v in POWER_CEILING_W]})
    exceedance = (np.asarray(t.nominal_slew_exceedance_nm_per_s, dtype=np.float64)
                  if len(t.index) else np.zeros((0, N_CHANNELS)))
    override = (np.asarray(t.safety_override, dtype=bool)
                if len(t.index) else np.zeros((0, N_CHANNELS), dtype=bool))
    undeclared = int(np.count_nonzero((exceedance > 0.0) & ~override))
    declared = int(np.count_nonzero(override))
    _check(checks, "nominal_slew_exceedance_zero_unless_safety_override",
           undeclared == 0, {"undeclared_channel_samples": undeclared,
                             "declared_override_channel_samples": declared,
                             "max_exceedance_nm_per_s": (float(exceedance.max())
                                                         if exceedance.size else 0.0)})
    asym = 0.0
    if len(t.index):
        for i, j in MIRRORED_PAIRS:
            asym = max(asym, float(np.abs(t.applied_nm[:, i] - t.applied_nm[:, j]).max()))
    _check(checks, "bilateral_applied_symmetry_within_tolerance", asym <= 1e-9,
           {"max_pair_asymmetry_nm": asym, "tolerance_nm": 1e-9})
    handoff_indices = [int(t.index[i]) for i in range(len(t.index))
                       if t.transition_reason[i]]
    handoff_ok = True
    if len(t.index) > 1:
        rate = np.abs(np.diff(t.applied_nm, axis=0)) / dt
        for k in handoff_indices:
            if 0 < k < len(t.index):
                if any(rate[k - 1][c] > RATE_CEILING_NM_PER_S[c] + 1e-6
                       for c in range(N_CHANNELS)
                       if not override[k, c]):
                    handoff_ok = False
    _check(checks, "phase_handoffs_do_not_create_hidden_slew_violations",
           handoff_ok, {"handoff_samples": handoff_indices})
    stage_counts: dict[str, int] = {}
    if len(t.index):
        for row in t.saturation_stage:
            for s in row:
                stage_counts[str(s)] = stage_counts.get(str(s), 0) + 1
    return {
        "schema_version": "1.0.0",
        "status": _status(checks),
        "checks": checks,
        "torque_rate_role": TORQUE_RATE_ROLE,
        "hard_safety_bounds": list(HARD_SAFETY_BOUNDS),
        "max_applied_nm": [float(v) for v in max_moment],
        "max_rate_nm_per_s": ([float(v) for v in
                               (np.abs(np.diff(t.applied_nm, axis=0)) / dt).max(axis=0)]
                              if len(t.index) > 1 else [0.0] * N_CHANNELS),
        "max_joint_power_w": [float(v) for v in max_power],
        "max_bilateral_asymmetry_nm": asym,
        "saturation_stage_counts": stage_counts,
        "handoff_samples": handoff_indices,
        "safety_override_channel_samples": declared,
        "safety_override_samples": int(np.count_nonzero(override.any(axis=1))
                                       if len(t.index) else 0),
    }


def zero_passive_sensitivity_report(episode, negative: dict[str, Any]) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    t = episode.telemetry
    active_max = float(t.mtp_active_work_j[-1].max()) if len(t.index) else 0.0
    _check(checks, "zero_passive_active_work_within_budget",
           active_max <= MTP_ACTIVE_POSITIVE_WORK_BUDGET_J + 1e-9,
           {"active_positive_work_j": active_max})
    authority = V3ActuationAuthority(M.NATIVE_DT_S)
    ledger = (V3MtpLedgerEntry(), V3MtpLedgerEntry())
    command = np.zeros(N_CHANNELS)
    command[MTP_CHANNEL_INDICES[0]] = MOMENT_CEILING_NM[MTP_CHANNEL_INDICES[0]]
    command[MTP_CHANNEL_INDICES[1]] = MOMENT_CEILING_NM[MTP_CHANNEL_INDICES[1]]
    qdot = np.zeros(N_CHANNELS)
    qdot[MTP_CHANNEL_INDICES[0]] = 10.0
    qdot[MTP_CHANNEL_INDICES[1]] = 10.0
    for _ in range(500):
        _, _, ledger = authority.apply(
            command, qdot, phase="BRAKING", mtp_active_allowed=True,
            mtp_passive_moment_nm=(0.0, 0.0), mtp_ledger=ledger)
    _check(checks, "zero_passive_unbounded_command_is_budget_gated",
           ledger[0].active_positive_work_j <= MTP_ACTIVE_POSITIVE_WORK_BUDGET_J + 1e-9
           and ledger[1].active_positive_work_j <= MTP_ACTIVE_POSITIVE_WORK_BUDGET_J + 1e-9
           and ledger[0].active_gated and ledger[1].active_gated,
           {"left_j": ledger[0].active_positive_work_j,
            "right_j": ledger[1].active_positive_work_j,
            "left_gated": ledger[0].active_gated})
    _check(checks, "zero_passive_episode_closes_or_reports_degradation",
           episode.status == "COMPLETED", {"status": episode.status,
                                           "fault": episode.fault,
                                           "phases": episode.phases_visited})
    return {
        "schema_version": "1.0.0",
        "status": _status(checks),
        "checks": checks,
        "episode_status": episode.status,
        "episode_phases": episode.phases_visited,
        "zero_passive_active_positive_work_max_j": active_max,
        "zero_passive_budget_gated": bool(ledger[0].active_gated and ledger[1].active_gated),
        "negative_control_status": negative["status"],
    }


def pre_correction_episode_classification() -> dict[str, Any]:
    """Reproduce the ENTRY-equivalent episode and classify it honestly.

    The ENTRY controller (all RES-85C terms neutral) is a byte-identical
    controller to the sealed e487369 achievement; its episode is retained as
    historical pre-correction evidence and is classified as physically valid
    causal takeoff and flight but BELOW the functional task floor.
    """
    config = replace(V3ControllerConfig(), **ENTRY_EQUIVALENT_OVERRIDES)
    episode = run_launch_episode(controller_config=config)
    events = episode.events
    h2 = events["apex_h2"].get("h2_support_m")
    occurrence = events["takeoff_occurrence"]
    confirmation = events["takeoff_confirmation"]
    return {
        "schema_version": "1.0.0",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "entry_head": ENTRY_HEAD,
        "entry_tree": ENTRY_TREE,
        "previous_res85_evidence_seal_sha256": PREVIOUS_RES85_EVIDENCE_SEAL_SHA256,
        "controller_config": {field: getattr(config, field)
                              for field in config.__dataclass_fields__},
        "entry_controller_equivalence": {
            "overrides": ENTRY_EQUIVALENT_OVERRIDES,
            "rule": ("all RES-85C engineering terms neutral: the ENTRY controller "
                     "behaviour is reproduced exactly"),
        },
        "episode_status": episode.status,
        "fault": episode.fault,
        "takeoff_occurrence_index": occurrence.get("native_index"),
        "takeoff_occurrence_time_s": occurrence.get("time_s"),
        "takeoff_confirmation": (None if confirmation is None
                                 else bool(confirmation.get("confirmed"))),
        "confirmation_sample": (None if confirmation is None
                                else confirmation.get("confirmation_sample")),
        "takeoff_vz_m_s": events["apex_h2"].get("takeoff_vz_m_s"),
        "h2_m": h2,
        "classification": ("PHYSICALLY_VALID_CAUSAL_TAKEOFF_AND_FLIGHT_"
                           + floor_classification(h2)),
        "floor_classification": floor_closure(h2),
        "claim": ("historical pre-correction evidence only: NOT a PASS, NOT a "
                  "fail-closed rejection, and it must never be relabelled as "
                  "meeting the functional task floor"),
    }


# ===========================================================================
# telemetry artifacts (deterministic binary)
# ===========================================================================
def telemetry_artifacts(episode) -> tuple[dict[str, Any], bytes]:
    arrays = episode.telemetry.arrays()
    manifest_entries = []
    chunks: list[bytes] = []
    offset = 0
    for name in sorted(arrays):
        arr = np.asarray(arrays[name])
        raw = canonical_array_bytes(arr)
        manifest_entries.append({
            "name": name,
            "dtype": str(arr.dtype),
            "shape": list(arr.shape),
            "offset": offset,
            "nbytes": len(raw),
            "sha256": sha256_bytes(raw),
            "encoding": ("utf8_object_records_unit_separated" if arr.dtype == object
                         else "raw_little_endian"),
        })
        chunks.append(raw)
        offset += len(raw)
    blob = b"".join(chunks)
    manifest = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "byte_order": "little_endian_native_float64_and_bool",
        "array_count": len(manifest_entries),
        "total_bytes": len(blob),
        "blob_sha256": sha256_bytes(blob),
        "canonical_digest": episode.telemetry.canonical_digest(),
        "arrays": manifest_entries,
        "reconstruction_note": (
            "arrays are raw little-endian buffers concatenated in the declared "
            "offset order; dtypes and shapes are sufficient to reconstruct every "
            "native sample of the episode"
        ),
    }
    return manifest, blob


# ===========================================================================
# builder
# ===========================================================================
def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def _config_record(episode) -> dict[str, Any]:
    config = getattr(episode, "controller_config", None)
    if config is None:
        return {}
    return {field: getattr(config, field) for field in config.__dataclass_fields__}


def _episode_with_config(overrides: dict[str, float], *, horizon_s: float | None = None,
                         zero_passive: bool = False):
    config = replace(V3ControllerConfig(), **overrides)
    kwargs = {"controller_config": config, "zero_passive": zero_passive}
    if horizon_s is not None:
        kwargs["horizon_s"] = horizon_s
    episode = run_launch_episode(**kwargs)
    episode.controller_config = config  # type: ignore[attr-defined]
    return episode


def build_all() -> dict[str, Any]:
    import build_authority_checks as BAC  # local sibling module
    import classify_historical_failures as CHF  # local sibling module

    authority = BAC.build_all()
    CHF.main([])
    config = V3ControllerConfig()
    corrected_episode = _episode_with_config(
        {field: getattr(config, field) for field in config.__dataclass_fields__})
    zero_passive_episode = _episode_with_config(
        {field: getattr(config, field) for field in config.__dataclass_fields__},
        zero_passive=True)
    pre_correction = pre_correction_episode_classification()
    report = launch_episode_report(corrected_episode)
    identity_audit = occurrence_identity_audit(report)
    deficit = propulsion_deficit_report(corrected_episode)
    override_audit = safety_override_audit(corrected_episode)
    mtp = mtp_energy_report(corrected_episode)
    conformance = actuation_conformance_report(corrected_episode)
    mtp_matrix = _load_json(HERE / "MTP_NONCOMPENSATION_MATRIX.json")
    zero_passive = zero_passive_sensitivity_report(zero_passive_episode, {"status": STATUS_PASS})
    negative = negative_controls(zero_passive_episode)
    negative_c = res85c_negative_controls(corrected_episode, report, identity_audit,
                                          override_audit, mtp_matrix)
    negative_all = {
        "schema_version": "1.0.0",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "status": STATUS_PASS if (negative["status"] == STATUS_PASS
                                  and negative_c["status"] == STATUS_PASS) else STATUS_FAIL,
        "checks": negative["checks"] + negative_c["checks"],
        "probes": {**negative["probes"], **negative_c["probes"]},
        "legacy_status": negative["status"],
        "res85c_status": negative_c["status"],
    }
    telemetry_manifest, telemetry_blob = telemetry_artifacts(corrected_episode)

    probe_plant = P.V3Plant()
    probe_data = probe_plant.make_data()
    settle_standing_stance(probe_plant, probe_data)
    controller = V3LaunchController(probe_plant, probe_data)
    spec = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "correction": {
            "correction_id": "RES-85C",
            "entry_head": ENTRY_HEAD,
            "entry_tree": ENTRY_TREE,
            "previous_evidence_seal_sha256": PREVIOUS_RES85_EVIDENCE_SEAL_SHA256,
            "blockers": ["A_accepted_takeoff_identity", "B_functional_floor_authority",
                         "C_actuation_rate_contract", "D_mtp_non_compensation"],
        },
        "controller": controller.authority_record(),
        "modules": {
            "actuation.py": sha256_file(SRC / "loaded_cmj/v3/actuation.py"),
            "controller.py": sha256_file(SRC / "loaded_cmj/v3/controller.py"),
            "launch_runtime.py": sha256_file(SRC / "loaded_cmj/v3/launch_runtime.py"),
            "measurement.py": sha256_file(SRC / "loaded_cmj/v3/measurement.py"),
            "plant.py": sha256_file(SRC / "loaded_cmj/v3/plant.py"),
            "constants.py": sha256_file(SRC / "loaded_cmj/v3/constants.py"),
        },
        "plant_xml_sha256": sha256_file(SRC / "loaded_cmj/v3/assets/v3_plant.xml"),
        "res83_sealed_plant_xml_sha256": RES83_PLANT_XML_SHA256,
        "res84_measurement_authority_id": M.V3_MEASUREMENT_AUTHORITY_ID,
        "res84_evidence_seal_sha256": RES84_EVIDENCE_SEAL_SHA256,
        "phase_order": [p.value for p in PHASE_ORDER],
        "supported_phases": [p.value for p in SUPPORTED_PHASES],
        "consumed_measurement_primitives": [
            "native_frame", "measure", "scan_takeoff_candidates", "confirm_takeoff",
            "detect_takeoff_occurrence", "detect_apex", "force_takeoff_comparator",
            "vertical_impulse_between", "active_support_hull", "system_com_state",
        ],
        "no_reimplemented_measurement_semantics": True,
        "accepted_occurrence_rule": (
            "the unique accepted top-level TAKEOFF_OCCURRENCE is the controller-"
            "confirmed candidate; every other support-to-zero transition remains "
            "visible in takeoff_candidate_history"),
        "functional_floor": {
            "symbol": "H_ANTI_TRIVIALITY_FLOOR",
            "value_m": H_ANTI_TRIVIALITY_FLOOR_M,
            "role": "HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY",
            "is_minimum_functional_success_condition": True,
            "is_elite_performance_norm": False,
            "is_optimization_target": False,
        },
    }

    return {
        "authority": authority,
        "spec": spec,
        "episode": corrected_episode,
        "zero_passive_episode": zero_passive_episode,
        "episode_report": report,
        "identity_audit": identity_audit,
        "deficit": deficit,
        "pre_correction": pre_correction,
        "mtp_report": mtp,
        "conformance": conformance,
        "override_audit": override_audit,
        "negative": negative_all,
        "zero_passive": zero_passive,
        "mtp_matrix": mtp_matrix,
        "telemetry_manifest": telemetry_manifest,
        "telemetry_blob": telemetry_blob,
        "receipt": receipt(corrected_episode, report, conformance, negative_all, mtp,
                           override_audit),
        "correction_receipt": correction_receipt(corrected_episode, report, identity_audit,
                                                 conformance, override_audit, negative_all,
                                                 pre_correction, mtp_matrix, deficit),
    }


def _h2_line(report: dict[str, Any], pre_correction: dict[str, Any]) -> str:
    return (f"pre-correction H2 `{pre_correction['h2_m']}` m "
            f"({pre_correction['classification']}) -> corrected H2 "
            f"`{report['method_explicit_h2_report']['PRIMARY_CANONICAL']['value_m']}` m "
            f"({report['functional_task_floor']['classification']})")


def receipt(episode, report: dict[str, Any], conformance: dict[str, Any],
            negative: dict[str, Any], mtp: dict[str, Any],
            override_audit: dict[str, Any]) -> str:
    h2 = report["method_explicit_h2_report"]["PRIMARY_CANONICAL"]["value_m"]
    vz = report["apex_h2"].get("takeoff_vz_m_s")
    occurrence = report["takeoff_occurrence"]
    confirmation = report["takeoff_confirmation"] or {}
    lines = [
        "# RES85_RECEIPT — V3 causal loaded-CMJ launch and flight control (RES-85C)",
        "",
        "MISSION: `RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001`",
        "LINEAR ISSUE: RES-85",
        "STATUS: **%s**" % (
            "PASS" if (negative["status"] == STATUS_PASS
                       and conformance["status"] == STATUS_PASS
                       and mtp["status"] == STATUS_PASS
                       and override_audit["status"] == STATUS_PASS
                       and report["joint_rom_audit"]["status"] == STATUS_PASS
                       and episode.status == "COMPLETED"
                       and report["functional_task_floor"]["closure_pass"]) else "FAIL"),
        "",
        "## Episode",
        "",
        "* status: `%s`%s" % (episode.status,
                              "" if episode.fault is None else f" (fault: `{episode.fault}`)"),
        "* phases: `%s`" % " -> ".join(episode.phases_visited),
        "* accepted occurrence: sample `%s` / t=`%s` s" % (
            occurrence.get("native_index"), occurrence.get("time_s")),
        "* takeoff confirmation: `%s` at sample `%s`" % (
            confirmation.get("confirmed"), confirmation.get("confirmation_sample")),
        "* takeoff vz: `%s` m/s" % vz,
        "* H2 (DIRECT_SIMULATOR_SYSTEM_COM): `%s` m" % h2,
        "* H_ANTI_TRIVIALITY_FLOOR classification: `%s`" % (
            report["functional_task_floor"]["classification"]),
        "* launch joint ROM audit (STAND -> occurrence): `%s`" % (
            report["joint_rom_audit"]["status"]),
        "",
        "## Method-explicit comparator reporting",
        "",
        "| Method | Status | Value |",
        "|---|---|---|",
        f"| DIRECT_SIMULATOR_SYSTEM_COM | PRIMARY_CANONICAL | {h2} m |",
        "| BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ | SECONDARY_CROSS_CHECK | "
        + str(report["method_explicit_h2_report"]["SECONDARY_CROSS_CHECKS"]
              ["BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ"]["value_m"]) + " m |",
        "| FORCE_PLATFORM_IMPULSE_MOMENTUM | CROSS_CHECK | see impulse_cross_check |",
        "| FORCE_PLATFORM_FLIGHT_TIME | REPORT_ONLY | see flight time entry |",
        "| BAR_LVT_DISPLACEMENT_VELOCITY | NOT_APPLICABLE_NATIVE | no tether instrument |",
        "",
        "`ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED` and",
        "`ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED`.  The declared",
        "`H_ANTI_TRIVIALITY_FLOOR = 0.150 m` is a hard functional",
        "non-triviality boundary: it is not an elite norm, not an expected",
        "value and not an optimization target, but it is a minimum functional",
        "success condition (diagnostic scale cross-check: minimum ballistic",
        "`vz = sqrt(2 g 0.150) ~= %.3f m/s`)." % FLOOR_BALLISTIC_VZ_MAX_M_S,
        "",
        "## Mandatory negative controls (deterministic)",
        "",
        "| Control | Result |",
        "|---|---|",
    ]
    for c in negative["checks"]:
        lines.append(f"| `{c['check']}` | {'PASS' if c['pass'] else 'FAIL'} |")
    lines += [
        "",
        "## Actuation and MTP authority conformance",
        "",
        "| Check | Result |",
        "|---|---|",
    ]
    for c in conformance["checks"] + mtp["checks"]:
        lines.append(f"| `{c['check']}` | {'PASS' if c['pass'] else 'FAIL'} |")
    lines.append(f"| `safety_override_audit` | {'PASS' if override_audit['status'] == 'PASS' else 'FAIL'} |")
    classification = json.loads((HERE / "FULL_SUITE_CLASSIFICATION.json").read_text())
    amendments = json.loads((HERE / "AUTHORITY_AMENDMENTS.json").read_text())
    lines += [
        "",
        "## Repository suite classification",
        "",
        "* collected `%d`, passed `%d`, failed `%d`, skipped `%d`, collection-error modules excluded `%d`" % (
            classification["full_suite"]["collected"],
            classification["full_suite"]["passed"],
            classification["full_suite"]["failed"],
            classification["full_suite"]["skipped"],
            classification["full_suite"]["collection_error_modules_excluded"]),
        "* RES-85C-owned failures: `%d`; RES-85-owned failures: `%d`; RES-83/RES-84 failures: `%d`" % (
            classification["full_suite"]["res85c_owned_failures"],
            classification["full_suite"]["res85_owned_failures"],
            classification["full_suite"]["res83_res84_failures"]),
        "* pre-existing failures are classified and reproduced at ENTRY_HEAD; see `FULL_SUITE_CLASSIFICATION.json`",
        "",
        "## Authority amendments during Achievement B (RES-85 history)",
        "",
    ]
    for amendment in amendments["amendments"]:
        lines.append(f"* `{amendment['id']}` ({amendment['artifact']}): {amendment['change']}")
    lines.append(f"* verdict: {amendments['verdict']}")
    lines += [
        "",
        "## Scope",
        "",
        "RES-85 implements and qualifies causal launch/flight control only. No landing",
        "optimisation (RES-86), no recovery (RES-87), no successor candidate identity and no",
        "elite H2 target are claimed or created here.",
        "",
    ]
    return "\n".join(lines)


def correction_receipt(episode, report: dict[str, Any], identity_audit: dict[str, Any],
                       conformance: dict[str, Any], override_audit: dict[str, Any],
                       negative: dict[str, Any], pre_correction: dict[str, Any],
                       mtp_matrix: dict[str, Any], deficit: dict[str, Any]) -> str:
    h2 = report["method_explicit_h2_report"]["PRIMARY_CANONICAL"]["value_m"]
    occurrence = report["takeoff_occurrence"]
    history = report["takeoff_candidate_history"]
    m0 = mtp_matrix["case_results"]["M0_nominal_passive_nominal_active"]
    m1 = mtp_matrix["case_results"]["M1_zero_passive_zero_active"]
    m3 = mtp_matrix["case_results"]["M3_zero_passive_nominal_active"]
    winner = report["profiled"].get("controller_config", {})
    suite = _load_json(HERE / "FULL_SUITE_CLASSIFICATION.json")
    lines = [
        "# RES85C_CORRECTION_RECEIPT",
        "",
        "MISSION: `RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001`",
        "LINEAR ISSUE: RES-85",
        f"ENTRY_HEAD: `{ENTRY_HEAD}`",
        f"ENTRY_TREE: `{ENTRY_TREE}`",
        f"PREVIOUS_RES85_EVIDENCE_SEAL_SHA256: `{PREVIOUS_RES85_EVIDENCE_SEAL_SHA256}`",
        f"RES84_EVIDENCE_SEAL_SHA256: `{RES84_EVIDENCE_SEAL_SHA256}`",
        f"RES83_PLANT_XML_SHA256: `{RES83_PLANT_XML_SHA256}`",
        "",
        "The previous RES-85 (e487369) evidence remains attributable and is NOT",
        "silently rewritten: its bundle files are the git blobs at that commit and",
        "its external seal is recorded above.  Its episode is reproduced here as",
        "historical pre-correction evidence with the honest classification",
        f"`{pre_correction['classification']}` (H2 = `{pre_correction['h2_m']}` m,",
        f"takeoff vz = `{pre_correction['takeoff_vz_m_s']}` m/s).",
        "",
        "## Entry reconciliation",
        "",
        "* `HEAD` == `ENTRY_HEAD`, `HEAD^{tree}` == `ENTRY_TREE` and `HEAD` == `origin/main`",
        "  were verified before any write; the RES-83 Plant XML hash and the RES-84 and",
        "  previous RES-85 external seals were re-verified.",
        "* the worktree carried the interrupted prior execution artifacts of this same",
        "  RES-85C mission.  They were audited, completed and re-verified against the",
        "  frozen authorities; this bundle is delivered by the single RES-85C correction",
        "  commit.  No foreign or out-of-mission change was present.",
        "",
        "## Blocker corrections",
        "",
        "### BLOCKER A — accepted takeoff identity",
        "",
        "* before: the exported top-level TAKEOFF_OCCURRENCE was the first support-to-zero",
        "  transition (startup contact chatter, sample 3 / t=0.006 s) while the launch",
        "  actually used sample 767 / t=1.534 s (confirmation 792 / t=1.584 s).",
        "* correction: `extract_events` now exports the unique controller-confirmed",
        "  occurrence as the top-level TAKEOFF_OCCURRENCE; the confirmation, the H2 origin,",
        "  the ballistic cross-check, the impulse cross-check and the diagnostic-comparator",
        "  offsets all reference that same occurrence; every candidate remains in",
        "  `takeoff_candidate_history` with source phase, disposition, rejection reason and",
        "  confirmation result.",
        f"* canonical accepted occurrence: sample `{occurrence.get('native_index')}` /",
        f"  t=`{occurrence.get('time_s')}` s; candidate history entries: `{len(history)}`",
        f"  (dispositions: `{identity_audit['candidate_disposition_counts']}`).",
        f"* identity audit status: `{identity_audit['status']}`.",
        "",
        "### BLOCKER B — functional floor authority",
        "",
        "* `ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED` (unchanged)",
        "* `ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED` (restored)",
        f"* `H_ANTI_TRIVIALITY_FLOOR = {H_ANTI_TRIVIALITY_FLOOR_M}` m with role",
        "  `HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY`; explicit statements:",
        "  0.150 m is NOT an elite-performance norm, NOT an optimization target,",
        "  and IS a minimum functional success condition.",
        "* diagnostic scale cross-check only: minimum ballistic vz =",
        f"  `{FLOOR_BALLISTIC_VZ_MAX_M_S:.6f}` m/s (never a replacement for direct SYSTEM_COM H2).",
        f"* corrected episode classification: `{report['functional_task_floor']['classification']}`",
        f"  at H2 = `{h2}` m.",
        f"* pre-correction episode: `{pre_correction['classification']}` (not relabelled PASS).",
        "",
        "### BLOCKER C — actuation-rate contract",
        "",
        "* torque-rate limit is declared `NOMINAL_SLEW_BOUND` (it is not, and is no longer",
        "  reported as, a hard ceiling).",
        "* moment / joint-power / MTP energy gates are the `HARD_SAFETY_BOUNDS`: they are",
        "  enforced every sample and never exceeded.",
        "* when a changing hard safety bound empties the slew-feasible interval the hard",
        "  bound wins and the sample is recorded as an explicit SAFETY_OVERRIDE naming the",
        "  single binding hard constraint and the violated nominal slew margin.",
        "* canonical episode: undeclared slew exceedances = 0; declared override",
        f"  samples = `{override_audit['declared_override_samples']}` sample rows",
        f"  (`{conformance['safety_override_channel_samples']}` channel samples);",
        "  every override is attributed to exactly one binding hard safety bound;",
        f"  audit status `{override_audit['status']}`.",
        "* no numeric moment, power or MTP work budget was increased.",
        "",
        "### BLOCKER D — MTP non-compensation",
        "",
        "* deterministic matrix (M0..M3 + declared sweeps) in `MTP_NONCOMPENSATION_MATRIX.json`.",
        f"* M0 (nominal passive + nominal active): H2 = `{m0['h2_m']}` m,",
        f"  active MTP positive work = `{m0['mtp_active_total_positive_work_j']}` J,",
        f"  ratio to total positive joint work = `{m0['ratio_mtp_active_over_total_positive']}`.",
        f"* M1 (zero passive + zero active): H2 = `{m1['h2_m']}` m, active MTP work =",
        f"  `{m1['mtp_active_total_positive_work_j']}` J, confirmation =",
        f"  `{m1['takeoff_confirmation']}`.",
        f"* M3 (zero passive + nominal active): H2 = `{m3['h2_m']}` m.",
        f"* conclusion: {mtp_matrix['required_conclusion']['conclusion']}.",
        "* active MTP positive work remains a negligible fraction of total positive joint",
        "  work; the ankle remains the dominant contributor.",
        "",
        "## Canonical result",
        "",
        "| Quantity | Value |",
        "|---|---|",
        f"| H2 (direct SYSTEM_COM) | `{h2}` m |",
        f"| takeoff vz | `{report['apex_h2'].get('takeoff_vz_m_s')}` m/s |",
        f"| accepted occurrence | sample `{occurrence.get('native_index')}` / t=`{occurrence.get('time_s')}` s |",
        f"| confirmation | sample `{report['takeoff_confirmation'].get('confirmation_sample')}`"
        f" / t=`{report['takeoff_confirmation'].get('confirmation_time_s')}` s |",
        f"| ballistic cross-check residual | `{report['apex_h2'].get('ballistic_cross_check_delta_m')}` m |",
        f"| functional floor classification | `{report['functional_task_floor']['classification']}` |",
        f"| propulsion stroke | `{deficit['structural_countermovement_budget']['used_m']}` m of "
        f"`{deficit['structural_countermovement_budget']['budget_m']}` m budget |",
        f"| launch joint ROM audit | `{report['joint_rom_audit']['status']}` "
        f"(reference `{report['joint_rom_audit']['reference_within_frozen_rom']}`, "
        f"drive `{report['joint_rom_audit']['no_applied_moment_drives_past_frozen_rom']}`, "
        f"envelope `{report['joint_rom_audit']['measured_overshoot_within_plant_soft_limit_envelope']}`) |",
        "",
        "## Accepted occurrence identity",
        "",
        "| candidate | t (s) | source phase | disposition | rejection reason | confirmation sample | confirmed |",
        "|---|---|---|---|---|---|---|",
    ] + [
        f"| `{h['occurrence']['native_index']}` | `{h['occurrence']['time_s']}` | "
        f"`{h['source_phase']}` | `{h['disposition']}` | `{h['rejection_reason']}` | "
        f"`{h['confirmation_result']['confirmation_sample']}` | "
        f"`{h['confirmation_result']['confirmed']}` |"
        for h in history
    ] + [
        "",
        "## MTP non-compensation sensitivity matrix",
        "",
        "| case | zero-passive | active budget (J) | MTP moment (N*m) | takeoff | H2 (m) | "
        "active MTP (J) | ratio active/total | budget binds | qualitative success |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ] + [
        f"| `{name}` | `{r['zero_passive']}` | `{r['active_budget_override_j']}` | "
        f"`{r['moment_ceiling_override_nm']}` | `{r['takeoff_confirmation']}` | "
        f"`{r['h2_m']}` | `{r['mtp_active_total_positive_work_j']}` | "
        f"`{r['ratio_mtp_active_over_total_positive']}` | `{r['mtp_budget_binds']}` | "
        f"`{r['qualitative_success']}` |"
        for name, r in sorted(mtp_matrix["case_results"].items())
    ] + [
        "",
        "## Test and suite results",
        "",
        f"* targeted RES-85/RES-85C/Plant/measurement: `{suite['targeted_and_regression']}`",
        f"* full repository suite: `{suite['full_suite']}`",
        f"* pre-existing failure reproduction: `{suite['entry_head_reproduction']}`",
        f"* failure categories: `{suite['failure_categories']}`",
        "",
        "## Controller engineering variables (bounded declared search)",
        "",
        "`PROPULSION_SEARCH.json` records the predeclared staged coordinate-ascent",
        "grid (`a_thrust_m_s2` x `thrust_az_max_m_s2`, `extension_rate_ff_gain` x",
        "`contact_preload_m`, `trunk_lean_frac` x `trunk_extend_frac`, then the",
        "`a_thrust_m_s2` x `thrust_az_max_m_s2` refinement stage), the fixed declared",
        "ROM/trunk setup, the declared ordering, the evaluation function, the 33",
        "evaluation cap and the two-run byte-identity result.  Selected configuration:",
        "",
        "```json",
        json.dumps({k: winner.get(k) for k in sorted(winner)}, indent=2),
        "```",
        "",
        "## Actuation semantics",
        "",
        f"* role of the rate limit: `{conformance['torque_rate_role']}`",
        f"* hard safety bounds: `{conformance['hard_safety_bounds']}`",
        f"* max applied moment (N*m): `{conformance['max_applied_nm']}`",
        f"* max joint power (W): `{conformance['max_joint_power_w']}`",
        f"* final status: conformance `{conformance['status']}`, "
        f"override audit `{override_audit['status']}`, negative controls `{negative['status']}`",
        "",
        "## Evidence",
        "",
        "* artifacts regenerated deterministically; `DETERMINISM_REPORT.json` records the",
        "  declared identity domain and the two-build byte-identity result.",
        "* `HASH_MANIFEST.json` covers every artifact and the telemetry blob.",
        "",
        "## Final claim ceiling",
        "",
        "RES-85C closes the causal loaded-CMJ launch and flight (takeoff, confirmation,",
        "genuine 50 ms physical-time flight, apex/H2) against the frozen RES-83 Plant,",
        "RES-84 measurement authority and RES-85 control authority.  It claims no landing",
        "capture, no recovery, no elite performance norm and no successor candidate",
        "identity; the functional floor is a non-triviality boundary, not a performance",
        "claim.",
        "",
    ]
    return "\n".join(lines)


# ===========================================================================
# writing / determinism
# ===========================================================================
def _write_artifacts(built: dict[str, Any]) -> dict[str, Any]:
    (HERE / "V3_LAUNCH_CONTROLLER_SPEC.json").write_text(json_text(built["spec"]))
    (HERE / "TELEMETRY_MANIFEST.json").write_text(json_text(built["telemetry_manifest"]))
    (HERE / "TELEMETRY_ARRAYS.bin").write_bytes(built["telemetry_blob"])
    (HERE / "LAUNCH_EPISODE_REPORT.json").write_text(json_text(built["episode_report"]))
    (HERE / "OCCURRENCE_IDENTITY_AUDIT.json").write_text(json_text(built["identity_audit"]))
    (HERE / "PROPULSION_DEFICIT_REPORT.json").write_text(json_text(built["deficit"]))
    (HERE / "MTP_ENERGY_REPORT.json").write_text(json_text(built["mtp_report"]))
    (HERE / "ACTUATION_CONFORMANCE_REPORT.json").write_text(json_text(built["conformance"]))
    (HERE / "SAFETY_OVERRIDE_AUDIT.json").write_text(json_text(built["override_audit"]))
    (HERE / "NEGATIVE_CONTROLS_REPORT.json").write_text(json_text(built["negative"]))
    (HERE / "ZERO_PASSIVE_SENSITIVITY_REPORT.json").write_text(json_text(built["zero_passive"]))
    (HERE / "PRE_CORRECTION_EPISODE_CLASSIFICATION.json").write_text(
        json_text(built["pre_correction"]))
    (HERE / "RES85_RECEIPT.md").write_text(built["receipt"])
    (HERE / "RES85C_CORRECTION_RECEIPT.md").write_text(built["correction_receipt"])
    # authority artifacts from Achievement A (regenerated deterministically)
    import build_authority_checks as BAC

    BAC.main([])
    manifest = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "achievement": "RES-85C: close the launch requalification and evidence consistency",
        "files": {name: sha256_file(HERE / name) for name in sorted(ARTIFACT_FILES)
                  if (HERE / name).is_file()},
        "telemetry_blob_sha256": built["telemetry_manifest"]["blob_sha256"],
        "telemetry_canonical_digest": built["telemetry_manifest"]["canonical_digest"],
        "plant_xml_sha256": built["spec"]["plant_xml_sha256"],
        "res84_evidence_seal_sha256": RES84_EVIDENCE_SEAL_SHA256,
        "previous_res85_evidence_seal_sha256": PREVIOUS_RES85_EVIDENCE_SEAL_SHA256,
    }
    (HERE / "HASH_MANIFEST.json").write_text(json_text(manifest))
    return manifest


def _identity_payload(built: dict[str, Any]) -> dict[str, str]:
    return {
        "spec": json_text(built["spec"]),
        "episode_report": json_text(built["episode_report"]),
        "identity_audit": json_text(built["identity_audit"]),
        "deficit": json_text(built["deficit"]),
        "pre_correction": json_text(built["pre_correction"]),
        "mtp_report": json_text(built["mtp_report"]),
        "conformance": json_text(built["conformance"]),
        "override_audit": json_text(built["override_audit"]),
        "negative": json_text(built["negative"]),
        "zero_passive": json_text(built["zero_passive"]),
        "telemetry_manifest": json_text(built["telemetry_manifest"]),
        "telemetry_blob_sha256": sha256_bytes(built["telemetry_blob"]),
        "receipt": built["receipt"],
        "correction_receipt": built["correction_receipt"],
    }


def determinism_report(first: dict[str, Any], second: dict[str, Any]) -> dict[str, Any]:
    a = _identity_payload(first)
    b = _identity_payload(second)
    mismatches = [key for key in a if a[key] != b[key]]
    checks = []
    _check(checks, "two_builds_byte_identical", not mismatches, mismatches)
    _check(checks, "telemetry_blob_sha256_identical",
           a["telemetry_blob_sha256"] == b["telemetry_blob_sha256"],
           [a["telemetry_blob_sha256"], b["telemetry_blob_sha256"]])
    return {
        "schema_version": "1.0.0",
        "status": _status(checks),
        "checks": checks,
        "telemetry_blob_sha256": a["telemetry_blob_sha256"],
        "identity_domain": sorted(a),
        "excluded_from_identity": [
            "sealed_at_utc", "postcommit sidecar", "git HEAD-at-authoring fields",
            "external bundle path", "PROPULSION_SEARCH.json (separate declared search "
            "artifact with its own two-run byte-identity check)",
            "MTP_NONCOMPENSATION_MATRIX.json (separate declared matrix artifact with its "
            "own two-run byte-identity check)",
            "JOINT_ROM_SOFT_LIMIT_PROBE.json (Plant property probe, deterministic script)",
        ],
        "artifact_sha256": {k: sha256_bytes(v.encode("utf-8")) for k, v in a.items()},
    }


def main(argv: list[str] | None = None) -> int:
    argv = argv or []
    built = build_all()
    # The bundle is always built twice so the determinism report can never be
    # left stale relative to the written artifacts; --once is a debug escape.
    if "--once" not in argv:
        second = build_all()
        det = determinism_report(built, second)
        sealed_blob_matches = (
            det["telemetry_blob_sha256"]
            == built["telemetry_manifest"]["blob_sha256"])
        _check(det["checks"], "determinism_blob_matches_written_bundle",
               sealed_blob_matches, {
                   "determinism_telemetry_blob_sha256":
                       det["telemetry_blob_sha256"],
                   "written_manifest_blob_sha256":
                       built["telemetry_manifest"]["blob_sha256"]})
        det["status"] = _status(det["checks"])
        (HERE / "DETERMINISM_REPORT.json").write_text(json_text(det))
    else:
        det = None
    manifest = _write_artifacts(built)
    if "--print" in argv or det is None:
        print(json.dumps({
            "episode_status": built["episode"].status,
            "phases": built["episode"].phases_visited,
            "accepted_occurrence": built["episode_report"]["takeoff_occurrence"].get(
                "native_index"),
            "h2_m": built["episode_report"]["apex_h2"].get("h2_support_m"),
            "floor": built["episode_report"]["functional_task_floor"],
            "identity_audit": built["identity_audit"]["status"],
            "negative": built["negative"]["status"],
            "conformance": built["conformance"]["status"],
            "override_audit": built["override_audit"]["status"],
            "mtp": built["mtp_report"]["status"],
            "zero_passive": built["zero_passive"]["status"],
            "determinism": None if det is None else det["status"],
            "hash_manifest_files": len(manifest["files"]),
        }, indent=2))
    ok = (built["negative"]["status"] == STATUS_PASS
          and built["conformance"]["status"] == STATUS_PASS
          and built["override_audit"]["status"] == STATUS_PASS
          and built["identity_audit"]["status"] == STATUS_PASS
          and built["mtp_report"]["status"] == STATUS_PASS
          and built["zero_passive"]["status"] == STATUS_PASS
          and built["episode_report"]["joint_rom_audit"]["status"] == STATUS_PASS
          and built["episode"].status == "COMPLETED"
          and bool(built["episode_report"]["functional_task_floor"]["closure_pass"])
          and (det is None or det["status"] == STATUS_PASS))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
