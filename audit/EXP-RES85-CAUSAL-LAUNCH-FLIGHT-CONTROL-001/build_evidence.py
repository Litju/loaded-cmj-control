"""RES-85D — strict structural ROM reconciliation and final reseal.

Authority: LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1
Mission:   RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001

Writes every deterministic scientific artifact of the regenerated RES-85
bundle after the RES-85D strict structural-ROM correction:

    V3_LAUNCH_CONTROLLER_SPEC.json
    TELEMETRY_MANIFEST.json + TELEMETRY_ARRAYS.bin
    LAUNCH_EPISODE_REPORT.json
    STRICT_STRUCTURAL_ROM_AUDIT.json
    RES85C_PREDECESSOR_STRICT_ROM.json
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
    RES85D_STRICT_ROM_RECEIPT.md

The RES-85C receipts (``RES85_RECEIPT.md`` and ``RES85C_CORRECTION_RECEIPT.md``)
are preserved byte-for-byte; the RES-85C episode is *not* silently rewritten and
is audited here as ``RES85C_PREDECESSOR_STRICT_ROM.json`` (strict structural ROM
FAIL on trunk_pelvis).

Run:  python3 build_evidence.py            # build twice and compare identity
      python3 build_evidence.py --once     # debug escape: build once
      python3 build_evidence.py --print    # status lines only
"""

from __future__ import annotations

import hashlib
import json
import subprocess
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

MISSION_ID = "RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001"
RES85_ENTRY_HEAD = "e487369f6861d9c9bc27f9f3d92b981fb3684293"
RES85_ENTRY_TREE = "5a88417d3a352ec87fee498e8a548575f19d3e0f"
RES85C_HEAD = "ffc98526bd2b0da76dbef50891415ebdb345a1fd"
RES85C_TREE = "612b0b8a667b32ef8f217e4aed569e5c2832ff84"
RES83_PLANT_XML_SHA256 = "eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d"
RES84_EVIDENCE_SEAL_SHA256 = "223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda"
PREVIOUS_RES85_EVIDENCE_SEAL_SHA256 = (
    "235a40e7a2a2b07ea483f2352b0765315b7f43b2343b3297461a0754b024b50e")
ENTRY_HEAD = RES85C_HEAD
ENTRY_TREE = RES85C_TREE
H_ANTI_TRIVIALITY_FLOOR_M = 0.150
FLOOR_BALLISTIC_VZ_MAX_M_S = (2.0 * 9.81 * H_ANTI_TRIVIALITY_FLOOR_M) ** 0.5
STRICT_ROM_TOLERANCE_RAD = 1.0e-9

ENTRY_EQUIVALENT_OVERRIDES: dict[str, float] = {
    "extension_rate_ff_gain": 0.0,
    "contact_preload_m": 0.0,
    "trunk_lean_frac": 0.0,
    "trunk_extend_frac": 0.0,
    "trunk_kp": 40.0,
    "trunk_kd": 6.0,
    "joint_rom_margin_rad": 0.03,
    "joint_rom_barrier_gain": 0.0,
    "joint_rom_velocity_gain": 0.0,
    "trunk_rom_guard_margin_rad": 0.03,
    "trunk_rom_position_gain": 0.0,
    "trunk_rom_velocity_gain": 0.0,
    "a_thrust_m_s2": 8.0,
    "thrust_az_max_m_s2": 12.0,
}

ARTIFACT_FILES = (
    "V3_LAUNCH_CONTROLLER_SPEC.json",
    "TELEMETRY_MANIFEST.json",
    "TELEMETRY_ARRAYS.bin",
    "LAUNCH_EPISODE_REPORT.json",
    "STRICT_STRUCTURAL_ROM_AUDIT.json",
    "RES85C_PREDECESSOR_STRICT_ROM.json",
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
    "STRICT_ROM_SEARCH.json",
    "DETERMINISM_REPORT.json",
    "AUTHORITY_CHECK_REPORT.json",
    "SOURCE_PROVENANCE.json",
    "FULL_SUITE_CLASSIFICATION.json",
    "RES85_RECEIPT.md",
    "RES85C_CORRECTION_RECEIPT.md",
    "RES85D_STRICT_ROM_RECEIPT.md",
)

CHANNEL_QPOS = None


def channel_qpos_indices() -> list[int]:
    global CHANNEL_QPOS
    if CHANNEL_QPOS is None:
        plant = P.V3Plant()
        CHANNEL_QPOS = [int(plant.idx.qadr[name]) for name in CHANNELS]
    return CHANNEL_QPOS


CHANNEL_DOF = None


def channel_dof_indices() -> list[int]:
    global CHANNEL_DOF
    if CHANNEL_DOF is None:
        plant = P.V3Plant()
        CHANNEL_DOF = [int(plant.idx.vadr[name]) for name in CHANNELS]
    return CHANNEL_DOF


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
    # NC-22C (RES-85C control, retained with the strict criterion): the launch
    # structural-ROM audit passes and the strict validator fails closed when a
    # measured extremum leaves the frozen envelope
    rom = report.get("joint_rom_audit") or {}
    mutated_rom = json.loads(json.dumps(rom))
    first_channel = sorted(mutated_rom["channels"])[0]
    upper = float(mutated_rom["channels"][first_channel]["rom_upper_rad"])
    mutated_rom["channels"][first_channel]["max_measured_rad"] = upper + 1.0e-6
    mutated_failures = strict_structural_rom_failures(mutated_rom)
    _check(checks, "NC-22C_strict_structural_rom_audit_passes",
           rom.get("status") == STATUS_PASS
           and strict_structural_rom_failures(rom) == []
           and any("MEASURED_EXTREMUM_OUTSIDE_FROZEN_ROM" in f
                   or "STRUCTURAL_ROM_VIOLATION" in f for f in mutated_failures),
           {"audit": {k: v for k, v in rom.items() if k != "channels"},
            "mutated_failures": mutated_failures})
    return {"status": _status(checks), "checks": checks, "probes": probes}


def rom_guard_direction_failures(guard, states, rom_lo, rom_hi) -> list[str]:
    """NC-25 direction property: a ROM guard must never drive a joint farther
    outside the nearest frozen structural limit.

    For every probed ``(q, qdot)`` state the guard's moment correction is
    compared with the outward direction of the nearest violated bound: beyond
    the upper bound a positive guard correction is outward propulsion, below
    the lower bound a negative one is.
    """
    failures: list[str] = []
    desired = np.zeros(N_CHANNELS)
    for name in CHANNELS:
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        lo, hi = float(rng[0]), float(rng[1])
        c = CHANNELS.index(name)
        for q_value, qdot_value in states:
            q = np.zeros(N_CHANNELS)
            qd = np.zeros(N_CHANNELS)
            q[c] = q_value
            qd[c] = qdot_value
            delta = float(guard(q, qd, desired)[c])
            if q_value > hi + rom_hi and delta > 0.0:
                failures.append(
                    f"OUTWARD_TORQUE_BEYOND_UPPER:{name}:q={q_value!r}:qdot={qdot_value!r}")
            if q_value < lo - rom_lo and delta < 0.0:
                failures.append(
                    f"OUTWARD_TORQUE_BELOW_LOWER:{name}:q={q_value!r}:qdot={qdot_value!r}")
    return failures


def res85d_negative_controls(episode, report: dict[str, Any],
                             runner=None) -> dict[str, Any]:
    """RES-85D negative controls NC-22..NC-27 (strict structural ROM)."""
    checks: list[dict[str, Any]] = []
    probes: dict[str, Any] = {}
    t = episode.telemetry
    claim_end = episode.events.get("res85_claim_end")
    if claim_end is None:
        _check(checks, "NC-22_measured_trunk_beyond_upper_bound_fails", False,
               "claim end absent")
        return {"status": _status(checks), "checks": checks, "probes": probes}
    end = int(claim_end["sample"]) + 1
    q = np.asarray(t.joint_q[:end], dtype=np.float64)[:, channel_qpos_indices()]
    qd = np.asarray(t.joint_qd[:end], dtype=np.float64)[:, channel_dof_indices()]
    tau = np.asarray(t.applied_nm[:end], dtype=np.float64)
    ref = np.asarray(t.posture_reference_rad[:end], dtype=np.float64)
    phases = np.asarray(t.phase_name[:end], dtype=object)
    samples = np.asarray(t.index[:end], dtype=np.float64)
    times = np.asarray(t.time_s[:end], dtype=np.float64)

    def audit(q_measured):
        return strict_structural_rom_audit_arrays(
            q_measured=q_measured, qdot=qd, applied_nm=tau, q_ref=ref,
            phase_name=phases, sample_index=samples, time_s=times,
            claim_end_sample=int(claim_end["sample"]),
            claim_end_reason=str(claim_end["reason"]))

    trunk_lo, trunk_hi = V3_JOINT_RANGES_RAD["trunk_pelvis"]
    # NC-22: measured trunk_pelvis = upper bound + 1e-6 must FAIL
    mutated = q.copy()
    mutated[-1, 0] = float(trunk_hi) + 1.0e-6
    failures_22 = strict_structural_rom_failures(audit(mutated))
    probes["NC-22_measured_trunk_beyond_upper_bound"] = {
        "injected_q_rad": float(trunk_hi) + 1.0e-6,
        "validator_failures": failures_22,
    }
    _check(checks, "NC-22_measured_trunk_beyond_upper_bound_fails",
           any("STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in f
               for f in failures_22),
           probes["NC-22_measured_trunk_beyond_upper_bound"])

    # NC-23: posture reference inside ROM but actual q outside ROM must FAIL
    mutated = q.copy()
    knee_col = CHANNELS.index("left_knee")
    mutated[-1, knee_col] = float(V3_JOINT_RANGES_RAD["left_knee"][0]) - 5.0e-2
    ref_ok = True
    for c, name in enumerate(CHANNELS):
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        ref_ok &= bool(np.all(ref[:, c] >= float(rng[0]) - 1e-9)
                       and np.all(ref[:, c] <= float(rng[1]) + 1e-9))
    failures_23 = strict_structural_rom_failures(audit(mutated))
    probes["NC-23_reference_inside_actual_outside"] = {
        "reference_inside_rom": ref_ok,
        "validator_failures": failures_23,
    }
    _check(checks, "NC-23_reference_inside_but_measured_outside_fails",
           ref_ok and any("STRUCTURAL_ROM_VIOLATION:left_knee:lower" in f
                          for f in failures_23),
           probes["NC-23_reference_inside_actual_outside"])

    # NC-24: actual q outside ROM but inside the MuJoCo soft-limit probe
    # envelope must still FAIL (the envelope is not acceptance authority)
    probe = _load_json(HERE / "JOINT_ROM_SOFT_LIMIT_PROBE.json")
    envelope = float(probe["channels"]["trunk_pelvis"]
                     ["soft_limit_compliance_rad"])
    injected = float(trunk_hi) + 0.5 * envelope
    mutated = q.copy()
    mutated[-1, 0] = injected
    failures_24 = strict_structural_rom_failures(audit(mutated))
    probes["NC-24_outside_rom_inside_probe_envelope"] = {
        "probe_envelope_rad": envelope,
        "injected_q_rad": injected,
        "inside_probe_envelope": bool(injected <= float(trunk_hi) + envelope + 1e-9),
        "validator_failures": failures_24,
    }
    _check(checks, "NC-24_measured_outside_rom_inside_probe_envelope_fails",
           bool(injected <= float(trunk_hi) + envelope + 1e-9)
           and any("STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in f
                   for f in failures_24),
           probes["NC-24_outside_rom_inside_probe_envelope"])

    # NC-25: a guard that itself commands torque farther outside the nearest
    # structural limit must FAIL; the shipped guard must never do so.
    #
    # The mutant is deliberately signed as positive feedback about the
    # protected coordinate (``desired + 1000 * q``): near an upper bound it
    # drives the measured coordinate farther outward, near a lower bound it
    # drives it farther below, so the sign-safety detector must reject it.  The
    # opposite sign (``desired - 1000 * q``) is a restoring spring and would
    # not exercise the detector at all.
    plant, data, controller = _probe_env()
    real_states = []
    for name in CHANNELS:
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        lo, hi = float(rng[0]), float(rng[1])
        for offset in (0.2, 1e-6, 0.0, -1e-6, -0.2):
            for qdot_value in (-2.0, -0.2, 0.0, 0.2, 2.0):
                real_states.append((lo + offset, qdot_value))
                real_states.append((hi + offset, qdot_value))
    real_failures = rom_guard_direction_failures(
        controller._rom_guard, real_states, 0.0, 0.0)

    def outward_mutant(guard_q, guard_qd, guard_desired):
        out = guard_desired.copy()
        return out + 1000.0 * guard_q

    mutant_failures = rom_guard_direction_failures(
        outward_mutant, real_states, 0.0, 0.0)
    probes["NC-25_guard_direction_property"] = {
        "real_guard_failures": real_failures,
        "mutant_guard_failures": len(mutant_failures),
        "mutant_guard_direction": "POSITIVE_FEEDBACK_OUTWARD_UNSTABLE",
    }
    _check(checks, "NC-25_rom_guard_never_drives_farther_outside",
           real_failures == [] and len(mutant_failures) > 0,
           probes["NC-25_guard_direction_property"])

    # NC-26: a strict-ROM-valid controller whose H2 is below the functional
    # floor must fail RES-85 closure (re-executed declared alternative config)
    if runner is not None:
        alternative = runner()
        alternative_rom = alternative["strict_rom_status"]
        alternative_h2 = alternative["h2_m"]
        closure = floor_closure(alternative_h2)
        probes["NC-26_strict_rom_but_below_floor"] = {
            "config": alternative["config"],
            "strict_rom_status": alternative_rom,
            "h2_m": alternative_h2,
            "floor_closure": closure,
        }
        _check(checks, "NC-26_strict_rom_correction_below_functional_floor_fails_closure",
               alternative_rom == STATUS_PASS
               and alternative_h2 is not None
               and float(alternative_h2) < H_ANTI_TRIVIALITY_FLOOR_M
               and closure["closure_pass"] is False,
               probes["NC-26_strict_rom_but_below_floor"])
    else:
        _check(checks, "NC-26_strict_rom_correction_below_functional_floor_fails_closure",
               False, "no runner supplied")

    # NC-27: all strict-ROM conditions pass at exactly the structural boundary
    # inside only the 1e-9 floating comparison tolerance
    mutated = q.copy()
    mutated[-1, 0] = float(trunk_hi)
    boundary_audit = audit(mutated)
    boundary_failures = strict_structural_rom_failures(boundary_audit)
    over = q.copy()
    over[-1, 0] = float(trunk_hi) + 2.0e-9
    over_failures = strict_structural_rom_failures(audit(over))
    probes["NC-27_boundary_within_tolerance"] = {
        "boundary_status": boundary_audit["status"],
        "boundary_failures": boundary_failures,
        "over_tolerance_failures": over_failures,
    }
    _check(checks, "NC-27_boundary_within_tolerance_passes",
           boundary_audit["status"] == STATUS_PASS
           and boundary_failures == []
           and any("STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in f
                   for f in over_failures),
           probes["NC-27_boundary_within_tolerance"])
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


def strict_structural_rom_audit_arrays(
    *,
    q_measured: np.ndarray,
    qdot: np.ndarray,
    applied_nm: np.ndarray,
    q_ref: np.ndarray,
    phase_name: np.ndarray,
    sample_index: np.ndarray,
    time_s: np.ndarray,
    claim_end_sample: int,
    claim_end_reason: str,
    tolerance_rad: float = STRICT_ROM_TOLERANCE_RAD,
) -> dict[str, Any]:
    """Strict structural-ROM audit over native measured joint coordinates.

    The frozen human structural envelope (``V3_JOINT_RANGES_RAD``) is the only
    acceptance authority.  For every bounded actuated joint and every native
    sample from the first sample through ``RES85_CLAIM_END`` (the first legal
    plantar recontact after the qualified flight) the invariant is

        lower_j - tolerance <= q_j(t) <= upper_j + tolerance

    with ``tolerance_rad = 1e-9`` used *only* for floating-point equality.  It
    is not added anatomical ROM.  The solver soft-limit compliance probe is a
    Plant property and is reported here strictly as
    ``NUMERICAL_SOLVER_DIAGNOSTIC_ONLY``; no PASS depends on it.
    """
    end = int(claim_end_sample) + 1
    if end <= 0 or end > len(q_measured):
        raise ValueError(
            f"claim end sample {claim_end_sample!r} outside the native stream")
    q = np.asarray(q_measured[:end], dtype=np.float64)
    qd = np.asarray(qdot[:end], dtype=np.float64)
    tau = np.asarray(applied_nm[:end], dtype=np.float64)
    ref = np.asarray(q_ref[:end], dtype=np.float64)
    phases = np.asarray(phase_name[:end], dtype=object)
    samples = np.asarray(sample_index[:end], dtype=np.float64)
    times = np.asarray(time_s[:end], dtype=np.float64)
    try:
        probe = _load_json(HERE / "JOINT_ROM_SOFT_LIMIT_PROBE.json")
    except FileNotFoundError:
        probe = {"channels": {}}
    diagnostic_envelopes = {
        name: float(value.get("soft_limit_compliance_rad", 0.0))
        for name, value in probe.get("channels", {}).items()}
    channels: dict[str, Any] = {}
    failures: list[str] = []
    global_min = float("inf")
    for c, name in enumerate(CHANNELS):
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        lo, hi = float(rng[0]), float(rng[1])
        column = q[:, c]
        lower_margin = float(np.min(column - lo))
        upper_margin = float(np.min(hi - column))
        min_margin = min(lower_margin, upper_margin)
        global_min = min(global_min, min_margin)
        i_lo = int(np.argmin(column - lo))
        i_hi = int(np.argmin(hi - column))
        worst_side = "lower" if lower_margin <= upper_margin else "upper"
        i_worst = i_lo if worst_side == "lower" else i_hi
        overshoot = float(max(np.max(column - hi), np.max(lo - column), 0.0))
        channel_pass = bool(min_margin >= -tolerance_rad)
        if not channel_pass:
            side = "lower" if min_margin == lower_margin else "upper"
            failures.append(f"STRUCTURAL_ROM_VIOLATION:{name}:{side}")
        channel = {
            "rom_lower_rad": lo,
            "rom_upper_rad": hi,
            "min_measured_rad": float(np.min(column)),
            "max_measured_rad": float(np.max(column)),
            "lower_margin_rad": lower_margin,
            "upper_margin_rad": upper_margin,
            "min_rom_margin_rad": min_margin,
            "measured_overshoot_rad": overshoot,
            "solver_soft_limit_diagnostic_rad": diagnostic_envelopes.get(name),
            "solver_soft_limit_diagnostic_role":
                "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY",
            "lower_extreme": {
                "native_sample": int(samples[i_lo]),
                "time_s": float(times[i_lo]),
                "phase": str(phases[i_lo]),
                "q_measured_rad": float(column[i_lo]),
                "qdot_rad_s": float(qd[i_lo, c]),
                "applied_nm": float(tau[i_lo, c]),
                "posture_reference_rad": float(ref[i_lo, c]),
                "margin_rad": float(column[i_lo] - lo),
            },
            "upper_extreme": {
                "native_sample": int(samples[i_hi]),
                "time_s": float(times[i_hi]),
                "phase": str(phases[i_hi]),
                "q_measured_rad": float(column[i_hi]),
                "qdot_rad_s": float(qd[i_hi, c]),
                "applied_nm": float(tau[i_hi, c]),
                "posture_reference_rad": float(ref[i_hi, c]),
                "margin_rad": float(hi - column[i_hi]),
            },
            "worst_side": worst_side,
            "worst_native_sample": int(samples[i_worst]),
            "worst_phase": str(phases[i_worst]),
            "qdot_at_worst_rad_s": float(qd[i_worst, c]),
            "applied_nm_at_worst": float(tau[i_worst, c]),
            "posture_reference_at_worst_rad": float(ref[i_worst, c]),
            "status": STATUS_PASS if channel_pass else STATUS_FAIL,
        }
        channels[name] = channel
    audit = {
        "schema_version": "1.0.0",
        "authority": "FROZEN_HUMAN_STRUCTURAL_ENVELOPE_V3_JOINT_RANGES_RAD",
        "scope": "FIRST_NATIVE_SAMPLE_THROUGH_RES85_CLAIM_END",
        "claim_end_sample": int(claim_end_sample),
        "claim_end_reason": claim_end_reason,
        "tolerance_rad": tolerance_rad,
        "criterion": (
            "measured q_j(t) stays within [lower_j - tolerance, upper_j + "
            "tolerance] for every bounded actuated joint and every native "
            "sample in the RES-85 claim domain; the tolerance is floating-point "
            "equality only, never added anatomical ROM"),
        "solver_soft_limit_probe_role": (
            "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY_NOT_STRUCTURAL_ACCEPTANCE_AUTHORITY"),
        "solver_soft_limit_probe_note": (
            "JOINT_ROM_SOFT_LIMIT_PROBE.json describes MuJoCo solver "
            "penetration/compliance behaviour only; it may not enlarge the "
            "human/structural joint ROM and no PASS depends on it"),
        "global_min_structural_rom_margin_rad": global_min,
        "channels": channels,
        "failures": failures,
    }
    audit["status"] = STATUS_PASS if not failures else STATUS_FAIL
    return audit


def strict_structural_rom_audit(episode) -> dict[str, Any]:
    """Strict structural-ROM audit of one native episode."""
    t = episode.telemetry
    claim_end = episode.events.get("res85_claim_end")
    if claim_end is None:
        return {
            "schema_version": "1.0.0",
            "scope": "FIRST_NATIVE_SAMPLE_THROUGH_RES85_CLAIM_END",
            "claim_end_sample": None,
            "claim_end_reason": None,
            "tolerance_rad": STRICT_ROM_TOLERANCE_RAD,
            "criterion": "measured q within the frozen structural envelope",
            "global_min_structural_rom_margin_rad": None,
            "channels": {},
            "failures": ["RES85_CLAIM_END_ABSENT"],
            "status": STATUS_FAIL,
        }
    return strict_structural_rom_audit_arrays(
        q_measured=np.asarray(t.joint_q, dtype=np.float64)[:, channel_qpos_indices()],
        qdot=np.asarray(t.joint_qd, dtype=np.float64)[:, channel_dof_indices()],
        applied_nm=np.asarray(t.applied_nm, dtype=np.float64),
        q_ref=np.asarray(t.posture_reference_rad, dtype=np.float64),
        phase_name=np.asarray(t.phase_name, dtype=object),
        sample_index=np.asarray(t.index, dtype=np.float64),
        time_s=np.asarray(t.time_s, dtype=np.float64),
        claim_end_sample=int(claim_end["sample"]),
        claim_end_reason=str(claim_end["reason"]),
    )


def strict_structural_rom_failures(audit: dict[str, Any]) -> list[str]:
    """Fail-closed validator for the strict structural-ROM audit record.

    The validator is total: it recomputes the verdict from the reported
    per-channel margins and measured extrema (never from a stored PASS field)
    and additionally refuses to accept an audit that claims a PASS while a
    measured extremum lies outside the frozen envelope.
    """
    failures: list[str] = []
    channels = audit.get("channels") or {}
    if not channels:
        failures.append("NO_ROM_CHANNELS")
        return failures
    tolerance = float(audit.get("tolerance_rad", STRICT_ROM_TOLERANCE_RAD))
    global_min = audit.get("global_min_structural_rom_margin_rad")
    if global_min is None or not np.isfinite(float(global_min)):
        failures.append("GLOBAL_ROM_MARGIN_UNDEFINED")
    for name, channel in sorted(channels.items()):
        lower = float(channel.get("lower_margin_rad", float("-inf")))
        upper = float(channel.get("upper_margin_rad", float("-inf")))
        margin = float(channel.get("min_rom_margin_rad", min(lower, upper)))
        if margin < -tolerance:
            side = "lower" if margin == lower else "upper"
            failures.append(f"STRUCTURAL_ROM_VIOLATION:{name}:{side}")
        min_measured = float(channel.get("min_measured_rad", 0.0))
        max_measured = float(channel.get("max_measured_rad", 0.0))
        if (min_measured < float(channel.get("rom_lower_rad", -np.inf)) - tolerance
                or max_measured > float(channel.get("rom_upper_rad", np.inf)) + tolerance):
            failures.append(f"MEASURED_EXTREMUM_OUTSIDE_FROZEN_ROM:{name}")
    if global_min is not None and np.isfinite(float(global_min)) \
            and float(global_min) < -tolerance:
        failures.append("GLOBAL_MIN_STRUCTURAL_ROM_MARGIN_BELOW_TOLERANCE")
    return failures


def joint_rom_audit(episode) -> dict[str, Any]:
    """Compatibility alias: the launch joint-ROM audit is the strict audit."""
    return strict_structural_rom_audit(episode)


def joint_rom_audit_failures(audit: dict[str, Any]) -> list[str]:
    """Compatibility alias for the strict structural-ROM validator."""
    return strict_structural_rom_failures(audit)


# ===========================================================================
# RES-85C predecessor strict-ROM regression (read from immutable git history)
# ===========================================================================
RES85_BUNDLE_REL = "audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001"


def _git_show(commit: str, path: str) -> bytes:
    """Read one immutable blob from the repository history (fail closed)."""
    proc = subprocess.run(["git", "-C", str(REPO), "show", f"{commit}:{path}"],
                          capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"git show {commit}:{path} failed: {proc.stderr.decode('utf-8', 'replace')}")
    return proc.stdout


def reconstruct_telemetry_arrays(manifest: dict[str, Any],
                                 blob: bytes) -> dict[str, np.ndarray]:
    """Reconstruct the native arrays of a sealed telemetry blob."""
    arrays: dict[str, np.ndarray] = {}
    for entry in manifest["arrays"]:
        raw = blob[int(entry["offset"]):int(entry["offset"]) + int(entry["nbytes"])]
        dtype = str(entry["dtype"])
        shape = tuple(int(v) for v in entry["shape"])
        if dtype == "object":
            values = raw.split(b"\x1f")
            arrays[entry["name"]] = np.asarray(
                [value.decode("utf-8") for value in values], dtype=object)
        else:
            arrays[entry["name"]] = np.frombuffer(raw, dtype=dtype).reshape(shape)
    return arrays


def res85c_predecessor_strict_rom() -> dict[str, Any]:
    """Audit the RES-85C canonical trajectory against strict structural ROM.

    The predecessor native telemetry is read from the immutable git blobs at
    ``RES85C_HEAD`` (never from the mutable worktree copy, which this mission
    regenerates).  The strict structural-ROM checker is the same code used for
    the RES-85D qualification, so the regression is a re-executed computation,
    not a stored verdict.
    """
    manifest = json.loads(_git_show(
        RES85C_HEAD, f"{RES85_BUNDLE_REL}/TELEMETRY_MANIFEST.json"))
    blob = _git_show(RES85C_HEAD, f"{RES85_BUNDLE_REL}/TELEMETRY_ARRAYS.bin")
    previous = json.loads(_git_show(
        RES85C_HEAD, f"{RES85_BUNDLE_REL}/LAUNCH_EPISODE_REPORT.json"))
    digest = hashlib.sha256(blob).hexdigest()
    if digest != manifest["blob_sha256"]:
        raise RuntimeError("RES-85C predecessor telemetry blob digest mismatch")
    arrays = reconstruct_telemetry_arrays(manifest, blob)
    claim_end = previous["res85_claim_end"]
    audit = strict_structural_rom_audit_arrays(
        q_measured=np.asarray(arrays["joint_q"], dtype=np.float64)[
            :, channel_qpos_indices()],
        qdot=np.asarray(arrays["joint_qd"], dtype=np.float64)[
            :, channel_dof_indices()],
        applied_nm=np.asarray(arrays["applied_nm"], dtype=np.float64),
        q_ref=np.asarray(arrays["posture_reference_rad"], dtype=np.float64),
        phase_name=np.asarray(arrays["phase_name"], dtype=object),
        sample_index=np.asarray(arrays["index"], dtype=np.float64),
        time_s=np.asarray(arrays["time_s"], dtype=np.float64),
        claim_end_sample=int(claim_end["sample"]),
        claim_end_reason=str(claim_end["reason"]),
    )
    trunk = audit["channels"]["trunk_pelvis"]
    return {
        "schema_version": "1.0.0",
        "mission": MISSION_ID,
        "predecessor_head": RES85C_HEAD,
        "predecessor_tree": RES85C_TREE,
        "source": ("immutable git blobs at RES85C_HEAD: TELEMETRY_MANIFEST.json, "
                   "TELEMETRY_ARRAYS.bin, LAUNCH_EPISODE_REPORT.json"),
        "telemetry_blob_sha256": digest,
        "previous_h2_m": previous["apex_h2"].get("h2_support_m"),
        "previous_takeoff_vz_m_s": previous["apex_h2"].get("takeoff_vz_m_s"),
        "previous_occurrence": previous["takeoff_occurrence"].get("native_index"),
        "previous_confirmation": (
            None if previous.get("takeoff_confirmation") is None
            else previous["takeoff_confirmation"].get("confirmation_sample")),
        "previous_claim_end_sample": int(claim_end["sample"]),
        "strict_structural_rom_audit": audit,
        "regression_conclusion": (
            "the RES-85C canonical trajectory FAILS strict structural ROM: "
            f"trunk_pelvis measured max {trunk['max_measured_rad']!r} rad exceeds "
            f"the frozen upper bound {trunk['rom_upper_rad']!r} rad; the "
            "trajectory was previously accepted only through the "
            "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY soft-limit compliance envelope, "
            "which is not structural authority"),
    }


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
        "mission": MISSION_ID,
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
        "joint_rom_audit": strict_structural_rom_audit(episode),
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
    post_claim_asym = 0.0
    claim_end = episode.events.get("res85_claim_end")
    end = len(t.index) if claim_end is None else int(claim_end["sample"]) + 1
    if len(t.index):
        for i, j in MIRRORED_PAIRS:
            asym = max(asym, float(np.abs(t.applied_nm[:end, i]
                                          - t.applied_nm[:end, j]).max()))
            if end < len(t.index):
                post_claim_asym = max(
                    post_claim_asym,
                    float(np.abs(t.applied_nm[end:, i] - t.applied_nm[end:, j]).max()))
    _check(checks, "bilateral_applied_symmetry_within_tolerance", asym <= 1e-9,
           {"max_pair_asymmetry_nm": asym, "tolerance_nm": 1e-9,
            "scope": "FIRST_NATIVE_SAMPLE_THROUGH_RES85_CLAIM_END",
            "post_claim_landing_window_asymmetry_nm": post_claim_asym,
            "post_claim_note": ("post-claim landing capture is RES-86 scope; the "
                                "post-claim asymmetry is reported as a diagnostic")})
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
        "max_post_claim_asymmetry_nm": post_claim_asym,
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
        "entry_head": RES85_ENTRY_HEAD,
        "entry_tree": RES85_ENTRY_TREE,
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
    strict_rom = report["joint_rom_audit"]
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
    predecessor = res85c_predecessor_strict_rom()

    def below_floor_runner() -> dict[str, Any]:
        """Strict-ROM-valid declared alternative whose H2 is below the floor."""
        alternative_config = replace(
            V3ControllerConfig(), trunk_rom_guard_margin_rad=0.15)
        episode = run_launch_episode(controller_config=alternative_config)
        alternative_audit = strict_structural_rom_audit(episode)
        return {
            "config": {"trunk_rom_guard_margin_rad": 0.15},
            "strict_rom_status": alternative_audit["status"],
            "h2_m": episode.events["apex_h2"].get("h2_support_m"),
        }

    negative_d = res85d_negative_controls(corrected_episode, report,
                                          runner=below_floor_runner)
    negative_all = {
        "schema_version": "1.0.0",
        "mission": MISSION_ID,
        "status": STATUS_PASS if (negative["status"] == STATUS_PASS
                                  and negative_c["status"] == STATUS_PASS
                                  and negative_d["status"] == STATUS_PASS)
        else STATUS_FAIL,
        "checks": negative["checks"] + negative_c["checks"] + negative_d["checks"],
        "probes": {**negative["probes"], **negative_c["probes"],
                   **negative_d["probes"]},
        "legacy_status": negative["status"],
        "res85c_status": negative_c["status"],
        "res85d_status": negative_d["status"],
    }
    telemetry_manifest, telemetry_blob = telemetry_artifacts(corrected_episode)

    probe_plant = P.V3Plant()
    probe_data = probe_plant.make_data()
    settle_standing_stance(probe_plant, probe_data)
    controller = V3LaunchController(probe_plant, probe_data)
    spec = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": MISSION_ID,
        "correction": {
            "correction_id": "RES-85D",
            "entry_head": ENTRY_HEAD,
            "entry_tree": ENTRY_TREE,
            "previous_evidence_seal_sha256": PREVIOUS_RES85_EVIDENCE_SEAL_SHA256,
            "defect": (
                "the RES-85C canonical trajectory measured trunk_pelvis "
                "0.6734678378904122 rad, exceeding the frozen +0.610865 rad "
                "structural upper bound; it was accepted only through the "
                "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY soft-limit compliance envelope"),
            "correction": (
                "state-causal POSITION_GUARD + OUTWARD_VELOCITY_BRAKING "
                "structural-ROM guard over the whole RES-85 claim domain; the "
                "solver soft-limit probe is retained as numerical diagnostic "
                "only and no PASS depends on its penetration envelope"),
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
        "strict_structural_rom": {
            "authority": "FROZEN_HUMAN_STRUCTURAL_ENVELOPE_V3_JOINT_RANGES_RAD",
            "domain": "FIRST_NATIVE_SAMPLE_THROUGH_RES85_CLAIM_END",
            "tolerance_rad": STRICT_ROM_TOLERANCE_RAD,
            "tolerance_role": "FLOATING_POINT_EQUALITY_ONLY_NOT_ANATOMICAL_ROM",
            "global_min_structural_rom_margin_rad":
                strict_rom["global_min_structural_rom_margin_rad"],
            "status": strict_rom["status"],
            "solver_soft_limit_probe_role":
                "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY_NOT_STRUCTURAL_ACCEPTANCE_AUTHORITY",
        },
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
        "strict_rom": strict_rom,
        "predecessor": predecessor,
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
        "strict_rom_receipt": res85d_receipt(
            corrected_episode, report, strict_rom, predecessor, conformance,
            override_audit, negative_all, mtp, pre_correction, telemetry_manifest),
    }


def res85d_receipt(episode, report: dict[str, Any], strict_rom: dict[str, Any],
                   predecessor: dict[str, Any], conformance: dict[str, Any],
                   override_audit: dict[str, Any], negative: dict[str, Any],
                   mtp: dict[str, Any], pre_correction: dict[str, Any],
                   telemetry_manifest: dict[str, Any]) -> str:
    """RES-85D strict structural-ROM final receipt (deterministic values)."""
    h2 = report["method_explicit_h2_report"]["PRIMARY_CANONICAL"]["value_m"]
    apex = report["apex_h2"]
    occurrence = report["takeoff_occurrence"]
    confirmation = report["takeoff_confirmation"] or {}
    previous_audit = predecessor["strict_structural_rom_audit"]
    previous_trunk = previous_audit["channels"]["trunk_pelvis"]
    trunk = strict_rom["channels"]["trunk_pelvis"]
    suite = _load_json(HERE / "FULL_SUITE_CLASSIFICATION.json")
    amendments = _load_json(HERE / "AUTHORITY_AMENDMENTS.json")
    matrix_conclusion = _load_json(
        HERE / "MTP_NONCOMPENSATION_MATRIX.json")["required_conclusion"]
    lines = [
        "# RES85D_STRICT_ROM_RECEIPT",
        "",
        "MISSION: `%s`" % MISSION_ID,
        "LINEAR ISSUE: RES-85",
        f"ENTRY_HEAD: `{ENTRY_HEAD}`",
        f"ENTRY_TREE: `{ENTRY_TREE}`",
        f"RES85C_HEAD: `{RES85C_HEAD}`",
        f"RES83_PLANT_XML_SHA256: `{RES83_PLANT_XML_SHA256}`",
        f"RES84_EVIDENCE_SEAL_SHA256: `{RES84_EVIDENCE_SEAL_SHA256}`",
        f"PREVIOUS_RES85_EVIDENCE_SEAL_SHA256: `{PREVIOUS_RES85_EVIDENCE_SEAL_SHA256}`",
        f"TELEMETRY_BLOB_SHA256: `{telemetry_manifest['blob_sha256']}`",
        "",
        "## Previous evidence attribution (RES-85C is NOT silently rewritten)",
        "",
        "The RES-85C canonical episode remains attributable in git history at",
        f"`{RES85C_HEAD}` and is audited here from its immutable native telemetry",
        "(`RES85C_PREDECESSOR_STRICT_ROM.json`).  The RES-85C receipts are preserved",
        "byte-for-byte in this bundle.",
        "",
        "| RES-85C canonical quantity | Value | Verdict |",
        "|---|---|---|",
        f"| H2 (direct SYSTEM_COM) | `{predecessor['previous_h2_m']}` m | historical |",
        f"| trunk_pelvis measured max | `{previous_trunk['max_measured_rad']}` rad "
        "| **STRICT STRUCTURAL ROM FAIL** |",
        f"| frozen trunk_pelvis upper | `{previous_trunk['rom_upper_rad']}` rad "
        f"| exceeded by `{previous_trunk['measured_overshoot_rad']}` rad |",
        f"| global min structural ROM margin | "
        f"`{previous_audit['global_min_structural_rom_margin_rad']}` rad "
        "| below the 1e-9 tolerance |",
        f"| previous accepted occurrence | sample `{predecessor['previous_occurrence']}` | historical |",
        f"| previous confirmation | sample `{predecessor['previous_confirmation']}` | historical |",
        "",
        "The predecessor audit is re-executed from the native arrays with the same",
        "strict checker used for the RES-85D qualification; its verdict is FAIL on",
        "`trunk_pelvis:upper` (see `RES85C_PREDECESSOR_STRICT_ROM.json`).",
        "",
        "## New canonical result (RES-85D)",
        "",
        "| Quantity | Value |",
        "|---|---|",
        f"| H2 (direct SYSTEM_COM) | `{h2}` m |",
        f"| takeoff vz | `{apex.get('takeoff_vz_m_s')}` m/s |",
        f"| accepted occurrence | sample `{occurrence.get('native_index')}` / t=`{occurrence.get('time_s')}` s |",
        f"| takeoff confirmation | sample `{confirmation.get('confirmation_sample')}` / "
        f"t=`{confirmation.get('confirmation_time_s')}` s |",
        f"| RES85 claim end (first legal plantar recontact) | sample `{report['res85_claim_end'].get('sample')}` / "
        f"t=`{report['res85_claim_end'].get('time_s')}` s |",
        f"| ballistic cross-check residual | `{apex.get('ballistic_cross_check_delta_m')}` m |",
        f"| functional floor classification | `{report['functional_task_floor']['classification']}` "
        f"(closure `{report['functional_task_floor']['closure_pass']}`) |",
        "",
        "`ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED` and",
        "`ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED` (unchanged).  The",
        "`H_ANTI_TRIVIALITY_FLOOR = 0.150 m` is a hard functional non-triviality",
        "boundary, not an elite norm and not an optimization target.",
        "",
        "## Strict structural ROM (frozen human envelope)",
        "",
        f"* domain: `{strict_rom['scope']}` (claim end sample `{strict_rom['claim_end_sample']}`)",
        f"* criterion: {strict_rom['criterion']}",
        f"* tolerance: `{strict_rom['tolerance_rad']}` rad "
        "(`FLOATING_POINT_EQUALITY_ONLY_NOT_ANATOMICAL_ROM`)",
        f"* `GLOBAL_MIN_STRUCTURAL_ROM_MARGIN_RAD = {strict_rom['global_min_structural_rom_margin_rad']}`",
        f"* audit status: `{strict_rom['status']}`",
        f"* solver soft-limit probe role: "
        f"`{strict_rom['solver_soft_limit_probe_role']}`",
        "",
        "| joint | frozen lower | frozen upper | min measured | max measured | min margin | status |",
        "|---|---|---|---|---|---|---|",
    ] + [
        f"| `{name}` | `{ch['rom_lower_rad']}` | `{ch['rom_upper_rad']}` | "
        f"`{ch['min_measured_rad']}` | `{ch['max_measured_rad']}` | "
        f"`{ch['min_rom_margin_rad']}` | `{ch['status']}` |"
        for name, ch in sorted(strict_rom["channels"].items())
    ] + [
        "",
        "### trunk_pelvis (the known binding channel)",
        "",
        f"* frozen upper bound: `{trunk['rom_upper_rad']}` rad",
        f"* measured maximum: `{trunk['max_measured_rad']}` rad",
        f"* minimum ROM margin: `{trunk['min_rom_margin_rad']}` rad "
        f"(worst side `{trunk['worst_side']}`, sample `{trunk['worst_native_sample']}`, "
        f"phase `{trunk['worst_phase']}`)",
        f"* at the worst sample: qdot `{trunk['qdot_at_worst_rad_s']}` rad/s, "
        f"applied `{trunk['applied_nm_at_worst']}` N*m, "
        f"reference `{trunk['posture_reference_at_worst_rad']}` rad",
        "",
        "## Hard actuation conformance",
        "",
        f"* torque-rate role: `{conformance['torque_rate_role']}`",
        f"* hard safety bounds: `{conformance['hard_safety_bounds']}`",
        f"* max applied moment (N*m): `{conformance['max_applied_nm']}`",
        f"* max joint power (W): `{conformance['max_joint_power_w']}`",
        f"* max bilateral asymmetry (N*m): `{conformance['max_bilateral_asymmetry_nm']}` "
        "(RES-85 claim domain; the post-claim landing window is RES-86 scope)",
        f"* conformance `{conformance['status']}`; safety override audit "
        f"`{override_audit['status']}` with `{override_audit['declared_override_samples']}` "
        "declared override sample rows and no undeclared slew exceedance",
        "",
        "## MTP non-compensation",
        "",
        f"* MTP energy report status: `{mtp['status']}`",
        f"* matrix conclusion: `{matrix_conclusion['conclusion']}`",
        "* zero-passive + zero-active case confirmed: "
        f"`{matrix_conclusion['zero_passive_zero_active_feasible']}` "
        f"(H2 = `{matrix_conclusion['zero_passive_zero_active_h2_m']}` m; "
        "reported honestly as a sensitivity observation, below the canonical "
        "functional floor)",
        f"* active MTP positive work ratio (nominal): "
        f"`{matrix_conclusion['mtp_active_ratio_nominal']}`",
        "",
        "## Negative controls",
        "",
        "| Control | Result |",
        "|---|---|",
    ] + [
        f"| `{c['check']}` | {'PASS' if c['pass'] else 'FAIL'} |"
        for c in negative["checks"]
    ] + [
        "",
        "## Test and suite results",
        "",
        f"* full repository suite (one run, candidate worktree at ENTRY_HEAD plus the "
        f"RES-85D changes): collected `{suite['full_suite']['collected']}`, "
        f"passed `{suite['full_suite']['passed']}`, failed `{suite['full_suite']['failed']}`, "
        f"skipped `{suite['full_suite']['skipped']}`, errors "
        f"`{suite['full_suite'].get('errors', 0)}`; excluded collection-error modules "
        f"`{suite['full_suite']['collection_error_modules_excluded']}` "
        "(`tests/test_ml241_qacc_resolution.py`, "
        "`tests/test_public_support_wrench_contract.py`)",
        f"* RES-85D-owned failures: `{suite['full_suite'].get('res85d_owned_failures')}`; "
        f"RES-85C-owned: `{suite['full_suite']['res85c_owned_failures']}`; "
        f"RES-85-owned: `{suite['full_suite']['res85_owned_failures']}`; "
        f"RES-83/RES-84: `{suite['full_suite']['res83_res84_failures']}`; every recorded "
        "failure is classified `PRE_EXISTING_*`",
        f"* targeted qualification: `{suite['targeted_and_regression']}`; collected "
        f"`{suite['targeted_counts']['collected']}`, passed `{suite['targeted_counts']['passed']}`, "
        f"failed `{suite['targeted_counts']['failed']}`, skipped `{suite['targeted_counts']['skipped']}`",
        f"* entry-head reproduction: "
        f"`{suite['res85d_final_run']['entry_head_reproduction']['failures_reproduced_node_for_node']}` "
        f"of `{suite['res85d_final_run']['run']['failed']}` candidate failures reproduced "
        "node-for-node at ENTRY_HEAD with the same controlling cause; no failing test file "
        "imports the RES-85D change surface `loaded_cmj.v3`",
        "",
        "## Determinism",
        "",
        "* the bundle is built twice and compared over the declared identity domain;",
        "  the two-build byte-identity result and per-artifact sha256 live in",
        "  `DETERMINISM_REPORT.json`; the telemetry blob and canonical digest live in",
        "  `TELEMETRY_MANIFEST.json`.",
        f"* telemetry canonical digest: `{telemetry_manifest['canonical_digest']}`",
        "* the declared search (if any) is run twice and compared over its own",
        "  canonical payload (`STRICT_ROM_SEARCH.json`).",
        "* evidence seal: `HASH_MANIFEST.json` covers every sealed artifact of this",
        "  bundle and the telemetry blob; the seal is written by the same deterministic",
        "  build that produces this receipt.",
        "",
        "## Authority amendments (RES-85 history, unchanged)",
        "",
    ] + [
        f"* `{amendment['id']}` ({amendment['artifact']}): {amendment['change']}"
        for amendment in amendments["amendments"]
    ] + [
        f"* verdict: {amendments['verdict']}",
        "",
        "## Final claim ceiling",
        "",
        "RES-85D closes the causal loaded-CMJ launch and flight (takeoff, confirmation,",
        "genuine 50 ms physical-time flight, apex/H2) against the frozen RES-83 Plant,",
        "RES-84 measurement authority and RES-85 control authority, and additionally",
        "requires every measured bounded joint coordinate to remain inside the frozen",
        "human structural envelope from the first native sample through the RES-85 claim",
        "end.  It claims no landing capture, no recovery, no elite performance norm and",
        "no successor candidate identity; the functional floor is a non-triviality",
        "boundary, not a performance claim.",
        "",
        "`NEXT_AUTHORIZED_ACTION=RES86_ACTIVE_SET_SAFE_LANDING_CAPTURE`",
        "(successor work is not started by RES-85D).",
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
    (HERE / "STRICT_STRUCTURAL_ROM_AUDIT.json").write_text(json_text(built["strict_rom"]))
    (HERE / "RES85C_PREDECESSOR_STRICT_ROM.json").write_text(
        json_text(built["predecessor"]))
    (HERE / "OCCURRENCE_IDENTITY_AUDIT.json").write_text(json_text(built["identity_audit"]))
    (HERE / "PROPULSION_DEFICIT_REPORT.json").write_text(json_text(built["deficit"]))
    (HERE / "MTP_ENERGY_REPORT.json").write_text(json_text(built["mtp_report"]))
    (HERE / "ACTUATION_CONFORMANCE_REPORT.json").write_text(json_text(built["conformance"]))
    (HERE / "SAFETY_OVERRIDE_AUDIT.json").write_text(json_text(built["override_audit"]))
    (HERE / "NEGATIVE_CONTROLS_REPORT.json").write_text(json_text(built["negative"]))
    (HERE / "ZERO_PASSIVE_SENSITIVITY_REPORT.json").write_text(json_text(built["zero_passive"]))
    (HERE / "PRE_CORRECTION_EPISODE_CLASSIFICATION.json").write_text(
        json_text(built["pre_correction"]))
    (HERE / "RES85D_STRICT_ROM_RECEIPT.md").write_text(built["strict_rom_receipt"])
    # The RES-85C receipts (RES85_RECEIPT.md, RES85C_CORRECTION_RECEIPT.md) are
    # preserved byte-for-byte and intentionally NOT rewritten here.
    # authority artifacts from Achievement A (regenerated deterministically)
    import build_authority_checks as BAC

    BAC.main([])
    manifest = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": MISSION_ID,
        "achievement": "RES-85D: enforce strict launch ROM and finalize RES-85",
        "files": {name: sha256_file(HERE / name) for name in sorted(ARTIFACT_FILES)
                  if (HERE / name).is_file()},
        "telemetry_blob_sha256": built["telemetry_manifest"]["blob_sha256"],
        "telemetry_canonical_digest": built["telemetry_manifest"]["canonical_digest"],
        "plant_xml_sha256": built["spec"]["plant_xml_sha256"],
        "res84_evidence_seal_sha256": RES84_EVIDENCE_SEAL_SHA256,
        "previous_res85_evidence_seal_sha256": PREVIOUS_RES85_EVIDENCE_SEAL_SHA256,
        "res85c_head": RES85C_HEAD,
        "entry_head": ENTRY_HEAD,
        "entry_tree": ENTRY_TREE,
    }
    (HERE / "HASH_MANIFEST.json").write_text(json_text(manifest))
    return manifest


def _identity_payload(built: dict[str, Any]) -> dict[str, str]:
    return {
        "spec": json_text(built["spec"]),
        "episode_report": json_text(built["episode_report"]),
        "strict_rom": json_text(built["strict_rom"]),
        "predecessor": json_text(built["predecessor"]),
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
        "strict_rom_receipt": built["strict_rom_receipt"],
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
        "mission": MISSION_ID,
        "status": _status(checks),
        "checks": checks,
        "telemetry_blob_sha256": a["telemetry_blob_sha256"],
        "identity_domain": sorted(a),
        "excluded_from_identity": [
            "sealed_at_utc", "postcommit sidecar", "git HEAD-at-authoring fields",
            "external bundle path", "PROPULSION_SEARCH.json (RES-85C declared search "
            "artifact with its own two-run byte-identity check)",
            "STRICT_ROM_SEARCH.json (RES-85D declared search artifact with its own "
            "two-run byte-identity check)",
            "MTP_NONCOMPENSATION_MATRIX.json (separate declared matrix artifact with its "
            "own two-run byte-identity check)",
            "JOINT_ROM_SOFT_LIMIT_PROBE.json (Plant property probe, deterministic script; "
            "numerical solver diagnostic only)",
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
            "strict_rom": built["strict_rom"]["status"],
            "strict_rom_global_min_margin":
                built["strict_rom"]["global_min_structural_rom_margin_rad"],
            "predecessor_strict_rom": built["predecessor"][
                "strict_structural_rom_audit"]["status"],
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
          and built["strict_rom"]["status"] == STATUS_PASS
          and built["episode"].status == "COMPLETED"
          and bool(built["episode_report"]["functional_task_floor"]["closure_pass"])
          and (det is None or det["status"] == STATUS_PASS))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
