"""RES-85 Achievement B — build the causal launch/flight control evidence bundle.

Authority: LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1
Mission:   RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001

Writes every deterministic scientific artifact of the RES-85 bundle:

    V3_LAUNCH_CONTROLLER_SPEC.json
    TELEMETRY_MANIFEST.json + TELEMETRY_ARRAYS.bin
    LAUNCH_EPISODE_REPORT.json
    MTP_ENERGY_REPORT.json
    ACTUATION_CONFORMANCE_REPORT.json
    NEGATIVE_CONTROLS_REPORT.json
    ZERO_PASSIVE_SENSITIVITY_REPORT.json
    DETERMINISM_REPORT.json
    HASH_MANIFEST.json

Run:  python3 build_evidence.py            # build once
      python3 build_evidence.py --twice    # build twice and compare identity
      python3 build_evidence.py --print    # status lines only
"""

from __future__ import annotations

import hashlib
import json
import sys
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
    MIRRORED_PAIRS,
    MOMENT_CEILING_NM,
    MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
    MTP_CHANNEL_INDICES,
    MTP_TOTAL_POSITIVE_WORK_BUDGET_J,
    N_CHANNELS,
    POWER_CEILING_W,
    RATE_CEILING_NM_PER_S,
    V3ActuationAuthority,
    V3MtpLedgerEntry,
)
from loaded_cmj.v3.controller import (  # noqa: E402
    PHASE_ORDER,
    SUPPORTED_PHASES,
    V3ControllerFault,
    V3LaunchController,
    V3Phase,
)
from loaded_cmj.v3.launch_runtime import (  # noqa: E402
    canonical_array_bytes,
    run_launch_episode,
    settle_standing_stance,
)

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"

RES83_PLANT_XML_SHA256 = "eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d"
RES84_EVIDENCE_SEAL_SHA256 = "223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda"
ANTI_TRIVIALITY_REFERENCE_M = 0.150

ARTIFACT_FILES = (
    "V3_LAUNCH_CONTROLLER_SPEC.json",
    "TELEMETRY_MANIFEST.json",
    "TELEMETRY_ARRAYS.bin",
    "LAUNCH_EPISODE_REPORT.json",
    "MTP_ENERGY_REPORT.json",
    "ACTUATION_CONFORMANCE_REPORT.json",
    "NEGATIVE_CONTROLS_REPORT.json",
    "ZERO_PASSIVE_SENSITIVITY_REPORT.json",
    "DETERMINISM_REPORT.json",
    "AUTHORITY_CHECK_REPORT.json",
    "SOURCE_PROVENANCE.json",
    "RES85_RECEIPT.md",
)


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


# ===========================================================================
# phase-machine probes for the synthetic negative controls
# ===========================================================================
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
# negative controls
# ===========================================================================
def negative_controls() -> dict[str, Any]:
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

    return {"status": _status(checks), "checks": checks, "probes": probes}


# ===========================================================================
# episode-derived reports
# ===========================================================================
def launch_episode_report(episode) -> dict[str, Any]:
    t = episode.telemetry
    events = episode.events
    H2 = events["apex_h2"]
    flight_time = None
    comparator = events["diagnostic_comparator"]
    if comparator and comparator.get("triggered"):
        # report-only experimental comparability metric from comparator boundaries
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
    report = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001",
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
        },
        "phase_trajectory": episode.phases_visited,
        "transition_log": [
            {"sample": int(t.index[i]), "time_s": float(t.time_s[i]),
             "from": str(t.phase_name[i]), "reason": str(t.transition_reason[i])}
            for i in range(len(t.index)) if t.transition_reason[i]
        ],
        "takeoff_occurrence": events["takeoff_occurrence"],
        "takeoff_confirmation": events["takeoff_confirmation"],
        "diagnostic_comparator": comparator,
        "apex_h2": H2,
        "impulse_cross_check": events["impulse_cross_check"],
        "res85_claim_end": events["res85_claim_end"],
        "method_explicit_h2_report": {
            "PRIMARY_CANONICAL": {
                "method_id": "DIRECT_SIMULATOR_SYSTEM_COM",
                "quantity": "COM_RISE_TAKEOFF_TO_APEX",
                "value_m": H2.get("h2_support_m"),
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
            "anti_triviality_reference": {
                "value_m": ANTI_TRIVIALITY_REFERENCE_M,
                "role": "HISTORICAL_ANTI_TRIVIALITY_NEGATIVE_CONTROL_ONLY",
                "used_as_target": False,
            },
        },
    }
    return report


def mtp_energy_report(episode) -> dict[str, Any]:
    events = episode.events
    t = episode.telemetry
    checks: list[dict[str, Any]] = []
    zero_passive = episode.events.get("zero_passive", False)
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
        "zero_passive_episode": bool(zero_passive),
    }


def actuation_conformance_report(episode) -> dict[str, Any]:
    t = episode.telemetry
    checks: list[dict[str, Any]] = []
    dt = M.NATIVE_DT_S
    moments = np.abs(t.applied_nm) if len(t.index) else np.zeros((0, N_CHANNELS))
    max_moment = moments.max(axis=0) if len(t.index) else np.zeros(N_CHANNELS)
    moment_ok = bool(np.all(max_moment <= MOMENT_CEILING_NM + 1e-9))
    _check(checks, "moment_ceiling_never_exceeded", moment_ok,
           {"max_applied_nm": [float(v) for v in max_moment],
            "ceilings": [float(v) for v in MOMENT_CEILING_NM]})
    emergency_stages = ("power_emergency", "moment_emergency")
    if len(t.index) > 1:
        rate = np.abs(np.diff(t.applied_nm, axis=0)) / dt
        max_rate = rate.max(axis=0)
        rate_violations = int(np.count_nonzero(rate > RATE_CEILING_NM_PER_S + 1e-6))
        emergency_pairs = 0
        emergency_samples = []
        for i in range(len(t.index)):
            n_emergency = sum(1 for stage in t.saturation_stage[i]
                              if str(stage) in emergency_stages)
            emergency_pairs += n_emergency
            if n_emergency:
                emergency_samples.append(int(t.index[i]))
    else:
        max_rate = np.zeros(N_CHANNELS)
        rate_violations = 0
        emergency_pairs = 0
        emergency_samples = []
    _check(checks, "torque_rate_ceiling_respected_except_declared_emergencies",
           rate_violations <= emergency_pairs,
           {"max_rate_nm_per_s": [float(v) for v in max_rate],
            "ceilings": [float(v) for v in RATE_CEILING_NM_PER_S],
            "violating_channel_samples": rate_violations,
            "declared_emergency_channel_samples": emergency_pairs,
            "emergency_samples": emergency_samples})
    power = np.abs(t.joint_power_w) if len(t.index) else np.zeros((0, N_CHANNELS))
    max_power = power.max(axis=0) if len(t.index) else np.zeros(N_CHANNELS)
    power_ok = bool(np.all(max_power <= POWER_CEILING_W + 1e-6))
    _check(checks, "joint_power_ceiling_never_exceeded", power_ok,
           {"max_power_w": [float(v) for v in max_power],
            "ceilings": [float(v) for v in POWER_CEILING_W]})
    asym = 0.0
    if len(t.index):
        for i, j in MIRRORED_PAIRS:
            asym = max(asym, float(np.abs(t.applied_nm[:, i] - t.applied_nm[:, j]).max()))
    _check(checks, "bilateral_applied_symmetry_within_tolerance", asym <= 1e-9,
           {"max_pair_asymmetry_nm": asym,
            "tolerance_nm": 1e-9})
    pre_asym = float(t.symmetry_asymmetry_nm.max()) if len(t.index) else 0.0
    handoff_indices = [int(t.index[i]) for i in range(len(t.index))
                       if t.transition_reason[i]]
    handoff_rate_ok = True
    if len(t.index) > 1:
        rate = np.abs(np.diff(t.applied_nm, axis=0)) / dt
        for k in handoff_indices:
            if 0 < k < len(t.index):
                if np.any(rate[k - 1] > RATE_CEILING_NM_PER_S + 1e-6):
                    handoff_rate_ok = False
    _check(checks, "phase_handoffs_respect_authority", handoff_rate_ok,
           {"handoff_samples": handoff_indices})
    stage_counts: dict[str, int] = {}
    if len(t.index):
        for row in t.saturation_stage:
            for s in row:
                stage_counts[str(s)] = stage_counts.get(str(s), 0) + 1
    return {
        "schema_version": "1.0.0",
        "status": _status(checks),
        "checks": checks,
        "max_applied_nm": [float(v) for v in max_moment],
        "max_rate_nm_per_s": [float(v) for v in max_rate],
        "max_joint_power_w": [float(v) for v in max_power],
        "max_bilateral_asymmetry_nm": asym,
        "max_pre_projection_asymmetry_nm": pre_asym,
        "saturation_stage_counts": stage_counts,
        "handoff_samples": handoff_indices,
        "emergency_samples": emergency_samples,
    }


def zero_passive_sensitivity_report(episode, negative: dict[str, Any]) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    t = episode.telemetry
    active_max = float(t.mtp_active_work_j[-1].max()) if len(t.index) else 0.0
    _check(checks, "zero_passive_active_work_within_budget",
           active_max <= MTP_ACTIVE_POSITIVE_WORK_BUDGET_J + 1e-9,
           {"active_positive_work_j": active_max})
    # adversarial authority probe: unbounded active MTP command on the zero-passive model
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
def build_all() -> dict[str, Any]:
    import build_authority_checks as BAC  # local sibling module
    import classify_historical_failures as CHF  # local sibling module

    authority = BAC.build_all()
    CHF.main([])
    episode = run_launch_episode()
    zero_passive_episode = run_launch_episode(zero_passive=True)
    negative = negative_controls()
    telemetry_manifest, telemetry_blob = telemetry_artifacts(episode)

    probe_plant = P.V3Plant()
    probe_data = probe_plant.make_data()
    settle_standing_stance(probe_plant, probe_data)
    controller = V3LaunchController(probe_plant, probe_data)
    spec = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001",
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
    }

    report = launch_episode_report(episode)
    mtp = mtp_energy_report(episode)
    conformance = actuation_conformance_report(episode)
    zero_passive = zero_passive_sensitivity_report(zero_passive_episode, negative)
    return {
        "authority": authority,
        "spec": spec,
        "episode": episode,
        "zero_passive_episode": zero_passive_episode,
        "episode_report": report,
        "mtp_report": mtp,
        "conformance": conformance,
        "negative": negative,
        "zero_passive": zero_passive,
        "telemetry_manifest": telemetry_manifest,
        "telemetry_blob": telemetry_blob,
        "receipt": receipt(episode, report, conformance, negative, mtp),
    }


def receipt(episode, report: dict[str, Any], conformance: dict[str, Any],
            negative: dict[str, Any], mtp: dict[str, Any]) -> str:
    h2 = report["method_explicit_h2_report"]["PRIMARY_CANONICAL"]["value_m"]
    vz = report["apex_h2"].get("takeoff_vz_m_s")
    lines = [
        "# RES85_RECEIPT — V3 causal loaded-CMJ launch and flight control",
        "",
        "MISSION: `RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001`",
        "LINEAR ISSUE: RES-85",
        "STATUS: **%s**" % (
            "PASS" if (negative["status"] == STATUS_PASS
                       and conformance["status"] == STATUS_PASS
                       and mtp["status"] == STATUS_PASS
                       and episode.status == "COMPLETED") else "FAIL"),
        "",
        "## Episode",
        "",
        "* status: `%s`%s" % (episode.status,
                               "" if episode.fault is None else f" (fault: `{episode.fault}`)"),
        "* phases: `%s`" % " -> ".join(episode.phases_visited),
        "* takeoff confirmation: `%s`" % (
            None if report["takeoff_confirmation"] is None
            else report["takeoff_confirmation"]["confirmed"]),
        "* takeoff vz: `%s` m/s" % vz,
        "* H2 (DIRECT_SIMULATOR_SYSTEM_COM): `%s` m" % h2,
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
        "`ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED`. The historical 0.150 m value is",
        "retained only as a labelled anti-triviality negative-control magnitude and is never",
        "used as an elite target, band or performance floor.",
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
    classification = json.loads((HERE / "FULL_SUITE_CLASSIFICATION.json").read_text())
    amendments = json.loads((HERE / "AUTHORITY_AMENDMENTS.json").read_text())
    lines += [
        "",
        "## Repository suite classification",
        "",
        "* collected `%d`, passed `%d`, failed `%d`, collection-error modules `%d`" % (
            classification["full_suite"]["collected"],
            classification["full_suite"]["passed"],
            classification["full_suite"]["failed"],
            classification["full_suite"]["collection_error_modules"]),
        "* RES-85-owned failures: `%d`; RES-83/RES-84 failures: `%d`" % (
            classification["full_suite"]["res85_owned_failures"],
            classification["full_suite"]["res83_res84_failures"]),
        "* every recorded failure reproduces at ENTRY_HEAD "
        "(`b0eccb8a6eeda40950665b3be517854f8b48baab`); see `FULL_SUITE_CLASSIFICATION.json`",
        "",
        "## Authority amendments during Achievement B",
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


# ===========================================================================
# writing / determinism
# ===========================================================================
def _write_artifacts(built: dict[str, Any]) -> dict[str, Any]:
    (HERE / "V3_LAUNCH_CONTROLLER_SPEC.json").write_text(json_text(built["spec"]))
    (HERE / "TELEMETRY_MANIFEST.json").write_text(json_text(built["telemetry_manifest"]))
    (HERE / "TELEMETRY_ARRAYS.bin").write_bytes(built["telemetry_blob"])
    (HERE / "LAUNCH_EPISODE_REPORT.json").write_text(json_text(built["episode_report"]))
    (HERE / "MTP_ENERGY_REPORT.json").write_text(json_text(built["mtp_report"]))
    (HERE / "ACTUATION_CONFORMANCE_REPORT.json").write_text(json_text(built["conformance"]))
    (HERE / "NEGATIVE_CONTROLS_REPORT.json").write_text(json_text(built["negative"]))
    (HERE / "ZERO_PASSIVE_SENSITIVITY_REPORT.json").write_text(json_text(built["zero_passive"]))
    (HERE / "RES85_RECEIPT.md").write_text(built["receipt"])
    # authority artifacts from Achievement A (regenerated deterministically)
    import build_authority_checks as BAC

    BAC.main([])
    manifest = {
        "schema_version": "1.0.0",
        "authority_id": "LCMJ_RES85_CAUSAL_LAUNCH_FLIGHT_CONTROL_V1",
        "mission": "RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001",
        "achievement": "B: implement and qualify causal launch/flight control",
        "files": {name: sha256_file(HERE / name) for name in sorted(ARTIFACT_FILES)
                  if (HERE / name).is_file()},
        "telemetry_blob_sha256": built["telemetry_manifest"]["blob_sha256"],
        "telemetry_canonical_digest": built["telemetry_manifest"]["canonical_digest"],
        "plant_xml_sha256": built["spec"]["plant_xml_sha256"],
        "res84_evidence_seal_sha256": RES84_EVIDENCE_SEAL_SHA256,
    }
    (HERE / "HASH_MANIFEST.json").write_text(json_text(manifest))
    return manifest


def _identity_payload(built: dict[str, Any]) -> dict[str, str]:
    return {
        "spec": json_text(built["spec"]),
        "episode_report": json_text(built["episode_report"]),
        "mtp_report": json_text(built["mtp_report"]),
        "conformance": json_text(built["conformance"]),
        "negative": json_text(built["negative"]),
        "zero_passive": json_text(built["zero_passive"]),
        "telemetry_manifest": json_text(built["telemetry_manifest"]),
        "telemetry_blob_sha256": sha256_bytes(built["telemetry_blob"]),
        "receipt": built["receipt"],
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
        "identity_domain": sorted(a),
        "excluded_from_identity": [
            "sealed_at_utc", "postcommit sidecar", "git HEAD-at-authoring fields",
            "external bundle path",
        ],
        "artifact_sha256": {k: sha256_bytes(v.encode("utf-8")) for k, v in a.items()},
    }


def main(argv: list[str] | None = None) -> int:
    argv = argv or []
    built = build_all()
    if "--twice" in argv:
        second = build_all()
        det = determinism_report(built, second)
        (HERE / "DETERMINISM_REPORT.json").write_text(json_text(det))
    else:
        det = None
    manifest = _write_artifacts(built)
    if "--print" in argv or det is None:
        print(json.dumps({
            "episode_status": built["episode"].status,
            "phases": built["episode"].phases_visited,
            "negative": built["negative"]["status"],
            "conformance": built["conformance"]["status"],
            "mtp": built["mtp_report"]["status"],
            "zero_passive": built["zero_passive"]["status"],
            "determinism": None if det is None else det["status"],
            "hash_manifest_files": len(manifest["files"]),
        }, indent=2))
    ok = (built["negative"]["status"] == STATUS_PASS
          and built["conformance"]["status"] == STATUS_PASS
          and built["mtp_report"]["status"] == STATUS_PASS
          and built["zero_passive"]["status"] == STATUS_PASS
          and built["episode"].status == "COMPLETED"
          and (det is None or det["status"] == STATUS_PASS))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
