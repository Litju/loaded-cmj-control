"""Independent RES-86 E10 landing event and hard-gate evaluation.

Authority: ``LCMJ_RES86_V3_LANDING_ACCEPTANCE_AUTHORITY_V1``.

This module evaluates a recorded landing trace **independently of any
controller private memory** and returns the physical event sequence plus the
frozen RES-86 gates:

``E8_FIRST_CONTACT`` -> ``E9_BILATERAL_ESTABLISHED`` -> sustained impact
absorption -> ``E10_CONFIRMATION``.

Correct RES-86 dwell semantics:

* ``D_EST`` is a latency bound, not a sustain dwell:
  ``t_bilateral_established - t_first_contact <= 0.050 s``.  There is no
  required true-sample count for ``D_EST``.
* ``D_BL`` is the actual bilateral-loaded sustain dwell: the predicate remains
  continuously true for ``>= 0.050 s`` (25 elapsed intervals / 26 inclusive
  true samples at nominal dt), with ``abs(SYSTEM_COM_vz) < 0.05 m/s``
  throughout the E10 sustain run.

The predicate force threshold ``F_thr = 10 N`` is the bilateral-loaded task
predicate only; it never defines physical contact truth.  The evaluator
consumes derived measurement rows (legal support counts, per-foot legal
normal force, SYSTEM_COM, orientation, centroidal Hy, contact rows); it never
re-implements contact semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from loaded_cmj.v3 import constants as C
from loaded_cmj.v3.landing_authority import (
    D_BL_S,
    D_EST_S,
    V3_STRUCTURAL_ROM_TOLERANCE_RAD,
    authority_sha256,
    dwell_confirmed,
    establishment_latency_satisfied,
    landing_acceptance_authority,
)
from loaded_cmj.v3.landing_metrics import chatter_transition_count, material_reflight_intervals

V3_LANDING_EVENT_AUTHORITY_ID = "LCMJ_RES86_LANDING_EVENT_EVALUATION_V1"

F_THR_N = 10.0
V_ABS_TAIL_M_S = 0.05
BW_N = float(C.V3_SYSTEM_WEIGHT_N)


@dataclass(frozen=True)
class V3LandingTrace:
    """Raw native landing trace consumed by the independent event authority."""

    index: np.ndarray
    time_s: np.ndarray
    legal_plantar_active: np.ndarray
    left_legal_plantar_active: np.ndarray
    right_legal_plantar_active: np.ndarray
    left_fz_n: np.ndarray
    right_fz_n: np.ndarray
    prohibited_detected: np.ndarray
    prohibited_active: np.ndarray
    com_world_m: np.ndarray
    com_velocity_world_m_s: np.ndarray
    hy_kg_m2_s: np.ndarray
    root_pitch_rad: np.ndarray
    trunk_pitch_rad: np.ndarray
    root_pitch_rate_rad_s: np.ndarray
    trunk_pitch_rate_rad_s: np.ndarray
    total_floor_fz_n: np.ndarray
    max_penetration_m: np.ndarray
    rom_margin_min_rad: np.ndarray
    max_abs_moment_ratio: np.ndarray
    max_abs_power_ratio: np.ndarray
    first_contact_position: int

    @property
    def length(self) -> int:
        return int(self.index.shape[0])


def _bilateral_loaded(trace: V3LandingTrace, position: int) -> bool:
    return bool(trace.left_legal_plantar_active[position] > 0
                and trace.right_legal_plantar_active[position] > 0
                and trace.left_fz_n[position] > F_THR_N
                and trace.right_fz_n[position] > F_THR_N)


def bilateral_establishment_position(trace: V3LandingTrace) -> int | None:
    for position in range(int(trace.first_contact_position), trace.length):
        if _bilateral_loaded(trace, position):
            return position
    return None


def e10_confirmation_position(trace: V3LandingTrace) -> tuple[int | None, int | None]:
    """(run onset position, confirmation position) under the corrected D_BL."""
    establishment = bilateral_establishment_position(trace)
    if establishment is None:
        return None, None
    run_start: int | None = None
    for position in range(establishment, trace.length):
        predicate = (_bilateral_loaded(trace, position)
                     and abs(float(trace.com_velocity_world_m_s[position, 2])) < V_ABS_TAIL_M_S)
        if not predicate:
            run_start = None
            continue
        if run_start is None:
            run_start = position
        if dwell_confirmed(float(trace.time_s[run_start]), float(trace.time_s[position]), D_BL_S):
            return run_start, position
    return None, None


def _slice_max(values: np.ndarray, start: int, stop_inclusive: int) -> float:
    if stop_inclusive < start:
        return float("nan")
    return float(np.max(values[start:stop_inclusive + 1]))


def evaluate_landing_trace(trace: V3LandingTrace) -> dict:
    """Independent E10 event + frozen hard-gate evaluation of a landing trace."""
    authority = landing_acceptance_authority()
    physical = authority["physical_gates"]
    window = authority["first_contact_to_E10_window"]
    point = authority["E10_point_in_time"]
    n = trace.length
    e8 = int(trace.first_contact_position)
    if e8 < 0 or e8 >= n:
        raise ValueError("first_contact_position outside the trace")

    establishment = bilateral_establishment_position(trace)
    run_onset, e10 = e10_confirmation_position(trace)
    first_contact_time = float(trace.time_s[e8])
    e8_sample = int(trace.index[e8])

    report: dict[str, object] = {
        "authority_id": V3_LANDING_EVENT_AUTHORITY_ID,
        "landing_acceptance_authority_sha256": authority_sha256(authority),
        "trace_length": n,
        "f_thr_n": F_THR_N,
        "v_abs_tail_m_s": V_ABS_TAIL_M_S,
        "e8_first_contact": {
            "position": e8,
            "sample": e8_sample,
            "time_s": first_contact_time,
            "legal_plantar_active": int(trace.legal_plantar_active[e8]),
            "prohibited_detected": int(trace.prohibited_detected[e8]),
            "com_vz_m_s": float(trace.com_velocity_world_m_s[e8, 2]),
        },
        "physics_gate_scope": "[E8_FIRST_CONTACT, E10_CONFIRMATION]",
        "branch_scope": "[E8_FIRST_CONTACT, end of executed trace]",
    }

    if establishment is None:
        report.update({
            "e9_bilateral_established": None,
            "d_est": {"satisfied": False, "reason": "NO_BILATERAL_ESTABLISHMENT"},
            "e10": {"reached": False, "reason": "NO_BILATERAL_ESTABLISHMENT"},
            "status": "FAIL",
        })
        return report

    establishment_time = float(trace.time_s[establishment])
    d_est_s = establishment_time - first_contact_time
    d_est_satisfied = bool(establishment_latency_satisfied(first_contact_time,
                                                           establishment_time, D_EST_S))
    report["e9_bilateral_established"] = {
        "position": establishment,
        "sample": int(trace.index[establishment]),
        "time_s": establishment_time,
        "left_fz_n": float(trace.left_fz_n[establishment]),
        "right_fz_n": float(trace.right_fz_n[establishment]),
    }
    report["d_est"] = {
        "value_s": float(d_est_s),
        "limit_s": D_EST_S,
        "satisfied": d_est_satisfied,
        "semantics": ("t_bilateral_established - t_first_contact <= D_EST_S "
                      "(latency bound, not a sustain dwell)"),
        "exact_nanoseconds": True,
        "diagnostic_intervals": int(round(d_est_s / 0.002)) if d_est_s >= 0 else None,
    }

    if e10 is None:
        report.update({
            "e10": {
                "reached": False,
                "reason": "NO_SUSTAINED_BILATERAL_LOADED_PREDICATE_WITH_ABS_COM_VZ_BELOW_TAIL",
                "d_bl_s": D_BL_S,
            },
            "status": "FAIL",
        })
        report["hard_gate_audit"] = _hard_gate_audit(trace, e8, n - 1, physical)
        return report

    e10_time = float(trace.time_s[e10])
    d_bl_s = e10_time - float(trace.time_s[run_onset])
    d_bl_satisfied = bool(dwell_confirmed(float(trace.time_s[run_onset]), e10_time, D_BL_S))
    gate_stop = e10
    window_slice = slice(e8, gate_stop + 1)

    point_report = {
        "position": e10,
        "sample": int(trace.index[e10]),
        "time_s": e10_time,
        "d_bl_from_run_onset_s": float(d_bl_s),
        "d_bl_satisfied": bool(d_bl_satisfied),
        "run_onset_position": run_onset,
        "run_onset_sample": int(trace.index[run_onset]),
        "run_onset_time_s": float(trace.time_s[run_onset]),
        "com_vx_m_s": float(trace.com_velocity_world_m_s[e10, 0]),
        "com_vz_m_s": float(trace.com_velocity_world_m_s[e10, 2]),
        "hy_kg_m2_s": float(trace.hy_kg_m2_s[e10]),
        "root_pitch_rad": float(trace.root_pitch_rad[e10]),
        "trunk_pitch_rad": float(trace.trunk_pitch_rad[e10]),
        "root_pitch_rate_rad_s": float(trace.root_pitch_rate_rad_s[e10]),
        "trunk_pitch_rate_rad_s": float(trace.trunk_pitch_rate_rad_s[e10]),
        "gates": {
            "max_abs_com_vx": abs(float(trace.com_velocity_world_m_s[e10, 0]))
            <= point["max_abs_com_vx_m_s"],
            "max_abs_Hy": abs(float(trace.hy_kg_m2_s[e10])) <= point["max_abs_Hy_kg_m2_s"],
            "max_abs_root_pitch": abs(float(trace.root_pitch_rad[e10]))
            <= point["max_abs_root_pitch_rad"],
            "max_abs_trunk_pitch": abs(float(trace.trunk_pitch_rad[e10]))
            <= point["max_abs_trunk_pitch_rad"],
            "max_abs_root_pitch_rate": abs(float(trace.root_pitch_rate_rad_s[e10]))
            <= point["max_abs_root_pitch_rate_rad_s"],
            "max_abs_trunk_pitch_rate": abs(float(trace.trunk_pitch_rate_rad_s[e10]))
            <= point["max_abs_trunk_pitch_rate_rad_s"],
        },
    }
    point_report["all_pass"] = all(point_report["gates"].values())
    report["e10"] = {
        "reached": True,
        "position": e10,
        "sample": int(trace.index[e10]),
        "time_s": e10_time,
        "d_bl_s": float(d_bl_s),
        "d_bl_satisfied": bool(d_bl_satisfied),
        "run_onset_position": run_onset,
        "run_onset_sample": int(trace.index[run_onset]),
        "run_onset_time_s": float(trace.time_s[run_onset]),
        "point": point_report,
    }

    window_report = {
        "scope": [e8_sample, int(trace.index[e10])],
        "max_abs_com_vx_m_s": _slice_max(np.abs(trace.com_velocity_world_m_s[:, 0]),
                                          e8, gate_stop),
        "max_abs_hy_kg_m2_s": _slice_max(np.abs(trace.hy_kg_m2_s), e8, gate_stop),
        "max_abs_root_pitch_rate_rad_s": _slice_max(np.abs(trace.root_pitch_rate_rad_s),
                                                     e8, gate_stop),
        "max_abs_trunk_pitch_rate_rad_s": _slice_max(np.abs(trace.trunk_pitch_rate_rad_s),
                                                      e8, gate_stop),
        "max_root_pitch_rad": _slice_max(trace.root_pitch_rad, e8, gate_stop),
        "min_root_pitch_rad": float(np.min(trace.root_pitch_rad[e8:gate_stop + 1])),
        "max_trunk_pitch_rad": float(np.max(trace.trunk_pitch_rad[e8:gate_stop + 1])),
        "min_trunk_pitch_rad": float(np.min(trace.trunk_pitch_rad[e8:gate_stop + 1])),
    }
    window_report["gates"] = {
        "max_abs_com_vx": window_report["max_abs_com_vx_m_s"] <= window["max_abs_com_vx_m_s"],
        "max_abs_Hy": window_report["max_abs_hy_kg_m2_s"] <= window["max_abs_Hy_kg_m2_s"],
        "max_abs_root_pitch_rate":
            window_report["max_abs_root_pitch_rate_rad_s"]
            <= window["max_abs_root_pitch_rate_rad_s"],
        "max_abs_trunk_pitch_rate":
            window_report["max_abs_trunk_pitch_rate_rad_s"]
            <= window["max_abs_trunk_pitch_rate_rad_s"],
        "root_pitch_envelope": (window_report["max_root_pitch_rad"]
                                <= window["root_pitch_envelope_rad"]
                                and window_report["min_root_pitch_rad"]
                                >= -window["root_pitch_envelope_rad"]),
        "trunk_pitch_envelope": (window_report["max_trunk_pitch_rad"]
                                 <= window["trunk_pitch_envelope_rad"]
                                 and window_report["min_trunk_pitch_rad"]
                                 >= -window["trunk_pitch_envelope_rad"]),
    }
    window_report["all_pass"] = all(window_report["gates"].values())
    report["first_contact_to_e10_window"] = window_report

    hard = _hard_gate_audit(trace, e8, n - 1, physical)
    # Penetration and peak Fz are scoped [E8, E10] by the authority; also report
    # the whole executed branch for transparency.
    hard["landing_scope"] = {
        "scope": [e8_sample, int(trace.index[e10])],
        "max_penetration_m": _slice_max(trace.max_penetration_m, e8, gate_stop),
        "peak_total_floor_fz_n": _slice_max(trace.total_floor_fz_n, e8, gate_stop),
        "peak_total_floor_fz_bw": _slice_max(trace.total_floor_fz_n, e8, gate_stop) / BW_N,
    }
    hard["gates"]["max_penetration_within_limit"] = bool(
        hard["landing_scope"]["max_penetration_m"] <= physical["max_penetration_m"])
    hard["gates"]["peak_total_floor_fz_within_limit"] = bool(
        hard["landing_scope"]["peak_total_floor_fz_bw"] <= physical["peak_total_floor_fz_bw"])
    hard["all_pass"] = bool(all(hard["gates"].values()))
    report["hard_gate_audit"] = hard
    report["status"] = "PASS" if (
        d_est_satisfied and point_report["all_pass"] and window_report["all_pass"]
        and hard["all_pass"]) else "FAIL"
    report["run_onset_to_confirmation_elapsed_s"] = float(d_bl_s)
    report["window_max_abs_com_vx"] = window_report["max_abs_com_vx_m_s"]
    report["window_max_abs_hy"] = window_report["max_abs_hy_kg_m2_s"]
    report["window_max_abs_root_rate"] = window_report["max_abs_root_pitch_rate_rad_s"]
    report["window_max_abs_trunk_rate"] = window_report["max_abs_trunk_pitch_rate_rad_s"]
    report["window_root_pitch_envelope_rad"] = max(window_report["max_root_pitch_rad"],
                                                   -window_report["min_root_pitch_rad"])
    report["window_trunk_pitch_envelope_rad"] = max(window_report["max_trunk_pitch_rad"],
                                                    -window_report["min_trunk_pitch_rad"])
    report["first_contact_to_e10_max_abs_com_vx"] = window_report["max_abs_com_vx_m_s"]
    report["first_contact_to_e10_max_abs_hy"] = window_report["max_abs_hy_kg_m2_s"]
    report["first_contact_to_e10_max_abs_root_rate"] = window_report["max_abs_root_pitch_rate_rad_s"]
    report["first_contact_to_e10_max_abs_trunk_rate"] = window_report["max_abs_trunk_pitch_rate_rad_s"]
    return report


def _hard_gate_audit(trace: V3LandingTrace, start: int, stop: int, physical: dict) -> dict:
    support = np.asarray(trace.legal_plantar_active[start:stop + 1], dtype=bool)
    times = np.asarray(trace.time_s, dtype=np.float64)
    reflights = material_reflight_intervals(support, 0, sample_times_s=times[start:stop + 1],
                                            min_duration_s=D_BL_S)
    chatter = chatter_transition_count(support, 0)
    prohibited_detected = int(np.count_nonzero(trace.prohibited_detected[start:stop + 1] > 0))
    prohibited_active = int(np.count_nonzero(trace.prohibited_active[start:stop + 1] > 0))
    max_penetration = float(np.max(trace.max_penetration_m[start:stop + 1]))
    peak_fz = float(np.max(trace.total_floor_fz_n[start:stop + 1]))
    max_moment_ratio = float(np.max(trace.max_abs_moment_ratio[start:stop + 1]))
    max_power_ratio = float(np.max(trace.max_abs_power_ratio[start:stop + 1]))
    min_rom_margin = float(np.min(trace.rom_margin_min_rad[start:stop + 1]))
    return {
        "scope": [int(trace.index[start]), int(trace.index[stop])],
        "prohibited_detected_samples": prohibited_detected,
        "prohibited_active_samples": prohibited_active,
        "max_penetration_m": max_penetration,
        "peak_total_floor_fz_n": peak_fz,
        "peak_total_floor_fz_bw": peak_fz / BW_N,
        "chatter_transitions": int(chatter),
        "material_reflight_intervals": [list(pair) for pair in reflights],
        "min_rom_margin_rad": min_rom_margin,
        "max_abs_moment_ratio": max_moment_ratio,
        "max_abs_power_ratio": max_power_ratio,
        "gates": {
            "no_prohibited_or_fall_contact": bool(prohibited_detected == 0
                                                  and prohibited_active == 0),
            "max_penetration_within_limit": bool(max_penetration <= physical["max_penetration_m"]),
            "peak_total_floor_fz_within_limit": bool(
                peak_fz / BW_N <= physical["peak_total_floor_fz_bw"]),
            "chatter_within_limit": bool(chatter <= physical["chatter_transitions_max"]),
            "no_material_reflight": bool(len(reflights) == 0),
            "strict_structural_rom": bool(
                min_rom_margin >= -V3_STRUCTURAL_ROM_TOLERANCE_RAD),
            "hard_actuation_authority": bool(max_moment_ratio <= 1.0 + 1.0e-9
                                             and max_power_ratio <= 1.0 + 1.0e-9),
        },
    }


__all__ = [
    "F_THR_N",
    "V3LandingTrace",
    "V3_LANDING_EVENT_AUTHORITY_ID",
    "bilateral_establishment_position",
    "e10_confirmation_position",
    "evaluate_landing_trace",
]
