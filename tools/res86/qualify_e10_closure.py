#!/usr/bin/env python3
"""RES-86 production E10 closure attempt and qualification.

MISSION: RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001
LINEAR ISSUE: RES-86

Runs the production post-apex landing runtime (qualified plantar contact
realization, post-apex LANDING_PREP, active MTP within the frozen budget,
legal contact progression, two-stage absorption, exact branch validation) with
the declared best-effort E10 preparation configuration, evaluates the recorded
trace with the independent event authority and records the exact controlling
outcome.  The same episode is executed twice and the telemetry arrays are
compared bit-exactly, so the result is a deterministic reproduction rather
than a single sample.

This tool never relaxes a frozen gate: it records what the frozen Plant,
contact realization, authority and gates actually produce.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT / "src"), str(ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3.landing_control import (  # noqa: E402
    E10_POINT_LIMITS,
    E10_WINDOW_LIMITS,
    V3LandingConfig,
)
from loaded_cmj.v3.landing_runtime import run_landing_episode  # noqa: E402

# Declared E10 preparation candidates.  Both are parameter choices of the one
# evolving controller (never a Plant, contact, authority or gate change):
#
# * ``best_effort_touchdown``: full preparation posture gains, a moderate
#   flexion offset and a flat-ankle target.  The declared preparation search
#   found this to be the slowest, flattest touchdown that keeps the
#   first-contact-to-E10 window envelopes within reach (peak |Hy| 4.91 vs the
#   5.0 envelope, contact at native sample 797 with com vz -1.847 m/s);
# * ``forward_reach_sensitivity``: the same preparation with the declared
#   independent knee reach (-0.6 rad) that places the foot closest to under the
#   SYSTEM_COM (contact x -0.049 m vs the 0.069 m COM at native sample 777,
#   com vz -1.455 m/s).  It is recorded as a design sensitivity: the foot
#   placement improves while the impact angular impulse does not.
PREPARATION_CANDIDATES: dict[str, dict[str, float]] = {
    "best_effort_touchdown": {
        "prep_position_gain_scale": 1.0,
        "prep_flexion_target_rad": 0.4,
        "prep_knee_offset_rad": 0.0,
        "prep_ankle_offset_rad": -0.7,
        "prep_ankle_rate_rad_s": 8.0,
        "prep_posture_rate_rad_s": 4.0,
    },
    "forward_reach_sensitivity": {
        "prep_position_gain_scale": 1.0,
        "prep_flexion_target_rad": 0.4,
        "prep_knee_offset_rad": -0.6,
        "prep_ankle_offset_rad": -0.7,
        "prep_ankle_rate_rad_s": 8.0,
        "prep_posture_rate_rad_s": 4.0,
    },
}
HORIZON_S = 2.05


def _telemetry_digest(episode) -> str:
    """The sealed telemetry identity (object arrays digested by value)."""
    return episode.telemetry.canonical_digest()


def _post_contact_window(episode) -> dict | None:
    """Post-first-contact extrema from the recorded trace (diagnostic).

    When E10 is not reached the frozen window gate is evaluated over the whole
    executed post-contact trace; these are the exact trace extrema for that
    scope, recorded so the controlling envelope is reported even on failure.
    """
    trace = episode.trace
    e8 = int(trace.first_contact_position)
    if e8 < 0 or trace.length == 0:
        return None
    window = slice(e8, trace.length)
    return {
        "scope_native_samples": [int(trace.index[e8]), int(trace.index[-1])],
        "max_abs_com_vx_m_s": float(np.max(np.abs(trace.com_velocity_world_m_s[window, 0]))),
        "max_abs_hy_kg_m2_s": float(np.max(np.abs(trace.hy_kg_m2_s[window]))),
        "max_abs_root_pitch_rad": float(np.max(np.abs(trace.root_pitch_rad[window]))),
        "max_abs_trunk_pitch_rad": float(np.max(np.abs(trace.trunk_pitch_rad[window]))),
        "max_abs_root_pitch_rate_rad_s": float(
            np.max(np.abs(trace.root_pitch_rate_rad_s[window]))),
        "max_abs_trunk_pitch_rate_rad_s": float(
            np.max(np.abs(trace.trunk_pitch_rate_rad_s[window]))),
        "max_penetration_m": float(np.max(trace.max_penetration_m[window])),
        "peak_total_floor_fz_bw": float(np.max(trace.total_floor_fz_n[window]))
        / float(C.V3_SYSTEM_WEIGHT_N),
        "note": ("diagnostic post-first-contact extrema over the executed trace; "
                 "the frozen first-contact-to-E10 window is this scope exactly "
                 "when E10 is never confirmed"),
    }


def _report(episode) -> dict:
    events = episode.events
    hard = events.get("hard_gate_audit") or {}
    window = events.get("first_contact_to_e10_window") or {}
    point = ((events.get("e10") or {}).get("point") or {})
    return {
        "status": episode.status,
        "fault": episode.fault,
        "warnings": list(episode.warnings),
        "handoff_kind": episode.diagnostics.get("handoff_kind"),
        "handoff_sample": episode.diagnostics.get("handoff_sample"),
        "handoff_state_sha256": episode.diagnostics.get("handoff_state_sha256"),
        "contact_realization": episode.diagnostics.get("contact_realization"),
        "contact_realization_expected_realized_solref": episode.diagnostics.get(
            "contact_realization_expected_realized_solref"),
        "contact_realization_verification_status": (
            episode.diagnostics.get("contact_realization_verification") or {}).get("status"),
        "contact_realization_final_rows": episode.diagnostics.get(
            "contact_realization_final_rows"),
        "landing_samples": episode.diagnostics.get("landing_samples"),
        "fallback_count": episode.diagnostics.get("fallback_count"),
        "live_branch_identity_failures": episode.diagnostics.get(
            "live_branch_identity_failures"),
        "e10_confirmed_time_s": episode.diagnostics.get("e10_confirmed_time_s"),
        "e10_window_violated_time_s": episode.diagnostics.get("e10_window_violated_time_s"),
        "e10_window_violation_reason": episode.diagnostics.get("e10_window_violation_reason"),
        "e8_first_contact": events.get("e8_first_contact"),
        "e9_bilateral_established": events.get("e9_bilateral_established"),
        "d_est": events.get("d_est"),
        "e10": {k: v for k, v in (events.get("e10") or {}).items() if k != "point"},
        "e10_point_values": {k: point.get(k) for k in (
            "time_s", "com_vx_m_s", "com_vz_m_s", "hy_kg_m2_s", "root_pitch_rad",
            "trunk_pitch_rad", "root_pitch_rate_rad_s", "trunk_pitch_rate_rad_s")},
        "e10_point_gates": point.get("gates"),
        "window_values": {k: v for k, v in window.items() if k != "gates"},
        "window_gates": window.get("gates"),
        "post_contact_window_extrema": _post_contact_window(episode),
        "hard_gate_values": {k: hard.get(k) for k in (
            "max_penetration_m", "peak_total_floor_fz_bw", "chatter_transitions",
            "material_reflight_intervals", "min_rom_margin_rad",
            "max_abs_moment_ratio", "max_abs_power_ratio",
            "prohibited_detected_samples", "prohibited_active_samples")},
        "hard_gate_gates": hard.get("gates"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--horizon-s", type=float, default=HORIZON_S)
    args = parser.parse_args()

    t0 = time.time()
    candidates: dict[str, dict] = {}
    for label, overrides in PREPARATION_CANDIDATES.items():
        config = V3LandingConfig(**overrides)
        first = run_landing_episode(horizon_s=args.horizon_s, landing_config=config)
        second = run_landing_episode(horizon_s=args.horizon_s, landing_config=config)
        first_digest = _telemetry_digest(first)
        second_digest = _telemetry_digest(second)
        candidates[label] = {
            "preparation_config": overrides,
            "episode_1": _report(first),
            "episode_2": _report(second),
            "deterministic_reproduction": {
                "episode_1_telemetry_sha256": first_digest,
                "episode_2_telemetry_sha256": second_digest,
                "bit_identical": bool(first_digest == second_digest),
            },
            "e10_pass": bool(first.events.get("status") == "PASS"
                             and (first.events.get("e10") or {}).get("reached") is True),
        }
    report = {
        "authority_id": "LCMJ_RES86_E10_CLOSURE_QUALIFICATION_V1",
        "mission": "RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001",
        "linear_issue": "RES-86",
        "contact_realization": "candidate",
        "preparation_candidates": candidates,
        "e10_point_limits": E10_POINT_LIMITS,
        "e10_window_limits": E10_WINDOW_LIMITS,
        "horizon_s": float(args.horizon_s),
        "e10_pass": bool(any(entry["e10_pass"] for entry in candidates.values())),
        "wall_s": time.time() - t0,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, default=str) + "\n")
    for label, entry in candidates.items():
        episode = entry["episode_1"]
        print(json.dumps({
            "candidate": label,
            "status": episode["status"],
            "fault": episode["fault"],
            "e8_time_s": (episode["e8_first_contact"] or {}).get("time_s"),
            "e8_com_vz": (episode["e8_first_contact"] or {}).get("com_vz_m_s"),
            "window_violation": episode["e10_window_violation_reason"],
            "window_violation_t": episode["e10_window_violated_time_s"],
            "window_max_abs_hy": (episode["window_values"] or {}).get("max_abs_hy_kg_m2_s"),
            "hard_max_penetration_m": episode["hard_gate_values"].get("max_penetration_m"),
            "e10_confirmed": episode["e10_confirmed_time_s"],
            "deterministic": entry["deterministic_reproduction"]["bit_identical"],
        }, default=str), flush=True)
    print(json.dumps({"e10_pass": report["e10_pass"], "wall_s": report["wall_s"]}, default=str))


if __name__ == "__main__":
    main()
