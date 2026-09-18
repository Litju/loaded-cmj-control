"""Frozen V3 landing acceptance authority (RES-86A).

Authority: ``LCMJ_RES86_V3_LANDING_ACCEPTANCE_AUTHORITY_V1``.

This module is the single source of truth for the RES-86 landing acceptance
numbers implementing the downstream RES-82 decision record
(``audit/EXP-RES82-SUCCESSOR-SCIENTIFIC-TASK-CONTRACT-001``).  The numbers are
classified ``OWNER_ENGINEERING_TASK_BOUND_WITH_SENSITIVITY``: they are task
bounds for the deterministic V3 loaded-CMJ landing and must never be presented
as elite-soccer normative physiology.

Scope boundaries frozen with the authority:

* L4-T8 (the behavioral window ``[E10_confirmation, E11_onset]``) is **not**
  closed here: E11 bounds are OD-05 / RES-87 work and remain a RES-86 -> RES-87
  handoff condition.  E11 and E12 are likewise deferred.
* No landing-controller optimisation, no parameter search and no RES-87
  recovery work is authorized by this artifact.
* ``Fz`` is the total floor vertical force (including prohibited/fall contact
  load); the declared body weight is the V3 SYSTEM weight (athlete + 20 kg
  load), never athlete-only weight.  The 8 BW peak-force bound is a
  fail-safe/task envelope, not a human normative target.
"""

from __future__ import annotations

import hashlib
import json
from fractions import Fraction

from loaded_cmj.v3 import constants as C

V3_LANDING_AUTHORITY_ID = "LCMJ_RES86_V3_LANDING_ACCEPTANCE_AUTHORITY_V1"
V3_LANDING_AUTHORITY_VERSION = "1.0.0"
V3_LANDING_AUTHORITY_CLASS = "OWNER_ENGINEERING_TASK_BOUND_WITH_SENSITIVITY"

# RES-82 PHYSICAL_TIME dwell semantics (controlling convention).
V3_DWELL_SEMANTICS = "PHYSICAL_TIME"
V3_DWELL_DT_NOMINAL_S = 0.002
D_EST_S = 0.050
D_BL_S = 0.050
V3_DWELL_NANOSECONDS_PER_SECOND = 1_000_000_000


def dwell_requirements(duration_s: float, dt_s: float = V3_DWELL_DT_NOMINAL_S) -> dict:
    """Exact physical-time dwell requirement on a native grid.

    ``REQUIRED_INTERVALS = K_D = ceil(D / dt)`` and
    ``REQUIRED_TRUE_SAMPLES_INCLUSIVE = K_D + 1`` (the onset sample plus one
    sample per interval).  The sample count is diagnostic only; the runtime
    authority is ``t_current - t_onset >= D``.  Values are computed with exact
    integer nanoseconds, never with floating ``ceil``.
    """
    duration_ns = Fraction(float(duration_s)).limit_denominator(V3_DWELL_NANOSECONDS_PER_SECOND)
    dt_ns = Fraction(float(dt_s)).limit_denominator(V3_DWELL_NANOSECONDS_PER_SECOND)
    duration_scaled = duration_ns * V3_DWELL_NANOSECONDS_PER_SECOND
    dt_scaled = dt_ns * V3_DWELL_NANOSECONDS_PER_SECOND
    if duration_scaled.denominator != 1 or dt_scaled.denominator != 1:
        raise ValueError("dwell duration/dt not exact at integer-nanosecond resolution")
    duration_ns_int = int(duration_scaled)
    dt_ns_int = int(dt_scaled)
    if dt_ns_int <= 0 or duration_ns_int <= 0:
        raise ValueError("dwell duration/dt must be positive")
    required_intervals = -(-duration_ns_int // dt_ns_int)
    return {
        "DURATION_S": float(duration_s),
        "DT_NOMINAL_S": float(dt_s),
        "REQUIRED_INTERVALS": required_intervals,
        "REQUIRED_TRUE_SAMPLES_INCLUSIVE": required_intervals + 1,
        "RUNTIME_AUTHORITY": "t_current - t_onset >= DURATION_S",
        "SAMPLE_COUNT_ROLE": "DIAGNOSTIC_ONLY",
        "DWELL_TYPE": V3_DWELL_SEMANTICS,
    }


def dwell_confirmed(onset_time_s: float, current_time_s: float, duration_s: float) -> bool:
    """Runtime physical-time authority: ``t_current - t_onset >= D``."""
    return float(current_time_s) - float(onset_time_s) >= float(duration_s)


D_EST_REQUIREMENTS = dwell_requirements(D_EST_S)
D_BL_REQUIREMENTS = dwell_requirements(D_BL_S)

# RES-85D sealed strict structural-ROM comparison semantics.
V3_STRUCTURAL_ROM_TOLERANCE_RAD = 1e-9
V3_STRUCTURAL_ROM_TOLERANCE_ROLE = "FLOATING_POINT_EQUALITY_ONLY_NOT_ANATOMICAL_ROM"


def structural_rom_violation(q_rad: float, lower_rad: float, upper_rad: float, *,
                             tolerance_rad: float = V3_STRUCTURAL_ROM_TOLERANCE_RAD) -> bool:
    """Inherited RES-85D strict structural-ROM comparison.

    A coordinate is a violation only when it lies outside
    ``[lower - tolerance, upper + tolerance]``.  Equality with a bound passes,
    a value inside the 1e-9 floating-point comparison tolerance passes, and a
    material overshoot beyond the tolerance fails.  The tolerance is never
    added anatomical ROM.
    """
    return bool(float(q_rad) < float(lower_rad) - float(tolerance_rad)
                or float(q_rad) > float(upper_rad) + float(tolerance_rad))

RES82_BUNDLE = "audit/EXP-RES82-SUCCESSOR-SCIENTIFIC-TASK-CONTRACT-001"
RES82_LANDING_CONTRACT = f"{RES82_BUNDLE}/LANDING_BALANCE_RECOVERY_CONTRACT.json"
RES82_EVENT_CONTRACT = f"{RES82_BUNDLE}/EVENT_CONTRACT_E1_E12.json"
RES82_OWNER_DECISIONS = f"{RES82_BUNDLE}/OWNER_DECISIONS_REQUIRED.json"

BW_N = float(C.V3_SYSTEM_WEIGHT_N)


def landing_acceptance_authority() -> dict:
    """The complete frozen authority as a JSON-serializable dictionary."""
    return {
        "schema_version": "1.0.0",
        "authority_id": V3_LANDING_AUTHORITY_ID,
        "authority_version": V3_LANDING_AUTHORITY_VERSION,
        "mission": "RES86A_LANDING_AUTHORITY_BRANCH_AND_RECORDER_FOUNDATION_001",
        "linear_issue": "RES-86",
        "status": "FROZEN",
        "implements": {
            "res82_landing_contract": RES82_LANDING_CONTRACT,
            "res82_event_contract": RES82_EVENT_CONTRACT,
            "res82_owner_decisions": RES82_OWNER_DECISIONS,
            "res82_gates_implemented": [
                "L4-G1", "L4-G2", "L4-G3", "L4-G4", "L4-G5", "L4-G6",
                "L4-T1", "L4-T3", "L4-T4", "L4-T5", "L4-T6", "L4-T7",
                "L4-T8", "L4-T9", "L4-T10",
            ],
            "od_decisions_carried": {
                "OD-04": "L4 landing numeric bounds (medium candidate set)",
                "OD-08": "V_DESC / V_ABS_TAIL 0.10 / 0.05 m/s",
                "OD-10": "D_EST / D_BL 0.050 / 0.050 s",
            },
        },
        "classification": {
            "class": V3_LANDING_AUTHORITY_CLASS,
            "statement": (
                "The frozen limits are owner engineering task bounds with a "
                "predeclared sensitivity domain.  They are not elite-soccer "
                "normative physiology and no human-capability claim is derived."
            ),
            "must_not_claim": [
                "elite-soccer normative landing physiology",
                "injury-risk or safety thresholds",
                "optimal landing technique",
            ],
        },
        "event_labels": {
            "note": (
                "The RES-82 event contract numbers the apex E8 and first physical "
                "re-contact E9/LANDING_FIRST_CONTACT.  The RES-86 branch authority "
                "uses E8_FIRST_CONTACT as the branch label for the first physical "
                "re-contact after flight (the mission branch point); it is the same "
                "physical event as the RES-82 LANDING_FIRST_CONTACT."
            ),
            "PRE_TOUCHDOWN": "last native sample before any post-flight floor contact",
            "E8_FIRST_CONTACT": (
                "first native sample after physical flight with at least one legal plantar "
                "floor contact row and foot clearance <= 0"
            ),
            "E8_FIRST_CONTACT_RES82_MAPPING": "RES-82 LANDING_FIRST_CONTACT (contract E9)",
            "E9_BILATERAL_ESTABLISHED": (
                "first native sample after E8_FIRST_CONTACT where both feet have legal "
                "plantar contact rows and both feet carry Fz > F_thr"
            ),
            "E10_IMPACT_ABSORPTION": (
                "bilateral established then both feet loaded sustained for D_BL with "
                "abs(com_vz) < V_ABS_TAIL continuously"
            ),
            "E10_CONFIRMATION": (
                "first native sample at or after E9_bilateral_onset + D_BL satisfying the "
                "sustained E10 predicate"
            ),
            "E11_BALANCE_CAPTURE": "RES-82 E11; OD-05 bounds; deferred to RES-87",
            "E12_STABLE_RECOVERY": "RES-82 E12; deferred to RES-87",
        },
        "dwells": {
            "D_EST_S": D_EST_S,
            "D_BL_S": D_BL_S,
            "DT_NOMINAL_S": V3_DWELL_DT_NOMINAL_S,
            "DWELL_TYPE": V3_DWELL_SEMANTICS,
            "semantics": (
                "D_EST is the allowance from E8_FIRST_CONTACT to bilateral "
                "establishment; D_BL is the bilateral-loaded sustain dwell that "
                "confirms E10.  They are separate quantities and one value is "
                "never reused for the other.  Both are PHYSICAL_TIME dwells: the "
                "runtime authority is t_current - t_onset >= D, never a sample "
                "count.  The inclusive true-sample count is diagnostic only."
            ),
            "D_EST": D_EST_REQUIREMENTS,
            "D_BL": D_BL_REQUIREMENTS,
            "negative_controls": {
                "24_INTERVALS_0P048_S": "MUST_NOT_CONFIRM",
                "25_INTERVALS_0P050_S": "BOUNDARY_MAY_CONFIRM_IF_ALL_OTHER_GATES_PASS",
            },
        },
        "vertical_rates": {
            "V_DESC_M_S": 0.10,
            "V_ABS_TAIL_M_S": 0.05,
            "E8_E9_onset": "com_vz < -V_DESC_M_S at the first-contact sample (descending COM)",
            "E10_sustain": "abs(com_vz) < V_ABS_TAIL_M_S at every native sample of the E10 run",
        },
        "body_weight": {
            "BW_N": BW_N,
            "definition": "V3 SYSTEM weight = (79 kg athlete + 20 kg load) * 9.81 m/s^2",
            "never": "athlete-only weight",
        },
        "threshold_friction": {
            "F_thr_N": 10.0,
            "role": (
                "per-foot loaded predicate for bilateral establishment (RES-82/OD-09); "
                "never defines physical takeoff or flight"
            ),
        },
        "E10_point_in_time": {
            "scope": "E10_CONFIRMATION native sample",
            "max_abs_com_vx_m_s": 0.30,
            "max_abs_Hy_kg_m2_s": 5.0,
            "max_abs_root_pitch_rad": 0.35,
            "max_abs_trunk_pitch_rad": 0.45,
            "max_abs_root_pitch_rate_rad_s": 3.0,
            "max_abs_trunk_pitch_rate_rad_s": 3.0,
            "definitions": {
                "root_pitch": "orientation.root_pitch_rad (measured root joint coordinate)",
                "trunk_pitch": "orientation.trunk_absolute_pitch_rad (root pitch + trunk/pelvis relative pitch)",
                "Hy": "whole-body centroidal angular momentum about world y (RES-85E sealed convention)",
            },
        },
        "first_contact_to_E10_window": {
            "scope": "inclusive native window [E8_FIRST_CONTACT, E10_CONFIRMATION]",
            "max_abs_com_vx_m_s": 0.40,
            "max_abs_root_pitch_rate_rad_s": 5.0,
            "max_abs_trunk_pitch_rate_rad_s": 5.0,
            "max_abs_Hy_kg_m2_s": 5.0,
            "root_pitch_envelope_rad": 0.45,
            "trunk_pitch_envelope_rad": 0.55,
        },
        "physical_gates": {
            "max_penetration_m": 0.010,
            "penetration_definition": (
                "max over all detected contacts of max(0, -contact.dist) in the landing gate scope"
            ),
            "peak_total_floor_fz_bw": 8.0,
            "peak_fz_definition": (
                "max total_floor_force_world_n[2] (every active floor contact, including "
                "prohibited/fall load) / BW_N"
            ),
            "peak_fz_role": "FAIL_SAFE_TASK_ENVELOPE_NOT_A_HUMAN_NORMATIVE_TARGET",
            "prohibited_or_fall_contact_allowed": False,
            "material_reflight_allowed": False,
            "material_reflight_definition": (
                "a legal-support-free native interval after E8_FIRST_CONTACT of at least "
                "D_BL_S (shorter losses are chatter)"
            ),
            "chatter_transitions_max": 8,
            "chatter_transition_definition": (
                "native samples k > E8_FIRST_CONTACT whose boolean legal plantar support "
                "(legal_plantar_active > 0) differs from sample k - 1"
            ),
            "landing_gate_scope": (
                "[E8_FIRST_CONTACT, E10_CONFIRMATION] for penetration, peak Fz, "
                "prohibited/fall, E10 point gates and first-contact-to-E10 window gates"
            ),
            "branch_scope": (
                "[E8_FIRST_CONTACT, end of executed branch] for chatter, material reflight, "
                "frozen structural ROM and RES-85 hard actuation authority"
            ),
        },
        "structural_rom": {
            "authority": "loaded_cmj.v3.constants.V3_JOINT_RANGES_RAD (frozen RES-83/RES-95 Plant ROM)",
            "scope": "every bounded joint coordinate, every native sample of the branch scope",
            "tolerance_rad": V3_STRUCTURAL_ROM_TOLERANCE_RAD,
            "tolerance_role": V3_STRUCTURAL_ROM_TOLERANCE_ROLE,
            "inherits": (
                "RES-85D strict structural-ROM comparison semantics "
                "(audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001); the joint "
                "ranges themselves are unchanged and the tolerance is never "
                "added anatomical ROM"
            ),
        },
        "actuation_authority": {
            "authority": "LCMJ_RES85_ACTUATION_AUTHORITY_V1 (loaded_cmj.v3.actuation.V3ActuationAuthority)",
            "scope": "every native sample of the branch scope",
            "rule": "unchanged; RES-86 adds no bypass, override or state write",
        },
        "sensitivity_predeclaration": {
            "status": "PREDECLARED_NOT_EXECUTED_IN_RES86A",
            "mujoco_version": "3.8.0",
            "nominal": {
                "dt_s": 0.002,
                "solref": [0.02, 1.0],
                "solimp": [0.9, 0.95, 0.001, 0.5, 2.0],
                "plantar_condim": 4,
                "mu_slide": 0.9,
            },
            "domains": {
                "mu_slide": [0.5, 0.9, 1.5],
                "condim": [3, 4, 6],
                "dt_s": [0.001, 0.002, 0.004],
            },
            "provenance": [
                "RES-84 CONTACT_PARAMETER_AUTHORITY sensitivity_domain",
                "RES-84 SAMPLING_RESAMPLING_AUTHORITY timestep obligation "
                "(>= 1000 Hz candidates require dt <= 0.001 s)",
            ],
            "rule": (
                "No broad sensitivity search is executed in RES-86A.  RES-86B may "
                "run targeted derivative/branch checks; the final solution "
                "verification sweep belongs to RES-89/RES-91."
            ),
        },
        "gate_closure_status": {
            "principle": "thresholds are frozen here; closure status is per gate and never silently promoted",
            "L4-G1": "OPEN_PENDING_RES86B",
            "L4-G2": "OPEN_PENDING_RES86B",
            "L4-G3": "OPEN_PENDING_RES86B",
            "L4-G4": "OPEN_PENDING_RES86B",
            "L4-G5": "OPEN_PENDING_RES86B",
            "L4-G6": "OPEN_PENDING_RES86B",
            "L4-T1": "OPEN_PENDING_RES86B",
            "L4-T3": "OPEN_PENDING_RES86B",
            "L4-T4": "OPEN_PENDING_RES86B",
            "L4-T5": "OPEN_PENDING_RES86B",
            "L4-T6": "OPEN_PENDING_RES86B",
            "L4-T7": "OPEN_PENDING_RES86B",
            "L4-T8": "NOT_EVALUABLE_IN_RES86A_DEFERRED_RES86_TO_RES87_HANDOFF",
            "L4-T9": "OPEN_PENDING_RES86B",
            "L4-T10": "OPEN_PENDING_RES86B",
            "E11": "DEFERRED_TO_RES87",
            "E12": "DEFERRED_TO_RES87",
        },
        "handoff": {
            "RES86A": "authority + branch evidence + lossless recorder foundation only",
            "RES86B": "active-set-safe derivative and branch validation against the captured branch state",
            "RES87": "L4-T8 behavioral window, E11 bounds (OD-05) and E12 stable-recovery envelope",
            "false_closure_prohibition": [
                "L4-T8", "E11", "E12", "C_11", "HY_11", "M_11",
            ],
        },
    }


def authority_canonical_bytes(authority: dict | None = None) -> bytes:
    """Canonical byte view of the authority (sorted keys, compact separators)."""
    payload = landing_acceptance_authority() if authority is None else authority
    return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")


def authority_sha256(authority: dict | None = None) -> str:
    return hashlib.sha256(authority_canonical_bytes(authority)).hexdigest()


def validate_authority(authority: dict | None = None) -> list[str]:
    """Fail-closed structural validation; returns an empty list when valid."""
    payload = landing_acceptance_authority() if authority is None else authority
    failures: list[str] = []
    if payload.get("authority_id") != V3_LANDING_AUTHORITY_ID:
        failures.append("AUTHORITY_ID_MISMATCH")
    if payload.get("classification", {}).get("class") != V3_LANDING_AUTHORITY_CLASS:
        failures.append("CLASSIFICATION_MISMATCH")
    required = ("dwells", "vertical_rates", "E10_point_in_time", "first_contact_to_E10_window",
                "physical_gates", "sensitivity_predeclaration", "gate_closure_status", "handoff")
    for key in required:
        if key not in payload:
            failures.append(f"MISSING_SECTION:{key}")
    if failures:
        return failures
    dwells = payload["dwells"]
    if dwells["D_EST_S"] != 0.050 or dwells["D_BL_S"] != 0.050:
        failures.append("DWELL_VALUES_NOT_FROZEN")
    if dwells.get("DT_NOMINAL_S") != V3_DWELL_DT_NOMINAL_S:
        failures.append("DWELL_DT_NOT_FROZEN")
    if dwells.get("DWELL_TYPE") != V3_DWELL_SEMANTICS:
        failures.append("DWELL_TYPE_NOT_PHYSICAL_TIME")
    for label, expected in (("D_EST", D_EST_REQUIREMENTS), ("D_BL", D_BL_REQUIREMENTS)):
        requirements = dwells.get(label)
        if not isinstance(requirements, dict):
            failures.append(f"DWELL_REQUIREMENTS_MISSING:{label}")
            continue
        if requirements.get("REQUIRED_INTERVALS") != expected["REQUIRED_INTERVALS"]:
            failures.append(f"DWELL_REQUIRED_INTERVALS_NOT_FROZEN:{label}")
        if requirements.get("REQUIRED_TRUE_SAMPLES_INCLUSIVE") != \
                expected["REQUIRED_TRUE_SAMPLES_INCLUSIVE"]:
            failures.append(f"DWELL_REQUIRED_SAMPLES_NOT_FROZEN:{label}")
        if requirements.get("SAMPLE_COUNT_ROLE") != "DIAGNOSTIC_ONLY":
            failures.append(f"DWELL_SAMPLE_COUNT_NOT_DIAGNOSTIC:{label}")
    if payload["vertical_rates"]["V_DESC_M_S"] != 0.10 or payload["vertical_rates"]["V_ABS_TAIL_M_S"] != 0.05:
        failures.append("VERTICAL_RATE_VALUES_NOT_FROZEN")
    if payload["physical_gates"]["chatter_transitions_max"] != 8:
        failures.append("CHATTER_LIMIT_NOT_FROZEN")
    if payload["physical_gates"]["peak_total_floor_fz_bw"] != 8.0:
        failures.append("PEAK_FZ_LIMIT_NOT_FROZEN")
    if payload["physical_gates"]["max_penetration_m"] != 0.010:
        failures.append("PENETRATION_LIMIT_NOT_FROZEN")
    closure = payload["gate_closure_status"]
    for gate in ("L4-T8", "E11", "E12"):
        if "DEFERRED" not in str(closure.get(gate, "")) and "NOT_EVALUABLE" not in str(closure.get(gate, "")):
            failures.append(f"FALSE_CLOSURE:{gate}")
    if payload["sensitivity_predeclaration"]["status"] != "PREDECLARED_NOT_EXECUTED_IN_RES86A":
        failures.append("SENSITIVITY_STATUS_MISMATCH")
    structural_rom = payload.get("structural_rom", {})
    if structural_rom.get("tolerance_rad") != V3_STRUCTURAL_ROM_TOLERANCE_RAD:
        failures.append("STRUCTURAL_ROM_TOLERANCE_NOT_INHERITED_RES85D")
    if structural_rom.get("tolerance_role") != V3_STRUCTURAL_ROM_TOLERANCE_ROLE:
        failures.append("STRUCTURAL_ROM_TOLERANCE_ROLE_MISMATCH")
    return failures


__all__ = [
    "BW_N",
    "D_BL_REQUIREMENTS",
    "D_BL_S",
    "D_EST_REQUIREMENTS",
    "D_EST_S",
    "RES82_BUNDLE",
    "RES82_EVENT_CONTRACT",
    "RES82_LANDING_CONTRACT",
    "RES82_OWNER_DECISIONS",
    "V3_DWELL_DT_NOMINAL_S",
    "V3_DWELL_SEMANTICS",
    "V3_LANDING_AUTHORITY_CLASS",
    "V3_LANDING_AUTHORITY_ID",
    "V3_LANDING_AUTHORITY_VERSION",
    "V3_STRUCTURAL_ROM_TOLERANCE_RAD",
    "V3_STRUCTURAL_ROM_TOLERANCE_ROLE",
    "authority_canonical_bytes",
    "authority_sha256",
    "dwell_confirmed",
    "dwell_requirements",
    "landing_acceptance_authority",
    "structural_rom_violation",
    "validate_authority",
]
