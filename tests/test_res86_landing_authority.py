"""RES-86A frozen V3 landing acceptance authority tests.

MISSION: `RES86A_LANDING_AUTHORITY_BRANCH_AND_RECORDER_FOUNDATION_001`
LINEAR ISSUE: RES-86

The tracked machine-readable authority
(`audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/`) must equal the
canonical authority computed by `src/loaded_cmj/v3/landing_authority.py`, the
numeric limits must be exactly the frozen RES-82 downstream decisions, and the
authority must never falsely close L4-T8 / E11 / E12.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np

TASK_ROOT = Path(__file__).resolve().parents[1]
SRC = TASK_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3.landing_authority import (  # noqa: E402
    D_BL_REQUIREMENTS,
    D_BL_S,
    D_EST_REQUIREMENTS,
    D_EST_S,
    V3_DWELL_DT_NOMINAL_S,
    V3_DWELL_SEMANTICS,
    V3_LANDING_AUTHORITY_CLASS,
    V3_LANDING_AUTHORITY_ID,
    authority_sha256,
    dwell_confirmed,
    dwell_requirements,
    landing_acceptance_authority,
    validate_authority,
)
from loaded_cmj.v3.landing_metrics import (  # noqa: E402
    material_reflight_intervals,
)

AUDIT_DIR = TASK_ROOT / "audit" / "EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001"
AUTHORITY_JSON = AUDIT_DIR / "V3_LANDING_ACCEPTANCE_AUTHORITY.json"
DIGEST_JSON = AUDIT_DIR / "V3_LANDING_ACCEPTANCE_AUTHORITY_DIGEST.json"


def test_tracked_authority_matches_module_canonical_digest():
    tracked = json.loads(AUTHORITY_JSON.read_text())
    assert validate_authority(tracked) == []
    assert authority_sha256(tracked) == authority_sha256(landing_acceptance_authority())
    digest = json.loads(DIGEST_JSON.read_text())
    assert digest["canonical_sha256"] == authority_sha256(tracked)
    assert digest["authority_id"] == V3_LANDING_AUTHORITY_ID
    assert digest["classification"] == V3_LANDING_AUTHORITY_CLASS
    module = TASK_ROOT / "src/loaded_cmj/v3/landing_authority.py"
    assert digest["source_module_sha256"] == hashlib.sha256(module.read_bytes()).hexdigest()


def test_frozen_dwells_and_vertical_rates():
    authority = landing_acceptance_authority()
    assert authority["dwells"]["D_EST_S"] == 0.050
    assert authority["dwells"]["D_BL_S"] == 0.050
    assert authority["vertical_rates"]["V_DESC_M_S"] == 0.10
    assert authority["vertical_rates"]["V_ABS_TAIL_M_S"] == 0.05


# ---------------------------------------------------------------------------
# A3: physical-time dwell semantics
# ---------------------------------------------------------------------------
def test_dwell_requirements_are_physical_time_intervals():
    assert V3_DWELL_SEMANTICS == "PHYSICAL_TIME"
    assert V3_DWELL_DT_NOMINAL_S == 0.002
    assert D_EST_S == D_BL_S == 0.050
    for requirements in (D_EST_REQUIREMENTS, D_BL_REQUIREMENTS):
        assert requirements["DWELL_TYPE"] == "PHYSICAL_TIME"
        assert requirements["REQUIRED_INTERVALS"] == 25
        assert requirements["REQUIRED_TRUE_SAMPLES_INCLUSIVE"] == 26
        assert requirements["RUNTIME_AUTHORITY"] == "t_current - t_onset >= DURATION_S"
        assert requirements["SAMPLE_COUNT_ROLE"] == "DIAGNOSTIC_ONLY"
    assert dwell_requirements(0.050, 0.001)["REQUIRED_INTERVALS"] == 50
    assert dwell_requirements(0.050, 0.001)["REQUIRED_TRUE_SAMPLES_INCLUSIVE"] == 51


def test_dwell_negative_control_24_intervals_must_not_confirm():
    dt = 0.002
    samples = 25
    assert dwell_confirmed(0.0, (samples - 1) * dt, 0.050) is False
    assert (samples - 1) == 24
    assert (samples - 1) * dt == 0.048


def test_dwell_boundary_25_intervals_may_confirm():
    dt = 0.002
    samples = 26
    assert dwell_confirmed(0.0, (samples - 1) * dt, 0.050) is True
    assert (samples - 1) == 25
    assert (samples - 1) * dt == 0.050


def test_authority_dwell_section_declares_negative_controls():
    dwells = landing_acceptance_authority()["dwells"]
    assert dwells["DT_NOMINAL_S"] == 0.002
    assert dwells["DWELL_TYPE"] == "PHYSICAL_TIME"
    assert dwells["D_EST"]["REQUIRED_INTERVALS"] == 25
    assert dwells["D_EST"]["REQUIRED_TRUE_SAMPLES_INCLUSIVE"] == 26
    assert dwells["D_BL"]["REQUIRED_INTERVALS"] == 25
    assert dwells["D_BL"]["REQUIRED_TRUE_SAMPLES_INCLUSIVE"] == 26
    controls = dwells["negative_controls"]
    assert controls["24_INTERVALS_0P048_S"] == "MUST_NOT_CONFIRM"
    assert controls["25_INTERVALS_0P050_S"] == \
        "BOUNDARY_MAY_CONFIRM_IF_ALL_OTHER_GATES_PASS"
    assert "native_samples_at_nominal_dt" not in dwells
    assert validate_authority() == []


def test_material_reflight_uses_physical_time_not_sample_count():
    dt = 0.002
    support = np.zeros(60, dtype=bool)
    support[:10] = True
    support[10:35] = False   # 25 inclusive samples = 24 intervals = 0.048 s
    support[35:] = True
    times = np.arange(60, dtype=np.float64) * dt
    assert material_reflight_intervals(
        support, 0, sample_times_s=times, min_duration_s=0.050) == []
    support = np.zeros(61, dtype=bool)
    support[:10] = True
    support[10:36] = False   # 26 inclusive samples = 25 intervals = 0.050 s
    support[36:] = True
    times = np.arange(61, dtype=np.float64) * dt
    assert material_reflight_intervals(
        support, 0, sample_times_s=times, min_duration_s=0.050) == [(10, 35)]


def test_frozen_e10_point_in_time_limits():
    point = landing_acceptance_authority()["E10_point_in_time"]
    assert point["max_abs_com_vx_m_s"] == 0.30
    assert point["max_abs_Hy_kg_m2_s"] == 5.0
    assert point["max_abs_root_pitch_rad"] == 0.35
    assert point["max_abs_trunk_pitch_rad"] == 0.45
    assert point["max_abs_root_pitch_rate_rad_s"] == 3.0
    assert point["max_abs_trunk_pitch_rate_rad_s"] == 3.0


def test_frozen_first_contact_to_e10_window_limits():
    window = landing_acceptance_authority()["first_contact_to_E10_window"]
    assert window["max_abs_com_vx_m_s"] == 0.40
    assert window["max_abs_root_pitch_rate_rad_s"] == 5.0
    assert window["max_abs_trunk_pitch_rate_rad_s"] == 5.0
    assert window["max_abs_Hy_kg_m2_s"] == 5.0
    assert window["root_pitch_envelope_rad"] == 0.45
    assert window["trunk_pitch_envelope_rad"] == 0.55


def test_frozen_physical_gates():
    physical = landing_acceptance_authority()["physical_gates"]
    assert physical["max_penetration_m"] == 0.010
    assert physical["peak_total_floor_fz_bw"] == 8.0
    assert physical["prohibited_or_fall_contact_allowed"] is False
    assert physical["material_reflight_allowed"] is False
    assert physical["chatter_transitions_max"] == 8
    assert physical["peak_fz_role"] == "FAIL_SAFE_TASK_ENVELOPE_NOT_A_HUMAN_NORMATIVE_TARGET"
    authority = landing_acceptance_authority()
    assert authority["body_weight"]["BW_N"] == C.V3_SYSTEM_WEIGHT_N
    assert authority["body_weight"]["never"] == "athlete-only weight"


def test_classification_is_never_physiology():
    authority = landing_acceptance_authority()
    assert authority["classification"]["class"] == "OWNER_ENGINEERING_TASK_BOUND_WITH_SENSITIVITY"
    forbidden = " ".join(authority["classification"]["must_not_claim"]).lower()
    assert "normative" in forbidden and "physiology" in forbidden


def test_no_false_closure_of_t8_e11_e12():
    authority = landing_acceptance_authority()
    closure = authority["gate_closure_status"]
    assert "DEFERRED" in closure["L4-T8"] or "NOT_EVALUABLE" in closure["L4-T8"]
    assert closure["E11"] == "DEFERRED_TO_RES87"
    assert closure["E12"] == "DEFERRED_TO_RES87"
    for gate in ("L4-T8", "E11", "E12"):
        assert gate in authority["handoff"]["false_closure_prohibition"]
    assert validate_authority(authority) == []


def test_sensitivity_predeclaration_is_frozen_not_executed():
    sensitivity = landing_acceptance_authority()["sensitivity_predeclaration"]
    assert sensitivity["status"] == "PREDECLARED_NOT_EXECUTED_IN_RES86A"
    assert sensitivity["mujoco_version"] == "3.8.0"
    assert sensitivity["domains"]["mu_slide"] == [0.5, 0.9, 1.5]
    assert sensitivity["domains"]["condim"] == [3, 4, 6]
    assert sensitivity["domains"]["dt_s"] == [0.001, 0.002, 0.004]
    nominal = sensitivity["nominal"]
    assert nominal["dt_s"] == 0.002
    assert nominal["solref"] == [0.02, 1.0]
    assert nominal["solimp"] == [0.9, 0.95, 0.001, 0.5, 2.0]
    assert nominal["plantar_condim"] == 4
    assert nominal["mu_slide"] == 0.9


def test_authority_implements_res82_artifacts():
    implements = landing_acceptance_authority()["implements"]
    assert implements["res82_landing_contract"].endswith("LANDING_BALANCE_RECOVERY_CONTRACT.json")
    assert "L4-T8" in implements["res82_gates_implemented"]
    assert set(implements["od_decisions_carried"]) == {"OD-04", "OD-08", "OD-10"}


# ---------------------------------------------------------------------------
# A4: structural-ROM tolerance inheritance (RES-85D semantics)
# ---------------------------------------------------------------------------
from loaded_cmj.v3.landing_authority import (  # noqa: E402
    V3_STRUCTURAL_ROM_TOLERANCE_RAD,
    V3_STRUCTURAL_ROM_TOLERANCE_ROLE,
    structural_rom_violation,
)


def test_structural_rom_tolerance_inherits_res85d():
    assert V3_STRUCTURAL_ROM_TOLERANCE_RAD == 1e-9
    assert V3_STRUCTURAL_ROM_TOLERANCE_ROLE == \
        "FLOATING_POINT_EQUALITY_ONLY_NOT_ANATOMICAL_ROM"
    rom = landing_acceptance_authority()["structural_rom"]
    assert rom["tolerance_rad"] == 1e-9
    assert rom["tolerance_role"] == "FLOATING_POINT_EQUALITY_ONLY_NOT_ANATOMICAL_ROM"
    assert "RES-85D" in rom["inherits"]
    assert validate_authority() == []


def test_structural_rom_tolerance_controls_upper_and_lower_bounds():
    lo, hi = -0.610865, 0.610865
    # q == bound passes; within the 1e-9 comparison tolerance passes
    for bound in (lo, hi):
        assert structural_rom_violation(bound, lo, hi) is False
        assert structural_rom_violation(bound + 5e-10, lo, hi) is False
        assert structural_rom_violation(bound - 5e-10, lo, hi) is False
    # material overshoot beyond the tolerance fails on each side
    assert structural_rom_violation(lo - 2e-9, lo, hi) is True
    assert structural_rom_violation(hi + 2e-9, lo, hi) is True
    assert structural_rom_violation(lo - 1e-3, lo, hi) is True
    assert structural_rom_violation(hi + 1e-3, lo, hi) is True


def test_structural_rom_tolerance_is_not_anatomical_rom():
    lo, hi = 0.0, 2.443461
    assert structural_rom_violation(hi + 0.001, lo, hi) is True
    assert structural_rom_violation(lo - 0.001, lo, hi) is True


def test_tracked_structural_rom_authority_matches_module():
    tracked = json.loads(AUTHORITY_JSON.read_text())["structural_rom"]
    assert tracked["tolerance_rad"] == V3_STRUCTURAL_ROM_TOLERANCE_RAD
    assert tracked["tolerance_role"] == V3_STRUCTURAL_ROM_TOLERANCE_ROLE
    assert tracked["tolerance_rad"] != 0.0
