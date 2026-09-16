"""RES-84 mandatory tests — V3 measurement / contact authority.

Authority: LCMJ_RES84_V3_MEASUREMENT_CONTACT_AUTHORITY_V1
Mission:   RES84_REBUILD_V3_MEASUREMENT_CONTACT_AUTHORITY_001

Every test asserts the same deterministic audit computation that is archived in
``audit/EXP-RES84-V3-MEASUREMENT-CONTACT-AUTHORITY-001/``, so the executed
proof and the sealed evidence are one computation.

The mandatory adversarial set (mission section 21) is covered by the tests
below: flat bilateral stance, left/right single support, heel-only,
forefoot-only, toe-only, heel rise, toe off, rotated ankle, rotated MTP, true
flight, near-contact inactive gap contact, active margin contact, known static
load, known applied contact point / CoP, very-low-Fz CoP invalidation,
prohibited pelvis/HAT/bar-floor, toe/forefoot-first landing geometry, nonzero
root pitch, nonzero trunk pitch, takeoff occurrence + valid confirmation,
transient dropout, recontact, 10 N comparator offset, raw/native vs 1000 Hz
provenance and force/COM consistency.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
SRC = TASK_ROOT / "src"
EVIDENCE_DIR = TASK_ROOT / "audit" / "EXP-RES84-V3-MEASUREMENT-CONTACT-AUTHORITY-001"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _load_builder():
    spec = importlib.util.spec_from_file_location("res84_build_evidence", EVIDENCE_DIR / "build_evidence.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


B = _load_builder()

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3 import plant as P  # noqa: E402


@pytest.fixture(scope="session")
def audits():
    return B.build_all()


def _report(audits, name):
    return audits[name]


def _failed(report):
    return [c["check"] for c in report.get("checks", []) if not c["pass"]]


# ---------------------------------------------------------------------------
# authority surface
# ---------------------------------------------------------------------------
def test_measurement_module_is_v3_local_and_distinct():
    source = (SRC / "loaded_cmj" / "v3" / "measurement.py").read_text()
    for forbidden in ("loaded_cmj.v2", "loaded_cmj.simulation", "loaded_cmj.control",
                      "loaded_cmj.oracle", "loaded_cmj.runtime"):
        assert forbidden not in source
    assert "H2_TARGET" not in source


def test_authority_constants_frozen():
    assert M.SYSTEM_MASS_KG == 99.0
    assert M.ATHLETE_MASS_KG == 79.0
    assert M.WRENCH_SIGN_CONVENTION == "GROUND_ON_ATHLETE"
    assert M.WRENCH_REFERENCE_ORIGIN_M == (0.0, 0.0, 0.0)
    assert M.SUPPORT_PLANE_Z_M == 0.0
    assert M.CLEARANCE_GUARD_M == 0.002
    assert M.COP_LOW_FZ_TOLERANCE_N == 1.0e-3
    assert M.TAKEOFF_DWELL_S == 0.050
    assert M.COMPARATOR_FORCE_N == 10.0
    assert M.COMPARATOR_DWELL_S == 0.010
    assert "LEFT_FOOT_FZ" in M.COMPARATOR_PREDICATE
    assert "RIGHT_FOOT_FZ" in M.COMPARATOR_PREDICATE
    assert "total" not in M.COMPARATOR_PREDICATE.lower()
    assert M.COMPARATOR_TOTAL_FZ_ROLE == "REPORT_FIELD_ONLY"
    assert M.NATIVE_DT_S == 0.002 and M.NATIVE_FREQUENCY_HZ == 500.0
    assert M.NATIVE_1000HZ_STREAM_STATUS == "NATIVE_1000HZ_EVENT_TRUTH"
    assert M.CANONICAL_DOWNSAMPLING_STATUS == "DOWNSAMPLING_NOT_AUTHORIZED"
    assert M.REQUIRED_CANDIDATE_MAX_DT_S <= 0.001
    assert M.DOWN_SAMPLING_AUTHORIZED is False
    assert M.DWELL_TYPE_PHYSICAL_TIME == "PHYSICAL_TIME"
    assert M.NANOSECONDS_PER_SECOND == 1_000_000_000
    assert M.DWELL_REJECT_COVERAGE == "REJECT_DWELL_COVERAGE"
    assert M.DWELL_REJECT_SUSTAIN == "REJECT_SUSTAIN_GAP"


def test_mujoco_contact_semantics_version_boundary():
    """MuJoCo 3.8.0 contact semantics are provenance, not a Plant change."""
    import mujoco
    assert M.MUJOCO_CONTACT_SEMANTICS_VERSION == "3.8.0"
    assert mujoco.__version__ == "3.8.0"
    assert "3.9" in M.MUJOCO_CONTACT_SEMANTICS_REQUALIFICATION_NOTE
    pyproject = (TASK_ROOT / "pyproject.toml").read_text()
    assert "mujoco==3.8.0" in pyproject
    report = B.measurement_implementation_spec()
    provenance = report["contact_semantics_provenance"]
    assert provenance["mujoco_contact_semantics_version"] == "3.8.0"
    assert provenance["detection"] == "dist < margin"
    assert provenance["force"] == "dist < margin - gap"


def test_measurement_implementation_spec(audits):
    report = _report(audits, "MEASUREMENT_IMPLEMENTATION_SPEC.json")
    assert report["status"] == "PASS"
    assert report["module"].endswith("src/loaded_cmj/v3/measurement.py")
    assert len(report["module_sha256"]) == 64
    assert len(report["plant_xml_sha256"]) == 64


# ---------------------------------------------------------------------------
# contact semantics
# ---------------------------------------------------------------------------
def test_contact_semantics_audit(audits):
    report = _report(audits, "CONTACT_SEMANTICS_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["summary"]["detected_implies_active"] is False
    assert report["probes"]["exact_touch_ncon"] == 0


def test_detected_but_inactive_penetrating_contact(audits):
    """ncon > 0 must not imply active support (mission section 4)."""
    report = _report(audits, "CONTACT_SEMANTICS_AUDIT.json")
    probe = report["probes"]["sealed_penetrating_inactive"]
    assert probe["ncon"] > 0
    assert all(address >= 0 for address in probe["efc_addresses"])
    assert all(force == 0.0 for force in probe["normal_forces_n"])
    assert all(penetration > 1.0e-4 for penetration in probe["penetrations_m"])
    plant, data = B.probe_sealed_forefoot_penetrating_inactive()
    assert M.contact_state(plant, data).legal_plantar_active == 0
    assert M.cop_from_plant(plant, data).validity is M.V3CopValidity.NOT_EVALUABLE_NO_SUPPORT


def test_margin_gap_detection_vs_activation_bands(audits):
    report = _report(audits, "CONTACT_SEMANTICS_AUDIT.json")
    bands = report["probes"]["margin_gap_bands"]
    assert bands["0.006"]["detected"] is False
    assert bands["0.0045"]["detected"] is True and bands["0.0045"]["active"] is False
    assert bands["0.0035"]["detected"] is True and bands["0.0035"]["active"] is False
    assert bands["0.0028"]["detected"] is True and bands["0.0028"]["active"] is True
    assert bands["0.0045"]["first_efc"] == -1


def test_prohibited_contacts_are_exposed(audits):
    report = _report(audits, "CONTACT_SEMANTICS_AUDIT.json")
    assert report["probes"]["prohibited_pelvis"]["detected"] > 0


# ---------------------------------------------------------------------------
# force frame and wrench
# ---------------------------------------------------------------------------
def test_contact_force_frame_convention(audits):
    report = _report(audits, "CONTACT_FORCE_FRAME_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    agreement = report["details"]["transform_agreement"]
    assert agreement["max_force_error_n"] < 1e-12
    assert agreement["max_torque_error_nm"] < 1e-12


def test_two_box_geom_ordering_independence(audits):
    report = _report(audits, "CONTACT_FORCE_FRAME_AUDIT.json")
    probe = report["details"]["two_box_probe"]
    assert abs(probe["A_first"]["B_force"][2] - 5.0 * C.V3_GRAVITY_M_S2) < 0.02 * 5.0 * C.V3_GRAVITY_M_S2
    assert abs(probe["B_first"]["B_force"][2] - 5.0 * C.V3_GRAVITY_M_S2) < 0.02 * 5.0 * C.V3_GRAVITY_M_S2
    invariant = report["details"]["ordering_invariance"]
    assert np.allclose(invariant["original"], invariant["swapped"], atol=1e-12, rtol=0.0)


def test_ground_wrench_audit(audits):
    report = _report(audits, "GROUND_WRENCH_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert abs(report["flat_stance"]["total_force_n"][2] - 99.0 * C.V3_GRAVITY_M_S2) < 1e-3
    assert report["max_per_body_cfrc_ext_error_n"] < 1e-12


def test_known_static_load_returns_system_weight():
    plant, data, _ = B.flat_stance_equilibrium()
    wrench = M.total_ground_wrench(plant, data)
    assert abs(wrench.force_world_n[2] - 99.0 * C.V3_GRAVITY_M_S2) < 1e-3
    assert abs(wrench.force_world_n[2] - M.SYSTEM_MASS_KG * C.V3_GRAVITY_M_S2) < 1e-3


def test_left_right_foot_wrench_sum():
    plant, data, _ = B.flat_stance_equilibrium()
    total = M.total_ground_wrench(plant, data)
    left = M.foot_wrench(plant, data, "left")
    right = M.foot_wrench(plant, data, "right")
    assert np.allclose(np.asarray(left.force_world_n) + np.asarray(right.force_world_n),
                       total.force_world_n, atol=1e-9, rtol=0.0)
    assert np.allclose(np.asarray(left.moment_world_nm) + np.asarray(right.moment_world_nm),
                       total.moment_world_nm, atol=1e-9, rtol=0.0)


# ---------------------------------------------------------------------------
# SYSTEM_COM
# ---------------------------------------------------------------------------
def test_system_com_audit(audits):
    report = _report(audits, "SYSTEM_COM_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert abs(report["system_mass_kg"] - 99.0) < 1e-12
    assert abs(report["athlete_mass_kg"] - 79.0) < 1e-12


def test_system_com_matches_manual_fk_and_independent_paths():
    plant, data, _ = B.flat_stance_equilibrium()
    state = M.system_com_state(plant, data)
    assert np.allclose(state.com_world_m, B.indep_system_com(plant.model, data), atol=1e-12, rtol=0.0)
    assert np.allclose(state.com_world_m, B.indep_system_com_manual_fk(plant, data), atol=1e-12, rtol=0.0)
    assert np.allclose(state.com_world_m,
                       np.asarray(data.subtree_com[plant.idx.body["pelvis"]]), atol=1e-12, rtol=0.0)


def test_system_and_athlete_com_closure():
    plant, data, _ = B.flat_stance_equilibrium()
    system = M.system_com_state(plant, data)
    athlete = M.athlete_com_state(plant, data)
    bar = plant.idx.body["bar"]
    predicted = (plant.model.body_mass[bar]
                 * (np.asarray(data.xipos[bar]) - np.asarray(system.com_world_m))) / athlete.mass_kg
    assert np.allclose(np.asarray(system.com_world_m) - np.asarray(athlete.com_world_m),
                       predicted, atol=1e-12, rtol=0.0)


# ---------------------------------------------------------------------------
# CoP
# ---------------------------------------------------------------------------
def test_cop_authority_audit(audits):
    report = _report(audits, "COP_AUTHORITY_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["policy"]["low_fz_tolerance_n"] == 1.0e-3
    assert report["policy"]["derivation"]["measured_no_contact_force_floor_n"] == 0.0


def test_cop_synthetic_known_application_point(audits):
    report = _report(audits, "COP_AUTHORITY_AUDIT.json")
    for tag in ("single_point", "single_point_shift", "inclined_force"):
        assert tag in report["synthetic_wrenches"]


def test_cop_validity_enumeration():
    """All six validity states are reachable through declared inputs."""
    seen = set()
    plant, data, _ = B.flat_stance_equilibrium()
    seen.add(M.cop_from_plant(plant, data).validity)
    plant_f, data_f = B.probe_pose("flight")
    seen.add(M.cop_from_plant(plant_f, data_f, flight_context=True).validity)
    seen.add(M.cop_from_plant(plant_f, data_f).validity)
    plant_p, data_p = B.probe_prohibited("pelvis_floor")
    seen.add(M.cop_from_plant(plant_p, data_p).validity)
    low = M.V3Wrench(force_world_n=(0.0, 0.0, 1e-6), moment_world_nm=(0.0, 0.0, 0.0),
                     reference_origin_m=(0.0, 0.0, 0.0), contact_count=1, normal_force_n=1e-6)
    seen.add(M.cop_from_wrench(low).validity)
    bad = M.V3Wrench(force_world_n=(0.0, 0.0, 100.0), moment_world_nm=(float("nan"), 0.0, 0.0),
                     reference_origin_m=(0.0, 0.0, 0.0), contact_count=1, normal_force_n=100.0)
    seen.add(M.cop_from_wrench(bad).validity)
    assert seen == set(M.V3CopValidity)


def test_cop_is_not_contact_centroid():
    plant = P.V3Plant()
    data = plant.make_data()
    B._materialize(plant, data, {}, press_m=3.55785e-4, balance=False)
    data.qpos[plant.idx.qadr["root_tx"]] += 0.08
    import mujoco
    mujoco.mj_forward(plant.model, data)
    cop = M.cop_from_plant(plant, data)
    centroid = np.mean([r.position_world_m for r in M.contact_records(plant, data)], axis=0)
    assert cop.validity is M.V3CopValidity.VALID
    assert abs(cop.cop_x_m - float(centroid[0])) > 5e-3


def test_cop_reference_origin_invariance():
    plant, data, _ = B.flat_stance_equilibrium()
    base = M.cop_from_plant(plant, data)
    records = M.active_legal_plantar_records(M.contact_records(plant, data))
    shifted = M.cop_from_wrench(M._wrench_from_records(records, (0.0, 0.0, 0.5)))
    assert shifted.validity is M.V3CopValidity.VALID
    assert abs(shifted.cop_x_m - base.cop_x_m) < 1e-12
    assert abs(shifted.cop_y_m - base.cop_y_m) < 1e-12


# ---------------------------------------------------------------------------
# support hull
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("name,expected_mode", [
    ("flat", M.V3SupportMode.BILATERAL),
    ("left_support", M.V3SupportMode.LEFT_ONLY),
    ("right_support", M.V3SupportMode.RIGHT_ONLY),
    ("heel_only", M.V3SupportMode.BILATERAL),
    ("toe_only", M.V3SupportMode.BILATERAL),
    ("heel_rise", M.V3SupportMode.BILATERAL),
    ("toe_off", M.V3SupportMode.BILATERAL),
    ("toe_first_landing", M.V3SupportMode.BILATERAL),
])
def test_support_modes(name, expected_mode):
    plant, data = B.probe_pose(name)
    hull = M.active_support_hull(plant, data)
    assert hull.support_mode is expected_mode


def test_support_hull_audit(audits):
    report = _report(audits, "ACTIVE_SUPPORT_HULL_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["cases"]["flight"]["sagittal_margin_m"] is None


def test_flight_never_positive_support_margin():
    plant, data = B.probe_pose("flight")
    hull = M.active_support_hull(plant, data, flight_context=True)
    assert not hull.evaluable
    assert hull.support_mode is M.V3SupportMode.NOT_EVALUABLE
    assert all(v is None for v in M.support_margin_from_point((0.0, 0.0), hull))
    assert hull.sagittal_margin_m is None and hull.planar_margin_m is None and hull.lateral_margin_m is None


def test_active_only_support_geometry():
    """Single support must not use a static both-feet AABB."""
    plant, data = B.probe_pose("left_support")
    hull = M.active_support_hull(plant, data)
    assert {(s, r) for s, r in hull.regions} == {("left", "heel"), ("left", "forefoot"), ("left", "toe")}
    assert all(region[0] == "left" for region in hull.regions)
    assert hull.support_mode is M.V3SupportMode.LEFT_ONLY


def test_forefoot_only_support_zero_passive():
    plant, data = B.probe_zero_passive_forefoot_only()
    hull = M.active_support_hull(plant, data)
    assert {(s, r) for s, r in hull.regions} == {("left", "forefoot"), ("right", "forefoot")}


def test_support_footprint_follows_active_regions():
    flat_plant, flat_data = B.probe_pose("flat")
    heel_plant, heel_data = B.probe_pose("heel_only")
    flat = M.active_support_hull(flat_plant, flat_data)
    heel = M.active_support_hull(heel_plant, heel_data)
    assert flat.area_m2 > heel.area_m2
    assert len(heel.vertices_xy) < len(flat.vertices_xy) or heel.area_m2 < 0.5 * flat.area_m2


# ---------------------------------------------------------------------------
# foot clearance
# ---------------------------------------------------------------------------
def test_foot_clearance_audit(audits):
    report = _report(audits, "FOOT_CLEARANCE_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["semantics"]["no_fixed_local_z_shortcut"] is True
    assert len(report["rom_extremes"]) == 4


@pytest.mark.parametrize("name", ["flat", "heel_only", "toe_only", "heel_rise", "toe_off",
                                  "toe_first_landing", "rotated_ankle_mtp", "rotated_mtp_negative",
                                  "root_pitch", "flight"])
def test_clearance_matches_brute_force(name):
    plant, data = B.probe_pose(name)
    for side in C.V3_SIDES:
        clearance = M.foot_clearance(plant, data, side)
        brute = B.indep_foot_clearance_bruteforce(plant, data, side)
        assert abs(clearance.min_gap_m - brute) < 1e-12
        body_id = plant.idx.body[clearance.governing_body]
        point = np.asarray(clearance.governing_material_point_world_m)
        fd = B.indep_material_point_velocity_fd(plant, data, body_id, point)
        assert abs(clearance.normal_velocity_m_s - float(fd[2])) < 1e-6


def test_clearance_is_never_a_fixed_local_z():
    """The governing gap must respond to the ankle/MTP chain, not a constant."""
    values = []
    for name in ("rotated_ankle_mtp", "rotated_mtp_negative", "heel_rise", "toe_off"):
        plant, data = B.probe_pose(name)
        values.append(M.foot_clearance(plant, data, "left").region_gaps_m)
    assert len({tuple(round(v, 9) for _, v in gaps) for gaps in values}) == len(values)


# ---------------------------------------------------------------------------
# orientation
# ---------------------------------------------------------------------------
def test_orientation_audit(audits):
    report = _report(audits, "ORIENTATION_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    probe = report["details"]["root_pitch_probe"]
    assert abs(probe["root_pitch_rad"]) > 0.1
    assert abs(probe["root_pitch_from_quaternion_rad"] - probe["root_pitch_rad"]) < 1e-9


def test_nonzero_root_and_trunk_pitch_never_identity_quaternion():
    plant, data = B.probe_pose("root_pitch")
    state = M.orientation_state(plant, data)
    assert abs(state.root_pitch_rad) > 0.1
    assert abs(state.trunk_pelvis_relative_pitch_rad) > 0.1
    assert not np.allclose(state.pelvis_quaternion_wxyz, (1.0, 0.0, 0.0, 0.0), atol=1e-6)
    assert not np.allclose(state.hat_quaternion_wxyz, (1.0, 0.0, 0.0, 0.0), atol=1e-6)
    assert abs(state.trunk_absolute_pitch_rad
               - (state.root_pitch_rad + state.trunk_pelvis_relative_pitch_rad)) < 1e-15


# ---------------------------------------------------------------------------
# prohibited / penetration
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("kind", ["pelvis_floor", "hat_floor", "bar_floor"])
def test_prohibited_floor_visible(kind):
    plant, data = B.probe_prohibited(kind)
    state = M.contact_state(plant, data)
    assert state.prohibited_detected > 0
    assert state.body_floor_fall_visible is True
    assert state.max_penetration_prohibited_m > 0.0
    assert M.cop_from_plant(plant, data).validity is M.V3CopValidity.NOT_EVALUABLE_PROHIBITED_CONTACT


def test_penetration_metric_covers_full_contact_buffer():
    plant, data = B.probe_prohibited("pelvis_floor")
    state = M.contact_state(plant, data)
    manual = max(r.penetration_m for r in M.contact_records(plant, data))
    assert abs(state.max_penetration_all_detected_m - manual) < 1e-15


# ---------------------------------------------------------------------------
# clearance guard
# ---------------------------------------------------------------------------
def test_clearance_guard_audit(audits):
    report = _report(audits, "CLEARANCE_GUARD_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["sealed_clearance_guard_m"] == 0.002
    terms = report["terms"]
    assert terms["effective_contact_margin_m"]["value"] == 0.0
    assert terms["verified_penetration_allowance_m"]["value"] < 0.002


# ---------------------------------------------------------------------------
# events
# ---------------------------------------------------------------------------
def test_takeoff_occurrence_confirmation_audit(audits):
    report = _report(audits, "TAKEOFF_OCCURRENCE_CONFIRMATION_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    hop = report["ballistic_hop"]
    assert hop["occurrence"]["valid"] is True
    assert hop["confirmation"]["confirmed"] is True
    assert hop["occurrence"]["interpolated"] in (True, False)
    assert hop["confirmation"]["TAKEOFF_CONFIRMATION_SAMPLE"] == 39
    assert hop["confirmation"]["TAKEOFF_CONFIRMATION_TIME"] == 0.078
    assert hop["confirmation"]["TAKEOFF_CONFIRMATION_ELAPSED_S"] == 0.050
    assert hop["confirmation"]["required_end_time_s"] == 0.078
    offgrid = report["offgrid_occurrence_regression"]
    assert offgrid["confirmed"] is True
    assert offgrid["confirmation_sample"] == 39
    assert offgrid["not_the_last_sample_before_required_end"] is True
    truncated = report["truncated_stream_control"]
    assert truncated["confirmed"] is False
    assert truncated["reason"] == "REJECT_DWELL_COVERAGE"
    controls = report["negative_controls"]
    assert controls["final_sample_sustain_gap"]["reason"] == "REJECT_SUSTAIN_GAP"
    assert controls["prohibited_at_confirmation_sample"]["confirmed"] is False


def test_takeoff_occurrence_is_support_to_zero_transition():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    assert occurrence.valid
    assert frames[occurrence.last_support_index].legal_plantar_active > 0
    assert frames[occurrence.native_index].legal_plantar_active == 0
    assert frames[occurrence.last_support_index].time_s <= occurrence.occurrence_time_s
    assert occurrence.occurrence_time_s <= frames[occurrence.native_index].time_s + 1e-15
    assert occurrence.com_velocity_world_m_s[2] > 0.0


def test_candidate_scan_returns_ordered_unshifted_candidates():
    plant, data, frames = B.launch_flight_stream()
    candidates = M.scan_takeoff_candidates(frames)
    assert candidates
    assert all(c.valid for c in candidates)
    indices = [c.native_index for c in candidates]
    assert indices == sorted(indices)
    first = M.detect_takeoff_occurrence(frames)
    assert candidates[0].native_index == first.native_index
    assert candidates[0].occurrence_time_s == first.occurrence_time_s


def test_takeoff_confirmation_is_fail_closed_on_recontact():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    confirmation = M.confirm_takeoff(frames, occurrence)
    assert confirmation.confirmed
    # recontact inside the dwell window must reject, never shift
    import dataclasses
    synthetic = list(frames[: occurrence.native_index + 5])
    next_index = synthetic[-1].index + 1
    synthetic.append(dataclasses.replace(
        synthetic[-1], index=next_index, time_s=next_index * M.NATIVE_DT_S,
        legal_plantar_active=2, legal_plantar_detected=2))
    rejected = M.confirm_takeoff(synthetic, occurrence)
    assert not rejected.confirmed
    assert "no_legal_plantar_recontact" in rejected.failed_checks


def test_transient_dropout_does_not_confirm():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)

    def synthetic_frame(i, active, clearance, vz):
        return M.V3NativeFrame(
            index=i, time_s=i * M.NATIVE_DT_S,
            com_world_m=(0.0, 0.0, 1.0), com_velocity_world_m_s=(0.0, 0.0, vz),
            athlete_com_world_m=(0.0, 0.0, 0.9), left_clearance_m=clearance, right_clearance_m=clearance,
            legal_plantar_detected=active, legal_plantar_active=active,
            legal_plantar_normal_force_n=100.0 if active else 0.0,
            prohibited_detected=0, prohibited_active=0,
            total_floor_force_world_n=(0.0, 0.0, 900.0 if active else 0.0),
            legal_ground_force_world_n=(0.0, 0.0, 900.0 if active else 0.0),
            legal_ground_moment_world_nm=(0.0, 0.0, 0.0),
            left_foot_force_world_n=(0.0, 0.0, 450.0 if active else 0.0),
            right_foot_force_world_n=(0.0, 0.0, 450.0 if active else 0.0),
            nonplantar_floor_active=0,
            cop_validity="VALID", cop_x_m=0.0,
            support_mode="BILATERAL" if active else "NOT_EVALUABLE")

    dropout = [synthetic_frame(i, 0 if i in (10, 11) else 2, 0.003 if i in (10, 11) else 1e-4, 1.0)
               for i in range(40)]
    candidate = M.detect_takeoff_occurrence(dropout)
    confirmation = M.confirm_takeoff(dropout, candidate)
    assert candidate.valid
    assert not confirmation.confirmed
    del occurrence


def test_clearance_guard_rejection_is_real():
    """A shallower real launch is detected but rejected by the 2 mm guard."""
    plant, data, frames = B.launch_flight_stream(press_m=0.0485)
    occurrence = M.detect_takeoff_occurrence(frames)
    confirmation = M.confirm_takeoff(frames, occurrence)
    assert occurrence.valid
    assert not confirmation.confirmed
    assert confirmation.bilateral_clearance_max_m < M.CLEARANCE_GUARD_M


def test_comparator_is_diagnostic_only(audits):
    report = _report(audits, "FORCE_THRESHOLD_COMPARATOR_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["hop_result"]["triggered"] is True
    assert report["hop_result"]["status"] == "TRIGGERED"
    assert report["role"] == "diagnostic/comparability only"
    assert "define physical takeoff" in report["must_not"]
    assert report["res95_em10_assertion"] == "RES95_EM10_PER_FOOT_BILATERAL_10N_10MS"
    assert report["res95_em10_status"] == "PASS"
    assert report["total_fz_role"] == "REPORT_FIELD_ONLY"
    assert "LEFT_FOOT_FZ" in report["predicate"] and "RIGHT_FOOT_FZ" in report["predicate"]
    for name, entry in report["regression_receipt"].items():
        assert entry["status"] == "PASS", name


def test_per_foot_comparator_regression_cases():
    """RES-84A erratum 1 regression cases A-E."""
    # A: left = 8 N, right = 8 N, total = 16 N -> TRUE (a total comparator would fail)
    assert M.bilateral_per_foot_below_threshold(8.0, 8.0) is True
    # B: left = 12 N, right = 0 N -> FALSE
    assert M.bilateral_per_foot_below_threshold(12.0, 0.0) is False

    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    base = frames[0]

    def synth(i: int, left_fz: float, right_fz: float, nonplantar: int = 0) -> M.V3NativeFrame:
        return dataclasses.replace(
            base, index=i, time_s=i * M.NATIVE_DT_S,
            left_foot_force_world_n=(0.0, 0.0, left_fz),
            right_foot_force_world_n=(0.0, 0.0, right_fz),
            total_floor_force_world_n=(0.0, 0.0, left_fz + right_fz),
            nonplantar_floor_active=nonplantar)

    # C: left = 0 N, right = 0 N, prohibited floor support active -> INVALID / FALSE
    c_result = M.force_takeoff_comparator(
        [synth(i, 0.0, 0.0, 1 if i == 8 else 0) for i in range(20)], occurrence)
    assert c_result.status == "INVALID"
    assert c_result.invalid is True
    assert c_result.triggered is False
    assert c_result.comparator_time_s is None
    assert "non-plantar" in c_result.invalidation_reason

    # D: 4 true samples (first-to-last 0.006 s) -> FALSE
    d_result = M.force_takeoff_comparator(
        [synth(i, 3.0, 3.0) if 4 <= i <= 7 else synth(i, 400.0, 400.0) for i in range(20)], occurrence)
    assert d_result.status == "NOT_TRIGGERED"
    assert d_result.triggered is False
    assert d_result.invalid is False

    # E: exactly K_D = 5 intervals / 6 true samples (elapsed 0.010 s) -> TRUE
    # confirmed at the confirmation sample (onset + K_D), not at the onset
    e_result = M.force_takeoff_comparator(
        [synth(i, 3.0, 3.0) if 4 <= i <= 9 else synth(i, 400.0, 400.0) for i in range(20)], occurrence)
    assert e_result.status == "TRIGGERED"
    assert e_result.triggered is True
    assert e_result.invalid is False
    assert e_result.k_d == 5
    assert e_result.required_true_samples == 6
    assert e_result.onset_time_s == 4 * M.NATIVE_DT_S
    assert abs(e_result.comparator_time_s - 9 * M.NATIVE_DT_S) < 1e-15
    assert e_result.elapsed_duration_s == 0.010
    assert e_result.confirmation_time_s > e_result.onset_time_s
    assert e_result.onset_offset_s is not None and e_result.confirmation_offset_s is not None
    assert e_result.onset_offset_s != e_result.confirmation_offset_s

    # F: K_D - 1 = 4 intervals / 5 true samples (elapsed 0.008 s) -> FALSE
    f_result = M.force_takeoff_comparator(
        [synth(i, 3.0, 3.0) if 4 <= i <= 8 else synth(i, 400.0, 400.0) for i in range(20)], occurrence)
    assert f_result.status == "NOT_TRIGGERED"
    assert f_result.triggered is False

    # single false sample at the final required sample resets the run
    gap_result = M.force_takeoff_comparator(
        [synth(i, 3.0, 3.0) if 4 <= i <= 8 else synth(i, 400.0, 400.0) for i in range(20)], occurrence)
    assert gap_result.status == "NOT_TRIGGERED"
    assert gap_result.k_d == 5 and gap_result.required_true_samples == 6


def test_comparator_offset_reported():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    comparator = M.force_takeoff_comparator(frames, occurrence)
    assert comparator.triggered
    assert comparator.offset_s is not None
    assert abs(comparator.offset_s) <= 0.05
    assert comparator.left_fz_at_trigger_n < M.COMPARATOR_FORCE_N
    assert comparator.right_fz_at_trigger_n < M.COMPARATOR_FORCE_N
    assert comparator.total_fz_at_trigger_n is not None
    assert comparator.offset_s == comparator.confirmation_offset_s
    assert comparator.onset_time_s == 0.024
    assert abs(comparator.onset_offset_s + 0.004) < 1e-15
    assert comparator.confirmation_time_s == 0.034
    assert abs(comparator.confirmation_offset_s - 0.006) < 1e-15
    assert comparator.k_d == 5 and comparator.required_true_samples == 6
    assert comparator.elapsed_duration_s == 0.010


def test_comparator_never_shifts_occurrence():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    before = occurrence.occurrence_time_s
    M.force_takeoff_comparator(frames, occurrence)
    after = M.detect_takeoff_occurrence(frames).occurrence_time_s
    assert before == after


# ---------------------------------------------------------------------------
# PHYSICAL_TIME dwell semantics (RES-82 DWELL_SEMANTICS closure)
# ---------------------------------------------------------------------------
def test_physical_time_dwell_audit(audits):
    report = _report(audits, "PHYSICAL_TIME_DWELL_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["authority_id"] == M.V3_MEASUREMENT_AUTHORITY_ID
    assert report["semantics"]["k_d"].startswith("K_D = ceil(D / dt)")
    assert "first" in report["semantics"]["off_grid_onset"].lower()
    assert report["shared_primitive"]["used_by"] == ["force_takeoff_comparator", "confirm_takeoff"]
    boundaries = report["comparator_boundaries_500hz"]
    assert boundaries["K_D"] == 5 and boundaries["required_true_samples"] == 6
    assert boundaries["4_true_samples"]["status"] == "NOT_TRIGGERED"
    assert boundaries["5_true_samples"]["status"] == "NOT_TRIGGERED"
    assert boundaries["6_true_samples"]["status"] == "TRIGGERED"
    assert boundaries["6_true_samples"]["first_to_last_elapsed_s"] == 0.010
    flight = report["flight_boundaries_500hz"]
    assert flight["exactly_k_d_intervals"]["confirmed"] is True
    assert flight["exactly_k_d_intervals"]["confirmation_elapsed_s"] == 0.050
    assert flight["k_d_minus_1_intervals"]["confirmed"] is False
    assert flight["k_d_minus_1_intervals"]["reason"] == M.DWELL_REJECT_COVERAGE


def test_physical_time_dwell_primitive_k_d_intervals():
    """K_D intervals and K_D + 1 true samples confirm; K_D samples do not."""
    for dt, k_d in ((0.002, 5), (0.001, 10), (0.0005, 20)):
        times = [j * dt for j in range(k_d + 3)]
        exact = M.physical_time_dwell(
            times, [j <= k_d for j in range(len(times))],
            onset_index=0, onset_time_s=0.0, required_duration_s=0.010, dt_s=dt)
        assert exact.dwell_type == M.DWELL_TYPE_PHYSICAL_TIME
        assert exact.k_d == k_d
        assert exact.confirmation_index == k_d
        assert exact.confirmation_intervals == k_d
        assert exact.true_sample_count == k_d + 1
        assert exact.elapsed_duration_s == 0.010
        assert exact.confirmed is True
        short = M.physical_time_dwell(
            times[: k_d + 1], [True] * k_d + [False],
            onset_index=0, onset_time_s=0.0, required_duration_s=0.010, dt_s=dt)
        assert short.confirmed is False
        assert short.reason == M.DWELL_REJECT_SUSTAIN


def test_physical_time_dwell_flight_boundaries_all_timesteps():
    """Flight dwell 0.050 s: exact K_D intervals eligible, K_D - 1 reject."""
    for dt, k_d in ((0.002, 25), (0.001, 50), (0.0005, 100)):
        onset = 10
        frames = [
            dataclasses.replace(
                B.launch_flight_stream()[2][0], index=i, time_s=i * dt,
                legal_plantar_active=2 if i < onset else 0,
                legal_plantar_detected=2 if i < onset else 0,
                left_clearance_m=1e-4 if i < onset else 5e-3,
                right_clearance_m=1e-4 if i < onset else 5e-3)
            for i in range(onset + k_d + 1)
        ]
        occurrence = M.V3TakeoffOccurrence(
            valid=True, occurrence_time_s=onset * dt, native_index=onset, last_support_index=onset - 1,
            bracket_weight=1.0, interpolated=False,
            com_world_m=(0.0, 0.0, 1.0), com_velocity_world_m_s=(0.0, 0.0, 1.0),
            left_clearance_m=1e-4, right_clearance_m=1e-4, legal_plantar_normal_force_n=0.0,
            total_floor_force_world_n=(0.0, 0.0, 0.0), support_mode_before="BILATERAL",
            prohibited_detected_at_bracket=0, reason="")
        confirmed = M.confirm_takeoff(frames, occurrence, dt_s=dt)
        assert confirmed.confirmed is True
        assert confirmed.confirmation_sample == onset + k_d
        assert confirmed.confirmation_elapsed_s == 0.050
        assert confirmed.dwell.k_d == k_d
        short = M.confirm_takeoff(frames[:-1], occurrence, dt_s=dt)
        assert short.confirmed is False
        assert short.dwell.reason == M.DWELL_REJECT_COVERAGE


def test_confirm_takeoff_offgrid_occurrence():
    """t* = 0.0273 s off the 2 ms grid: first sample >= 0.0773 confirms."""
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    offgrid = dataclasses.replace(occurrence, occurrence_time_s=0.0273,
                                  interpolated=True, bracket_weight=0.65)
    confirmation = M.confirm_takeoff(frames, offgrid)
    assert confirmation.confirmed
    assert confirmation.confirmation_sample == 39
    assert frames[38].time_s < 0.0773 <= frames[39].time_s
    assert abs(confirmation.confirmation_elapsed_s - 0.0507) < 1e-15
    assert confirmation.confirmation_elapsed_s >= M.TAKEOFF_DWELL_S
    assert confirmation.dwell.confirmation_intervals >= 25


def test_confirm_takeoff_truncated_stream_rejects_early_allowance():
    """A stream ending at 0.076 s must never confirm a 0.078 s requirement."""
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    truncated = frames[:39]
    assert truncated[-1].time_s == 0.076
    confirmation = M.confirm_takeoff(truncated, occurrence)
    assert confirmation.confirmed is False
    assert confirmation.dwell.reason == M.DWELL_REJECT_COVERAGE
    assert confirmation.confirmation_sample is None
    assert confirmation.window_end_time_s is None
    assert "dwell_coverage" in confirmation.failed_checks


def test_confirm_takeoff_recontact_at_confirmation_sample():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    reference = M.confirm_takeoff(frames, occurrence)
    j = reference.confirmation_sample
    probe = list(frames[: j + 1])
    probe[j] = dataclasses.replace(probe[j], legal_plantar_active=2, legal_plantar_detected=2)
    rejected = M.confirm_takeoff(probe, occurrence)
    assert rejected.confirmed is False
    assert "no_legal_plantar_recontact" in rejected.failed_checks
    assert rejected.occurrence.occurrence_time_s == occurrence.occurrence_time_s


def test_confirm_takeoff_prohibited_at_confirmation_sample():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    reference = M.confirm_takeoff(frames, occurrence)
    j = reference.confirmation_sample
    probe = list(frames[: j + 1])
    probe[j] = dataclasses.replace(probe[j], prohibited_detected=1, prohibited_active=1,
                                   nonplantar_floor_active=1)
    rejected = M.confirm_takeoff(probe, occurrence)
    assert rejected.confirmed is False
    assert "no_prohibited_contact" in rejected.failed_checks


def test_dwell_primitive_fails_closed_on_nonexact_grid():
    with pytest.raises(M.V3DwellAuthorityError):
        M.physical_time_dwell([0.0, 0.0021], [True, True], onset_index=0, onset_time_s=0.0,
                              required_duration_s=0.010, dt_s=0.002)


def test_confirm_takeoff_reports_confirmation_sample_time_elapsed():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    confirmation = M.confirm_takeoff(frames, occurrence)
    assert confirmation.confirmation_sample == 39
    assert confirmation.confirmation_time_s == 0.078
    assert confirmation.confirmation_elapsed_s == 0.050
    assert confirmation.window_end_time_s == confirmation.confirmation_time_s
    assert confirmation.dwell.required_duration_s == M.TAKEOFF_DWELL_S
    assert confirmation.dwell.elapsed_duration_s >= confirmation.dwell.required_duration_s


def test_apex_h2_audit(audits):
    report = _report(audits, "APEX_H2_MEASUREMENT_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    hop = report["hop_result"]
    assert hop["h2_m"] and hop["h2_m"] > 0.0
    assert abs(hop["h2_m"] - hop["ballistic_height_m"]) < 0.01
    assert report["no_elite_h2_target_created"] is True


def test_apex_time_origin_is_occurrence_not_clearance():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    confirmation = M.confirm_takeoff(frames, occurrence)
    apex = M.detect_apex(frames, confirmation)
    assert apex.evaluable
    assert abs(apex.com_z_takeoff_m - occurrence.com_world_m[2]) < 1e-12


# ---------------------------------------------------------------------------
# sampling / signal processing
# ---------------------------------------------------------------------------
def test_sampling_authority_is_explicit(audits):
    report = _report(audits, "SAMPLING_RESAMPLING_AUTHORITY.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["raw_native_stream"]["frequency_hz"] == 500.0
    assert report["canonical_1000hz_stream"]["status"] == "DERIVED_UPSAMPLED_NOT_EVENT_TRUTH"
    assert report["canonical_1000hz_stream"]["down_sampling_authorized"] is False


def test_sampling_authority_fail_closed_cases(audits):
    """RES-84A erratum 2 frozen sampling cases."""
    report = _report(audits, "SAMPLING_RESAMPLING_AUTHORITY.json")
    cases = report["fail_closed_cases"]
    assert cases["native_dt_gt_canonical"]["canonical_status"] == "DERIVED_UPSAMPLED_NOT_EVENT_TRUTH"
    assert cases["native_dt_gt_canonical"]["generation_authorized"] is True
    assert cases["native_dt_eq_canonical_grid_unverified"]["canonical_status"] == "NATIVE_GRID_COINCIDENCE_UNVERIFIED"
    assert cases["native_dt_eq_canonical_grid_unverified"]["generation_authorized"] is False
    assert cases["native_dt_eq_canonical_grid_coincident"]["canonical_status"] == "NATIVE_1000HZ_EVENT_TRUTH"
    assert cases["native_dt_eq_canonical_grid_coincident"]["generation_authorized"] is True
    assert cases["native_dt_eq_canonical_grid_coincident"]["grid_coincidence"] is True
    downsampling = cases["native_dt_lt_canonical_2000hz_to_1000hz"]
    assert downsampling["canonical_status"] == "DOWNSAMPLING_NOT_AUTHORIZED"
    assert downsampling["canonical_status"] != "RAW_NATIVE_EVENT_TRUTH"
    assert downsampling["generation_authorized"] is False
    assert downsampling["anti_alias_authority_frozen"] is False


def test_sampling_2000hz_must_not_be_raw_native_event_truth():
    """dt = 0.0005 s / 2000 Hz regression (RES-84A erratum 2)."""
    result = M.sampling_authority(0.0005, 2000.0)
    assert result["canonical_status"] != "RAW_NATIVE_EVENT_TRUTH"
    assert result["canonical_status"] == M.CANONICAL_DOWNSAMPLING_STATUS
    assert result["canonical_generation_authorized"] is False


def test_sampling_1000hz_requires_exact_grid_coincidence():
    times = [j * 0.001 for j in range(16)]
    coincident = M.sampling_authority(0.001, 1000.0, sample_times=times)
    assert coincident["canonical_status"] == "NATIVE_1000HZ_EVENT_TRUTH"
    assert coincident["canonical_generation_authorized"] is True
    unverified = M.sampling_authority(0.001, 1000.0)
    assert unverified["canonical_status"] == "NATIVE_GRID_COINCIDENCE_UNVERIFIED"
    assert unverified["canonical_generation_authorized"] is False
    jittered = [j * 0.001 + (3.0e-4 if j == 3 else 0.0) for j in range(16)]
    mismatch = M.sampling_authority(0.001, 1000.0, sample_times=jittered)
    assert mismatch["canonical_status"] == "NATIVE_GRID_MISMATCH_NOT_EVENT_TRUTH"
    assert mismatch["canonical_generation_authorized"] is False


def test_canonical_generation_fails_closed():
    base = B.launch_flight_stream()[2][0]
    native_2k = [dataclasses.replace(base, index=j, time_s=j * 0.0005) for j in range(8)]
    with pytest.raises(M.V3SamplingAuthorityError):
        M.canonical_1000hz_stream(native_2k)
    jittered = [dataclasses.replace(base, index=j, time_s=j * 0.001 + (3.0e-4 if j == 3 else 0.0))
                for j in range(8)]
    with pytest.raises(M.V3SamplingAuthorityError):
        M.canonical_1000hz_stream(jittered)
    native_1k = [dataclasses.replace(base, index=j, time_s=j * 0.001) for j in range(8)]
    stream = M.canonical_1000hz_stream(native_1k)
    assert stream.status == "NATIVE_1000HZ_EVENT_TRUTH"
    assert stream.sample_count == len(native_1k)
    assert all(s.native_index0 == s.native_index1 == j and s.weight == 0.0
               for j, s in enumerate(stream.samples))


def test_canonical_stream_provenance():
    plant, data, frames = B.launch_flight_stream(n_samples=20)
    stream = M.canonical_1000hz_stream(frames)
    assert stream.status == "DERIVED_UPSAMPLED_NOT_EVENT_TRUTH"
    assert stream.native_frequency_hz == 500.0
    for sample in stream.samples:
        assert 0 <= sample.native_index0 <= sample.native_index1 < len(frames)
        assert 0.0 <= sample.weight <= 1.0
    # exact midpoint interpolation
    mid = stream.samples[1]
    assert mid.native_index0 == 0 and mid.native_index1 == 1 and abs(mid.weight - 0.5) < 1e-12
    assert abs(mid.com_world_m[2] - 0.5 * (frames[0].com_world_m[2] + frames[1].com_world_m[2])) < 1e-12


def test_signal_processing_authority_has_no_hidden_filter(audits):
    report = _report(audits, "SIGNAL_PROCESSING_AUTHORITY.json")
    assert report["status"] == "PASS", _failed(report)
    policy = report["policy"]
    assert policy["filter_family"] == "NONE"
    assert all(value == "NONE" for value in policy["hidden_smoothing_before"].values())
    assert policy["impulse_integration_rule"] == M.IMPULSE_INTEGRATION_RULE


def test_impulse_rule_is_integrator_consistent():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    left = M.vertical_impulse_between(frames, 0, occurrence.native_index)
    trapezoid = M.vertical_impulse_between(frames, 0, occurrence.native_index, rule="TRAPEZOIDAL")
    dv = occurrence.com_velocity_world_m_s[2] - frames[0].com_velocity_world_m_s[2]
    assert abs(left / M.SYSTEM_MASS_KG - dv) < 1e-3
    assert abs(trapezoid / M.SYSTEM_MASS_KG - dv) > abs(left / M.SYSTEM_MASS_KG - dv)


# ---------------------------------------------------------------------------
# contact parameter authority
# ---------------------------------------------------------------------------
def test_contact_parameter_authority(audits):
    report = _report(audits, "CONTACT_PARAMETER_AUTHORITY.json")
    assert report["status"] == "PASS", _failed(report)
    decisions = report["sealed_decisions"]
    assert decisions["condim"]["nominal_plantar_floor"] == 4
    assert decisions["condim"]["semantics"].startswith("condim 3")
    assert decisions["sliding_friction"]["sensitivity_domain"] == [0.5, 1.5]
    assert decisions["torsional_friction"]["declared"] == 0.0
    assert decisions["rolling_friction"]["declared"] == 0.0


def test_friction_sensitivity_executed():
    report = B.contact_parameter_authority_audit()
    sensitivity = report["sensitivity_execution"]
    assert set(sensitivity) == {"0.5", "0.9", "1.5"}
    assert sensitivity["0.5"]["max_friction_utilization"] >= 0.99
    assert sensitivity["1.5"]["max_friction_utilization"] < 0.99


# ---------------------------------------------------------------------------
# out-of-plane
# ---------------------------------------------------------------------------
def test_out_of_plane_observables(audits):
    report = _report(audits, "OUT_OF_PLANE_REACTION_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["status"] != "PASS_THRESHOLD"
    flat = report["observables"]["flat"]
    assert abs(flat["total_fy_n"]) < 1e-9
    assert flat["total_mx_nm"] is not None


# ---------------------------------------------------------------------------
# force <-> COM consistency
# ---------------------------------------------------------------------------
def test_force_com_consistency_audit(audits):
    report = _report(audits, "FORCE_COM_CONSISTENCY_AUDIT.json")
    assert report["status"] == "PASS", _failed(report)
    results = report["results"]
    assert results["flight_window_max_residual_n"] < 1e-5 * 99.0 * C.V3_GRAVITY_M_S2
    assert results["landing_window_max_residual_n"] < 0.05 * 99.0 * C.V3_GRAVITY_M_S2
    assert results["trapezoid_form_max_residual_n"] > results["launch_window_max_residual_n"]


# ---------------------------------------------------------------------------
# conformance / red team / validation report
# ---------------------------------------------------------------------------
def test_authority_conformance_matrix(audits):
    report = _report(audits, "AUTHORITY_CONFORMANCE_MATRIX.json")
    assert report["status"] == "PASS", _failed(report)
    assert report["zero_unresolved_current_patterns"] is True
    assert len(report["conformance"]) == 24
    assert report["frozen_assertions"]["RES95_EM10_PER_FOOT_BILATERAL_10N_10MS"] == "PASS"
    assert report["frozen_assertions"]["SAMPLING_FAIL_CLOSED_1000HZ_CANONICAL"] == "PASS"
    assert report["frozen_assertions"]["RES82_PHYSICAL_TIME_DWELL_SEMANTICS"] == "PASS"
    assert report["frozen_assertions"]["MUJOCO_CONTACT_SEMANTICS_VERSION"] == "3.8.0"


def test_red_team_patterns_absent():
    report = B.authority_conformance_matrix()
    for name, entry in report["red_team_scan"].items():
        assert entry["present"] is False, f"{name}: {entry['pattern']}"


def test_validation_report_all_gates_pass(audits):
    report = B.measurement_validation_report(audits)
    assert report["status"] == "PASS", report["failed_checks"]
    assert report["artifact_count"] == 21
    assert not report["failed_checks"]


def test_receipt_count_is_generated_from_validation_report(audits):
    """The receipt aggregate must never be a stale handwritten literal."""
    import re
    report = B.measurement_validation_report(audits)
    receipt = (EVIDENCE_DIR / "RES84_RECEIPT.md").read_text()
    aggregates = re.findall(r"(\d+) checks, (\d+) failed", receipt)
    assert aggregates, "receipt must display a generated aggregate"
    expected = (report["total_checks"], len(report["failed_checks"]))
    assert all((int(total), int(failed)) == expected for total, failed in aggregates)


def test_evidence_artifacts_present():
    for name in ("MEASUREMENT_IMPLEMENTATION_SPEC.json", "CONTACT_SEMANTICS_AUDIT.json",
                 "CONTACT_FORCE_FRAME_AUDIT.json", "GROUND_WRENCH_AUDIT.json", "SYSTEM_COM_AUDIT.json",
                 "COP_AUTHORITY_AUDIT.json", "ACTIVE_SUPPORT_HULL_AUDIT.json", "FOOT_CLEARANCE_AUDIT.json",
                 "TAKEOFF_OCCURRENCE_CONFIRMATION_AUDIT.json", "CLEARANCE_GUARD_AUDIT.json",
                 "FORCE_THRESHOLD_COMPARATOR_AUDIT.json", "PHYSICAL_TIME_DWELL_AUDIT.json",
                 "APEX_H2_MEASUREMENT_AUDIT.json",
                 "SAMPLING_RESAMPLING_AUTHORITY.json", "SIGNAL_PROCESSING_AUTHORITY.json",
                 "ORIENTATION_AUDIT.json", "PROHIBITED_CONTACT_AUDIT.json", "CONTACT_PARAMETER_AUTHORITY.json",
                 "OUT_OF_PLANE_REACTION_AUDIT.json", "FORCE_COM_CONSISTENCY_AUDIT.json",
                 "AUTHORITY_CONFORMANCE_MATRIX.json", "MEASUREMENT_VALIDATION_REPORT.json",
                 "HASH_MANIFEST.json", "RES84_RECEIPT.md", "build_evidence.py", "seal_evidence.py"):
        assert (EVIDENCE_DIR / name).is_file(), name


def test_no_elite_h2_or_controller_surface():
    module = sys.modules[M.__name__]
    assert not hasattr(module, "H2_TARGET")
    assert not any("controller" in name.lower() for name in dir(module))
    assert math.isclose(M.CLEARANCE_GUARD_M, 0.002)
