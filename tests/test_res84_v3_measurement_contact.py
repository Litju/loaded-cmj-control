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
    assert M.NATIVE_DT_S == 0.002 and M.NATIVE_FREQUENCY_HZ == 500.0
    assert M.REQUIRED_CANDIDATE_MAX_DT_S <= 0.001
    assert M.DOWN_SAMPLING_AUTHORIZED is False


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
    synthetic.append(dataclasses.replace(synthetic[-1], legal_plantar_active=2, legal_plantar_detected=2))
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
            legal_ground_moment_world_nm=(0.0, 0.0, 0.0), cop_validity="VALID", cop_x_m=0.0,
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
    assert report["role"] == "diagnostic/comparability only"
    assert "define physical takeoff" in report["must_not"]


def test_comparator_offset_reported():
    plant, data, frames = B.launch_flight_stream()
    occurrence = M.detect_takeoff_occurrence(frames)
    comparator = M.force_takeoff_comparator(frames, occurrence)
    assert comparator.triggered
    assert comparator.offset_s is not None
    assert abs(comparator.offset_s) <= 0.05


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
    assert len(report["conformance"]) == 19


def test_red_team_patterns_absent():
    report = B.authority_conformance_matrix()
    for name, entry in report["red_team_scan"].items():
        assert entry["present"] is False, f"{name}: {entry['pattern']}"


def test_validation_report_all_gates_pass(audits):
    report = B.measurement_validation_report(audits)
    assert report["status"] == "PASS", report["failed_checks"]
    assert report["artifact_count"] == 20
    assert not report["failed_checks"]


def test_evidence_artifacts_present():
    for name in ("MEASUREMENT_IMPLEMENTATION_SPEC.json", "CONTACT_SEMANTICS_AUDIT.json",
                 "CONTACT_FORCE_FRAME_AUDIT.json", "GROUND_WRENCH_AUDIT.json", "SYSTEM_COM_AUDIT.json",
                 "COP_AUTHORITY_AUDIT.json", "ACTIVE_SUPPORT_HULL_AUDIT.json", "FOOT_CLEARANCE_AUDIT.json",
                 "TAKEOFF_OCCURRENCE_CONFIRMATION_AUDIT.json", "CLEARANCE_GUARD_AUDIT.json",
                 "FORCE_THRESHOLD_COMPARATOR_AUDIT.json", "APEX_H2_MEASUREMENT_AUDIT.json",
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
