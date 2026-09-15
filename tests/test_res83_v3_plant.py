"""RES-83 mandatory tests — elite-soccer human-valid loaded-CMJ Plant V3.

Authority: LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1
Mission:   RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001

These are the 25 mandatory RES-83 Plant tests.  Every test asserts the same
deterministic audit computation that is archived in
audit/EXP-RES83-ELITE-SOCCER-HUMAN-VALID-PLANT-001/, so the executed proof
and the sealed evidence are one computation.
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
EVIDENCE_DIR = TASK_ROOT / "audit" / "EXP-RES83-ELITE-SOCCER-HUMAN-VALID-PLANT-001"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _load_builder():
    spec = importlib.util.spec_from_file_location(
        "res83_build_evidence", EVIDENCE_DIR / "build_evidence.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


B = _load_builder()

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3 import plant as P  # noqa: E402


@pytest.fixture(scope="session")
def audits():
    return B.build_all()


def _report(audits, name):
    return audits[name]


def _failed(report):
    return [c for c in report.get("checks", []) if not c["pass"]]


# 1 -------------------------------------------------------------------------
def test_xml_compile_load(audits):
    model = P.build_model()
    assert model.nbody == 14
    assert _report(audits, "MODEL_INTROSPECTION.json")["status"] == "PASS"
    assert "<mujoco model=\"loaded-cmj-20kg-elite-soccer-v3\">" in P.model_xml()


# 2 -------------------------------------------------------------------------
def test_runtime_introspection_dimensions(audits):
    report = _report(audits, "MODEL_INTROSPECTION.json")
    counts = report["counts"]
    assert counts["nbody"] == 14
    assert counts["nq"] == 12
    assert counts["nv"] == 12
    assert counts["nu"] == 9
    assert counts["njnt"] == 12
    assert counts["neq"] == 0
    assert report["status"] == "PASS"


# 3 -------------------------------------------------------------------------
def test_athlete_mass_79kg(audits):
    report = _report(audits, "MASS_INERTIA_AUDIT.json")
    assert report["status"] == "PASS"
    assert abs(report["athlete_mass_kg"] - 79.0) <= 1e-9


# 4 -------------------------------------------------------------------------
def test_system_mass_99kg(audits):
    report = _report(audits, "MASS_INERTIA_AUDIT.json")
    assert abs(report["system_mass_kg"] - 99.0) <= 1e-9
    assert report["status"] == "PASS"


# 5 -------------------------------------------------------------------------
def test_segment_mass_inertia_audit(audits):
    report = _report(audits, "MASS_INERTIA_AUDIT.json")
    assert report["status"] == "PASS"
    names = {row["name"] for row in report["body_table"]}
    assert {"pelvis", "HAT", "bar", "left_thigh", "left_shank", "left_hindfoot",
            "left_forefoot", "left_toe", "right_thigh", "right_shank",
            "right_hindfoot", "right_forefoot", "right_toe"} <= names
    checks = {c["check"] for c in report["checks"]}
    for body in ("pelvis", "left_thigh", "left_shank", "left_hindfoot",
                 "left_forefoot", "left_toe"):
        assert f"mass:{body}" in checks
        assert f"diaginertia:{body}" in checks


# 6 -------------------------------------------------------------------------
def test_hat_bar_com_inertia_audit(audits):
    report = _report(audits, "HAT_BAR_REALIZATION_AUDIT.json")
    assert report["status"] == "PASS"
    assert abs(report["hat_realized"]["mass_kg"] - 38.7969) <= 1e-9
    assert abs(report["bar_realized"]["mass_kg"] - 20.0) <= 1e-9
    system = report["hat_bar_system_realized"]
    assert abs(system["mass_kg"] - 58.7969) <= 1e-9
    assert np.allclose(system["com_m"], C.V3_HAT_BAR_SYSTEM["com_m"], atol=1e-12, rtol=0.0)
    assert np.allclose(system["inertia_about_com_kg_m2"],
                       C.V3_HAT_BAR_SYSTEM["inertia_about_com_kg_m2"], atol=1e-12, rtol=0.0)
    # Ixz is preserved and the bar is not the uniform-rod surrogate
    assert abs(report["hat_realized"]["full_inertia_about_com_kg_m2"][0][2] + 0.02077944879714553) <= 1e-12
    assert abs(system["inertia_about_com_kg_m2"][0][2] - 0.1360143545930838) <= 1e-12
    assert abs(report["bar_realized"]["diaginertia_kg_m2"][0] - 11.75585696777048) <= 1e-9


# 7 -------------------------------------------------------------------------
def test_joint_address_order_sign_range_audit(audits):
    report = _report(audits, "JOINT_FK_SIGN_AUDIT.json")
    assert report["status"] == "PASS"
    order = report["joint_order"]
    assert order == list(C.V3_JOINT_NAMES)
    assert report["qpos_addresses"] == {n: i for i, n in enumerate(C.V3_JOINT_NAMES)}
    for name in C.V3_JOINT_NAMES:
        assert f"axis:{name}" in {c["check"] for c in report["checks"]}
        assert f"range:{name}" in {c["check"] for c in report["checks"]}


# 8 -------------------------------------------------------------------------
def test_deterministic_fk_sign_probes_for_every_joint(audits):
    report = _report(audits, "JOINT_FK_SIGN_AUDIT.json")
    probes = {p["joint"]: p for p in report["fk_probes"]}
    # every joint has a deterministic probe with correct sign and exact FK match
    expected = {"root_tx", "root_tz", "root_ry", "trunk_pelvis", "left_hip",
                "left_knee", "left_ankle", "left_mtp"}
    assert expected <= set(probes)
    for name in ("root_tx", "root_tz", "root_ry", "trunk_pelvis", "left_hip",
                 "left_knee", "left_ankle", "left_mtp"):
        assert probes[name]["sign_ok"], name
        assert probes[name]["max_abs_error_m"] <= 1e-9, name
    # JC-09 qualitative signs
    root_ry = probes["root_ry"]["points"]["bar_origin"]["measured_delta_m"]
    assert root_ry[0] > 0.0
    trunk = probes["trunk_pelvis"]["points"]["head"]["measured_delta_m"]
    assert trunk[0] > 0.0
    hip = probes["left_hip"]["points"]["knee_origin"]["measured_delta_m"]
    assert hip[0] > 0.0
    knee = probes["left_knee"]["points"]["ankle_origin"]["measured_delta_m"]
    assert knee[0] < 0.0
    ankle = probes["left_ankle"]["points"]["toe_patch"]["measured_delta_m"]
    assert ankle[2] > 0.0
    mtp = probes["left_mtp"]["points"]["toe_patch"]["measured_delta_m"]
    assert mtp[2] > 0.0


# 9 -------------------------------------------------------------------------
def test_planar_root_passivity_no_hidden_support(audits):
    report = _report(audits, "ROOT_PASSIVITY_AUDIT.json")
    assert report["status"] == "PASS"
    for name, attrs in report["root_joint_attributes"].items():
        assert attrs["limited"] is False
        assert attrs["stiffness"] == 0.0
        assert attrs["damping"] == 0.0
        assert attrs["armature"] == 0.0
    checks = {c["check"]: c for c in report["checks"]}
    assert checks["no_equality_constraints"]["pass"]
    assert checks["no_tendons"]["pass"]
    assert checks["flight_qacc_tx_zero"]["pass"]
    assert checks["flight_qacc_ry_zero"]["pass"]
    assert checks["ballistic_z"]["pass"]
    assert checks["flight_constraint_forces_zero"]["pass"]


# 10 ------------------------------------------------------------------------
def test_reverse_knee_geometry_structurally_impossible(audits):
    report = _report(audits, "ROM_REACHABILITY_AUDIT.json")
    assert report["status"] == "PASS"
    assert report["reverse_knee"]["active_limit_constraints"] > 0
    checks = {c["check"]: c for c in report["checks"]}
    assert checks["reverse_knee_limit_active"]["pass"]
    assert checks["knee_no_hyperextension_range"]["pass"]


# 11-19 ---------------------------------------------------------------------
def _pose(audits, name):
    report = _report(audits, "REPRESENTATIVE_POSE_AUDIT.json")
    assert report["status"] == "PASS"
    poses = {p["name"]: p for p in report["poses"]}
    assert name in poses
    pose = poses[name]
    assert pose["joint_limit_violations"] == []
    assert pose["prohibited_contacts"] == []
    assert pose["self_contacts"] == []
    return pose


def test_legal_standing_pose_reachable(audits):
    pose = _pose(audits, "legal_standing")
    assert abs(pose["support_min_z_m"]) <= 1e-9
    assert pose["com_over_support"] is True
    assert pose["qpos_rad"]["root_tz"] == pytest.approx(0.9740179999999999, abs=1e-9)
    assert pose["qpos_rad"]["left_knee"] == 0.0
    assert pose["qpos_rad"]["left_ankle"] == 0.0


def test_deep_legal_countermovement_pose_reachable(audits):
    pose = _pose(audits, "deep_legal_countermovement")
    assert pose["qpos_rad"]["left_knee"] >= math.radians(100.0)
    assert pose["qpos_rad"]["left_knee"] <= math.radians(140.0)
    assert pose["qpos_rad"]["left_hip"] >= math.radians(80.0)
    assert pose["support_min_z_m"] <= 1e-9


def test_braking_propulsion_configuration_reachable(audits):
    braking = _pose(audits, "braking")
    propulsion = _pose(audits, "propulsion")
    assert braking["qpos_rad"]["left_ankle"] > 0.0  # dorsiflexed braking
    assert propulsion["qpos_rad"]["left_ankle"] < 0.0  # plantarflexed propulsion
    assert propulsion["qpos_rad"]["left_mtp"] > 0.0


def test_heel_rise_toe_rocker_pose_reachable(audits):
    pose = _pose(audits, "heel_rise_toe_rocker")
    assert pose["qpos_rad"]["left_mtp"] >= math.radians(39.0)
    assert pose["qpos_rad"]["left_ankle"] < 0.0
    assert all(c["region"] != "heel" for c in pose["contacts"]
               if c["kind"] == "legal_plantar_floor")


def test_toe_off_configuration_reachable(audits):
    pose = _pose(audits, "toe_off")
    assert pose["qpos_rad"]["left_mtp"] >= math.radians(49.0)
    assert pose["qpos_rad"]["left_ankle"] <= math.radians(-40.0)
    assert all(c["region"] == "toe" for c in pose["contacts"]
               if c["kind"] == "legal_plantar_floor")


def test_flight_configuration_reachable(audits):
    pose = _pose(audits, "flight")
    assert pose["support_min_z_m"] >= 0.079
    assert pose["contact_counts"] == {}


def test_toe_forefoot_first_touchdown_configuration_reachable(audits):
    pose = _pose(audits, "toe_forefoot_first_touchdown")
    assert pose["qpos_rad"]["left_ankle"] < 0.0
    regions = {c["region"] for c in pose["contacts"] if c["kind"] == "legal_plantar_floor"}
    assert regions  # forefoot/toe contact established
    assert all(r in ("forefoot", "toe") for r in regions)


def test_landing_absorption_pose_reachable(audits):
    pose = _pose(audits, "landing_absorption")
    assert pose["qpos_rad"]["left_knee"] >= math.radians(90.0)
    assert abs(pose["support_min_z_m"]) <= 1e-9


def test_recovered_standing_pose_reachable(audits):
    pose = _pose(audits, "recovered_standing")
    assert abs(pose["support_min_z_m"]) <= 1e-9
    assert pose["com_over_support"] is True


# 20 ------------------------------------------------------------------------
def test_deliberate_fall_prohibited_contact_probe(audits):
    report = _report(audits, "COLLISION_MATRIX.json")
    assert report["status"] == "PASS"
    probes = {p["body"]: p for p in report["prohibited_floor_probes"]}
    for body in C.V3_PROHIBITED_FLOOR_BODIES:
        assert probes[body]["contact"] is True, body
        assert probes[body]["hit_geoms"], body


# 21 ------------------------------------------------------------------------
def test_representative_pose_self_intersection_audit(audits):
    report = _report(audits, "REPRESENTATIVE_POSE_AUDIT.json")
    assert report["status"] == "PASS"
    assert report["pose_count"] >= 9
    for pose in report["poses"]:
        assert pose["self_contacts"] == [], pose["name"]
        assert pose["prohibited_contacts"] == [], pose["name"]


# 22 ------------------------------------------------------------------------
def test_collision_pair_matrix_audit(audits):
    report = _report(audits, "COLLISION_MATRIX.json")
    assert report["status"] == "PASS"
    checks = {c["check"]: c for c in report["checks"]}
    assert checks["declared_matches_empirical"]["pass"]
    # opposite-foot, bar/lower-limb and non-adjacent classes must not disappear
    for label, check in checks.items():
        if label.startswith(("opposite_foot:", "bar_vs_lower_limb", "nonadjacent:")):
            assert check["pass"], label
    # explicit excludes exactly the authority pairs
    assert len(report["excluded_pairs"]) == 12
    disabled = {(row["body1"], row["body2"]) for row in report["matrix"] if not row["enabled"]}
    for pair in C.V3_EXCLUDED_BODY_PAIRS:
        assert pair in disabled, pair


# 23 ------------------------------------------------------------------------
def test_plantar_region_identity_contact_audit(audits):
    report = _report(audits, "PLANTAR_REGION_IDENTITY_AUDIT.json")
    assert report["status"] == "PASS"
    regions = {(r["side"], r["region"]) for r in report["regions"]}
    assert regions == {(s, r) for s in C.V3_SIDES for r in C.V3_SUPPORT_REGIONS}
    checks = {c["check"]: c for c in report["checks"]}
    for side in C.V3_SIDES:
        for region in C.V3_SUPPORT_REGIONS:
            assert checks[f"contact_registration:{side}:{region}"]["pass"]
            assert checks[f"sole_plane:{side}:{region}"]["pass"]


# 24 ------------------------------------------------------------------------
def test_no_r001_v2_controller_or_trajectory_assumptions_imported(audits):
    report = _report(audits, "IMPORT_BOUNDARY_AUDIT.json")
    assert report["status"] == "PASS"
    for module in report["modules"]:
        imports = module["imports"]
        for forbidden in ("loaded_cmj.v2", "loaded_cmj.simulation", "loaded_cmj.control",
                          "loaded_cmj.runtime", "loaded_cmj.oracle", "loaded_cmj.biomechanics"):
            assert not any(imp == forbidden or imp.startswith(forbidden + ".")
                           for imp in imports), (module["file"], forbidden)
    # V3 Plant code needs no controller/runtime module (fresh interpreter with a
    # stub root package, so the pre-existing eager root __init__ cannot mask it)
    independence = report["plant_code_level_independence"]
    assert independence["pass"], independence
    assert independence["forbidden_loaded"] == []
    assert all(m.startswith("loaded_cmj.v3") or m == "loaded_cmj"
               for m in independence["modules_loaded"])


# 25 ------------------------------------------------------------------------
def test_no_final_res84_res85_constants_introduced(audits):
    report = _report(audits, "FINAL_CONSTANT_AUDIT.json")
    assert report["status"] == "PASS"
    checks = {c["check"]: c for c in report["checks"]}
    for key in ("xml_no_attribute:solref", "xml_no_attribute:solimp", "xml_no_attribute:margin",
                "xml_no_attribute:gap", "xml_no_attribute:forcerange", "xml_no_attribute:ctrlrange",
                "xml_all_limits_disabled", "xml_only_mtp_nonzero_passive", "xml_no_equality",
                "xml_no_tendon", "provisional_registry_present", "zero_passive_model_representable"):
        assert checks[key]["pass"], key


# supplementary ------------------------------------------------------------
def test_authority_bundle_integrity(audits):
    report = _report(audits, "AUTHORITY_BUNDLE_INTEGRITY.json")
    assert report["status"] == "PASS"
    assert report["file_count"] >= 30
    assert report["authority_id"] == C.V3_AUTHORITY_ID


def test_authority_conformance_matrix(audits):
    report = _report(audits, "AUTHORITY_CONFORMANCE_MATRIX.json")
    assert report["status"] == "PASS"
    decisions = {row["decision"] for row in report["rows"]}
    for required in ("PT-01", "PT-03", "PT-04", "JC-02", "JC-05", "JC-08", "CC-01", "CC-02",
                     "CC-03", "FM-09", "UB-05", "LB-08", "AB-04", "AB-13"):
        assert required in decisions, required


def test_system_iyy_sensitivity_range_matches_res95(audits):
    """Regression: the declared system Iyy sensitivity range is the sealed
    RES-95 value, recomputed by the builder from the authority's own cases."""
    report = _report(audits, "HAT_BAR_REALIZATION_AUDIT.json")
    checks = {c["check"]: c for c in report["checks"]}
    check = checks["system_iyy_sensitivity_range_matches_res95"]
    assert check["pass"], check
    assert check["declared"] == [1.7316734138069028, 2.0243773264768694]
    assert check["authority_declared"] == [1.7316734138069028, 2.0243773264768694]
    assert check["authority_recomputed_from_sensitivity_cases"] == \
        [1.7316734138069028, 2.0243773264768694]
    declared = report["hat_bar_system_realized"]["declared_sensitivity_iyy_range_kg_m2"]
    assert declared == [1.7316734138069028, 2.0243773264768694]


def test_hat_bar_inertia_matches_res95_derived(audits):
    """Regression: realized HAT and bar inertias are anchored to the sealed
    DERIVED_QUANTITIES.json values, not only to transcribed constants."""
    report = _report(audits, "HAT_BAR_REALIZATION_AUDIT.json")
    checks = {c["check"]: c for c in report["checks"]}
    assert checks["hat_full_inertia_matches_res95_derived"]["pass"]
    assert checks["bar_inertia_matches_res95_derived"]["pass"]


def test_mtp_passive_prior_and_active_channel(audits):
    plant = P.V3Plant()
    assert len(C.V3_MTP_JOINT_NAMES) == 2
    for name in C.V3_MTP_JOINT_NAMES:
        jid = plant.idx.joint[name]
        assert plant.model.jnt_stiffness[jid] == 25.0
        assert plant.model.dof_damping[int(plant.model.jnt_dofadr[jid])] == 2.0
        assert plant.model.jnt_range[jid][0] == pytest.approx(-0.523599, abs=1e-9)
        assert plant.model.jnt_range[jid][1] == pytest.approx(1.570796, abs=1e-9)
    # active channel present in parallel, zeroed, and bounded only by RES-85 later
    assert plant.model.nu == 9
    for aid in range(plant.model.nu):
        assert plant.model.actuator_ctrllimited[aid] == 0
        assert plant.model.actuator_forcelimited[aid] == 0
    zero_model = P.build_zero_passive_model()
    zero_plant = P.V3Plant(zero_model)
    assert zero_model.nu == 9
    for name in C.V3_MTP_JOINT_NAMES:
        jid = zero_plant.idx.joint[name]
        assert zero_model.jnt_stiffness[jid] == 0.0
        assert zero_model.dof_damping[int(zero_model.jnt_dofadr[jid])] == 0.0


def test_validation_report_all_gates_pass(audits):
    report = _report(audits, "PLANT_VALIDATION_REPORT.json") \
        if "PLANT_VALIDATION_REPORT.json" in audits else B.validation_report(audits)
    assert report["status"] == "PASS"
    assert report["claims_total"] >= 13
    for claim in report["claims"]:
        assert claim["status"] == "PASS", claim["artifact"]
        assert claim["checks_failed"] == [], claim["artifact"]
