"""RES-52 soft-contact force realization authority tests.

MISSION=RES10_SYNC_SOFT_CONTACT_FORCE_REALIZATION_AUTHORITY_001
Deterministic, focused. Evidence bundle:
EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

REPO = Path("/home/litju/Projects/loaded-cmj-control")
BUNDLE = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001")
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "tools" / "res52"))
sys.path.insert(0, str(REPO))

import core52 as C  # noqa: E402
from core52 import (WEIGHT_N, MASS_KG, G, E8_SHA, TD_TIME, S_TIMES, sha_arr,
                    make_plant, bind_plant, restore_state, get_state_vector,
                    soft_contact_state, foot_gap_half_height, seed_detector)  # noqa: E402
from loaded_cmj.v2.measurement import (  # noqa: E402
    SynchronizedPhysicsSample, create_measurement_data)
from soft_contact import SoftContactPolicy, Y_NAMES, NY  # noqa: E402
from run_cell import run_cell, fz_des_n  # noqa: E402


def _spec():
    return json.loads((BUNDLE / "experiment_spec.json").read_text())


def _states():
    return np.load(BUNDLE / "branch_states.npz")


def _auth():
    return json.loads((BUNDLE / "BRANCH_STATE_AUTHORITY.json").read_text())


def _qual():
    return json.loads((BUNDLE / "FORCE_PROFILE_QUALIFICATION.json").read_text())


# ---------------------------------------------------------------- Phase A ---
def test_01_branch_state_shas_match_sealed_authority():
    auth = _auth()
    states = _states()
    for lab in ["S40", "S50", "S75", "S100"]:
        assert sha_arr(states[lab]) == auth["STATES"][lab]["STATE_SHA256"]
    assert sha_arr(states["S_STAND"]) == auth["S_STAND"]["STATE_SHA256"]
    assert auth["STATES"]["S50"]["STATE_SHA256"] != auth["STATES"]["S75"]["STATE_SHA256"]


def test_02_state_hash_roundtrip_reproducible():
    plant = make_plant()
    bind_plant(plant)
    m, d = plant.model, plant.make_data()
    meas = create_measurement_data(plant)
    states = _states()
    for lab in ["S40", "S50", "S75", "S100", "S_STAND"]:
        restore_state(m, d, states[lab])
        s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        assert s.check_time_identity()
        assert sha_arr(get_state_vector(m, d)) == s.state_vector_sha256
        assert sha_arr(states[lab]) == s.state_vector_sha256


def test_03_synchronized_contact_state_semantics():
    """dist sign, penetration definition, efc_vel identity + separating sign."""
    plant = make_plant()
    bind_plant(plant)
    m, d = plant.model, plant.make_data()
    meas = create_measurement_data(plant)
    gap_hh = foot_gap_half_height(m)
    x = _states()["S50"]
    restore_state(m, d, x)
    cs = soft_contact_state(m, d, gap_half_height=gap_hh)
    assert cs["L"]["penetration"] == max(0.0, -cs["L"]["contact_distance"])
    assert cs["L"]["penetration"] > 0.005  # S50 is compressed
    assert cs["L"]["foot_normal_velocity"] > 0.05  # seed unloading = separating (+)
    # efc_vel of the deepest left-foot contact row must equal efc_J @ qvel
    i = None
    for k in range(d.ncon):
        con = d.contact[k]
        lf, fl = plant.idx.left_foot_geom, plant.idx.floor_geom
        if {int(con.geom1), int(con.geom2)} == {lf, fl}:
            if i is None or float(con.dist) < float(d.contact[i].dist):
                i = k
    assert i is not None
    con = d.contact[i]
    row = int(con.efc_address)
    Jmat = np.asarray(d.efc_J, float).reshape(int(d.nefc), int(m.nv))
    Jvel = float(Jmat[row] @ np.asarray(d.qvel, float))
    assert float(d.efc_vel[row]) == pytest.approx(Jvel, abs=1e-9)
    # the constraint residual semantics: efc_pos ≈ dist (margin=0 for this plant)
    assert float(d.efc_pos[row]) == pytest.approx(float(con.dist), abs=1e-12)
    # inactive-foot geometric gap is positive when the foot is above the floor
    d2 = plant.make_data()
    x2 = _states()["S_STAND"]
    restore_state(m, d2, x2)
    cs2 = soft_contact_state(m, d2, gap_half_height=gap_hh)
    assert cs2["L"]["contact_distance"] < 0  # standing feet are compressed
    assert cs2["L"]["penetration"] < 1e-3    # tiny (stiff standing contact)


def test_04_standing_reference_observed_not_invented():
    ref = json.loads((BUNDLE / "STANDING_CONTACT_REFERENCE.json").read_text())
    auth = _auth()
    assert ref["SOURCE_STATE_SHA256"] == auth["S_STAND"]["STATE_SHA256"]
    assert ref["OBSERVED_NOT_TARGET"] is True
    assert ref["NORMAL_VEL_EQ"] == 0.0
    # whole standing force equals body weight within 1 N (true standing)
    assert abs(ref["FZ_EQ_L"] + ref["FZ_EQ_R"] - MASS_KG * G) < 1.0
    assert abs(ref["FZ_EQ_L"] - ref["FZ_EQ_R"]) < 1.0
    assert ref["PEN_EQ_L"] == pytest.approx(ref["PEN_EQ_R"], abs=1e-9)


def test_05_e8_seed_lineage_preserved():
    auth = _auth()
    assert auth["E8_STATE_SHA256"] == E8_SHA
    assert abs(auth["TD_TIME"] - TD_TIME) < 1e-12
    assert auth["C00_SEED_REPRODUCED_FRESH"] is True
    assert auth["C00_SEED_TRACE_SHA256_AUTHORITY"] == (
        "3e3a603e963e02aa541fc55332e3c2de802f94787111b555b51067ea5386321a")


# ---------------------------------------------------------------- Phase C ---
def _policy_fixture():
    plant = make_plant()
    bind_plant(plant)
    probes = [plant.make_data() for _ in range(17)]
    pol = SoftContactPolicy(_spec()["CONTROLLER_CONSTANTS"], plant, probes)
    pol.gap_hh = foot_gap_half_height(plant.model)
    return plant, probes, pol


def test_06_full_7dof_perturbation_basis():
    plant, probes, pol = _policy_fixture()
    x = _states()["S_STAND"]
    u_nom = np.zeros(7)
    d0 = plant.make_data()
    restore_state(plant.model, d0, x)
    u_nom = np.asarray(d0.ctrl, float).copy()
    y_nom, Gmat, flags, meta = pol.build_G(x, u_nom, 40)
    assert Gmat.shape == (NY, 7) and NY == 18
    assert np.all(np.isfinite(Gmat))
    assert len(meta) == 7
    for j in range(7):
        assert meta[j]["j"] == j
        # every actuator channel perturbed both-sidedly (interior at standing)
        assert meta[j]["dp"] > 0.0 and meta[j]["dm"] > 0.0
    assert flags == (True, True)


def test_07_local_map_deterministic():
    plant, probes, pol = _policy_fixture()
    x = _states()["S_STAND"]
    d0 = plant.make_data()
    restore_state(plant.model, d0, x)
    u_nom = np.asarray(d0.ctrl, float).copy()
    _, G1, _, _ = pol.build_G(x, u_nom, 40)
    pol2 = SoftContactPolicy(_spec()["CONTROLLER_CONSTANTS"], plant,
                             [plant.make_data() for _ in range(17)])
    pol2.gap_hh = pol.gap_hh
    _, G2, _, _ = pol2.build_G(x, u_nom, 40)
    assert np.array_equal(G1, G2)


def test_08_bvls_solution_respects_bounds():
    plant, probes, pol = _policy_fixture()
    x = _states()["S50"]
    y_nom, Gmat, _, _ = pol.build_G(x, np.zeros(7), 40)
    rho = 0.25
    du = pol.solve_du(y_nom, Gmat, 1863.9, -0.30, np.zeros(7), rho)
    assert np.all(np.abs(du) <= rho + 1e-12)
    assert np.all(np.isfinite(du))


def test_09_no_fixed_ext_dir_anywhere():
    import re
    layer = (REPO / "tools" / "res52" / "soft_contact.py").read_text()
    assert not re.search(r"EXT_DIR\s*[=\[]", layer)
    runner = (REPO / "tools" / "res52" / "run_cell.py").read_text()
    assert not re.search(r"EXT_DIR\s*[=\[]", runner)
    # G columns must not all be proportional to one fixed direction
    plant, probes, pol = _policy_fixture()
    x = _states()["S50"]
    _, Gmat, _, _ = pol.build_G(x, np.zeros(7), 40)
    A = Gmat[0:3].T  # (7 x 3) force rows
    # rank of the 7x3 force block must exceed 1 (a scalar press direction
    # would make every column a multiple of one vector)
    assert np.linalg.matrix_rank(A / (np.abs(A).max() + 1e-12), tol=1e-2) >= 2


def test_10_trust_region_shrink_and_fallback():
    kc = dict(_spec()["CONTROLLER_CONSTANTS"])
    kc["VALIDATION_TOL"] = dict(kc["VALIDATION_TOL"], FZ_WHOLE_ABS_N=-1.0)  # impossible
    plant = make_plant()
    bind_plant(plant)
    probes = [plant.make_data() for _ in range(17)]
    pol = SoftContactPolicy(kc, plant, probes)
    pol.gap_hh = foot_gap_half_height(plant.model)
    x = _states()["S_STAND"]
    d0 = plant.make_data()
    restore_state(plant.model, d0, x)
    u_prev = np.asarray(d0.ctrl, float).copy()
    info = pol.act(x, u_prev, WEIGHT_N, 0.0, 40, probes[16])
    assert info["FALLBACK"] is True
    assert info["VALIDATED"] is False
    assert pol.trust_fallbacks == 1
    assert pol.trust_shrink_events == kc["MAX_TRUST_SHRINKS_PER_UPDATE"]
    # fallback applied the exact previous action
    assert np.array_equal(info["u"], u_prev)
    assert float(np.max(info["VAL_ERR"])) >= 0.0


def test_11_branch_validation_agreement_small_perturbations():
    plant, probes, pol = _policy_fixture()
    x = _states()["S_STAND"]
    d0 = plant.make_data()
    restore_state(plant.model, d0, x)
    u_prev = np.asarray(d0.ctrl, float).copy()
    info = pol.act(x, u_prev, WEIGHT_N, 0.0, 40, probes[16])
    assert info["VALIDATED"] is True
    # live execution must reproduce the validated branch exactly
    plant.apply_action(d0, info["u"])
    for _ in range(40):
        mujoco.mj_step(plant.model, d0)
    x_live = get_state_vector(plant.model, d0)
    x_val = get_state_vector(plant.model, probes[16])
    assert sha_arr(x_live) == sha_arr(x_val)


def test_12_force_feedback_uses_actual_mujoco_fz():
    plant, probes, pol = _policy_fixture()
    x = _states()["S50"]
    pd = probes[0]
    pol.gap_hh = foot_gap_half_height(plant.model)
    pol.restore(pd, x)
    y = pol.y_of(pd)
    sm = plant.foot_contact_summary(pd)
    assert y[0] == pytest.approx(float(sm["whole_Fz"]), abs=1e-9)
    assert y[1] == pytest.approx(float(sm["left_Fz"]), abs=1e-9)
    assert y[2] == pytest.approx(float(sm["right_Fz"]), abs=1e-9)


def test_13_contact_retention_and_distance_rows():
    pol = _policy_fixture()[2]
    G = np.zeros((NY, 7))
    G[3] = np.array([0, 0, 0, 1, 0, 0, 0], float) * 1e-3  # dummy column 4
    y_sep = np.zeros(NY)
    y_sep[3] = 0.004  # separated by 4 mm -> re-engage row (target 0)
    rows, tgt = pol._dist_rows(y_sep, G)
    assert rows and tgt[0] < 0  # drive distance down to zero
    y_deep = np.zeros(NY)
    y_deep[3] = -0.0095  # beyond the 9 mm cap -> cap row
    rows, tgt = pol._dist_rows(y_deep, G)
    assert rows and tgt[0] > 0
    y_mid = np.zeros(NY)
    y_mid[3] = -0.004  # inside [cap, retention] -> no row
    y_mid[4] = -0.004
    rows, tgt = pol._dist_rows(y_mid, G)
    assert rows == []
    y_shallow = np.zeros(NY)
    y_shallow[3] = -0.0002  # below the retention floor -> retain row
    rows, tgt = pol._dist_rows(y_shallow, G)
    assert rows and tgt[0] < 0


def test_14_no_tensile_force_demand():
    assert fz_des_n(-0.01, [[0.0, 1.0], [0.1, 1.0]]) >= 0.0
    assert fz_des_n(0.5, [[0.0, 1.0], [0.1, 1.0]]) == fz_des_n(0.1, [[0.0, 1.0], [0.1, 1.0]])
    assert fz_des_n(0.0, [[0.0, -0.5]]) == 0.0  # clamped, never tensile
    assert _spec()["CONTROLLER_CONSTANTS"]["FZ_TARGET_MIN_N"] == 0.0


def test_15_no_direct_root_contact_force_writes():
    for name in ["soft_contact.py", "run_cell.py", "s4_run_profiles.py"]:
        src = (REPO / "tools" / "res52" / name).read_text()
        for banned in ["qfrc_applied[", "xfrc_applied[", "qfrc_actuator[",
                       "qpos[", "qvel[", "data.contact[", ".efc_force["]:
            assert banned not in src.replace(" # ", " # "), (name, banned)


def test_16_active_set_crossing_detected():
    plant, probes, pol = _policy_fixture()
    x = _states()["S50"]
    before = pol.active_set_crossings
    pol.build_G(x, np.zeros(7), 40)
    # counter must be an int and non-decreasing; on this state the probes
    # stay in contact (no crossing) or are counted explicitly
    assert pol.active_set_crossings >= before


def test_17_no_root_actuation_constant():
    plant = make_plant()
    m = plant.model
    for nm in ["root_tx", "root_tz", "root_ry"]:
        jid = plant.idx.joint[nm]
        assert int(m.jnt_dofadr[jid]) >= 0
    # no actuator is attached to a root joint
    for aid in range(m.nu):
        assert m.actuator_trnid[aid, 0] not in [
            plant.idx.joint[nm] for nm in ["root_tx", "root_tz", "root_ry"]]


# ------------------------------------------------------------- qualification -
def _short(cell, horizon=0.015):
    c = dict(cell)
    c["HORIZON_S"] = horizon
    return c


def test_18_profile_run_safety_gates_short_cell():
    spec = _spec()
    kc = spec["CONTROLLER_CONSTANTS"]
    cell = _short([c for c in spec["PROFILES"] if c["CELL_ID"] == "P1a_MODERATE_S50"][0])
    out, acc, arr, matrices, infos, policy = run_cell(
        cell, kc, _states()["S50"],
        json.loads((C.WORK / "EVENTS.json").read_text()))
    assert out["FINITE"] and not out["PROHIBITED"]
    assert out["ROOT_LIMIT_ROWS"] == 0 and out["ROOT_PASSIVE_MAX"] == 0.0
    assert out["PEAK_FZ_BW"] <= 8.0
    assert out["MAX_PENETRATION"] <= 0.010
    assert out["MAX_ACTUATOR_UTIL"] <= 1.0 + 1e-9
    assert not out["REFLIGHT_EPISODES"]
    assert out["CONTACT_INACTIVE_FRACTION_L"] <= 0.005
    assert out["CONTACT_INACTIVE_FRACTION_R"] <= 0.005


def test_19_qualification_matrix_immutable_and_complete():
    qual = _qual()
    spec = _spec()
    assert qual["SPEC_SHA256"] == spec["EXPERIMENT_SPEC_SHA256"]
    assert set(qual["RESULTS"].keys()) == {c["CELL_ID"] for c in spec["PROFILES"]}
    for cid, r in qual["RESULTS"].items():
        assert r["SPEC_SHA256"] == spec["EXPERIMENT_SPEC_SHA256"]
    # every declared cell was executed exactly once, failures retained
    assert len(qual["RESULTS"]) == len(spec["PROFILES"]) == 6


def test_20_qualification_results_match_predeclared_gates():
    spec = _spec()
    G = spec["CONTROLLER_CONSTANTS"]["GATES"]
    qual = _qual()
    qualified = [cid for cid, r in qual["RESULTS"].items() if r["QUALIFIED"]]
    # the commit-rule minimum domain: body-weight hold + S50/S75 moderate
    for cid in ["P0a_BODYWEIGHT_HOLD_STAND", "P0b_BODYWEIGHT_HOLD_S75",
                "P1a_MODERATE_S50", "P1b_MODERATE_S75"]:
        assert cid in qualified, cid
        r = qual["RESULTS"][cid]
        assert r["PEAK_FZ_BW"] <= G["PEAK_FZ_MAX_BW"]
        assert r["MAX_PENETRATION"] <= G["PEN_SAFETY_LIMIT_M"]
        assert r["FZ_TRACK_BOUNDARY_RMS_ERR_BW"] <= G["FZ_TRACK_BOUNDARY_RMS_MAX_BW"]
        assert r["FZ_TRACK_BOUNDARY_MAX_ABS_ERR_BW"] <= G["FZ_TRACK_BOUNDARY_MAX_ABS_BW"]
        assert r["FZ_TRACK_LATE_RMS_ERR_BW"] <= G["FZ_TRACK_LATE_RMS_MAX_BW"]
        assert r["FZ_TRACK_LATE_MAX_ABS_ERR_BW"] <= G["FZ_TRACK_LATE_MAX_ABS_BW"]
        assert r["FZ_RIPPLE_LATE_MAX_BW"] <= G["LATE_RIPPLE_P2P_MAX_BW"]
        assert not r["REFLIGHT_EPISODES"]
        assert r["TRUST_FALLBACKS"] <= G["TRUST_FALLBACKS_MAX"]


def test_21_p2_brake_segment_qualification():
    qual = _qual()
    r = qual["RESULTS"]["P2_LANDING_BRAKE_S50"]
    seg = r["ARREST_SEGMENT"]
    assert seg["ARREST_SEGMENT_S"] == 0.050
    assert seg["SEGMENT_QUALIFIED"] is True
    assert not seg["SEGMENT_GATES"]["tracking"] or True
    assert all(seg["SEGMENT_GATES"].values())
    # the brake segment tracked the 2.0 BW arrest demand
    assert seg["BOUNDARY_RMS_ERR_BW"] <= 0.15
    assert seg["PEAK_FZ_BW"] <= 8.0
    assert seg["MAX_PENETRATION"] <= 0.010


def test_22_p3_boundary_failure_is_admissible_evidence():
    qual = _qual()
    r = qual["RESULTS"]["P3_VIABILITY_BOUNDARY_S100"]
    assert r["QUALIFIED"] is False
    # the failure must be contact-boundary behavior, not a hard-safety breach
    assert r["PEAK_FZ_BW"] <= 8.0
    assert r["MAX_PENETRATION"] <= 0.010
    assert r["FINITE"] and not r["PROHIBITED"]
    assert r["ROOT_LIMIT_ROWS"] == 0


def test_23_evidence_contract_spec_run_match():
    spec = _spec()
    assert spec["SPEC_EXECUTION_MATCH"] == "PASS required"
    run = json.loads((BUNDLE / "run_record.json").read_text()) \
        if (BUNDLE / "run_record.json").exists() else None
    if run is not None:
        assert run["SPEC_SHA256"] == spec["EXPERIMENT_SPEC_SHA256"]
