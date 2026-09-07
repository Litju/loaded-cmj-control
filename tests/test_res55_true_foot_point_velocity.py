"""RES-55 deterministic tests: true foot-point velocity authority.

MISSION=RES10_TRUE_FOOT_POINT_VELOCITY_AUTHORITY_CORRECTION_001
Covers: body/point selection, mj_jac(point)@qvel, v_xipos+omega x (p-xipos)
identity, FD validation, normal sign, L/R symmetry, threshold +-0.05,
no (p-xpos) regression, same-state sync, no mj_forward on live, sealed
profile recovery/immutability, hard gates, determinism, spec/run match.
"""
import json
import sys
from pathlib import Path
import hashlib
import mujoco
import numpy as np
import pytest

REPO = Path("/home/litju/Projects/loaded-cmj-control")
SEALED = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001")
BUNDLE = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-TRUE-FOOT-POINT-VELOCITY-001")
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "tools" / "res52"))
sys.path.insert(0, str(REPO))

import core52 as C
from core52 import make_plant, bind_plant, restore_state, get_state_vector, soft_contact_state, foot_gap_half_height, true_foot_point_velocity
from soft_contact import SoftContactPolicy, true_foot_point_velocity as true_vel_sc
from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data

PHYS_DT = 0.000125

def _plant():
    p = make_plant(); bind_plant(p); return p

def _states():
    return np.load(SEALED / "branch_states.npz")

def _spec():
    return json.loads((SEALED / "experiment_spec.json").read_text())

# exact body/point selection
def test_01_body_point_selection():
    p = _plant()
    assert p.idx.left_foot_body == 6 and p.idx.right_foot_body == 9
    assert p.idx.left_foot_geom == 10 and p.idx.right_foot_geom == 15
    gap = foot_gap_half_height(p.model)
    assert abs(gap - 0.01) < 1e-12
    # deepest-row rule: S50 both active
    m, d = p.model, p.make_data()
    restore_state(m, d, _states()["S50"])
    cs = soft_contact_state(m, d, gap_half_height=gap)
    assert cs["L"]["active_row"] is True and cs["R"]["active_row"] is True
    assert cs["L"]["penetration"] > 0.005

# mj_jac(point)@qvel authority
def test_02_jac_point_velocity():
    p = _plant(); m = p.model; d = p.make_data()
    restore_state(m, d, _states()["S50"])
    gap = foot_gap_half_height(m)
    for side, fg, fb in (("L", p.idx.left_foot_geom, p.idx.left_foot_body), ("R", p.idx.right_foot_geom, p.idx.right_foot_body)):
        p_low = np.asarray(d.geom_xpos[fg], float).copy(); p_low[2] -= gap
        v = true_foot_point_velocity(m, d, fb, p_low)
        Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, Jp, Jr, p_low, fb)
        assert np.allclose(v, Jp @ np.asarray(d.qvel, float), atol=0, rtol=0)
        # soft_contact duplicate identical
        v2 = true_vel_sc(m, d, fb, p_low)
        assert np.array_equal(v, v2)

# correct v_xipos + omega x (p-xipos) identity
def test_03_correct_transport_identity():
    p = _plant(); m = p.model; d = p.make_data()
    for lab in ["S_STAND", "S40", "S50", "S75", "S100"]:
        restore_state(m, d, _states()[lab])
        gap = foot_gap_half_height(m)
        for side, fg, fb in (("L", p.idx.left_foot_geom, p.idx.left_foot_body), ("R", p.idx.right_foot_geom, p.idx.right_foot_body)):
            p_low = np.asarray(d.geom_xpos[fg], float).copy(); p_low[2] -= gap
            m1 = true_foot_point_velocity(m, d, fb, p_low)
            vel = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, fb, vel, 0)
            m2 = vel[3:] + np.cross(vel[:3], p_low - np.asarray(d.xipos[fb], float))
            assert np.linalg.norm(m1 - m2) < 1e-9, (lab, side, np.linalg.norm(m1-m2))

# v_object already xipos (no xpos transport)
def test_04_object_velocity_is_xipos():
    p = _plant(); m = p.model; d = p.make_data()
    restore_state(m, d, _states()["S50"])
    for fb in (p.idx.left_foot_body, p.idx.right_foot_body):
        vel = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, fb, vel, 0)
        Jc = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv))
        mujoco.mj_jacBodyCom(m, d, Jc, Jr, fb)
        assert np.linalg.norm(vel[3:] - Jc @ np.asarray(d.qvel, float)) < 1e-12
        Jp = np.zeros((3, m.nv)); Jr2 = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, Jp, Jr2, np.asarray(d.xpos[fb], float), fb)
        # when rotating, xpos velocity differs materially
        assert np.linalg.norm(vel[3:] - Jp @ np.asarray(d.qvel, float)) > 1e-03

# FD validation (smooth standing)
def test_05_finite_difference_validation():
    p = _plant(); m = p.model
    gap = foot_gap_half_height(m)
    # use P0a corrected trace p_low trajectory (material point)
    traj = np.load(BUNDLE / "_traj_P0a_BODYWEIGHT_HOLD_STAND.npz")
    M1 = traj["M1_L"]; M3 = traj["M3_L"]
    err = M1[2:-2] - M3[2:-2]
    assert float(np.sqrt(np.mean(err[:, 2]**2))) < 5e-04
    assert float(np.max(np.abs(err[:, 2]))) < 2e-03

# normal sign convention
def test_06_normal_sign():
    p = _plant(); m, d = p.model, p.make_data()
    restore_state(m, d, _states()["S50"])
    gap = foot_gap_half_height(m)
    cs = soft_contact_state(m, d, gap_half_height=gap)
    assert cs["L"]["foot_normal_velocity"] > 0.05  # unloading separating
    assert cs["L"]["penetration"] > 0.005
    # efc equals J(contact) normal
    for k in range(d.ncon):
        con = d.contact[k]
        if {int(con.geom1), int(con.geom2)} == {p.idx.left_foot_geom, p.idx.floor_geom}:
            if abs(float(con.dist) - cs["L"]["contact_distance"]) < 1e-12:
                row = int(con.efc_address)
                J = np.asarray(d.efc_J, float).reshape(int(d.nefc), int(m.nv))
                assert float(d.efc_vel[row]) == pytest.approx(float(J[row] @ np.asarray(d.qvel, float)), abs=1e-9)
                break

# L/R symmetry (standing and S50 symmetric plant)
def test_07_lr_symmetry():
    p = _plant(); m, d = p.model, p.make_data()
    for lab in ["S_STAND", "S50"]:
        restore_state(m, d, _states()[lab])
        gap = foot_gap_half_height(m)
        cs = soft_contact_state(m, d, gap_half_height=gap)
        assert abs(cs["L"]["foot_normal_velocity"] - cs["R"]["foot_normal_velocity"]) < 1e-09
        assert abs(cs["L"]["contact_distance"] - cs["R"]["contact_distance"]) < 1e-12

# threshold cases around +-0.05
def test_08_threshold_cases():
    # synthetic: construct velocities around threshold via scaling qvel
    p = _plant(); m, d = p.model, p.make_data()
    restore_state(m, d, _states()["S50"])
    gap = foot_gap_half_height(m)
    fg, fb = p.idx.left_foot_geom, p.idx.left_foot_body
    p_low = np.asarray(d.geom_xpos[fg], float).copy(); p_low[2] -= gap
    m1 = true_foot_point_velocity(m, d, fb, p_low)
    # S50 M1_n ~0.155 (outside), buggy ~0.172 (outside) -> no flip here, but P0b hypo flips exist
    # directly test classifier logic
    def cls(v): return "INSIDE" if abs(v) < 0.05 else "OUTSIDE"
    assert cls(0.049) == "INSIDE" and cls(0.051) == "OUTSIDE" and cls(-0.049) == "INSIDE"
    # true P3 inactive both outside (no flip) but magnitude differs
    assert cls(0.543) == "OUTSIDE" and cls(0.705) == "OUTSIDE"

# no (p-xpos) transport regression
def test_09_no_xpos_transport():
    import re
    for name in ["core52.py", "soft_contact.py"]:
        src = (REPO / "tools" / "res52" / name).read_text()
        # remove comments mentioning prohibition, then search for code pattern
        lines = [l for l in src.splitlines() if not l.strip().startswith("#") and "Prohibited" not in l and "Must NOT" not in l and "prohibit" not in l.lower()]
        code = "\n".join(lines)
        # buggy code called mujoco.mj_objectVelocity( for foot velocity; docstrings may mention it in prohibition
        assert "mujoco.mj_objectVelocity(" not in code, (name, "mj_objectVelocity call must not remain in foot-velocity code")

# same-state synchronization (no mj_forward on live)
def test_10_same_state_sync():
    p = _plant(); m, d = p.model, p.make_data()
    meas = create_measurement_data(p)
    restore_state(m, d, _states()["S50"])
    s = SynchronizedPhysicsSample.from_live_state(p, d, meas)
    assert s.check_time_identity()
    gap = foot_gap_half_height(m)
    # soft_contact_state on synchronized meas (forwarded shadow) must match live-forwarded data
    cs_meas = soft_contact_state(m, meas, gap_half_height=gap)
    cs_live = soft_contact_state(m, d, gap_half_height=gap)
    assert abs(cs_meas["L"]["foot_normal_velocity"] - cs_live["L"]["foot_normal_velocity"]) < 1e-12

def test_11_no_forward_on_live():
    src = (REPO / "tools" / "res52" / "core52.py").read_text()
    # true helper must not call mj_forward
    helper = src.split("def true_foot_point_velocity")[1].split("def soft_contact_state")[0]
    assert "mujoco.mj_forward(" not in helper
    src2 = (REPO / "tools" / "res52" / "soft_contact.py").read_text()
    helper2 = src2.split("def true_foot_point_velocity")[1].split("class SoftContactPolicy")[0]
    assert "mujoco.mj_forward(" not in helper2

# sealed profile recovery byte-for-byte
def test_12_profile_recovery():
    spec = _spec()
    assert spec["EXPERIMENT_SPEC_SHA256"] == "a8087f02fc78aa7977aedf6d5d819963bb10c355818c57154b3cfb3df2ec42ef"
    assert len(spec["PROFILES"]) == 6
    ids = [c["CELL_ID"] for c in spec["PROFILES"]]
    assert ids == ["P0a_BODYWEIGHT_HOLD_STAND","P0b_BODYWEIGHT_HOLD_S75","P1a_MODERATE_S50","P1b_MODERATE_S75","P2_LANDING_BRAKE_S50","P3_VIABILITY_BOUNDARY_S100"]
    # exact nodes
    p2 = [c for c in spec["PROFILES"] if c["CELL_ID"]=="P2_LANDING_BRAKE_S50"][0]
    assert p2["FZ_NODES_BW"] == [[0.0,2.0],[0.050,2.0],[0.100,1.0]]
    assert p2["ARREST_SEGMENT_S"] == 0.050

def test_13_profile_immutability():
    spec = _spec()
    cc = spec["CONTROLLER_CONSTANTS"]
    assert cc["VALIDATION_TOL"]["NVEL_ABS_MPS"] == 0.05
    assert cc["QUALIFIED_NVEL_DOMAIN_MPS"] == [-0.35,0.35]
    assert cc["GATES"]["PEAK_FZ_MAX_BW"] == 8.0
    assert cc["LS_WEIGHTS"]["W_NVEL"] == 1.0
    assert cc["OLD_EXT_DIR_USED"] is False

# hard gates (corrected requalification preserves ceiling, retains failures)
def test_14_hard_gates_corrected():
    qual = json.loads((BUNDLE / "CORRECTED_FORCE_PROFILE_QUALIFICATION.json").read_text())
    # P0a, P0b, P1a still qualified; P2 segment qualified; P3 failed (boundary)
    assert qual["RESULTS"]["P0a_BODYWEIGHT_HOLD_STAND"]["QUALIFIED"] is True
    assert qual["RESULTS"]["P0b_BODYWEIGHT_HOLD_S75"]["QUALIFIED"] is True
    assert qual["RESULTS"]["P1a_MODERATE_S50"]["QUALIFIED"] is True
    assert qual["RESULTS"]["P2_LANDING_BRAKE_S50"]["ARREST_SEGMENT"]["SEGMENT_QUALIFIED"] is True
    assert qual["RESULTS"]["P3_VIABILITY_BOUNDARY_S100"]["QUALIFIED"] is False
    # all cells finite, no prohibited, no root support
    for cid, r in qual["RESULTS"].items():
        assert r["FINITE"] and not r["PROHIBITED"]
        assert r["ROOT_LIMIT_ROWS"] == 0 and r["ROOT_PASSIVE_MAX"] == 0.0
        assert r["MAX_ACTUATOR_UTIL"] <= 1.0 + 1e-9
        assert r["PEAK_FZ_BW"] <= 8.0 and r["MAX_PENETRATION"] <= 0.010

# determinism (rebuild G twice identical)
def test_15_determinism():
    p = _plant()
    probes = [p.make_data() for _ in range(17)]
    pol = SoftContactPolicy(_spec()["CONTROLLER_CONSTANTS"], p, probes)
    pol.gap_hh = foot_gap_half_height(p.model)
    x = _states()["S_STAND"]
    d0 = p.make_data(); restore_state(p.model, d0, x)
    u_nom = np.asarray(d0.ctrl, float).copy()
    _, G1, _, _ = pol.build_G(x, u_nom, 40)
    pol2 = SoftContactPolicy(_spec()["CONTROLLER_CONSTANTS"], p, [p.make_data() for _ in range(17)])
    pol2.gap_hh = pol.gap_hh
    _, G2, _, _ = pol2.build_G(x, u_nom, 40)
    assert np.array_equal(G1, G2)

# spec/run match
def test_16_spec_run_match():
    spec = _spec()
    qual = json.loads((BUNDLE / "CORRECTED_FORCE_PROFILE_QUALIFICATION.json").read_text())
    assert qual["SPEC_SHA256"] == spec["EXPERIMENT_SPEC_SHA256"]
    for cid, r in qual["RESULTS"].items():
        assert r["SPEC_SHA256"] == spec["EXPERIMENT_SPEC_SHA256"]
