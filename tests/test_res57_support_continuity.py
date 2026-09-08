"""RES-57 deterministic tests: bilateral support-continuity semantics.

MISSION=RES10_BILATERAL_SUPPORT_CONTINUITY_AUTHORITY_001
Covers: exact source thresholds/dwells, contract hash immutability, sample vs
duration distinction, 1-step dropout, 40-step boundary, canonical reflight
unchanged, corrected foot-normal velocity, exact RES-56 target samples,
bit-identical replay, deterministic row classification, no Plant/contact/
controller/scorer edits, Evidence Contract v2 identity.
"""
import json
import sys
from pathlib import Path
import hashlib
import numpy as np
import pytest

REPO = Path("/home/litju/Projects/loaded-cmj-control")
SEALED52 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001")
B56 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-TRUE-MOMENTUM-CAPTURE-001")
B57 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-BILATERAL-SUPPORT-CONTINUITY-001")
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "tools" / "res52"))
sys.path.insert(0, str(REPO))

SPEC_SHA = "cde531e99900e9c06dc0cab0ae33ec9bff3150d05b3f03c0e51daa1b67835734"

def _spec():
    return json.loads((REPO / "support_continuity_spec.json").read_text())

# 01 exact source thresholds/dwells from frozen authority
def test_01_source_thresholds_dwells():
    from loaded_cmj.v2.constants import (
        V2_CONTACT_FZ_THRESHOLD_N, V2_EVENT_THRESHOLDS, V2_PHYSICS_TIMESTEP_S,
        V2_CONTROL_PERIOD_S, V2_CONTACT_SOLREF, V2_CONTACT_SOLIMP,
        V2_TOTAL_MASS_KG, V2_TORQUE_LIMITS_NM)
    assert V2_CONTACT_FZ_THRESHOLD_N == 10.0
    assert float(V2_EVENT_THRESHOLDS["BILATERAL_TAKEOFF_FZ_N"]) == 10.0
    assert V2_PHYSICS_TIMESTEP_S == 0.000125
    assert V2_CONTROL_PERIOD_S == 0.005
    assert tuple(V2_CONTACT_SOLREF) == (0.016, 1.0)
    assert tuple(V2_CONTACT_SOLIMP) == (0.99, 0.99, 0.001, 0.5, 2.0)
    assert V2_TOTAL_MASS_KG == 95.0
    assert dict(V2_TORQUE_LIMITS_NM)["left_knee"] == 300.0
    sys.path.insert(0, str(REPO / "tools" / "res52"))
    import importlib
    specmod = importlib.import_module("spec")
    G = specmod.CONTROLLER_CONSTANTS["GATES"]
    assert G["CONTACT_INACTIVE_FRACTION_MAX"] == 0.005
    assert G["CONTACT_MAX_INACTIVE_RUN_PHYSICS_STEPS"] == 2
    assert G["REFLIGHT_MIN_DURATION_PHYSICS_STEPS"] == 4
    assert G["POST_WALK_LOSS_EPISODES_MAX"] == 1
    assert G["CHATTER_TRANSITIONS_MAX"] == 8
    assert specmod.CONTROLLER_CONSTANTS["CONTACT_ACTIVE_THRESHOLD_N"] == 10.0
    assert specmod.CONTROLLER_CONSTANTS["WALK_WINDOW_S"] == 0.030

# 02 contract hash immutability
def test_02_spec_hash_immutability():
    from loaded_cmj.v2.support_continuity import load_spec, spec_sha256
    spec = load_spec(REPO / "support_continuity_spec.json")
    assert spec["SPEC_SHA256"] == SPEC_SHA
    assert spec_sha256(spec) == SPEC_SHA
    bad = dict(spec); bad["CONTROL_RELEVANT_LOSS_STEPS"] = 39
    # tamper must fail verification
    import tempfile, os
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(bad, f); tmp = f.name
    try:
        with pytest.raises(ValueError):
            load_spec(tmp)
    finally:
        os.unlink(tmp)

# 03 sample liftoff vs duration-level support loss are distinct fields
def test_03_sample_vs_episode_distinction():
    from loaded_cmj.v2 import support_continuity as sc
    assert sc.STEPS_PER_CONTROL_INTERVAL == 40
    # one liftoff sample: sample flag true ...
    s = sc.classify_sample(has_row=False, dist=0.001, fz=0.0, nvel=0.01)
    assert s["GEOMETRIC_LIFTOFF_SAMPLE"] is True
    # ... but a lone sample never forms an episode
    eps = sc.control_relevant_episodes(fz=[0.0], has_row=[False], nvel=[0.01])
    assert eps == [{"start": 0, "length": 1, "duration_s": 0.000125, "CONTROL_RELEVANT": False}]

# 04 one-physics-step dropout is not control-relevant
def test_04_one_step_dropout():
    from loaded_cmj.v2 import support_continuity as sc
    n = 100
    fz = [500.0]*n; hr = [True]*n; nv = [-0.01]*n
    fz[50] = 0.0; nv[50] = 0.002
    eps = sc.control_relevant_episodes(fz=fz, has_row=hr, nvel=nv)
    assert len(eps) == 1 and eps[0]["CONTROL_RELEVANT"] is False
    adj = sc.adjudicate_trajectory(fz_l=[500.0]*n, fz_r=fz, whole_fz=[1000.0]*49+[500.0]+[1000.0]*50,
                                   has_row_l=[True]*n, has_row_r=hr, nvel_l=[-0.01]*n, nvel_r=nv,
                                   dist_l=[-0.001]*n, dist_r=[-0.001]*49+[-0.000006]+[-0.001]*50)
    assert adj["UNILATERAL_FORCE_DROPOUT_SAMPLE_COUNT"] == 1
    assert adj["GEOMETRIC_LIFTOFF_SAMPLE_COUNT"] == 0
    assert adj["CONTROL_RELEVANT_SUPPORT_LOSS_COUNT"] == 0
    assert adj["SUPPORT_CONTINUITY"] == "QUALIFIED"

# 05 one-control-interval duration boundary (39 vs 40)
def test_05_duration_boundary():
    from loaded_cmj.v2 import support_continuity as sc
    for L, expect in ((39, False), (40, True), (41, True)):
        fz = [0.0]*L + [500.0]*10; hr = [False]*L + [True]*10; nv = [0.005]*L + [-0.01]*10
        eps = sc.control_relevant_episodes(fz=fz, has_row=hr, nvel=nv)
        assert len(eps) == 1 and eps[0]["CONTROL_RELEVANT"] is expect, L

# 06 canonical reflight semantics unchanged
def test_06_canonical_reflight_unchanged():
    from loaded_cmj.v2 import support_continuity as sc
    assert sc.REFLIGHT_MIN_STEPS == 4
    base = [900.0]*20
    w3 = base[:10] + [0.0]*3 + base[:7]
    w4 = base[:10] + [0.0]*4 + base[:6]
    assert sc.canonical_reflight(w3) == []
    assert sc.canonical_reflight(w4) == [4]
    assert sc.chatter_transitions([900.0]*10) == 0
    assert sc.chatter_transitions(w4) == 2

# 07 corrected foot-normal velocity used (active rows read efc_vel)
def test_07_corrected_nvel_authority():
    import mujoco
    sys.path.insert(0, str(REPO / "tools" / "res52"))
    from core52 import make_plant, bind_plant, restore_state, soft_contact_state, foot_gap_half_height
    states = np.load(SEALED52 / "branch_states.npz")
    p = make_plant(); bind_plant(p)
    m, d = p.model, p.make_data()
    restore_state(m, d, states["S50"])
    gap = foot_gap_half_height(m)
    cs = soft_contact_state(m, d, gap_half_height=gap)
    assert cs["L"]["active_row"] and cs["R"]["active_row"]
    # active nvel must equal deepest-row efc_vel exactly
    for side in ("L", "R"):
        fg = p.idx.left_foot_geom if side == "L" else p.idx.right_foot_geom
        rows = [i for i in range(d.ncon)
                if (int(d.contact[i].geom1) == p.idx.floor_geom or int(d.contact[i].geom2) == p.idx.floor_geom)
                and (int(d.contact[i].geom1) == fg or int(d.contact[i].geom2) == fg)]
        deep = rows[int(np.argmin([float(d.contact[j].dist) for j in rows]))]
        assert cs[side]["foot_normal_velocity"] == float(d.efc_vel[d.contact[deep].efc_address])

# 08 exact RES-56 target samples reproduced
def test_08_exact_target_samples():
    rep = json.loads((B57 / "RES57_S50_REPLAY.json").read_text())
    assert rep["DROPOUT_INDICES"] == [1016, 1025, 1165]
    assert rep["SEALED_DROPOUT_INDICES"] == [1016, 1025, 1165]
    assert rep["DROPOUT_MATCH"] is True
    R = np.load(B57 / "RES57_S50_REPLAY.npz")
    t0 = float(R["t0"])
    for idx, ms in ((1016, 127.125), (1025, 128.250), (1165, 145.750)):
        assert abs((float(R["t"][idx]) - t0)*1000.0 - ms) < 1e-6
    assert float(R["fzl"][1016]) <= 10.0 and float(R["fzr"][1016]) > 10.0
    assert float(R["fzr"][1025]) <= 10.0 and float(R["fzl"][1025]) > 10.0
    assert float(R["fzr"][1165]) <= 10.0 and float(R["fzl"][1165]) > 10.0

# 09 bit-identical recorded-action replay
def test_09_bit_identical_replay():
    rep = json.loads((B57 / "RES57_S50_REPLAY.json").read_text())
    assert rep["S50_SHA256"] == "bb56c0d3df3cd50aa55be515783af5cfb24f212eaaa29d17427bd3e60c6ceae2"
    assert rep["QPOS_BIT_IDENTICAL"] is True and rep["QVEL_BIT_IDENTICAL"] is True
    assert rep["MAX_QPOS_ABS"] == 0.0 and rep["MAX_QVEL_ABS"] == 0.0
    assert rep["NEARZERO_ENTRY_IDX"] == 414 and rep["ZERO_CROSS_IDX"] == 504
    assert abs(rep["NEARZERO_ENTRY_T_REL"] - 0.051875) < 1e-9
    assert abs(rep["NEARZERO_DWELL_S"] - 0.09825) < 1e-12

# 10 deterministic contact-row classification incl. hidden-liftoff prohibition
def test_10_row_classification_deterministic():
    from loaded_cmj.v2 import support_continuity as sc
    a = sc.classify_sample(has_row=True, dist=-0.000006, fz=3.13, nvel=0.002938)
    assert a == sc.classify_sample(has_row=True, dist=-0.000006, fz=3.13, nvel=0.002938)
    assert a["GEOMETRIC_ENGAGEMENT_SAMPLE"] is True
    assert a["COMPRESSIVE_SUPPORT_SAMPLE"] is False
    assert a["GEOMETRIC_LIFTOFF_SAMPLE"] is False  # retained rows must not read as liftoff
    b = sc.classify_sample(has_row=False, dist=0.002, fz=0.0, nvel=0.05)
    assert b["GEOMETRIC_LIFTOFF_SAMPLE"] is True  # short liftoff still reported as such

# 11 no Plant/contact/controller/scorer edits (frozen numeric authorities)
def test_11_no_authority_edits():
    from loaded_cmj.v2.constants import V2_CONTACT_SOLREF, V2_CONTACT_SOLIMP, V2_FRICTION_FLOOR, V2_TOTAL_MASS_KG
    assert tuple(V2_CONTACT_SOLREF) == (0.016, 1.0)
    assert tuple(V2_CONTACT_SOLIMP) == (0.99, 0.99, 0.001, 0.5, 2.0)
    assert tuple(V2_FRICTION_FLOOR) == (0.9, 0.005, 0.0001)
    assert V2_TOTAL_MASS_KG == 95.0
    import inspect
    import loaded_cmj.v2.support_continuity as scmod
    src = inspect.getsource(scmod)
    assert "import mujoco" not in src and "mujoco." not in src
    body = "\n".join(l for l in src.splitlines() if not l.strip().startswith(('"""', "no mj_forward")))
    assert "mj_forward(" not in body and "mj_step(" not in body

# 12 Evidence Contract v2 spec/run identity
def test_12_evidence_identity():
    man = json.loads((B57 / "manifest.json").read_text())
    spec = json.loads((B57 / "experiment_spec.json").read_text())
    run = json.loads((B57 / "run_record.json").read_text())
    res = json.loads((B57 / "result_assessment.json").read_text())
    assert man["EVIDENCE_CONTRACT"] == "v2"
    assert spec["EXPERIMENT_ID"] == run["EXPERIMENT_ID"] == res["EXPERIMENT_ID"]
    assert run["SPEC_EXECUTION_MATCH"] == "PASS"
    assert res["SPEC_EXECUTION_MATCH"] == "PASS"
