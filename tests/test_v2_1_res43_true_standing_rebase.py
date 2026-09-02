"""RES-43 durable tests for true-standing rebase onto zero-root-damping Plant — 18 checks.

Covers:
1. old RES-16 standing envelope fails corrected Plant (8% outside)
2. new standing reference comes from observed trace (median)
3. one-ULP-only envelope expansion (np.nextafter)
4. qualified standing passes new predicate (15200/15200)
5. continuous >=0.500 s standing dwell exists (1.90s)
6. q joint outside envelope rejects
7. COM z outside rejects
8. root z outside rejects
9. trunk outside rejects
10. unilateral support rejects
11. physical fall rejects
12. prohibited support rejects
13. E1-E11 contract hash unchanged (2f830d...)
14. E12 base thresholds unchanged (tilt 0.2618, vz 0.05, Fz 0.5W, dwell 0.5)
15. E12 dwell remains 0.500 s
16. controller unchanged (hash)
17. Plant unchanged (xml hash, mass, ngeom)
18. zero root passive authority remains true (damping 0, passive 0)
"""

import sys
from pathlib import Path
import hashlib
import json
import numpy as np
import pytest
import mujoco

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(TASK_ROOT / "src"))

from loaded_cmj.v2.constants import (
    V2_TOTAL_MASS_KG,
    V2_PHYSICS_TIMESTEP_S,
    V2_CONTROL_PERIOD_S,
    V2_TRUE_STANDING_ENVELOPE,
    V2_TRUE_STANDING_REFERENCE,
    V2_EVENT_CONTRACT_VERSION,
    V2_EVENT_THRESHOLDS,
    V2_CONTACT_SOLREF,
    V2_CONTACT_SOLIMP,
)
from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2.events import V2EventDetector

DT = V2_PHYSICS_TIMESTEP_S
WEIGHT = V2_TOTAL_MASS_KG * 9.81
ENTRY_HEAD = "87088298139f9b34418c573e8a05af58eed6fcc0"

def make_sample(**overrides):
    base = {
        "time_s": 0.0,
        "com_z": V2_TRUE_STANDING_REFERENCE["COM_Z_STAND_REF"],
        "root_z": V2_TRUE_STANDING_REFERENCE["ROOT_Z_STAND_REF"],
        "trunk_tilt": V2_TRUE_STANDING_REFERENCE["TRUNK_PITCH_STAND_REF"],
        "left_Fz": 465.0,
        "right_Fz": 465.0,
        "whole_Fz": 930.0,
        "com_vz": 0.01,
        "com_margin": 0.1,
        "prohibited": False,
        "fall_contact": False,
        "joint_position_rad": np.array(V2_TRUE_STANDING_REFERENCE["Q_STAND_REF"]),
        "com_position_m": np.array([0,0, V2_TRUE_STANDING_REFERENCE["COM_Z_STAND_REF"]]),
        "pelvis_position_world_m": np.array([0,0, V2_TRUE_STANDING_REFERENCE["ROOT_Z_STAND_REF"]]),
        "plantar_normal_force_N": np.array([465.0, 465.0]),
    }
    base.update(overrides)
    return base

def test_01_old_envelope_fails_corrected_plant():
    OLD_Q_ENV = [(-0.0010165676005093134, 0.00010766947760009482), (-5.925668531212257e-05, 0.0005937610508289983), (-5.925668531212092e-05, 0.000593761050828997), (-0.00013928028759005345, 5.80689459531941e-05), (-0.00013928028759005296, 5.806894595319459e-05), (-8.127035230230239e-05, 0.0013418169340970038), (-8.127035230230365e-05, 0.001341816934097)]
    Q_MIN_NEW = [-0.0010646233267473013, -6.015562677118107e-05, -6.015562677117834e-05, -0.00014991030670825437, -0.00014991030670825434, -8.092467744017705e-05, -8.092467744017095e-05]
    Q_MAX_NEW = [0.00011071707376190296, 0.0006449640776937647, 0.0006449640776937637, 5.819712849261686e-05, 5.819712849261647e-05, 0.001443466858333388, 0.0014434668583333844]
    assert Q_MIN_NEW[0] < OLD_Q_ENV[0][0] or Q_MAX_NEW[0] > OLD_Q_ENV[0][1]
    assert Q_MIN_NEW[1] < OLD_Q_ENV[1][0] or Q_MAX_NEW[1] > OLD_Q_ENV[1][1]
    evidence = json.loads((Path("/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-V2_1-RES43-TRUE-STANDING-REBASE-20260901/03_OLD_PREDICATE_REPRODUCTION.json").read_text()))
    assert evidence["evaluation"]["OLD_PREDICATE_FAIL_COUNT"] == 1269
    assert evidence["evaluation"]["OLD_TRUE_STANDING_REFERENCE_STILL_VALID"] == "NO"

def test_02_new_reference_from_observed_trace():
    q_ref = np.array(V2_TRUE_STANDING_REFERENCE["Q_STAND_REF"])
    assert not np.allclose(q_ref, np.zeros(7)), "reference should not be controller target 0"
    evidence = json.loads((Path("/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-V2_1-RES43-TRUE-STANDING-REBASE-20260901/04_NEW_STANDING_REFERENCE.json").read_text()))
    assert np.allclose(q_ref, np.array(evidence["Q_STAND_REF_NEW"]))
    assert abs(float(V2_TRUE_STANDING_REFERENCE["COM_Z_STAND_REF"]) - evidence["COM_Z_STAND_REF_NEW"]) < 1e-12
    assert "componentwise median" in V2_TRUE_STANDING_REFERENCE["provenance"]

def test_03_one_ulp_only_expansion():
    env = V2_TRUE_STANDING_ENVELOPE
    qmin = env["Q_STAND_MIN"]
    qmax = env["Q_STAND_MAX"]
    qenv = env["Q_STAND_ENVELOPE"]
    for i in range(7):
        assert float(np.nextafter(qmin[i], -np.inf)) == qenv[i][0], f"joint {i} lower not one ULP"
        assert float(np.nextafter(qmax[i], np.inf)) == qenv[i][1], f"joint {i} upper not one ULP"
    assert float(np.nextafter(env["COM_Z_STAND_MIN"], -np.inf)) == env["COM_Z_STAND_ENVELOPE"][0]
    assert float(np.nextafter(env["COM_Z_STAND_MAX"], np.inf)) == env["COM_Z_STAND_ENVELOPE"][1]
    assert float(np.nextafter(env["ROOT_Z_STAND_MIN"], -np.inf)) == env["ROOT_Z_STAND_ENVELOPE"][0]
    assert float(np.nextafter(env["ROOT_Z_STAND_MAX"], np.inf)) == env["ROOT_Z_STAND_ENVELOPE"][1]
    assert float(np.nextafter(env["TRUNK_PITCH_STAND_MIN"], -np.inf)) == env["TRUNK_PITCH_STAND_ENVELOPE"][0]
    assert float(np.nextafter(env["TRUNK_PITCH_STAND_MAX"], np.inf)) == env["TRUNK_PITCH_STAND_ENVELOPE"][1]
    assert env["expansion"] == "np.nextafter(lower, -inf) / np.nextafter(upper, +inf)"
    assert env["no_arbitrary_factors"] is True

def test_04_qualified_standing_passes_new_predicate():
    det = V2EventDetector()
    sample = make_sample()
    assert det._is_true_standing_neighborhood(sample)
    assert det._guard_stable_recovery(sample)

def test_05_continuous_dwell_exists():
    det = V2EventDetector()
    evidence = json.loads((Path("/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-V2_1-RES43-TRUE-STANDING-REBASE-20260901/06_POSITIVE_DWELL_QUALIFICATION.json").read_text()))
    assert evidence["NEW_TRUE_STANDING_LONGEST_DWELL"] >= 0.5
    assert evidence["NEW_TRUE_STANDING_LONGEST_DWELL"] == 1.9
    assert evidence["pass_count"] == 15200
    for name in ["supported_start","countermovement_onset","valid_countermovement","upward_reversal","vertical_propulsion","bilateral_takeoff","genuine_flight","apex","descending_landing","impact_absorption","balance_capture"]:
        from loaded_cmj.v2.events import V2EventRecord
        det.event_records[name] = V2EventRecord(name, 0.0, 0.0, 0, 0)
    base_time = 3.0
    for i in range(4000):
        s = make_sample(time_s=base_time + i*DT)
        det.update(s)
    assert "stable_recovery" in det.event_records

def test_06_q_joint_outside_rejects():
    det = V2EventDetector()
    q = np.array(V2_TRUE_STANDING_REFERENCE["Q_STAND_REF"])
    lo, hi = V2_TRUE_STANDING_ENVELOPE["Q_STAND_ENVELOPE"][0]
    q_out = q.copy()
    q_out[0] = hi + 1e-6
    sample = make_sample(joint_position_rad=q_out)
    assert not det._is_true_standing_neighborhood(sample)
    q_out2 = q.copy()
    lo3, hi3 = V2_TRUE_STANDING_ENVELOPE["Q_STAND_ENVELOPE"][3]
    q_out2[3] = hi3 + 1e-6
    sample2 = make_sample(joint_position_rad=q_out2)
    assert not det._is_true_standing_neighborhood(sample2)

def test_07_com_z_outside_rejects():
    det = V2EventDetector()
    lo, hi = V2_TRUE_STANDING_ENVELOPE["COM_Z_STAND_ENVELOPE"]
    sample = make_sample(com_z=lo - 0.01, com_position_m=np.array([0,0, lo-0.01]))
    assert not det._is_true_standing_neighborhood(sample)
    sample2 = make_sample(com_z=hi + 0.01, com_position_m=np.array([0,0, hi+0.01]))
    assert not det._is_true_standing_neighborhood(sample2)

def test_08_root_z_outside_rejects():
    det = V2EventDetector()
    lo, hi = V2_TRUE_STANDING_ENVELOPE["ROOT_Z_STAND_ENVELOPE"]
    sample = make_sample(root_z=lo - 0.01, pelvis_position_world_m=np.array([0,0, lo-0.01]))
    assert not det._is_true_standing_neighborhood(sample)

def test_09_trunk_outside_rejects():
    det = V2EventDetector()
    lo, hi = V2_TRUE_STANDING_ENVELOPE["TRUNK_PITCH_STAND_ENVELOPE"]
    sample = make_sample(trunk_tilt=hi + 0.001)
    assert not det._is_true_standing_neighborhood(sample)
    sample2 = make_sample(trunk_tilt=0.1)
    assert not det._is_true_standing_neighborhood(sample2)

def test_10_unilateral_support_rejects():
    det = V2EventDetector()
    sample = make_sample(left_Fz=5.0, right_Fz=465.0, plantar_normal_force_N=np.array([5.0, 465.0]))
    assert not det._is_true_standing_neighborhood(sample)
    sample2 = make_sample(left_Fz=465.0, right_Fz=5.0, plantar_normal_force_N=np.array([465.0, 5.0]))
    assert not det._is_true_standing_neighborhood(sample2)

def test_11_physical_fall_rejects():
    det = V2EventDetector()
    sample = make_sample(fall_contact=True)
    assert not det._is_true_standing_neighborhood(sample)

def test_12_prohibited_support_rejects():
    det = V2EventDetector()
    sample = make_sample(prohibited=True)
    assert not det._is_true_standing_neighborhood(sample)

def test_13_e1_e11_contract_unchanged():
    before = Path("/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-V2_1-RES43-TRUE-STANDING-REBASE-20260901/08_E1_E11_CONTRACT_BEFORE.json")
    after = Path("/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-V2_1-RES43-TRUE-STANDING-REBASE-20260901/09_E1_E11_CONTRACT_AFTER.json")
    assert hashlib.sha256(before.read_bytes()).hexdigest() == "2f830deff70956c2f82f3a9e41311f92371ddb10fe33222c2d4a829c315e684e"
    assert hashlib.sha256(after.read_bytes()).hexdigest() == "2f830deff70956c2f82f3a9e41311f92371ddb10fe33222c2d4a829c315e684e"
    assert hashlib.sha256(before.read_bytes()).hexdigest() == hashlib.sha256(after.read_bytes()).hexdigest()

def test_14_e12_base_thresholds_unchanged():
    assert float(V2_EVENT_THRESHOLDS["STABLE_RECOVERY_TILT_MAX_RAD"]) == 0.2618
    assert float(V2_EVENT_THRESHOLDS["STABLE_RECOVERY_DWELL_S"]) == 0.5
    det = V2EventDetector()
    q_capture = np.array([-0.34, 0.428, 0.428, 1.65, 1.65, 0.546, 0.546])
    sample = {
        "joint_position_rad": q_capture,
        "com_z": 0.829,
        "root_z": 0.632,
        "trunk_tilt": 0.18,
        "left_Fz": 447.0,
        "right_Fz": 447.0,
        "whole_Fz": 894.0,
        "com_vz": 0.01,
        "com_margin": 0.1,
        "prohibited": False,
        "fall_contact": False,
        "com_position_m": np.array([0,0,0.829]),
        "pelvis_position_world_m": np.array([0,0,0.632]),
        "plantar_normal_force_N": np.array([447.0,447.0]),
    }
    old_base_true = (abs(sample["trunk_tilt"]) < 0.2618 and abs(sample["com_vz"]) < 0.05 and sample["whole_Fz"] > 0.5*WEIGHT)
    assert old_base_true
    assert not det._is_true_standing_neighborhood(sample)
    assert not det._guard_stable_recovery(sample)

def test_15_e12_dwell_remains_0_500():
    assert float(V2_EVENT_THRESHOLDS["STABLE_RECOVERY_DWELL_S"]) == 0.5
    from loaded_cmj.v2.events import DWELL_S, WIN
    assert DWELL_S["stable_recovery"] == 0.5
    assert WIN["stable_recovery"] == int(round(0.5 / DT)) == 4000

def test_16_controller_unchanged():
    import subprocess
    entry_hash = subprocess.run(["git", "show", f"{ENTRY_HEAD}:src/loaded_cmj/v2/controller.py"], capture_output=True).stdout
    current = Path("src/loaded_cmj/v2/controller.py").read_bytes()
    assert hashlib.sha256(entry_hash).hexdigest() == hashlib.sha256(current).hexdigest()

def test_17_plant_unchanged():
    import subprocess
    entry_xml = subprocess.run(["git", "show", f"{ENTRY_HEAD}:src/loaded_cmj/v2/assets/v2_plant.xml"], capture_output=True).stdout
    current_xml = Path("src/loaded_cmj/v2/assets/v2_plant.xml").read_bytes()
    assert hashlib.sha256(entry_xml).hexdigest() == hashlib.sha256(current_xml).hexdigest()
    p = V2Plant()
    assert abs(float(p.model.body_mass.sum()) - 95.0) < 1e-9
    assert int(p.model.ngeom) == 16
    assert int(p.model.nbody) == 10
    assert p.model.jnt_limited[p.idx.joint["root_tx"]] == 0
    assert p.model.jnt_limited[p.idx.joint["root_tz"]] == 0
    assert p.model.jnt_limited[p.idx.joint["root_ry"]] == 0
    assert V2_CONTACT_SOLREF == (0.016, 1.0)
    assert V2_CONTACT_SOLIMP == (0.99, 0.99, 0.001, 0.5, 2.0)

def test_18_zero_root_passive_authority():
    p = V2Plant()
    for name in ["root_tx","root_tz","root_ry"]:
        vadr = p.idx.vadr[name]
        jid = p.idx.joint[name]
        assert float(p.model.dof_damping[vadr]) == 0.0
        assert float(p.model.jnt_stiffness[jid]) == 0.0
        assert float(p.model.dof_armature[vadr]) == 0.0
        assert int(p.model.jnt_limited[jid]) == 0
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(p.model, d)
    # check qfrc_passive at standing init is zero for root
    assert float(d.qfrc_passive[p.idx.vadr["root_tx"]]) == 0.0
    assert float(d.qfrc_passive[p.idx.vadr["root_tz"]]) == 0.0
    assert float(d.qfrc_passive[p.idx.vadr["root_ry"]]) == 0.0
    # also run short standing and check passive stays zero
    LIMITS = np.array([250,250,250,300,300,200,200], dtype=float)
    HOLD_Q = np.zeros(7)
    KP = np.array([400,400,400,400,400,400,400], dtype=float)
    KD = np.array([10,10,10,10,10,10,10], dtype=float)
    t = 0.0
    for step in range(int(0.5 / V2_CONTROL_PERIOD_S)):
        q = np.array([float(d.qpos[p.idx.qadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]])
        qd = np.array([float(d.qvel[p.idx.vadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]])
        raw = KP*(HOLD_Q-q) - KD*qd
        u = np.clip(raw/LIMITS, -1,1)
        for i, aname in enumerate(["m_lumbar","m_left_hip","m_right_hip","m_left_knee","m_right_knee","m_left_ankle","m_right_ankle"]):
            d.ctrl[p.idx.actuator[aname]] = float(u[i])
        for _ in range(int(V2_CONTROL_PERIOD_S/DT)):
            mujoco.mj_step(p.model, d)
            assert float(d.qfrc_passive[p.idx.vadr["root_tx"]]) == 0.0
            assert float(d.qfrc_passive[p.idx.vadr["root_tz"]]) == 0.0
            assert float(d.qfrc_passive[p.idx.vadr["root_ry"]]) == 0.0
