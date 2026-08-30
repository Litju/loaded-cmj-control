"""RES-16 deterministic tests for true-standing recovery scorer.

Covers 14 checks per Phase 13:
1. old captured-squat satisfies legacy base guard but fails new true-standing
2. qualified standing samples pass true-standing
3. one joint outside envelope -> false
4. COM z outside -> false
5. root z outside -> false
6. trunk pitch outside -> false
7. unilateral contact -> false
8. physical fall -> false
9. candidate starts, leaves before 0.50s -> resets
10. candidate re-enters and holds 0.50s -> latches
11. E12 cannot occur before E11
12. E1-E11 timestamps unchanged
13. OBJECTIVE_COMPLETE cannot be true without new E12
14. quiet deep squat never counts as success
"""
import sys
from pathlib import Path
import json
import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(TASK_ROOT / "src"))

from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2.events import V2EventDetector
from loaded_cmj.v2.constants import V2_TOTAL_MASS_KG, V2_PHYSICS_TIMESTEP_S, V2_CONTROL_PERIOD_S, V2_TRUE_STANDING_ENVELOPE, V2_EVENT_THRESHOLDS

DT = V2_PHYSICS_TIMESTEP_S
WEIGHT = V2_TOTAL_MASS_KG * 9.81

# Helpers
def make_sample(**overrides):
    # default standing sample (should pass true standing) — use exact median values from 05
    base = {
        "time_s": 0.0,
        "com_z": 1.0748482293284303,
        "com_vz": 0.01,
        "left_Fz": 465.0,
        "right_Fz": 465.0,
        "whole_Fz": 930.0,
        "com_margin": 0.05,
        "trunk_tilt": 0.001,
        "fall_contact": False,
        "prohibited": False,
        "joint_position_rad": np.array([-7.815675406786884e-05, 4.432221158197431e-05, 4.432221158197231e-05, -1.3226150450247243e-05, -1.3226150450247385e-05, 0.00014468478071002664, 0.00014468478071002924]),
        "root_z": 0.8999333849297984,
    }
    base.update(overrides)
    return base

def test_01_captured_satisfies_old_but_fails_new():
    # Load captured at OLD_E12 onset from 02
    evidence = json.loads((TASK_ROOT.parent / "loaded-cmj-control-evidence" / "LCMJ-V2_1-RES16-TRUE-STANDING-SCORER-20260830" / "02_FALSE_POSITIVE_E12_REPRODUCTION.json").read_text())
    rec = evidence["at_onset"]
    # Check old guard: tilt<0.2618, |vz|<0.05, whole>0.5W
    old_pass = (abs(rec["trunk_tilt"]) < 0.2618) and (abs(rec["com_vz"]) < 0.05) and (rec["whole_Fz"] > 0.5*WEIGHT)
    assert old_pass, "captured should satisfy legacy base guard"
    # Now check new predicate fails
    det = V2EventDetector()
    sample = {
        "joint_position_rad": rec["joint_q7"],
        "com_z": rec["com_z"],
        "root_z": rec["root_z"],
        "trunk_tilt": rec["trunk_tilt"],
        "left_Fz": rec["left_Fz"],
        "right_Fz": rec["right_Fz"],
        "whole_Fz": rec["whole_Fz"],
        "com_vz": rec["com_vz"],
        "fall_contact": rec["fall"],
        "prohibited": rec["fall"],
    }
    assert not det._is_true_standing_neighborhood(sample), "captured squat must fail true standing"
    assert not det._guard_stable_recovery(sample), "captured squat must fail new E12 guard (old+true)"

def test_02_standing_passes_true():
    # Use 03 window median sample
    det = V2EventDetector()
    # Use standing reference sample (median)
    sample = make_sample()
    assert det._is_true_standing_neighborhood(sample)
    assert det._guard_stable_recovery(sample)

def test_03_one_joint_outside_fails():
    det = V2EventDetector()
    # perturb one joint outside envelope: lumbar 0.001 -> outside? standing lumbar envelope is [-0.001016, 0.000107]
    # Set lumbar to 0.01 (outside)
    q = np.array([0.01, 4.4e-05, 4.4e-05, -1.3e-05, -1.3e-05, 0.00014, 0.00014])
    sample = make_sample(joint_position_rad=q)
    assert not det._is_true_standing_neighborhood(sample)

def test_04_com_z_outside_fails():
    det = V2EventDetector()
    sample = make_sample(com_z=0.83)  # captured COM, outside standing 1.074
    assert not det._is_true_standing_neighborhood(sample)

def test_05_root_z_outside_fails():
    det = V2EventDetector()
    sample = make_sample(root_z=0.63)  # captured root, outside 0.899
    assert not det._is_true_standing_neighborhood(sample)

def test_06_trunk_pitch_outside_fails():
    det = V2EventDetector()
    sample = make_sample(trunk_tilt=0.01)  # outside 0.00321
    assert not det._is_true_standing_neighborhood(sample)

def test_07_unilateral_contact_fails():
    det = V2EventDetector()
    sample = make_sample(left_Fz=5.0, right_Fz=465.0)  # left below 10
    assert not det._is_true_standing_neighborhood(sample)
    sample2 = make_sample(left_Fz=465.0, right_Fz=5.0)
    assert not det._is_true_standing_neighborhood(sample2)

def test_08_physical_fall_fails():
    det = V2EventDetector()
    sample = make_sample(fall_contact=True)
    assert not det._is_true_standing_neighborhood(sample)
    sample2 = make_sample(prohibited=True)
    assert not det._is_true_standing_neighborhood(sample2)

def test_09_candidate_resets_if_leaves_before_dwell():
    det = V2EventDetector()
    # Need to simulate: first latched E11, then E12 candidate starts, leaves before 0.5
    # Instead directly test dwell logic via update sequence
    # Build minimal trace: first ensure E11 is done, then feed E12 candidate that fails midway
    # Simulate time series where guard true for 0.2s then false then true again, check that first candidate discarded
    # We can use the detector's internal candidate handling by feeding samples
    # First, we need to get to balance_capture: easiest is to use a standing trace that has E11 already?
    # Alternative: directly test that a single false sample resets candidate
    # We'll simulate a simple E12 dwell: 0.5s =4000 samples at 0.000125
    # Create 2000 true, 1 false, 4000 true -> should latch at second true segment's 4000th
    det2 = V2EventDetector()
    # Manually set predecessor as if E11 done
    det2.event_records["balance_capture"] = det2.event_records.get("supported_start", None)  # dummy
    # Actually we need to set all 11 events as present to allow E12
    for name in ["supported_start","countermovement_onset","valid_countermovement","upward_reversal","vertical_propulsion","bilateral_takeoff","genuine_flight","apex","descending_landing","impact_absorption","balance_capture"]:
        from loaded_cmj.v2.events import V2EventRecord
        det2.event_records[name] = V2EventRecord(name, 0.0, 0.0, 0, 0)
    det2._candidate_start_time = None
    det2._candidate_start_idx = None
    # Now feed 2000 true
    base_time = 3.0
    for i in range(2000):
        s = make_sample(time_s=base_time + i*DT)
        det2.update(s)
        # candidate should have started at i=0, not yet latched
        assert "stable_recovery" not in det2.event_records, f"should not latch before dwell at i={i}"
    # Next sample false (trunk outside)
    s_false = make_sample(time_s=base_time+2000*DT, trunk_tilt=0.1)
    det2.update(s_false)
    # candidate should have been reset
    assert det2._candidate_start_time is None, "candidate should reset after false"
    # Now feed 4000 true again -> should latch at 4000th
    for i in range(4000):
        s = make_sample(time_s=base_time+2001*DT + i*DT)
        det2.update(s)
    assert "stable_recovery" in det2.event_records, "should latch after re-entering and holding 0.5s"
    rec = det2.event_records["stable_recovery"]
    # Check dwell: confirmed - occurred should be ~0.5 (WIN samples => (WIN-1)*DT =0.499875)
    dwell = rec.confirmed_at - rec.occurred_at
    assert abs(dwell - 0.5) < 1e-6 or abs(dwell - 0.499875) < 1e-9, f"dwell {dwell} !=0.5"

def test_10_candidate_reenters_and_latches():
    # This is similar to previous but ensures latch after full dwell
    det = V2EventDetector()
    for name in ["supported_start","countermovement_onset","valid_countermovement","upward_reversal","vertical_propulsion","bilateral_takeoff","genuine_flight","apex","descending_landing","impact_absorption","balance_capture"]:
        from loaded_cmj.v2.events import V2EventRecord
        det.event_records[name] = V2EventRecord(name, 0.0, 0.0, 0, 0)
    base = 5.0
    for i in range(4000):
        s = make_sample(time_s=base + i*DT)
        det.update(s)
    assert "stable_recovery" in det.event_records

def test_11_E12_cannot_occur_before_E11():
    det = V2EventDetector()
    # No E11, try to feed stable_recovery guard true for 4000 samples
    for i in range(4000):
        s = make_sample(time_s=float(i)*DT)
        det.update(s)
    assert "stable_recovery" not in det.event_records, "E12 should not latch without E11"
    # Also test with only 10 of 11 predecessors
    for name in ["supported_start","countermovement_onset","valid_countermovement","upward_reversal","vertical_propulsion","bilateral_takeoff","genuine_flight","apex","descending_landing","impact_absorption"]:
        from loaded_cmj.v2.events import V2EventRecord
        det.event_records[name] = V2EventRecord(name, 0.0, 0.0, 0, 0)
    det._candidate_start_time=None
    for i in range(4000):
        s = make_sample(time_s=10.0 + i*DT)
        det.update(s)
    assert "stable_recovery" not in det.event_records, "E12 should not latch without balance_capture"

def test_12_E1_E11_unchanged():
    # Run RES8 full with new detector and compare E1-E11 to old
    import json, pathlib
    EVIDENCE_ROOT = pathlib.Path("/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-V2_1-RES16-TRUE-STANDING-SCORER-20260830")
    old_data = json.loads((EVIDENCE_ROOT/"02_FALSE_POSITIVE_E12_REPRODUCTION.json").read_text())
    # old E1-E11 from that file's event_records (which is new? Actually 02 was taken with old detector before change, but we can load 07 trace old)
    # Instead directly run with new detector and check that E1-E11 times match old committed values from 07_FINAL_FORWARD_TRACE
    import sys
    sys.path.insert(0, str(TASK_ROOT/"src"))
    import mujoco
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.controller import act, reset
    plant=V2Plant()
    m=plant.model
    d=plant.make_data()
    plant.reset(d)
    reset(0.0)
    prev=np.zeros(7)
    det=V2EventDetector()
    for ctrl in range(800):
        obs=plant.public_observation(d,float(d.time),ctrl,episode_reset=(ctrl==0),previous_action=prev)
        a=np.asarray(act(obs),dtype=float)
        plant.apply_action(d,a)
        prev=a
        for _ in range(40):
            mujoco.mj_step(m,d)
            summ=plant.foot_contact_summary(d)
            com=plant.center_of_mass(d)
            comv=plant.center_of_mass_velocity(d)
            joint_q=np.array([float(d.qpos[plant.idx.qadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]],dtype=float)
            root_z=float(d.qpos[plant.idx.qadr["root_tz"]])
            trunk_tilt=plant.trunk_tilt(d)
            det.update({"time_s": float(d.time), "com_z": float(com[2]), "com_vz": float(comv[2]), "left_Fz": float(summ["left_Fz"]), "right_Fz": float(summ["right_Fz"]), "whole_Fz": float(summ["whole_Fz"]), "com_margin": float(summ["support_margin"]), "trunk_tilt": float(trunk_tilt), "fall_contact": bool(summ["prohibited_contact"]), "joint_position_rad": joint_q, "root_z": root_z})
    res=det.finalize()
    # Expected old E1-E11 from RES8
    expected={
        'supported_start': 0.00025,
        'countermovement_onset': 0.1628749999999959,
        'valid_countermovement': 0.3811249999999719,
        'upward_reversal': 0.5417499999999728,
        'vertical_propulsion': 0.551624999999976,
        'bilateral_takeoff': 1.5601249999998152,
        'genuine_flight': 1.5699999999998098,
        'apex': 1.7965848282999188,
        'descending_landing': 2.0072499999995803,
        'impact_absorption': 2.3217499999999647,
        'balance_capture': 2.785125000000531,
    }
    for k,v in expected.items():
        assert k in res.events, f"missing {k}"
        assert abs(res.events[k]-v) < 1e-9, f"{k} {res.events[k]} != {v}"

def test_13_objective_complete_requires_new_E12():
    det = V2EventDetector()
    # Feed a full trace that would previously have succeeded with old E12 but now should not
    # Use RES8 trace: it has E1-E11 but no E12, so termination should be INCOMPLETE_HORIZON, not OBJECTIVE_COMPLETE
    import mujoco, numpy as np, sys
    sys.path.insert(0, str(TASK_ROOT/"src"))
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.controller import act, reset
    plant=V2Plant()
    m=plant.model
    d=plant.make_data()
    plant.reset(d)
    reset(0.0)
    prev=np.zeros(7)
    for ctrl in range(800):
        obs=plant.public_observation(d,float(d.time),ctrl,episode_reset=(ctrl==0),previous_action=prev)
        a=np.asarray(act(obs),dtype=float)
        plant.apply_action(d,a)
        prev=a
        for _ in range(40):
            mujoco.mj_step(m,d)
            summ=plant.foot_contact_summary(d)
            com=plant.center_of_mass(d)
            comv=plant.center_of_mass_velocity(d)
            joint_q=np.array([float(d.qpos[plant.idx.qadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]],dtype=float)
            root_z=float(d.qpos[plant.idx.qadr["root_tz"]])
            det.update({"time_s": float(d.time), "com_z": float(com[2]), "com_vz": float(comv[2]), "left_Fz": float(summ["left_Fz"]), "right_Fz": float(summ["right_Fz"]), "whole_Fz": float(summ["whole_Fz"]), "com_margin": float(summ["support_margin"]), "trunk_tilt": float(plant.trunk_tilt(d)), "fall_contact": bool(summ["prohibited_contact"]), "joint_position_rad": joint_q, "root_z": root_z})
    res=det.finalize()
    assert res.termination != "OBJECTIVE_COMPLETE", "should not be complete without true standing"
    assert res.termination == "INCOMPLETE_HORIZON"

def test_14_quiet_deep_squat_never_counts():
    # Build a synthetic quiet deep squat sample that satisfies old guard but not new
    det = V2EventDetector()
    # Need 4000 samples of deep squat with old guard true
    # Use captured squat values
    q_capture = np.array([-0.34, 0.428, 0.428, 1.65, 1.65, 0.546, 0.546])
    for i in range(4000):
        s={
            "time_s": float(i)*DT,
            "com_z": 0.829,
            "com_vz": 0.01,
            "left_Fz": 447.0,
            "right_Fz": 447.0,
            "whole_Fz": 894.0,
            "com_margin": 0.05,
            "trunk_tilt": 0.18,  # <0.2618 true
            "fall_contact": False,
            "joint_position_rad": q_capture,
            "root_z": 0.632,
        }
        # Need predecessor E11
        if i==0:
            for name in ["supported_start","countermovement_onset","valid_countermovement","upward_reversal","vertical_propulsion","bilateral_takeoff","genuine_flight","apex","descending_landing","impact_absorption","balance_capture"]:
                from loaded_cmj.v2.events import V2EventRecord
                det.event_records[name] = V2EventRecord(name, 0.0, 0.0, 0, 0)
        det.update(s)
    assert "stable_recovery" not in det.event_records, "deep squat should never count as success"
    res=det.finalize()
    assert res.termination != "OBJECTIVE_COMPLETE"

